////////////////////////////////////////////////////////////////////////////////////////////////////
// "FreeSpace IMU" - By Terence J. Bordelon (www.bordelon.net/tj@bordelon.net)
//  
// This code was removed from my working IMU and stripped down for your use and to help you 
// understand the algorithm presented in the article. 
//
// A few notes on the coordinate system used:
//
//   Theta = Pitch = (Circle with horizontal line)
//   Phi = Roll = (Circle with vertical line)
//   Psi = Yaw = Y with vertical line down the center
//
// Rotations (point your thumb down +axis, fingers curl to + rotation):
//   About X = Roll = P = positive rotations to the right
//   About Y = Pitch = Q = positive raises the nose
//   About Z = Yaw = R = positive turns to the right
//
// Aircraft coordinate system:
//   X axis is forward, Y is to the right, Z is down.
//
//	Visualizing Quaternions:
//
//	Pretty much axis/angle, specifies displacement from identity (system described above).
//	the quaternion will rotate world space to aircraft local space. If you draw the vector,
//  it will tell you what axis to rotate to get to the aircraft space.
//	
////////////////////////////////////////////////////////////////////////////////////////////////////

//---[Includes]------------------------------------------------------
//#include "Arduino.h"
#include "imu.h"
//#include "vector.h" // from compass module
#include <math.h>

// This is the Declination  aka magnetic variance  of magnetic north to true north of your location 
// this is used here so that the device reads true heading in the end 
// For now this is set to zero, so the compass will indicate heading referenced to magnetic north as a pure compass should 
#define DECLINATION_DEGREES 0
#define DECLINATION_MINUTES 0
#define ERECT_RPS RADIANS_PER_DEGREE 		 // for now 1 degree per second 
#define NORTH_RPS RADIANS_PER_DEGREE 
//---[Defines]--------------------------------------------------------
extern vector acc;			// From Qheading.cpp
extern vector mag;
//---[Static]--------------------------------------------------------

static quat 	g_gyroFrame;
static vec3		g_world_down;
static vec3     g_world_north;
// Globals
float gErectionRateMult = 1.0;				// This is initially set to a high number in init, after initialization period it needs to be set to ~1	
//---[Functions]------------------------------------------------------
// TODO:  make C++ class out of this

static void
IMU_SetNorthVector( float declination_degrees, float declination_minutes, boolean East)
// Set magnetometer "north" vector, set from NOAA's geomag data.
// We flatten this vector to the X-Y plane (no Z!)
// http://www.ngdc.noaa.gov/geomagmodels/struts/igrfWmmZip
{
	quat q;
	vec3 rot;
	float declination = (declination_degrees  + declination_minutes / 60.0f) * (float) RADIANS_PER_DEGREE;
	
	if ( East )
		declination *= -1;		// East declination 

	// Start vector pointing "real" north, along the X axis.
	v3set( &g_world_north, 1.0f, 0.0f, 0.0f);

	// Rotate according to the declination	
	v3set(&rot, 0.0f, 0.0f, 1.0f);	
	Quat_SetAxisAndAngle( &q, &rot, declination);
	Quat_RotateVec3(&q, &g_world_north, &g_world_north);
}

void 
IMU_Init( void )
// Initialize
{
	// Start off with no orientation. 
	//
	//This is most certainly a bad estimate on powerup, and it will take time to settle.
	// You can create a better estimate by temporarily running with high value in gErectionRateMult
	// and bring them back to normal after a few seconds pass. 

digitalWrite(13,HIGH);   // set the LED on 

	Quat_Set( &g_gyroFrame, 1.0f, 0.0f, 0.0f, 0.0f );	// North and Level
	v3set(&g_world_down, 0.0f,0.0f, 1.0f);
	
// TODO: These are set for your location. See the comments in this function.
	IMU_SetNorthVector( DECLINATION_DEGREES, DECLINATION_MINUTES, true );
	gErectionRateMult = 40.0;	// initial erection rate multiplier 
}
	

static void 
IMU_IntegrateGyros(float dt, vec3 *rates, quat *pGyroFrame)
// Integrate gyros into our gyro frame. 
{
	quat q_dot;
	vec3 axis;
	float rate;	
	
	// Get rotation axis from gyros, and rate in rads/sec	
	rate = v3normalize(&axis,rates);	
	// 
	if (rate == 0.0f)	// there is nothing to integrate 
		return;

	// Get a quat for the displacement this timestep did.
	Quat_SetAxisAndAngle( &q_dot, &axis, rate * dt);

	// Multiply it in (This is euler integration)
	Quat_Multiply(pGyroFrame, pGyroFrame, &q_dot);
	Quat_Normalize( pGyroFrame);
}
// NOTE on axis related to NASA standard Aircraft, This is different from gaming 3D standards as the X and Y axes are swapped
//This is importent for the converstion to Euler of the Quaternion.
// X axis is along fuselage 
// Y axis is along wing
// Z axis is along vertical stabilizer

static void
IMU_GetAircraftSpaceState( vec3 *imu_north, vec3 *imu_down, vec3 *rates )
// Fill in this function and return sensor values for the IMU algorithm to use:
//		imu_north - Normalized magnetometer vector
//		imu_down  - Normalized accelerometer, with optional centrepetal force correction applied.
//		rates	  - rads/sec gyro rates
//
{
	// TODO: You need to read your ADC, apply calibration tables, and return these values.	
	rates->x = rates->y= rates->z = 0.0; // this would be the Gyro rates -- set to 0 for no gyro
	// NOTE: normalizing of the two vectors below is done in the heading calculation of the compass module 	
	imu_down->x = acc.x; imu_down->y = acc.y; imu_down->z = acc.z;    
	imu_north->x = mag.x; imu_north->y = mag.y; imu_north->z = mag.z;

}

static boolean 
IMU_GetErectorQuat( quat *q_erect_out, vec3 *v_measured, vec3 *v_reference, float rads_sec, float dt)
// Calculates the quaternion needed to rotate the measured vector to the reference vector (world space) at a fixed correction rate
// Returns TRUE if correction is necessary, or FALSE if it is below the threshold of caring.
{
	// Get the rotation axis we'll use to erect.
	// (Normalize returns the length. We do a lower limit. No sense in correcting if we're close)

	vec3 c;
	v3cross(&c,v_measured, v_reference);	
	if( v3normalize(&c,&c) > 0.5f * RADIANS_PER_DEGREE)		// calculate  only if error is bigger than 	
	{
		// Get the angle between the two vectors, and clamp to that limit. We don't want to overshoot.
		// Angles are always positive since the rotation angle flips appropriately.
		float rads =  rads_sec * dt * gErectionRateMult;
		float a = fabs( clamped_acos( v3dot( v_measured, v_reference) ) );
		
		//Limit  to not overshoot		
		if ( rads > a)
			rads = a;
		// Get the quat that rotates our sensor toward the world vector by the specified amount
		Quat_SetAxisAndAngle( q_erect_out, &c, rads); 		

		return true;
	}

	return false;
}



void IMU_Update( quat *Out,float dt )
// Main IMU sensor fusion function. Call frequently.
// R_estimate - Estimated vehicle orientation (in/out)
// dt - timestep, in seconds.
{
	quat erector;
	vec3 imu_north;		// vector pointing north as seen by the device body
	vec3 imu_down;		// vector pointing to center of earth as seen by device body
	vec3 rates;			// Velocity vector of turn rates of body as seen by device body
	vec3 v_measured_down_world;
	vec3 v_measured_north_world;
	
	// Read our sensors and get engineering units...
	IMU_GetAircraftSpaceState( &imu_north, &imu_down, &rates);
	
	// Euler Integrate our angular rate gyros into R
	IMU_IntegrateGyros( dt, &rates, &g_gyroFrame);

	// Get the down vector in world space.	
	//                                    quat_in                     v3_out                                             v3_in
	Quat_RotateVec3( &g_gyroFrame, &v_measured_down_world, &imu_down);

	// Get erector quaternion from measured down and "reference" down.
	if( IMU_GetErectorQuat( &erector, &v_measured_down_world, &g_world_down , ERECT_RPS, dt) )
	{
		// If a correction is needed, apply the erector rotation to our estimate.
		//                               out,                           in1,                 in2
		Quat_Multiply( &g_gyroFrame, &erector, &g_gyroFrame );
digitalWrite(13,HIGH);   // Blip led to indicate ERROR update  	
	}	

	// Get the north vector in world space, flattened on the X-Y plane
	//                                    quat_in                     v3_out                                             v3_in
	Quat_RotateVec3( &g_gyroFrame, &v_measured_north_world, &imu_north);
	v_measured_north_world.z = 0.0f;
	v3normalize(&v_measured_north_world,&v_measured_north_world);

	// Get erector quaternion from measured north and "reference" north.
	if( IMU_GetErectorQuat( &erector, &v_measured_north_world, &g_world_north , NORTH_RPS, dt) )
	{
		// If a correction is needed, apply the erector rotation to our estimate.
		//                               out,                           in1,                 in2
		Quat_Multiply( &g_gyroFrame, &erector, &g_gyroFrame );
digitalWrite(13,HIGH);   // Blip led to indicate ERROR update  	
	}	
	
	*Out = g_gyroFrame;

}
