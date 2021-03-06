////////////////////////////////////////////////////////////////////////////////////////////////////
// "FreeSpace IMU" - By Terence J. Bordelon (www.bordelon.net/tj@bordelon.net)
// Extra Math functions
////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes
#include "exmath.h"
#include "math.h"
float 
clamped_acos(float ratio)
{
	if (ratio > 1.0 )
		return 0;
	else if( ratio < -1.0)
		return PI;
	else return acos(ratio);	
} 
//---[Vector Math]-------------------------------------------------------------------------

void v3set( vec3 *v, float x, float y, float z)
{
  v->x = x;
  v->y = y;
  v->z = z;
}

float v3dot( vec3 *v1, vec3 *v2)
// Vector dot product
{
  return v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
}

void v3mul( vec3 *out, vec3 *v1, vec3 *v2)
// Vector multiply
{
  out->x = v1->x * v2->x;
  out->y = v1->y * v2->y;
  out->z = v1->z * v2->z;
}


void v3cross( vec3 *out, vec3 *v1, vec3 *v2)
// Cross product
{
  out->x = v1->y * v2->z - v1->z * v2->y;
  out->y = v1->z * v2->x - v1->x * v2->z;
  out->z = v1->x * v2->y - v1->y * v2->x;
}

float v3length( vec3 *v )
// Vector length
{
  return (float) sqrt(v->x*v->x + v->y*v->y + v->z*v->z);
}

float v3dist( vec3 *v1, vec3 *v2 )
// Vector length
{
	vec3 tv;
	v3sub(&tv, v1, v2);
	return v3length(&tv);
}


void v3add( vec3 *out, vec3 *v1, vec3 *v2)
// Vector add
{
  out->x = v1->x + v2->x;
  out->y = v1->y + v2->y;
  out->z = v1->z + v2->z;
}

void v3sub( vec3 *out, vec3 *v1, vec3 *v2)
// Vector subtract
{
  out->x = v1->x - v2->x;
  out->y = v1->y - v2->y;
  out->z = v1->z - v2->z;
}


void v3scale( vec3 *out, vec3 *in, float s)
// Vector scale
{
  out->x = in->x * s;
  out->y = in->y * s;
  out->z = in->z * s;
}

// Normalize vector and return magnitude 
float v3normalize( vec3 *out, vec3 *in)
{
  float magnitude = v3length(in);

  if( magnitude == 0.0f )
  {
	out->x =out->y=out->z =0.0f;
	return magnitude;
  }
  v3scale(out,in,1.0f/magnitude);
  return magnitude;
}

void v3invert( vec3 *out, vec3 *in)
{
	out->x = -in->x;
	out->y = -in->y;
	out->z = -in->z;
}

//---[ Quaternion Math]-------------------------------------------------------------------------

void  Quat_Normalize(quat *q)
{
	float dist, square;

	square = q->x * q->x + q->y * q->y + q->z * q->z + q->w * q->w;
	
	if (square > 0.0)
		dist = (float)(1.0 / sqrt(square));
	else 
		dist = 1;

	q->x *= dist;
	q->y *= dist;
	q->z *= dist;
	q->w *= dist;

}

void Quat_Invert( quat *q)
{
	float norm, invNorm;

	norm = q->x * q->x + q->y * q->y + q->z * q->z  + q->w * q->w;
	
	invNorm = (float) (1.0 / norm);
	
	q->x = -q->x * invNorm;
	q->y = -q->y * invNorm;
	q->z = -q->z * invNorm;
	q->w =  q->w * invNorm;
}


void Quat_Multiply ( quat *res, quat *q1, quat *q2 )
// Resulting quat will act as if it does Q2 first then Q1 
{
	float A, B, C, D, E, F, G, H;

	A = (q1->w + q1->x)*(q2->w + q2->x);
	B = (q1->z - q1->y)*(q2->y - q2->z);
	C = (q1->w - q1->x)*(q2->y + q2->z); 
	D = (q1->y + q1->z)*(q2->w - q2->x);
	E = (q1->x + q1->z)*(q2->x + q2->y);
	F = (q1->x - q1->z)*(q2->x - q2->y);
	G = (q1->w + q1->y)*(q2->w - q2->z);
	H = (q1->w - q1->y)*(q2->w + q2->z);


	res->w = B + (-E - F + G + H) * 0.50f;
	res->x = A - (E + F + G + H) * 0.50f; 
	res->y = C + (E - F + G - H) * 0.50f; 
	res->z = D + (E - F - G + H) * 0.50f;
}

void Quat_RotateVec3( quat *q, vec3 *vOut, vec3 *vIn)
{
	// v' = q v q^-1

	quat qv,iq,r,qvo;
	qv.x = vIn->x;
	qv.y = vIn->y;
	qv.z = vIn->z;
	qv.w = 0.0f;			// Get v

	iq.x = q->x;
	iq.y = q->y;
	iq.z = q->z;
	iq.w = q->w;
	Quat_Invert( &iq );		// get q^-1

	Quat_Multiply ( &r, q, &qv );
	Quat_Multiply ( &qvo, &r, &iq );	
	
	vOut->x = qvo.x;
	vOut->y = qvo.y;
	vOut->z = qvo.z;	
}

void Quat_Set( quat *q, float w, float x, float y, float z)
{
	q->w = w;
	q->x = x;
	q->y = y;
	q->z = z;
}

void Quat_Add( quat *res, quat *q1, quat *q2 )
{
	res->w = q1->w + q2->w;
	res->x = q1->x + q2->x;
	res->y = q1->y + q2->y;
	res->z = q1->z + q2->z;
}

void Quat_SetAxisAndAngle( quat *q, vec3 *axis, float angle)
{
	float l = sqrt( axis->x*axis->x + axis->y*axis->y + axis->z*axis->z);

	float omega=0.5*angle;
	float s=sin(omega)/l;
	q->x = axis->x * s;
	q->y = axis->y * s;
	q->z = axis->z * s;
	q->w = cos(omega);
}

void 
Quat_GetAxisAngle( quat *q, vec3 *axis, float *angle )
{
	float scale = sqrt(q->x * q->x + q->y * q->y + q->z * q->z);
	axis->x = q->x / scale;
	axis->y = q->y / scale;
	axis->z = q->z / scale;
	*angle = acos(q->w) * 2.0f;
}

// This is for cockpit view of the world
void
Quat_ToEuler(quat *q, float *heading,  float *bank, float *attitude)
{
    float sqw = q->w*q->w;
    float sqx = q->x*q->x;
    float sqy = q->y*q->y;
    float sqz = q->z*q->z;
	
	*heading = atan2(2.0 * (q->x*q->y + q->z*q->w),-(sqx - sqy - sqz + sqw));
	*heading = (*heading*180/PI) + 180;		// in degrees 
	
    *bank = atan2(2.0 * (q->y*q->z + q->x*q->w),(-sqx - sqy + sqz + sqw));
    *bank =  *bank *180/PI;						// in degrees
	//*attitude = asin(2.0 * (q.x*q.z - q.y*q.w));			// requires q to be normalized on entry  
	*attitude = asin(2.0 * (q->x*q->z - q->y*q->w)/(sqx + sqy + sqz + sqw));  // Normalized in place
	*attitude  =  *attitude*180/PI;					// in degrees

}
void
QtoE(quat *q1, float *heading,  float *bank, float *attitude)
{ 
    float sqw = q1->w*q1->w;
    float sqx = q1->x*q1->x;
    float sqy = q1->y*q1->y;
    float sqz = q1->z*q1->z;
	float unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
	float test = q1->x*q1->y + q1->z*q1->w;
/*
	if (test > 0.499*unit) { // singularity at north pole
		heading = 2 * atan2(q1.x,q1.w);
		attitude = PI/2;
		bank = 0;
		return;
	}
	if (test < -0.499*unit) { // singularity at south pole
		heading = -2 * atan2(q1.x,q1.w);
		attitude = -PI/2;
		bank = 0;
		return;
	}
*/ 
	*heading = atan2(2.0*(q1->y * q1->w - q1->x*q1->z) , -(sqx - sqy - sqz + sqw));
	*attitude = asin(2.0*test/unit);
	*bank = atan2(2.0* (q1->x*q1->w - q1->y*q1->z ), ( sqy - sqz -sqx  + sqw));
}
