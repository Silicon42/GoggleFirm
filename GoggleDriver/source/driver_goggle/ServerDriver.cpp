#include "ServerDriver.h"
#include <openvr_driver.h>
#include "driverlog.h"
#include "hidapi.h"	//https://github.com/libusb/hidapi release 0.11.0
using namespace vr;
//-----------------------------------------------------------------------------
// Purpose: Requested and used in vrserver to query tracking and other information
// about tracked devices. It must be implemented in driver dynamic libraries.
// https://github.com/ValveSoftware/openvr/wiki/IServerTrackedDeviceProvider_Overview
// NOTE: as of time of writing this documentation hasn't been updated since SDK version 0.9.1 (2015)
//-----------------------------------------------------------------------------

EVRInitError ServerDriver::Init(IVRDriverContext* pDriverContext)
{
	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
	InitDriverLog(VRDriverLog());	//goes to vrserver.txt
	DriverLog("Driver Init");

	int result = hid_init();	//returns 0 if successful
	if (result)
	{
		DriverLog("Failed to init HID API, hid_init() returned: %d", result);
		return VRInitError_Init_InterfaceNotFound;
	}
	unsigned short vid = 1155;	//TODO: convert hardcoded vid, pid, and sn to be modifiable in settings
	unsigned short pid = 22352;
	hid_device* hHID = hid_open(vid, pid, NULL);
	if (!hHID)
	{
		DriverLog("Couldn't open HID with vid=%d and pid=%d", vid, pid);
		return VRInitError_Init_HmdNotFound;
	}

	pGoggleHMD = new HMDDeviceDriver(hHID);
	VRServerDriverHost()->TrackedDeviceAdded("0042", TrackedDeviceClass_HMD, pGoggleHMD);	//TODO: replace temporarily hardcoded serial number

	//DriverPose_t pose = pGoggleHMD->GetPose();
	//DriverLog("Pose check: %f %f %f %f", pose.qWorldFromDriverRotation.w, pose.qWorldFromDriverRotation.x, pose.qWorldFromDriverRotation.y, pose.qWorldFromDriverRotation.z);

	return VRInitError_None;
}

void ServerDriver::Cleanup()
{
	CleanupDriverLog();
	delete pGoggleHMD;
	pGoggleHMD = NULL;
}


void ServerDriver::RunFrame()	//TODO: split tracking into it's own thread
{
	if (pGoggleHMD)
	{
		pGoggleHMD->RunFrame();
	}
}
