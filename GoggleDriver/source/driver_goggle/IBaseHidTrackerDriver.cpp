#include "IBaseHidTrackerDriver.h"
#include "driverlog.h"
using namespace vr;

void IBaseHidTrackerDriver::BaseActivate(TrackedDeviceIndex_t unObjectId)
{
	objectId = unObjectId;
	PropertyContainerHandle_t hPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(objectId);

	//OpenVR sets the Serial Number property when calling VRServerDriverHost()->TrackedDeviceAdded()
	//Model Number doesn't need to be tracked by the driver so get it here instead of in the constructor
	char buf[1024];
	VRSettings()->GetString(k_pch_Goggle_Section, "modelNumber", buf, sizeof(buf));
	DriverLog("BaseTracker: model number: %s", buf);

	vr::VRProperties()->SetStringProperty(hPropertyContainer, Prop_ModelNumber_String, buf);
	vr::VRProperties()->SetStringProperty(hPropertyContainer, Prop_RenderModelName_String, buf);

	// return a constant that's not 0 (invalid) or 1 (reserved for Oculus)
	vr::VRProperties()->SetUint64Property(hPropertyContainer, Prop_CurrentUniverseId_Uint64, 42);	//DON'T PANIC

	// avoid "not fullscreen" warnings from vrmonitor
	vr::VRProperties()->SetBoolProperty(hPropertyContainer, Prop_IsOnDesktop_Bool, false);

	return;
}

DriverPose_t IBaseHidTrackerDriver::GetPose()
{
	union
	{
		float floats[7];
		unsigned char bytes[28];
	}buffer;
	hid_read(hHID, buffer.bytes, 28);
	lastQuat = { buffer.floats[0], buffer.floats[1], buffer.floats[2], buffer.floats[3] };
	DriverLog("floats: %f %f %f %f", buffer.floats[0], buffer.floats[1], buffer.floats[2], buffer.floats[3]);
	DriverLog("floats: %f %f %f %f", lastQuat.w, lastQuat.x, lastQuat.y, lastQuat.z);

	DriverPose_t pose = { 0 };
	pose.poseIsValid = true;
	pose.result = TrackingResult_Running_OK;
	pose.deviceIsConnected = true;

	pose.qWorldFromDriverRotation = { 1,0,0,0 };
	pose.qDriverFromHeadRotation = { 1,0,0,0 };
	pose.qRotation = lastQuat;	//this line is what actually rotates the pose
	return pose;
}