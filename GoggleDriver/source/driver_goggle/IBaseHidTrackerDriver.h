#ifndef IBASEHIDTRACKERDRIVER_H
#define IBASEHIDTRACKERDRIVER_H
#pragma once

#include <openvr_driver.h>
#include "hidapi.h"
using namespace vr;

inline HmdQuaternion_t TrackerQuaternion_Init(double w, double x, double y, double z)
{
	HmdQuaternion_t quat;
	quat.w = w;
	quat.x = x;
	quat.y = y;
	quat.z = z;
	return quat;
}

// keys for use with the settings API
static const char* const k_pch_Goggle_Section = "driver_goggle";	//doesn't really belong here but it's needed everywhere this is included


/*
A basic tracker that provides orientation as a quaternion and a 3D velocity vector, meant to be extended for HMD, controllers, etc.
Intended to be used with HID devices using an IMU, not intended for positional tracking devices
*/
class IBaseHidTrackerDriver : public ITrackedDeviceServerDriver
{
public:
	IBaseHidTrackerDriver(hid_device* handle) : hHID(handle) {}
	virtual ~IBaseHidTrackerDriver() {};
	virtual void Deactivate() { objectId = k_unTrackedDeviceIndexInvalid; }
	virtual void EnterStandby() {}
	//void* GetComponent(const char* pchComponentNameAndVersion) = 0;
	/** debug request from a client */
	//virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) = 0;
	virtual DriverPose_t GetPose();
protected:
	//BaseActivate() should be run at the start of an inheriting class' Activate() function as it sets hPropertyContainer
	//It also handles setting the generic properties that all tracked devices have
	void BaseActivate(TrackedDeviceIndex_t unObjectId);

	hid_device* hHID = nullptr;
	TrackedDeviceIndex_t objectId = k_unTrackedDeviceIndexInvalid;
	//kept so that different levels of inheritance don't need to re-call "vr::VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId)"
	PropertyContainerHandle_t hPropertyContainer = k_ulInvalidPropertyContainer;
	HmdQuaternion_t lastQuat = TrackerQuaternion_Init(0, 1, 0, 0);
};

#endif //IBASEHIDTRACKERDRIVER_H