#ifndef HMDDEVICEDRIVER_H
#define HMDDEVICEDRIVER_H
#pragma once

#include <openvr_driver.h>
#include "hidapi.h"
#include "IBaseHidTrackerDriver.h"
using namespace vr;

//-----------------------------------------------------------------------------
// Purpose: HMD driver
//-----------------------------------------------------------------------------
class HMDDeviceDriver : public IBaseHidTrackerDriver, public IVRDisplayComponent
{
public:
	HMDDeviceDriver(hid_device* handle);
	virtual ~HMDDeviceDriver() {}
	virtual EVRInitError Activate(TrackedDeviceIndex_t unObjectId);
//	virtual void Deactivate() {	m_unObjectId = k_unTrackedDeviceIndexInvalid; }
//	virtual void EnterStandby() {}
	void* GetComponent(const char* pchComponentNameAndVersion);
	/** debug request from a client */
	virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize);
//	virtual DriverPose_t GetPose();

	virtual void GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight);
	virtual bool IsDisplayOnDesktop() { return true; }
	virtual bool IsDisplayRealDisplay() { return false; }
	virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight);
	virtual void GetEyeOutputViewport(EVREye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight);
	virtual void GetProjectionRaw(EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom);
	virtual DistortionCoordinates_t ComputeDistortion(EVREye eEye, float fU, float fV);
	
	void RunFrame();
	//std::string GetSerialNumber() const { return serialNum; }
	
private:
//	hid_device* hHID = nullptr;
//	TrackedDeviceIndex_t m_unObjectId = k_unTrackedDeviceIndexInvalid;
	//PropertyContainerHandle_t m_ulPropertyContainer;	//so long as this is only used in Activate() it doesn't need to be kept as a member
	//even then it's probably fine to just call "VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);" again

//	std::string modelNum;	//not strictly required to be stored but you probably get it at the same time as the serial number and need it when Activate() gets called
//	std::string serialNum;	//needs to be available to the server driver before Activate() is called

	int32_t windowX;
	int32_t windowY;
	int32_t windowWidth;
	int32_t windowHeight;
	int32_t renderWidth;
	int32_t renderHeight;
//	float secondsFromVsyncToPhotons;
//	float displayFrequency;
//	float IPD;
};
#endif //HMDDEVICEDRIVER_H