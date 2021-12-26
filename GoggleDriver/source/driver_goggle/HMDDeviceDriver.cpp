#include "HMDDeviceDriver.h"
#include "driverlog.h"
using namespace vr;

//-----------------------------------------------------------------------------
// Purpose: HMD driver
//-----------------------------------------------------------------------------

HMDDeviceDriver::HMDDeviceDriver(hid_device* handle) : IBaseHidTrackerDriver(handle)
{
	DriverLog("Using settings values\n");

	windowX = VRSettings()->GetInt32(k_pch_Goggle_Section, "windowX");
	windowY = VRSettings()->GetInt32(k_pch_Goggle_Section, "windowY");
	windowWidth = VRSettings()->GetInt32(k_pch_Goggle_Section, "windowWidth");
	windowHeight = VRSettings()->GetInt32(k_pch_Goggle_Section, "windowHeight");
	renderWidth = VRSettings()->GetInt32(k_pch_Goggle_Section, "renderWidth");
	renderHeight = VRSettings()->GetInt32(k_pch_Goggle_Section, "renderHeight");

	DriverLog("hmd: Window: %d %d %d %d\n", windowX, windowY, windowWidth, windowHeight);
	DriverLog("hmd: Render Target: %d %d\n", renderWidth, renderHeight);
}

EVRInitError HMDDeviceDriver::Activate(TrackedDeviceIndex_t unObjectId)
{
	BaseActivate(unObjectId);
	float tempFl;

	tempFl = VRSettings()->GetFloat(k_pch_SteamVR_Section, k_pch_SteamVR_IPD_Float);
	VRProperties()->SetFloatProperty(hPropertyContainer, Prop_UserIpdMeters_Float, tempFl);
	DriverLog("hmd: IPD: %f\n", tempFl);

	tempFl = VRSettings()->GetFloat(k_pch_Goggle_Section, "headToEyeDepthMeters");
	VRProperties()->SetFloatProperty(hPropertyContainer, Prop_UserHeadToEyeDepthMeters_Float, tempFl);
	DriverLog("hmd: Head to Eye Depth in Meters: %f\n", tempFl);

	tempFl = VRSettings()->GetFloat(k_pch_Goggle_Section, "displayFrequency");
	VRProperties()->SetFloatProperty(hPropertyContainer, Prop_DisplayFrequency_Float, tempFl);
	DriverLog("hmd: Display Frequency: %f\n", tempFl);

	tempFl = VRSettings()->GetFloat(k_pch_Goggle_Section, "secondsFromVsyncToPhotons");
	VRProperties()->SetFloatProperty(hPropertyContainer, Prop_SecondsFromVsyncToPhotons_Float, tempFl);
	DriverLog("hmd: Seconds from Vsync to Photons: %f\n", tempFl);

	return VRInitError_None;
}

void* HMDDeviceDriver::GetComponent(const char* pchComponentNameAndVersion)
{
	if (!_stricmp(pchComponentNameAndVersion, IVRDisplayComponent_Version))
	{
		return (IVRDisplayComponent*)this;
	}

	// override this to add a component to a driver
	return NULL;
}

/** debug request from a client */
void HMDDeviceDriver::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
{
	if (unResponseBufferSize >= 1)
		pchResponseBuffer[0] = 0;
}

void HMDDeviceDriver::GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight)
{
	*pnX = windowX;
	*pnY = windowY;
	*pnWidth = windowWidth;
	*pnHeight = windowHeight;
}

void HMDDeviceDriver::GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight)
{
	*pnWidth = renderWidth;
	*pnHeight = renderHeight;
}

void HMDDeviceDriver::GetEyeOutputViewport(EVREye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight)
{
	*pnY = 0;
	*pnWidth = windowWidth / 2;
	*pnHeight = windowHeight;

	if (eEye == Eye_Left)
	{
		*pnX = 0;
	}
	else
	{
		*pnX = windowWidth / 2;
	}
}

void HMDDeviceDriver::GetProjectionRaw(EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom)
{
	*pfLeft = -1.0;
	*pfRight = 1.0;
	*pfTop = -1.0;
	*pfBottom = 1.0;
}

DistortionCoordinates_t HMDDeviceDriver::ComputeDistortion(EVREye eEye, float fU, float fV)
{
	DistortionCoordinates_t coordinates;
	coordinates.rfBlue[0] = fU;
	coordinates.rfBlue[1] = fV;
	coordinates.rfGreen[0] = fU;
	coordinates.rfGreen[1] = fV;
	coordinates.rfRed[0] = fU;
	coordinates.rfRed[1] = fV;
	return coordinates;
}


void HMDDeviceDriver::RunFrame()
{
	// In a real driver, this should happen from some pose tracking thread.
	// The RunFrame interval is unspecified and can be very irregular if some other
	// driver blocks it for some periodic task.
	if (objectId != k_unTrackedDeviceIndexInvalid)
	{
		VRServerDriverHost()->TrackedDevicePoseUpdated(objectId, GetPose(), sizeof(DriverPose_t));
	}
}
