#include <openvr_driver.h>
#include "ServerDriver.h"
using namespace vr;

#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec( dllexport )
#define HMD_DLL_IMPORT extern "C" __declspec( dllimport )
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#define HMD_DLL_IMPORT extern "C" 
#else
#error "Unsupported Platform."
#endif

//-----------------------------------------------------------------------------
// Purpose: Standard driver factory function, used to return implementations of
// the OpenVR driver interfaces. This function is the entry-point for every driver
// https://github.com/ValveSoftware/openvr/wiki/Driver-Factory-Function
//-----------------------------------------------------------------------------
ServerDriver g_serverDriverNull;

HMD_DLL_EXPORT
void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode)
{
	if (0 == strcmp(IServerTrackedDeviceProvider_Version, pInterfaceName))
	{
		return &g_serverDriverNull;
	}

	if (pReturnCode)
		*pReturnCode = VRInitError_Init_InterfaceNotFound;

	return NULL;
}
