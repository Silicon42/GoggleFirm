#ifndef SERVERDRIVER_H
#define SERVERDRIVER_H
#pragma once

#include <openvr_driver.h>
#include "HMDDeviceDriver.h"
using namespace vr;
//-----------------------------------------------------------------------------
// Purpose: Requested and used in vrserver to query tracking and other information
// about tracked devices. It must be implemented in driver dynamic libraries.
// https://github.com/ValveSoftware/openvr/wiki/IServerTrackedDeviceProvider_Overview
// NOTE: as of time of writing this documentation hasn't been updated since SDK version 0.9.1 (2015)
//-----------------------------------------------------------------------------
class ServerDriver : public IServerTrackedDeviceProvider
{
public:
	virtual EVRInitError Init(IVRDriverContext* pDriverContext);
	virtual void Cleanup();
	virtual const char* const* GetInterfaceVersions() { return k_InterfaceVersions; }
	virtual void RunFrame();
	virtual bool ShouldBlockStandbyMode() { return false; }
	virtual void EnterStandby() {}
	virtual void LeaveStandby() {}

private:
	HMDDeviceDriver* pGoggleHMD = nullptr;
};

#endif //SERVERDRIVER_H