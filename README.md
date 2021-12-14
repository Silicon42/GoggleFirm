# GoggleFirm
Custom open source firmware for the VRtek WVR1 (aka SpearII) to unlock it's full potential.

This project was originally intended as a custom firmware for the WVR1 because I wanted to tinker with HMDs and I got a couple for cheap. Shortly after, the company making them dropped support for them and stopped acknowledging their existence, most likely because the reason they were cheap was that they were just liquidating some leftover Chinese stock. In any case, the goal of this firmware is to be as transparent and configurable as reasonably possible and maintaining SteamVR support. I would like to eventually expand this as a generalized custom firmware for other stm32 based headsets like the other VRtek headsets, Occulus's Dev Kits, OSVR's HDKs, and even custom headsets, but that's currently not feasible for me as I don't have the hardware nor the free time. Maybe after I get this one working I'll see about getting some test hardware.

## Current Capabilities:
- Screen displays properly
- Working USB descriptor

## Immediate Goals:
- Working SteamVR driver, either through emulating another headset or writing my own
- sensor fusion and tracking, currently only outputting dummy data
- tweak edid and panel settings, currently only using a mildly optimized duplicate of the factory provided setup

## Stretch/Eventual Goals:
- See if mic support can be added

## Thanks:
- the Relativity Discord, for being helpful and nice to talk to about custom HMD stuff