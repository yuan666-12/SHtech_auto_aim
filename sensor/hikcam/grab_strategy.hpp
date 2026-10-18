#pragma once

#include "MvCameraControl.h"

// The repository carries an older MVS header. The installed MVS SDK declares
// MV_CC_SetGrabStrategy(void*, MV_GRAB_STRATEGY) and exports this C entry point.
// Its enum has int ABI; LatestImagesOnly is 1. Keep the existing frame ABI.
extern "C" MV_CAMCTRL_API int __stdcall MV_CC_SetGrabStrategy(void* handle, int strategy);

namespace hikcam {
inline int select_latest_frame(void* handle) {
    constexpr int latest_images_only = 1;
    return MV_CC_SetGrabStrategy(handle, latest_images_only);
}
} // namespace hikcam
