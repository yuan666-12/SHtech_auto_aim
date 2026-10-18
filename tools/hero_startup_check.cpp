// Recording SDK replacement: tests production HikCamWrapper without any camera.
#include "hikcam_wrapper.hpp"
#include "grab_strategy.hpp"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
namespace {
std::vector<std::string> calls;
bool opened=false, grabbing=false, policy=false;
bool reject_policy=false, reject_open=false, reject_start=false;
int bad_order=0;
MV_CC_DEVICE_INFO device{};
unsigned char pixels[4]{};
void record(const char* name) { calls.emplace_back(name); }
int bad() { ++bad_order; return int(0x80000003u); }
}
extern "C" {
int MV_CC_EnumDevices(unsigned int, MV_CC_DEVICE_INFO_LIST* list) {
#ifdef STARTUP_SDK_PRELOAD
    std::fprintf(stderr, "TEST_CAMERA_RETRY_NO_HARDWARE\n"); std::fflush(stderr);
    return -1;
#else
    record("enum"); list->nDeviceNum=1; list->pDeviceInfo[0]=&device; return 0;
#endif
}
int MV_CC_CreateHandle(void** handle, const MV_CC_DEVICE_INFO*) {
    record("create"); *handle=&device; return 0;
}
int MV_CC_OpenDevice(void*, unsigned int, unsigned short) {
    record("open"); if (reject_open) return -1; opened=true; return 0;
}
int MV_CC_GetPixelFormat(void*, MVCC_ENUMVALUE* value) {
    value->nCurValue=PixelType_Gvsp_BayerRG8; return opened?0:bad();
}
int MV_CC_SetEnumValue(void*, const char*, unsigned int) { return opened?0:bad(); }
int MV_CC_GetIntValue(void*, const char*, MVCC_INTVALUE* value) {
    value->nCurValue=4; return opened?0:bad();
}
int MV_CC_SetGrabStrategy(void*, int strategy) {
    record("strategy");
    if (!opened || grabbing || strategy!=1) return bad();
    if (reject_policy) return -1;
    policy=true; return 0;
}
int MV_CC_StartGrabbing(void*) {
    record("start"); if (!opened || !policy || grabbing) return bad();
    if (reject_start) return -1;
    grabbing=true; return 0;
}
int MV_CC_GetImageBuffer(void*, MV_FRAME_OUT* frame, unsigned int) {
    record("read"); if (!grabbing) return bad();
    frame->pBufAddr=pixels;
    frame->stFrameInfo.nWidth=2; frame->stFrameInfo.nHeight=2;
    frame->stFrameInfo.enPixelType=PixelType_Gvsp_BayerRG8; return 0;
}
int MV_CC_FreeImageBuffer(void*, MV_FRAME_OUT* frame) {
    record("free"); frame->pBufAddr=nullptr; return 0;
}
int MV_CC_StopGrabbing(void*) {
    record("stop"); if (!grabbing) return bad(); grabbing=false; return 0;
}
int MV_CC_CloseDevice(void*) {
    record("close"); if (!opened || grabbing) return bad(); opened=false; return 0;
}
int MV_CC_DestroyHandle(void*) {
    record("destroy"); if (opened || grabbing) return bad(); policy=false; return 0;
}
int MV_CC_GetImageInfo(void*, MV_IMAGE_BASIC_INFO*) { return 0; }
int MV_CC_SetBrightness(void*, unsigned int) { return 0; }
}
#ifndef STARTUP_SDK_PRELOAD
int checks=0;
void require(bool ok, const char* what) {
    ++checks; if (!ok) throw std::runtime_error(what);
}
void reset() {
    calls.clear(); bad_order=0; reject_policy=reject_open=reject_start=false;
    require(!opened && !grabbing, "previous case released SDK resources");
}
int count(const char* name) { return std::count(calls.begin(),calls.end(),name); }
int main() {
    try {
        {
            HikCamWrapper camera;
            require(camera.init(), "production wrapper initializes with strict SDK ordering");
            require(std::find(calls.begin(),calls.end(),"strategy") <
                    std::find(calls.begin(),calls.end(),"start"), "policy precedes acquisition");
            require(count("read")==10, "warm-up occurs after successful configuration");
            require(camera.close(), "normal close succeeds");
            auto before=calls.size(); camera.close();
            require(calls.size()==before, "repeated close makes no SDK calls");
            require(bad_order==0, "no invalid SDK ordering in normal lifecycle");
            require(camera.init(), "camera can initialize again after close");
            camera.close();
        }
        reset(); reject_policy=true;
        {
            HikCamWrapper camera;
            require(!camera.init(), "rejected policy fails initialization");
            require(count("start")==0 && count("stop")==0, "policy failure never starts or stops stream");
            require(count("close")==1 && count("destroy")==1, "policy failure releases open handle");
            require(bad_order==0, "failure cleanup respects SDK lifecycle");
        }
        reset(); reject_open=true;
        {
            HikCamWrapper camera;
            require(!camera.init(), "open failure is propagated");
            require(count("close")==0 && count("stop")==0 && count("destroy")==1,
                    "unopened handle is destroyed without invalid close or stop");
        }
        reset(); reject_start=true;
        {
            HikCamWrapper camera;
            require(!camera.init(), "start failure is propagated");
            require(count("stop")==0 && count("close")==1 && count("destroy")==1,
                    "failed start closes device without stopping a nonexistent stream");
            require(bad_order==0, "failed start cleanup has no order violation");
        }
        std::cout<<"PASS "<<checks<<" camera SDK lifecycle checks; real hardware not linked\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr<<"FAIL "<<e.what()<<'\n'; return 1;
    }
}
#endif
