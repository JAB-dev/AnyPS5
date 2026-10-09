#include "SceTypes.hpp"
#include "prx/libc/include/general/VabiMacros.hpp"
#include <cstdint>
#include <cstdlib>

extern "C" {
std::int32_t APS5_VABI sceSystemGestureOpen(std::int32_t, const void*);
int APS5_VABI sceSystemGestureClose(std::int32_t);
int APS5_VABI sceSystemGestureCreateTouchRecognizer(std::int32_t, SystemGestureTouchRecognizer*, std::int32_t, const SystemGestureRectangle*, const void*);
int APS5_VABI sceSystemGestureAppendTouchRecognizer(std::int32_t, SystemGestureTouchRecognizer*);
int APS5_VABI sceSystemGestureRemoveTouchRecognizer(std::int32_t, SystemGestureTouchRecognizer*);
int APS5_VABI sceSystemGestureUpdateAllTouchRecognizer(std::int32_t);
int APS5_VABI sceSystemGestureGetTouchEventsCount(std::int32_t, const SystemGestureTouchRecognizer*);
}

static void Require(bool value) { if (!value) std::abort(); }

static constexpr int InvalidArgument = static_cast<int>(0x80D10002);
static constexpr int InvalidHandle = static_cast<int>(0x80D10003);

int main() {
    const std::int32_t handle = sceSystemGestureOpen(0, nullptr);
    Require(handle > 0);
    static SystemGestureTouchRecognizer recognizer{};
    const SystemGestureRectangle rectangle{};
    Require(sceSystemGestureCreateTouchRecognizer(handle, &recognizer, 1, &rectangle, nullptr) == 0);

    Require(sceSystemGestureAppendTouchRecognizer(handle, &recognizer) == 0);
    Require(sceSystemGestureAppendTouchRecognizer(handle, nullptr) == InvalidArgument);
    Require(sceSystemGestureAppendTouchRecognizer(handle + 1, &recognizer) == InvalidHandle);
    Require(sceSystemGestureAppendTouchRecognizer(handle + 1, nullptr) == InvalidHandle);

    Require(sceSystemGestureUpdateAllTouchRecognizer(handle) == 0);
    Require(sceSystemGestureUpdateAllTouchRecognizer(handle + 1) == InvalidHandle);
    Require(sceSystemGestureGetTouchEventsCount(handle, &recognizer) == 0);

    Require(sceSystemGestureRemoveTouchRecognizer(handle, &recognizer) == 0);
    Require(sceSystemGestureRemoveTouchRecognizer(handle, nullptr) == InvalidArgument);
    Require(sceSystemGestureRemoveTouchRecognizer(handle + 1, &recognizer) == InvalidHandle);
    Require(sceSystemGestureRemoveTouchRecognizer(handle + 1, nullptr) == InvalidHandle);

    Require(sceSystemGestureClose(handle) == 0);
}
