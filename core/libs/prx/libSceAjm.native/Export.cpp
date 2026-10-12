#include <cstdint>
#include <cstddef>
#include "SceTypes.hpp"
#include "prx/libc/include/General.hpp"
#include <atomic>
#include <cstdio>

// Entry points this library does not implement: reported before the unimplemented-call exception so an
// APS5_TRACE_AJM run shows which one a title reached.
static void AjmStub(const char* name) {
    std::fprintf(stderr, "[ajm] unimplemented %s called\n", name);
    NotImplemented_nid_no_patch(name);
}

extern "C" {

int APS5_VABI sceAjmBatchJobEncode(AjmBatchInfo* info, uint32_t instance, const void* pcm_input, size_t pcm_input_size, void* bitstream_output, size_t bitstream_output_size, void* result) {
 (void)info;
 (void)instance;
 (void)pcm_input;
 (void)pcm_input_size;
 (void)bitstream_output;
 (void)bitstream_output_size;
 (void)result;
 AjmStub(__func__);
 return 0;
}

const char* APS5_VABI sceAjmStrError(int error) {
 switch (static_cast<uint32_t>(error)) {
 case 0: return "ok";
 case 0x80930002u: return "context does not exist";
 case 0x80930003u: return "instance does not exist";
 case 0x80930004u: return "batch does not exist";
 case 0x80930005u: return "invalid parameter";
 case 0x80930006u: return "not enough memory";
 case 0x80930007u: return "not enough resources";
 case 0x80930008u: return "codec is not supported";
 case 0x80930009u: return "codec is already registered in the context";
 case 0x8093000Au: return "codec is not registered in the context";
 case 0x8093000Bu: return "wrong revision in the flags";
 case 0x8093000Cu: return "flag is not supported";
 case 0x8093000Du: return "busy";
 case 0x8093000Eu: return "priority is invalid";
 case 0x8093000Fu: return "in progress";
 case 0x80930010u: return "operation cannot be performed at the moment";
 case 0x80930011u: return "malformed batch";
 case 0x80930012u: return "error at job creation";
 case 0x80930013u: return "invalid opcode";
 case 0x80930014u: return "priority violation";
 case 0x80930015u: return "buffer is too big";
 case 0x80930016u: return "invalid address";
 case 0x80930017u: return "batch was cancelled";
 default: return "(unknown)";
 }
}

int APS5_VABI sceAjmDecWVorbisCreateHeaderPacket(void) {
    NotImplemented_nid_no_patch(__func__);
    return 0;
}

int APS5_VABI sceAjmDecWVorbisCreateSetupPacket(void) {
    NotImplemented_nid_no_patch(__func__);
    return 0;
}

}
