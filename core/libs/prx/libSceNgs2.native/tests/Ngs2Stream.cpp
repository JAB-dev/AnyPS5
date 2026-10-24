#include "Ngs2Test.hpp"

#include <cstdint>
#include <stdexcept>
#include <vector>

static uintptr_t RackVoice(uintptr_t rack, std::uint32_t index) {
    uintptr_t voice = 0;
    Require(sceNgs2RackGetVoiceHandle(rack, index, &voice) == SCE_NGS2_OK && voice != 0);
    return voice;
}

static void AddBlock(uintptr_t voice, const void* data, std::uint64_t bytes, std::uint32_t samples, std::uint32_t flags) {
    const Ngs2WaveformBlock block{0, bytes, 0, 0, samples, 0, 0};
    Control(voice, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS, Ngs2SamplerVoiceWaveformBlocksParam{{}, data, flags, 1, &block});
}

template <typename TFunction>
static bool Throws(TFunction function) {
    try {
        function();
    } catch (const std::exception&) {
        return true;
    }
    return false;
}

static const std::int16_t Unused = 0;

static void TestNoBlocksForClearedVoice() {
    const auto system = CreateSystem();
    const auto rack = CreateRack(system, SCE_NGS2_RACK_ID_SAMPLER);
    const auto voice = RackVoice(rack, 0);
    Control(voice, SCE_NGS2_SAMPLER_VOICE_PARAM_SETUP, Ngs2SamplerVoiceSetupParam{{}, {}});
    const Ngs2WaveformBlock unused{};
    Control(voice, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS, Ngs2SamplerVoiceWaveformBlocksParam{{}, &Unused, 0, 0, &unused});
    Control(voice, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS, Ngs2SamplerVoiceWaveformBlocksParam{{}, &Unused, SCE_NGS2_WAVEFORM_BLOCKS_FLAG_RESET, 0, &unused});
    const float level = 1.0f;
    Control(voice, SCE_NGS2_VOICE_PARAM_MATRIX_LEVELS, Ngs2VoiceMatrixLevelsParam{{}, 0, 0, &level});
    Require(Throws([&] { Control(voice, SCE_NGS2_VOICE_PARAM_MATRIX_LEVELS, Ngs2VoiceMatrixLevelsParam{{}, 1, 0, &level}); }));
    Require(Flags(voice) == 0);
    const std::vector<std::int16_t> pcm(4, 16384);
    Require(Throws([&] { AddBlock(voice, pcm.data(), pcm.size() * 2, 4, 0); }));
    Require(Throws([&] { Control(voice, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS, Ngs2SamplerVoiceWaveformBlocksParam{{}, &Unused, SCE_NGS2_WAVEFORM_BLOCKS_FLAG_APPEND, 0, &unused}); }));
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

int main() {
    TestNoBlocksForClearedVoice();
    return 0;
}
