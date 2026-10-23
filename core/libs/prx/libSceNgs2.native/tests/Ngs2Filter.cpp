#include "Ngs2Test.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <vector>

using Response = std::array<float, 12>;

static constexpr float Q = 0.707f;
static constexpr Response FC_1000 = {0.0019580326f, 0.00747051649f, 0.0138923051f, 0.019011084f, 0.0229669027f, 0.0258943513f,
                                     0.0279213972f, 0.0291684661f, 0.0297478512f, 0.0297633335f, 0.0293099806f, 0.0284741335f};
static constexpr Response FC_LOW_CLAMP = {4.46836239e-08f, 1.78694776e-07f, 3.57270437e-07f, 5.3568732e-07f, 7.13945383e-07f, 8.92044852e-07f,
                                          1.0699855e-06f, 1.24776761e-06f, 1.42539102e-06f, 1.60285617e-06f, 1.78016262e-06f, 1.95731081e-06f};
static constexpr Response FC_HIGH_CLAMP = {0.497783214f, 0.00442367792f, -0.00440394599f, 0.00438421499f, -0.00436448585f, 0.00434475951f,
                                           -0.00432503596f, 0.0043053166f, -0.00428560097f, 0.00426589232f, -0.00424618833f, 0.00422649132f};
static constexpr Response Q_LOW_CLAMP = {3.2721498e-06f, 6.55422627e-06f, 6.5541708e-06f, 6.55405438e-06f, 6.5539989e-06f, 6.5538834e-06f,
                                         6.55382746e-06f, 6.55371196e-06f, 6.55365557e-06f, 6.55354006e-06f, 6.55348413e-06f, 6.55336862e-06f};
static constexpr Response Q_HIGH_CLAMP = {0.00213442557f, 0.00849256851f, 0.016814243f, 0.0248149335f, 0.0323593318f, 0.0393204764f,
                                          0.0455818586f, 0.0510394238f, 0.0556032583f, 0.0591990575f, 0.0617693663f, 0.0632744804f};
static constexpr Response FC_1000_AT_24000 = {0.00720055448f, 0.0261592902f, 0.0449450873f, 0.0553287566f, 0.0593121126f, 0.0586462021f,
                                              0.0548080765f, 0.0490004346f, 0.0421672612f, 0.0350195095f, 0.0280662924f, 0.0216479376f};

static Ngs2SamplerVoiceFilterParam Filter(std::uint32_t index, std::uint32_t location, std::uint32_t type, std::uint64_t mask, float frequency, float q, float level) {
    Ngs2SamplerVoiceFilterParam param{};
    param.header = {sizeof(param), 0, SCE_NGS2_SAMPLER_VOICE_PARAM_FILTER};
    auto* bytes = reinterpret_cast<std::uint8_t*>(&param);
    std::memcpy(bytes + 0x8, &index, sizeof(index));
    std::memcpy(bytes + 0xC, &location, sizeof(location));
    std::memcpy(bytes + 0x10, &mask, sizeof(mask));
    std::memcpy(bytes + 0x18, &type, sizeof(type));
    std::memcpy(bytes + 0x1C, &frequency, sizeof(frequency));
    std::memcpy(bytes + 0x20, &q, sizeof(q));
    std::memcpy(bytes + 0x24, &level, sizeof(level));
    return param;
}

static void SetFilter(uintptr_t voice, const Ngs2SamplerVoiceFilterParam& param) {
    Require(sceNgs2VoiceControl(voice, &param.header) == SCE_NGS2_OK);
}

static uintptr_t Sampler(uintptr_t system, const std::vector<std::int16_t>& pcm, std::uint32_t channels, std::uint32_t rate) {
    const auto voice = Voice(CreateRack(system, SCE_NGS2_RACK_ID_SAMPLER));
    Control(voice, SCE_NGS2_SAMPLER_VOICE_PARAM_SETUP, Ngs2SamplerVoiceSetupParam{{}, {SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, channels, rate, 0, 0, 0}});
    const auto frames = static_cast<std::uint32_t>(pcm.size() / channels);
    const Ngs2WaveformBlock block{0, pcm.size() * sizeof(std::int16_t), 0, 0, frames, 0, 0};
    Control(voice, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS, Ngs2SamplerVoiceWaveformBlocksParam{{}, pcm.data(), 0, 1, &block});
    return voice;
}

static std::vector<float> RenderGrain(uintptr_t system, std::uint32_t channels) {
    std::vector<float> out(Grain * channels, -1.0f);
    const Ngs2RenderBufferInfo info{out.data(), out.size() * sizeof(float), SCE_NGS2_WAVEFORM_TYPE_PCM_F32L, channels};
    Require(sceNgs2SystemRender(system, &info, 1) == SCE_NGS2_OK);
    return out;
}

static std::vector<std::int16_t> Impulse(std::size_t frames) {
    std::vector<std::int16_t> pcm(frames, 0);
    pcm[0] = 16384;
    return pcm;
}

static std::vector<float> ImpulseResponse(const Ngs2SamplerVoiceFilterParam& param, std::uint32_t rate = 48000, std::uint32_t frames = 2 * Grain) {
    const auto system = CreateSystem();
    Require(sceNgs2SystemSetSampleRate(system, rate) == SCE_NGS2_OK);
    const auto master = Mastering(system, 1);
    const auto pcm = Impulse(frames);
    const auto sampler = Sampler(system, pcm, 1, rate);
    Patch(sampler, master);
    SetFilter(sampler, param);
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    std::vector<float> out;
    while (out.size() < frames) {
        const auto grain = RenderGrain(system, 1);
        out.insert(out.end(), grain.begin(), grain.end());
    }
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
    return out;
}

static void RequireResponse(const std::vector<float>& out, const Response& expected, float tolerance) {
    for (std::size_t i = 0; i < expected.size(); i++) Require(std::abs(out[i] - expected[i]) <= tolerance * std::abs(expected[i]));
}

static void TestLowPassAndTail() {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 1);
    const auto pcm = Impulse(12);
    const auto sampler = Sampler(system, pcm, 1, 48000);
    Patch(sampler, master);
    const auto param = Filter(7, 1, 1, 0, 1000.0f, Q, 0.75f);
    SetFilter(sampler, param);
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);

    auto out = RenderGrain(system, 1);
    Require(Flags(sampler) == (SCE_NGS2_VOICE_STATE_FLAG_INUSE | SCE_NGS2_VOICE_STATE_FLAG_PLAYING));
    SetFilter(sampler, param);
    const auto second = RenderGrain(system, 1);
    out.insert(out.end(), second.begin(), second.end());
    RequireResponse(out, FC_1000, 1e-5f);
    Require(out[2 * Grain - 1] != 0.0f);
    Require(Flags(sampler) == 0);

    for (float sample : RenderGrain(system, 1)) Require(sample == 0.0f);
    Require(Flags(sampler) == 0);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static void TestBypassMaskIgnoresLevel() {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 2);
    std::vector<std::int16_t> pcm;
    for (std::uint32_t i = 0; i < Grain; i++) {
        pcm.push_back(static_cast<std::int16_t>(i * 1000));
        pcm.push_back(static_cast<std::int16_t>(i == 0 ? 16384 : 0));
    }
    const auto sampler = Sampler(system, pcm, 2, 48000);
    Patch(sampler, master);
    SetFilter(sampler, Filter(0, 1, 1, 1, 1000.0f, Q, 0.5f));
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);

    const auto out = RenderGrain(system, 2);
    for (std::uint32_t i = 0; i < Grain; i++) {
        Require(out[i * 2] == pcm[i * 2] / 32768.0f);
        Require(std::abs(out[i * 2 + 1] - FC_1000[i]) <= 1e-5f * FC_1000[i]);
    }
    Require(Flags(sampler) == 0);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static void TestPs5ClampsAndIgnoredLevel() {
    RequireResponse(ImpulseResponse(Filter(0, 1, 1, 0, 0.0f, Q, 1.0f)), FC_LOW_CLAMP, 1e-3f);
    RequireResponse(ImpulseResponse(Filter(0, 1, 1, 0, -1.0f, Q, 1.0f)), FC_LOW_CLAMP, 1e-3f);
    RequireResponse(ImpulseResponse(Filter(0, 1, 1, 0, 24000.0f, Q, 1.0f)), FC_HIGH_CLAMP, 1e-4f);
    RequireResponse(ImpulseResponse(Filter(0, 1, 1, 0, INFINITY, Q, 1.0f)), FC_HIGH_CLAMP, 1e-4f);
    RequireResponse(ImpulseResponse(Filter(0, 1, 1, 0, 1000.0f, 0.0f, 1.0f)), Q_LOW_CLAMP, 1e-4f);
    RequireResponse(ImpulseResponse(Filter(0, 1, 1, 0, 1000.0f, -1.0f, 1.0f)), Q_LOW_CLAMP, 1e-4f);
    RequireResponse(ImpulseResponse(Filter(0, 1, 1, 0, 1000.0f, 1e6f, 1.0f)), Q_HIGH_CLAMP, 1e-4f);
    RequireResponse(ImpulseResponse(Filter(0, 1, 1, 0, 1000.0f, INFINITY, 1.0f)), Q_HIGH_CLAMP, 1e-4f);
    for (float level : {0.0f, 2.0f, -1.0f, NAN}) RequireResponse(ImpulseResponse(Filter(0, 1, 1, 0, 1000.0f, Q, level)), FC_1000, 1e-5f);
    for (std::uint32_t location : {0u, 2u}) RequireResponse(ImpulseResponse(Filter(0, location, 1, 0, 1000.0f, Q, 1.0f)), FC_1000, 1e-5f);
    RequireResponse(ImpulseResponse(Filter(0, 1, 1, 0, 1000.0f, Q, 1.0f), 24000), FC_1000_AT_24000, 1e-5f);
    for (const auto& param : {Filter(0, 1, 1, 0, NAN, Q, 1.0f), Filter(0, 1, 1, 0, 1000.0f, NAN, 1.0f)}) {
        for (float sample : ImpulseResponse(param)) Require(std::isnan(sample));
    }
}

static void TestRejectedParams() {
    const auto system = CreateSystem();
    const auto sampler = Sampler(system, std::vector<std::int16_t>(Grain, 0), 1, 48000);
    auto small = Filter(0, 1, 1, 0, 1000.0f, 1.0f, 1.0f);
    small.header.size = sizeof(small) - 4;
    bool rejected = false;
    try {
        sceNgs2VoiceControl(sampler, &small.header);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    Require(rejected);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static void TestRateChangeUnderFilter() {
    const auto system = CreateSystem();
    const auto sampler = Sampler(system, std::vector<std::int16_t>(Grain, 0), 1, 48000);
    SetFilter(sampler, Filter(0, 1, 1, 0, 1000.0f, 1.0f, 1.0f));
    Require(sceNgs2SystemSetSampleRate(system, 48000) == SCE_NGS2_OK);
    bool rejected = false;
    try {
        sceNgs2SystemSetSampleRate(system, 96000);
    } catch (const std::runtime_error&) {
        rejected = true;
    }
    Require(rejected);
    SetFilter(sampler, Filter(0, 1, 0, 0, 0.0f, 1.0f, 1.0f));
    Require(sceNgs2SystemSetSampleRate(system, 96000) == SCE_NGS2_OK);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static void TestFilterIndexReturnsCode() {
    RequireResponse(ImpulseResponse(Filter(7, 1, 1, 0, 1000.0f, Q, 1.0f)), FC_1000, 1e-5f);
    const auto system = CreateSystem();
    const auto master = Mastering(system, 1);
    const auto pcm = Impulse(Grain);
    const auto sampler = Sampler(system, pcm, 1, 48000);
    Patch(sampler, master);
    const auto param = Filter(8, 1, 1, 0, 1000.0f, Q, 1.0f);
    Require(sceNgs2VoiceControl(sampler, &param.header) == SCE_NGS2_ERROR_INVALID_VOICE_PARAM_ID);
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    const auto out = RenderGrain(system, 1);
    for (std::uint32_t i = 0; i < Grain; i++) Require(out[i] == (i == 0 ? 0.5f : 0.0f));
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static void TestFilterBeforeSetupIsCleared() {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 1);
    const auto sampler = Voice(CreateRack(system, SCE_NGS2_RACK_ID_SAMPLER));
    SetFilter(sampler, Filter(0, 1, 1, 0, 1000.0f, Q, 1.0f));
    const auto pcm = Impulse(Grain);
    Control(sampler, SCE_NGS2_SAMPLER_VOICE_PARAM_SETUP, Ngs2SamplerVoiceSetupParam{{}, {SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, 1, 48000, 0, 0, 0}});
    const Ngs2WaveformBlock block{0, pcm.size() * sizeof(std::int16_t), 0, 0, Grain, 0, 0};
    Control(sampler, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS, Ngs2SamplerVoiceWaveformBlocksParam{{}, pcm.data(), 0, 1, &block});
    Patch(sampler, master);
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    const auto out = RenderGrain(system, 1);
    for (std::uint32_t i = 0; i < Grain; i++) Require(out[i] == (i == 0 ? 0.5f : 0.0f));
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

template <typename TCall>
static bool ThrowsRuntimeError(TCall call) {
    try {
        call();
    } catch (const std::runtime_error&) {
        return true;
    }
    return false;
}

static void TestUnmeasuredFilterCasesThrow() {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 1);
    const auto pcm = Impulse(Grain);
    const auto sampler = Sampler(system, pcm, 1, 48000);
    for (const auto& param : {Filter(0, 3, 1, 0, 1000.0f, Q, 1.0f), Filter(0, 1, 2, 0, 1000.0f, Q, 1.0f)}) {
        Require(ThrowsRuntimeError([&] { sceNgs2VoiceControl(sampler, &param.header); }));
    }
    const auto slow = Sampler(system, pcm, 1, 24000);
    const auto lowPass = Filter(0, 1, 1, 0, 1000.0f, Q, 1.0f);
    Require(ThrowsRuntimeError([&] { sceNgs2VoiceControl(slow, &lowPass.header); }));

    Patch(sampler, master);
    Control(sampler, SCE_NGS2_VOICE_PARAM_PATCH, Ngs2VoicePatchParam{{}, 1, 0, master});
    SetFilter(sampler, Filter(0, 2, 1, 0, 1000.0f, Q, 1.0f));
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    std::vector<float> out(Grain);
    const Ngs2RenderBufferInfo info{out.data(), out.size() * sizeof(float), SCE_NGS2_WAVEFORM_TYPE_PCM_F32L, 1};
    Require(ThrowsRuntimeError([&] { sceNgs2SystemRender(system, &info, 1); }));
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static void TestTailIsNotFlushed() {
    const auto out = ImpulseResponse(Filter(0, 1, 1, 0, 8000.0f, Q, 1.0f), 48000, 16 * Grain);
    Require(out[100] != 0.0f && std::isfinite(out[100]));
}

int main() {
    TestLowPassAndTail();
    TestBypassMaskIgnoresLevel();
    TestPs5ClampsAndIgnoredLevel();
    TestRejectedParams();
    TestFilterIndexReturnsCode();
    TestRateChangeUnderFilter();
    TestFilterBeforeSetupIsCleared();
    TestUnmeasuredFilterCasesThrow();
    TestTailIsNotFlushed();
    return 0;
}
