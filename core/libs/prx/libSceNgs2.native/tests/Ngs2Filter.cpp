#include "Ngs2Test.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <numbers>
#include <stdexcept>
#include <utility>
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

    const Ngs2WaveformBlock block{0, pcm.size() * sizeof(std::int16_t), 0, 0, static_cast<std::uint32_t>(pcm.size()), 0, 0};
    Control(sampler, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS, Ngs2SamplerVoiceWaveformBlocksParam{{}, pcm.data(), SCE_NGS2_WAVEFORM_BLOCKS_FLAG_RESET, 1, &block});
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    out = RenderGrain(system, 1);
    const auto replay = RenderGrain(system, 1);
    out.insert(out.end(), replay.begin(), replay.end());
    RequireResponse(out, FC_1000, 1e-5f);
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
    const auto badIndex = Filter(8, 1, 1, 0, 1000.0f, Q, 1.0f);
    const auto badLocation = Filter(0, 10, 1, 0, 1000.0f, Q, 1.0f);
    const auto badType = Filter(0, 1, 12, 0, 1000.0f, Q, 1.0f);
    Require(sceNgs2VoiceControl(sampler, &badIndex.header) == SCE_NGS2_ERROR_INVALID_VOICE_PARAM_ID);
    Require(sceNgs2VoiceControl(sampler, &badLocation.header) == SCE_NGS2_ERROR_INVALID_MODULE_LOCATION);
    Require(sceNgs2VoiceControl(sampler, &badType.header) == SCE_NGS2_ERROR_INVALID_FILTER_TYPE);
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
    for (const auto& param : {Filter(0, 1, 2, 0, 1000.0f, Q, 1.0f), Filter(0, 1, 11, 0, 1000.0f, Q, 1.0f), Filter(8, 10, 1, 0, 1000.0f, Q, 1.0f),
                              Filter(0, 10, 12, 0, 1000.0f, Q, 1.0f)}) {
        Require(ThrowsRuntimeError([&] { sceNgs2VoiceControl(sampler, &param.header); }));
    }
    auto small = Filter(8, 1, 1, 0, 1000.0f, Q, 1.0f);
    small.header.size = 0x30;
    Require(ThrowsRuntimeError([&] { sceNgs2VoiceControl(sampler, &small.header); }));

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

static std::vector<float> RenderGrains(uintptr_t system, std::size_t count) {
    std::vector<float> out;
    for (std::size_t i = 0; i < count; i++) {
        const auto grain = RenderGrain(system, 1);
        out.insert(out.end(), grain.begin(), grain.end());
    }
    return out;
}

static std::vector<float> ResampledOutput(std::uint32_t rate, float pitch, const Ngs2SamplerVoiceFilterParam& param) {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 1);
    const auto pcm = Impulse(64 * Grain);
    const auto sampler = Sampler(system, pcm, 1, rate);
    Patch(sampler, master);
    Control(sampler, SCE_NGS2_SAMPLER_VOICE_PARAM_PITCH, Ngs2SamplerVoicePitchParam{{}, pitch});
    SetFilter(sampler, param);
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    const auto out = RenderGrains(system, 16);
    Require(Flags(sampler) == (SCE_NGS2_VOICE_STATE_FLAG_INUSE | SCE_NGS2_VOICE_STATE_FLAG_PLAYING));
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
    return out;
}

static void TestFilterRunsAfterResamplingAtSystemRate() {
    const double omega = 2.0 * std::numbers::pi * 1000.0 / 48000.0;
    const double alpha = std::sin(omega) / (2.0 * Q);
    const double b0 = (1.0 - std::cos(omega)) / 2.0 / (1.0 + alpha);
    const double a1 = -2.0 * std::cos(omega) / (1.0 + alpha);
    const double a2 = (1.0 - alpha) / (1.0 + alpha);
    const std::pair<std::uint32_t, float> cases[] = {{24000, 1.0f}, {44100, 1.0f}, {96000, 1.0f}, {48000, 0.5f}, {48000, 2.0f}, {24000, 2.0f}};
    for (const auto& [rate, pitch] : cases) {
        const auto unfiltered = ResampledOutput(rate, pitch, Filter(0, 1, 0, 0, 0.0f, Q, 1.0f));
        std::vector<double> expected;
        double x1 = 0.0, x2 = 0.0, y1 = 0.0, y2 = 0.0, peak = 0.0;
        for (const float x : unfiltered) {
            const double y = b0 * x + 2.0 * b0 * x1 + b0 * x2 - a1 * y1 - a2 * y2;
            x2 = x1;
            x1 = x;
            y2 = y1;
            y1 = y;
            expected.push_back(y);
            peak = std::max(peak, std::abs(y));
        }
        Require(peak > 0.01);
        const auto front = ResampledOutput(rate, pitch, Filter(0, 0, 1, 0, 1000.0f, Q, 1.0f));
        for (std::size_t i = 0; i < front.size(); i++) Require(std::abs(front[i] - expected[i]) <= 1e-5 * peak);
        for (std::uint32_t location : {1u, 2u}) Require(ResampledOutput(rate, pitch, Filter(0, location, 1, 0, 1000.0f, Q, 1.0f)) == front);
    }
}

static std::vector<float> RenderAfterRejectedParam(const Ngs2SamplerVoiceFilterParam* rejected, int expected) {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 1);
    const auto pcm = Impulse(4 * Grain);
    const auto sampler = Sampler(system, pcm, 1, 48000);
    Patch(sampler, master);
    SetFilter(sampler, Filter(0, 1, 1, 0, 1000.0f, Q, 1.0f));
    if (rejected != nullptr) Require(sceNgs2VoiceControl(sampler, &rejected->header) == expected);
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    const auto out = RenderGrains(system, 4);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
    return out;
}

static void TestRejectedParamKeepsFilter() {
    const auto kept = RenderAfterRejectedParam(nullptr, SCE_NGS2_OK);
    RequireResponse(kept, FC_1000, 1e-5f);
    auto small = Filter(0, 1, 1, 0, 4000.0f, Q, 1.0f);
    small.header.size = 0x30;
    const std::pair<Ngs2SamplerVoiceFilterParam, int> rejected[] = {
        {Filter(0, 18, 1, 0, 1000.0f, Q, 1.0f), SCE_NGS2_ERROR_INVALID_MODULE_LOCATION},
        {Filter(0, 18, 1, 0, 4000.0f, Q, 1.0f), SCE_NGS2_ERROR_INVALID_MODULE_LOCATION},
        {Filter(0, 0xffffffff, 1, 0, 4000.0f, Q, 1.0f), SCE_NGS2_ERROR_INVALID_MODULE_LOCATION},
        {Filter(0, 1, 12, 0, 4000.0f, Q, 1.0f), SCE_NGS2_ERROR_INVALID_FILTER_TYPE},
        {Filter(0, 1, 100, 0, 4000.0f, Q, 1.0f), SCE_NGS2_ERROR_INVALID_FILTER_TYPE},
        {small, SCE_NGS2_ERROR_INVALID_VOICE_CONTROL_SIZE},
    };
    for (const auto& [param, code] : rejected) Require(RenderAfterRejectedParam(&param, code) == kept);
    auto large = Filter(0, 1, 1, 0, 1000.0f, Q, 1.0f);
    large.header.size = 0x40;
    Require(RenderAfterRejectedParam(&large, SCE_NGS2_OK) == kept);
}

static std::vector<float> PortsOutput(std::initializer_list<std::uint32_t> ports, std::uint32_t location) {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 1);
    const auto pcm = Impulse(4 * Grain);
    const auto sampler = Sampler(system, pcm, 1, 48000);
    for (const auto port : ports) Control(sampler, SCE_NGS2_VOICE_PARAM_PATCH, Ngs2VoicePatchParam{{}, port, 0, master});
    SetFilter(sampler, Filter(0, location, 1, 0, 1000.0f, Q, 1.0f));
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    std::vector<float> out;
    const bool threw = ThrowsRuntimeError([&] { out = RenderGrains(system, 2); });
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
    if (threw) out.clear();
    return out;
}

static void TestLocationsFilterTheirPorts() {
    std::vector<float> unfiltered(2 * Grain, 0.0f);
    unfiltered[0] = 0.5f;
    for (const auto& [port, location] : {std::pair{0u, 0u}, std::pair{1u, 1u}, std::pair{0u, 2u}, std::pair{1u, 3u}, std::pair{7u, 9u}}) {
        RequireResponse(PortsOutput({port}, location), FC_1000, 1e-5f);
    }
    for (std::uint32_t location : {0u, 1u}) {
        auto both = PortsOutput({0, 7}, location);
        for (auto& sample : both) sample /= 2.0f;
        RequireResponse(both, FC_1000, 1e-5f);
    }
    Require(PortsOutput({1}, 2) == unfiltered);
    Require(PortsOutput({1}, 4) == unfiltered);
    Require(PortsOutput({0}, 9) == unfiltered);
    Require(PortsOutput({0, 1}, 3).empty());

    const auto system = CreateSystem();
    const auto sampler = Sampler(system, Impulse(Grain), 1, 48000);
    for (std::uint32_t location = 0; location < 10; location++) SetFilter(sampler, Filter(0, location, 1, 0, 1000.0f, Q, 1.0f));
    for (std::uint32_t location : {10u, 17u, 0xffffffffu}) {
        const auto param = Filter(0, location, 1, 0, 1000.0f, Q, 1.0f);
        Require(sceNgs2VoiceControl(sampler, &param.header) == SCE_NGS2_ERROR_INVALID_MODULE_LOCATION);
    }
    Ngs2SamplerRackOption option{};
    option.rack_option.size = sizeof(option);
    option.rack_option.max_grain_samples = 512;
    option.rack_option.max_voices = 1;
    option.rack_option.max_matrices = 1;
    option.rack_option.max_ports = 1;
    option.max_filters = 8;
    Ngs2ContextBufferInfo query{};
    Require(sceNgs2RackQueryBufferSize(SCE_NGS2_RACK_ID_SAMPLER, &option.rack_option, &query) == SCE_NGS2_OK);
    const auto info = Buffer(query);
    uintptr_t rack = 0;
    Require(sceNgs2RackCreate(system, SCE_NGS2_RACK_ID_SAMPLER, &option.rack_option, &info, &rack) == SCE_NGS2_OK);
    const auto onePort = Voice(rack);
    SetFilter(onePort, Filter(0, 2, 1, 0, 1000.0f, Q, 1.0f));
    const auto past = Filter(0, 3, 1, 0, 1000.0f, Q, 1.0f);
    Require(sceNgs2VoiceControl(onePort, &past.header) == SCE_NGS2_ERROR_INVALID_MODULE_LOCATION);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static void AddBlock(uintptr_t sampler, const std::vector<std::int16_t>& pcm) {
    const Ngs2WaveformBlock block{0, pcm.size() * sizeof(std::int16_t), 0, 0, static_cast<std::uint32_t>(pcm.size()), 0, 0};
    Control(sampler, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS, Ngs2SamplerVoiceWaveformBlocksParam{{}, pcm.data(), SCE_NGS2_WAVEFORM_BLOCKS_FLAG_RESET, 1, &block});
}

static std::vector<std::int16_t> SecondBlock() {
    std::vector<std::int16_t> pcm(4 * Grain, 0);
    pcm[3] = -8192;
    return pcm;
}

static std::vector<float> FreshOutput(bool filter) {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 1);
    const auto pcm = SecondBlock();
    const auto sampler = Sampler(system, pcm, 1, 48000);
    Patch(sampler, master);
    if (filter) SetFilter(sampler, Filter(0, 1, 1, 0, 1000.0f, Q, 1.0f));
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    const auto out = RenderGrains(system, 4);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
    return out;
}

template <typename TBetween>
static std::vector<float> RestartOutput(std::uint32_t firstFrames, std::size_t firstGrains, TBetween between) {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 1);
    const std::vector<std::int16_t> first(firstFrames, 16384);
    const auto sampler = Sampler(system, first, 1, 48000);
    Patch(sampler, master);
    SetFilter(sampler, Filter(0, 1, 1, 0, 1000.0f, Q, 1.0f));
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    RenderGrains(system, firstGrains);
    between(sampler, master);
    const auto second = SecondBlock();
    AddBlock(sampler, second);
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    const auto out = RenderGrains(system, 4);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
    return out;
}

static void TestRestartClearsFilterHistory() {
    const auto fresh = FreshOutput(true);
    const auto unfiltered = FreshOutput(false);
    Require(unfiltered[3] == -0.25f && fresh[3] < 0.0f && fresh[3] > -0.01f);
    const auto none = [](uintptr_t, uintptr_t) {};
    Require(RestartOutput(Grain, 2, none) == fresh);
    Require(RestartOutput(64 * Grain, 1, none) == fresh);
    for (std::uint32_t event : {SCE_NGS2_VOICE_EVENT_STOP_IMM, SCE_NGS2_VOICE_EVENT_KILL}) {
        Require(RestartOutput(64 * Grain, 1, [event](uintptr_t sampler, uintptr_t) { Event(sampler, event); }) == fresh);
    }
    const auto setupAgain = [](uintptr_t sampler, uintptr_t master) {
        Control(sampler, SCE_NGS2_SAMPLER_VOICE_PARAM_SETUP, Ngs2SamplerVoiceSetupParam{{}, {SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, 1, 48000, 0, 0, 0}});
        Patch(sampler, master);
    };
    Require(RestartOutput(64 * Grain, 1, setupAgain) == unfiltered);
}

static void TestResentFilterKeepsHistory() {
    const auto render = [](const Ngs2SamplerVoiceFilterParam* resent) {
        const auto system = CreateSystem();
        const auto master = Mastering(system, 1);
        const auto pcm = Impulse(64 * Grain);
        const auto sampler = Sampler(system, pcm, 1, 48000);
        Patch(sampler, master);
        SetFilter(sampler, Filter(0, 1, 1, 0, 1000.0f, Q, 1.0f));
        Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
        auto out = RenderGrains(system, 1);
        if (resent != nullptr) SetFilter(sampler, *resent);
        const auto next = RenderGrains(system, 1);
        out.insert(out.end(), next.begin(), next.end());
        Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
        return out;
    };
    const auto same = Filter(0, 1, 1, 0, 1000.0f, Q, 1.0f);
    Require(render(&same) == render(nullptr));
    const auto higher = Filter(0, 1, 1, 0, 4000.0f, Q, 1.0f);
    const auto out = render(&higher);
    const double omega = 2.0 * std::numbers::pi * 4000.0 / 48000.0;
    const double alpha = std::sin(omega) / (2.0 * Q);
    const double a1 = -2.0 * std::cos(omega) / (1.0 + alpha);
    const double a2 = (1.0 - alpha) / (1.0 + alpha);
    for (std::size_t n = Grain; n < out.size(); n++) Require(std::abs(out[n] + a1 * out[n - 1] + a2 * out[n - 2]) <= 1e-5 * std::abs(out[n]));
}

int main() {
    TestLowPassAndTail();
    TestBypassMaskIgnoresLevel();
    TestPs5ClampsAndIgnoredLevel();
    TestFilterIndexReturnsCode();
    TestRateChangeUnderFilter();
    TestFilterBeforeSetupIsCleared();
    TestUnmeasuredFilterCasesThrow();
    TestTailIsNotFlushed();
    TestFilterRunsAfterResamplingAtSystemRate();
    TestRejectedParamKeepsFilter();
    TestLocationsFilterTheirPorts();
    TestRestartClearsFilterHistory();
    TestResentFilterKeepsHistory();
    return 0;
}
