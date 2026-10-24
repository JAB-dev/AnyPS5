#include "Ngs2Test.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <sys/mman.h>
#include <unistd.h>
#endif

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

struct Coefficients {
    float i0;
    float i1;
    float i2;
    float o1;
    float o2;
};

static float Bits(std::uint32_t bits) {
    return std::bit_cast<float>(bits);
}

static Coefficients Cookbook(std::uint32_t b0, std::uint32_t b1, std::uint32_t b2, std::uint32_t a1, std::uint32_t a2) {
    return {Bits(b0), Bits(b1), Bits(b2), -Bits(a1), -Bits(a2)};
}

static const Coefficients LOW_PASS_1000 = Cookbook(0x3b805256, 0x3c005256, 0x3b805256, 0xbfe85c56, 0x3f54bb3f);
static const Coefficients LOW_PASS_4000 = Cookbook(0x3d4ab3f0, 0x3dcab3f0, 0x3d4ab3f0, 0xbfa3c957, 0x3ef47f54);

struct BlockWeights {
    float h[4];
    float c[4];
    float d[4];
    float e1[4];
    float e2[4];
};

static BlockWeights Expand(const Coefficients& f) {
    BlockWeights b{};
    b.h[0] = f.i0;
    b.h[1] = f.i1 + f.o1 * b.h[0];
    b.h[2] = (f.i2 + f.o2 * b.h[0]) + f.o1 * b.h[1];
    b.h[3] = f.o2 * b.h[1] + f.o1 * b.h[2];
    b.c[0] = f.o1;
    b.c[1] = f.o2 + f.o1 * b.c[0];
    b.c[2] = f.o2 * b.c[0] + f.o1 * b.c[1];
    b.c[3] = f.o2 * b.c[1] + f.o1 * b.c[2];
    b.d[0] = f.o2;
    b.d[1] = f.o1 * b.d[0];
    b.d[2] = f.o2 * b.d[0] + f.o1 * b.d[1];
    b.d[3] = f.o2 * b.d[1] + f.o1 * b.d[2];
    b.e1[0] = f.i1;
    b.e1[1] = f.i2 + f.o1 * b.e1[0];
    b.e1[2] = f.o2 * b.e1[0] + f.o1 * b.e1[1];
    b.e1[3] = f.o2 * b.e1[1] + f.o1 * b.e1[2];
    b.e2[0] = f.i2;
    b.e2[1] = f.o1 * b.e2[0];
    b.e2[2] = f.o2 * b.e2[0] + f.o1 * b.e2[1];
    b.e2[3] = f.o2 * b.e2[1] + f.o1 * b.e2[2];
    return b;
}

static std::vector<float> BlockModel(const std::vector<float>& u, const Coefficients& first, const Coefficients& second, std::size_t switchAt) {
    const BlockWeights weights[2] = {Expand(first), Expand(second)};
    std::vector<float> y(u.size(), 0.0f);
    const auto input = [&u](std::ptrdiff_t i) { return i >= 0 ? u[static_cast<std::size_t>(i)] : 0.0f; };
    const auto output = [&y](std::ptrdiff_t i) { return i >= 0 ? y[static_cast<std::size_t>(i)] : 0.0f; };
    for (std::size_t k = 0; k < u.size(); k += 4) {
        const auto& b = weights[k >= switchAt ? 1 : 0];
        const auto at = static_cast<std::ptrdiff_t>(k);
        for (int j = 0; j < 4; j++) {
            const auto h = [&b, j](int m) { return m <= j ? b.h[j - m] : 0.0f; };
            float a = h(2) * input(at + 2);
            a = std::fma(b.e2[j], input(at - 2), a);
            a = std::fma(h(0), input(at), a);
            a = std::fma(b.d[j], output(at - 2), a);
            float c = h(3) * input(at + 3);
            c = std::fma(b.e1[j], input(at - 1), c);
            c = std::fma(h(1), input(at + 1), c);
            c = std::fma(b.c[j], output(at - 1), c);
            y[k + static_cast<std::size_t>(j)] = a + c;
        }
    }
    return y;
}

static std::vector<float> BlockModel(const std::vector<float>& u, const Coefficients& f) {
    return BlockModel(u, f, f, u.size());
}

static bool SameBits(const std::vector<float>& a, const std::vector<float>& b) {
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0;
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
    const auto master = Mastering(system, 1);
    const auto pcm = Impulse(Grain);
    const auto sampler = Sampler(system, pcm, 1, 48000);
    Patch(sampler, master);
    SetFilter(sampler, Filter(0, 1, 1, 1, 1000.0f, Q, 0.5f));
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);

    const auto out = RenderGrain(system, 1);
    for (std::uint32_t i = 0; i < Grain; i++) Require(out[i] == (i == 0 ? 0.5f : 0.0f));
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
    const std::vector<std::int16_t> silence(Grain, 0);
    const auto system = CreateSystem();
    const auto sampler = Sampler(system, silence, 1, 48000);
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
    for (const auto& param : {Filter(8, 10, 1, 0, 1000.0f, Q, 1.0f), Filter(0, 10, 12, 0, 1000.0f, Q, 1.0f)}) {
        Require(ThrowsRuntimeError([&] { sceNgs2VoiceControl(sampler, &param.header); }));
    }

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

static void TestShortHeaderIsNotReadPastItsSize() {
#ifdef _WIN32
    SYSTEM_INFO info{};
    GetSystemInfo(&info);
    const std::size_t page = info.dwPageSize;
    auto* region = static_cast<std::uint8_t*>(VirtualAlloc(nullptr, 2 * page, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE));
    DWORD previous = 0;
    Require(region != nullptr && VirtualProtect(region + page, page, PAGE_NOACCESS, &previous) != 0);
#else
    const auto page = static_cast<std::size_t>(sysconf(_SC_PAGESIZE));
    void* mapping = mmap(nullptr, 2 * page, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    Require(mapping != MAP_FAILED);
    auto* region = static_cast<std::uint8_t*>(mapping);
    Require(mprotect(region + page, page, PROT_NONE) == 0);
#endif
    auto* header = reinterpret_cast<Ngs2VoiceParamHeader*>(region + page - sizeof(Ngs2VoiceParamHeader));
    *header = {sizeof(Ngs2VoiceParamHeader), 0, SCE_NGS2_SAMPLER_VOICE_PARAM_FILTER};
    const auto pcm = Impulse(Grain);
    const auto system = CreateSystem();
    const auto sampler = Sampler(system, pcm, 1, 48000);
    Require(sceNgs2VoiceControl(sampler, header) == SCE_NGS2_ERROR_INVALID_VOICE_CONTROL_SIZE);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
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
    const std::pair<std::uint32_t, float> cases[] = {{24000, 1.0f}, {44100, 1.0f}, {96000, 1.0f}, {48000, 0.5f}, {48000, 2.0f}, {24000, 2.0f}};
    for (const auto& [rate, pitch] : cases) {
        const auto unfiltered = ResampledOutput(rate, pitch, Filter(0, 1, 0, 0, 0.0f, Q, 1.0f));
        Require(std::any_of(unfiltered.begin(), unfiltered.end(), [](float sample) { return sample > 0.01f; }));
        const auto front = ResampledOutput(rate, pitch, Filter(0, 0, 1, 0, 1000.0f, Q, 1.0f));
        Require(SameBits(front, BlockModel(unfiltered, LOW_PASS_1000)));
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

    const auto pcm = Impulse(Grain);
    const auto system = CreateSystem();
    const auto sampler = Sampler(system, pcm, 1, 48000);
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

static const Ngs2SamplerVoiceFilterParam LOW_PASS = Filter(0, 1, 1, 0, 1000.0f, Q, 1.0f);

static std::vector<float> FreshOutput(const Ngs2SamplerVoiceFilterParam* filter) {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 1);
    const auto pcm = SecondBlock();
    const auto sampler = Sampler(system, pcm, 1, 48000);
    Patch(sampler, master);
    if (filter != nullptr) SetFilter(sampler, *filter);
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    const auto out = RenderGrains(system, 4);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
    return out;
}

template <typename TBetween>
static std::vector<float> RestartOutput(std::uint32_t firstFrames, std::size_t firstGrains, TBetween between, const Ngs2SamplerVoiceFilterParam& filter = LOW_PASS) {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 1);
    const std::vector<std::int16_t> first(firstFrames, 16384);
    const auto sampler = Sampler(system, first, 1, 48000);
    Patch(sampler, master);
    SetFilter(sampler, filter);
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
    const auto fresh = FreshOutput(&LOW_PASS);
    const auto unfiltered = FreshOutput(nullptr);
    Require(unfiltered[3] == -0.25f && fresh[3] < 0.0f && fresh[3] > -0.01f);
    const auto none = [](uintptr_t, uintptr_t) {};
    Require(RestartOutput(Grain, 2, none) == fresh);
    for (std::uint32_t event : {SCE_NGS2_VOICE_EVENT_STOP_IMM, SCE_NGS2_VOICE_EVENT_KILL}) {
        Require(RestartOutput(64 * Grain, 1, [event](uintptr_t sampler, uintptr_t) { Event(sampler, event); }) == fresh);
    }
    const auto setupAgain = [](uintptr_t sampler, uintptr_t master) {
        Control(sampler, SCE_NGS2_SAMPLER_VOICE_PARAM_SETUP, Ngs2SamplerVoiceSetupParam{{}, {SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, 1, 48000, 0, 0, 0}});
        Patch(sampler, master);
    };
    Require(RestartOutput(64 * Grain, 1, setupAgain) == unfiltered);
}

static void TestStopAndPlayWithoutNewBlocksClearsHistory() {
    const std::vector<std::int16_t> dc(64 * Grain, 16384);
    const auto render = [&dc](bool restart) {
        const auto system = CreateSystem();
        const auto master = Mastering(system, 1);
        const auto sampler = Sampler(system, dc, 1, 48000);
        Patch(sampler, master);
        SetFilter(sampler, Filter(0, 1, 1, 0, 1000.0f, Q, 1.0f));
        Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
        if (restart) {
            RenderGrains(system, 8);
            Event(sampler, SCE_NGS2_VOICE_EVENT_STOP_IMM);
            Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
        }
        const auto out = RenderGrains(system, 1);
        Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
        return out;
    };
    const auto first = render(false);
    Require(first[0] > 0.0f && first[0] < 0.01f);
    Require(render(true) == first);
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
    std::vector<float> input(2 * Grain, 0.0f);
    input[0] = 0.5f;
    Require(SameBits(render(&higher), BlockModel(input, LOW_PASS_1000, LOW_PASS_4000, Grain)));
}

static constexpr std::size_t ORACLE_FRAMES = 2560;
static constexpr std::size_t CLAMP_FRAMES = 512;
static constexpr std::size_t ORACLE_IMPULSE_AT = 13;

static Ngs2SamplerVoiceFilterParam Direct(const Coefficients& f) {
    auto param = Filter(0, 1, 11, 0, f.i0, f.i1, f.i2);
    auto* bytes = reinterpret_cast<std::uint8_t*>(&param);
    std::memcpy(bytes + 0x28, &f.o1, sizeof(f.o1));
    std::memcpy(bytes + 0x2C, &f.o2, sizeof(f.o2));
    return param;
}

static Ngs2CustomVoiceFilterParam CustomFilter(const Ngs2SamplerVoiceFilterParam& sampler) {
    Ngs2CustomVoiceFilterParam param{};
    param.header = {sizeof(param), 0, SCE_NGS2_CUSTOM_VOICE_PARAM_FILTER};
    param.channel_mask = sampler.channel_mask;
    param.type = sampler.type;
    param.param.fcq.frequency = sampler.frequency;
    param.param.fcq.q = sampler.q;
    param.param.fcq.level = sampler.level;
    param.param.fcq.reserved = sampler.reserved[0];
    param.param.fcq.reserved2 = sampler.reserved[1];
    return param;
}

static uintptr_t CustomFilterVoice(uintptr_t system) {
    Ngs2CustomSamplerRackOption option{};
    option.custom_rack_option.rack_option.size = sizeof(option);
    option.custom_rack_option.rack_option.max_grain_samples = 512;
    option.custom_rack_option.rack_option.max_voices = 1;
    option.custom_rack_option.rack_option.max_matrices = 1;
    option.custom_rack_option.rack_option.max_ports = 1;
    option.custom_rack_option.num_buffers = 1;
    option.custom_rack_option.num_modules = 1;
    option.custom_rack_option.module[0].module_id = SCE_NGS2_CUSTOM_MODULE_ID_FILTER;
    option.max_channel_works = 1;
    Ngs2ContextBufferInfo query{};
    Require(sceNgs2RackQueryBufferSize(SCE_NGS2_RACK_ID_CUSTOM_SAMPLER, &option.custom_rack_option.rack_option, &query) == SCE_NGS2_OK);
    const auto info = Buffer(query);
    uintptr_t rack = 0;
    Require(sceNgs2RackCreate(system, SCE_NGS2_RACK_ID_CUSTOM_SAMPLER, &option.custom_rack_option.rack_option, &info, &rack) == SCE_NGS2_OK);
    return Voice(rack);
}

static std::vector<std::int16_t> OraclePcm() {
    std::vector<std::int16_t> pcm(4096, 0);
    pcm[ORACLE_IMPULSE_AT] = 16384;
    return pcm;
}

static std::vector<float> OracleInput() {
    std::vector<float> input(ORACLE_FRAMES, 0.0f);
    input[ORACLE_IMPULSE_AT] = 0.5f;
    return input;
}

static std::vector<float> FilteredOutput(const std::vector<std::int16_t>& pcm, const Ngs2SamplerVoiceFilterParam& filter, bool custom, int expected, std::size_t frames) {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 1);
    const Ngs2WaveformBlock block{0, pcm.size() * sizeof(std::int16_t), 0, 0, static_cast<std::uint32_t>(pcm.size()), 0, 0};
    uintptr_t voice = 0;
    if (custom) {
        voice = CustomFilterVoice(system);
        Control(voice, SCE_NGS2_CUSTOM_SAMPLER_VOICE_PARAM_SETUP, Ngs2SamplerVoiceSetupParam{{}, {SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, 1, 48000, 0, 0, 0}});
        Control(voice, SCE_NGS2_CUSTOM_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS, Ngs2SamplerVoiceWaveformBlocksParam{{}, pcm.data(), 0, 1, &block});
        const auto param = CustomFilter(filter);
        Require(sceNgs2VoiceControl(voice, &param.header) == expected);
    } else {
        voice = Sampler(system, pcm, 1, 48000);
        Require(sceNgs2VoiceControl(voice, &filter.header) == expected);
    }
    Patch(voice, master);
    Event(voice, SCE_NGS2_VOICE_EVENT_PLAY);
    const auto out = RenderGrains(system, frames / Grain);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
    return out;
}

static std::vector<float> OracleOutput(const Ngs2SamplerVoiceFilterParam& filter, bool custom = false, int expected = SCE_NGS2_OK, std::size_t frames = ORACLE_FRAMES) {
    const auto pcm = OraclePcm();
    return FilteredOutput(pcm, filter, custom, expected, frames);
}

static std::uint64_t Hash(const std::vector<float>& out) {
    std::uint64_t hash = 1469598103934665603ull;
    for (const float sample : out) {
        const auto bits = std::bit_cast<std::uint32_t>(sample);
        for (int k = 0; k < 4; k++) {
            hash ^= (bits >> (8 * k)) & 0xff;
            hash *= 1099511628211ull;
        }
    }
    return hash;
}

static bool AllNan(const std::vector<float>& out) {
    return std::all_of(out.begin(), out.end(), [](float sample) { return std::isnan(sample); });
}

struct OracleCase {
    std::uint32_t type;
    float frequency;
    float q;
    float level;
    std::uint64_t hash;
};

static void TestEveryTypeMatchesPs5Output() {
    const OracleCase cases[] = {
        {1, 1000.0f, Q, 1.0f, 0x0095ebf1d025e56aull},  {1, 10000.0f, 5.0f, 1.0f, 0xa6f6a10cbf05a25eull}, {2, 1000.0f, Q, 1.0f, 0x807d3bf7d9b26fbeull},
        {2, 10000.0f, 5.0f, 1.0f, 0xd59046d880fa7f6cull}, {3, 1000.0f, Q, 1.0f, 0x7d892e6cc02df0efull},  {3, 10000.0f, 5.0f, 1.0f, 0xdad2fa19337c925bull},
        {4, 1000.0f, Q, 1.0f, 0xa4865a5dcffaab81ull},  {4, 10000.0f, 5.0f, 1.0f, 0x1d3bf7f32ccd1d18ull}, {5, 1000.0f, Q, 1.0f, 0xd867a3747a0167fbull},
        {5, 10000.0f, 5.0f, 1.0f, 0xf163b55969a36c16ull}, {6, 1000.0f, Q, 2.0f, 0x4196c4bd1997975cull},  {6, 10000.0f, 5.0f, 0.25f, 0x94486a940e6bd407ull},
        {6, 20000.0f, 0.5f, 4.0f, 0xa465d0867e25cb4full}, {7, 1000.0f, Q, 2.0f, 0x28c13aeaac03d0dbull},  {7, 10000.0f, 5.0f, 0.25f, 0xf3e42b57382d5b98ull},
        {7, 20000.0f, 0.5f, 4.0f, 0xc974f6d9ea02386aull}, {8, 1000.0f, Q, 2.0f, 0x517f0b4c4e7c49e5ull},  {8, 10000.0f, 5.0f, 0.25f, 0xf0cf1f7da1e37d31ull},
        {8, 20000.0f, 0.5f, 4.0f, 0xa81ad720770ae18eull}, {9, 1000.0f, Q, 1.0f, 0xb2ea3fb54cc0ccdfull},  {9, 10000.0f, 5.0f, 1.0f, 0xaf5ca2f664c91ec7ull},
        {10, 1000.0f, Q, 1.0f, 0xa5c96fe50a199cbdull}, {10, 10000.0f, 5.0f, 1.0f, 0x48b228a62429560aull}, {9, 100.0f, 0.5f, 1.0f, 0xa6257a9e174ba461ull},
        {9, 20000.0f, 0.5f, 1.0f, 0x888ce86f1b65bd8cull}, {10, 100.0f, 0.5f, 1.0f, 0xf219922d8f04b5a9ull}, {10, 20000.0f, 0.5f, 1.0f, 0x9c536238aaf24e69ull},
        {8, 10000.0f, 0.5f, 2.0f, 0xa25fa136d6d96c0bull},
    };
    for (const auto& c : cases) Require(Hash(OracleOutput(Filter(0, 1, c.type, 0, c.frequency, c.q, c.level))) == c.hash);
    for (std::uint32_t type : {1u, 2u, 3u, 4u, 5u, 9u, 10u}) {
        Require(SameBits(OracleOutput(Filter(0, 1, type, 0, 1000.0f, Q, 0.25f)), OracleOutput(Filter(0, 1, type, 0, 1000.0f, Q, 4.0f))));
    }
    for (std::uint32_t type : {1u, 7u, 9u}) Require(SameBits(OracleOutput(Filter(0, 0, type, 0, 4000.0f, 2.0f, 2.0f)), OracleOutput(Filter(0, 1, type, 0, 4000.0f, 2.0f, 2.0f))));
}

static void TestEveryTypeMatchesBlockModelOnNoise() {
    std::vector<std::int16_t> pcm(4096);
    std::uint32_t state = 12345;
    for (auto& sample : pcm) {
        state = state * 1664525u + 1013904223u;
        sample = static_cast<std::int16_t>(state >> 16);
    }
    std::vector<float> input(ORACLE_FRAMES);
    for (std::size_t n = 0; n < input.size(); n++) input[n] = pcm[n] / 32768.0f;
    const std::pair<Ngs2SamplerVoiceFilterParam, Coefficients> cases[] = {
        {Filter(0, 1, 1, 0, 1000.0f, Q, 1.0f), LOW_PASS_1000},
        {Filter(0, 1, 2, 0, 1000.0f, Q, 1.0f), Cookbook(0x3f695cfa, 0xbfe95cfa, 0x3f695cfa, 0xbfe85c56, 0x3f54bb3f)},
        {Filter(0, 1, 3, 0, 1000.0f, Q, 1.0f), Cookbook(0x3d74ba18, 0x00000000, 0xbd74ba18, 0xbfe85c56, 0x3f54bb3f)},
        {Filter(0, 1, 4, 0, 1000.0f, Q, 1.0f), Cookbook(0x3dad12ff, 0x00000000, 0xbdad12ff, 0xbfe85c56, 0x3f54bb3f)},
        {Filter(0, 1, 5, 0, 1000.0f, Q, 1.0f), Cookbook(0x3f6a5d9f, 0xbfe85c56, 0x3f6a5d9f, 0xbfe85c56, 0x3f54bb3f)},
        {Filter(0, 1, 6, 0, 1000.0f, Q, 2.0f), Cookbook(0x3f842ee8, 0xbfeb615b, 0x3f542632, 0xbfec1983, 0x3f5b13ac)},
        {Filter(0, 1, 7, 0, 1000.0f, Q, 2.0f), Cookbook(0x3ff7e5f9, 0xc064a0b5, 0x3fd424c8, 0xbfe3ee61, 0x3f4d6f6e)},
        {Filter(0, 1, 8, 0, 1000.0f, Q, 2.0f), Cookbook(0x3f87d7cf, 0xbfee4212, 0x3f50f126, 0xbfee4212, 0x3f60a0c4)},
        {Filter(0, 1, 9, 0, 1000.0f, Q, 1.0f), Cookbook(0x3d7bf3c4, 0x3d7bf3c4, 0x00000000, 0xbf608188, 0x80000000)},
        {Filter(0, 1, 10, 0, 1000.0f, Q, 1.0f), Cookbook(0x3f7040c3, 0xbf7040c3, 0x00000000, 0xbf608187, 0x80000000)},
        {Direct({0.3f, -0.41f, 0.17f, 1.2f, -0.5f}), {0.3f, -0.41f, 0.17f, 1.2f, -0.5f}},
    };
    for (const auto& [filter, coefficients] : cases) {
        for (bool custom : {false, true}) Require(SameBits(FilteredOutput(pcm, filter, custom, SCE_NGS2_OK, ORACLE_FRAMES), BlockModel(input, coefficients)));
    }
}

static void TestDirectCoefficientsRunInBlocksOfFour() {
    const Coefficients sets[] = {
        {0.3f, -0.41f, 0.17f, 1.2f, -0.5f},
        {0.7071f, 0.1234567f, -0.3333333f, -0.9f, -0.3f},
        {1.1f, -1.9f, 0.93f, 1.85f, -0.89f},
        {0.0123f, 0.0246f, 0.0123f, 1.9f, -0.905f},
    };
    for (const auto& set : sets) Require(SameBits(OracleOutput(Direct(set)), BlockModel(OracleInput(), set)));
    const auto response = OracleOutput(Direct({0.5f, 0.25f, 0.125f, 0.5f, -0.25f}));
    const float h[] = {0.5f, 0.5f, 0.25f, 0.0f, -0.0625f, -0.03125f, 0.0f, 0.0078125f};
    for (std::size_t n = 0; n < std::size(h); n++) Require(response[ORACLE_IMPULSE_AT + n] == 0.5f * h[n]);
    Require(OracleOutput(Direct({30000.0f, 0.0f, 0.0f, 0.0f, 0.0f}))[ORACLE_IMPULSE_AT] == 15000.0f);

    const float nan = std::numeric_limits<float>::quiet_NaN();
    Require(AllNan(OracleOutput(Direct({0.5f, 0.0f, 0.0f, nan, 0.0f}))));
    Require(AllNan(OracleOutput(Direct({nan, 0.0f, 0.0f, 0.0f, 0.0f}))));
    struct Edge {
        Coefficients set;
        float out[4];
        float last;
        std::size_t nonFinite;
    };
    const Edge edges[] = {
        {{0.5f, 0.0f, 0.0f, 1.5f, 0.0f}, {0.25f, 0.375f, 0.5625f, 0.84375f}, nan, 2324},
        {{0.5f, 0.0f, 0.0f, 1.0f, 0.0f}, {0.25f, 0.25f, 0.25f, 0.25f}, 0.25f, 0},
        {{0.5f, 0.0f, 0.0f, 0.0f, -1.0f}, {0.25f, 0.0f, -0.25f, 0.0f}, -0.25f, 0},
        {{1e30f, 1e30f, 0.0f, 0.0f, 0.0f}, {5e29f, 5e29f, 0.0f, 0.0f}, 0.0f, 0},
        {{0.5f, 0.0f, 0.0f, 2.0f, -1.0f}, {0.25f, 0.5f, 0.75f, 1.0f}, 636.75f, 0},
    };
    for (const auto& edge : edges) {
        const auto out = OracleOutput(Direct(edge.set));
        for (std::size_t n = 0; n < 4; n++) Require(out[ORACLE_IMPULSE_AT + n] == edge.out[n]);
        Require(std::isnan(edge.last) ? std::isnan(out.back()) : out.back() == edge.last);
        Require(static_cast<std::size_t>(std::count_if(out.begin(), out.end(), [](float sample) { return !std::isfinite(sample); })) == edge.nonFinite);
        Require(SameBits(out, BlockModel(OracleInput(), edge.set)));
    }
}

static void TestDeadCellsOnePoleOnCustomRack() {
    const auto deadCells = Filter(0, 1, 9, 0, 10000.0f, Q, 1.0f);
    const auto out = OracleOutput(deadCells, true);
    Require(Hash(out) == 0xaf5ca2f664c91ec7ull);
    const float b0 = Bits(0x3ede4c06);
    const float pole = Bits(0x3e06cfec);
    Require(b0 == 0.434173763f && pole == 0.131652534f);
    Require(out[ORACLE_IMPULSE_AT] == 0.5f * b0 && out[ORACLE_IMPULSE_AT + 1] == 0.5f * (b0 + pole * b0));
    for (std::uint32_t type = 1; type <= 11; type++) {
        const float level = type >= 6 && type <= 8 ? 2.0f : 1.0f;
        const auto param = type == 11 ? Direct({0.3f, -0.41f, 0.17f, 1.2f, -0.5f}) : Filter(0, 1, type, 0, 10000.0f, Q, level);
        Require(SameBits(OracleOutput(param, true), OracleOutput(param)));
    }
}

struct ClampCase {
    std::uint32_t type;
    std::uint32_t lowLast;
    std::uint32_t lowFirst;
    std::uint32_t qHighLast;
    std::uint32_t qHighFirst;
};

static void TestEveryTypeClamps() {
    const ClampCase cases[] = {
        {1, 0x40999bfe, 0x40999bff, 0x41ffff59, 0x41ffff58}, {2, 0x40999bfe, 0x40999bff, 0x41ffff59, 0x41ffff58}, {3, 0x40999999, 0x4099999a, 0x41ffff59, 0x41ffff58},
        {4, 0x40999999, 0x4099999a, 0x42000000, 0x41ffffff}, {5, 0x40999bfe, 0x40999bff, 0x41ffff59, 0x41ffff58}, {6, 0x40999d9f, 0x40999da0, 0x41ffff83, 0x41ffff82},
        {7, 0x40999d9f, 0x40999da0, 0x41ffff83, 0x41ffff82}, {8, 0x40999b5c, 0x40999b5d, 0x41ffff8d, 0x41ffff8c}, {9, 0x40999999, 0x4099999a, 0, 0},
        {10, 0x40999bb5, 0x40999bb6, 0, 0},
    };
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (const auto& c : cases) {
        const float level = c.type >= 6 && c.type <= 8 ? 2.0f : 1.0f;
        const auto out = [&c, level](float frequency, float q) { return OracleOutput(Filter(0, 1, c.type, 0, frequency, q, level), false, SCE_NGS2_OK, CLAMP_FRAMES); };
        const auto low = out(0.0f, Q);
        for (float frequency : {-1.0f, 1.0f, Bits(c.lowLast)}) Require(SameBits(out(frequency, Q), low));
        Require(!SameBits(out(Bits(c.lowFirst), Q), low));
        const auto high = out(30000.0f, Q);
        for (float frequency : {24000.0f, 23952.0f, INFINITY, 1e6f}) Require(SameBits(out(frequency, Q), high));
        Require(!SameBits(out(Bits(0x46bb1fff), Q), high));
        Require(AllNan(out(nan, Q)));
        const auto qLow = out(1000.0f, 0.0f);
        const auto qHigh = out(1000.0f, 1e6f);
        if (c.type >= 9) {
            Require(SameBits(qLow, qHigh) && SameBits(out(1000.0f, 5.0f), qLow));
            continue;
        }
        for (float q : {-1.0f, Bits(0x38d1b717)}) Require(SameBits(out(1000.0f, q), qLow));
        Require(!SameBits(out(1000.0f, Bits(0x38d1b718)), qLow));
        for (float q : {INFINITY, Bits(c.qHighLast)}) Require(SameBits(out(1000.0f, q), qHigh));
        Require(!SameBits(out(1000.0f, Bits(c.qHighFirst)), qHigh));
        Require(AllNan(out(1000.0f, nan)));
    }
    for (std::uint32_t type : {6u, 7u, 8u}) {
        const auto out = [type](float level) { return OracleOutput(Filter(0, 1, type, 0, 1000.0f, Q, level), false, SCE_NGS2_OK, CLAMP_FRAMES); };
        const auto clamped = out(1e-4f);
        for (float level : {0.0f, -0.0f, -1.0f, 1e-6f, nan, Bits(0x38d1b718)}) Require(SameBits(out(level), clamped));
        Require(!SameBits(out(Bits(0x38d1b719)), clamped));
        Require(AllNan(OracleOutput(Filter(0, 1, type, 0, 1000.0f, Q, INFINITY))));
    }
}

static void TestInvalidTypesLeaveOutputUnfiltered() {
    std::vector<float> unfiltered(ORACLE_FRAMES, 0.0f);
    unfiltered[ORACLE_IMPULSE_AT] = 0.5f;
    for (std::uint32_t type : {12u, 13u, 100u, 0x7fffffffu, 0xffffffffu}) {
        for (bool custom : {false, true}) Require(SameBits(OracleOutput(Filter(0, 1, type, 0, 1000.0f, Q, 1.0f), custom, SCE_NGS2_ERROR_INVALID_FILTER_TYPE), unfiltered));
    }
}

static void TestEveryTypeRestartsWithZeroHistory() {
    const auto none = [](uintptr_t, uintptr_t) {};
    for (std::uint32_t type = 1; type <= 11; type++) {
        const auto filter = type == 11 ? Direct({0.0123f, 0.0246f, 0.0123f, 1.9f, -0.905f}) : Filter(0, 1, type, 0, 1000.0f, Q, 2.0f);
        const auto fresh = FreshOutput(&filter);
        Require(!SameBits(fresh, FreshOutput(nullptr)));
        Require(SameBits(RestartOutput(Grain, 2, none, filter), fresh));
    }
}

static void TestUnestablishedBlockLayoutsThrow() {
    std::vector<std::int16_t> stereo(4 * Grain, 0);
    stereo[0] = 16384;
    stereo[1] = 16384;
    const auto render = [&stereo](const Ngs2SamplerVoiceFilterParam& filter, std::uint32_t grain) {
        const auto system = CreateSystem();
        Require(sceNgs2SystemSetGrainSamples(system, grain) == SCE_NGS2_OK);
        const auto master = Mastering(system, 2);
        const auto sampler = Sampler(system, stereo, 2, 48000);
        Patch(sampler, master);
        SetFilter(sampler, filter);
        Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
        std::vector<float> out(2 * grain);
        const Ngs2RenderBufferInfo info{out.data(), out.size() * sizeof(float), SCE_NGS2_WAVEFORM_TYPE_PCM_F32L, 2};
        const bool threw = ThrowsRuntimeError([&] { sceNgs2SystemRender(system, &info, 1); });
        Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
        return threw;
    };
    for (std::uint32_t type : {2u, 9u, 11u}) {
        Require(render(Filter(0, 1, type, 1, 1000.0f, Q, 1.0f), Grain));
        Require(render(Filter(0, 1, type, 2, 1000.0f, Q, 1.0f), Grain));
        for (std::uint64_t mask : {0ull, 3ull, 0xffull}) Require(!render(Filter(0, 1, type, mask, 1000.0f, Q, 1.0f), Grain));
    }
    Require(!render(Filter(0, 1, 1, 1, 1000.0f, Q, 1.0f), Grain));
    Require(render(Filter(0, 1, 1, 0, 1000.0f, Q, 1.0f), 6));
    Require(!render(Filter(0, 1, 0, 0, 1000.0f, Q, 1.0f), 6));
    Require(!render(Filter(0, 1, 1, 0, 1000.0f, Q, 1.0f), 12));
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
    TestShortHeaderIsNotReadPastItsSize();
    TestFilterRunsAfterResamplingAtSystemRate();
    TestRejectedParamKeepsFilter();
    TestLocationsFilterTheirPorts();
    TestRestartClearsFilterHistory();
    TestStopAndPlayWithoutNewBlocksClearsHistory();
    TestResentFilterKeepsHistory();

    TestEveryTypeMatchesPs5Output();
    TestEveryTypeMatchesBlockModelOnNoise();
    TestDirectCoefficientsRunInBlocksOfFour();
    TestDeadCellsOnePoleOnCustomRack();
    TestEveryTypeClamps();
    TestInvalidTypesLeaveOutputUnfiltered();
    TestEveryTypeRestartsWithZeroHistory();
    TestUnestablishedBlockLayoutsThrow();
    return 0;
}
