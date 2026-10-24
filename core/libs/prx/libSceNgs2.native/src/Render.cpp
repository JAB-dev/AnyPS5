#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

#include "prx/libc/include/General.hpp"
#include "Ngs2Internal.hpp"

static constexpr std::uint64_t POSITION_FRACTION = (std::uint64_t{1} << NGS2_POSITION_BITS) - 1;
static constexpr std::uint32_t MAX_STEP = 65535;
static constexpr std::uint32_t PHASE_BITS = 9;
static constexpr std::uint32_t KERNEL_PHASES = 1u << PHASE_BITS;
static constexpr std::uint32_t FADE_FRAMES = 64;
static constexpr std::uint32_t PARK_FRAMES = 4;

static float ReadPcm(const std::uint8_t* frame, std::uint32_t channel) {
    if (frame == nullptr) return 0.0f;
    std::int16_t sample;
    std::memcpy(&sample, frame + channel * sizeof(sample), sizeof(sample));
    return sample / 32768.0f;
}

static const float* Kernel() {
    static const auto kernel = [] {
        const auto window = [](double t) {
            if (std::abs(t) >= 2.0) return 0.0;
            const double sinc = t == 0.0 ? 1.0 : std::sin(std::numbers::pi * t) / (std::numbers::pi * t);
            return sinc * (0.5 + 0.5 * std::cos(std::numbers::pi * t / 2.0));
        };
        std::array<float, 2 * KERNEL_PHASES + 1> table{};
        for (std::uint32_t phase = 0; phase < KERNEL_PHASES; phase++) {
            const double f = static_cast<double>(phase) / KERNEL_PHASES;
            const double taps[4] = {1.0 + f, f, 1.0 - f, 2.0 - f};
            const std::uint32_t entries[4] = {KERNEL_PHASES + phase, phase, KERNEL_PHASES - phase, 2 * KERNEL_PHASES - phase};
            const double sum = window(taps[0]) + window(taps[1]) + window(taps[2]) + window(taps[3]);
            for (int tap = 0; tap < 4; tap++) table[entries[tap]] = static_cast<float>(std::round(window(taps[tap]) / sum * 1e10) / 1e10);
        }
        return table;
    }();
    return kernel.data();
}

static float Resample(const float* history, const float* kernel, std::uint32_t phase) {
    const float a = history[0] * kernel[KERNEL_PHASES + phase];
    const float b = history[1] * kernel[phase];
    const float c = history[2] * kernel[KERNEL_PHASES - phase];
    const float d = history[3] * kernel[2 * KERNEL_PHASES - phase];
    return (a + b) + (c + d);
}

static std::uint32_t SamplerStep(const Ngs2Voice& voice, std::uint32_t systemRate) {
    const float position = voice.pitch * (static_cast<float>(voice.sampleRate) / static_cast<float>(systemRate)) * static_cast<float>(1u << NGS2_POSITION_BITS);
    return static_cast<std::uint32_t>(std::fmin(std::fmax(position, 0.0f), static_cast<float>(MAX_STEP)));
}

bool Ngs2FinishBlock(Ngs2Voice& voice) {
    auto& block = voice.blocks.front();
    const bool repeat = block.info.num_repeats != 0;
    if (repeat) {
        if (block.info.num_repeats != UINT32_MAX) block.info.num_repeats--;
        block.numRepeated++;
        block.started = false;
        block.dataCursor = 0;
        Ngs2RestartAtrac9(voice);
    }
    const Ngs2VoiceCallbackInfo info{
        voice.callbackData,
        reinterpret_cast<Ngs2Handle>(&voice),
        repeat ? SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_REPEAT : SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_END,
        0,
        block.info.user_data,
        block.data,
        static_cast<std::uint32_t>(block.info.data_size),
        block.numRepeated,
        0,
        0,
    };
    if (!repeat) {
        voice.waveformEnd = block.data + block.dataCursor;
        voice.blocks.pop_front();
    }
    const auto revision = voice.waveformRevision;
    if (voice.callback != nullptr && (voice.callbackFlags & info.flag) != 0) {
        const Ngs2Unlocked unlocked;
        voice.callback(&info);
    }
    return voice.waveformRevision == revision && voice.state == Ngs2PlayState::Playing;
}

static void ApplyPause(Ngs2Voice& voice) {
    if (voice.pausePending) {
        voice.pause = voice.renderedSincePlay ? Ngs2Pause::FadeOut : Ngs2Pause::Silent;
        voice.pausePending = false;
    }
    voice.ran = voice.pause;
    if (voice.pause == Ngs2Pause::FadeOut) voice.pause = Ngs2Pause::Silent;
    else if (voice.pause == Ngs2Pause::Silent) voice.pause = Ngs2Pause::Frozen;
    else if (voice.pause == Ngs2Pause::FadeIn) voice.pause = Ngs2Pause::None;
}

static bool SignalPasses(Ngs2Voice& voice) {
    auto limit = std::min(voice.position >> NGS2_POSITION_BITS, voice.total);
    if (voice.acceptsBlocks && !voice.runs.empty() && limit == voice.total && limit != 0) {
        const auto& last = voice.runs.back().block;
        if (last.Pass() < static_cast<std::uint64_t>(last.info.num_skip_samples) + last.info.num_samples) limit--;
    }
    if (limit <= voice.signalledTo) return true;
    std::vector<Ngs2VoiceCallbackInfo> infos;
    for (const auto& run : voice.runs) {
        if (run.length == 0) continue;
        const auto first = voice.signalledTo < run.start ? 0 : (voice.signalledTo - run.start) / run.length;
        for (auto pass = first; run.passes == 0 || pass < run.passes; pass++) {
            const auto end = run.start + (pass + 1) * run.length;
            if (end > limit) break;
            if (end <= voice.signalledTo) continue;
            const bool repeat = run.passes == 0 || pass + 1 < run.passes;
            infos.push_back({
                voice.callbackData,
                reinterpret_cast<Ngs2Handle>(&voice),
                repeat ? SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_REPEAT : SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_END,
                0,
                run.block.info.user_data,
                run.block.pieces.empty() ? nullptr : run.block.pieces.front().data,
                static_cast<std::uint32_t>(run.block.info.data_size),
                static_cast<std::uint32_t>(repeat ? pass + 1 : pass),
                0,
                0,
            });
        }
    }
    voice.signalledTo = limit;
    const auto revision = voice.waveformRevision;
    for (const auto& info : infos) {
        if (voice.callback == nullptr) return true;
        if ((voice.callbackFlags & info.flag) == 0) continue;
        voice.decodedSamples = Ngs2PcmDecoded(voice);
        voice.decodedBytes = Ngs2PcmDecodedBytes(voice);
        voice.callbackThread = std::this_thread::get_id();
        try {
            const Ngs2Unlocked unlocked;
            voice.callback(&info);
        } catch (...) {
            voice.callbackThread = {};
            throw;
        }
        voice.callbackThread = {};
        if (voice.waveformRevision != revision || voice.state != Ngs2PlayState::Playing) return false;
    }
    return true;
}

static bool ConsumeSamples(Ngs2Voice& voice, std::uint32_t grain, std::uint32_t systemRate) {
    if (voice.clearPending) Ngs2ApplyPendingClear(voice);
    ApplyPause(voice);
    const bool fadeOut = voice.ran == Ngs2Pause::FadeOut;
    const bool fadeIn = voice.ran == Ngs2Pause::FadeIn;
    if ((fadeOut || fadeIn) && grain < FADE_FRAMES) throw std::runtime_error("NGS2: a PAUSE or RESUME fade over a " + std::to_string(grain) + "-frame grain is not implemented");
    const bool audible = voice.ran == Ngs2Pause::None || fadeOut || fadeIn;
    const auto step = voice.ran == Ngs2Pause::Frozen ? 0 : SamplerStep(voice, systemRate);
    const float* kernel = Kernel();
    bool running = true;
    for (std::uint32_t frame = 0; frame < grain && audible && running; frame++) {
        const auto index = static_cast<std::int64_t>(voice.position >> NGS2_POSITION_BITS);
        const auto phase = static_cast<std::uint32_t>((voice.position >> (NGS2_POSITION_BITS - PHASE_BITS)) & (KERNEL_PHASES - 1));
        const std::uint8_t* taps[4];
        for (int tap = 0; tap < 4; tap++) taps[tap] = Ngs2PcmFrame(voice, index - 4 + tap);
        for (std::uint32_t channel = 0; channel < voice.channels; channel++) {
            const float history[4] = {ReadPcm(taps[0], channel), ReadPcm(taps[1], channel), ReadPcm(taps[2], channel), ReadPcm(taps[3], channel)};
            float sample = Resample(history, kernel, phase);
            if (fadeOut && frame >= grain - FADE_FRAMES) sample = sample * (static_cast<float>(grain - frame) / FADE_FRAMES);
            if (fadeIn && frame < FADE_FRAMES) sample = sample * (static_cast<float>(frame) / FADE_FRAMES);
            voice.samples[channel * grain + frame] = sample;
        }
        voice.position += step;
        running = SignalPasses(voice);
    }
    if (!audible) {
        voice.position += static_cast<std::uint64_t>(step) * grain;
        running = SignalPasses(voice);
    }
    voice.hasSamples = audible;
    voice.renderedSincePlay = true;
    voice.starving = false;
    if (!running) return false;
    if ((voice.position >> NGS2_POSITION_BITS) <= voice.total) return false;
    if (!voice.acceptsBlocks) return true;
    voice.starving = true;
    voice.position = ((voice.total + PARK_FRAMES) << NGS2_POSITION_BITS) | (voice.position & POSITION_FRACTION);
    return false;
}

static bool Bypasses(const Ngs2Filter& filter, std::uint32_t channel) {
    return channel < 64 && (filter.bypassMask & (std::uint64_t{1} << channel)) != 0;
}

static bool HasHistory(const Ngs2Filter& filter) {
    if (!filter.enabled) return false;
    for (std::uint32_t channel = 0; channel < filter.history.size(); channel++) {
        const auto& h = filter.history[channel];
        if (!Bypasses(filter, channel) && (h.x1 != 0.0 || h.x2 != 0.0 || h.y1 != 0.0 || h.y2 != 0.0)) return true;
    }
    return false;
}

static void ApplyFilter(Ngs2Voice& voice, Ngs2Filter& filter, std::uint32_t grain) {
    for (std::uint32_t channel = 0; channel < voice.channels; channel++) {
        if (Bypasses(filter, channel)) continue;
        auto& h = filter.history[channel];
        for (std::uint32_t i = 0; i < grain; i++) {
            auto& sample = voice.samples[channel * grain + i];
            const double x = sample;
            double y = filter.b0 * x + filter.b1 * h.x1 + filter.b2 * h.x2 - filter.a1 * h.y1 - filter.a2 * h.y2;
            if (std::abs(y) < 1e-20) y = 0.0;
            h = {x, h.x1, y, h.y1};
            sample = static_cast<float>(y);
        }
    }
}

static void RenderSampler(Ngs2Voice& voice, std::uint32_t grain, std::uint32_t systemRate) {
    const auto channels = voice.channels;
    const bool atrac9 = voice.waveformType == SCE_NGS2_WAVEFORM_TYPE_ATRAC9;
    bool ended = false;
    if (atrac9) Ngs2ConsumeAtrac9(voice, grain, systemRate);
    else ended = ConsumeSamples(voice, grain, systemRate);
    if (voice.channels == channels) {
        for (auto& filter : voice.filters) {
            if (!filter.enabled || (!voice.hasSamples && !HasHistory(filter))) continue;
            ApplyFilter(voice, filter, grain);
            voice.hasSamples = true;
        }
        if (atrac9) ended = voice.blocks.empty() && voice.atrac9.windowCursor * voice.channels >= voice.atrac9.window.size() && !voice.acceptsBlocks;
        if (voice.state == Ngs2PlayState::Playing && ended && std::none_of(voice.filters.begin(), voice.filters.end(), HasHistory)) {
            voice.state = Ngs2PlayState::Empty;
            voice.inUse = false;
        }
        if (!atrac9) {
            const bool playing = voice.state == Ngs2PlayState::Playing;
            voice.decodedSamples = playing || voice.rack->rackId != SCE_NGS2_RACK_ID_SAMPLER ? Ngs2PcmDecoded(voice) : 0;
            voice.decodedBytes = playing ? Ngs2PcmDecodedBytes(voice) : 0;
        }
    }
    if (voice.channels != channels) {
        voice.samples.assign(static_cast<std::size_t>(grain) * voice.channels, 0.0f);
        voice.hasSamples = false;
    }
}

static void RenderVoice(Ngs2Voice& voice, const std::vector<Ngs2Voice*>& voices, std::uint32_t grain, std::uint32_t systemRate);

struct Ngs2DefaultRow {
    float mono;
    float stereo[2];
    float surround51[6];
    float surround71[8];
};

static constexpr float HALF_POWER = std::numbers::sqrt2_v<float> / 2.0f;
static constexpr float QUARTER_POWER = std::numbers::sqrt2_v<float> / 4.0f;

static constexpr Ngs2DefaultRow MONO_CENTER{1.0f, {HALF_POWER, HALF_POWER}, {0, 0, 1, 0, 0, 0}, {0, 0, 1, 0, 0, 0, 0, 0}};
static constexpr Ngs2DefaultRow FRONT_LEFT{HALF_POWER, {1, 0}, {1, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0, 0, 0}};
static constexpr Ngs2DefaultRow FRONT_RIGHT{HALF_POWER, {0, 1}, {0, 1, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0, 0, 0}};
static constexpr Ngs2DefaultRow LFE{0.0f, {0, 0}, {0, 0, 0, 1, 0, 0}, {0, 0, 0, 1, 0, 0, 0, 0}};
static constexpr Ngs2DefaultRow QUAD_LEFT{0.5f, {HALF_POWER, 0}, {0, 0, 0, 0, 1, 0}, {0, 0, 0, 0, 1, 0, 0, 0}};
static constexpr Ngs2DefaultRow QUAD_RIGHT{0.5f, {0, HALF_POWER}, {0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 1, 0, 0}};
static constexpr Ngs2DefaultRow FIVE_SURROUND_LEFT{0.5f, {0, 0}, {0, 0, 0, 0, 1, 0}, {0, 0, 0, 0, 1, 0, 0, 0}};
static constexpr Ngs2DefaultRow FIVE_SURROUND_RIGHT{0.5f, {0, 0}, {0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 1, 0, 0}};
static constexpr Ngs2DefaultRow SURROUND_LEFT{0.5f, {0.5f, 0}, {0, 0, 0, 0, 1, 0}, {0, 0, 0, 0, 1, 0, 0, 0}};
static constexpr Ngs2DefaultRow SURROUND_RIGHT{0.5f, {0, 0.5f}, {0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 1, 0, 0}};
static constexpr Ngs2DefaultRow BACK_CENTER{QUARTER_POWER, {QUARTER_POWER, QUARTER_POWER}, {0, 0, 0, 0, HALF_POWER, HALF_POWER}, {0, 0, 0, 0, 0, 0, HALF_POWER, HALF_POWER}};
static constexpr Ngs2DefaultRow BACK_LEFT{0.5f, {0.5f, 0}, {0, 0, 0, 0, 1, 0}, {0, 0, 0, 0, 0, 0, 1, 0}};
static constexpr Ngs2DefaultRow BACK_RIGHT{0.5f, {0, 0.5f}, {0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 0, 1}};

static constexpr Ngs2DefaultRow DEFAULT_MAP[NGS2_MAX_CHANNELS][NGS2_MAX_CHANNELS] = {
    {MONO_CENTER},
    {FRONT_LEFT, FRONT_RIGHT},
    {FRONT_LEFT, FRONT_RIGHT, LFE},
    {FRONT_LEFT, FRONT_RIGHT, QUAD_LEFT, QUAD_RIGHT},
    {FRONT_LEFT, FRONT_RIGHT, MONO_CENTER, FIVE_SURROUND_LEFT, FIVE_SURROUND_RIGHT},
    {FRONT_LEFT, FRONT_RIGHT, MONO_CENTER, LFE, SURROUND_LEFT, SURROUND_RIGHT},
    {FRONT_LEFT, FRONT_RIGHT, MONO_CENTER, LFE, SURROUND_LEFT, SURROUND_RIGHT, BACK_CENTER},
    {FRONT_LEFT, FRONT_RIGHT, MONO_CENTER, LFE, SURROUND_LEFT, SURROUND_RIGHT, BACK_LEFT, BACK_RIGHT},
};

float Ngs2DefaultLevel(std::uint32_t sourceChannels, std::uint32_t source, std::uint32_t destChannels, std::uint32_t dest) {
    const auto& row = DEFAULT_MAP[sourceChannels - 1][source];
    switch (destChannels) {
        case 1: return row.mono;
        case 2: return row.stereo[dest];
        case 6: return row.surround51[dest];
        case 8: return row.surround71[dest];
        default: throw std::runtime_error("NGS2: the default channel map into " + std::to_string(destChannels) + " channels is not implemented");
    }
}

static void MixPort(Ngs2Voice& voice, const Ngs2Voice& source, const Ngs2Port& port, std::uint32_t grain) {
    const auto* matrix = port.matrix < 0 || source.matrices[port.matrix].empty() ? nullptr : &source.matrices[port.matrix];
    const auto channels = voice.InputChannels();
    const std::size_t outputs = matrix == nullptr ? channels : std::min<std::size_t>(channels, matrix->size() / source.channels);
    for (std::size_t dst = 0; dst < outputs; dst++) {
        for (std::uint32_t src = 0; src < source.channels; src++) {
            const float level = port.volume * (matrix == nullptr ? Ngs2DefaultLevel(source.channels, src, channels, static_cast<std::uint32_t>(dst)) : (*matrix)[dst * source.channels + src]);
            if (level == 0.0f) continue;
            for (std::uint32_t i = 0; i < grain; i++) voice.samples[dst * grain + i] += source.samples[src * grain + i] * level;
        }
    }
}

static void MixInputs(Ngs2Voice& voice, const std::vector<Ngs2Voice*>& voices, std::uint32_t grain, std::uint32_t systemRate) {
    for (auto* source : voices) {
        for (const auto& port : source->ports) {
            if (port.dest != &voice || port.volume == 0.0f) continue;
            RenderVoice(*source, voices, grain, systemRate);
            if (!source->hasSamples) continue;
            MixPort(voice, *source, port, grain);
            voice.hasSamples = true;
        }
    }
}

static void RenderVoice(Ngs2Voice& voice, const std::vector<Ngs2Voice*>& voices, std::uint32_t grain, std::uint32_t systemRate) {
    if (voice.rendered) return;
    if (voice.rendering) throw std::invalid_argument("NGS2: the voice patches form a cycle");
    voice.rendering = true;
    struct RenderingScope {
        bool& active;
        ~RenderingScope() { active = false; }
    } renderingScope{voice.rendering};
    voice.samples.assign(static_cast<std::size_t>(grain) * voice.InputChannels(), 0.0f);
    voice.hasSamples = false;
    if (voice.state == Ngs2PlayState::Playing && voice.channels != 0) {
        if (voice.rack->rackId == SCE_NGS2_RACK_ID_SAMPLER || voice.rack->rackId == SCE_NGS2_RACK_ID_CUSTOM_SAMPLER) {
            RenderSampler(voice, grain, systemRate);
        } else {
            MixInputs(voice, voices, grain, systemRate);
        }
        if (voice.rack->rackId == SCE_NGS2_RACK_ID_REVERB && voice.reverb) voice.hasSamples = Ngs2ProcessReverb(voice, grain, systemRate);
        Ngs2ProcessLegacyUserFx(voice, grain, systemRate);
        Ngs2ProcessUserFx(voice, grain, systemRate);
    }
    if (voice.samples.size() != static_cast<std::size_t>(grain) * voice.channels) voice.samples.assign(static_cast<std::size_t>(grain) * voice.channels, 0.0f);
    voice.rendered = true;
}

static std::size_t SampleBytes(std::uint32_t waveformType) {
    if (waveformType == SCE_NGS2_WAVEFORM_TYPE_PCM_I16L) return sizeof(std::int16_t);
    if (waveformType == SCE_NGS2_WAVEFORM_TYPE_PCM_F32L) return sizeof(float);
    throw std::runtime_error("NGS2: render waveform type " + Ngs2Hex(waveformType) + " is not implemented");
}

static bool IsStereoIntoSurround(const Ngs2Voice& voice, const Ngs2RenderBufferInfo& output) {
    return voice.channels == 2 && (output.num_channels == 6 || output.num_channels == 8);
}

static std::vector<float>& OutputMix(std::vector<std::vector<float>>& mixes, const Ngs2RenderBufferInfo* bufferInfo, std::uint32_t numBufferInfo,
                                     const Ngs2Voice& voice, std::uint32_t grain) {
    if (voice.outputId >= numBufferInfo) throw std::invalid_argument("NGS2: mastering output " + std::to_string(voice.outputId) + " has no render buffer");
    const auto& output = bufferInfo[voice.outputId];
    const auto sampleBytes = SampleBytes(output.waveform_type);
    if (output.num_channels != voice.channels && !IsStereoIntoSurround(voice, output)) throw std::runtime_error("NGS2: mixing " + std::to_string(voice.channels) + " mastering channels into " + std::to_string(output.num_channels) + " is not implemented");
    if (output.buffer == nullptr || output.buffer_size < static_cast<std::size_t>(grain) * output.num_channels * sampleBytes) {
        throw std::invalid_argument("NGS2: render buffer " + std::to_string(voice.outputId) + " is missing or too small");
    }
    auto& mix = mixes[voice.outputId];
    if (mix.empty()) mix.resize(static_cast<std::size_t>(grain) * output.num_channels);
    return mix;
}

static void WriteOutput(const Ngs2RenderBufferInfo& output, const std::vector<float>& mix) {
    if (output.waveform_type == SCE_NGS2_WAVEFORM_TYPE_PCM_F32L) {
        std::memcpy(output.buffer, mix.data(), mix.size() * sizeof(float));
        return;
    }
    auto* pcm = static_cast<std::int16_t*>(output.buffer);
    for (std::size_t i = 0; i < mix.size(); i++) {
        pcm[i] = static_cast<std::int16_t>(std::min(std::lrint(std::clamp(mix[i], -1.0f, 1.0f) * 32768.0f), 32767L));
    }
}

static std::vector<std::size_t> OutputBytes(const Ngs2System& system, const Ngs2RenderBufferInfo* bufferInfo, std::uint32_t numBufferInfo) {
    const auto grain = system.option.num_grain_samples;
    std::vector<std::size_t> outputBytes(numBufferInfo);
    for (std::uint32_t i = 0; i < numBufferInfo; i++) {
        const auto& output = bufferInfo[i];
        if (output.buffer == nullptr) continue;
        if (output.num_channels == 0 || output.num_channels > NGS2_MAX_CHANNELS) {
            throw std::invalid_argument("NGS2: render channel count is invalid");
        }
        outputBytes[i] = static_cast<std::size_t>(grain) * output.num_channels * SampleBytes(output.waveform_type);
        if (output.buffer_size < outputBytes[i]) throw std::invalid_argument("NGS2: render buffer " + std::to_string(i) + " is too small");
    }
    return outputBytes;
}

void Ngs2RenderSilence(const Ngs2System& system, const Ngs2RenderBufferInfo* bufferInfo, std::uint32_t numBufferInfo) {
    const auto outputBytes = OutputBytes(system, bufferInfo, numBufferInfo);
    for (std::uint32_t i = 0; i < numBufferInfo; i++) {
        if (outputBytes[i] != 0) std::memset(bufferInfo[i].buffer, 0, outputBytes[i]);
    }
}

void Ngs2RenderSystem(Ngs2System& system, const Ngs2RenderBufferInfo* bufferInfo, std::uint32_t numBufferInfo) {
    const auto grain = system.option.num_grain_samples;
    const auto outputBytes = OutputBytes(system, bufferInfo, numBufferInfo);
    std::vector<Ngs2Voice*> voices;
    for (auto* rack : system.racks) {
        for (auto& voice : rack->voices) {
            voice.rendered = false;
            voices.push_back(&voice);
        }
    }
    std::vector<std::vector<float>> mixes(numBufferInfo);
    for (auto* voice : voices) {
        RenderVoice(*voice, voices, grain, system.option.sample_rate);
        if ((voice->rack->rackId != SCE_NGS2_RACK_ID_MASTERING && voice->rack->rackId != SCE_NGS2_RACK_ID_CUSTOM_MASTERING) || !voice->hasSamples) continue;
        auto& mix = OutputMix(mixes, bufferInfo, numBufferInfo, *voice, grain);
        const std::uint32_t outputChannels = bufferInfo[voice->outputId].num_channels;
        for (std::uint32_t channel = 0; channel < voice->channels; channel++) {
            const bool lfe = channel == 3 && (voice->channels == 6 || voice->channels == 8);
            const float level = lfe ? voice->lfeLevel : voice->fbwLevel;
            for (std::uint32_t i = 0; i < grain; i++) mix[i * outputChannels + channel] += voice->samples[channel * grain + i] * level;
        }
    }
    for (std::uint32_t i = 0; i < numBufferInfo; i++) {
        if (!mixes[i].empty()) WriteOutput(bufferInfo[i], mixes[i]);
        else if (outputBytes[i] != 0) std::memset(bufferInfo[i].buffer, 0, outputBytes[i]);
    }
    for (auto* voice : voices) {
        if (voice->state == Ngs2PlayState::Stopped) voice->state = Ngs2PlayState::Empty;
        voice->stateFlags = voice->Pcm() ? Ngs2PcmStateFlags(*voice) : static_cast<std::uint32_t>(voice->state);
        if (voice->state == Ngs2PlayState::Playing && voice->rack->rackId == SCE_NGS2_RACK_ID_SAMPLER && !voice->Pcm() && voice->blocks.empty() && voice->acceptsBlocks) voice->stateFlags |= SCE_NGS2_VOICE_STATE_FLAG_EMPTY;
    }
    system.renderCount++;
}
