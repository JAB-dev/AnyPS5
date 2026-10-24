#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>

#include "prx/libc/include/General.hpp"
#include "Ngs2Internal.hpp"

static constexpr std::uint32_t COMMAND_EVENT = 2;
static constexpr std::uint32_t COMMAND_MATRIX_LEVELS = 5;
static constexpr std::uint32_t COMMAND_PORT_VOLUME = 6;
static constexpr std::uint32_t COMMAND_PORT_MATRIX = 7;
static constexpr std::uint8_t COMMAND_TYPE_FLOAT = 1;
static constexpr std::uint8_t COMMAND_TYPE_INT = 3;
static constexpr std::uint8_t COMMAND_TYPE_UINT = 4;
static constexpr std::uint8_t COMMAND_TYPE_FLOAT_ARRAY = 0x11;
static constexpr float MAX_MATRIX_LEVEL = 4.0f;

struct Ngs2ReturnCode {
    int code;
};

void Ngs2Voice::SetEvent(std::uint32_t eventId) {
    switch (eventId) {
        case SCE_NGS2_VOICE_EVENT_PLAY:
            if ((state == Ngs2PlayState::Playing || state == Ngs2PlayState::Paused) && std::any_of(filters.begin(), filters.end(), [](const Ngs2Filter& filter) { return filter.enabled; })) {
                throw std::runtime_error("NGS2: PLAY on a playing or paused voice with a filter is not implemented");
            }
            if (state == Ngs2PlayState::Empty || state == Ngs2PlayState::Stopped) {
                for (auto& filter : filters) std::fill(filter.history.begin(), filter.history.end(), Ngs2FilterHistory{});
                state = Ngs2PlayState::Playing;
                stateFlags |= SCE_NGS2_VOICE_STATE_FLAG_INUSE;
                if (userFxHandler) userFxFlags = 1;
            }
            break;
        case SCE_NGS2_VOICE_EVENT_STOP:
            if (state == Ngs2PlayState::Playing || state == Ngs2PlayState::Paused) state = Ngs2PlayState::Stopped;
            break;
        case SCE_NGS2_VOICE_EVENT_STOP_IMM:
        case SCE_NGS2_VOICE_EVENT_KILL:
            state = Ngs2PlayState::Empty;
            Ngs2ClearReverb(*this);
            break;
        case SCE_NGS2_VOICE_EVENT_PAUSE:
            if (state == Ngs2PlayState::Playing) state = Ngs2PlayState::Paused;
            break;
        case SCE_NGS2_VOICE_EVENT_RESUME:
            if (state == Ngs2PlayState::Paused) state = Ngs2PlayState::Playing;
            break;
        default: throw std::invalid_argument("NGS2: unknown voice event " + Ngs2Hex(eventId));
    }
}

void Ngs2Voice::ResetSetup() {
    ++waveformRevision;
    SetEvent(SCE_NGS2_VOICE_EVENT_STOP_IMM);
    std::fill(ports.begin(), ports.end(), Ngs2Port{});
    for (auto& matrix : matrices) matrix.clear();
    filters.clear();
    fbwLevel = 1.0f;
    lfeLevel = 1.0f;
    channels = 0;
    sampleRate = 0;
    waveformType = 0;
    atrac9 = {};
    pitch = 1.0f;
    phase = 0;
    blocks.clear();
    acceptsBlocks = true;
    decodedSamples = 0;
    decodedBytes = 0;
    waveformEnd = nullptr;
    userFxHandler = nullptr;
    userFxData = {};
    userFxFlags = 0;
    callback = nullptr;
    callbackData = 0;
    callbackFlags = 0;
}

const std::uint8_t* Ngs2StreamEnd(const Ngs2Voice& voice, const Ngs2Block& block) {
    const auto& last = block.pieces.back();
    return last.data + last.frames * voice.channels * sizeof(std::int16_t);
}

const std::uint8_t* Ngs2Voice::WaveformData() const {
    if (blocks.empty()) return waveformEnd;
    const auto& block = blocks.front();
    if (waveformType == SCE_NGS2_WAVEFORM_TYPE_ATRAC9) return block.data + block.dataCursor;
    if (block.streaming) {
        for (const auto& piece : block.pieces) {
            if (block.cursor >= piece.firstFrame && block.cursor - piece.firstFrame < piece.frames) return piece.data + (block.cursor - piece.firstFrame) * channels * sizeof(std::int16_t);
        }
        return Ngs2StreamEnd(*this, block);
    }
    return block.data + (static_cast<std::size_t>(block.info.num_skip_samples) + block.cursor) * channels * sizeof(std::int16_t);
}

static Ngs2Port& PortAt(Ngs2Voice& voice, std::uint32_t port) {
    if (port >= voice.ports.size()) throw std::invalid_argument("NGS2: voice port " + std::to_string(port) + " is out of range");
    return voice.ports[port];
}

static void SetMatrixLevels(Ngs2Voice& voice, std::uint32_t matrixId, const float* levels, std::uint32_t numLevels) {
    if (numLevels == 0 && voice.channels == 0 && matrixId < voice.matrices.size()) {
        voice.matrices[matrixId].clear();
        return;
    }
    if (matrixId >= voice.matrices.size() || numLevels == 0 || numLevels > NGS2_MAX_CHANNELS * NGS2_MAX_CHANNELS || levels == nullptr) {
        APS5_INVALID_ARG_EX;
    }
    auto& matrix = voice.matrices[matrixId];
    matrix.resize(numLevels);
    for (std::uint32_t i = 0; i < numLevels; i++) matrix[i] = std::clamp(levels[i], -MAX_MATRIX_LEVEL, MAX_MATRIX_LEVEL);
}

static void SetPortMatrix(Ngs2Voice& voice, std::uint32_t port, std::int32_t matrixId) {
    if (matrixId < -1 || matrixId >= static_cast<std::int32_t>(voice.matrices.size())) APS5_INVALID_ARG_EX;
    PortAt(voice, port).matrix = matrixId;
}

static void Patch(Ngs2Voice& voice, const Ngs2VoicePatchParam& patch) {
    auto& port = PortAt(voice, patch.port);
    if (patch.dest_input_id != 0) throw std::runtime_error("NGS2: voice input " + std::to_string(patch.dest_input_id) + " is not implemented");
    if (patch.dest_handle == 0) {
        port.dest = nullptr;
        return;
    }
    auto* dest = Ngs2FindVoice(patch.dest_handle);
    if (dest == nullptr) {
        auto* rack = Ngs2FindRack(patch.dest_handle);
        if (rack != nullptr && !rack->voices.empty()) dest = &rack->voices.front();
    }
    if (dest == nullptr || dest->rack->system != voice.rack->system || dest->rack->rackId == SCE_NGS2_RACK_ID_SAMPLER ||
        dest->rack->rackId == SCE_NGS2_RACK_ID_CUSTOM_SAMPLER) {
        APS5_INVALID_ARG_EX;
    }
    port.dest = dest;
}

static void ApplyCommonParam(Ngs2Voice& voice, const Ngs2VoiceParamHeader& param) {
    switch (param.id) {
        case SCE_NGS2_VOICE_PARAM_MATRIX_LEVELS: {
            const auto& levels = ParamAs<Ngs2VoiceMatrixLevelsParam>(param);
            SetMatrixLevels(voice, levels.matrix_id, levels.levels, levels.num_levels);
            break;
        }
        case SCE_NGS2_VOICE_PARAM_PORT_VOLUME: {
            const auto& volume = ParamAs<Ngs2VoicePortVolumeParam>(param);
            PortAt(voice, volume.port).volume = volume.level;
            break;
        }
        case SCE_NGS2_VOICE_PARAM_PORT_MATRIX: {
            const auto& matrix = ParamAs<Ngs2VoicePortMatrixParam>(param);
            SetPortMatrix(voice, matrix.port, matrix.matrix_id);
            break;
        }
        case SCE_NGS2_VOICE_PARAM_PORT_DELAY: {
            const auto& delay = ParamAs<Ngs2VoicePortDelayParam>(param);
            PortAt(voice, delay.port);
            if (delay.num_samples != 0) throw std::runtime_error("NGS2: port delay is not implemented");
            break;
        }
        case SCE_NGS2_VOICE_PARAM_PATCH: Patch(voice, ParamAs<Ngs2VoicePatchParam>(param)); break;
        case SCE_NGS2_VOICE_PARAM_EVENT: voice.SetEvent(ParamAs<Ngs2VoiceEventParam>(param).event_id); break;
        case SCE_NGS2_VOICE_PARAM_CALLBACK: {
            const auto& callback = ParamAs<Ngs2VoiceCallbackParam>(param);
            voice.callback = callback.callback;
            voice.callbackData = callback.callback_data;
            voice.callbackFlags = callback.flags;
            break;
        }
        default: throw std::runtime_error("NGS2: voice param " + Ngs2Hex(param.id) + " is not implemented");
    }
}

static void SetupSampler(Ngs2Voice& voice, const Ngs2WaveformFormat& format) {
    voice.ResetSetup();
    if (format.waveform_type == 0 && format.num_channels == 0 && format.sample_rate == 0) return;
    if (format.waveform_type != SCE_NGS2_WAVEFORM_TYPE_PCM_I16L && format.waveform_type != SCE_NGS2_WAVEFORM_TYPE_ATRAC9) {
        throw std::runtime_error("NGS2: waveform type " + Ngs2Hex(format.waveform_type) + " is not implemented");
    }
    if (format.frame_offset != 0 || format.frame_margin != 0) throw std::runtime_error("NGS2: waveform frame offset and margin are not implemented");
    if (format.num_channels == 0 || format.num_channels > NGS2_MAX_CHANNELS || format.sample_rate == 0) APS5_INVALID_ARG_EX;
    voice.channels = format.num_channels;
    voice.sampleRate = format.sample_rate;
    voice.waveformType = format.waveform_type;
    if (format.waveform_type == SCE_NGS2_WAVEFORM_TYPE_ATRAC9) Ngs2SetupAtrac9(voice, format);
}

static void AppendStreamData(Ngs2Voice& voice, const Ngs2SamplerVoiceWaveformBlocksParam& param, std::size_t frameBytes) {
    if (voice.waveformType == SCE_NGS2_WAVEFORM_TYPE_ATRAC9) {
        const bool pendingWaveform = voice.atrac9.remainingSamples != 0 ||
            std::any_of(voice.blocks.begin(), voice.blocks.end(), [](const Ngs2Block& block) { return block.info.num_samples != 0; });
        if (!pendingWaveform) throw std::invalid_argument("NGS2: appending compressed data requires a waveform");
        for (std::uint32_t i = 0; i < param.num_blocks; ++i) {
            auto block = param.blocks[i];
            if (block.data_size == 0) continue;
            block.num_samples = 0;
            block.num_skip_samples = 0;
            block.num_repeats = 0;
            voice.blocks.push_back({static_cast<const std::uint8_t*>(param.data) + block.data_offset, block});
        }
        return;
    }
    if (voice.blocks.empty() || !voice.blocks.back().streaming) throw std::runtime_error("NGS2: appending waveform data to a voice without a streaming waveform is not implemented");
    auto& stream = voice.blocks.back();
    for (std::uint32_t i = 0; i < param.num_blocks; i++) {
        const auto& block = param.blocks[i];
        if (block.data_size == 0) continue;
        if (block.data_size % frameBytes != 0 || block.num_skip_samples != 0 || block.num_repeats != 0) {
            throw std::runtime_error("NGS2: appending a waveform block with skipped samples, repeats or a partial frame is not implemented");
        }
        stream.pieces.push_back({static_cast<const std::uint8_t*>(param.data) + block.data_offset, stream.availableFrames, block.data_size / frameBytes});
        stream.availableFrames += block.data_size / frameBytes;
    }
}

static void AddWaveformBlocks(Ngs2Voice& voice, const Ngs2SamplerVoiceWaveformBlocksParam& param) {
    constexpr std::uint32_t knownFlags = SCE_NGS2_WAVEFORM_BLOCKS_FLAG_CONTINUE | SCE_NGS2_WAVEFORM_BLOCKS_FLAG_APPEND | SCE_NGS2_WAVEFORM_BLOCKS_FLAG_RESET | SCE_NGS2_WAVEFORM_BLOCKS_FLAG_SILENCE;
    if ((param.flags & ~knownFlags) != 0) throw std::runtime_error("NGS2: waveform block flags " + Ngs2Hex(param.flags) + " are not implemented");
    if ((param.num_blocks != 0 && (voice.channels == 0 || param.blocks == nullptr || param.data == nullptr)) || (voice.channels == 0 && (param.flags & SCE_NGS2_WAVEFORM_BLOCKS_FLAG_APPEND) != 0)) APS5_INVALID_ARG_EX;
    const bool reset = (param.flags & SCE_NGS2_WAVEFORM_BLOCKS_FLAG_RESET) != 0;
    if (!voice.acceptsBlocks && !reset) throw std::invalid_argument("NGS2: the voice waveform was already closed");
    if (reset) {
        ++voice.waveformRevision;
        voice.blocks.clear();
        voice.phase = 0;
        voice.waveformEnd = nullptr;
        if (voice.waveformType == SCE_NGS2_WAVEFORM_TYPE_ATRAC9) Ngs2RestartAtrac9(voice);
    }
    const std::size_t frameBytes = voice.channels * sizeof(std::int16_t);
    if ((param.flags & SCE_NGS2_WAVEFORM_BLOCKS_FLAG_APPEND) != 0) {
        AppendStreamData(voice, param, frameBytes);
        voice.acceptsBlocks = (param.flags & SCE_NGS2_WAVEFORM_BLOCKS_FLAG_CONTINUE) != 0;
        return;
    }
    voice.acceptsBlocks = (param.flags & SCE_NGS2_WAVEFORM_BLOCKS_FLAG_CONTINUE) != 0;
    const bool allowsSilence = (param.flags & SCE_NGS2_WAVEFORM_BLOCKS_FLAG_SILENCE) != 0;
    for (std::uint32_t i = 0; i < param.num_blocks; i++) {
        const auto& block = param.blocks[i];
        if (block.num_samples == 0 && block.data_size == 0) continue;
        if (allowsSilence && block.data_size == 0) {
            if (voice.waveformType == SCE_NGS2_WAVEFORM_TYPE_ATRAC9) throw std::runtime_error("NGS2: a dataless waveform block under the silence flag is not implemented for ATRAC9 voices");
            voice.blocks.push_back({nullptr, block});
            continue;
        }
        const std::uint64_t bytes = voice.waveformType == SCE_NGS2_WAVEFORM_TYPE_ATRAC9 ? Ngs2Atrac9BlockBytes(voice, block)
                                  : (static_cast<std::uint64_t>(block.num_skip_samples) + block.num_samples) * frameBytes;
        const bool streaming = block.num_samples != 0 && bytes > block.data_size && voice.waveformType == SCE_NGS2_WAVEFORM_TYPE_PCM_I16L &&
                               block.num_skip_samples == 0 && block.num_repeats == 0 && block.data_size >= frameBytes && block.data_size % frameBytes == 0;
        const bool compressedStream = voice.waveformType == SCE_NGS2_WAVEFORM_TYPE_ATRAC9 && voice.acceptsBlocks && block.num_repeats == 0;
        if (block.num_samples == 0 || (bytes > block.data_size && !streaming && !compressedStream)) {
            throw std::invalid_argument("NGS2: waveform block " + std::to_string(i) + " does not fit its data");
        }
        voice.blocks.push_back({static_cast<const std::uint8_t*>(param.data) + block.data_offset, block});
        if (streaming) {
            auto& stream = voice.blocks.back();
            stream.streaming = true;
            stream.availableFrames = block.data_size / frameBytes;
            stream.pieces.push_back({stream.data, 0, stream.availableFrames});
        }
    }
}

static std::array<float, 5> FilterCoefficients(const Ngs2SamplerVoiceFilterParam& param, float rate) {
    if (param.type == 11) return {param.frequency, param.q, param.level, std::bit_cast<float>(param.reserved[0]), std::bit_cast<float>(param.reserved[1])};
    const float fc = std::clamp(param.frequency, 0.0001f * rate, 0.499f * rate);
    const float w = (fc / rate) * 6.28318548f;
    const auto s = static_cast<float>(std::sin(static_cast<double>(w)));
    const auto c = static_cast<float>(std::cos(static_cast<double>(w)));
    if (param.type == 9) {
        const float inv = 1.0f / (1.0f + (c + s));
        const float g = s * inv;
        return {g, g, 0.0f, (1.0f + (c - s)) * inv, 0.0f};
    }
    if (param.type == 10) {
        const float inv = 1.0f / ((1.0f + c) + s);
        const float g = (1.0f + c) * inv;
        return {g, -g, 0.0f, ((1.0f + c) - s) * inv, 0.0f};
    }
    const float q = std::clamp(param.q, 0.0001f, 32.0f);
    const float alpha = s / (2.0f * q);
    const float level = param.level > 0.0001f ? param.level : 0.0001f;
    const float A = std::sqrt(level);
    const float t = (std::sqrt(A) / q) * s;
    const float pPlus = (A + 1.0f) + (A - 1.0f) * c;
    const float pMinus = (A + 1.0f) - (A - 1.0f) * c;
    const float mPlus = (A - 1.0f) + (A + 1.0f) * c;
    const float mMinus = (A - 1.0f) - (A + 1.0f) * c;
    float b0 = 0.0f, b1 = 0.0f, b2 = 0.0f, a0 = 1.0f + alpha, a1 = -2.0f * c, a2 = 1.0f - alpha;
    switch (param.type) {
        case 1: b1 = 1.0f - c; b0 = b1 * 0.5f; b2 = b0; break;
        case 2: b1 = -(1.0f + c); b0 = (1.0f + c) * 0.5f; b2 = b0; break;
        case 3: b0 = s * 0.5f; b2 = -b0; break;
        case 4: b0 = alpha; b2 = -alpha; break;
        case 5: b0 = 1.0f; b1 = -2.0f * c; b2 = 1.0f; break;
        case 6:
            b0 = A * (pMinus + t); b1 = 2.0f * A * mMinus; b2 = A * (pMinus - t);
            a0 = pPlus + t; a1 = -2.0f * mPlus; a2 = pPlus - t;
            break;
        case 7:
            b0 = A * (pPlus + t); b1 = -2.0f * A * mPlus; b2 = A * (pPlus - t);
            a0 = pMinus + t; a1 = 2.0f * mMinus; a2 = pMinus - t;
            break;
        default:
            b0 = 1.0f + alpha * A; b1 = -2.0f * c; b2 = 1.0f - alpha * A;
            a0 = 1.0f + alpha / A; a2 = 1.0f - alpha / A;
            break;
    }
    const float inv = 1.0f / a0;
    return {b0 * inv, b1 * inv, b2 * inv, -(a1 * inv), -(a2 * inv)};
}

void Ngs2SetFilter(Ngs2Voice& voice, const Ngs2SamplerVoiceFilterParam& param) {
    if (param.header.size < sizeof(Ngs2SamplerVoiceFilterParam)) throw Ngs2ReturnCode{SCE_NGS2_ERROR_INVALID_VOICE_CONTROL_SIZE};
    const bool badIndex = param.index >= voice.rack->maxFilters;
    if (!badIndex && voice.rack->rackId == SCE_NGS2_RACK_ID_CUSTOM_SAMPLER && voice.rack->customModuleIds[param.index] != SCE_NGS2_CUSTOM_MODULE_ID_FILTER) {
        throw std::runtime_error("NGS2: a custom filter param naming a module other than a filter is not implemented");
    }
    const bool badLocation = param.location >= std::uint64_t{2} + voice.ports.size();
    const bool badType = param.type >= 12;
    if (badIndex) throw Ngs2ReturnCode{SCE_NGS2_ERROR_INVALID_VOICE_PARAM_ID};
    if (badType) throw Ngs2ReturnCode{SCE_NGS2_ERROR_INVALID_FILTER_TYPE};
    if (badLocation) throw Ngs2ReturnCode{SCE_NGS2_ERROR_INVALID_MODULE_LOCATION};
    if (voice.filters.size() <= param.index) voice.filters.resize(param.index + 1);
    auto& filter = voice.filters[param.index];
    if (param.type == 0) {
        filter = {};
        return;
    }
    const auto [i0, i1, i2, o1, o2] = FilterCoefficients(param, static_cast<float>(voice.rack->system->option.sample_rate));
    filter.enabled = true;
    filter.location = param.location;
    filter.bypassMask = param.channel_mask;
    filter.history.resize(voice.channels);
    auto& h = filter.h;
    auto& c = filter.c;
    auto& d = filter.d;
    auto& e1 = filter.e1;
    auto& e2 = filter.e2;
    h[0] = i0;
    h[1] = i1 + o1 * h[0];
    h[2] = (i2 + o2 * h[0]) + o1 * h[1];
    h[3] = o2 * h[1] + o1 * h[2];
    c[0] = o1;
    c[1] = o2 + o1 * c[0];
    d[0] = o2;
    d[1] = o1 * d[0];
    e1[0] = i1;
    e1[1] = i2 + o1 * e1[0];
    e2[0] = i2;
    e2[1] = o1 * e2[0];
    for (int j = 2; j < 4; j++) {
        c[j] = o2 * c[j - 2] + o1 * c[j - 1];
        d[j] = o2 * d[j - 2] + o1 * d[j - 1];
        e1[j] = o2 * e1[j - 2] + o1 * e1[j - 1];
        e2[j] = o2 * e2[j - 2] + o1 * e2[j - 1];
    }
}

static void ApplySamplerParam(Ngs2Voice& voice, const Ngs2VoiceParamHeader& param) {
    switch (param.id) {
        case SCE_NGS2_SAMPLER_VOICE_PARAM_SETUP:
        case SCE_NGS2_CUSTOM_SAMPLER_VOICE_PARAM_SETUP:
            SetupSampler(voice, ParamAs<Ngs2SamplerVoiceSetupParam>(param).format);
            break;
        case SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS:
        case SCE_NGS2_CUSTOM_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS:
            AddWaveformBlocks(voice, ParamAs<Ngs2SamplerVoiceWaveformBlocksParam>(param));
            break;
        case SCE_NGS2_SAMPLER_VOICE_PARAM_EXIT_LOOP:
        case SCE_NGS2_CUSTOM_SAMPLER_VOICE_PARAM_EXIT_LOOP: {
            const auto looping = std::find_if(voice.blocks.begin(), voice.blocks.end(), [](const Ngs2Block& block) { return block.info.num_repeats != 0; });
            if (looping != voice.blocks.end()) looping->info.num_repeats = 0;
            break;
        }
        case SCE_NGS2_SAMPLER_VOICE_PARAM_PITCH:
        case SCE_NGS2_CUSTOM_SAMPLER_VOICE_PARAM_PITCH: {
            const float ratio = ParamAs<Ngs2SamplerVoicePitchParam>(param).ratio;
            if (!std::isfinite(ratio) || ratio < 0.0f) APS5_INVALID_ARG_EX;
            voice.pitch = ratio;
            break;
        }
        case SCE_NGS2_SAMPLER_VOICE_PARAM_FILTER: Ngs2SetFilter(voice, reinterpret_cast<const Ngs2SamplerVoiceFilterParam&>(param)); break;
        default: throw std::runtime_error("NGS2: sampler voice param " + Ngs2Hex(param.id) + " is not implemented");
    }
}

static void SetupMixer(Ngs2Voice& voice, std::uint32_t numChannels) {
    if (numChannels == 0 || numChannels > voice.rack->maxChannels) APS5_INVALID_ARG_EX;
    voice.ResetSetup();
    voice.channels = numChannels;
}

static void ApplyParam(Ngs2Voice& voice, const Ngs2VoiceParamHeader& param) {
    const std::uint32_t rackId = param.id >> 16;
    if (rackId == 0) {
        ApplyCommonParam(voice, param);
        return;
    }
    if (rackId == (SCE_NGS2_CUSTOM_VOICE_PARAM_USER_FX2 >> 16) || rackId == SCE_NGS2_RACK_ID_CUSTOM_SUBMIXER) {
        Ngs2ApplyCustomParam(voice, param);
        return;
    }
    if (rackId != voice.rack->rackId) throw std::invalid_argument("NGS2: voice param " + Ngs2Hex(param.id) + " belongs to another rack");
    switch (rackId) {
        case SCE_NGS2_RACK_ID_SAMPLER:
        case SCE_NGS2_RACK_ID_CUSTOM_SAMPLER: ApplySamplerParam(voice, param); return;
        case SCE_NGS2_RACK_ID_SUBMIXER:
            if (param.id == SCE_NGS2_SUBMIXER_VOICE_PARAM_USER_FX) {
                const auto& fx = ParamAs<Ngs2SubmixerVoiceUserFxParam>(param);
                voice.userFxHandler = fx.handler;
                voice.userFxData = {fx.user_data0, fx.user_data1, fx.user_data2};
                voice.userFxFlags = fx.handler ? 1 : 0;
                return;
            }
            if (param.id != SCE_NGS2_SUBMIXER_VOICE_PARAM_SETUP) break;
            if (ParamAs<Ngs2SubmixerVoiceSetupParam>(param).flags != 0) throw std::runtime_error("NGS2: submixer setup flags are not implemented");
            SetupMixer(voice, ParamAs<Ngs2SubmixerVoiceSetupParam>(param).num_io_channels);
            return;
        case SCE_NGS2_RACK_ID_REVERB:
            if (param.id == SCE_NGS2_REVERB_VOICE_PARAM_SETUP) {
                const auto& setup = ParamAs<Ngs2ReverbVoiceSetupParam>(param);
                if (setup.flags != 0) throw std::runtime_error("NGS2: reverb setup flags are not implemented");
                if (setup.num_input_channels != setup.num_output_channels) throw std::runtime_error("NGS2: reverb channel conversion is not implemented");
                if (setup.num_output_channels != 1 && setup.num_output_channels != 2 && setup.num_output_channels != 6 && setup.num_output_channels != 8) APS5_INVALID_ARG_EX;
                SetupMixer(voice, setup.num_output_channels);
                Ngs2SetupReverb(voice);
                return;
            }
            if (param.id == SCE_NGS2_REVERB_VOICE_PARAM_I3DL2) {
                Ngs2SetReverbParams(voice, ParamAs<Ngs2ReverbVoiceI3DL2Param>(param).i3dl2);
                return;
            }
            break;
        case SCE_NGS2_RACK_ID_MASTERING:
            if (param.id == SCE_NGS2_MASTERING_VOICE_PARAM_SETUP) {
                SetupMixer(voice, ParamAs<Ngs2MasteringVoiceSetupParam>(param).num_io_channels);
                return;
            }
            if (param.id == SCE_NGS2_MASTERING_VOICE_PARAM_GAIN) {
                const auto& gain = ParamAs<Ngs2MasteringVoiceGainParam>(param);
                if (!std::isfinite(gain.fbw_level) || !std::isfinite(gain.lfe_level)) APS5_INVALID_ARG_EX;
                voice.fbwLevel = gain.fbw_level;
                voice.lfeLevel = gain.lfe_level;
                return;
            }
            if (param.id == SCE_NGS2_MASTERING_VOICE_PARAM_OUTPUT) {
                voice.outputId = ParamAs<Ngs2MasteringVoiceOutputParam>(param).output_id;
                return;
            }
            break;
        default: break;
    }
    throw std::runtime_error("NGS2: voice param " + Ngs2Hex(param.id) + " is not implemented");
}

#pragma GCC visibility push(default)

extern "C" {

int APS5_VABI sceNgs2VoiceControl(uintptr_t voice_handle, const Ngs2VoiceParamHeader* param_list) {
    if (param_list == nullptr) return SCE_NGS2_ERROR_NULL_PARAM_LIST;
    const Ngs2Lock lock;
    auto* found = Ngs2FindVoice(voice_handle);
    if (found == nullptr) return SCE_NGS2_ERROR_INVALID_VOICE_HANDLE;
    auto& voice = *found;
    try {
        for (const auto* param = param_list;; param = reinterpret_cast<const Ngs2VoiceParamHeader*>(reinterpret_cast<const std::uint8_t*>(param) + param->next)) {
            ApplyParam(voice, *param);
            if (param->next == 0) break;
        }
    } catch (const Ngs2ReturnCode& result) {
        return result.code;
    }
    return SCE_NGS2_OK;
}

int APS5_VABI sceNgs2VoiceRunCommands(uintptr_t voice_handle, const Ngs2VoiceCommand* commands, size_t num_commands) {
    const Ngs2Lock lock;
    auto* found = Ngs2FindVoice(voice_handle);
    if (found == nullptr) return SCE_NGS2_ERROR_INVALID_VOICE_HANDLE;
    auto& voice = *found;
    if (commands == nullptr && num_commands != 0) APS5_INVALID_ARG_EX;
    for (std::size_t i = 0; i < num_commands; i++) {
        const auto& command = commands[i];
        const std::uint32_t index = command.id >> 24;
        const std::uint32_t id = command.id & 0xffffff;
        const std::uint8_t expectedType = id == COMMAND_EVENT ? COMMAND_TYPE_UINT
                                        : id == COMMAND_MATRIX_LEVELS ? COMMAND_TYPE_FLOAT_ARRAY
                                        : id == COMMAND_PORT_VOLUME ? COMMAND_TYPE_FLOAT
                                        : id == COMMAND_PORT_MATRIX ? COMMAND_TYPE_INT : 0;
        if (expectedType == 0) throw std::runtime_error("NGS2: voice command " + Ngs2Hex(command.id) + " is not implemented");
        if (command.type != expectedType) throw std::invalid_argument("NGS2: voice command " + Ngs2Hex(command.id) + " has value type " + Ngs2Hex(command.type));
        if (id == COMMAND_EVENT) voice.SetEvent(command.value.u);
        else if (id == COMMAND_MATRIX_LEVELS) SetMatrixLevels(voice, index, command.value.levels, command.count);
        else if (id == COMMAND_PORT_VOLUME) PortAt(voice, index).volume = command.value.f;
        else SetPortMatrix(voice, index, command.value.i);
    }
    return SCE_NGS2_OK;
}

int APS5_VABI sceNgs2VoiceGetState(uintptr_t voice_handle, Ngs2VoiceState* state, size_t state_size) {
    const Ngs2Lock lock;
    auto* found = Ngs2FindVoice(voice_handle);
    if (found == nullptr) return SCE_NGS2_ERROR_INVALID_VOICE_HANDLE;
    if (state_size < sizeof(Ngs2VoiceState)) return SCE_NGS2_ERROR_INVALID_OUT_SIZE;
    if (state == nullptr) return SCE_NGS2_ERROR_INVALID_OUT_ADDRESS;
    const auto& voice = *found;
    switch (voice.rack->rackId) {
        case SCE_NGS2_RACK_ID_SAMPLER: {
            if (state_size != sizeof(Ngs2SamplerVoiceState)) return SCE_NGS2_ERROR_INVALID_OUT_SIZE;
            auto& sampler = *reinterpret_cast<Ngs2SamplerVoiceState*>(state);
            sampler = {};
            sampler.voice_state.state_flags = voice.stateFlags;
            sampler.envelope_height = 1.0f;
            sampler.num_decoded_samples = voice.decodedSamples;
            sampler.decoded_data_size = voice.decodedBytes;
            sampler.user_data = voice.blocks.empty() ? 0 : voice.blocks.front().info.user_data;
            sampler.waveform_data = voice.WaveformData();
            return SCE_NGS2_OK;
        }
        case SCE_NGS2_RACK_ID_REVERB: {
            if (state_size != sizeof(Ngs2VoiceState)) return SCE_NGS2_ERROR_INVALID_OUT_SIZE;
            *state = {};
            state->state_flags = voice.stateFlags;
            return SCE_NGS2_OK;
        }
        case SCE_NGS2_RACK_ID_SUBMIXER: {
            if (state_size != sizeof(Ngs2SubmixerVoiceState)) return SCE_NGS2_ERROR_INVALID_OUT_SIZE;
            auto& submixer = *reinterpret_cast<Ngs2SubmixerVoiceState*>(state);
            submixer = {};
            submixer.voice_state.state_flags = voice.stateFlags;
            return SCE_NGS2_OK;
        }
        case SCE_NGS2_RACK_ID_CUSTOM_SAMPLER: {
            if (state_size != sizeof(Ngs2CustomSamplerVoiceState)) return SCE_NGS2_ERROR_INVALID_OUT_SIZE;
            auto& sampler = *reinterpret_cast<Ngs2CustomSamplerVoiceState*>(state);
            sampler = {};
            sampler.voice_state.state_flags = voice.stateFlags;
            sampler.num_decoded_samples = voice.decodedSamples;
            return SCE_NGS2_OK;
        }
        default: throw std::runtime_error("NGS2: voice state of rack " + Ngs2Hex(voice.rack->rackId) + " is not implemented");
    }
}

int APS5_VABI sceNgs2VoiceGetPortInfo(uintptr_t voice_handle, uint32_t port, Ngs2VoicePortInfo* info, size_t info_size) {
    if (info == nullptr) return SCE_NGS2_ERROR_INVALID_OUT_ADDRESS;
    if (info_size != sizeof(Ngs2VoicePortInfo)) return SCE_NGS2_ERROR_INVALID_OUT_SIZE;
    const Ngs2Lock lock;
    auto* found = Ngs2FindVoice(voice_handle);
    if (found == nullptr) return SCE_NGS2_ERROR_INVALID_VOICE_HANDLE;
    auto& voice = *found;
    const auto& source = PortAt(voice, port);
    *info = {};
    info->matrix_id = source.matrix;
    info->volume = source.volume;
    info->dest_handle = reinterpret_cast<Ngs2Handle>(source.dest);
    return SCE_NGS2_OK;
}

int APS5_VABI sceNgs2VoiceQueryInfo(uintptr_t voice_handle, uint32_t info_id, void* info, size_t info_size) {
    if (info == nullptr) return SCE_NGS2_ERROR_INVALID_OUT_ADDRESS;
    const Ngs2Lock lock;
    auto* found = Ngs2FindVoice(voice_handle);
    if (found == nullptr) return SCE_NGS2_ERROR_INVALID_VOICE_HANDLE;
    const auto& voice = *found;
    switch (info_id) {
        case SCE_NGS2_VOICE_INFO_CHANNELS: {
            if (info_size != sizeof(Ngs2VoiceChannelsInfo)) return SCE_NGS2_ERROR_INVALID_OUT_SIZE;
            *static_cast<Ngs2VoiceChannelsInfo*>(info) = {voice.channels, 0};
            return SCE_NGS2_OK;
        }
        default: throw std::runtime_error("NGS2: voice info " + Ngs2Hex(info_id) + " is not implemented");
    }
}

int APS5_VABI sceNgs2VoiceGetStateFlags(uintptr_t voice_handle, uint32_t* state_flags) {
    if (state_flags == nullptr) return SCE_NGS2_ERROR_INVALID_OUT_ADDRESS;
    const Ngs2Lock lock;
    const auto* found = Ngs2FindVoice(voice_handle);
    if (found == nullptr) return SCE_NGS2_ERROR_INVALID_VOICE_HANDLE;
    *state_flags = found->stateFlags;
    return SCE_NGS2_OK;
}

int APS5_VABI sceNgs2VoiceGetOwner(uintptr_t voice_handle, uintptr_t* owner_rack, uint32_t* voice_index) {
    std::lock_guard lock(Ngs2Mutex());
    const auto* found = Ngs2FindVoice(voice_handle);
    if (found == nullptr) return SCE_NGS2_ERROR_INVALID_VOICE_HANDLE;
    if (owner_rack == nullptr || voice_index == nullptr) return SCE_NGS2_ERROR_INVALID_OUT_ADDRESS;
    *owner_rack = reinterpret_cast<Ngs2Handle>(found->rack);
    *voice_index = static_cast<uint32_t>(found - found->rack->voices.data());
    return SCE_NGS2_OK;
}

int APS5_VABI sceNgs2VoiceGetMatrixInfo(uintptr_t voice_handle, uint32_t matrix_id, Ngs2VoiceMatrixInfo* info, size_t info_size) {
    if (info == nullptr) return SCE_NGS2_ERROR_INVALID_OUT_ADDRESS;
    if (info_size != sizeof(Ngs2VoiceMatrixInfo)) return SCE_NGS2_ERROR_INVALID_OUT_SIZE;
    const Ngs2Lock lock;
    const auto* found = Ngs2FindVoice(voice_handle);
    if (found == nullptr) return SCE_NGS2_ERROR_INVALID_VOICE_HANDLE;
    if (matrix_id >= found->matrices.size()) APS5_INVALID_ARG_EX;
    const auto& levels = found->matrices[matrix_id];
    *info = {};
    info->num_levels = static_cast<std::uint32_t>(levels.size());
    std::copy(levels.begin(), levels.end(), info->levels);
    return SCE_NGS2_OK;
}

int APS5_VABI sceNgs2PanInit(Ngs2PanWork* work, const float* speaker_angles, float unit_angle, uint32_t num_speakers) {
    return Ngs2PanInit(work, speaker_angles, unit_angle, num_speakers);
}

int APS5_VABI sceNgs2PanGetVolumeMatrix(Ngs2PanWork* work, const Ngs2PanParam* params, uint32_t num_params, uint32_t matrix_format, float* out_volume_matrix) {
    return Ngs2PanGetVolumeMatrix(work, params, num_params, matrix_format, out_volume_matrix);
}

int APS5_VABI sceNgs2GeomApply(const Ngs2GeomListenerWork* listener, const Ngs2GeomSourceParam* source, Ngs2GeomAttribute* out_attrib, std::uint32_t flags) {
    return Ngs2GeomApply(listener, source, out_attrib, flags);
}

int APS5_VABI sceNgs2GeomResetListenerParam(Ngs2GeomListenerParam* out_listener_param) {
    return Ngs2GeomResetListenerParam(out_listener_param);
}

int APS5_VABI sceNgs2GeomCalcListener(const Ngs2GeomListenerParam* param, Ngs2GeomListenerWork* outWork, std::uint32_t flags) {
    return Ngs2GeomCalcListener(param, outWork, flags);
}

int APS5_VABI sceNgs2GeomResetSourceParam(Ngs2GeomSourceParam* out_source_param) {
    return Ngs2GeomResetSourceParam(out_source_param);
}

}

#pragma GCC visibility pop
