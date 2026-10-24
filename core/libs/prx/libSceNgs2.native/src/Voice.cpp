#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <mutex>
#include <numbers>
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
static constexpr std::uint32_t FOREVER = 0xffffffff;
static constexpr std::uint64_t NEVER = std::uint64_t{1} << 62;

struct Ngs2ReturnCode {
    int code;
};

std::uint64_t Ngs2PcmBlock::Frames() const {
    std::uint64_t frames = 0;
    for (const auto& piece : pieces) frames += piece.frames;
    return frames;
}

std::uint64_t Ngs2PcmBlock::Pass() const {
    return empty ? 0 : std::min<std::uint64_t>(static_cast<std::uint64_t>(info.num_skip_samples) + info.num_samples, Frames());
}

bool Ngs2Voice::Pcm() const {
    return waveformType == SCE_NGS2_WAVEFORM_TYPE_PCM_I16L;
}

static std::uint64_t RunEnd(const Ngs2PcmRun& run) {
    return run.passes == 0 ? NEVER : run.start + run.length * run.passes;
}

void Ngs2BuildPcmRuns(Ngs2Voice& voice) {
    voice.runs = voice.prefix;
    std::uint64_t start = voice.runs.empty() ? 0 : RunEnd(voice.runs.back());
    for (const auto& queued : voice.order) {
        const auto& block = queued.block;
        const bool forever = block.info.num_repeats == FOREVER;
        if (forever && block.Pass() == 0) throw std::runtime_error("NGS2: an endlessly repeated PCM waveform block with no frames is not implemented");
        voice.runs.push_back({block, start, block.Pass(), forever ? 0 : std::max<std::uint64_t>(block.info.num_repeats, 1), 0});
        start = std::min(RunEnd(voice.runs.back()), NEVER);
    }
    voice.total = start;
}

static const std::uint8_t* PieceFrame(const Ngs2PcmBlock& block, std::uint64_t frame, std::size_t frameBytes) {
    const auto& piece = *std::prev(std::upper_bound(block.pieces.begin(), block.pieces.end(), frame, [](std::uint64_t at, const Ngs2Piece& next) { return at < next.firstFrame; }));
    return piece.data + (frame - piece.firstFrame) * frameBytes;
}

const std::uint8_t* Ngs2PcmFrame(const Ngs2Voice& voice, std::int64_t index) {
    if (index < 0) return nullptr;
    const auto at = static_cast<std::uint64_t>(index);
    for (const auto& run : voice.runs) {
        if (run.length == 0 || at < run.start || at >= RunEnd(run)) continue;
        return PieceFrame(run.block, (at - run.start) % run.length + run.first, voice.channels * sizeof(std::int16_t));
    }
    return nullptr;
}

static std::uint64_t PcmDecodedFrames(const Ngs2Voice& voice) {
    return std::min(voice.position >> NGS2_POSITION_BITS, voice.total);
}

std::uint64_t Ngs2PcmDecoded(const Ngs2Voice& voice) {
    const auto decoded = PcmDecodedFrames(voice);
    std::uint64_t skipped = 0;
    for (const auto& run : voice.runs) {
        if (run.length == 0 || run.start > decoded) continue;
        const auto started = (decoded - run.start) / run.length + 1;
        skipped += (run.passes == 0 ? started : std::min(started, run.passes)) * run.block.info.num_skip_samples;
    }
    return decoded > skipped ? decoded - skipped : 0;
}

std::uint64_t Ngs2PcmDecodedBytes(const Ngs2Voice& voice) {
    const auto frameBytes = voice.channels * sizeof(std::int16_t);
    if (!voice.starving) return PcmDecodedFrames(voice) * frameBytes;
    std::uint64_t frames = 0;
    for (const auto& run : voice.runs) frames += run.block.Frames() * run.passes;
    return frames * frameBytes;
}

std::uint32_t Ngs2PcmStateFlags(const Ngs2Voice& voice) {
    if (voice.state != Ngs2PlayState::Playing) return voice.inUse ? SCE_NGS2_VOICE_STATE_FLAG_INUSE : static_cast<std::uint32_t>(voice.state);
    std::uint32_t flags = SCE_NGS2_VOICE_STATE_FLAG_INUSE | SCE_NGS2_VOICE_STATE_FLAG_PLAYING;
    if (voice.ran == Ngs2Pause::FadeOut) flags = SCE_NGS2_VOICE_STATE_FLAG_INUSE | SCE_NGS2_VOICE_STATE_FLAG_PAUSED;
    if (voice.ran == Ngs2Pause::Silent || voice.ran == Ngs2Pause::Frozen) flags |= SCE_NGS2_VOICE_STATE_FLAG_PAUSED;
    return voice.starving ? flags | SCE_NGS2_VOICE_STATE_FLAG_EMPTY : flags;
}

static const Ngs2PcmRun* RunAt(const Ngs2Voice& voice, std::uint64_t at) {
    for (const auto& run : voice.runs) {
        if (run.length != 0 && at >= run.start && at < RunEnd(run)) return &run;
    }
    return nullptr;
}

static std::vector<Ngs2PcmRun> Clip(const std::vector<Ngs2PcmRun>& runs, std::uint64_t at) {
    std::vector<Ngs2PcmRun> kept;
    for (const auto& run : runs) {
        if (run.start >= at) continue;
        if (RunEnd(run) <= at) {
            kept.push_back(run);
            continue;
        }
        const auto full = (at - run.start) / run.length;
        const auto part = (at - run.start) % run.length;
        if (full != 0) {
            kept.push_back(run);
            kept.back().passes = full;
        }
        if (part != 0) {
            kept.push_back(run);
            kept.back().start = run.start + full * run.length;
            kept.back().length = part;
            kept.back().passes = 1;
        }
    }
    return kept;
}

void Ngs2ApplyPendingClear(Ngs2Voice& voice) {
    voice.clearPending = false;
    const auto old = std::move(voice.clearedRuns);
    voice.clearedRuns.clear();
    const auto unsupported = [] { throw std::runtime_error("NGS2: a RESET add on a playing PCM sampler voice with no PLAY before the next render is not implemented for this queue"); };
    if (voice.order.size() != 1) unsupported();
    const auto block = voice.order.front().block;
    if (block.Pass() == 0 || block.info.num_skip_samples != 0 || block.info.num_repeats > 1) unsupported();
    voice.runs = old;
    voice.total = old.empty() ? 0 : std::min(RunEnd(old.back()), NEVER);
    const auto decoded = PcmDecodedFrames(voice);
    if (voice.starving) {
        voice.prefix = old;
        voice.prefix.push_back({block, decoded, block.Pass(), 1, 0});
    } else {
        const auto* run = RunAt(voice, decoded);
        if (run == nullptr) unsupported();
        const auto offset = (decoded - run->start) % run->length;
        if (offset >= block.Pass()) unsupported();
        const auto length = std::min(block.Pass() - offset, run->length - offset);
        voice.prefix = Clip(old, decoded);
        voice.prefix.push_back({block, decoded, length, 1, offset});
    }
    voice.order.clear();
    Ngs2BuildPcmRuns(voice);
}

static void DropPendingClear(Ngs2Voice& voice) {
    if (!voice.clearPending) return;
    voice.clearPending = false;
    voice.clearedRuns.clear();
    voice.prefix.clear();
    Ngs2BuildPcmRuns(voice);
}

static void PlayPcm(Ngs2Voice& voice) {
    voice.clearPending = false;
    voice.clearedRuns.clear();
    voice.prefix.clear();
    voice.order.clear();
    for (std::uint32_t slot = 0; slot < voice.writeIndex; slot++) voice.order.push_back({slot, voice.slots[slot]});
    Ngs2BuildPcmRuns(voice);
    voice.inUse = true;
    voice.stateFlags |= SCE_NGS2_VOICE_STATE_FLAG_INUSE;
    if (voice.order.empty()) return;
    voice.state = Ngs2PlayState::Playing;
    voice.position = 0;
    voice.signalledTo = 0;
    voice.pause = Ngs2Pause::None;
    voice.pausePending = false;
    voice.renderedSincePlay = false;
    voice.starving = false;
}

static bool SetPcmEvent(Ngs2Voice& voice, std::uint32_t eventId) {
    switch (eventId) {
        case SCE_NGS2_VOICE_EVENT_PLAY:
            PlayPcm(voice);
            if (voice.userFxHandler) voice.userFxFlags = 1;
            return true;
        case SCE_NGS2_VOICE_EVENT_PAUSE:
            if (voice.state != Ngs2PlayState::Playing || voice.pause != Ngs2Pause::None) return true;
            if (voice.clearPending) throw std::runtime_error("NGS2: PAUSE after a RESET add on a playing PCM sampler voice is not implemented");
            voice.pausePending = true;
            return true;
        case SCE_NGS2_VOICE_EVENT_RESUME:
            if (voice.pausePending) voice.pausePending = false;
            else if (voice.pause == Ngs2Pause::Silent || voice.pause == Ngs2Pause::Frozen) voice.pause = Ngs2Pause::FadeIn;
            return true;
        case SCE_NGS2_VOICE_EVENT_STOP:
        case SCE_NGS2_VOICE_EVENT_STOP_IMM:
        case SCE_NGS2_VOICE_EVENT_KILL:
            voice.inUse = false;
            if (voice.rack->rackId == SCE_NGS2_RACK_ID_SAMPLER) voice.decodedSamples = 0;
            voice.decodedBytes = 0;
            DropPendingClear(voice);
            return false;
        default: return false;
    }
}

void Ngs2Voice::SetEvent(std::uint32_t eventId) {
    if (Pcm() && SetPcmEvent(*this, eventId)) return;
    switch (eventId) {
        case SCE_NGS2_VOICE_EVENT_PLAY:
            if (state == Ngs2PlayState::Empty || state == Ngs2PlayState::Stopped) {
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
    slots.clear();
    writeIndex = 0;
    order.clear();
    prefix.clear();
    runs.clear();
    clearedRuns.clear();
    total = 0;
    position = 0;
    signalledTo = 0;
    pause = Ngs2Pause::None;
    ran = Ngs2Pause::None;
    pausePending = false;
    renderedSincePlay = false;
    starving = false;
    inUse = false;
    clearPending = false;
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

static const Ngs2PcmRun* DecodedRun(const Ngs2Voice& voice) {
    if (voice.state != Ngs2PlayState::Playing) return nullptr;
    const auto decoded = PcmDecodedFrames(voice);
    const auto* run = RunAt(voice, decoded);
    if (run != nullptr || decoded == 0) return run;
    return RunAt(voice, decoded - 1);
}

static std::uint64_t PcmUserData(const Ngs2Voice& voice) {
    const auto* run = DecodedRun(voice);
    return run == nullptr ? 0 : run->block.info.user_data;
}

const std::uint8_t* Ngs2Voice::WaveformData() const {
    if (Pcm()) {
        const auto* run = DecodedRun(*this);
        if (run == nullptr) return nullptr;
        const auto offset = PcmDecodedFrames(*this) - run->start;
        return PieceFrame(run->block, (offset == run->length * run->passes ? run->length : offset % run->length) + run->first, channels * sizeof(std::int16_t));
    }
    if (blocks.empty()) return waveformEnd;
    return blocks.front().data + blocks.front().dataCursor;
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

static void AppendStreamData(Ngs2Voice& voice, const Ngs2SamplerVoiceWaveformBlocksParam& param) {
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
}

static std::uint32_t Unconsumed(const Ngs2Voice& voice) {
    const auto decoded = PcmDecodedFrames(voice);
    std::uint32_t count = 0;
    for (const auto& run : voice.runs) {
        if (voice.state != Ngs2PlayState::Playing || RunEnd(run) > decoded) count++;
    }
    return count;
}

static void AddPcmBlocks(Ngs2Voice& voice, const Ngs2SamplerVoiceWaveformBlocksParam& param) {
    const auto capacity = voice.rack->maxWaveformBlocks;
    if (capacity == 0) throw std::runtime_error("NGS2: a sampler rack with no waveform blocks is not implemented");
    if (voice.clearPending) throw std::runtime_error("NGS2: an add after a RESET add on a playing PCM sampler voice is not implemented");
    const std::size_t frameBytes = voice.channels * sizeof(std::int16_t);
    for (std::uint32_t i = 0; i < param.num_blocks; i++) {
        if (param.blocks[i].data_size % frameBytes != 0) throw std::runtime_error("NGS2: a PCM waveform block with a partial frame is not implemented");
    }
    if (voice.slots.empty()) voice.slots.resize(capacity);
    if ((param.flags & SCE_NGS2_WAVEFORM_BLOCKS_FLAG_RESET) != 0) {
        ++voice.waveformRevision;
        if (voice.state == Ngs2PlayState::Playing) {
            if (voice.pausePending || voice.pause != Ngs2Pause::None) throw std::runtime_error("NGS2: a RESET add on a paused PCM sampler voice is not implemented");
            voice.clearedRuns = voice.runs;
            voice.clearPending = true;
        }
        voice.writeIndex = 0;
        voice.order.clear();
        voice.prefix.clear();
        voice.acceptsBlocks = true;
        Ngs2BuildPcmRuns(voice);
    }
    if (!voice.acceptsBlocks) throw Ngs2ReturnCode{SCE_NGS2_ERROR_FULL_WAVEFORM_BLOCKS};
    const bool onlyData = (param.flags & SCE_NGS2_WAVEFORM_BLOCKS_FLAG_APPEND) != 0;
    const auto dataOnly = [&](const Ngs2WaveformBlock& block) { return onlyData || (block.num_samples == 0 && block.num_skip_samples == 0); };
    const auto normal = static_cast<std::uint32_t>(std::count_if(param.blocks, param.blocks + param.num_blocks, [&](const Ngs2WaveformBlock& block) { return !dataOnly(block); }));
    if (normal > capacity - std::min(capacity, Unconsumed(voice))) throw Ngs2ReturnCode{SCE_NGS2_ERROR_FULL_WAVEFORM_BLOCKS};
    std::uint32_t placed = 0;
    for (std::uint32_t i = 0; i < param.num_blocks; i++) {
        const auto& block = param.blocks[i];
        const auto* data = static_cast<const std::uint8_t*>(param.data) + block.data_offset;
        if (dataOnly(block) && !voice.order.empty()) {
            auto& last = voice.order.back();
            last.block.pieces.push_back({data, last.block.Frames(), block.data_size / frameBytes});
            voice.slots[last.slot] = last.block;
            continue;
        }
        if (dataOnly(block) && !voice.prefix.empty()) throw std::runtime_error("NGS2: data added after a RESET add on a playing PCM sampler voice is not implemented");
        if (voice.writeIndex == capacity) {
            if (placed != 0) throw std::runtime_error("NGS2: a waveform block add that wraps the block ring part way is not implemented");
            voice.writeIndex = 0;
        }
        voice.slots[voice.writeIndex] = {block, {{data, 0, block.data_size / frameBytes}}, dataOnly(block)};
        voice.order.push_back({voice.writeIndex, voice.slots[voice.writeIndex]});
        voice.writeIndex++;
        placed++;
    }
    voice.acceptsBlocks = (param.flags & SCE_NGS2_WAVEFORM_BLOCKS_FLAG_CONTINUE) != 0;
    Ngs2BuildPcmRuns(voice);
}

static void AddWaveformBlocks(Ngs2Voice& voice, const Ngs2SamplerVoiceWaveformBlocksParam& param) {
    constexpr std::uint32_t knownFlags = SCE_NGS2_WAVEFORM_BLOCKS_FLAG_CONTINUE | SCE_NGS2_WAVEFORM_BLOCKS_FLAG_APPEND | SCE_NGS2_WAVEFORM_BLOCKS_FLAG_RESET | SCE_NGS2_WAVEFORM_BLOCKS_FLAG_SILENCE;
    constexpr std::uint32_t pcmFlags = knownFlags | SCE_NGS2_WAVEFORM_BLOCKS_FLAG_CANCEL;
    if ((param.flags & ~(voice.Pcm() ? pcmFlags : knownFlags)) != 0) throw std::runtime_error("NGS2: waveform block flags " + Ngs2Hex(param.flags) + " are not implemented");
    if ((param.num_blocks != 0 && (voice.channels == 0 || param.blocks == nullptr || param.data == nullptr)) || (voice.channels == 0 && (param.flags & SCE_NGS2_WAVEFORM_BLOCKS_FLAG_APPEND) != 0)) APS5_INVALID_ARG_EX;
    if (voice.Pcm()) {
        AddPcmBlocks(voice, param);
        return;
    }
    const bool reset = (param.flags & SCE_NGS2_WAVEFORM_BLOCKS_FLAG_RESET) != 0;
    if (!voice.acceptsBlocks && !reset) throw std::invalid_argument("NGS2: the voice waveform was already closed");
    if (reset) {
        ++voice.waveformRevision;
        voice.blocks.clear();
        voice.phase = 0;
        voice.waveformEnd = nullptr;
        if (voice.waveformType == SCE_NGS2_WAVEFORM_TYPE_ATRAC9) Ngs2RestartAtrac9(voice);
    }
    if ((param.flags & SCE_NGS2_WAVEFORM_BLOCKS_FLAG_APPEND) != 0) {
        AppendStreamData(voice, param);
        voice.acceptsBlocks = (param.flags & SCE_NGS2_WAVEFORM_BLOCKS_FLAG_CONTINUE) != 0;
        return;
    }
    voice.acceptsBlocks = (param.flags & SCE_NGS2_WAVEFORM_BLOCKS_FLAG_CONTINUE) != 0;
    const bool allowsSilence = (param.flags & SCE_NGS2_WAVEFORM_BLOCKS_FLAG_SILENCE) != 0;
    for (std::uint32_t i = 0; i < param.num_blocks; i++) {
        const auto& block = param.blocks[i];
        if (block.num_samples == 0 && block.data_size == 0) continue;
        if (allowsSilence && block.data_size == 0) throw std::runtime_error("NGS2: a dataless waveform block under the silence flag is not implemented for ATRAC9 voices");
        const std::uint64_t bytes = Ngs2Atrac9BlockBytes(voice, block);
        const bool compressedStream = voice.acceptsBlocks && block.num_repeats == 0;
        if (block.num_samples == 0 || (bytes > block.data_size && !compressedStream)) {
            throw std::invalid_argument("NGS2: waveform block " + std::to_string(i) + " does not fit its data");
        }
        voice.blocks.push_back({static_cast<const std::uint8_t*>(param.data) + block.data_offset, block});
    }
}

void Ngs2SetFilter(Ngs2Voice& voice, const Ngs2SamplerVoiceFilterParam& param) {
    if (param.index >= voice.rack->maxFilters) APS5_INVALID_ARG_EX;
    if (voice.filters.size() <= param.index) voice.filters.resize(param.index + 1);
    auto& filter = voice.filters[param.index];
    if (param.type == 0) {
        filter = {};
        return;
    }
    if (param.location != 1 || param.type != 1) {
        throw std::runtime_error("NGS2: sampler filter location " + std::to_string(param.location) + " type " + std::to_string(param.type) + " is not implemented");
    }
    if (!std::isfinite(param.frequency) || param.frequency < 0.0f || !std::isfinite(param.q) || param.q <= 0.0f || !std::isfinite(param.level) || param.level < 0.0f) {
        APS5_INVALID_ARG_EX;
    }
    const double rate = voice.rack->system->option.sample_rate;
    filter.enabled = true;
    filter.bypassMask = param.channel_mask;
    filter.history.resize(voice.channels);
    if (param.frequency == 0.0f || param.frequency >= rate * 0.5) {
        filter.b0 = param.frequency == 0.0f ? 0.0 : param.level;
        filter.b1 = filter.b2 = filter.a1 = filter.a2 = 0.0;
        return;
    }
    const double omega = 2.0 * std::numbers::pi * param.frequency / rate;
    const double cosine = std::cos(omega);
    const double alpha = std::sin(omega) / (2.0 * param.q);
    const double inverseA0 = 1.0 / (1.0 + alpha);
    filter.b0 = (1.0 - cosine) * 0.5 * inverseA0 * param.level;
    filter.b1 = 2.0 * filter.b0;
    filter.b2 = filter.b0;
    filter.a1 = -2.0 * cosine * inverseA0;
    filter.a2 = (1.0 - alpha) * inverseA0;
}

static void ExitPcmLoop(Ngs2Voice& voice) {
    for (std::size_t i = 0; i < voice.order.size(); i++) {
        const auto& run = voice.runs[voice.prefix.size() + i];
        if (run.passes == 1) continue;
        const auto decoded = PcmDecodedFrames(voice);
        const auto current = voice.state == Ngs2PlayState::Playing && decoded > run.start ? (decoded - run.start) / run.length + 1 : 1;
        voice.order[i].block.info.num_repeats = static_cast<std::uint32_t>(run.passes == 0 ? current : std::min(current, run.passes));
        Ngs2BuildPcmRuns(voice);
        return;
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
            if (voice.Pcm()) {
                ExitPcmLoop(voice);
                break;
            }
            const auto looping = std::find_if(voice.blocks.begin(), voice.blocks.end(), [](const Ngs2Block& block) { return block.info.num_repeats != 0; });
            if (looping != voice.blocks.end()) looping->info.num_repeats = 0;
            break;
        }
        case SCE_NGS2_SAMPLER_VOICE_PARAM_PITCH:
        case SCE_NGS2_CUSTOM_SAMPLER_VOICE_PARAM_PITCH: {
            const float ratio = ParamAs<Ngs2SamplerVoicePitchParam>(param).ratio;
            if (voice.waveformType != SCE_NGS2_WAVEFORM_TYPE_PCM_I16L && (!std::isfinite(ratio) || ratio < 0.0f)) APS5_INVALID_ARG_EX;
            voice.pitch = ratio;
            break;
        }
        case SCE_NGS2_SAMPLER_VOICE_PARAM_FILTER: Ngs2SetFilter(voice, ParamAs<Ngs2SamplerVoiceFilterParam>(param)); break;
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
            sampler.user_data = voice.Pcm() ? PcmUserData(voice) : voice.blocks.empty() ? 0 : voice.blocks.front().info.user_data;
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
