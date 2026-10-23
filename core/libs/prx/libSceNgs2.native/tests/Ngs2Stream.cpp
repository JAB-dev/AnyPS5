#include "Ngs2Test.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

static constexpr int FullWaveformBlocks = static_cast<int>(0x804A8054u);

static std::vector<std::int16_t> RenderI16(uintptr_t system) {
    std::vector<std::int16_t> out(Grain, -1);
    const Ngs2RenderBufferInfo info{out.data(), out.size() * sizeof(std::int16_t), SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, 1};
    Require(sceNgs2SystemRender(system, &info, 1) == SCE_NGS2_OK);
    return out;
}

static uintptr_t RackVoice(uintptr_t rack, std::uint32_t index) {
    uintptr_t voice = 0;
    Require(sceNgs2RackGetVoiceHandle(rack, index, &voice) == SCE_NGS2_OK && voice != 0);
    return voice;
}

static uintptr_t StreamVoice(uintptr_t rack, std::uint32_t index, uintptr_t master) {
    const auto voice = RackVoice(rack, index);
    Control(voice, SCE_NGS2_SAMPLER_VOICE_PARAM_SETUP, Ngs2SamplerVoiceSetupParam{{}, {SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, 1, 48000, 0, 0, 0}});
    Patch(voice, master);
    return voice;
}

static void AddBlock(uintptr_t voice, const void* data, std::uint64_t bytes, std::uint32_t samples, std::uint32_t flags) {
    const Ngs2WaveformBlock block{0, bytes, 0, 0, samples, 0, 0};
    Control(voice, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS, Ngs2SamplerVoiceWaveformBlocksParam{{}, data, flags, 1, &block});
}

static int Add(uintptr_t voice, const std::vector<std::int16_t>& pcm, std::uint32_t numBlocks, std::uint32_t flags) {
    const Ngs2WaveformBlock block{0, pcm.size() * sizeof(std::int16_t), 0, 0, static_cast<std::uint32_t>(pcm.size()), 0, 0};
    const Ngs2SamplerVoiceWaveformBlocksParam param{{sizeof(Ngs2SamplerVoiceWaveformBlocksParam), 0, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS}, pcm.data(), flags, numBlocks, &block};
    return sceNgs2VoiceControl(voice, &param.header);
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

static void TestStreamEndsAtDeclaredSamples() {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 1);
    const auto rack = CreateRack(system, SCE_NGS2_RACK_ID_SAMPLER);
    const auto voice = StreamVoice(rack, 0, master);
    const std::vector<std::int16_t> first(6, 16384);
    const std::vector<std::int16_t> second(8, 8192);
    const std::vector<std::int16_t> last(6, 4096);

    AddBlock(voice, first.data(), first.size() * 2, 20, SCE_NGS2_WAVEFORM_BLOCKS_FLAG_CONTINUE);
    Event(voice, SCE_NGS2_VOICE_EVENT_PLAY);
    auto out = RenderI16(system);
    for (std::uint32_t i = 0; i < Grain; i++) Require(out[i] == (i < 6 ? 16384 : 0));
    Require(Flags(voice) == (SCE_NGS2_VOICE_STATE_FLAG_INUSE | SCE_NGS2_VOICE_STATE_FLAG_PLAYING));
    Ngs2SamplerVoiceState state{};
    Require(sceNgs2VoiceGetState(voice, &state.voice_state, sizeof(state)) == SCE_NGS2_OK);
    Require(state.num_decoded_samples == 6 && state.decoded_data_size == 12 && state.waveform_data == first.data() + 6);

    out = RenderI16(system);
    for (std::uint32_t i = 0; i < Grain; i++) Require(out[i] == 0);
    Require(sceNgs2VoiceGetState(voice, &state.voice_state, sizeof(state)) == SCE_NGS2_OK);
    Require(state.num_decoded_samples == 6 && state.decoded_data_size == 12);

    AddBlock(voice, second.data(), second.size() * 2, 20, SCE_NGS2_WAVEFORM_BLOCKS_FLAG_CONTINUE | SCE_NGS2_WAVEFORM_BLOCKS_FLAG_APPEND);
    Require(sceNgs2VoiceGetState(voice, &state.voice_state, sizeof(state)) == SCE_NGS2_OK && state.waveform_data == second.data());
    out = RenderI16(system);
    for (std::uint32_t i = 0; i < Grain; i++) Require(out[i] == 8192);
    Require(sceNgs2VoiceGetState(voice, &state.voice_state, sizeof(state)) == SCE_NGS2_OK);
    Require(state.num_decoded_samples == 14 && state.decoded_data_size == 28 && state.waveform_data == second.data() + 8);

    AddBlock(voice, last.data(), last.size() * 2, 20, SCE_NGS2_WAVEFORM_BLOCKS_FLAG_APPEND);
    out = RenderI16(system);
    for (std::uint32_t i = 0; i < Grain; i++) Require(out[i] == (i < 6 ? 4096 : 0));
    Require(Flags(voice) == 0);
    Require(sceNgs2VoiceGetState(voice, &state.voice_state, sizeof(state)) == SCE_NGS2_OK);
    Require(state.num_decoded_samples == 20 && state.decoded_data_size == 40 && state.waveform_data == last.data() + 6);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static void TestStreamClosedBeforeDeclaredSamples() {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 1);
    const auto rack = CreateRack(system, SCE_NGS2_RACK_ID_SAMPLER);
    const auto voice = StreamVoice(rack, 0, master);
    const std::vector<std::int16_t> first(6, 16384);
    const std::vector<std::int16_t> closing(4, 8192);

    AddBlock(voice, first.data(), first.size() * 2, 1000, SCE_NGS2_WAVEFORM_BLOCKS_FLAG_CONTINUE);
    Event(voice, SCE_NGS2_VOICE_EVENT_PLAY);
    AddBlock(voice, closing.data(), closing.size() * 2, 1000, SCE_NGS2_WAVEFORM_BLOCKS_FLAG_APPEND);
    auto out = RenderI16(system);
    for (std::uint32_t i = 0; i < Grain; i++) Require(out[i] == (i < 6 ? 16384 : 8192));
    Require(Flags(voice) == (SCE_NGS2_VOICE_STATE_FLAG_INUSE | SCE_NGS2_VOICE_STATE_FLAG_PLAYING));
    out = RenderI16(system);
    for (std::uint32_t i = 0; i < Grain; i++) Require(out[i] == (i < 2 ? 8192 : 0));
    Require(Flags(voice) == 0);
    Ngs2SamplerVoiceState state{};
    Require(sceNgs2VoiceGetState(voice, &state.voice_state, sizeof(state)) == SCE_NGS2_OK);
    Require(state.num_decoded_samples == 10 && state.decoded_data_size == 20);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static void TestStreamRejections() {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 1);
    const auto rack = CreateRack(system, SCE_NGS2_RACK_ID_SAMPLER);
    const std::vector<std::int16_t> pcm(6, 1000);

    const auto empty = StreamVoice(rack, 0, master);
    Require(Throws([&] { AddBlock(empty, pcm.data(), pcm.size() * 2, 6, SCE_NGS2_WAVEFORM_BLOCKS_FLAG_APPEND); }));

    const auto complete = StreamVoice(rack, 1, master);
    AddBlock(complete, pcm.data(), pcm.size() * 2, 6, SCE_NGS2_WAVEFORM_BLOCKS_FLAG_CONTINUE);
    Require(Throws([&] { AddBlock(complete, pcm.data(), pcm.size() * 2, 6, SCE_NGS2_WAVEFORM_BLOCKS_FLAG_CONTINUE | SCE_NGS2_WAVEFORM_BLOCKS_FLAG_APPEND); }));

    const auto repeating = StreamVoice(rack, 2, master);
    const Ngs2WaveformBlock repeated{0, pcm.size() * 2, 1, 0, 20, 0, 0};
    Require(Throws([&] { Control(repeating, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS, Ngs2SamplerVoiceWaveformBlocksParam{{}, pcm.data(), SCE_NGS2_WAVEFORM_BLOCKS_FLAG_CONTINUE, 1, &repeated}); }));

    const auto skipping = StreamVoice(rack, 3, master);
    const Ngs2WaveformBlock skipped{0, pcm.size() * 2, 0, 1, 20, 0, 0};
    Require(Throws([&] { Control(skipping, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS, Ngs2SamplerVoiceWaveformBlocksParam{{}, pcm.data(), SCE_NGS2_WAVEFORM_BLOCKS_FLAG_CONTINUE, 1, &skipped}); }));

    const auto partial = StreamVoice(rack, 4, master);
    AddBlock(partial, pcm.data(), pcm.size() * 2, 20, SCE_NGS2_WAVEFORM_BLOCKS_FLAG_CONTINUE);
    Require(Throws([&] { AddBlock(partial, pcm.data(), 3, 20, SCE_NGS2_WAVEFORM_BLOCKS_FLAG_CONTINUE | SCE_NGS2_WAVEFORM_BLOCKS_FLAG_APPEND); }));
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static const std::int16_t Unused = 0;

static void TestSilenceBlock() {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 1);
    const auto rack = CreateRack(system, SCE_NGS2_RACK_ID_SAMPLER);
    const auto voice = StreamVoice(rack, 0, master);

    AddBlock(voice, &Unused, 0, Grain, SCE_NGS2_WAVEFORM_BLOCKS_FLAG_SILENCE);
    Event(voice, SCE_NGS2_VOICE_EVENT_PLAY);
    auto out = RenderI16(system);
    for (std::uint32_t i = 0; i < Grain; i++) Require(out[i] == 0);
    Require(Flags(voice) == 0);
    Ngs2SamplerVoiceState state{};
    Require(sceNgs2VoiceGetState(voice, &state.voice_state, sizeof(state)) == SCE_NGS2_OK);
    Require(state.num_decoded_samples == Grain && state.decoded_data_size == Grain * 2);

    const auto withData = StreamVoice(rack, 1, master);
    const std::vector<std::int16_t> pcm(Grain, 1234);
    AddBlock(withData, pcm.data(), pcm.size() * 2, Grain, SCE_NGS2_WAVEFORM_BLOCKS_FLAG_SILENCE);
    Event(withData, SCE_NGS2_VOICE_EVENT_PLAY);
    out = RenderI16(system);
    for (std::uint32_t i = 0; i < Grain; i++) Require(out[i] == 1234);
    Require(Flags(withData) == 0);

    const auto dataless = StreamVoice(rack, 2, master);
    Require(Throws([&] { AddBlock(dataless, &Unused, 0, Grain, 0); }));
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

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
    Require(Add(voice, pcm, 1, 0) == FullWaveformBlocks);
    Ngs2SamplerVoiceWaveformBlocksParam append{{sizeof(Ngs2SamplerVoiceWaveformBlocksParam), 0, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS}, &Unused, SCE_NGS2_WAVEFORM_BLOCKS_FLAG_APPEND, 0, &unused};
    Require(sceNgs2VoiceControl(voice, &append.header) == FullWaveformBlocks);
    Control(voice, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS, Ngs2SamplerVoiceWaveformBlocksParam{{}, &Unused, SCE_NGS2_WAVEFORM_BLOCKS_FLAG_RESET | SCE_NGS2_WAVEFORM_BLOCKS_FLAG_CONTINUE, 0, &unused});
    append.flags |= SCE_NGS2_WAVEFORM_BLOCKS_FLAG_CONTINUE;
    Require(sceNgs2VoiceControl(voice, &append.header) == SCE_NGS2_OK);
    Require(Throws([&] { Add(voice, pcm, 1, SCE_NGS2_WAVEFORM_BLOCKS_FLAG_CONTINUE); }));
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static void TestDrainedStreamReportsEmpty() {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 1);
    const auto rack = CreateRack(system, SCE_NGS2_RACK_ID_SAMPLER);
    const auto voice = StreamVoice(rack, 0, master);
    const std::vector<std::int16_t> pcm(4, 16384);
    constexpr std::uint32_t playing = SCE_NGS2_VOICE_STATE_FLAG_INUSE | SCE_NGS2_VOICE_STATE_FLAG_PLAYING;

    AddBlock(voice, pcm.data(), pcm.size() * 2, 4, SCE_NGS2_WAVEFORM_BLOCKS_FLAG_RESET | SCE_NGS2_WAVEFORM_BLOCKS_FLAG_CONTINUE);
    Event(voice, SCE_NGS2_VOICE_EVENT_PLAY);
    RenderI16(system);
    Require(Flags(voice) == (playing | SCE_NGS2_VOICE_STATE_FLAG_EMPTY));

    const std::vector<std::int16_t> longer(Grain * 2, 8192);
    AddBlock(voice, longer.data(), longer.size() * 2, Grain * 2, SCE_NGS2_WAVEFORM_BLOCKS_FLAG_CONTINUE);
    const auto out = RenderI16(system);
    Require(out[0] == 8192);
    Require(Flags(voice) == playing);

    AddBlock(voice, pcm.data(), pcm.size() * 2, 4, 0);
    RenderI16(system);
    RenderI16(system);
    Require((Flags(voice) & SCE_NGS2_VOICE_STATE_FLAG_EMPTY) == 0);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static bool Silent(const std::vector<std::int16_t>& out) {
    return std::all_of(out.begin(), out.end(), [](std::int16_t sample) { return sample == 0; });
}

static Ngs2SamplerVoiceState SamplerState(uintptr_t voice) {
    Ngs2SamplerVoiceState state{};
    Require(sceNgs2VoiceGetState(voice, &state.voice_state, sizeof(state)) == SCE_NGS2_OK);
    return state;
}

static bool SameState(const Ngs2SamplerVoiceState& a, const Ngs2SamplerVoiceState& b) {
    return std::memcmp(&a, &b, sizeof(a)) == 0;
}

static void TestClosedWaveformReturnsFull() {
    constexpr int full = FullWaveformBlocks;
    constexpr std::uint32_t continued = SCE_NGS2_WAVEFORM_BLOCKS_FLAG_CONTINUE;
    constexpr std::uint32_t playing = SCE_NGS2_VOICE_STATE_FLAG_INUSE | SCE_NGS2_VOICE_STATE_FLAG_PLAYING;
    const auto system = CreateSystem();
    const auto master = Mastering(system, 1);
    const auto rack = CreateRack(system, SCE_NGS2_RACK_ID_SAMPLER);
    const std::vector<std::int16_t> pcm(Grain * 2, 4096);
    const std::vector<std::int16_t> other(Grain * 2, 1000);

    const auto bare = RackVoice(rack, 6);
    Require(Throws([&] { Add(bare, pcm, 1, 0); }));
    Require(Add(bare, pcm, 0, 0) == SCE_NGS2_OK);
    Require(Add(bare, pcm, 1, 0) == full && Add(bare, pcm, 1, 1) == full);

    const auto fresh = RackVoice(rack, 0);
    Require(Add(fresh, pcm, 0, 3) == SCE_NGS2_OK && Add(fresh, pcm, 0, 0x10) == SCE_NGS2_OK);
    for (std::uint32_t flags : {0x0u, 0x1u, 0x2u, 0x3u, 0x8u, 0xAu, 0x10u, 0x20u}) Require(Add(fresh, pcm, 0, flags) == full);
    for (std::uint32_t flags : {0x4u, 0xCu, 0x5u, 0x8u, 0x6u, 0x7u, 0x20u}) Require(Add(fresh, pcm, 0, flags) == SCE_NGS2_OK);
    Require(Add(fresh, pcm, 0, 1) == full);
    for (std::uint32_t flags : {0x9u, 0x21u, 0x28u, 0x40u}) Require(Throws([&] { Add(fresh, pcm, 0, flags | 4); }));

    const auto closed = StreamVoice(rack, 1, master);
    Require(Add(closed, pcm, 1, 0) == SCE_NGS2_OK);
    Event(closed, SCE_NGS2_VOICE_EVENT_PLAY);
    Require(RenderI16(system)[0] == 4096);
    const auto before = SamplerState(closed);
    Require(Add(closed, other, 1, 0) == full && Add(closed, other, 0, 2) == full);
    Require(SameState(before, SamplerState(closed)));
    struct List {
        Ngs2SamplerVoiceWaveformBlocksParam add;
        Ngs2VoicePortVolumeParam volume;
    };
    const Ngs2WaveformBlock otherBlock{0, other.size() * sizeof(std::int16_t), 0, 0, static_cast<std::uint32_t>(other.size()), 0, 0};
    const List list{{{sizeof(Ngs2SamplerVoiceWaveformBlocksParam), static_cast<std::int16_t>(offsetof(List, volume)), SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS}, other.data(), 0, 1, &otherBlock},
                    {{sizeof(Ngs2VoicePortVolumeParam), 0, SCE_NGS2_VOICE_PARAM_PORT_VOLUME}, 0, 0.0f}};
    Require(sceNgs2VoiceControl(closed, &list.add.header) == full);
    Require(SameState(before, SamplerState(closed)));
    const auto rest = RenderI16(system);
    Require(std::all_of(rest.begin(), rest.end(), [](std::int16_t sample) { return sample == 4096; }));
    Require(Silent(RenderI16(system)) && Flags(closed) == 0);
    for (std::uint32_t event : {SCE_NGS2_VOICE_EVENT_STOP_IMM, SCE_NGS2_VOICE_EVENT_KILL, SCE_NGS2_VOICE_EVENT_STOP, SCE_NGS2_VOICE_EVENT_PAUSE, SCE_NGS2_VOICE_EVENT_PLAY}) {
        Event(closed, event);
        Require(Add(closed, pcm, 0, 0) == full && Add(closed, pcm, 1, 1) == full);
    }
    Control(closed, SCE_NGS2_SAMPLER_VOICE_PARAM_SETUP, Ngs2SamplerVoiceSetupParam{{}, {SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, 1, 48000, 0, 0, 0}});
    Require(Add(closed, pcm, 0, 0) == SCE_NGS2_OK && Add(closed, pcm, 1, 0) == full);

    const auto open = StreamVoice(rack, 2, master);
    Require(Add(open, pcm, 1, continued) == SCE_NGS2_OK);
    Event(open, SCE_NGS2_VOICE_EVENT_PLAY);
    Require(RenderI16(system)[0] == 4096);
    for (std::uint32_t event : {SCE_NGS2_VOICE_EVENT_STOP, SCE_NGS2_VOICE_EVENT_PAUSE}) {
        Event(open, event);
        Require(Add(open, pcm, 0, continued) == SCE_NGS2_OK);
    }
    Require(Add(open, pcm, 0, SCE_NGS2_WAVEFORM_BLOCKS_FLAG_APPEND | continued) == SCE_NGS2_OK);
    Event(open, SCE_NGS2_VOICE_EVENT_STOP_IMM);
    Require(Add(open, pcm, 0, continued) == SCE_NGS2_OK);
    Require(Add(open, pcm, 0, 0) == SCE_NGS2_OK && Add(open, pcm, 1, continued) == full);
    Event(open, SCE_NGS2_VOICE_EVENT_PLAY);
    Require(RenderI16(system)[0] == 4096 && Flags(open) == 0);

    for (std::uint32_t flags : {SCE_NGS2_WAVEFORM_BLOCKS_FLAG_RESET, SCE_NGS2_WAVEFORM_BLOCKS_FLAG_RESET | continued}) {
        const auto cleared = StreamVoice(rack, flags == SCE_NGS2_WAVEFORM_BLOCKS_FLAG_RESET ? 3 : 4, master);
        Require(Add(cleared, pcm, 1, continued) == SCE_NGS2_OK);
        Event(cleared, SCE_NGS2_VOICE_EVENT_PLAY);
        Require(RenderI16(system)[0] == 4096);
        Require(Add(cleared, pcm, 0, flags) == SCE_NGS2_OK);
        Require(Silent(RenderI16(system)));
        Require(Flags(cleared) == ((flags & continued) != 0 ? playing | SCE_NGS2_VOICE_STATE_FLAG_EMPTY : 0));
    }

    const auto blocked = StreamVoice(rack, 5, master);
    for (std::uint32_t flags : {0x8u, 0x9u, 0xAu, 0x20u}) Require(Throws([&] { Add(blocked, pcm, 1, flags); }));
    Require(Add(blocked, pcm, 1, continued) == SCE_NGS2_OK);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

int main() {
    TestStreamEndsAtDeclaredSamples();
    TestStreamClosedBeforeDeclaredSamples();
    TestStreamRejections();
    TestSilenceBlock();
    TestDrainedStreamReportsEmpty();
    TestNoBlocksForClearedVoice();
    TestClosedWaveformReturnsFull();
    return 0;
}
