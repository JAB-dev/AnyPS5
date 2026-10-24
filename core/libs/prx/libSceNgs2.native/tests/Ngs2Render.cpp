#include "Ngs2Test.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

static uintptr_t Sampler(uintptr_t system, const std::vector<std::int16_t>& pcm, std::uint32_t repeats, uintptr_t* rack = nullptr) {
    const auto created = CreateRack(system, SCE_NGS2_RACK_ID_SAMPLER);
    if (rack != nullptr) *rack = created;
    const auto voice = Voice(created);
    Control(voice, SCE_NGS2_SAMPLER_VOICE_PARAM_SETUP, Ngs2SamplerVoiceSetupParam{{}, {SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, 1, 48000, 0, 0, 0}});
    const Ngs2WaveformBlock block{0, pcm.size() * sizeof(std::int16_t), repeats, 0, static_cast<std::uint32_t>(pcm.size()), 0, 0x55};
    Control(voice, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS, Ngs2SamplerVoiceWaveformBlocksParam{{}, pcm.data(), 0, 1, &block});
    return voice;
}

static std::int16_t Delayed(const std::vector<std::int16_t>& pcm, std::size_t frame) {
    return frame >= 3 && frame - 3 < pcm.size() ? pcm[frame - 3] : 0;
}

static std::vector<std::uint32_t> callbackFlags;
static void APS5_VABI OnBlock(const Ngs2VoiceCallbackInfo* info) {
    Require(info->callback_data == 7 && info->user_data == 0x55);
    callbackFlags.push_back(info->flag);
}

static std::vector<std::int16_t> RenderI16(uintptr_t system) {
    std::vector<std::int16_t> out(Grain, -1);
    const Ngs2RenderBufferInfo info{out.data(), out.size() * sizeof(std::int16_t), SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, 1};
    Require(sceNgs2SystemRender(system, &info, 1) == SCE_NGS2_OK);
    return out;
}

static void TestErrorsAndInfo() {
    Ngs2SystemOption option{};
    Require(sceNgs2SystemResetOption(&option) == SCE_NGS2_OK);
    Require(option.size == sizeof(option) && option.max_grain_samples == 512 && option.num_grain_samples == 256 && option.sample_rate == 48000);
    Require(sceNgs2SystemQueryBufferSize(&option, nullptr) == SCE_NGS2_ERROR_INVALID_OUT_ADDRESS);
    Require(sceNgs2RackQueryBufferSize(SCE_NGS2_RACK_ID_SAMPLER, nullptr, nullptr) == SCE_NGS2_ERROR_INVALID_OUT_ADDRESS);

    Ngs2SystemInfo info{};
    Require(sceNgs2SystemGetInfo(0x1234, &info, sizeof(info)) == static_cast<int>(0x804A8201u));
    Require(sceNgs2RackDestroy(0x1234, nullptr) == static_cast<int>(0x804A8202u));

    const auto system = CreateSystem();
    const auto rack = CreateRack(system, SCE_NGS2_RACK_ID_SAMPLER);
    Ngs2RackInfo rackInfo{};
    Require(sceNgs2RackGetInfo(0x1234, &rackInfo, sizeof(rackInfo)) == SCE_NGS2_ERROR_INVALID_RACK_HANDLE);
    Require(sceNgs2RackGetInfo(rack, nullptr, sizeof(rackInfo)) == SCE_NGS2_ERROR_INVALID_OUT_ADDRESS);
    Require(sceNgs2RackGetInfo(rack, &rackInfo, sizeof(rackInfo) - 1) == SCE_NGS2_ERROR_INVALID_OUT_SIZE);
    Require(sceNgs2RackGetInfo(rack, &rackInfo, sizeof(rackInfo)) == SCE_NGS2_OK);
    Require(rackInfo.rack_handle == rack && rackInfo.owner_system_handle == system && rackInfo.rack_id == SCE_NGS2_RACK_ID_SAMPLER && rackInfo.type == 1);
    Require(rackInfo.uid != 0 && rackInfo.max_voices == 256 && rackInfo.max_channel_works == 256 && rackInfo.max_grain_samples == 512);
    Require(rackInfo.max_ports == 8 && rackInfo.max_matrices == 1 && rackInfo.active_voice_count == 0 && rackInfo.name[0] == '\0');
    Ngs2SubmixerRackOption named{};
    named.rack_option.size = sizeof(named);
    for (auto& c : named.rack_option.name) c = 'n';
    named.rack_option.max_grain_samples = 256;
    named.rack_option.max_voices = 2;
    named.rack_option.max_matrices = 1;
    named.rack_option.max_ports = 2;
    named.max_channels = 2;
    named.max_inputs = 3;
    Ngs2ContextBufferInfo namedQuery{};
    Require(sceNgs2RackQueryBufferSize(SCE_NGS2_RACK_ID_SUBMIXER, &named.rack_option, &namedQuery) == SCE_NGS2_OK);
    const auto namedBuffer = Buffer(namedQuery);
    uintptr_t namedRack = 0;
    Require(sceNgs2RackCreate(system, SCE_NGS2_RACK_ID_SUBMIXER, &named.rack_option, &namedBuffer, &namedRack) == SCE_NGS2_OK);
    Ngs2RackInfo namedInfo{};
    Require(sceNgs2RackGetInfo(namedRack, &namedInfo, sizeof(namedInfo)) == SCE_NGS2_OK);
    Require(namedInfo.name[0] == 'n' && namedInfo.name[62] == 'n' && namedInfo.name[63] == '\0');
    Require(namedInfo.type == 2 && namedInfo.rack_id == SCE_NGS2_RACK_ID_SUBMIXER && namedInfo.uid != rackInfo.uid && namedInfo.max_grain_samples == 256);
    Require(namedInfo.max_voices == 2 && namedInfo.max_ports == 2 && namedInfo.max_inputs == 3 && namedInfo.max_channel_works == 0);
    Require(namedInfo.buffer_info.host_buffer == namedBuffer.host_buffer && namedInfo.render_count == 0);
    uintptr_t named0 = 0;
    Require(sceNgs2RackGetVoiceHandle(namedRack, 0, &named0) == SCE_NGS2_OK);
    Control(named0, SCE_NGS2_SUBMIXER_VOICE_PARAM_SETUP, Ngs2SubmixerVoiceSetupParam{{}, 2, 0});
    Event(named0, SCE_NGS2_VOICE_EVENT_PLAY);
    Require(sceNgs2RackGetInfo(namedRack, &namedInfo, sizeof(namedInfo)) == SCE_NGS2_OK && namedInfo.active_voice_count == 1);
    Ngs2VoiceChannelsInfo stereoInfo{};
    Require(sceNgs2VoiceQueryInfo(named0, SCE_NGS2_VOICE_INFO_CHANNELS, &stereoInfo, sizeof(stereoInfo)) == SCE_NGS2_OK && stereoInfo.num_channels == 2);
    Require(sceNgs2SystemGetInfo(system, nullptr, sizeof(info)) == SCE_NGS2_ERROR_INVALID_OUT_ADDRESS);
    Require(sceNgs2SystemGetInfo(system, &info, sizeof(info) - 1) == SCE_NGS2_ERROR_INVALID_OUT_SIZE);
    Require(sceNgs2SystemGetInfo(system, &info, sizeof(info)) == SCE_NGS2_OK);
    Require(info.system_handle == system && info.uid != 0 && info.rack_count == 2 && info.sample_rate == 48000);
    Require(info.num_grain_samples == Grain && info.max_grain_samples == 512 && info.render_count == 0);

    Ngs2ContextBufferInfo released{};
    Require(sceNgs2SystemDestroy(system, &released) == SCE_NGS2_OK && released.host_buffer != nullptr);
    Require(sceNgs2SystemGetInfo(system, &info, sizeof(info)) == SCE_NGS2_ERROR_INVALID_SYSTEM_HANDLE);
}

static void TestPcmBlockEnd() {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 1);
    std::vector<std::int16_t> pcm;
    for (int i = 0; i < 12; i++) pcm.push_back(static_cast<std::int16_t>(i * 1000 - 4000));
    const auto sampler = Sampler(system, pcm, 0);
    Patch(sampler, master);
    Ngs2VoicePortInfo portInfo{};
    Require(sceNgs2VoiceGetPortInfo(sampler, 0, nullptr, sizeof(portInfo)) == SCE_NGS2_ERROR_INVALID_OUT_ADDRESS);
    Require(sceNgs2VoiceGetPortInfo(sampler, 0, &portInfo, sizeof(portInfo) - 1) == SCE_NGS2_ERROR_INVALID_OUT_SIZE);
    Require(sceNgs2VoiceGetPortInfo(sampler, 0, &portInfo, sizeof(portInfo)) == SCE_NGS2_OK);
    Require(portInfo.dest_handle == master && portInfo.matrix_id == -1 && portInfo.volume == 1.0f);
    Control(sampler, SCE_NGS2_VOICE_PARAM_PORT_VOLUME, Ngs2VoicePortVolumeParam{{}, 0, 0.5f});
    Control(sampler, SCE_NGS2_VOICE_PARAM_PORT_MATRIX, Ngs2VoicePortMatrixParam{{}, 0, 0});
    Require(sceNgs2VoiceGetPortInfo(sampler, 0, &portInfo, sizeof(portInfo)) == SCE_NGS2_OK);
    Require(portInfo.dest_handle == master && portInfo.matrix_id == 0 && portInfo.volume == 0.5f);
    Control(sampler, SCE_NGS2_VOICE_PARAM_PORT_VOLUME, Ngs2VoicePortVolumeParam{{}, 0, 1.0f});
    Control(sampler, SCE_NGS2_VOICE_PARAM_PORT_MATRIX, Ngs2VoicePortMatrixParam{{}, 0, -1});
    Require(sceNgs2VoiceGetPortInfo(sampler, 1, &portInfo, sizeof(portInfo)) == SCE_NGS2_OK && portInfo.dest_handle == 0);
    bool portOutOfRange = false;
    try { sceNgs2VoiceGetPortInfo(sampler, 8, &portInfo, sizeof(portInfo)); } catch (const std::invalid_argument&) { portOutOfRange = true; }
    Require(portOutOfRange);
    Ngs2VoiceChannelsInfo channelsInfo{};
    Require(sceNgs2VoiceQueryInfo(sampler, SCE_NGS2_VOICE_INFO_CHANNELS, nullptr, sizeof(channelsInfo)) == SCE_NGS2_ERROR_INVALID_OUT_ADDRESS);
    Require(sceNgs2VoiceQueryInfo(sampler, SCE_NGS2_VOICE_INFO_CHANNELS, &channelsInfo, sizeof(channelsInfo) - 1) == SCE_NGS2_ERROR_INVALID_OUT_SIZE);
    Require(sceNgs2VoiceQueryInfo(sampler, SCE_NGS2_VOICE_INFO_CHANNELS, &channelsInfo, sizeof(channelsInfo)) == SCE_NGS2_OK);
    Require(channelsInfo.num_channels == 1 && channelsInfo.reserved == 0);
    bool unknownInfo = false;
    try { sceNgs2VoiceQueryInfo(sampler, 0x4000, &channelsInfo, sizeof(channelsInfo)); } catch (const std::runtime_error&) { unknownInfo = true; }
    Require(unknownInfo);
    Control(sampler, SCE_NGS2_VOICE_PARAM_CALLBACK, Ngs2VoiceCallbackParam{{}, OnBlock, 7, SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_END, 0});
    Require(Flags(sampler) == 0);
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    Require(Flags(sampler) == SCE_NGS2_VOICE_STATE_FLAG_INUSE);

    callbackFlags.clear();
    auto out = RenderI16(system);
    for (std::uint32_t i = 0; i < Grain; i++) Require(out[i] == Delayed(pcm, i));
    Require(Flags(sampler) == (SCE_NGS2_VOICE_STATE_FLAG_INUSE | SCE_NGS2_VOICE_STATE_FLAG_PLAYING) && callbackFlags.empty());

    Ngs2SamplerVoiceState state{};
    Require(sceNgs2VoiceGetState(sampler, &state.voice_state, sizeof(state) - 8) == SCE_NGS2_ERROR_INVALID_OUT_SIZE);
    Require(sceNgs2VoiceGetState(sampler, &state.voice_state, sizeof(state)) == SCE_NGS2_OK);
    Require(state.num_decoded_samples == Grain && state.decoded_data_size == Grain * 2 && state.user_data == 0x55);
    Require(state.waveform_data == pcm.data() + Grain);

    out = RenderI16(system);
    for (std::uint32_t i = 0; i < Grain; i++) Require(out[i] == Delayed(pcm, Grain + i));
    Require(callbackFlags.size() == 1 && callbackFlags[0] == SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_END);
    Require(Flags(sampler) == 0);
    Require(sceNgs2VoiceGetState(sampler, &state.voice_state, sizeof(state)) == SCE_NGS2_OK);
    Require(state.num_decoded_samples == 0 && state.waveform_data == nullptr && state.user_data == 0);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static void TestPatchToRackHandle() {
    const auto system = CreateSystem();
    const auto masteringRack = CreateRack(system, SCE_NGS2_RACK_ID_MASTERING);
    const auto master = Voice(masteringRack);
    Control(master, SCE_NGS2_MASTERING_VOICE_PARAM_SETUP, Ngs2MasteringVoiceSetupParam{{}, 1, 0});
    Control(master, SCE_NGS2_MASTERING_VOICE_PARAM_OUTPUT, Ngs2MasteringVoiceOutputParam{{}, 0, 0});
    Event(master, SCE_NGS2_VOICE_EVENT_PLAY);

    const std::vector<std::int16_t> pcm(Grain, 16384);
    const auto sampler = Sampler(system, pcm, 0);
    Control(sampler, SCE_NGS2_VOICE_PARAM_PATCH, Ngs2VoicePatchParam{{}, 0, 0, masteringRack});

    Ngs2VoicePortInfo port{};
    Require(sceNgs2VoiceGetPortInfo(sampler, 0, &port, sizeof(port)) == SCE_NGS2_OK);
    Require(port.dest_handle == master && port.dest_input_id == 0);
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    const auto out = RenderI16(system);
    for (std::uint32_t i = 0; i < Grain; i++) Require(out[i] == Delayed(pcm, i));
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static bool Near(float a, float b) {
    return std::fabs(a - b) < 1e-4f;
}

static bool PanThrows(Ngs2PanWork* work, const Ngs2PanParam* params, std::uint32_t numParams, std::uint32_t format) {
    float out[64] = {};
    try { sceNgs2PanGetVolumeMatrix(work, params, numParams, format, out); } catch (const std::exception&) { return true; }
    return false;
}

static bool PanInitThrows(Ngs2PanWork* work, const float* angles, float unitAngle, std::uint32_t numSpeakers) {
    try { sceNgs2PanInit(work, angles, unitAngle, numSpeakers); } catch (const std::exception&) { return true; }
    return false;
}

struct PanCase {
    std::vector<float> angles;
    float unitAngle;
    std::uint32_t numSpeakers;
    std::vector<Ngs2PanParam> params;
    std::uint32_t format;
    std::vector<float> expected;
};

static void TestPan() {
    const float pi = std::numbers::pi_v<float>;
    Ngs2PanWork work{};
    Require(sceNgs2PanInit(&work, nullptr, 360.0f, 2) == SCE_NGS2_OK);
    Require(work.num_speakers == 2 && work.unit_angle == 360.0f);
    Require(work.speaker_angles[0] == 90.0f && work.speaker_angles[1] == 270.0f && work.speaker_angles[2] == 450.0f);
    Require(sceNgs2PanInit(&work, nullptr, 2.0f * pi, 2) == SCE_NGS2_OK && work.unit_angle == 2.0f * pi);
    Require(work.speaker_angles[0] == 90.0f / 180.0f * pi && work.speaker_angles[1] == 270.0f / 180.0f * pi && work.speaker_angles[2] == 450.0f);
    Require(sceNgs2PanInit(&work, nullptr, 100.0f, 4) == SCE_NGS2_OK && work.unit_angle == 360.0f && work.speaker_angles[4] == 400.0f);
    const float frontBack[] = {-30.0f, 30.0f, 0.0f, -110.0f, 470.0f};
    Require(sceNgs2PanInit(&work, frontBack, 360.0f, 5) == SCE_NGS2_OK);
    const float circle[] = {0.0f, 30.0f, 110.0f, 250.0f, 330.0f, 360.0f};
    for (std::uint32_t i = 0; i < 6; i++) Require(work.speaker_angles[i] == circle[i]);

    const std::vector<PanCase> cases = {
        {{}, 360.0f, 2, {{0, 1, 1, 0}, {45, 1, 1, 0}, {90, 1, 1, 0}, {135, 1, 1, 0}, {180, 1, 1, 0}, {225, 1, 1, 0}, {270, 1, 1, 0}, {315, 1, 1, 0}}, 2,
         {0.707106769f, 0.382683456f, 0.0f, 0.382683456f, 0.707106769f, 0.923879504f, 1.0f, 0.923879504f, 0.707106769f, 0.923879504f, 1.0f, 0.923879504f, 0.707106769f, 0.382683456f, 0.0f, 0.382683456f}},
        {{}, 360.0f, 2, {{30, 0, 1, 0}, {30, -1, 1, 0}, {30, 0.5f, 0.5f, 0}, {30, 3, 1, 0}, {-30, -0.25f, 1, 0}}, 2,
         {0.965925813f, 0.866025448f, 0.39667666f, 0.49999997f, 0.896872759f, 0.965925813f, 0.49999997f, 0.495722443f, 0.866025448f, 0.997858942f}},
        {{}, 360.0f, 2, {{60, 1, 1, 0.5f}}, 1, {0.866025388f}},
        {{}, 360.0f, 2, {{60, 1, 1, 0.5f}}, 6, {0.258819073f, 0.965925813f, 0.0f, 0.0f, 0.0f, 0.0f}},
        {{}, 2.0f * pi, 2, {{0.25f * pi, 1, 1, 0}, {1.5f * pi, 1, 1, 0}}, 2, {0.999965429f, 1.0f, 0.00831161533f, 0.0f}},
        {{}, 360.0f, 4, {{100, 1, 1, 0.25f}}, 6, {0.0f, 0.222520918f, 0.0f, 0.25f, 0.0f, 0.974927902f}},
        {{}, 360.0f, 4, {{100, 1, 1, 0.25f}}, 2, {0.0f, 0.709984899f}},
        {{}, 360.0f, 5, {{70, 1, 0.8f, 0.5f}, {200, 0.5f, 1, 0}}, 6,
         {0.0f, 0.0f, 0.625465155f, 0.270598054f, 0.0f, 0.270598054f, 0.5f, 0.0f, 0.0f, 0.782271147f, 0.498791903f, 0.491533577f}},
        {{}, 360.0f, 5, {{70, 1, 0.8f, 0.5f}}, 1, {0.691666603f}},
        {{}, 360.0f, 7, {{180, 1, 1, 0.25f}}, 8, {0.0f, 0.0f, 0.0f, 0.25f, 0.0f, 0.0f, 0.707106769f, 0.707106769f}},
        {{}, 360.0f, 7, {{180, 1, 1, 0.25f}}, 6, {0.0f, 0.0f, 0.0f, 0.25f, 0.707106769f, 0.707106769f}},
        {{-pi / 6, pi / 6, 0, -110 * pi / 180, 110 * pi / 180}, 2.0f * pi, 5, {{1.0f, 1, 1, 0}, {-2.5f, 0.2f, 1, 0.1f}}, 6,
         {0.0f, 0.0f, 0.859783232f, 0.582614243f, 0.0f, 0.0f, 0.0f, 0.100000001f, 0.0f, 0.753403723f, 0.510659218f, 0.372568965f}},
        {{-30, 30, 0, -90, 90, -150, 150}, 360.0f, 7, {{-120, 1, 1, 0}}, 1, {0.707106769f}},
        {{-45, 45}, 360.0f, 2, {{10, 1, 1, 0}, {720 + 170, 1, 1, 0}}, 2, {0.57357651f, 0.664795876f, 0.819152057f, 0.747025073f}},
        {{-40, 40, -120, 120}, 360.0f, 4, {{-80, 0.75f, 1, 0}}, 8, {0.69351995f, 0.0746578425f, 0.0f, 0.0f, 0.69351995f, 0.180239946f, 0.0f, 0.0f}},
    };
    for (const auto& panCase : cases) {
        Require(sceNgs2PanInit(&work, panCase.angles.empty() ? nullptr : panCase.angles.data(), panCase.unitAngle, panCase.numSpeakers) == SCE_NGS2_OK);
        std::vector<float> out(panCase.expected.size(), -1.0f);
        Require(sceNgs2PanGetVolumeMatrix(&work, panCase.params.data(), static_cast<std::uint32_t>(panCase.params.size()), panCase.format, out.data()) == SCE_NGS2_OK);
        for (std::size_t i = 0; i < out.size(); i++) Require(std::fabs(out[i] - panCase.expected[i]) < 1e-6f);
    }

    const Ngs2PanParam front{0.0f, 1.0f, 1.0f, 0.0f};
    float out[2] = {};
    Require(sceNgs2PanInit(&work, nullptr, 360.0f, 2) == SCE_NGS2_OK);
    Require(sceNgs2PanGetVolumeMatrix(&work, &front, 1, 2, nullptr) == SCE_NGS2_ERROR_INVALID_OUT_ADDRESS);
    Require(sceNgs2PanGetVolumeMatrix(&work, &front, 0, 2, out) == SCE_NGS2_OK);
    Require(PanThrows(nullptr, &front, 1, 2) && PanThrows(&work, nullptr, 0, 2));
    Require(PanThrows(&work, &front, 1, 4) && PanThrows(&work, &front, 1, 3) && !PanThrows(&work, &front, 1, 8));
    const Ngs2PanParam many[9] = {};
    Require(PanThrows(&work, many, 9, 2) && !PanThrows(&work, many, 8, 2));
    const Ngs2PanParam nanAngle{NAN, 1.0f, 1.0f, 0.0f};
    Require(PanThrows(&work, &nanAngle, 1, 2));
    const Ngs2PanParam laterNan[2] = {front, nanAngle};
    float untouched[4] = {-1.0f, -1.0f, -1.0f, -1.0f};
    bool rejected = false;
    try { sceNgs2PanGetVolumeMatrix(&work, laterNan, 2, 2, untouched); } catch (const std::invalid_argument&) { rejected = true; }
    Require(rejected && std::all_of(std::begin(untouched), std::end(untouched), [](float level) { return level == -1.0f; }));
    const Ngs2PanParam huge{1e30f, 1.0f, 1.0f, 0.0f};
    Require(PanThrows(&work, &huge, 1, 2));
    const float hugeSpeaker[] = {1e30f, 30.0f};
    const float hugeNegative[] = {-1e30f, 30.0f};
    Require(PanInitThrows(&work, hugeSpeaker, 360.0f, 2) && PanInitThrows(&work, hugeNegative, 360.0f, 2));
    Require(sceNgs2PanInit(&work, nullptr, 360.0f, 2) == SCE_NGS2_OK);
    work.speaker_angles[0] = -INFINITY;
    Require(PanThrows(&work, &front, 1, 2));
    Require(sceNgs2PanInit(&work, nullptr, 360.0f, 2) == SCE_NGS2_OK);
    Ngs2PanWork marked;
    std::memset(&marked, 0x5a, sizeof(marked));
    Ngs2PanWork expected;
    std::memcpy(&expected, &marked, sizeof(marked));
    const auto unchanged = [&] { return std::memcmp(&marked, &expected, sizeof(marked)) == 0; };
    const float pair[] = {-30.0f, 30.0f};
    Require(sceNgs2PanInit(nullptr, nullptr, 360.0f, 2) == static_cast<int>(0x804A8010));
    Require(sceNgs2PanInit(nullptr, pair, 180.0f, 3) == static_cast<int>(0x804A8010));
    Require(sceNgs2PanInit(nullptr, nullptr, 100.0f, 3) == static_cast<int>(0x804A8010));
    for (std::uint32_t count : {1u, 3u, 6u, 8u, 255u}) {
        Require(sceNgs2PanInit(&marked, nullptr, 360.0f, count) == static_cast<int>(0x804A8451));
        Require(unchanged());
    }
    Require(PanInitThrows(&marked, nullptr, 360.0f, 0));
    Require(unchanged());
    for (std::uint32_t count : {0u, 1u, 3u, 6u, 8u, 255u}) {
        Require(sceNgs2PanInit(&marked, pair, 360.0f, count) == static_cast<int>(0x804A8451));
        Require(sceNgs2PanInit(&marked, pair, 2.0f * pi, count) == static_cast<int>(0x804A8451));
        Require(unchanged());
    }
    for (const float unitAngle : {180.0f, 0.0f, 359.0f, 361.0f, 90.0f, pi}) {
        Require(sceNgs2PanInit(&marked, pair, unitAngle, 2) == static_cast<int>(0x804A8450));
        Require(sceNgs2PanInit(&marked, pair, unitAngle, 3) == static_cast<int>(0x804A8450));
        Require(unchanged());
    }
    Require(sceNgs2PanInit(&work, pair, 2.0f * pi, 2) == SCE_NGS2_OK);
    const float invalid[] = {0.0f, NAN};
    Require(PanInitThrows(&work, invalid, 360.0f, 2));
    const float onUnit[] = {720.0f, 30.0f};
    const float belowZero[] = {-360.0f, 30.0f};
    Require(PanInitThrows(&work, onUnit, 360.0f, 2) && !PanInitThrows(&work, belowZero, 360.0f, 2));
    work.unit_angle = 100.0f;
    Require(PanThrows(&work, &front, 1, 2));
}

static void TestPitchAndRepeat() {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 1);
    const std::vector<std::int16_t> pcm{0, 8, 16, 24};
    const auto sampler = Sampler(system, pcm, 2);
    Patch(sampler, master);
    Control(sampler, SCE_NGS2_SAMPLER_VOICE_PARAM_PITCH, Ngs2SamplerVoicePitchParam{{}, 0.5f});
    Control(sampler, SCE_NGS2_VOICE_PARAM_CALLBACK,
            Ngs2VoiceCallbackParam{{}, OnBlock, 7, SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_END | SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_REPEAT, 0});
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);

    const auto played = [&](std::uint32_t frame) { return frame < 3 || frame - 3 >= 2 * pcm.size() ? std::int16_t{0} : pcm[(frame - 3) % pcm.size()]; };
    const std::vector<std::uint32_t> callbacksAfter[] = {
        {SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_REPEAT},
        {SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_REPEAT, SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_END},
        {SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_REPEAT, SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_END},
    };
    const float rampAt24000[] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -7.40480618e-06f, 0.0f, 0.000114665498f, 0.000244140625f, 0.000366210938f, 0.00048828125f};
    callbackFlags.clear();
    std::vector<float> out(Grain);
    const Ngs2RenderBufferInfo info{out.data(), out.size() * sizeof(float), SCE_NGS2_WAVEFORM_TYPE_PCM_F32L, 1};
    for (std::uint32_t render = 0; render < 3; render++) {
        Require(sceNgs2SystemRender(system, &info, 1) == SCE_NGS2_OK);
        for (std::uint32_t i = 0; i < Grain; i++) {
            const auto frame = render * Grain + i;
            if (frame < std::size(rampAt24000)) Require(out[i] == rampAt24000[frame]);
            if (frame % 2 == 0) Require(out[i] == played(frame / 2) / 32768.0f);
        }
        Require(callbackFlags == callbacksAfter[render]);
        Require(Flags(sampler) == (render < 2 ? (SCE_NGS2_VOICE_STATE_FLAG_INUSE | SCE_NGS2_VOICE_STATE_FLAG_PLAYING) : 0));
    }
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static void TestSetupClearsCallback() {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 1);
    const std::vector<std::int16_t> pcm{1000, 2000, 3000, 4000};
    const auto sampler = Sampler(system, pcm, 0);
    Control(sampler, SCE_NGS2_VOICE_PARAM_CALLBACK, Ngs2VoiceCallbackParam{{}, OnBlock, 7, SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_END, 0});
    Control(sampler, SCE_NGS2_SAMPLER_VOICE_PARAM_SETUP, Ngs2SamplerVoiceSetupParam{{}, {SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, 1, 48000, 0, 0, 0}});
    const Ngs2WaveformBlock oneShot{0, pcm.size() * sizeof(std::int16_t), 0, 0, static_cast<std::uint32_t>(pcm.size()), 0, 0};
    Control(sampler, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS, Ngs2SamplerVoiceWaveformBlocksParam{{}, pcm.data(), 0, 1, &oneShot});
    Patch(sampler, master);
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);

    callbackFlags.clear();
    auto out = RenderI16(system);
    for (std::uint32_t i = 0; i < Grain; i++) Require(out[i] == Delayed(pcm, i));
    Require(callbackFlags.empty() && Flags(sampler) == 0);

    Control(sampler, SCE_NGS2_SAMPLER_VOICE_PARAM_SETUP, Ngs2SamplerVoiceSetupParam{{}, {SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, 1, 48000, 0, 0, 0}});
    Patch(sampler, master);
    Control(sampler, SCE_NGS2_VOICE_PARAM_CALLBACK, Ngs2VoiceCallbackParam{{}, OnBlock, 7, SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_END, 0});
    const Ngs2WaveformBlock stream{0, pcm.size() * sizeof(std::int16_t), 0, 0, static_cast<std::uint32_t>(pcm.size()), 0, 0x55};
    Control(sampler, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS, Ngs2SamplerVoiceWaveformBlocksParam{{}, pcm.data(), 0, 1, &stream});
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    out = RenderI16(system);
    for (std::uint32_t i = 0; i < Grain; i++) Require(out[i] == Delayed(pcm, i));
    Require(callbackFlags.size() == 1 && callbackFlags[0] == SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_END);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

struct PcmResetCallback {
    std::vector<std::int16_t> pcm;
    std::uint32_t flag;
    int calls = 0;
    std::uint32_t waveformFlags = SCE_NGS2_WAVEFORM_BLOCKS_FLAG_RESET;
    bool pause = false;
};

static void APS5_VABI ResetPcm(const Ngs2VoiceCallbackInfo* info) {
    auto& callback = *reinterpret_cast<PcmResetCallback*>(info->callback_data);
    if (++callback.calls != 1) return;
    Require(info->flag == callback.flag);
    const Ngs2WaveformBlock block{0, callback.pcm.size() * sizeof(std::int16_t), 0, 0, static_cast<std::uint32_t>(callback.pcm.size()), 0, 0};
    Control(info->voice_handle, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS,
            Ngs2SamplerVoiceWaveformBlocksParam{{}, callback.pcm.data(), callback.waveformFlags, 1, &block});
    if (callback.pause) Event(info->voice_handle, SCE_NGS2_VOICE_EVENT_PAUSE);
}

static void TestPcmCallbackReset() {
    for (const auto flag : {SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_END, SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_REPEAT}) {
        const auto system = CreateSystem();
        const std::vector<std::int16_t> pcm{16384};
        const auto sampler = Sampler(system, pcm, flag == SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_REPEAT ? 2 : 0);
        Patch(sampler, Mastering(system, 1));
        PcmResetCallback callback{{}, flag};
        for (std::uint32_t frame = 0; frame < Grain; ++frame) callback.pcm.push_back(static_cast<std::int16_t>(1000 + frame * 100));
        Control(sampler, SCE_NGS2_VOICE_PARAM_CALLBACK,
                Ngs2VoiceCallbackParam{{}, ResetPcm, reinterpret_cast<std::uintptr_t>(&callback), flag, 0});
        Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
        bool unsupported = false;
        try {
            RenderI16(system);
        } catch (const std::runtime_error&) {
            unsupported = true;
        }
        Require(unsupported && callback.calls == 1);
        Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
    }
}

static void TestPcmCallbackRefill() {
    for (const bool pause : {false, true}) {
        const auto system = CreateSystem();
        const auto sampler = Voice(CreateRack(system, SCE_NGS2_RACK_ID_SAMPLER));
        Control(sampler, SCE_NGS2_SAMPLER_VOICE_PARAM_SETUP, Ngs2SamplerVoiceSetupParam{{}, {SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, 1, 48000, 0, 0, 0}});
        Patch(sampler, Mastering(system, 1));
        const std::int16_t pcm = 16384;
        const Ngs2WaveformBlock block{0, sizeof(pcm), 0, 0, 1, 0, 0};
        Control(sampler, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS,
                Ngs2SamplerVoiceWaveformBlocksParam{{}, &pcm, SCE_NGS2_WAVEFORM_BLOCKS_FLAG_CONTINUE, 1, &block});
        PcmResetCallback callback{{}, SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_END, 0, 0, pause};
        for (std::uint32_t frame = 0; frame < Grain * 2; ++frame) callback.pcm.push_back(static_cast<std::int16_t>(1000 + frame * 100));
        Control(sampler, SCE_NGS2_VOICE_PARAM_CALLBACK,
                Ngs2VoiceCallbackParam{{}, ResetPcm, reinterpret_cast<std::uintptr_t>(&callback), callback.flag, 0});
        Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
        const auto stream = [&](std::uint32_t frame) { return frame < 3 ? std::int16_t{0} : frame == 3 ? pcm : callback.pcm[frame - 4]; };
        if (pause) {
            bool unsupported = false;
            try {
                RenderI16(system);
            } catch (const std::runtime_error&) {
                unsupported = true;
            }
            Require(unsupported && callback.calls == 1);
            Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
            continue;
        }
        const auto first = RenderI16(system);
        Require(callback.calls == 1);
        for (std::uint32_t frame = 0; frame < Grain; ++frame) Require(first[frame] == (pause ? 0 : stream(frame)));
        Ngs2SamplerVoiceState state{};
        Require(sceNgs2VoiceGetState(sampler, &state.voice_state, sizeof(state)) == SCE_NGS2_OK);
        const std::uint32_t consumed = pause ? 0 : Grain - 1;
        Require(state.num_decoded_samples == consumed + 1 && state.waveform_data == callback.pcm.data() + consumed);
        if (pause) Event(sampler, SCE_NGS2_VOICE_EVENT_RESUME);
        const auto second = RenderI16(system);
        for (std::uint32_t frame = 0; frame < Grain; ++frame) Require(second[frame] == stream(consumed + 1 + frame));
        Require(callback.calls == 1);
        Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
    }
}

static void APS5_VABI EventFromCallback(const Ngs2VoiceCallbackInfo* info) {
    Event(info->voice_handle, static_cast<std::uint32_t>(info->callback_data));
}

static void TestPcmCallbackEvents() {
    for (const auto event : {SCE_NGS2_VOICE_EVENT_PLAY, SCE_NGS2_VOICE_EVENT_PAUSE, SCE_NGS2_VOICE_EVENT_RESUME}) {
        const auto system = CreateSystem();
        const std::vector<std::int16_t> pcm(Grain - 2, 16384);
        const auto sampler = Sampler(system, pcm, 0);
        Patch(sampler, Mastering(system, 1));
        Control(sampler, SCE_NGS2_VOICE_PARAM_CALLBACK, Ngs2VoiceCallbackParam{{}, EventFromCallback, event, SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_END, 0});
        Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
        if (event == SCE_NGS2_VOICE_EVENT_RESUME) Event(sampler, SCE_NGS2_VOICE_EVENT_PAUSE);
        bool unsupported = false;
        try {
            RenderI16(system);
        } catch (const std::runtime_error&) {
            unsupported = true;
        }
        Require(unsupported);
        Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
    }
}

static void APS5_VABI PlayFromAnotherThread(const Ngs2VoiceCallbackInfo* info) {
    auto& calls = *reinterpret_cast<int*>(info->callback_data);
    if (++calls != 1) return;
    std::thread([info] { Event(info->voice_handle, SCE_NGS2_VOICE_EVENT_PLAY); }).join();
}

static void TestPcmCallbackOtherThread() {
    const auto system = CreateSystem();
    const std::vector<std::int16_t> pcm(Grain - 2, 16384);
    const auto sampler = Sampler(system, pcm, 0);
    Patch(sampler, Mastering(system, 1));
    int calls = 0;
    Control(sampler, SCE_NGS2_VOICE_PARAM_CALLBACK, Ngs2VoiceCallbackParam{{}, PlayFromAnotherThread, reinterpret_cast<std::uintptr_t>(&calls), SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_END, 0});
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    RenderI16(system);
    Require(calls >= 1);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

struct PcmSetupCallback {
    uintptr_t master;
    std::uint32_t channels;
    std::uint32_t flag;
    std::vector<std::int16_t> pcm;
    int calls = 0;
};

static void APS5_VABI SetupPcm(const Ngs2VoiceCallbackInfo* info) {
    auto& callback = *reinterpret_cast<PcmSetupCallback*>(info->callback_data);
    if (++callback.calls != 1) return;
    Require(info->flag == callback.flag);
    Control(info->voice_handle, SCE_NGS2_SAMPLER_VOICE_PARAM_SETUP,
            Ngs2SamplerVoiceSetupParam{{}, {SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, callback.channels, 48000, 0, 0, 0}});
    const Ngs2WaveformBlock block{0, callback.pcm.size() * sizeof(std::int16_t), 0, 0, Grain * 2, 0, 0};
    Control(info->voice_handle, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS,
            Ngs2SamplerVoiceWaveformBlocksParam{{}, callback.pcm.data(), 0, 1, &block});
    Patch(info->voice_handle, callback.master);
    Event(info->voice_handle, SCE_NGS2_VOICE_EVENT_PLAY);
}

static void TestPcmCallbackSetup() {
    for (const auto channels : {2u, 8u}) {
        for (const auto flag : {SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_END, SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_REPEAT}) {
            const auto system = CreateSystem();
            const std::vector<std::int16_t> pcm{16384};
            const auto sampler = Sampler(system, pcm, flag == SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_REPEAT ? 2 : 0);
            PcmSetupCallback callback{Mastering(system, channels), channels, flag, {}};
            for (std::uint32_t frame = 0; frame < Grain * 2; ++frame) {
                for (std::uint32_t channel = 0; channel < channels; ++channel) callback.pcm.push_back(static_cast<std::int16_t>(1000 + frame * 25 + channel * 100));
            }
            Patch(sampler, callback.master);
            Control(sampler, SCE_NGS2_VOICE_PARAM_CALLBACK,
                    Ngs2VoiceCallbackParam{{}, SetupPcm, reinterpret_cast<std::uintptr_t>(&callback), flag, 0});
            Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
            std::vector<std::int16_t> output(Grain * channels, -1);
            const Ngs2RenderBufferInfo info{output.data(), output.size() * sizeof(std::int16_t), SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, channels};
            Require(sceNgs2SystemRender(system, &info, 1) == SCE_NGS2_OK);
            Require(callback.calls == 1 && std::all_of(output.begin(), output.end(), [](std::int16_t sample) { return sample == 0; }));
            Ngs2SamplerVoiceState state{};
            Require(sceNgs2VoiceGetState(sampler, &state.voice_state, sizeof(state)) == SCE_NGS2_OK);
            Require(state.num_decoded_samples == 0 && state.decoded_data_size == 0 && state.waveform_data == callback.pcm.data());
            Require(sceNgs2SystemRender(system, &info, 1) == SCE_NGS2_OK);
            Require(callback.calls == 1);
            for (std::uint32_t frame = 0; frame < Grain; ++frame) {
                for (std::uint32_t channel = 0; channel < channels; ++channel) {
                    Require(output[frame * channels + channel] == (frame < 3 ? 0 : callback.pcm[(frame - 3) * channels + channel]));
                }
            }
            Require(sceNgs2VoiceGetState(sampler, &state.voice_state, sizeof(state)) == SCE_NGS2_OK);
            Require(state.num_decoded_samples == Grain && state.decoded_data_size == Grain * channels * sizeof(std::int16_t));
            Require(state.waveform_data == callback.pcm.data() + Grain * channels);
            Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
        }
    }
}

static void TestSubmixerMatrix() {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 2);
    const auto submixer = Voice(CreateRack(system, SCE_NGS2_RACK_ID_SUBMIXER));
    Control(submixer, SCE_NGS2_SUBMIXER_VOICE_PARAM_SETUP, Ngs2SubmixerVoiceSetupParam{{}, 2, 0});
    Patch(submixer, master);
    const Ngs2VoiceCommand play{2, 0, 4, 0, {.u = SCE_NGS2_VOICE_EVENT_PLAY}};
    Require(sceNgs2VoiceRunCommands(submixer, &play, 1) == SCE_NGS2_OK);

    const std::vector<std::int16_t> pcm(Grain, 16384);
    const auto sampler = Sampler(system, pcm, 0);
    Patch(sampler, submixer);
    const float levels[2] = {1.0f, 0.5f};
    Control(sampler, SCE_NGS2_VOICE_PARAM_MATRIX_LEVELS, Ngs2VoiceMatrixLevelsParam{{}, 0, 2, levels});
    Control(sampler, SCE_NGS2_VOICE_PARAM_PORT_MATRIX, Ngs2VoicePortMatrixParam{{}, 0, 0});
    Control(sampler, SCE_NGS2_VOICE_PARAM_PORT_VOLUME, Ngs2VoicePortVolumeParam{{}, 0, 0.5f});
    Require(sceNgs2VoiceRunCommands(sampler, &play, 1) == SCE_NGS2_OK);

    std::vector<float> out(Grain * 2, -1.0f);
    const Ngs2RenderBufferInfo info{out.data(), out.size() * sizeof(float), SCE_NGS2_WAVEFORM_TYPE_PCM_F32L, 2};
    Require(sceNgs2SystemRender(system, &info, 1) == SCE_NGS2_OK);
    for (std::uint32_t i = 0; i < Grain; i++) Require(out[i * 2] == (i < 3 ? 0.0f : 0.25f) && out[i * 2 + 1] == (i < 3 ? 0.0f : 0.125f));

    Ngs2SubmixerVoiceState state{};
    Require(sceNgs2VoiceGetState(submixer, &state.voice_state, sizeof(state)) == SCE_NGS2_OK);
    Require(state.voice_state.state_flags == (SCE_NGS2_VOICE_STATE_FLAG_INUSE | SCE_NGS2_VOICE_STATE_FLAG_PLAYING));

    Require(sceNgs2SystemRender(system, &info, 1) == SCE_NGS2_OK);
    for (std::uint32_t i = 0; i < Grain; i++) Require(out[i * 2] == (i < 3 ? 0.25f : 0.0f) && out[i * 2 + 1] == (i < 3 ? 0.125f : 0.0f));
    Require(sceNgs2SystemRender(system, &info, 1) == SCE_NGS2_OK);
    for (float sample : out) Require(sample == 0.0f);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static std::vector<float> MatrixLevelFrame(const std::vector<float>& levels, bool command) {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 2);
    const std::vector<std::int16_t> pcm(Grain, 16384);
    const auto sampler = Sampler(system, pcm, 0);
    Patch(sampler, master);
    if (command) {
        const Ngs2VoiceCommand set{5, 0, 0x11, static_cast<std::uint16_t>(levels.size()), {.levels = levels.data()}};
        Require(sceNgs2VoiceRunCommands(sampler, &set, 1) == SCE_NGS2_OK);
    } else {
        Control(sampler, SCE_NGS2_VOICE_PARAM_MATRIX_LEVELS, Ngs2VoiceMatrixLevelsParam{{}, 0, static_cast<std::uint32_t>(levels.size()), levels.data()});
    }
    Control(sampler, SCE_NGS2_VOICE_PARAM_PORT_MATRIX, Ngs2VoicePortMatrixParam{{}, 0, 0});
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    std::vector<float> out(Grain * 2, -1.0f);
    const Ngs2RenderBufferInfo info{out.data(), out.size() * sizeof(float), SCE_NGS2_WAVEFORM_TYPE_PCM_F32L, 2};
    Require(sceNgs2SystemRender(system, &info, 1) == SCE_NGS2_OK);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
    return {out[6], out[7]};
}

static void TestMatrixLevelClamp() {
    const float inf = INFINITY;
    for (bool command : {false, true}) {
        Require(MatrixLevelFrame({4.0f, -4.0f}, command) == (std::vector<float>{2.0f, -2.0f}));
        Require(MatrixLevelFrame({4.0001f, -4.5f}, command) == (std::vector<float>{2.0f, -2.0f}));
        Require(MatrixLevelFrame({100.0f, -100.0f}, command) == (std::vector<float>{2.0f, -2.0f}));
        Require(MatrixLevelFrame({inf, -inf}, command) == (std::vector<float>{2.0f, -2.0f}));
        Require(MatrixLevelFrame({3.5f, -0.25f}, command) == (std::vector<float>{1.75f, -0.125f}));
        const auto nan = MatrixLevelFrame({NAN, 0.5f}, command);
        Require(std::isnan(nan[0]) && nan[1] == 0.25f);
    }

    const auto system = CreateSystem();
    const std::vector<std::int16_t> silence(Grain, 0);
    const auto sampler = Sampler(system, silence, 0);
    const float levels[2] = {1.0f, 1.0f};
    bool empty = false;
    try { Control(sampler, SCE_NGS2_VOICE_PARAM_MATRIX_LEVELS, Ngs2VoiceMatrixLevelsParam{{}, 0, 0, levels}); } catch (const std::invalid_argument&) { empty = true; }
    Require(empty);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static int allocations = 0;
static std::int32_t APS5_VABI Allocate(Ngs2ContextBufferInfo* info) {
    Require(info->host_buffer == nullptr && info->host_buffer_size != 0 && info->user_data == 9);
    info->host_buffer = std::calloc(1, info->host_buffer_size);
    allocations++;
    return SCE_NGS2_OK;
}
static std::int32_t APS5_VABI Release(Ngs2ContextBufferInfo* info) {
    Require(info->host_buffer != nullptr && info->user_data == 9);
    std::free(info->host_buffer);
    allocations--;
    return SCE_NGS2_OK;
}

struct ReverbFrame {
    std::uint32_t frame;
    std::vector<float> samples;
};

struct ReverbCase {
    std::uint32_t channels;
    float amplitude;
    bool noise;
    Ngs2ReverbI3DL2Param params;
    std::int32_t changeGrain;
    Ngs2ReverbI3DL2Param change;
    std::vector<ReverbFrame> expected;
};

static std::vector<float> RenderReverb(const ReverbCase& reverbCase, std::uint32_t frames) {
    const auto firstBuffer = usedBuffers;
    const auto system = CreateSystem();
    Require(sceNgs2SystemSetGrainSamples(system, 256) == SCE_NGS2_OK);
    const auto channels = reverbCase.channels;
    const auto master = Mastering(system, channels);
    const auto reverb = Voice(CreateRack(system, SCE_NGS2_RACK_ID_REVERB));
    Control(reverb, SCE_NGS2_REVERB_VOICE_PARAM_SETUP, Ngs2ReverbVoiceSetupParam{{}, channels, channels, 0, 0});
    Control(reverb, SCE_NGS2_REVERB_VOICE_PARAM_I3DL2, Ngs2ReverbVoiceI3DL2Param{{}, reverbCase.params});
    Patch(reverb, master);
    Event(reverb, SCE_NGS2_VOICE_EVENT_PLAY);
    std::vector<std::int16_t> pcm(4096 * channels, 0);
    std::uint32_t seed = 1;
    for (std::uint32_t i = 0; reverbCase.noise && i + 3 * channels < pcm.size(); i++) {
        seed = seed * 1664525u + 1013904223u;
        if (i < 3000 * channels) pcm[i] = static_cast<std::int16_t>(static_cast<std::int32_t>((seed >> 16) & 0x3fff) - 0x2000);
    }
    for (std::uint32_t c = 0; !reverbCase.noise && c < channels; c++) pcm[c] = static_cast<std::int16_t>((16384 - 4000 * static_cast<std::int32_t>(c)) * reverbCase.amplitude);
    const auto sampler = Voice(CreateRack(system, SCE_NGS2_RACK_ID_SAMPLER));
    Control(sampler, SCE_NGS2_SAMPLER_VOICE_PARAM_SETUP, Ngs2SamplerVoiceSetupParam{{}, {SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, channels, 48000, 0, 0, 0}});
    const Ngs2WaveformBlock block{0, pcm.size() * sizeof(std::int16_t), 0, 0, 4096, 0, 0};
    Control(sampler, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS, Ngs2SamplerVoiceWaveformBlocksParam{{}, pcm.data(), 0, 1, &block});
    Patch(sampler, reverb);
    std::vector<float> identity(channels * channels, 0.0f);
    for (std::uint32_t c = 0; c < channels; c++) identity[c * channels + c] = 1.0f;
    Control(sampler, SCE_NGS2_VOICE_PARAM_MATRIX_LEVELS, Ngs2VoiceMatrixLevelsParam{{}, 0, channels * channels, identity.data()});
    Control(sampler, SCE_NGS2_VOICE_PARAM_PORT_MATRIX, Ngs2VoicePortMatrixParam{{}, 0, 0});
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    std::vector<float> all;
    std::vector<float> out(256 * channels);
    for (std::uint32_t grain = 0; grain * 256 < frames; grain++) {
        if (static_cast<std::int32_t>(grain) == reverbCase.changeGrain) Control(reverb, SCE_NGS2_REVERB_VOICE_PARAM_I3DL2, Ngs2ReverbVoiceI3DL2Param{{}, reverbCase.change});
        const Ngs2RenderBufferInfo info{out.data(), out.size() * sizeof(float), SCE_NGS2_WAVEFORM_TYPE_PCM_F32L, channels};
        Require(sceNgs2SystemRender(system, &info, 1) == SCE_NGS2_OK);
        all.insert(all.end(), out.begin(), out.end());
    }
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
    usedBuffers = firstBuffer;
    return all;
}

static void TestReverb() {
    const Ngs2ReverbI3DL2Param cave{1.0f, 0.0f, -1000, 0, 0, 2.91f, 1.3f, -602, 0.015f, -302, 0.022f, 100.0f, 100.0f, 5000.0f, {}};
    auto changed = cave;
    changed.reflection_pattern = 5;
    changed.decay_time = 0.7f;
    changed.decay_hf_ratio = 0.6f;
    changed.reflections_delay = 0.004f;
    changed.reverb_delay = 0.03f;
    changed.room_hf = -900;
    changed.wet = 0.6f;
    changed.dry = 0.4f;
    auto denser = cave;
    denser.density = 99.999f;
    const std::vector<ReverbCase> cases = {
        {2, 1.0f, false, cave, -1, {}, {{100, {0, 0}}, {723, {0.0687032491f, 0.0687032491f}}, {1055, {0.0392590016f, 0.0392590016f}}, {1417, {0.0131517649f, 0.0131517649f}},
            {1779, {0.0098147504f, 0.0098147504f}}, {2500, {0, 0}}, {4000, {1.45101382e-07f, -1.15403088e-06f}}, {6000, {0.000655628915f, 3.03641864e-05f}},
            {9000, {0.000400578661f, -0.000684358703f}}, {12000, {0.000191548112f, 0.000220489033f}}, {16000, {0.000210488462f, 0.000146183695f}},
            {20000, {2.41150738e-05f, -0.000864534522f}}, {26000, {0.000125532999f, -0.000234790568f}}, {32000, {-0.000462158496f, 4.56995949e-05f}}}},
        {2, 1.0f, false, {1.0f, 0.0f, -500, -800, 3, 1.2f, 0.5f, -300, 0.01f, 0, 0.03f, 80.0f, 60.0f, 3000.0f, {}}, -1, {},
            {{100, {0, 0}}, {723, {0.000166983227f, 0.000166983227f}}, {1055, {-3.41919585e-05f, -3.41919585e-05f}}, {1417, {0, 0}}, {1779, {0, 0}},
            {2500, {0, 1.55222626e-08f}}, {4000, {-0.000106364969f, 9.90879998e-05f}}, {6000, {0.00013573296f, 7.22560799e-06f}},
            {9000, {-7.91876082e-05f, 0.000225067721f}}, {12000, {0.000150469248f, 0.00036288987f}}, {16000, {-7.86288219e-05f, -0.000195921995f}},
            {20000, {1.01910318e-05f, 5.79599109e-05f}}, {26000, {2.14232614e-06f, -2.05506512e-05f}}, {32000, {-4.31467561e-06f, 2.34100444e-05f}}}},
        {6, 1.0f, false, {1.0f, 0.2f, -200, -300, 12, 1.0f, 0.9f, -100, 0.03f, -100, 0.04f, 70.0f, 90.0f, 4000.0f, {}}, -1, {},
            {{6000, {-0.000167909253f, -0.000179917421f, -0.000647808949f, 0, 3.22720189e-05f, 0.000213803389f}},
            {9000, {8.47857882e-05f, 8.62322413e-05f, -0.000100753634f, 0, 0.000109430795f, -0.000122895101f}},
            {12000, {4.37544441e-05f, 3.58103216e-06f, -0.00013746327f, 0, 9.3237948e-05f, -0.00013529828f}},
            {16000, {8.63046807e-08f, 1.62214419e-05f, 8.89167641e-05f, 0, 1.02178528e-05f, 0.000201705057f}},
            {20000, {-9.6526619e-06f, -2.73812566e-05f, -1.92096536e-07f, 0, -0.000116049101f, 6.94858772e-06f}},
            {26000, {1.27750207e-06f, -1.59169917e-06f, 5.30674515e-05f, 0, -3.59285696e-05f, 5.16466662e-06f}},
            {32000, {3.5737969e-06f, 4.19655044e-06f, -2.30460955e-07f, 0, 1.16596939e-05f, 1.14889262e-05f}}}},
        {2, 1.0f, false, cave, 6, changed, {{723, {0.0687032491f, 0.0687032491f}}, {1055, {0.0392590016f, 0.0392590016f}}, {1417, {0.0131517649f, 0.0131517649f}},
            {1779, {4.38156894e-05f, 4.38156894e-05f}}, {2500, {0, 0}}, {4000, {-1.60370582e-05f, -4.44955149e-05f}}, {6000, {1.02755057e-05f, 8.34046841e-06f}},
            {9000, {1.1641363e-05f, -4.69685165e-06f}}, {12000, {3.57917179e-06f, -2.34550089e-06f}}, {16000, {-8.00212234e-08f, 3.19533115e-06f}},
            {20000, {5.66619519e-07f, 4.26969848e-07f}}, {26000, {1.46052287e-07f, -2.44572629e-08f}}, {32000, {-4.45668853e-08f, 1.51349102e-08f}}}},
        {2, 0.02f, false, {40.0f, -3.0f, 5000, 500, 20, 50.0f, 5.0f, 3000, 0.9f, 5000, 0.9f, 300.0f, 300.0f, 50000.0f, {}}, -1, {},
            {{14403, {0.877262115f, 0.877262115f}}, {14500, {0, 0}}, {15000, {0, 0}}, {15500, {0, 7.50572099e-07f}}, {16000, {4.95207075e-07f, 6.37215771e-06f}},
            {20000, {0.00482429564f, -0.00149071764f}}, {26000, {-0.0096597122f, 0.00718426611f}}, {32000, {0.0107425917f, -0.000738018774f}}}},
        {2, 1.0f, false, {1.0f, 0.3f, 0, -200, 15, 0.8f, 1.0f, 0, 0.002f, -500, 0.004f, 60.0f, 80.0f, 6000.0f, {}}, -1, {},
            {{60, {0, 0}}, {99, {0.0947210863f, 0.0947210863f}}, {100, {0.0472047739f, 0.0472047739f}}, {150, {0, 0}}, {200, {0, 0}}, {300, {0, 0}},
            {2000, {-9.86956729e-06f, 7.96923473e-07f}}, {3000, {-0.000152820328f, -4.59704825e-06f}}, {5000, {0.000203833857f, -0.000208990226f}},
            {8000, {-3.69777154e-05f, -0.000259475899f}}}},
        {2, 1.0f, true, cave, 6, changed, {{1540, {0.0222691782f, 0.0242232569f}}, {1600, {0.0201471522f, 0.0186609477f}}, {1700, {-0.0211425349f, -0.0235042125f}},
            {1780, {-0.0168117173f, 0.00417749817f}}, {1800, {-0.0866222829f, -0.0945812687f}}, {2000, {0.0131965755f, -0.0707511827f}},
            {2500, {0.0942443311f, -0.100531064f}}, {3500, {0.000985965831f, 0.000946713379f}}, {8000, {-0.000968122098f, 0.000556072395f}},
            {15000, {0.000227724231f, 0.000151095577f}}}},
        {2, 1.0f, true, cave, 20, denser, {{5200, {-0.00942458585f, -0.0064059021f}}, {6000, {0.00248076022f, 0.00317249796f}},
            {8000, {0.00102439919f, 0.00105210941f}}, {12000, {1.63059376e-05f, 6.10309735e-06f}}, {20000, {-1.08853078e-06f, -2.99162899e-08f}}}},
    };
    for (const auto& reverbCase : cases) {
        const auto out = RenderReverb(reverbCase, 32256);
        for (const auto& expected : reverbCase.expected) {
            for (std::uint32_t c = 0; c < reverbCase.channels; c++) {
                if (c == 3) continue;
                const float value = out[expected.frame * reverbCase.channels + c];
                Require(std::fabs(value - expected.samples[c]) <= 2e-7f + std::fabs(expected.samples[c]) * 1e-5f);
            }
        }
    }
}

static void TestReverbSetup() {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 2);
    const auto reverb = Voice(CreateRack(system, SCE_NGS2_RACK_ID_REVERB));
    for (std::uint32_t channels : {3u, 4u, 5u, 7u}) {
        bool rejected = false;
        try { Control(reverb, SCE_NGS2_REVERB_VOICE_PARAM_SETUP, Ngs2ReverbVoiceSetupParam{{}, channels, channels, 0, 0}); } catch (const std::invalid_argument&) { rejected = true; }
        Require(rejected);
    }
    bool conversion = false;
    try { Control(reverb, SCE_NGS2_REVERB_VOICE_PARAM_SETUP, Ngs2ReverbVoiceSetupParam{{}, 1, 2, 0, 0}); } catch (const std::runtime_error&) { conversion = true; }
    Require(conversion);
    bool flags = false;
    try { Control(reverb, SCE_NGS2_REVERB_VOICE_PARAM_SETUP, Ngs2ReverbVoiceSetupParam{{}, 2, 2, 1, 0}); } catch (const std::runtime_error&) { flags = true; }
    Require(flags);
    Control(reverb, SCE_NGS2_REVERB_VOICE_PARAM_SETUP, Ngs2ReverbVoiceSetupParam{{}, 2, 2, 0, 0});
    Ngs2ReverbI3DL2Param invalid{1.0f, 0.0f, 0, 0, 0, 1.0f, 1.0f, 0, 0.0f, 0, 0.0f, 100.0f, 100.0f, 5000.0f, {}};
    invalid.decay_time = NAN;
    bool nan = false;
    try { Control(reverb, SCE_NGS2_REVERB_VOICE_PARAM_I3DL2, Ngs2ReverbVoiceI3DL2Param{{}, invalid}); } catch (const std::invalid_argument&) { nan = true; }
    Require(nan);
    Patch(reverb, master);
    Event(reverb, SCE_NGS2_VOICE_EVENT_PLAY);
    const std::vector<std::int16_t> pcm(Grain * 64, 16384);
    const auto sampler = Sampler(system, pcm, 0);
    Patch(sampler, reverb);
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    std::vector<float> out(Grain * 2);
    const Ngs2RenderBufferInfo info{out.data(), out.size() * sizeof(float), SCE_NGS2_WAVEFORM_TYPE_PCM_F32L, 2};
    float loudest = 0.0f;
    for (std::uint32_t grain = 0; grain < 600; grain++) {
        Require(sceNgs2SystemRender(system, &info, 1) == SCE_NGS2_OK);
        for (float sample : out) loudest = std::max(loudest, std::fabs(sample));
    }
    Require(loudest > 0.0f && loudest < 1e-8f);
    Ngs2VoiceState state{};
    Require(sceNgs2VoiceGetState(reverb, &state, sizeof(state)) == SCE_NGS2_OK && (state.state_flags & SCE_NGS2_VOICE_STATE_FLAG_INUSE) != 0);
    Require(sceNgs2SystemSetSampleRate(system, 7000) == SCE_NGS2_ERROR_INVALID_SAMPLE_RATE);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);

    Ngs2ReverbRackOption option{};
    option.rack_option.size = sizeof(option);
    option.rack_option.max_grain_samples = 512;
    option.rack_option.max_voices = 1;
    option.rack_option.max_input_delay_blocks = 1;
    option.rack_option.max_matrices = 1;
    option.rack_option.max_ports = 8;
    option.max_channels = 8;
    option.reverb_size = 1;
    Ngs2ContextBufferInfo query{};
    Require(sceNgs2RackQueryBufferSize(SCE_NGS2_RACK_ID_REVERB, &option.rack_option, &query) == SCE_NGS2_OK);
    for (std::uint32_t size : {0u, 2u}) {
        option.reverb_size = size;
        bool unsupported = false;
        try { sceNgs2RackQueryBufferSize(SCE_NGS2_RACK_ID_REVERB, &option.rack_option, &query); } catch (const std::runtime_error&) { unsupported = true; }
        Require(unsupported);
    }
}

static void TestSampleRate() {
    const auto system = CreateSystem();
    Require(sceNgs2SystemSetSampleRate(0x1234, 96000) == SCE_NGS2_ERROR_INVALID_SYSTEM_HANDLE);
    Require(sceNgs2SystemSetSampleRate(system, 0) == SCE_NGS2_ERROR_INVALID_SAMPLE_RATE);
    Require(sceNgs2SystemSetSampleRate(system, 96000) == SCE_NGS2_OK);
    Ngs2SystemInfo info{};
    Require(sceNgs2SystemGetInfo(system, &info, sizeof(info)) == SCE_NGS2_OK && info.sample_rate == 96000);

    const auto master = Mastering(system, 1);
    const std::vector<std::int16_t> pcm{0, 1000, 2000, 3000};
    const auto sampler = Sampler(system, pcm, 1);
    Patch(sampler, master);
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    std::vector<std::int16_t> out(Grain);
    const Ngs2RenderBufferInfo renderInfo{out.data(), out.size() * sizeof(std::int16_t), SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, 1};
    Require(sceNgs2SystemRender(system, &renderInfo, 1) == SCE_NGS2_OK);
    const auto halfSystem = CreateSystem();
    const auto halfMaster = Mastering(halfSystem, 1);
    const auto halfSampler = Sampler(halfSystem, pcm, 1);
    Control(halfSampler, SCE_NGS2_SAMPLER_VOICE_PARAM_PITCH, Ngs2SamplerVoicePitchParam{{}, 0.5f});
    Patch(halfSampler, halfMaster);
    Event(halfSampler, SCE_NGS2_VOICE_EVENT_PLAY);
    Require(out == RenderI16(halfSystem) && out != std::vector<std::int16_t>(Grain, 0));
    Require(sceNgs2SystemDestroy(halfSystem, nullptr) == SCE_NGS2_OK);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static bool SystemOptionThrows(const Ngs2SystemOption& option) {
    Ngs2ContextBufferInfo query{};
    try {
        sceNgs2SystemQueryBufferSize(&option, &query);
    } catch (const std::runtime_error&) {
        return true;
    }
    return false;
}

static void TestSystemOptionFlags() {
    Ngs2SystemOption option{};
    Require(sceNgs2SystemResetOption(&option) == SCE_NGS2_OK && option.flags == 0);
    option.flags = 0x2;
    Ngs2ContextBufferInfo query{};
    Require(sceNgs2SystemQueryBufferSize(&option, &query) == SCE_NGS2_OK && query.host_buffer_size != 0);
    const auto info = Buffer(query);
    uintptr_t system = 0;
    Require(sceNgs2SystemCreate(&option, &info, &system) == SCE_NGS2_OK && system != 0);
    Require(sceNgs2SystemSetGrainSamples(system, Grain) == SCE_NGS2_OK);
    const auto master = Mastering(system, 1);
    const std::vector<std::int16_t> pcm{1000, 2000, 3000, 4000};
    const auto sampler = Sampler(system, pcm, 1);
    Patch(sampler, master);
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    const auto out = RenderI16(system);
    for (std::uint32_t i = 0; i < Grain; i++) Require(out[i] == Delayed(pcm, i));
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);

    for (const std::uint32_t flags : {0x1u, 0x3u, 0x4u, 0x80000000u, 0x80000002u}) {
        option.flags = flags;
        Require(SystemOptionThrows(option));
    }
}

static void TestUserData() {
    uintptr_t value = 1;
    Require(sceNgs2SystemSetUserData(0x1234, 5) == SCE_NGS2_ERROR_INVALID_SYSTEM_HANDLE);
    Require(sceNgs2SystemGetUserData(0x1234, &value) == SCE_NGS2_ERROR_INVALID_SYSTEM_HANDLE && value == 1);
    const auto system = CreateSystem();
    Require(sceNgs2SystemGetUserData(system, &value) == SCE_NGS2_OK && value == 0);
    Require(sceNgs2SystemSetUserData(system, 0xfedcba9876543210) == SCE_NGS2_OK);
    Require(sceNgs2SystemGetUserData(system, &value) == SCE_NGS2_OK && value == 0xfedcba9876543210);
    bool threw = false;
    try {
        sceNgs2SystemGetUserData(system, nullptr);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    Require(threw);

    Require(sceNgs2RackSetUserData(0x1234, 5) == SCE_NGS2_ERROR_INVALID_RACK_HANDLE);
    Require(sceNgs2RackSetUserData(system, 5) == SCE_NGS2_ERROR_INVALID_RACK_HANDLE);
    value = 1;
    Require(sceNgs2RackGetUserData(0x1234, &value) == SCE_NGS2_ERROR_INVALID_RACK_HANDLE && value == 1);
    Require(sceNgs2RackGetUserData(system, &value) == SCE_NGS2_ERROR_INVALID_RACK_HANDLE && value == 1);
    Require(sceNgs2RackGetUserData(0x1234, nullptr) == SCE_NGS2_ERROR_INVALID_RACK_HANDLE);
    const auto rack = CreateRack(system, SCE_NGS2_RACK_ID_SAMPLER);
    const auto other = CreateRack(system, SCE_NGS2_RACK_ID_MASTERING);
    Require(sceNgs2RackGetUserData(rack, &value) == SCE_NGS2_OK && value == 0);
    Require(sceNgs2RackSetUserData(rack, 0x0123456789abcdef) == SCE_NGS2_OK);
    Require(sceNgs2RackGetUserData(rack, &value) == SCE_NGS2_OK && value == 0x0123456789abcdef);
    Require(sceNgs2RackGetUserData(other, &value) == SCE_NGS2_OK && value == 0);
    Require(sceNgs2SystemGetUserData(system, &value) == SCE_NGS2_OK && value == 0xfedcba9876543210);
    Require(sceNgs2RackSetUserData(other, 0) == SCE_NGS2_OK);
    Require(sceNgs2RackGetUserData(rack, &value) == SCE_NGS2_OK && value == 0x0123456789abcdef);
    threw = false;
    try {
        sceNgs2RackGetUserData(rack, nullptr);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    Require(threw);
    Require(sceNgs2RackDestroy(rack, nullptr) == SCE_NGS2_OK);
    value = 1;
    Require(sceNgs2RackGetUserData(rack, &value) == SCE_NGS2_ERROR_INVALID_RACK_HANDLE && value == 1);
    Require(sceNgs2RackSetUserData(rack, 5) == SCE_NGS2_ERROR_INVALID_RACK_HANDLE);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static void TestMasteringGain() {
    const auto system = CreateSystem();
    const auto stereo = Mastering(system, 2);
    const auto surround = Mastering(system, 6);
    const auto quad = Mastering(system, 4);
    Control(stereo, SCE_NGS2_MASTERING_VOICE_PARAM_GAIN, Ngs2MasteringVoiceGainParam{{}, 0.5f, 1.0f});
    Control(surround, SCE_NGS2_MASTERING_VOICE_PARAM_GAIN, Ngs2MasteringVoiceGainParam{{}, 0.5f, 0.25f});
    Control(surround, SCE_NGS2_MASTERING_VOICE_PARAM_OUTPUT, Ngs2MasteringVoiceOutputParam{{}, 1});
    Control(quad, SCE_NGS2_MASTERING_VOICE_PARAM_GAIN, Ngs2MasteringVoiceGainParam{{}, 0.5f, 0.25f});
    Control(quad, SCE_NGS2_MASTERING_VOICE_PARAM_OUTPUT, Ngs2MasteringVoiceOutputParam{{}, 2});
    const std::vector<std::int16_t> pcm(Grain, 16384);
    const auto toStereo = Sampler(system, pcm, 0);
    const float stereoLevels[2] = {1.0f, 0.5f};
    Control(toStereo, SCE_NGS2_VOICE_PARAM_MATRIX_LEVELS, Ngs2VoiceMatrixLevelsParam{{}, 0, 2, stereoLevels});
    Control(toStereo, SCE_NGS2_VOICE_PARAM_PORT_MATRIX, Ngs2VoicePortMatrixParam{{}, 0, 0});
    Patch(toStereo, stereo);
    Event(toStereo, SCE_NGS2_VOICE_EVENT_PLAY);
    const auto toSurround = Sampler(system, pcm, 0);
    const float surroundLevels[6] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    Control(toSurround, SCE_NGS2_VOICE_PARAM_MATRIX_LEVELS, Ngs2VoiceMatrixLevelsParam{{}, 0, 6, surroundLevels});
    Control(toSurround, SCE_NGS2_VOICE_PARAM_PORT_MATRIX, Ngs2VoicePortMatrixParam{{}, 0, 0});
    Patch(toSurround, surround);
    Event(toSurround, SCE_NGS2_VOICE_EVENT_PLAY);
    const auto toQuad = Sampler(system, pcm, 0);
    Control(toQuad, SCE_NGS2_VOICE_PARAM_MATRIX_LEVELS, Ngs2VoiceMatrixLevelsParam{{}, 0, 4, surroundLevels});
    Control(toQuad, SCE_NGS2_VOICE_PARAM_PORT_MATRIX, Ngs2VoicePortMatrixParam{{}, 0, 0});
    Patch(toQuad, quad);
    Event(toQuad, SCE_NGS2_VOICE_EVENT_PLAY);
    bool rejected = false;
    try { Control(stereo, SCE_NGS2_MASTERING_VOICE_PARAM_GAIN, Ngs2MasteringVoiceGainParam{{}, NAN, 1.0f}); } catch (const std::exception&) { rejected = true; }
    Require(rejected);

    std::vector<float> outStereo(Grain * 2, -1.0f);
    std::vector<float> outSurround(Grain * 6, -1.0f);
    std::vector<float> outQuad(Grain * 4, -1.0f);
    const Ngs2RenderBufferInfo info[3] = {
        {outStereo.data(), outStereo.size() * sizeof(float), SCE_NGS2_WAVEFORM_TYPE_PCM_F32L, 2},
        {outSurround.data(), outSurround.size() * sizeof(float), SCE_NGS2_WAVEFORM_TYPE_PCM_F32L, 6},
        {outQuad.data(), outQuad.size() * sizeof(float), SCE_NGS2_WAVEFORM_TYPE_PCM_F32L, 4},
    };
    Require(sceNgs2SystemRender(system, info, 3) == SCE_NGS2_OK);
    for (std::uint32_t i = 0; i < Grain; i++) {
        const float delayed = i < 3 ? 0.0f : 1.0f;
        Require(outStereo[i * 2] == 0.25f * delayed && outStereo[i * 2 + 1] == 0.125f * delayed);
        for (std::uint32_t channel = 0; channel < 6; channel++) Require(outSurround[i * 6 + channel] == (channel == 3 ? 0.125f : 0.25f) * delayed);
        for (std::uint32_t channel = 0; channel < 4; channel++) Require(outQuad[i * 4 + channel] == 0.25f * delayed);
    }
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);

    const auto loudSystem = CreateSystem();
    const auto loud = Mastering(loudSystem, 1);
    Control(loud, SCE_NGS2_MASTERING_VOICE_PARAM_GAIN, Ngs2MasteringVoiceGainParam{{}, 2.0f, 2.0f});
    const std::vector<std::int16_t> loudPcm(Grain * 2, 24576);
    const auto toLoud = Sampler(loudSystem, loudPcm, 0);
    Patch(toLoud, loud);
    Event(toLoud, SCE_NGS2_VOICE_EVENT_PLAY);
    auto clipped = RenderI16(loudSystem);
    for (std::uint32_t i = 0; i < Grain; i++) Require(clipped[i] == (i < 3 ? 0 : 32767));
    Control(loud, SCE_NGS2_MASTERING_VOICE_PARAM_SETUP, Ngs2MasteringVoiceSetupParam{{}, 1, 0});
    Event(loud, SCE_NGS2_VOICE_EVENT_PLAY);
    auto reset = RenderI16(loudSystem);
    for (std::int16_t sample : reset) Require(sample == 24576);
    Require(sceNgs2SystemDestroy(loudSystem, nullptr) == SCE_NGS2_OK);
}

static uintptr_t RackVoice(uintptr_t rack, std::uint32_t index) {
    uintptr_t voice = 0;
    Require(sceNgs2RackGetVoiceHandle(rack, index, &voice) == SCE_NGS2_OK && voice != 0);
    return voice;
}

static uintptr_t MasteringRack(uintptr_t system, std::uint32_t voices) {
    Ngs2MasteringRackOption option{};
    option.rack_option.size = sizeof(option);
    option.rack_option.max_grain_samples = 512;
    option.rack_option.max_voices = voices;
    option.rack_option.max_input_delay_blocks = 1;
    option.rack_option.max_matrices = 1;
    option.rack_option.max_ports = 8;
    option.max_channels = 8;
    Ngs2ContextBufferInfo query{};
    Require(sceNgs2RackQueryBufferSize(SCE_NGS2_RACK_ID_MASTERING, &option.rack_option, &query) == SCE_NGS2_OK);
    const auto buffer = Buffer(query);
    uintptr_t rack = 0;
    Require(sceNgs2RackCreate(system, SCE_NGS2_RACK_ID_MASTERING, &option.rack_option, &buffer, &rack) == SCE_NGS2_OK && rack != 0);
    return rack;
}

static void StereoSource(uintptr_t voice, const std::vector<std::int16_t>& pcm, uintptr_t master) {
    Control(voice, SCE_NGS2_SAMPLER_VOICE_PARAM_SETUP, Ngs2SamplerVoiceSetupParam{{}, {SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, 1, 48000, 0, 0, 0}});
    const Ngs2WaveformBlock block{0, pcm.size() * sizeof(std::int16_t), 0, 0, static_cast<std::uint32_t>(pcm.size()), 0, 0};
    Control(voice, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS, Ngs2SamplerVoiceWaveformBlocksParam{{}, pcm.data(), 0, 1, &block});
    const float levels[2] = {1.0f, 0.5f};
    Control(voice, SCE_NGS2_VOICE_PARAM_MATRIX_LEVELS, Ngs2VoiceMatrixLevelsParam{{}, 0, 2, levels});
    Control(voice, SCE_NGS2_VOICE_PARAM_PORT_MATRIX, Ngs2VoicePortMatrixParam{{}, 0, 0});
    Patch(voice, master);
}

static void TestStereoIntoSurround() {
    const auto system = CreateSystem();
    const auto masteringRack = MasteringRack(system, 3);
    const auto samplerRack = CreateRack(system, SCE_NGS2_RACK_ID_SAMPLER);
    uintptr_t masters[3];
    for (std::uint32_t i = 0; i < 3; i++) {
        masters[i] = RackVoice(masteringRack, i);
        Control(masters[i], SCE_NGS2_MASTERING_VOICE_PARAM_SETUP, Ngs2MasteringVoiceSetupParam{{}, 2, 0});
        Control(masters[i], SCE_NGS2_MASTERING_VOICE_PARAM_OUTPUT, Ngs2MasteringVoiceOutputParam{{}, i, 0});
        Event(masters[i], SCE_NGS2_VOICE_EVENT_PLAY);
    }
    const std::vector<std::int16_t> pcm(Grain, 16384);
    for (std::uint32_t i = 0; i < 2; i++) {
        const auto source = RackVoice(samplerRack, i);
        StereoSource(source, pcm, masters[i]);
        Event(source, SCE_NGS2_VOICE_EVENT_PLAY);
    }
    std::vector<float> outFive(Grain * 6, -1.0f);
    std::vector<float> outSeven(Grain * 8, -1.0f);
    std::vector<float> outQuad(Grain * 4, -1.0f);
    const Ngs2RenderBufferInfo info[3] = {
        {outFive.data(), outFive.size() * sizeof(float), SCE_NGS2_WAVEFORM_TYPE_PCM_F32L, 6},
        {outSeven.data(), outSeven.size() * sizeof(float), SCE_NGS2_WAVEFORM_TYPE_PCM_F32L, 8},
        {outQuad.data(), outQuad.size() * sizeof(float), SCE_NGS2_WAVEFORM_TYPE_PCM_F32L, 4},
    };
    Require(sceNgs2SystemRender(system, info, 3) == SCE_NGS2_OK);
    for (std::uint32_t i = 0; i < Grain; i++) {
        const float delayed = i < 3 ? 0.0f : 1.0f;
        Require(outFive[i * 6] == 0.5f * delayed && outFive[i * 6 + 1] == 0.25f * delayed);
        for (std::uint32_t channel = 2; channel < 6; channel++) Require(outFive[i * 6 + channel] == 0.0f);
        Require(outSeven[i * 8] == 0.5f * delayed && outSeven[i * 8 + 1] == 0.25f * delayed);
        for (std::uint32_t channel = 2; channel < 8; channel++) Require(outSeven[i * 8 + channel] == 0.0f);
    }

    const auto rejectedSource = RackVoice(samplerRack, 2);
    StereoSource(rejectedSource, pcm, masters[2]);
    Event(rejectedSource, SCE_NGS2_VOICE_EVENT_PLAY);
    bool threw = false;
    try {
        sceNgs2SystemRender(system, info, 3);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    Require(threw);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static bool AnsweredWithin(const std::atomic<bool>& answered, std::chrono::milliseconds limit = std::chrono::seconds(2)) {
    const auto deadline = std::chrono::steady_clock::now() + limit;
    while (!answered && std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return answered;
}

static void TestRackLockLeavesQueries() {
    const auto system = CreateSystem();
    const auto rack = CreateRack(system, SCE_NGS2_RACK_ID_SAMPLER);
    const auto voice = Voice(rack);
    Require(sceNgs2RackLock(0x1234) == SCE_NGS2_ERROR_INVALID_RACK_HANDLE);
    Require(sceNgs2RackLock(rack) == SCE_NGS2_OK);
    Require(sceNgs2RackLock(rack) == SCE_NGS2_OK);
    std::atomic<bool> answered = false;
    std::thread query([&] {
        std::uint32_t flags = 0;
        Require(sceNgs2VoiceGetStateFlags(voice, &flags) == SCE_NGS2_OK);
        answered = true;
    });
    const bool answeredWhileLocked = AnsweredWithin(answered);
    std::atomic<bool> rendered = false;
    std::thread render([&] {
        RenderI16(system);
        rendered = true;
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const bool renderedWhileLocked = rendered;
    Require(sceNgs2RackUnlock(rack) == SCE_NGS2_OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const bool renderedWhileLockedOnce = rendered;
    Require(sceNgs2RackUnlock(rack) == SCE_NGS2_OK);
    query.join();
    render.join();
    Require(answeredWhileLocked);
    Require(!renderedWhileLocked && !renderedWhileLockedOnce && rendered);
    Require(sceNgs2RackUnlock(rack) == SCE_NGS2_ERROR_INVALID_RACK_HANDLE);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

struct QueryInCallback {
    uintptr_t voice = 0;
    std::atomic<bool> answered = false;
    bool answeredInCallback = false;
    std::thread query;
};

static void APS5_VABI QueryFromAnotherThread(const Ngs2VoiceCallbackInfo* info) {
    auto& callback = *reinterpret_cast<QueryInCallback*>(info->callback_data);
    callback.query = std::thread([&callback] {
        std::uint32_t flags = 0;
        Require(sceNgs2VoiceGetStateFlags(callback.voice, &flags) == SCE_NGS2_OK);
        callback.answered = true;
    });
    callback.answeredInCallback = AnsweredWithin(callback.answered);
}

static void TestVoiceCallbackLeavesQueries() {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 1);
    const std::vector<std::int16_t> pcm(Grain / 2, 1000);
    const auto sampler = Sampler(system, pcm, 0);
    Patch(sampler, master);
    QueryInCallback callback;
    callback.voice = sampler;
    Control(sampler, SCE_NGS2_VOICE_PARAM_CALLBACK, Ngs2VoiceCallbackParam{{}, QueryFromAnotherThread, reinterpret_cast<std::uintptr_t>(&callback), SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_END, 0});
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    RenderI16(system);
    Require(callback.query.joinable());
    callback.query.join();
    Require(callback.answered && callback.answeredInCallback);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

struct RackLockInCallback {
    uintptr_t rack = 0;
    std::atomic<bool> locked = false;
    std::atomic<bool> callbackReturned = false;
    std::atomic<bool> release = false;
    bool lockedDuringCallback = false;
    std::thread locker;
};

static void APS5_VABI LockFromAnotherThread(const Ngs2VoiceCallbackInfo* info) {
    auto& callback = *reinterpret_cast<RackLockInCallback*>(info->callback_data);
    callback.locker = std::thread([&callback] {
        Require(sceNgs2RackLock(callback.rack) == SCE_NGS2_OK);
        callback.lockedDuringCallback = !callback.callbackReturned;
        callback.locked = true;
        while (!callback.release) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        Require(sceNgs2RackUnlock(callback.rack) == SCE_NGS2_OK);
    });
    static_cast<void>(AnsweredWithin(callback.locked, std::chrono::milliseconds(200)));
    callback.callbackReturned = true;
}

static void TestRackLockWaitsForRender() {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 1);
    const std::vector<std::int16_t> pcm(Grain / 2, 1000);
    RackLockInCallback callback;
    const auto sampler = Sampler(system, pcm, 0, &callback.rack);
    Patch(sampler, master);
    Control(sampler, SCE_NGS2_VOICE_PARAM_CALLBACK, Ngs2VoiceCallbackParam{{}, LockFromAnotherThread, reinterpret_cast<std::uintptr_t>(&callback), SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_END, 0});
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    RenderI16(system);
    const bool lockedAfterRender = AnsweredWithin(callback.locked);
    callback.release = true;
    Require(callback.locker.joinable());
    callback.locker.join();
    Require(lockedAfterRender && !callback.lockedDuringCallback);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

struct CrossSystemCallback {
    uintptr_t otherRack = 0;
    std::atomic<bool>* entered = nullptr;
    const std::atomic<bool>* awaited = nullptr;
};

static void APS5_VABI LockAnotherSystem(const Ngs2VoiceCallbackInfo* info) {
    auto& callback = *reinterpret_cast<CrossSystemCallback*>(info->callback_data);
    *callback.entered = true;
    static_cast<void>(AnsweredWithin(*callback.awaited, std::chrono::milliseconds(200)));
    Require(sceNgs2RackLock(callback.otherRack) == SCE_NGS2_OK);
    Require(sceNgs2RackUnlock(callback.otherRack) == SCE_NGS2_OK);
}

static uintptr_t CallbackSampler(uintptr_t system, CrossSystemCallback& callback, uintptr_t* rack) {
    static const std::vector<std::int16_t> pcm(Grain / 2, 1000);
    const auto sampler = Sampler(system, pcm, 0, rack);
    Patch(sampler, Mastering(system, 1));
    Control(sampler, SCE_NGS2_VOICE_PARAM_CALLBACK, Ngs2VoiceCallbackParam{{}, LockAnotherSystem, reinterpret_cast<std::uintptr_t>(&callback), SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_END, 0});
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    return sampler;
}

static void TestCallbacksLockEachOthersSystem() {
    const auto first = CreateSystem();
    const auto second = CreateSystem();
    std::atomic<bool> firstEntered = false;
    std::atomic<bool> secondEntered = false;
    CrossSystemCallback firstCallback{0, &firstEntered, &secondEntered};
    CrossSystemCallback secondCallback{0, &secondEntered, &firstEntered};
    uintptr_t firstRack = 0;
    uintptr_t secondRack = 0;
    CallbackSampler(first, firstCallback, &firstRack);
    CallbackSampler(second, secondCallback, &secondRack);
    firstCallback.otherRack = secondRack;
    secondCallback.otherRack = firstRack;
    std::atomic<bool> firstRendered = false;
    std::atomic<bool> secondRendered = false;
    std::thread firstRender([&] {
        RenderI16(first);
        firstRendered = true;
    });
    std::thread secondRender([&] {
        RenderI16(second);
        secondRendered = true;
    });
    Require(AnsweredWithin(firstRendered) && AnsweredWithin(secondRendered));
    firstRender.join();
    secondRender.join();
    Require(firstEntered && secondEntered);
    Require(sceNgs2SystemDestroy(first, nullptr) == SCE_NGS2_OK);
    Require(sceNgs2SystemDestroy(second, nullptr) == SCE_NGS2_OK);
}

static void TestRackLocksAcrossSystemsAndRender() {
    const auto first = CreateSystem();
    const auto second = CreateSystem();
    const auto heldRack = CreateRack(second, SCE_NGS2_RACK_ID_SAMPLER);
    std::atomic<bool> entered = false;
    const std::atomic<bool> never = false;
    CrossSystemCallback callback{heldRack, &entered, &never};
    uintptr_t renderedRack = 0;
    CallbackSampler(first, callback, &renderedRack);
    std::atomic<bool> holding = false;
    std::atomic<bool> released = false;
    std::thread locker([&] {
        Require(sceNgs2RackLock(heldRack) == SCE_NGS2_OK);
        holding = true;
        static_cast<void>(AnsweredWithin(entered, std::chrono::milliseconds(200)));
        Require(sceNgs2RackLock(renderedRack) == SCE_NGS2_OK);
        Require(sceNgs2RackUnlock(renderedRack) == SCE_NGS2_OK);
        Require(sceNgs2RackUnlock(heldRack) == SCE_NGS2_OK);
        released = true;
    });
    Require(AnsweredWithin(holding));
    std::atomic<bool> rendered = false;
    std::thread render([&] {
        RenderI16(first);
        rendered = true;
    });
    Require(AnsweredWithin(released) && AnsweredWithin(rendered));
    locker.join();
    render.join();
    Require(entered);
    Require(sceNgs2SystemDestroy(first, nullptr) == SCE_NGS2_OK);
    Require(sceNgs2SystemDestroy(second, nullptr) == SCE_NGS2_OK);
}

static void TestRackDestroyWaitsForLock() {
    const auto system = CreateSystem();
    const auto rack = CreateRack(system, SCE_NGS2_RACK_ID_SAMPLER);
    Require(sceNgs2RackLock(rack) == SCE_NGS2_OK);
    std::atomic<bool> destroyed = false;
    std::thread destroyer([&] {
        Require(sceNgs2RackDestroy(rack, nullptr) == SCE_NGS2_OK);
        destroyed = true;
    });
    const bool destroyedWhileLocked = AnsweredWithin(destroyed, std::chrono::milliseconds(200));
    Require(!destroyedWhileLocked && sceNgs2RackUnlock(rack) == SCE_NGS2_OK);
    Require(AnsweredWithin(destroyed));
    destroyer.join();
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

struct DestroyInCallback {
    uintptr_t handle = 0;
    bool system = false;
};

static void APS5_VABI DestroyFromCallback(const Ngs2VoiceCallbackInfo* info) {
    const auto& destroy = *reinterpret_cast<const DestroyInCallback*>(info->callback_data);
    static_cast<void>(destroy.system ? sceNgs2SystemDestroy(destroy.handle, nullptr) : sceNgs2RackDestroy(destroy.handle, nullptr));
}

static void APS5_VABI RenderFromCallback(const Ngs2VoiceCallbackInfo* info) {
    static_cast<void>(RenderI16(info->callback_data));
}

static void TestDestroyDuringOwnRenderThrows() {
    for (const bool destroySystem : {false, true}) {
        const auto system = CreateSystem();
        uintptr_t rack = 0;
        const std::vector<std::int16_t> pcm(Grain / 2, 1000);
        const auto sampler = Sampler(system, pcm, 0, &rack);
        Patch(sampler, Mastering(system, 1));
        const DestroyInCallback destroy{destroySystem ? system : rack, destroySystem};
        Control(sampler, SCE_NGS2_VOICE_PARAM_CALLBACK, Ngs2VoiceCallbackParam{{}, DestroyFromCallback, reinterpret_cast<std::uintptr_t>(&destroy), SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_END, 0});
        Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
        bool refused = false;
        try {
            RenderI16(system);
        } catch (const std::runtime_error& error) {
            refused = std::string(error.what()).find("is not implemented") != std::string::npos;
        }
        Require(refused);
        Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
    }
    for (const bool destroySystem : {false, true}) {
        const auto outer = CreateSystem();
        const auto inner = CreateSystem();
        uintptr_t outerRack = 0;
        const std::vector<std::int16_t> pcm(Grain / 2, 1000);
        const auto outerSampler = Sampler(outer, pcm, 0, &outerRack);
        Patch(outerSampler, Mastering(outer, 1));
        const auto innerSampler = Sampler(inner, pcm, 0);
        Patch(innerSampler, Mastering(inner, 1));
        const DestroyInCallback destroy{destroySystem ? outer : outerRack, destroySystem};
        Control(outerSampler, SCE_NGS2_VOICE_PARAM_CALLBACK, Ngs2VoiceCallbackParam{{}, RenderFromCallback, inner, SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_END, 0});
        Control(innerSampler, SCE_NGS2_VOICE_PARAM_CALLBACK, Ngs2VoiceCallbackParam{{}, DestroyFromCallback, reinterpret_cast<std::uintptr_t>(&destroy), SCE_NGS2_VOICE_CALLBACK_FLAG_BLOCK_END, 0});
        Event(outerSampler, SCE_NGS2_VOICE_EVENT_PLAY);
        Event(innerSampler, SCE_NGS2_VOICE_EVENT_PLAY);
        bool refused = false;
        try {
            RenderI16(outer);
        } catch (const std::runtime_error& error) {
            refused = std::string(error.what()).find("is not implemented") != std::string::npos;
        }
        Require(refused);
        Require(sceNgs2SystemDestroy(inner, nullptr) == SCE_NGS2_OK);
        Require(sceNgs2SystemDestroy(outer, nullptr) == SCE_NGS2_OK);
    }
}

static void TestLock() {
    const auto system = CreateSystem();
    const auto voice = Voice(CreateRack(system, SCE_NGS2_RACK_ID_SAMPLER));
    Require(sceNgs2SystemLock(0x1234) == SCE_NGS2_ERROR_INVALID_SYSTEM_HANDLE);
    Require(sceNgs2SystemUnlock(0x1234) == SCE_NGS2_ERROR_INVALID_SYSTEM_HANDLE);
    Require(sceNgs2SystemUnlock(system) == SCE_NGS2_OK);

    Ngs2SystemInfo info{};
    Require(sceNgs2SystemLock(system) == SCE_NGS2_OK);
    Require(sceNgs2SystemLock(system) == SCE_NGS2_OK);
    std::atomic<bool> otherDone = false;
    std::thread other([&] {
        Ngs2SystemInfo otherInfo{};
        std::uint32_t flags = 0;
        Require(sceNgs2SystemGetInfo(system, &otherInfo, sizeof(otherInfo)) == SCE_NGS2_OK);
        Require(sceNgs2VoiceGetStateFlags(voice, &flags) == SCE_NGS2_OK);
        std::vector<std::int16_t> out(Grain * 2, -1);
        const Ngs2RenderBufferInfo buffer{out.data(), out.size() * sizeof(std::int16_t), SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, 1};
        Require(sceNgs2SystemRender(system, &buffer, 1) == SCE_NGS2_OK);
        Require(std::all_of(out.begin(), out.begin() + Grain, [](std::int16_t sample) { return sample == 0; }));
        Require(std::all_of(out.begin() + Grain, out.end(), [](std::int16_t sample) { return sample == -1; }));
        Require(sceNgs2SystemGetInfo(system, &otherInfo, sizeof(otherInfo)) == SCE_NGS2_OK && otherInfo.render_count == 0);
        otherDone = true;
    });
    for (int i = 0; i < 200 && !otherDone; i++) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    Require(otherDone);
    other.join();
    RenderI16(system);
    Require(sceNgs2SystemGetInfo(system, &info, sizeof(info)) == SCE_NGS2_OK && info.render_count == 1);

    std::atomic<bool> done = false;
    std::thread blocked([&] {
        Require(sceNgs2SystemLock(system) == SCE_NGS2_OK);
        done = true;
        RenderI16(system);
        Require(sceNgs2SystemUnlock(system) == SCE_NGS2_OK);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    Require(!done);
    Require(sceNgs2SystemUnlock(system) == SCE_NGS2_OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    Require(!done);
    Require(sceNgs2SystemUnlock(system) == SCE_NGS2_OK);
    blocked.join();
    Require(done);
    Require(sceNgs2SystemGetInfo(system, &info, sizeof(info)) == SCE_NGS2_OK && info.render_count == 2);

    Require(sceNgs2SystemLock(system) == SCE_NGS2_OK);
    std::thread unlocker([&] { Require(sceNgs2SystemUnlock(system) == SCE_NGS2_OK); });
    unlocker.join();
    std::thread renderer([&] { RenderI16(system); });
    renderer.join();
    Require(sceNgs2SystemGetInfo(system, &info, sizeof(info)) == SCE_NGS2_OK && info.render_count == 3);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static constexpr float HalfPower = std::numbers::sqrt2_v<float> / 2.0f;

static std::vector<float> MixFrame(std::uint32_t sourceChannels, std::uint32_t sourceChannel, std::uint32_t destChannels) {
    const auto firstBuffer = usedBuffers;
    const auto system = CreateSystem();
    const auto master = Mastering(system, destChannels);
    const auto voice = Voice(CreateRack(system, SCE_NGS2_RACK_ID_SAMPLER));
    Control(voice, SCE_NGS2_SAMPLER_VOICE_PARAM_SETUP, Ngs2SamplerVoiceSetupParam{{}, {SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, sourceChannels, 48000, 0, 0, 0}});
    std::vector<std::int16_t> frames(Grain * sourceChannels, 0);
    for (std::uint32_t i = 0; i < Grain; i++) frames[i * sourceChannels + sourceChannel] = 16384;
    const Ngs2WaveformBlock block{0, frames.size() * sizeof(std::int16_t), 0, 0, Grain, 0, 0};
    Control(voice, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS, Ngs2SamplerVoiceWaveformBlocksParam{{}, frames.data(), 0, 1, &block});
    Patch(voice, master);
    Event(voice, SCE_NGS2_VOICE_EVENT_PLAY);
    std::vector<float> out(Grain * destChannels, -1.0f);
    const Ngs2RenderBufferInfo info{out.data(), out.size() * sizeof(float), SCE_NGS2_WAVEFORM_TYPE_PCM_F32L, destChannels};
    Require(sceNgs2SystemRender(system, &info, 1) == SCE_NGS2_OK);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
    usedBuffers = firstBuffer;
    return std::vector<float>(out.begin() + 3 * destChannels, out.begin() + 4 * destChannels);
}

static void TestDefaultChannelMap() {
    const float half = 0.5f * HalfPower;
    const float quarter = 0.5f * (std::numbers::sqrt2_v<float> / 4.0f);
    Require(MixFrame(1, 0, 1) == std::vector<float>{0.5f});
    Require(MixFrame(1, 0, 2) == (std::vector<float>{half, half}));
    Require(MixFrame(1, 0, 6) == (std::vector<float>{0, 0, 0.5f, 0, 0, 0}));
    Require(MixFrame(1, 0, 8) == (std::vector<float>{0, 0, 0.5f, 0, 0, 0, 0, 0}));
    Require(MixFrame(2, 0, 1) == std::vector<float>{half} && MixFrame(2, 1, 1) == std::vector<float>{half});
    Require(MixFrame(2, 1, 2) == (std::vector<float>{0, 0.5f}));
    Require(MixFrame(2, 1, 6) == (std::vector<float>{0, 0.5f, 0, 0, 0, 0}));
    Require(MixFrame(3, 2, 2) == (std::vector<float>{0, 0}));
    Require(MixFrame(3, 2, 6) == (std::vector<float>{0, 0, 0, 0.5f, 0, 0}));
    Require(MixFrame(4, 2, 1) == std::vector<float>{0.25f});
    Require(MixFrame(4, 2, 2) == (std::vector<float>{half, 0}));
    Require(MixFrame(4, 3, 6) == (std::vector<float>{0, 0, 0, 0, 0, 0.5f}));
    Require(MixFrame(5, 2, 2) == (std::vector<float>{half, half}));
    Require(MixFrame(5, 3, 1) == std::vector<float>{0.25f});
    Require(MixFrame(5, 3, 2) == (std::vector<float>{0, 0}));
    Require(MixFrame(6, 3, 1) == std::vector<float>{0});
    Require(MixFrame(6, 4, 2) == (std::vector<float>{0.25f, 0}));
    Require(MixFrame(6, 5, 8) == (std::vector<float>{0, 0, 0, 0, 0, 0.5f, 0, 0}));
    Require(MixFrame(7, 6, 1) == std::vector<float>{quarter});
    Require(MixFrame(7, 6, 2) == (std::vector<float>{quarter, quarter}));
    Require(MixFrame(7, 6, 6) == (std::vector<float>{0, 0, 0, 0, half, half}));
    Require(MixFrame(7, 6, 8) == (std::vector<float>{0, 0, 0, 0, 0, 0, half, half}));
    Require(MixFrame(8, 6, 2) == (std::vector<float>{0.25f, 0}));
    Require(MixFrame(8, 7, 6) == (std::vector<float>{0, 0, 0, 0, 0, 0.5f}));
    Require(MixFrame(8, 7, 8) == (std::vector<float>{0, 0, 0, 0, 0, 0, 0, 0.5f}));

    const auto system = CreateSystem();
    const auto master = Mastering(system, 2);
    const auto submixer = Voice(CreateRack(system, SCE_NGS2_RACK_ID_SUBMIXER));
    Control(submixer, SCE_NGS2_SUBMIXER_VOICE_PARAM_SETUP, Ngs2SubmixerVoiceSetupParam{{}, 4, 0});
    Patch(submixer, master);
    Event(submixer, SCE_NGS2_VOICE_EVENT_PLAY);
    const std::vector<std::int16_t> pcm(Grain, 16384);
    const auto sampler = Sampler(system, pcm, 0);
    Patch(sampler, submixer);
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    std::vector<float> out(Grain * 2, -1.0f);
    const Ngs2RenderBufferInfo info{out.data(), out.size() * sizeof(float), SCE_NGS2_WAVEFORM_TYPE_PCM_F32L, 2};
    bool unsupported = false;
    try { sceNgs2SystemRender(system, &info, 1); } catch (const std::runtime_error&) { unsupported = true; }
    Require(unsupported);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static void TestUnsetMatrix() {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 2);
    const auto submixer = Voice(CreateRack(system, SCE_NGS2_RACK_ID_SUBMIXER));
    Control(submixer, SCE_NGS2_SUBMIXER_VOICE_PARAM_SETUP, Ngs2SubmixerVoiceSetupParam{{}, 2, 0});
    Patch(submixer, master);
    Event(submixer, SCE_NGS2_VOICE_EVENT_PLAY);

    const std::vector<std::int16_t> pcm(Grain, 16384);
    const auto sampler = Sampler(system, pcm, 0);
    Patch(sampler, submixer);
    Control(sampler, SCE_NGS2_VOICE_PARAM_PORT_MATRIX, Ngs2VoicePortMatrixParam{{}, 0, 0});
    Control(sampler, SCE_NGS2_VOICE_PARAM_PORT_VOLUME, Ngs2VoicePortVolumeParam{{}, 0, 0.5f});
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);

    std::vector<float> out(Grain * 2, -1.0f);
    const Ngs2RenderBufferInfo info{out.data(), out.size() * sizeof(float), SCE_NGS2_WAVEFORM_TYPE_PCM_F32L, 2};
    Require(sceNgs2SystemRender(system, &info, 1) == SCE_NGS2_OK);
    const float spread = 0.25f * HalfPower;
    for (std::uint32_t i = 0; i < Grain; i++) Require(out[i * 2] == (i < 3 ? 0.0f : spread) && out[i * 2 + 1] == (i < 3 ? 0.0f : spread));
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);

    const auto stereoSystem = CreateSystem();
    const auto stereoMaster = Mastering(stereoSystem, 2);
    const auto stereo = Voice(CreateRack(stereoSystem, SCE_NGS2_RACK_ID_SAMPLER));
    Control(stereo, SCE_NGS2_SAMPLER_VOICE_PARAM_SETUP, Ngs2SamplerVoiceSetupParam{{}, {SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, 2, 48000, 0, 0, 0}});
    std::vector<std::int16_t> frames;
    for (std::uint32_t i = 0; i < Grain; i++) {
        frames.push_back(16384);
        frames.push_back(8192);
    }
    const Ngs2WaveformBlock block{0, frames.size() * sizeof(std::int16_t), 0, 0, Grain, 0, 0};
    Control(stereo, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS, Ngs2SamplerVoiceWaveformBlocksParam{{}, frames.data(), 0, 1, &block});
    Patch(stereo, stereoMaster);
    Control(stereo, SCE_NGS2_VOICE_PARAM_PORT_MATRIX, Ngs2VoicePortMatrixParam{{}, 0, 0});
    Event(stereo, SCE_NGS2_VOICE_EVENT_PLAY);
    Require(sceNgs2SystemRender(stereoSystem, &info, 1) == SCE_NGS2_OK);
    for (std::uint32_t i = 0; i < Grain; i++) Require(out[i * 2] == (i < 3 ? 0.0f : 0.5f) && out[i * 2 + 1] == (i < 3 ? 0.0f : 0.25f));
    Require(sceNgs2SystemDestroy(stereoSystem, nullptr) == SCE_NGS2_OK);
}

constexpr std::uint32_t OracleGrain = 256;

struct ResampledOp {
    std::uint32_t render;
    std::uint32_t eventId;
    float pitch;
};

struct ResampledRun {
    std::vector<float> out;
    std::vector<std::uint32_t> flags;
    std::vector<std::uint64_t> decoded;
    std::vector<std::uint64_t> decodedBytes;
};

static ResampledRun RenderResampled(std::uint32_t rate, const float* pitch, const std::vector<std::int16_t>& pcm, std::uint32_t channels,
                                    const std::vector<Ngs2WaveformBlock>& blocks, std::uint32_t renders, const std::vector<ResampledOp>& ops = {}, std::uint32_t systemRate = 48000) {
    const auto firstBuffer = usedBuffers;
    const auto system = CreateSystem();
    Require(sceNgs2SystemSetGrainSamples(system, OracleGrain) == SCE_NGS2_OK && sceNgs2SystemSetSampleRate(system, systemRate) == SCE_NGS2_OK);
    const auto master = Mastering(system, channels);
    const auto sampler = Voice(CreateRack(system, SCE_NGS2_RACK_ID_SAMPLER));
    Control(sampler, SCE_NGS2_SAMPLER_VOICE_PARAM_SETUP, Ngs2SamplerVoiceSetupParam{{}, {SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, channels, rate, 0, 0, 0}});
    if (pitch != nullptr) Control(sampler, SCE_NGS2_SAMPLER_VOICE_PARAM_PITCH, Ngs2SamplerVoicePitchParam{{}, *pitch});
    Control(sampler, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS,
            Ngs2SamplerVoiceWaveformBlocksParam{{}, pcm.data(), 0, static_cast<std::uint32_t>(blocks.size()), blocks.data()});
    Patch(sampler, master);
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    ResampledRun run;
    std::vector<float> out(OracleGrain * channels);
    for (std::uint32_t render = 0; render < renders; render++) {
        for (const auto& op : ops) {
            if (op.render != render) continue;
            if (op.eventId == 0) Control(sampler, SCE_NGS2_SAMPLER_VOICE_PARAM_PITCH, Ngs2SamplerVoicePitchParam{{}, op.pitch});
            else Event(sampler, op.eventId);
        }
        const Ngs2RenderBufferInfo info{out.data(), out.size() * sizeof(float), SCE_NGS2_WAVEFORM_TYPE_PCM_F32L, channels};
        Require(sceNgs2SystemRender(system, &info, 1) == SCE_NGS2_OK);
        run.out.insert(run.out.end(), out.begin(), out.end());
        run.flags.push_back(Flags(sampler));
        Ngs2SamplerVoiceState state{};
        Require(sceNgs2VoiceGetState(sampler, &state.voice_state, sizeof(state)) == SCE_NGS2_OK);
        run.decoded.push_back(state.num_decoded_samples);
        run.decodedBytes.push_back(state.decoded_data_size);
    }
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
    usedBuffers = firstBuffer;
    return run;
}

static std::vector<Ngs2WaveformBlock> WholeBlock(const std::vector<std::int16_t>& pcm, std::uint32_t channels, std::uint32_t repeats = 0) {
    return {{0, pcm.size() * sizeof(std::int16_t), repeats, 0, static_cast<std::uint32_t>(pcm.size() / channels), 0, 0}};
}

static std::uint32_t OracleRenders(std::size_t frames, std::uint32_t rate, float pitch, std::uint32_t extra = 3) {
    const double ratio = std::clamp(rate * static_cast<double>(pitch) / 48000.0, 0.05, 65535.0 / 16384.0);
    return static_cast<std::uint32_t>(frames / ratio / OracleGrain) + extra;
}

static std::uint64_t Fnv(const std::vector<float>& values) {
    std::uint64_t hash = 1469598103934665603ull;
    for (const float value : values) {
        std::uint32_t bits;
        std::memcpy(&bits, &value, sizeof(bits));
        for (int i = 0; i < 4; i++) {
            hash ^= (bits >> (8 * i)) & 0xff;
            hash *= 1099511628211ull;
        }
    }
    return hash;
}

static std::uint32_t Bits(float value) {
    std::uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static std::vector<std::int16_t> ImpulsePcm(std::uint32_t frames, std::uint32_t at) {
    std::vector<std::int16_t> pcm(frames, 0);
    pcm[at] = -32768;
    return pcm;
}

static std::vector<std::int16_t> RampPcm(std::uint32_t frames) {
    std::vector<std::int16_t> pcm(frames);
    for (std::uint32_t i = 0; i < frames; i++) pcm[i] = static_cast<std::int16_t>(i * 8);
    return pcm;
}

static std::vector<std::int16_t> RandomPcm(std::uint32_t frames, std::uint32_t seed) {
    std::vector<std::int16_t> pcm(frames);
    for (auto& sample : pcm) {
        seed = seed * 1664525u + 1013904223u;
        sample = static_cast<std::int16_t>(static_cast<std::uint16_t>(seed >> 16));
    }
    return pcm;
}

static std::vector<float> Negated(std::vector<float> values) {
    for (auto& value : values) value = 0.0f - value;
    return values;
}

static std::vector<float> Channel(const std::vector<float>& out, std::uint32_t channels, std::uint32_t channel) {
    std::vector<float> values;
    for (std::size_t i = channel; i < out.size(); i += channels) values.push_back(out[i]);
    return values;
}

static void TestResamplerKernel() {
    const auto pcm = ImpulsePcm(64, 10);
    const float pitch = 1.0f / 512.0f;
    const auto run = RenderResampled(48000, &pitch, pcm, 1, WholeBlock(pcm, 1), 140);
    const auto h = Negated(run.out);
    constexpr std::size_t centre = 13 * 512;
    for (std::size_t n = 0; n < h.size(); n++) {
        const std::size_t distance = n > centre ? n - centre : centre - n;
        if (distance > 1024) Require(h[n] == 0.0f);
        else Require(Bits(h[n]) == Bits(h[centre + distance]));
    }
    Require(Fnv(std::vector<float>(h.begin() + centre, h.begin() + centre + 1025)) == 0xc28a375fd0e70d64ull);
    const std::pair<std::size_t, std::uint32_t> pinned[] = {{0, 0x3f800000}, {1, 0x3f7fff5c}, {2, 0x3f7ffd6e}, {128, 0x3f5ada30}, {256, 0x3f07c3b6}, {384, 0x3e51b7b0},
                                                           {511, 0x3a80a49c}, {512, 0x00000000}, {513, 0xba7eb718}, {640, 0xbd60b6eb}, {768, 0xbcf876cd}, {896, 0xbb9e56aa},
                                                           {1000, 0xb807cbae}, {1022, 0xb29e0e07}, {1023, 0xb11e0e07}, {1024, 0x00000000}};
    for (const auto& [j, bits] : pinned) Require(Bits(h[centre + j]) == bits);
}

static void TestResamplerMeasuredHashes() {
    struct Ratio {
        std::uint32_t rate;
        float pitch;
        std::uint64_t impulseAt3;
        std::uint64_t impulseAt100;
        std::uint64_t ramp;
        std::uint64_t random;
    };
    const Ratio ratios[] = {
        {8000, 0.0f, 0xb36982c3cdaf935d, 0x83b84937b197f9b3, 0x23cfa9ec827fc507, 0x5a5757645b27c56e},
        {11025, 0.0f, 0xd113fc7648c1b843, 0xdcff3690f403bc38, 0xd14aff544f942d91, 0x7f1af34df347d769},
        {16000, 0.0f, 0xfa44f26783294814, 0x24d46dcc972ecdb0, 0x72cf2b1a3cc6a8e4, 0x2809292da32c2890},
        {22050, 0.0f, 0x3cf4eb12a124d87f, 0x9c1652de78e4fc79, 0xbc4b5135cac2e7ed, 0x74df8aee4356cd7f},
        {24000, 0.0f, 0xc281124745a1c946, 0xed79ffefbdf13026, 0x5cc881ec44b31d2b, 0x90462b20289f334c},
        {32000, 0.0f, 0x89e8dfaad13daa8a, 0x9e0914cad09a7007, 0x9a025c6da4db1e43, 0x7bbf0b54dbe5371e},
        {44100, 0.0f, 0x0ae0e5103450c2e1, 0x9c21aab1fbf24709, 0x7c013934bb7d7967, 0x2c5e08146996f041},
        {48000, 0.0f, 0x840549260ddd06da, 0xa8fa5ce9e34b758a, 0x1b9fc0fa15527f48, 0xeeeb711ad686aa36},
        {96000, 0.0f, 0x199983bda764a04a, 0x7634e3f0d5b1f383, 0x41599b60d2e00206, 0x412b99814872f90d},
        {48000, 0.5f, 0xc281124745a1c946, 0xed79ffefbdf13026, 0x5cc881ec44b31d2b, 0x90462b20289f334c},
        {48000, 0.75f, 0xa1299dce03b15c7a, 0x6c1dbde9cdf9b54e, 0xa6320661e941e9da, 0x39c528682052481c},
        {48000, 1.25f, 0xdf16bde0add012b5, 0xfc5981c3141320c3, 0x24127a31cbae1587, 0xcb2bc76d06a224f2},
        {48000, 1.5f, 0xa2e81d1932165f0e, 0xf06e33ce4da1a2a2, 0x4655e34bbcd9f197, 0xfdd050c726477089},
        {48000, 2.0f, 0x199983bda764a04a, 0x7634e3f0d5b1f383, 0x41599b60d2e00206, 0x412b99814872f90d},
        {48000, 3.0f, 0xd6968c19e0e8249a, 0xb43a063055adc383, 0xd0eddd9d808662a7, 0xc9fb8999c5a8e22d},
        {48000, 0.1f, 0x3460ea263af17d06, 0xcd1d1b0f681bc6e1, 0xa8afe31921c51744, 0xc7f89c253f6379e3},
        {48000, 7.99f, 0x0cdfcba3c3593c0b, 0x675c1224ee3568a1, 0xc90900bd8a75edca, 0xf2d7111f370e4e2a},
        {44100, 1.1f, 0x203c30fb425c01ad, 0x3eb6fad4565c7437, 0x2575514eefe8553d, 0x0783176bebab3dec},
    };
    const auto at3 = ImpulsePcm(1024, 3);
    const auto at100 = ImpulsePcm(1024, 100);
    const auto ramp = RampPcm(2048);
    const auto random = RandomPcm(2048, 777);
    for (const auto& ratio : ratios) {
        const float* pitch = ratio.pitch == 0.0f ? nullptr : &ratio.pitch;
        const float applied = pitch == nullptr ? 1.0f : ratio.pitch;
        const auto hash = [&](const std::vector<std::int16_t>& pcm) {
            return Fnv(RenderResampled(ratio.rate, pitch, pcm, 1, WholeBlock(pcm, 1), OracleRenders(pcm.size(), ratio.rate, applied)).out);
        };
        Require(hash(at3) == ratio.impulseAt3);
        Require(hash(at100) == ratio.impulseAt100);
        Require(hash(ramp) == ratio.ramp);
        Require(hash(random) == ratio.random);
    }
}

static void TestResamplerMeasuredValues() {
    const auto ramp = RampPcm(2048);
    const auto values = [&](std::uint32_t rate, const float* pitch) { return RenderResampled(rate, pitch, ramp, 1, WholeBlock(ramp, 1), 3).out; };
    const auto at44100 = values(44100, nullptr);
    const float head44100[] = {0.0f, 0.0f, 0.0f, -1.33952826e-05f, 0.000162007826f, 0.000388622284f, 0.000613212585f, 0.00083732605f, 0.00106191635f, 0.00128602982f, 0.00151014328f, 0.00173473358f};
    const float grain44100[] = {0.0553402901f, 0.0555648766f, 0.0557889938f, 0.0560131148f, 0.0562376976f, 0.0564618111f};
    for (std::size_t i = 0; i < std::size(head44100); i++) Require(at44100[i] == head44100[i]);
    for (std::size_t i = 0; i < std::size(grain44100); i++) Require(at44100[250 + i] == grain44100[i]);
    const auto at24000 = values(24000, nullptr);
    const float head24000[] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -7.40480618e-06f, 0.0f, 0.000114665498f, 0.000244140625f, 0.000366210938f, 0.00048828125f, 0.000610351562f};
    for (std::size_t i = 0; i < std::size(head24000); i++) Require(at24000[i] == head24000[i]);
    const float faster = 1.1f;
    const auto at44100Faster = values(44100, &faster);
    const float head44100Faster[] = {0.0f, 0.0f, -5.66088842e-10f, 4.11847304e-06f, 0.000254154205f, 0.000501155853f, 0.000747680664f, 0.000994682312f};
    for (std::size_t i = 0; i < std::size(head44100Faster); i++) Require(at44100Faster[i] == head44100Faster[i]);
    const auto at96000 = values(96000, nullptr);
    const float three = 3.0f;
    const auto at3x = values(48000, &three);
    Require(at96000[0] == 0.0f && at96000[1] == 0.0f && at3x[0] == 0.0f && at3x[1] == 0.0f);
    for (std::uint32_t n = 2; n < 600; n++) {
        Require(at96000[n] == static_cast<float>(ramp[2 * n - 3]) / 32768.0f);
        Require(at3x[n] == static_cast<float>(ramp[3 * n - 3]) / 32768.0f);
    }
    const auto unity = values(48000, nullptr);
    for (std::uint32_t n = 0; n < 3 * OracleGrain; n++) Require(unity[n] == (n < 3 ? 0.0f : static_cast<float>(ramp[n - 3]) / 32768.0f));
}

static void TestResamplerStepFormula() {
    struct StepCase {
        std::uint32_t rate;
        std::uint32_t pitchBits;
        std::uint32_t render;
        std::uint64_t decoded;
    };
    const StepCase cases[] = {
        {44100, 0x3e809249, 15, 945}, {44100, 0x3e8174ae, 10, 654}, {37800, 0x3e8036db, 16, 858}, {37800, 0x3d4d1451, 10, 111},
        {37800, 0x3d4db6db, 7, 81}, {100, 0x3e960000, 6, 1}, {100, 0x3ea50000, 5, 1},
    };
    const std::vector<std::int16_t> silence(3000, 0);
    for (const auto& stepCase : cases) {
        float pitch;
        std::memcpy(&pitch, &stepCase.pitchBits, sizeof(pitch));
        const auto run = RenderResampled(stepCase.rate, &pitch, silence, 1, WholeBlock(silence, 1), stepCase.render + 1);
        Require(run.decoded[stepCase.render] == stepCase.decoded);
    }
}

static void TestResamplerSystemRate() {
    struct RateCase {
        std::uint32_t systemRate;
        std::uint32_t rate;
        std::uint32_t at;
        std::uint64_t hash;
        std::vector<std::uint64_t> decoded;
    };
    const RateCase cases[] = {
        {48000, 24000, 0, 0x0868cde833c08d26, {128, 256, 384, 512, 640, 768}},
        {48000, 24000, 100, 0x56d12998b5d47aa6, {128, 256, 384, 512, 640, 768}},
        {48000, 32000, 0, 0x9fef46657010a6bd, {170, 341, 511, 682, 853, 1023}},
        {48000, 32000, 100, 0xd24395b4e3d6cf07, {170, 341, 511, 682, 853, 1023}},
        {48000, 44100, 0, 0x81989fa7c08ede95, {235, 470, 705, 940, 0, 0}},
        {48000, 44100, 100, 0x3bcd8af1c95f6c89, {235, 470, 705, 940, 0, 0}},
        {48000, 48000, 0, 0x7b922e972a1fb9ca, {256, 512, 768, 1024, 0, 0}},
        {48000, 48000, 100, 0x22843f0f9395ef0a, {256, 512, 768, 1024, 0, 0}},
        {48000, 96000, 0, 0x26c8ba1e58b62383, {512, 1024, 0, 0, 0, 0}},
        {48000, 96000, 100, 0x26c8ba1e58b62383, {512, 1024, 0, 0, 0, 0}},
        {24000, 24000, 0, 0x7b922e972a1fb9ca, {256, 512, 768, 1024, 0, 0}},
        {24000, 24000, 100, 0x22843f0f9395ef0a, {256, 512, 768, 1024, 0, 0}},
        {24000, 32000, 0, 0x48cf788e8e19b666, {341, 682, 1023, 0, 0, 0}},
        {24000, 32000, 100, 0x11e02d28dd861df2, {341, 682, 1023, 0, 0, 0}},
        {24000, 44100, 0, 0xaa4a8df0f6034a42, {470, 940, 0, 0, 0, 0}},
        {24000, 44100, 100, 0x0e5328fa59d6f52e, {470, 940, 0, 0, 0, 0}},
        {24000, 48000, 0, 0x26c8ba1e58b62383, {512, 1024, 0, 0, 0, 0}},
        {24000, 48000, 100, 0x26c8ba1e58b62383, {512, 1024, 0, 0, 0, 0}},
        {24000, 96000, 0, 0x345ae3a927296881, {1023, 0, 0, 0, 0, 0}},
        {24000, 96000, 100, 0x2f87144c26a41a21, {1023, 0, 0, 0, 0, 0}},
        {44100, 24000, 0, 0xe4c3a442049ddc7b, {139, 278, 417, 557, 696, 835}},
        {44100, 24000, 100, 0x46af8a31bcb156d2, {139, 278, 417, 557, 696, 835}},
        {44100, 32000, 0, 0xec79396246cdd5ea, {185, 371, 557, 743, 928, 0}},
        {44100, 32000, 100, 0x8f848c6305b56581, {185, 371, 557, 743, 928, 0}},
        {44100, 44100, 0, 0x7b922e972a1fb9ca, {256, 512, 768, 1024, 0, 0}},
        {44100, 44100, 100, 0x22843f0f9395ef0a, {256, 512, 768, 1024, 0, 0}},
        {44100, 48000, 0, 0xe1c55a79e34e5b6e, {278, 557, 835, 0, 0, 0}},
        {44100, 48000, 100, 0xd26553d89884b260, {278, 557, 835, 0, 0, 0}},
        {44100, 96000, 0, 0xab7a201964a6d652, {557, 0, 0, 0, 0, 0}},
        {44100, 96000, 100, 0x344e6f13d282420a, {557, 0, 0, 0, 0, 0}},
        {96000, 24000, 0, 0x02bc2d4c460bac7a, {64, 128, 192, 256, 320, 384}},
        {96000, 24000, 100, 0x45b1194ef5534d7a, {64, 128, 192, 256, 320, 384}},
        {96000, 32000, 0, 0xb754bd2d66597144, {85, 170, 255, 341, 426, 511}},
        {96000, 32000, 100, 0x81265a52f0b060b0, {85, 170, 255, 341, 426, 511}},
        {96000, 44100, 0, 0xde37c28d81686fc2, {117, 235, 352, 470, 587, 705}},
        {96000, 44100, 100, 0x8cf54a2f76512ff9, {117, 235, 352, 470, 587, 705}},
        {96000, 48000, 0, 0x0868cde833c08d26, {128, 256, 384, 512, 640, 768}},
        {96000, 48000, 100, 0x56d12998b5d47aa6, {128, 256, 384, 512, 640, 768}},
        {96000, 96000, 0, 0x7b922e972a1fb9ca, {256, 512, 768, 1024, 0, 0}},
        {96000, 96000, 100, 0x22843f0f9395ef0a, {256, 512, 768, 1024, 0, 0}},
        {11025, 24000, 0, 0xab7a201964a6d652, {557, 0, 0, 0, 0, 0}},
        {11025, 24000, 100, 0x344e6f13d282420a, {557, 0, 0, 0, 0, 0}},
        {11025, 32000, 0, 0x252cc7a8b001ce2d, {743, 0, 0, 0, 0, 0}},
        {11025, 32000, 100, 0x4afe71c2097acea3, {743, 0, 0, 0, 0, 0}},
        {11025, 44100, 0, 0x345ae3a927296881, {1023, 0, 0, 0, 0, 0}},
        {11025, 44100, 100, 0x2f87144c26a41a21, {1023, 0, 0, 0, 0, 0}},
        {11025, 48000, 0, 0x345ae3a927296881, {1023, 0, 0, 0, 0, 0}},
        {11025, 48000, 100, 0x2f87144c26a41a21, {1023, 0, 0, 0, 0, 0}},
        {11025, 96000, 0, 0x345ae3a927296881, {1023, 0, 0, 0, 0, 0}},
        {11025, 96000, 100, 0x2f87144c26a41a21, {1023, 0, 0, 0, 0, 0}},
        {192000, 24000, 0, 0xde59e11e313dad6a, {32, 64, 96, 128, 160, 192}},
        {192000, 24000, 100, 0x73f7abcc8ccd0f6a, {32, 64, 96, 128, 160, 192}},
        {192000, 32000, 0, 0x74d703af2d7d5703, {42, 85, 127, 170, 213, 255}},
        {192000, 32000, 100, 0x09a979d7fa6198b3, {42, 85, 127, 170, 213, 255}},
        {192000, 44100, 0, 0x36aa4321f0d80b9c, {58, 117, 176, 235, 293, 352}},
        {192000, 44100, 100, 0x35aa57f10cf12138, {58, 117, 176, 235, 293, 352}},
        {192000, 48000, 0, 0x02bc2d4c460bac7a, {64, 128, 192, 256, 320, 384}},
        {192000, 48000, 100, 0x45b1194ef5534d7a, {64, 128, 192, 256, 320, 384}},
        {192000, 96000, 0, 0x0868cde833c08d26, {128, 256, 384, 512, 640, 768}},
        {192000, 96000, 100, 0x56d12998b5d47aa6, {128, 256, 384, 512, 640, 768}},
        {22050, 24000, 0, 0xe1c55a79e34e5b6e, {278, 557, 835, 0, 0, 0}},
        {22050, 24000, 100, 0xd26553d89884b260, {278, 557, 835, 0, 0, 0}},
        {22050, 32000, 0, 0x66f5b34c1d76a321, {371, 743, 0, 0, 0, 0}},
        {22050, 32000, 100, 0x600477907adc830c, {371, 743, 0, 0, 0, 0}},
        {22050, 44100, 0, 0x26c8ba1e58b62383, {512, 1024, 0, 0, 0, 0}},
        {22050, 44100, 100, 0x26c8ba1e58b62383, {512, 1024, 0, 0, 0, 0}},
        {22050, 48000, 0, 0xab7a201964a6d652, {557, 0, 0, 0, 0, 0}},
        {22050, 48000, 100, 0x344e6f13d282420a, {557, 0, 0, 0, 0, 0}},
        {22050, 96000, 0, 0x345ae3a927296881, {1023, 0, 0, 0, 0, 0}},
        {22050, 96000, 100, 0x2f87144c26a41a21, {1023, 0, 0, 0, 0, 0}},
    };
    for (const auto& rateCase : cases) {
        const auto pcm = ImpulsePcm(1024, rateCase.at);
        const auto run = RenderResampled(rateCase.rate, nullptr, pcm, 1, WholeBlock(pcm, 1), 6, {}, rateCase.systemRate);
        Require(Fnv(Negated(run.out)) == rateCase.hash && run.decoded == rateCase.decoded);
        for (std::size_t render = 0; render < run.flags.size(); render++) Require(run.flags[render] == (run.decoded[render] != 0 ? 3u : 0u));
    }
}

static void TestResamplerStereo() {
    const auto ramp = RampPcm(2048);
    const auto random = RandomPcm(2048, 777);
    std::vector<std::int16_t> stereo;
    for (std::size_t i = 0; i < ramp.size(); i++) {
        stereo.push_back(ramp[i]);
        stereo.push_back(random[i]);
    }
    const auto renders = OracleRenders(ramp.size(), 44100, 1.0f);
    const auto run = RenderResampled(44100, nullptr, stereo, 2, WholeBlock(stereo, 2), renders);
    Require(Fnv(Channel(run.out, 2, 0)) == 0x7c013934bb7d7967ull && Fnv(Channel(run.out, 2, 1)) == 0x2c5e08146996f041ull);
}

static void TestResamplerVoiceEnd() {
    struct EndCase {
        std::uint32_t rate;
        float pitch;
        std::uint32_t frames;
        std::uint32_t endRender;
        std::uint64_t lastDecoded;
        std::size_t lastNonzero;
    };
    const EndCase cases[] = {
        {48000, 0.0f, 1024, 5, 1024, 1026}, {48000, 0.0f, 1023, 4, 768, 1023}, {48000, 0.0f, 1020, 4, 768, 1022}, {48000, 0.5f, 512, 5, 512, 1031},
        {48000, 0.5f, 511, 4, 384, 1023}, {48000, 0.5f, 507, 4, 384, 1021}, {48000, 2.0f, 2048, 5, 2048, 1025}, {48000, 2.0f, 2047, 4, 1536, 1023},
        {48000, 0.75f, 768, 5, 768, 1029}, {48000, 0.75f, 767, 4, 576, 1023}, {44100, 0.0f, 941, 5, 940, 1028}, {44100, 0.0f, 940, 5, 940, 1027},
        {44100, 0.0f, 939, 4, 705, 1023},
    };
    for (const auto& endCase : cases) {
        const std::vector<std::int16_t> pcm(endCase.frames, -32768);
        const float* pitch = endCase.pitch == 0.0f ? nullptr : &endCase.pitch;
        const auto run = RenderResampled(endCase.rate, pitch, pcm, 1, WholeBlock(pcm, 1), endCase.endRender + 2);
        const std::uint64_t step = endCase.lastDecoded / (endCase.endRender - 1);
        for (std::uint32_t render = 0; render < run.flags.size(); render++) {
            const bool playing = render + 1 < endCase.endRender;
            Require(run.flags[render] == (playing ? 3u : 0u));
            Require(run.decoded[render] == (playing ? step * (render + 1) : 0u));
        }
        std::size_t lastNonzero = 0;
        for (std::size_t n = 0; n < run.out.size(); n++) {
            if (run.out[n] != 0.0f) lastNonzero = n;
        }
        Require(lastNonzero == endCase.lastNonzero);
    }
}

static void TestResamplerRepeats() {
    const auto pcm = ImpulsePcm(64, 5);
    const auto passes = [&](std::uint32_t rate, std::uint32_t repeats) { return RenderResampled(rate, nullptr, pcm, 1, WholeBlock(pcm, 1, repeats), 3); };
    const std::uint32_t expectedPasses[] = {1, 1, 2, 3};
    for (std::uint32_t repeats = 0; repeats < 4; repeats++) {
        const auto run = passes(48000, repeats);
        for (std::size_t n = 0; n < run.out.size(); n++) Require(run.out[n] == (n % 64 == 8 && n / 64 < expectedPasses[repeats] ? -1.0f : 0.0f));
        Require(run.flags == (std::vector<std::uint32_t>{0, 0, 0}) && run.decoded == (std::vector<std::uint64_t>{0, 0, 0}));
    }
    const auto forever = passes(48000, 0xffffffff);
    for (std::size_t n = 0; n < forever.out.size(); n++) Require(forever.out[n] == (n % 64 == 8 ? -1.0f : 0.0f));
    Require(forever.flags == (std::vector<std::uint32_t>{3, 3, 3}) && forever.decoded == (std::vector<std::uint64_t>{256, 512, 768}));
    const auto forever44100 = passes(44100, 0xffffffff);
    Require(forever44100.decoded == (std::vector<std::uint64_t>{235, 470, 705}));
    const float firstPass[] = {0.0211839676f, -0.325153351f, -0.835731983f, 0.0522350743f};
    const float secondPass[] = {0.0547228605f, -0.746484518f, -0.424194962f, 0.0308588985f};
    for (std::size_t i = 0; i < 4; i++) Require(forever44100.out[7 + i] == firstPass[i] && forever44100.out[77 + i] == secondPass[i]);

    const auto ramp = RampPcm(64);
    std::vector<std::int16_t> thrice;
    for (int pass = 0; pass < 3; pass++) thrice.insert(thrice.end(), ramp.begin(), ramp.end());
    const auto looped = RenderResampled(44100, nullptr, ramp, 1, WholeBlock(ramp, 1, 2), 2).out;
    const auto written = RenderResampled(44100, nullptr, thrice, 1, WholeBlock(thrice, 1), 2).out;
    Require(std::equal(looped.begin(), looped.begin() + 142, written.begin()) && looped[142] != written[142]);
    const auto longer = RampPcm(100);
    const std::vector<Ngs2WaveformBlock> split{{0, 37 * sizeof(std::int16_t), 0, 0, 37, 0, 0}, {37 * sizeof(std::int16_t), 63 * sizeof(std::int16_t), 0, 0, 63, 0, 0}};
    Require(RenderResampled(44100, nullptr, longer, 1, split, 2).out == RenderResampled(44100, nullptr, longer, 1, WholeBlock(longer, 1), 2).out);
}

static void TestResamplerPitchChanges() {
    struct PitchCase {
        std::uint32_t rate;
        std::vector<ResampledOp> ops;
        std::uint64_t hash;
        std::vector<std::uint64_t> decoded;
    };
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const std::vector<std::uint64_t> frozen{256, 512, 512, 512, 768, 1024, 1280};
    const PitchCase cases[] = {
        {48000, {{2, 0, 0.0f}, {4, 0, 1.0f}}, 0x23b98d40ef79ac59, frozen},
        {48000, {{2, 0, -1.0f}, {4, 0, 1.0f}}, 0x23b98d40ef79ac59, frozen},
        {48000, {{2, 0, nan}, {4, 0, 1.0f}}, 0x23b98d40ef79ac59, frozen},
        {48000, {{2, 0, -inf}, {4, 0, 1.0f}}, 0x23b98d40ef79ac59, frozen},
        {48000, {{2, 0, 1e-30f}, {4, 0, 1.0f}}, 0x23b98d40ef79ac59, frozen},
        {48000, {{2, 0, std::nextafter(1.0f / 16384, 0.0f)}, {4, 0, 1.0f}}, 0x23b98d40ef79ac59, frozen},
        {48000, {{2, 0, inf}, {4, 0, 1.0f}}, 0xea79120acbabbcf0, {256, 512, 1535, 2559, 2815, 3071, 3327}},
        {48000, {{2, 0, 1.0f / 16384}, {4, 0, 1.0f}}, 0x7f13e55de2aac137, frozen},
        {48000, {{2, 0, 32.0f / 16384}, {4, 0, 1.0f}}, 0xceb4fd052096b353, {256, 512, 512, 513, 769, 1025, 1281}},
        {44100, {{2, 0, 33.0f / 16384}, {4, 0, 1.0f}}, 0x08db59f01c576dbb, {235, 470, 470, 471, 706, 941, 1176}},
        {48000, {{2, 0, 0.25f}, {2, 0, 0.5f}}, 0x081dbcac74a97c93, {256, 512, 640, 768, 896}},
    };
    const auto ramp = RampPcm(4096);
    for (const auto& pitchCase : cases) {
        const auto renders = static_cast<std::uint32_t>(pitchCase.decoded.size());
        const auto run = RenderResampled(pitchCase.rate, nullptr, ramp, 1, WholeBlock(ramp, 1), renders, pitchCase.ops);
        Require(Fnv(run.out) == pitchCase.hash && run.decoded == pitchCase.decoded);
        Require(std::all_of(run.flags.begin(), run.flags.end(), [](std::uint32_t flags) { return flags == 3; }));
    }
    const auto random = RandomPcm(4096, 4242);
    const float slow = 0.3f;
    const float faster = 1.1f;
    Require(RenderResampled(48000, nullptr, random, 1, WholeBlock(random, 1), 10, {{2, 0, 0.75f}, {4, 0, 1.5f}, {6, 0, 1.0f}}).decoded ==
            (std::vector<std::uint64_t>{256, 512, 704, 896, 1280, 1664, 1920, 2176, 2432, 2688}));
    Require(RenderResampled(44100, &faster, random, 1, WholeBlock(random, 1), 8, {{3, 0, 0.9f}}).decoded == (std::vector<std::uint64_t>{258, 517, 776, 987, 1199, 1411, 1622, 1834}));
    Require(RenderResampled(48000, &slow, random, 1, WholeBlock(random, 1), 6, {{1, 0, 2.7f}, {2, 0, 0.3f}}).decoded == (std::vector<std::uint64_t>{76, 767, 844, 921, 998, 1075}));
    Require(RenderResampled(32000, nullptr, random, 1, WholeBlock(random, 1), 5, {{2, 0, 1.0f}}).decoded == (std::vector<std::uint64_t>{170, 341, 511, 682, 853}));
}

static void TestResamplerPitchAtSetup() {
    const auto ramp = RampPcm(4096);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    for (const float pitch : {0.0f, -0.0f, -1.0f, nan, -inf, 1e-30f, 1.0f / 16384, std::nextafter(1.0f / 16384, 0.0f), 2.0f / 16384, 0.5f / 16384, 1.5f / 16384}) {
        const auto run = RenderResampled(48000, &pitch, ramp, 1, WholeBlock(ramp, 1), 6);
        Require(std::all_of(run.out.begin(), run.out.end(), [](float sample) { return sample == 0.0f; }));
        Require(run.flags == (std::vector<std::uint32_t>(6, 3)) && run.decoded == (std::vector<std::uint64_t>(6, 0)));
    }
    const float head[] = {0.0f, 0.000243663788f, 0.0012202264f, 0.00219678879f, 0.00317335152f, 0.00414991379f, 0.00512647629f, 0.00610303925f};
    const float later[] = {0.29223156f, 0.293208122f, 0.294184655f, 0.295161217f};
    for (const float pitch : {65535.0f / 16384, std::nextafter(65535.0f / 16384, 4.0f), std::nextafter(4.0f, 0.0f), 4.0f, 8.0f, 1e30f, inf}) {
        const auto run = RenderResampled(48000, &pitch, ramp, 1, WholeBlock(ramp, 1), 6);
        for (std::size_t i = 0; i < std::size(head); i++) Require(run.out[i] == head[i]);
        for (std::size_t i = 0; i < std::size(later); i++) Require(run.out[300 + i] == later[i]);
        Require(run.flags == (std::vector<std::uint32_t>{3, 3, 3, 3, 0, 0}) && run.decoded == (std::vector<std::uint64_t>{1023, 2047, 3071, 4095, 0, 0}));
    }
    const float belowClamp = std::nextafter(65535.0f / 16384, 0.0f);
    std::vector<std::int16_t> train(3000, 0);
    for (std::size_t i = 0; i < train.size(); i += 6) train[i] = -32768;
    const float four = 4.0f;
    Require(RenderResampled(48000, &belowClamp, train, 1, WholeBlock(train, 1), 4).out != RenderResampled(48000, &four, train, 1, WholeBlock(train, 1), 4).out);
    const auto system = CreateSystem();
    const auto unset = Voice(CreateRack(system, SCE_NGS2_RACK_ID_SAMPLER));
    bool rejected = false;
    try {
        Control(unset, SCE_NGS2_SAMPLER_VOICE_PARAM_PITCH, Ngs2SamplerVoicePitchParam{{}, nan});
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    Require(rejected && sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static void TestResamplerVoiceRate() {
    const auto ramp = RampPcm(256);
    const float head192000[] = {0.0f, 0.000243663788f, 0.0012202264f, 0.00219678879f, 0.00317335152f, 0.00414991379f};
    const auto fastest = RenderResampled(192000, nullptr, ramp, 1, WholeBlock(ramp, 1), 1);
    for (std::size_t i = 0; i < std::size(head192000); i++) Require(fastest.out[i] == head192000[i]);
    Require(fastest.flags[0] == 0);
    const auto slowest = RenderResampled(1, nullptr, ramp, 1, WholeBlock(ramp, 1), 1);
    Require(slowest.flags[0] == 3 && std::all_of(slowest.out.begin(), slowest.out.end(), [](float sample) { return sample == 0.0f; }));
}

static void TestResamplerAfterEnd() {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 1);
    const auto voice = Voice(CreateRack(system, SCE_NGS2_RACK_ID_SAMPLER));
    Control(voice, SCE_NGS2_SAMPLER_VOICE_PARAM_SETUP, Ngs2SamplerVoiceSetupParam{{}, {SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, 1, 48000, 0, 0, 0}});
    Control(voice, SCE_NGS2_SAMPLER_VOICE_PARAM_PITCH, Ngs2SamplerVoicePitchParam{{}, 0.7f});
    const auto first = RandomPcm(4, 777);
    const Ngs2WaveformBlock firstBlock{0, first.size() * sizeof(std::int16_t), 0, 0, 4, 0, 0};
    Control(voice, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS, Ngs2SamplerVoiceWaveformBlocksParam{{}, first.data(), 0, 1, &firstBlock});
    Patch(voice, master);
    Event(voice, SCE_NGS2_VOICE_EVENT_PLAY);
    const auto played = RenderI16(system);
    Require(Flags(voice) == 0 && played != std::vector<std::int16_t>(Grain, 0));
    Event(voice, SCE_NGS2_VOICE_EVENT_PLAY);
    Require(RenderI16(system) == played && Flags(voice) == 0);
    const auto second = RandomPcm(3 * Grain, 4242);
    const Ngs2WaveformBlock secondBlock{0, second.size() * sizeof(std::int16_t), 0, 0, 3 * Grain, 0, 0};
    Control(voice, SCE_NGS2_SAMPLER_VOICE_PARAM_ADD_WAVEFORM_BLOCKS, Ngs2SamplerVoiceWaveformBlocksParam{{}, second.data(), SCE_NGS2_WAVEFORM_BLOCKS_FLAG_RESET, 1, &secondBlock});
    Control(voice, SCE_NGS2_SAMPLER_VOICE_PARAM_PITCH, Ngs2SamplerVoicePitchParam{{}, 1.0f});
    Event(voice, SCE_NGS2_VOICE_EVENT_PLAY);
    for (std::uint32_t render = 0; render < 2; render++) {
        const auto out = RenderI16(system);
        for (std::uint32_t i = 0; i < Grain; i++) Require(out[i] == Delayed(second, render * Grain + i));
        Require(Flags(voice) == (SCE_NGS2_VOICE_STATE_FLAG_INUSE | SCE_NGS2_VOICE_STATE_FLAG_PLAYING));
    }
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

template <typename TSample>
static void TestRenderCapacity(std::uint32_t waveformType, std::uint32_t channels) {
    const auto system = CreateSystem();
    const auto samples = Grain * channels;
    const TSample sentinel = 123;
    std::vector<TSample> out(samples * 3, sentinel);
    const Ngs2RenderBufferInfo info{out.data(), out.size() * sizeof(TSample), waveformType, channels};
    Require(sceNgs2SystemRender(system, &info, 1) == SCE_NGS2_OK);
    Require(std::all_of(out.begin(), out.begin() + samples, [](TSample value) { return value == 0; }));
    Require(std::all_of(out.begin() + samples, out.end(), [sentinel](TSample value) { return value == sentinel; }));

    const auto master = Mastering(system, channels);
    const std::vector<std::int16_t> pcm(Grain, 16384);
    const auto sampler = Sampler(system, pcm, 0);
    Patch(sampler, master);
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    std::fill(out.begin(), out.end(), sentinel);
    for (const bool head : {false, true}) {
        Require(sceNgs2SystemRender(system, &info, 1) == SCE_NGS2_OK);
        for (std::uint32_t i = 0; i < samples; i++) Require((i / channels < 3) == head ? out[i] > 0 && out[i] != sentinel : out[i] == 0);
        Require(std::all_of(out.begin() + samples, out.end(), [sentinel](TSample value) { return value == sentinel; }));
    }

    Require(sceNgs2SystemRender(system, &info, 1) == SCE_NGS2_OK);
    Require(std::all_of(out.begin(), out.begin() + samples, [](TSample value) { return value == 0; }));
    Require(std::all_of(out.begin() + samples, out.end(), [sentinel](TSample value) { return value == sentinel; }));
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static void TestRenderBufferCount() {
    const auto system = CreateSystem();
    std::vector<int16_t> out(Grain);
    Ngs2RenderBufferInfo infos[32];
    for (int i = 0; i < 32; i++) {
        infos[i] = Ngs2RenderBufferInfo{out.data(), out.size() * sizeof(int16_t), SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, 1};
    }
    Require(sceNgs2SystemRender(system, infos, 31) == SCE_NGS2_OK);
    Require(sceNgs2SystemRender(system, infos, 32) == SCE_NGS2_ERROR_INVALID_NUM_RENDER_BUFFERS);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static void TestRenderBufferTypeValidation() {
    const auto system = CreateSystem();
    std::vector<int16_t> out(Grain);
    Ngs2RenderBufferInfo info0{out.data(), out.size() * sizeof(int16_t), 0x50, 1};
    Require(sceNgs2SystemRender(system, &info0, 1) == SCE_NGS2_ERROR_INVALID_WAVEFORM_TYPE);
    Ngs2RenderBufferInfo valid{out.data(), out.size() * sizeof(int16_t), SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, 1};
    Ngs2RenderBufferInfo infos[2];
    infos[0] = valid;
    infos[1] = Ngs2RenderBufferInfo{out.data(), out.size() * sizeof(int16_t), 0x50, 1};
    Require(sceNgs2SystemRender(system, infos, 2) == SCE_NGS2_ERROR_INVALID_WAVEFORM_TYPE);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static void TestRenderBufferTypeBeforeNull() {
    const auto system = CreateSystem();
    std::vector<int16_t> out(Grain);
    Ngs2RenderBufferInfo info{nullptr, out.size() * sizeof(int16_t), 0x50, 1};
    Require(sceNgs2SystemRender(system, &info, 1) == SCE_NGS2_ERROR_INVALID_WAVEFORM_TYPE);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static void TestRenderBufferNull() {
    const auto system = CreateSystem();
    std::vector<int16_t> out(Grain);
    Ngs2RenderBufferInfo info{nullptr, out.size() * sizeof(int16_t), SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, 1};
    Require(sceNgs2SystemRender(system, &info, 1) == SCE_NGS2_ERROR_INVALID_RENDER_BUFFER);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static void TestRenderRefusedDoesNotRender() {
    const auto system = CreateSystem();
    const auto master = Mastering(system, 1);
    std::vector<std::int16_t> pcm(Grain, 16384);
    const auto sampler = Sampler(system, pcm, 0);
    Patch(sampler, master);
    Event(sampler, SCE_NGS2_VOICE_EVENT_PLAY);
    std::vector<int16_t> out(Grain * 2, 123);
    Ngs2RenderBufferInfo infos[2];
    infos[0] = Ngs2RenderBufferInfo{out.data(), out.size() * sizeof(int16_t), SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, 1};
    infos[1] = Ngs2RenderBufferInfo{out.data() + Grain, out.size() * sizeof(int16_t), 0x50, 1};
    Require(sceNgs2SystemRender(system, infos, 2) == SCE_NGS2_ERROR_INVALID_WAVEFORM_TYPE);
    Require(std::all_of(out.begin(), out.end(), [](int16_t value) { return value == 123; }));
    Ngs2SamplerVoiceState state{};
    Require(sceNgs2VoiceGetState(sampler, &state.voice_state, sizeof(state)) == SCE_NGS2_OK);
    Require(state.num_decoded_samples == 0);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static void TestRenderBigEndianThrows() {
    const auto system = CreateSystem();
    std::vector<int16_t> out(Grain);
    Ngs2RenderBufferInfo infoI16B{out.data(), out.size() * sizeof(int16_t), SCE_NGS2_WAVEFORM_TYPE_PCM_I16B, 1};
    bool i16bThrows = false;
    try { sceNgs2SystemRender(system, &infoI16B, 1); } catch (const std::runtime_error&) { i16bThrows = true; }
    Require(i16bThrows);
    std::vector<float> outF(Grain);
    Ngs2RenderBufferInfo infoF32B{outF.data(), outF.size() * sizeof(float), SCE_NGS2_WAVEFORM_TYPE_PCM_F32B, 1};
    bool f32bThrows = false;
    try { sceNgs2SystemRender(system, &infoF32B, 1); } catch (const std::runtime_error&) { f32bThrows = true; }
    Require(f32bThrows);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static void TestSilentRenderValidation() {
    const auto system = CreateSystem();
    std::vector<float> out(Grain, 123.0f);
    Ngs2RenderBufferInfo info{out.data(), out.size() * sizeof(float) - 1, SCE_NGS2_WAVEFORM_TYPE_PCM_F32L, 1};
    const auto invalid = [&] {
        bool rejected = false;
        try { sceNgs2SystemRender(system, &info, 1); } catch (const std::invalid_argument&) { rejected = true; }
        Require(rejected);
        Require(std::all_of(out.begin(), out.end(), [](float value) { return value == 123.0f; }));
    };
    invalid();
    info.buffer_size++;
    info.num_channels = 0;
    invalid();
    info.num_channels = 9;
    invalid();
    info.num_channels = 1;
    info.waveform_type = UINT32_MAX;
    Require(sceNgs2SystemRender(system, &info, 1) == SCE_NGS2_ERROR_INVALID_WAVEFORM_TYPE);
    Require(std::all_of(out.begin(), out.end(), [](float value) { return value == 123.0f; }));
    info.waveform_type = SCE_NGS2_WAVEFORM_TYPE_PCM_F32L;
    info.buffer = nullptr;
    Require(sceNgs2SystemRender(system, &info, 1) == SCE_NGS2_ERROR_INVALID_RENDER_BUFFER);
    Require(std::all_of(out.begin(), out.end(), [](float value) { return value == 123.0f; }));
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK);
}

static void TestAllocator() {
    const Ngs2BufferAllocator allocator{Allocate, Release, 9};
    uintptr_t system = 0;
    Require(sceNgs2SystemCreateWithAllocator(nullptr, &allocator, &system) == SCE_NGS2_OK && allocations == 1);
    uintptr_t sampler = 0;
    uintptr_t master = 0;
    Require(sceNgs2RackCreateWithAllocator(system, SCE_NGS2_RACK_ID_SAMPLER, nullptr, &allocator, &sampler) == SCE_NGS2_OK);
    Require(sceNgs2RackCreateWithAllocator(system, SCE_NGS2_RACK_ID_MASTERING, nullptr, &allocator, &master) == SCE_NGS2_OK && allocations == 3);
    Ngs2ContextBufferInfo released{};
    Require(sceNgs2RackDestroy(sampler, &released) == SCE_NGS2_OK && allocations == 2 && released.host_buffer == nullptr);
    Require(sceNgs2SystemDestroy(system, nullptr) == SCE_NGS2_OK && allocations == 0);
    Require(sceNgs2RackDestroy(master, nullptr) == SCE_NGS2_ERROR_INVALID_RACK_HANDLE);
}

static uintptr_t exitSystem = 0;

static void RenderAfterStaticTeardown() {
    RenderI16(exitSystem);
    Require(sceNgs2SystemDestroy(exitSystem, nullptr) == SCE_NGS2_OK);
}

int main() {
    Require(std::atexit(RenderAfterStaticTeardown) == 0);
    TestErrorsAndInfo();
    TestPcmBlockEnd();
    TestPatchToRackHandle();
    TestPan();
    TestPitchAndRepeat();
    TestSubmixerMatrix();
    TestReverb();
    TestReverbSetup();
    TestSampleRate();
    TestSystemOptionFlags();
    TestUserData();
    TestPcmCallbackReset();
    TestPcmCallbackRefill();
    TestPcmCallbackEvents();
    TestPcmCallbackOtherThread();
    TestPcmCallbackSetup();
    TestSetupClearsCallback();
    TestMasteringGain();
    TestMatrixLevelClamp();
    TestStereoIntoSurround();
    TestLock();
    TestRackLockLeavesQueries();
    TestVoiceCallbackLeavesQueries();
    TestRackLockWaitsForRender();
    TestCallbacksLockEachOthersSystem();
    TestRackLocksAcrossSystemsAndRender();
    TestRackDestroyWaitsForLock();
    TestDestroyDuringOwnRenderThrows();
    TestDefaultChannelMap();
    TestUnsetMatrix();

    TestResamplerKernel();
    TestResamplerMeasuredHashes();
    TestResamplerMeasuredValues();
    TestResamplerStepFormula();
    TestResamplerSystemRate();
    TestResamplerStereo();
    TestResamplerVoiceEnd();
    TestResamplerRepeats();
    TestResamplerPitchChanges();
    TestResamplerPitchAtSetup();
    TestResamplerVoiceRate();
    TestResamplerAfterEnd();

    TestRenderCapacity<float>(SCE_NGS2_WAVEFORM_TYPE_PCM_F32L, 1);
    TestRenderCapacity<float>(SCE_NGS2_WAVEFORM_TYPE_PCM_F32L, 2);
    TestRenderCapacity<std::int16_t>(SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, 1);
    TestRenderCapacity<std::int16_t>(SCE_NGS2_WAVEFORM_TYPE_PCM_I16L, 2);
    TestRenderBufferCount();
    TestRenderBufferTypeValidation();
    TestRenderBufferTypeBeforeNull();
    TestRenderBufferNull();
    TestRenderRefusedDoesNotRender();
    TestRenderBigEndianThrows();
    TestSilentRenderValidation();
    TestAllocator();
    exitSystem = CreateSystem();
    return 0;
}
