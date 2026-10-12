/**
 * @file main.cpp
 * @brief Toy robot firmware for the ESP32-S3-N16R8 with two INMP441 microphones and a MAX98357 amplifier.
 *
 * Mic audio is captured continuously and can be sent straight to the speaker (`loop on`). A USB console controls and debugs the
 * robot. The team's TensorFlow Lite model attaches to the audio stream in capture::on_audio().
 *
 * Layout (every part is a collapsible region):
 *  - Includes
 *  - Foundation: Config, Args, Utilities, Assets
 *  - Audio:      Mic, Speaker, Meter, Self-test, Capture
 *  - Board:      RGB LED
 *  - Console
 *  - Entry Point
 */

/* --------- BEGIN: Includes -------------------------------------------------------------------- */
#pragma region Includes

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <memory>
#include <numbers>
#include <optional>
#include <span>
#include <string_view>

#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_chip_info.h"
#include "esp_console.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "led_strip.h"

#pragma endregion  // Includes
/* --------- END: Includes ---------------------------------------------------------------------- */


/* --------- BEGIN: Foundation ------------------------------------------------------------------ */
#pragma region Foundation

/* ============================================================================================== */
/* --------- BEGIN: Config ---------------------------------------------------------------------- */
/* ============================================================================================== */
#pragma region Config

/**
 * @struct TaskConfig
 * @brief Name, stack size, priority and CPU core of one FreeRTOS task.
 */
struct TaskConfig {
    const char *name;
    uint32_t    stack_bytes;
    UBaseType_t priority;
    BaseType_t  core;
};

/**
 * @struct AudioConfig
 * @brief Sample rate and chunk size shared by the mics and the speaker.
 */
struct AudioConfig {
    int    sample_rate_hz;
    size_t chunk_frames;  // frames per capture read and per speaker write (16 ms at 16 kHz)
};

/**
 * @struct MicConfig
 * @brief I2S port, pins and signal conditioning of the two INMP441 microphones.
 */
struct MicConfig {
    int        port;  // I2S_NUM_0 or I2S_NUM_1
    gpio_num_t sck_gpio;
    gpio_num_t ws_gpio;
    gpio_num_t sd_gpio;
    int        channels;         // two INMP441s: L/R=GND -> left, L/R=3V3 -> right
    int        gain_shift;       // 32-bit slot -> 16-bit PCM: 16 is unity, lower is louder (14 = +12 dB)
    float      dc_blocker_pole;  // closer to 1.0 = lower corner frequency (0.995 is about 13 Hz at 16 kHz)
};

/**
 * @struct SpeakerConfig
 * @brief I2S port, pins, default volume and loopback limits of the MAX98357 amplifier.
 */
struct SpeakerConfig {
    int        port;  // I2S_NUM_0 or I2S_NUM_1
    gpio_num_t bclk_gpio;
    gpio_num_t lrc_gpio;
    gpio_num_t din_gpio;
    int        default_volume;      // percent
    int        fade_samples;        // ramp in/out of a tone to avoid clicks
    int        note_gap_ms;         // rest after each note of a song so repeated notes stay distinct
    int        tone_queue_depth;    // room for a whole melody
    int        loop_buffer_chunks;  // most mic audio that may wait for the speaker; keeps the delay low
    int        feedback_chunks;     // consecutive loud loopback chunks that count as feedback (12 = about 190 ms)
    float      feedback_level;      // RMS, fraction of full scale
};

/**
 * @struct MeterConfig
 * @brief Timing of the live level stream used for plotting.
 */
struct MeterConfig {
    int period_ms;  // one CSV row per period
};

/**
 * @struct SelfTestConfig
 * @brief Block length, thresholds and probe tone of the boot-time hardware check.
 */
struct SelfTestConfig {
    size_t frames;            // block length; 4096 frames = 256 ms
    float  loud_room_db;      // a quiet room reads about -60 or lower
    int    min_span;          // below this a channel counts as missing
    float  min_tone_rise_db;  // how much louder the probe tone must get to count as heard
    float  probe_hz;
    int    probe_ms;
    int    timeout_ms;  // how long `check` waits for the capture task
};

/**
 * @struct LedConfig
 * @brief Candidate data pins of the board's RGB LED.
 */
struct LedConfig {
    std::array<int, 2> gpios;  // the board's WS2812 is on GPIO 48 (v1.0 boards) or 38 (v1.1)
};

/**
 * @struct ConsoleConfig
 * @brief Look of the USB console.
 */
struct ConsoleConfig {
    const char *prompt;
};

/**
 * @class Config
 * @brief Every tunable value in one place.
 *
 * Pins and defaults come from `idf.py menuconfig` (Toy Robot menu); timing, sizes, thresholds and task settings are constants
 * here. Read-only: runtime state (volume, loop on/off) lives in its module.
 */
class Config {
public:
    Config() = delete;  // constants only

    static constexpr AudioConfig audio = {
        .sample_rate_hz = 16000,
        .chunk_frames   = 256,
    };

    static constexpr MicConfig mic = {
        .port            = I2S_NUM_0,
        .sck_gpio        = static_cast<gpio_num_t>(CONFIG_MIC_I2S_SCK_GPIO),
        .ws_gpio         = static_cast<gpio_num_t>(CONFIG_MIC_I2S_WS_GPIO),
        .sd_gpio         = static_cast<gpio_num_t>(CONFIG_MIC_I2S_SD_GPIO),
        .channels        = 2,
        .gain_shift      = CONFIG_MIC_GAIN_SHIFT,
        .dc_blocker_pole = 0.995f,
    };

    static constexpr SpeakerConfig speaker = {
        .port               = I2S_NUM_1,
        .bclk_gpio          = static_cast<gpio_num_t>(CONFIG_SPK_I2S_BCLK_GPIO),
        .lrc_gpio           = static_cast<gpio_num_t>(CONFIG_SPK_I2S_LRC_GPIO),
        .din_gpio           = static_cast<gpio_num_t>(CONFIG_SPK_I2S_DIN_GPIO),
        .default_volume     = CONFIG_SPK_VOLUME,
        .fade_samples       = 80,
        .note_gap_ms        = 40,
        .tone_queue_depth   = 128,
        .loop_buffer_chunks = 3,
        .feedback_chunks    = 12,
        .feedback_level     = 0.35f,
    };

    static constexpr MeterConfig meter = {
        .period_ms = 50,
    };

    static constexpr SelfTestConfig selftest = {
        .frames           = 4096,
        .loud_room_db     = -45.0f,
        .min_span         = 8,
        .min_tone_rise_db = 10.0f,
        .probe_hz         = 1000.0f,
        .probe_ms         = 400,
        .timeout_ms       = 5000,
    };

    static constexpr LedConfig led = {
        .gpios = {48, 38},
    };

    static constexpr ConsoleConfig console = {
        .prompt = "robot> ",
    };

    // FreeRTOS tasks. Stacks come from internal RAM, which is scarce, so keep them small.
    static constexpr TaskConfig capture_task = {.name = "capture", .stack_bytes = 6 * 1024, .priority = 5, .core = 1};
    static constexpr TaskConfig speaker_task = {.name = "speaker", .stack_bytes = 5 * 1024, .priority = 4, .core = 0};
    static constexpr TaskConfig meter_task   = {.name = "meter", .stack_bytes = 3 * 1024, .priority = 2, .core = 0};
};

#pragma endregion  // Config
/* --------- END: Config ------------------------------------------------------------------------ */


/* ============================================================================================== */
/* --------- BEGIN: Args ------------------------------------------------------------------------ */
/* ============================================================================================== */
#pragma region Args

// Usage text of a registered command ("" when it takes no arguments); the command table in `console` defines it
namespace console {
const char *syntax_of(std::string_view command);
}

/**
 * @class Args
 * @brief Command-line argument checks, like middleware.
 *
 * Each check validates, and on failure prints what was wrong plus the usage line and returns false / std::nullopt. A
 * std::nullopt result therefore always means "error, stop". To allow an argument to be left out, pass a fallback (get_*) or test
 * has() first. Indexes are argv indexes: 1 is the first argument after the command.
 */
class Args {
public:
    /**
     * @brief Wraps the command line exactly as the console hands it over.
     * @param argc Number of entries in argv, the command name included.
     * @param argv The command name followed by its arguments.
     */
    Args(int argc, char **argv) : argc_(argc), argv_(argv) {}

    /**
     * @brief Enforces how many arguments (not counting the command itself) may be given.
     * @param min_args Fewest arguments allowed.
     * @param max_args Most arguments allowed.
     * @return true when the count is within range; otherwise prints the problem and the usage line and returns false.
     */
    bool check_count(int min_args, int max_args) const {
        const int given = argc_ - 1;
        if (given >= min_args && given <= max_args) { return true; }
        std::printf("wrong number of arguments (%d given)\n", given);
        print_usage();
        return false;
    }

    /**
     * @brief Tells whether an argument was given.
     * @param index argv index (1 is the first argument).
     */
    bool has(int index) const { return index < argc_; }

    /**
     * @brief Reads a whole number between min_val and max_val.
     *
     * Rejects text like "abc" or "12x", which atoi would quietly read as 0 or 12.
     *
     * @param index    argv index.
     * @param min_val  Smallest accepted value.
     * @param max_val  Largest accepted value.
     * @param fallback Value to use when the argument is left out; without one, a missing argument is an error.
     * @return The number, or std::nullopt after printing the problem.
     */
    std::optional<int> get_int(int index, int min_val, int max_val, std::optional<int> fallback = std::nullopt) const {
        if (!has(index)) { return missing(index, fallback); }
        char *end        = nullptr;
        errno            = 0;
        const long value = std::strtol(argv_[index], &end, 10);
        if (end == argv_[index] || *end != '\0' || errno == ERANGE || value < min_val || value > max_val) {
            std::printf("'%s' is not a whole number from %d to %d\n", argv_[index], min_val, max_val);
            print_usage();
            return std::nullopt;
        }
        return static_cast<int>(value);
    }

    /**
     * @brief Reads one of the listed words (exact match, case-sensitive).
     * @param index    argv index.
     * @param choices  The accepted words.
     * @param fallback Word to use when the argument is left out; without one, a missing argument is an error.
     * @return The matching word, or std::nullopt after printing the problem.
     */
    std::optional<std::string_view> get_choice(int index, std::initializer_list<std::string_view> choices,
                                               std::optional<std::string_view> fallback = std::nullopt) const {
        if (!has(index)) { return missing(index, fallback); }
        const std::string_view arg = argv_[index];
        for (const std::string_view choice : choices) {
            if (arg == choice) { return choice; }
        }
        std::printf("'%s' is not one of:", argv_[index]);
        for (const std::string_view choice : choices) { std::printf(" %.*s", static_cast<int>(choice.size()), choice.data()); }
        std::printf("\n");
        print_usage();
        return std::nullopt;
    }

    /**
     * @brief Reads any word.
     * @param index    argv index.
     * @param fallback Word to use when the argument is left out; without one, a missing argument is an error.
     */
    std::optional<std::string_view> get_string(int index, std::optional<std::string_view> fallback = std::nullopt) const {
        if (!has(index)) { return missing(index, fallback); }
        return std::string_view(argv_[index]);
    }

    /** @brief Prints `usage: <command> <syntax>`, with the syntax taken from the console's command table. */
    void print_usage() const {
        const char *syntax = console::syntax_of(argv_[0]);
        std::printf("usage: %s%s%s\n", argv_[0], syntax[0] != '\0' ? " " : "", syntax);
    }

private:
    // An argument that was left out: fine when there is a fallback, an error when there is not
    template <typename T> std::optional<T> missing(int index, const std::optional<T> &fallback) const {
        if (!fallback) {
            std::printf("missing argument %d\n", index);
            print_usage();
        }
        return fallback;
    }

    int    argc_;
    char **argv_;
};

#pragma endregion  // Args
/* --------- END: Args -------------------------------------------------------------------------- */


/* ============================================================================================== */
/* --------- BEGIN: Utilities ------------------------------------------------------------------- */
/* ============================================================================================== */
#pragma region Utilities

/**
 * @brief Starts a pinned FreeRTOS task from its config.
 * @param task Name, stack size, priority and core.
 * @param fn   Task entry point.
 * @note A failure here means internal RAM ran out; it is logged, not fatal.
 */
void spawn(const TaskConfig &task, TaskFunction_t fn) {
    if (xTaskCreatePinnedToCore(fn, task.name, task.stack_bytes, nullptr, task.priority, nullptr, task.core) != pdPASS) {
        ESP_LOGE("task", "could not start %s: out of internal RAM", task.name);
    }
}

/**
 * @brief Logs free internal RAM and PSRAM.
 *
 * Internal RAM is the scarce resource on this board; log it at key points to see who takes it.
 *
 * @param where Label printed in front of the numbers.
 */
void log_heap(const char *where) {
    ESP_LOGI("heap", "%-16s internal %3u KB free (largest %3u KB), PSRAM %4u KB free", where,
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) >> 10),
             static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) >> 10),
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) >> 10));
}

#pragma endregion  // Utilities
/* --------- END: Utilities --------------------------------------------------------------------- */


/* ============================================================================================== */
/* --------- BEGIN: Assets ---------------------------------------------------------------------- */
/* ============================================================================================== */
#pragma region Assets

/**
 * @struct Note
 * @brief One tone of a melody.
 */
struct Note {
    uint16_t freq_hz;  // 0 = rest
    uint16_t ms;
};

/**
 * @struct Melody
 * @brief A named sequence of notes.
 */
struct Melody {
    std::string_view      name;  // what `play <name>` takes
    std::span<const Note> notes;
};

/**
 * @class Assets
 * @brief Everything the robot plays.
 *
 * Songs, melodies and status cues are lists of notes. Add a melody here and list it in `melodies`, and `play <name>` finds it.
 * Longer recorded clips (PCM) would also be stored here.
 */
class Assets {
public:
    Assets() = delete;  // constants only

    // Pitches in Hz (equal temperament, rounded) and note lengths in ms
    static constexpr uint16_t C4      = 262;
    static constexpr uint16_t D4      = 294;
    static constexpr uint16_t E4      = 330;
    static constexpr uint16_t F4      = 349;
    static constexpr uint16_t G4      = 392;
    static constexpr uint16_t A4      = 440;
    static constexpr uint16_t B4      = 494;
    static constexpr uint16_t C5      = 523;
    static constexpr uint16_t eighth  = 140;
    static constexpr uint16_t quarter = 280;
    static constexpr uint16_t half    = 560;

    // Wordless status cues
    static constexpr Note chirp[] = {{880, 70}, {1320, 90}};  // ready: every check passed
    static constexpr Note alert[] = {{220, 600}};             // something failed

    // Songs
    static constexpr Note scale[]   = {{C4, quarter}, {D4, quarter}, {E4, quarter}, {F4, quarter},
                                       {G4, quarter}, {A4, quarter}, {B4, quarter}, {C5, half}};
    static constexpr Note twinkle[] = {{C4, quarter}, {C4, quarter}, {G4, quarter}, {G4, quarter}, {A4, quarter},
                                       {A4, quarter}, {G4, half},    {F4, quarter}, {F4, quarter}, {E4, quarter},
                                       {E4, quarter}, {D4, quarter}, {D4, quarter}, {C4, half}};
    static constexpr Note ode[]     = {{E4, quarter}, {E4, quarter}, {F4, quarter},          {G4, quarter}, {G4, quarter},
                                       {F4, quarter}, {E4, quarter}, {D4, quarter},          {C4, quarter}, {C4, quarter},
                                       {D4, quarter}, {E4, quarter}, {E4, quarter + eighth}, {D4, eighth},  {D4, half}};
    // Low to high in steps, to hear where the speaker is weak, rattles or distorts
    static constexpr Note sweep[] = {{200, 400},  {300, 400},  {400, 400},  {600, 400},  {800, 400}, {1000, 400},
                                     {1500, 400}, {2000, 400}, {3000, 400}, {4000, 400}, {6000, 400}};

    static constexpr Melody melodies[] = {{"scale", scale}, {"twinkle", twinkle}, {"ode", ode}, {"sweep", sweep}};

    /**
     * @brief Looks a melody up by name.
     * @param name What `play` was given.
     * @return The melody, or nullptr when there is none with that name.
     */
    static const Melody *find(std::string_view name) {
        for (const Melody &melody : melodies) {
            if (melody.name == name) { return &melody; }
        }
        return nullptr;
    }
};

#pragma endregion  // Assets
/* --------- END: Assets ------------------------------------------------------------------------ */

#pragma endregion  // Foundation
/* --------- END: Foundation -------------------------------------------------------------------- */


/* --------- BEGIN: Audio ----------------------------------------------------------------------- */
#pragma region Audio

/* ============================================================================================== */
/* --------- BEGIN: Mic ------------------------------------------------------------------------- */
/* ============================================================================================== */
#pragma region Mic

/**
 * @namespace mic
 * @brief Two INMP441 microphones on one I2S port.
 */
namespace mic {

constexpr int kChannels = Config::mic.channels;

i2s_chan_handle_t rx = nullptr;

/** @brief Configures the I2S port as a stereo receiver for both microphones and starts it. */
void init() {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(Config::mic.port, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, nullptr, &rx));

    // clang-format off
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(Config::audio.sample_rate_hz),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = Config::mic.sck_gpio,
            .ws = Config::mic.ws_gpio,
            .dout = I2S_GPIO_UNUSED,
            .din = Config::mic.sd_gpio,
            .invert_flags = {},
        },
    };
    // clang-format on
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx));
}

/**
 * @struct DcBlocker
 * @brief One-pole DC blocker: y = x - x_prev + R*y_prev.
 *
 * The INMP441 carries a large DC offset (about half of 16-bit full scale here) that wastes headroom and is useless to anything
 * downstream.
 */
struct DcBlocker {
    float x1 = 0.0f;
    float y1 = 0.0f;

    /** @brief Filters one sample. */
    float operator()(float x) {
        const float y = x - x1 + Config::mic.dc_blocker_pole * y1;
        x1            = x;
        y1            = y;
        return y;
    }
};

DcBlocker dc_blockers[kChannels];

/**
 * @brief Converts one raw sample to 16-bit PCM: scales it by the gain shift and removes the DC offset.
 *
 * The INMP441 sends 24-bit samples left-aligned in 32-bit slots. Samples must be fed in order, once, per channel.
 *
 * @param raw     Sample as read from I2S.
 * @param channel 0 for the left mic, 1 for the right.
 * @return The 16-bit sample, clamped to the 16-bit range.
 */
int16_t to_pcm(int32_t raw, int channel) {
    constexpr float kScale = 1.0f / (1 << Config::mic.gain_shift);
    const float     y      = dc_blockers[channel](static_cast<float>(raw) * kScale);
    return static_cast<int16_t>(std::clamp<int32_t>(static_cast<int32_t>(y), INT16_MIN, INT16_MAX));
}

/**
 * @brief Reads stereo frames exactly as the mics send them (interleaved L,R 32-bit slots). Blocks until all are read.
 * @param out    Room for frames * 2 samples.
 * @param frames Number of stereo frames to read.
 */
void read_raw(int32_t *out, size_t frames) {
    size_t bytes_read = 0;
    ESP_ERROR_CHECK(i2s_channel_read(rx, out, frames * kChannels * sizeof(int32_t), &bytes_read, portMAX_DELAY));
}

/**
 * @brief Reads stereo frames converted to interleaved L,R 16-bit samples. Blocks until all are read.
 * @param out    Room for frames * 2 samples.
 * @param frames Number of stereo frames to read.
 * @note Only the capture task may call this.
 */
void read(int16_t *out, size_t frames) {
    const size_t samples = frames * kChannels;

    // The chunk size never changes, so this only allocates once
    static std::unique_ptr<int32_t[]> raw;
    static size_t                     raw_len = 0;
    if (raw_len < samples) {
        raw.reset(new int32_t[samples]);
        raw_len = samples;
    }

    read_raw(raw.get(), frames);
    for (size_t i = 0; i < samples; ++i) { out[i] = to_pcm(raw[i], static_cast<int>(i % kChannels)); }
}

}  // namespace mic

#pragma endregion  // Mic
/* --------- END: Mic --------------------------------------------------------------------------- */


/* ============================================================================================== */
/* --------- BEGIN: Speaker --------------------------------------------------------------------- */
/* ============================================================================================== */
#pragma region Speaker

/**
 * @namespace speaker
 * @brief MAX98357 amplifier on its own I2S port.
 *
 * One task mixes queued notes and the live mic loopback under a single master volume.
 */
namespace speaker {

constexpr const char *TAG = "speaker";

/** @brief Which microphone(s) feed the loopback. */
enum class Source { Both, Left, Right };

constexpr size_t kBlock = Config::audio.chunk_frames;  // samples per I2S write
constexpr float  kTwoPi = 2.0f * std::numbers::pi_v<float>;

i2s_chan_handle_t    tx         = nullptr;
QueueHandle_t        notes      = nullptr;  // Note, played in order
StreamBufferHandle_t loop_audio = nullptr;  // mono 16-bit mic audio on its way to the speaker

std::atomic<int>    volume{Config::speaker.default_volume};  // master volume, percent
std::atomic<bool>   loopback{false};
std::atomic<Source> loop_source{Source::Both};

/**
 * @brief Writes mono 16-bit samples to the amplifier. Blocks until the I2S buffer has room.
 * @param buf     Samples.
 * @param samples Number of samples.
 */
void write(const int16_t *buf, size_t samples) {
    size_t written = 0;
    i2s_channel_write(tx, buf, samples * sizeof(int16_t), &written, portMAX_DELAY);
}

/**
 * @brief Speaker task: mixes the queued notes and the mic loopback, applies the master volume and writes to the amplifier.
 *
 * Also runs the feedback guard, which switches the loopback off when the mics keep picking up the speaker.
 *
 * @param arg Unused.
 */
void task(void *) {
    int16_t out[kBlock];
    int16_t mic_block[kBlock];
    float   mix[kBlock];
    Note    note       = {};
    int     note_total = 0;
    int     note_pos   = 0;
    float   phase      = 0.0f;
    int     hot_blocks = 0;

    while (true) {
        const bool loop = loopback.load();

        // Start the next queued note once the current one has finished; sleep on the queue when there is nothing else to do
        if (note_pos >= note_total && xQueueReceive(notes, &note, loop ? 0 : pdMS_TO_TICKS(50)) == pdTRUE) {
            note_total = Config::audio.sample_rate_hz * note.ms / 1000;
            note_pos   = 0;
            phase      = 0.0f;
        }
        const bool playing = note_pos < note_total;
        if (!playing && !loop) { continue; }

        std::fill(std::begin(mix), std::end(mix), 0.0f);

        // Loopback is paced by the mics: one block arrives every 16 ms
        float loop_sum_sq = 0.0f;
        if (loop) {
            if (xStreamBufferReceive(loop_audio, mic_block, sizeof(mic_block), pdMS_TO_TICKS(30)) == sizeof(mic_block)) {
                for (size_t i = 0; i < kBlock; ++i) {
                    mix[i] = mic_block[i];
                    loop_sum_sq += mix[i] * mix[i];
                }
            } else if (!playing) {
                continue;
            }
        }

        if (playing) {
            const float step = kTwoPi * note.freq_hz / Config::audio.sample_rate_hz;
            for (size_t i = 0; i < kBlock && note_pos < note_total; ++i, ++note_pos) {
                const int   edge = std::min(note_pos, note_total - 1 - note_pos);
                const float env  = edge < Config::speaker.fade_samples ? static_cast<float>(edge) / Config::speaker.fade_samples : 1.0f;
                if (note.freq_hz != 0) { mix[i] += 32767.0f * env * std::sin(phase); }
                phase += step;
                if (phase > kTwoPi) { phase -= kTwoPi; }
            }
        }

        const float gain = volume.load() / 100.0f;
        for (size_t i = 0; i < kBlock; ++i) { out[i] = static_cast<int16_t>(std::clamp(mix[i] * gain, -32767.0f, 32767.0f)); }
        write(out, kBlock);

        // Feedback guard: judge only the mic part, so a loud beep or song never trips it
        const float loop_rms = std::sqrt(loop_sum_sq / kBlock) * gain / 32768.0f;
        hot_blocks           = (loop && loop_rms > Config::speaker.feedback_level) ? hot_blocks + 1 : 0;
        if (hot_blocks >= Config::speaker.feedback_chunks) {
            loopback.store(false);
            hot_blocks = 0;
            ESP_LOGW(TAG, "feedback: the mics are picking up the speaker, so loop was turned off (lower the volume or move them apart)");
        }
    }
}

/** @brief Configures the I2S port as a mono transmitter, creates the note queue and loopback buffer, and starts the speaker task. */
void init() {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(Config::speaker.port, I2S_ROLE_MASTER);
    chan_cfg.auto_clear        = true;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx, nullptr));

    // clang-format off
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(Config::audio.sample_rate_hz),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = Config::speaker.bclk_gpio,
            .ws = Config::speaker.lrc_gpio,
            .dout = Config::speaker.din_gpio,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {},
        },
    };
    // clang-format on
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx));

    notes = xQueueCreate(Config::speaker.tone_queue_depth, sizeof(Note));
    // A few chunks at most: keeps the mic -> speaker delay low, and a late speaker task just drops chunks
    loop_audio = xStreamBufferCreate(Config::speaker.loop_buffer_chunks * kBlock * sizeof(int16_t), kBlock * sizeof(int16_t));
    spawn(Config::speaker_task, task);
}

/**
 * @brief Queues one tone and returns immediately.
 * @param freq_hz     Frequency; 0 is a rest.
 * @param duration_ms Length in milliseconds.
 */
void tone(uint16_t freq_hz, uint16_t duration_ms) {
    if (notes == nullptr) { return; }
    const Note note = {freq_hz, duration_ms};
    xQueueSend(notes, &note, 0);
}

/**
 * @brief Queues a sequence of notes, with an optional rest after each.
 * @param sequence The notes, for example Assets::chirp or a Melody's notes.
 * @param gap_ms   Rest after each note in milliseconds; 0 for none.
 */
void play(std::span<const Note> sequence, int gap_ms = 0) {
    for (const Note &note : sequence) {
        tone(note.freq_hz, note.ms);
        if (gap_ms > 0) { tone(0, gap_ms); }
    }
}

/**
 * @brief Hands a chunk of microphone audio to the speaker task when the loopback is on.
 *
 * Mixes to mono according to loop_source and never blocks: when the speaker side is behind, the chunk is dropped.
 *
 * @param stereo Interleaved L,R 16-bit samples.
 * @param frames Number of stereo frames, at most kBlock.
 * @note Called by the capture task with every chunk.
 */
void feed_loopback(const int16_t *stereo, size_t frames) {
    if (!loopback.load(std::memory_order_relaxed)) { return; }
    configASSERT(frames <= kBlock);
    int16_t      mono[kBlock];
    const Source source = loop_source.load();
    for (size_t i = 0; i < frames; ++i) {
        const int left  = stereo[2 * i];
        const int right = stereo[2 * i + 1];
        mono[i]         = static_cast<int16_t>(source == Source::Left ? left : source == Source::Right ? right : (left + right) / 2);
    }
    // Never block the capture task: when the speaker side is behind, drop the chunk
    if (xStreamBufferSpacesAvailable(loop_audio) >= sizeof(mono)) { xStreamBufferSend(loop_audio, mono, sizeof(mono), 0); }
}

/**
 * @brief Switches the mic -> speaker loopback on or off.
 * @param on true to start passing mic audio to the speaker.
 */
void set_loopback(bool on) {
    if (on) {
        xStreamBufferReset(loop_audio);  // start from live audio, not whatever was left over
    }
    loopback.store(on);
}

}  // namespace speaker

#pragma endregion  // Speaker
/* --------- END: Speaker ----------------------------------------------------------------------- */


/* ============================================================================================== */
/* --------- BEGIN: Meter ----------------------------------------------------------------------- */
/* ============================================================================================== */
#pragma region Meter

/**
 * @namespace meter
 * @brief Live mic levels as CSV rows for host-side plotting.
 *
 * `plot on` starts the stream; scripts/mic_log.py saves it and scripts/mic_plot.sh draws it. Off by default so the console stays
 * readable.
 */
namespace meter {

constexpr const char *kHeader = "time_ms,l_db,r_db,l_peak,r_peak,l_dc,r_dc,loop";

std::atomic<bool> streaming{false};

portMUX_TYPE lock                   = portMUX_INITIALIZER_UNLOCKED;
int64_t      sum[mic::kChannels]    = {};
int64_t      sum_sq[mic::kChannels] = {};
int          peak[mic::kChannels]   = {};
uint32_t     count                  = 0;

/**
 * @brief Adds one chunk of mic audio to the running level statistics; does nothing unless streaming.
 * @param x      Interleaved L,R 16-bit samples.
 * @param frames Number of stereo frames.
 * @note Called by the capture task with every chunk.
 */
void feed(const int16_t *x, size_t frames) {
    if (!streaming.load(std::memory_order_relaxed)) { return; }
    int64_t s[mic::kChannels]  = {};
    int64_t sq[mic::kChannels] = {};
    int     pk[mic::kChannels] = {};
    for (size_t i = 0; i < frames; ++i) {
        for (int c = 0; c < mic::kChannels; ++c) {
            const int v = x[i * mic::kChannels + c];
            s[c] += v;
            sq[c] += static_cast<int64_t>(v) * v;
            pk[c] = std::max(pk[c], std::abs(v));
        }
    }
    taskENTER_CRITICAL(&lock);
    for (int c = 0; c < mic::kChannels; ++c) {
        sum[c] += s[c];
        sum_sq[c] += sq[c];
        peak[c] = std::max(peak[c], pk[c]);
    }
    count += frames;
    taskEXIT_CRITICAL(&lock);
}

/**
 * @brief Meter task: every period prints one CSV row (level, peak and DC of each mic, loopback state) while streaming.
 * @param arg Unused.
 */
void task(void *) {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(Config::meter.period_ms));
        if (!streaming.load()) { continue; }

        // Take everything accumulated since the last row and start over
        int64_t s[mic::kChannels];
        int64_t sq[mic::kChannels];
        int     pk[mic::kChannels];
        taskENTER_CRITICAL(&lock);
        const uint32_t n = count;
        for (int c = 0; c < mic::kChannels; ++c) {
            s[c]   = sum[c];
            sq[c]  = sum_sq[c];
            pk[c]  = peak[c];
            sum[c] = sum_sq[c] = 0;
            peak[c]            = 0;
        }
        count = 0;
        taskEXIT_CRITICAL(&lock);
        if (n == 0) {
            continue;  // the mics are not being read right now (hardware check running)
        }

        float db[mic::kChannels];
        int   dc[mic::kChannels];
        for (int c = 0; c < mic::kChannels; ++c) {
            const double mean = static_cast<double>(s[c]) / n;
            const double rms  = std::sqrt(std::max(static_cast<double>(sq[c]) / n - mean * mean, 0.0));
            db[c]             = rms > 0.0 ? static_cast<float>(20.0 * std::log10(rms / 32768.0)) : -120.0f;
            dc[c]             = static_cast<int>(mean);
        }

        std::printf("%lld,%.1f,%.1f,%d,%d,%d,%d,%d\n", static_cast<long long>(esp_timer_get_time() / 1000), db[0], db[1], pk[0], pk[1],
                    dc[0], dc[1], speaker::loopback.load() ? 1 : 0);
    }
}

/** @brief Starts the meter task. */
void start() {
    spawn(Config::meter_task, task);
}

/**
 * @brief Starts or stops the CSV stream; starting prints the header line first.
 * @param on true to stream.
 */
void set_streaming(bool on) {
    if (on) { std::printf("%s\n", kHeader); }
    streaming.store(on);
}

}  // namespace meter

#pragma endregion  // Meter
/* --------- END: Meter ------------------------------------------------------------------------- */


/* ============================================================================================== */
/* --------- BEGIN: Self-test ------------------------------------------------------------------- */
/* ============================================================================================== */
#pragma region Self-test

/**
 * @namespace selftest
 * @brief Boot-time check: are both mics alive, and does the speaker/amp work?
 *
 * A connected, powered INMP441 always shows a small noise floor, even in a silent room; an unplugged or unpowered one reads
 * flat. The MAX98357 has no feedback line, so the speaker is checked acoustically: play a tone and see whether the mics hear it.
 */
namespace selftest {

/** @struct Stats @brief Measurements of one mic channel over a block. */
struct Stats {
    float rms_db;   // dBFS, DC removed
    int   span;     // max - min in 16-bit counts; ~0 when the mic is not delivering data
    float tone_db;  // level of the probe tone in the block, dBFS
    int   dc;       // mean, 16-bit counts
};

/** @struct Result @brief Outcome of the last hardware check. */
struct Result {
    bool    done                   = false;
    bool    alive[mic::kChannels]  = {};
    Stats   quiet[mic::kChannels]  = {};
    int32_t raw[mic::kChannels][6] = {};       // a few samples straight off the wire, for diagnosing wiring
    float   tone_rise_db           = -120.0f;  // best rise of the probe tone over the room noise, across working mics
    bool    speaker_heard          = false;
};

constexpr const char *kChannelName[mic::kChannels] = {"mic A (left, L/R->GND)", "mic B (right, L/R->3V3)"};

Result result;

/**
 * @brief Level of a single frequency in one channel of an interleaved block (Goertzel algorithm).
 * @param x       Interleaved samples.
 * @param frames  Number of stereo frames.
 * @param ch      Channel to measure.
 * @param freq_hz Frequency to measure.
 * @return Level in dBFS.
 */
float tone_db(const int16_t *x, size_t frames, int ch, float freq_hz) {
    const float coeff = 2.0f * std::cos(2.0f * std::numbers::pi_v<float> * freq_hz / Config::audio.sample_rate_hz);
    float       s1    = 0.0f;
    float       s2    = 0.0f;
    for (size_t i = 0; i < frames; ++i) {
        const float s = x[i * mic::kChannels + ch] + coeff * s1 - s2;
        s2            = s1;
        s1            = s;
    }
    const float power     = s1 * s1 + s2 * s2 - coeff * s1 * s2;
    const float amplitude = 2.0f * std::sqrt(std::max(power, 0.0f)) / frames;
    return 20.0f * std::log10(amplitude / 32768.0f + 1e-9f);
}

/**
 * @brief Measures noise level, span, DC and probe-tone level of one channel.
 * @param x      Interleaved samples.
 * @param frames Number of stereo frames.
 * @param ch     Channel to measure.
 */
Stats measure(const int16_t *x, size_t frames, int ch) {
    int    lo     = INT16_MAX;
    int    hi     = INT16_MIN;
    double sum    = 0.0;
    double sum_sq = 0.0;
    for (size_t i = 0; i < frames; ++i) {
        const int v = x[i * mic::kChannels + ch];
        lo          = std::min(lo, v);
        hi          = std::max(hi, v);
        sum += v;
        sum_sq += static_cast<double>(v) * v;
    }
    const double mean   = sum / frames;
    const double rms    = std::sqrt(std::max(sum_sq / frames - mean * mean, 0.0));
    const float  rms_db = rms > 0.0 ? static_cast<float>(20.0 * std::log10(rms / 32768.0)) : -120.0f;
    return {rms_db, hi - lo, tone_db(x, frames, ch, Config::selftest.probe_hz), static_cast<int>(mean)};
}

/** @brief Prints the report of the last check, with hints for whatever failed. */
void print() {
    if (!result.done) {
        std::printf("self-test has not run yet\n");
        return;
    }
    std::printf("\n=== hardware check ===\n");
    bool any_mic = false;
    for (int c = 0; c < mic::kChannels; ++c) {
        any_mic |= result.alive[c];
        std::printf("%-24s %s  noise %6.1f dBFS, span %d, dc %d\n", kChannelName[c], result.alive[c] ? "OK     " : "MISSING",
                    result.quiet[c].rms_db, result.quiet[c].span, result.quiet[c].dc);
        std::printf("    raw:");
        for (const int32_t v : result.raw[c]) { std::printf(" %08lx", static_cast<unsigned long>(static_cast<uint32_t>(v))); }
        std::printf("\n");
        if (!result.alive[c]) {
            std::printf("    -> check VDD=3V3, GND, SD/SCK/WS wires and the L/R pin of this mic\n");
        } else if (result.quiet[c].rms_db > Config::selftest.loud_room_db) {
            std::printf("    -> far too loud for a quiet room (normal is -60 or lower): noisy surroundings, or junk on the "
                        "I2S wires (loose/floating SD, two mics on the same L/R setting, long wires)\n");
        }
    }
    if (!any_mic) {
        std::printf("speaker + amp            unknown (needs a working mic to listen)\n");
    } else if (result.speaker_heard) {
        std::printf("speaker + amp            OK       mics heard the %.0f Hz test tone (+%.0f dB)\n", Config::selftest.probe_hz,
                    result.tone_rise_db);
    } else {
        std::printf("speaker + amp            NOT HEARD (+%.0f dB)\n", result.tone_rise_db);
        std::printf("    -> check amp VIN=5V, GND, LRC/BCLK/DIN wires and the speaker leads; the mics may also be too far, or the "
                    "volume too low\n");
    }
    std::printf("======================\n\n");
}

/**
 * @brief Runs the hardware check: listens to the quiet room, plays a probe tone and listens again, then prints the report and
 *        plays a verdict (chirp = all good, low beep = something failed).
 * @note Needs the mic I2S running and the speaker task started. Must run inside the capture task, which owns the mics.
 */
void run() {
    constexpr size_t kFrames = Config::selftest.frames;
    auto             buf     = std::make_unique<int16_t[]>(kFrames * mic::kChannels);

    // INMP441 needs ~85 ms to start and the DMA still holds stale data, so throw the first blocks away
    mic::read(buf.get(), kFrames);
    mic::read(buf.get(), kFrames);

    auto raw = std::make_unique<int32_t[]>(kFrames * mic::kChannels);
    mic::read_raw(raw.get(), kFrames);
    for (size_t i = 0; i < kFrames * mic::kChannels; ++i) { buf[i] = mic::to_pcm(raw[i], static_cast<int>(i % mic::kChannels)); }
    for (int c = 0; c < mic::kChannels; ++c) {
        result.quiet[c] = measure(buf.get(), kFrames, c);
        result.alive[c] = result.quiet[c].span >= Config::selftest.min_span;
        for (int i = 0; i < 6; ++i) { result.raw[c][i] = raw[(1000 + i) * mic::kChannels + c]; }
    }

    // Probe tone; two blocks cover its whole length plus the playback latency
    speaker::tone(static_cast<uint16_t>(Config::selftest.probe_hz), Config::selftest.probe_ms);
    float best[mic::kChannels] = {-120.0f, -120.0f};
    for (int pass = 0; pass < 2; ++pass) {
        mic::read(buf.get(), kFrames);
        for (int c = 0; c < mic::kChannels; ++c) { best[c] = std::max(best[c], tone_db(buf.get(), kFrames, c, Config::selftest.probe_hz)); }
    }
    result.tone_rise_db = -120.0f;
    for (int c = 0; c < mic::kChannels; ++c) {
        if (result.alive[c]) { result.tone_rise_db = std::max(result.tone_rise_db, best[c] - result.quiet[c].tone_db); }
    }
    result.speaker_heard = result.tone_rise_db >= Config::selftest.min_tone_rise_db;
    result.done          = true;
    print();

    // Audible verdict
    speaker::play(result.alive[0] && result.alive[1] && result.speaker_heard ? std::span<const Note>(Assets::chirp)
                                                                             : std::span<const Note>(Assets::alert));
}

}  // namespace selftest

#pragma endregion  // Self-test
/* --------- END: Self-test --------------------------------------------------------------------- */


/* ============================================================================================== */
/* --------- BEGIN: Capture --------------------------------------------------------------------- */
/* ============================================================================================== */
#pragma region Capture

/**
 * @namespace capture
 * @brief The one task that reads the mics, and the single place audio fans out from.
 */
namespace capture {

constexpr size_t      kChunk     = Config::audio.chunk_frames;
constexpr EventBits_t kCheckDone = BIT0;

/** @brief Signature of the audio hook: interleaved L,R 16-bit samples and the number of stereo frames. */
using Consumer = void (*)(const int16_t *stereo, size_t frames);

/**
 * @brief Hook for the ML model.
 *
 * Called from the capture task with every 16 ms chunk of interleaved L,R 16-bit audio at 16 kHz with the DC offset removed. Keep
 * it short, or copy the data into a queue and process it in your own task.
 */
Consumer on_audio = nullptr;

std::atomic<bool>  check_requested{false};
EventGroupHandle_t events = nullptr;

/**
 * @brief Capture task: runs the boot-time hardware check, then reads the mics forever and hands every chunk to the level meter,
 *        the speaker loopback and the ML hook.
 * @param arg Unused.
 */
void task(void *) {
    selftest::run();  // boot-time hardware check, before audio starts flowing

    auto stereo = std::make_unique<int16_t[]>(kChunk * mic::kChannels);
    while (true) {
        if (check_requested.exchange(false)) {
            selftest::run();
            xEventGroupSetBits(events, kCheckDone);
        }
        mic::read(stereo.get(), kChunk);
        meter::feed(stereo.get(), kChunk);
        speaker::feed_loopback(stereo.get(), kChunk);
        if (on_audio != nullptr) { on_audio(stereo.get(), kChunk); }
    }
}

/** @brief Starts the capture task. */
void start() {
    events = xEventGroupCreate();
    spawn(Config::capture_task, task);
}

/**
 * @brief Runs the hardware check inside the capture task and returns once its report has printed.
 * @return false if the check did not finish within the configured timeout.
 */
bool run_check() {
    xEventGroupClearBits(events, kCheckDone);
    check_requested.store(true);
    return (xEventGroupWaitBits(events, kCheckDone, pdTRUE, pdFALSE, pdMS_TO_TICKS(Config::selftest.timeout_ms)) & kCheckDone) != 0;
}

}  // namespace capture

#pragma endregion  // Capture
/* --------- END: Capture ----------------------------------------------------------------------- */

#pragma endregion  // Audio
/* --------- END: Audio ------------------------------------------------------------------------- */


/* --------- BEGIN: Board ----------------------------------------------------------------------- */
#pragma region Board

/* ============================================================================================== */
/* --------- BEGIN: RGB LED --------------------------------------------------------------------- */
/* ============================================================================================== */
#pragma region RGB LED

/**
 * @namespace rgb_led
 * @brief The board's WS2812 RGB LED, which lights up blue/white on its own after power-up.
 *
 * Its data pin differs between board revisions, so "off" is sent to every candidate pin; a pin with no LED on it just ignores the
 * signal.
 */
namespace rgb_led {

/** @brief Turns the LED off on every candidate pin. */
void off() {
    for (const int gpio : Config::led.gpios) {
        led_strip_config_t strip_cfg     = {};
        strip_cfg.strip_gpio_num         = gpio;
        strip_cfg.max_leds               = 1;
        strip_cfg.led_model              = LED_MODEL_WS2812;
        strip_cfg.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;

        led_strip_rmt_config_t rmt_cfg = {};
        rmt_cfg.clk_src                = RMT_CLK_SRC_DEFAULT;
        rmt_cfg.resolution_hz          = 10 * 1000 * 1000;

        led_strip_handle_t strip = nullptr;
        ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &strip));
        ESP_ERROR_CHECK(led_strip_clear(strip));  // the LED latches black; the RMT channel can be freed after
        ESP_ERROR_CHECK(led_strip_del(strip));
    }
}

}  // namespace rgb_led

#pragma endregion  // RGB LED
/* --------- END: RGB LED ----------------------------------------------------------------------- */

#pragma endregion  // Board
/* --------- END: Board ------------------------------------------------------------------------- */


/* --------- BEGIN: Console --------------------------------------------------------------------- */
#pragma region Console

/**
 * @namespace console
 * @brief REPL over the S3's native USB (or UART if configured), with the command table and the command handlers.
 *
 * Every handler has the signature `int cmd_x(int argc, char **argv)` and returns 0 on success or 1 on a usage error.
 */
namespace console {

/**
 * @brief Human-readable name of a loopback source.
 * @param source Which mic(s).
 */
const char *source_name(speaker::Source source) {
    switch (source) {
    case speaker::Source::Left:
        return "mic A (left)";
    case speaker::Source::Right:
        return "mic B (right)";
    default:
        return "both mics";
    }
}

/** @brief Prints whether the loopback is on, from which mic(s), and the volume. */
void print_loop() {
    if (speaker::loopback.load()) {
        std::printf("loop on: %s -> speaker, volume %d%%\n", source_name(speaker::loop_source.load()), speaker::volume.load());
    } else {
        std::printf("loop off\n");
    }
}

/**
 * @brief `hello [name]`: says hello.
 * @param argc,argv Command line as given by the console.
 * @return 0 on success, 1 on a usage error.
 */
int cmd_hello(int argc, char **argv) {
    const Args args(argc, argv);
    if (!args.check_count(0, 1)) { return 1; }
    const auto name = args.get_string(1, "World");
    std::printf("Hello, %.*s!\n", static_cast<int>(name->size()), name->data());
    return 0;
}

/**
 * @brief `info`: prints chip revision, flash and PSRAM size and free memory.
 * @param argc,argv Command line as given by the console.
 * @return 0 on success, 1 on a usage error.
 */
int cmd_info(int argc, char **argv) {
    if (!Args(argc, argv).check_count(0, 0)) { return 1; }
    esp_chip_info_t chip;
    esp_chip_info(&chip);

    uint32_t flash_size = 0;
    esp_flash_get_size(nullptr, &flash_size);

    std::printf("cores:      %d\n", chip.cores);
    std::printf("revision:   v%d.%d\n", chip.revision / 100, chip.revision % 100);
    std::printf("flash:      %lu MB\n", static_cast<unsigned long>(flash_size >> 20));
    std::printf("psram:      %u MB\n", static_cast<unsigned>(esp_psram_get_size() >> 20));
    std::printf("free int:   %u KB\n", static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) >> 10));
    std::printf("free psram: %u KB\n", static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) >> 10));
    return 0;
}

/**
 * @brief `status`: prints uptime, loop state, volume, plot state, the last hardware check and free memory.
 * @param argc,argv Command line as given by the console.
 * @return 0 on success, 1 on a usage error.
 */
int cmd_status(int argc, char **argv) {
    if (!Args(argc, argv).check_count(0, 0)) { return 1; }
    const selftest::Result &r = selftest::result;
    std::printf("uptime:     %lld s\n", static_cast<long long>(esp_timer_get_time() / 1000000));
    std::printf("audio:      ");
    print_loop();
    std::printf("volume:     %d%%\n", speaker::volume.load());
    std::printf("plot:       %s\n", meter::streaming.load() ? "streaming" : "off");
    if (r.done) {
        std::printf("last check: mic A %s, mic B %s, speaker %s\n", r.alive[0] ? "OK" : "MISSING", r.alive[1] ? "OK" : "MISSING",
                    r.speaker_heard ? "heard" : "NOT heard");
    } else {
        std::printf("last check: not run yet\n");
    }
    std::printf("heap:       %u KB internal, %u KB PSRAM free\n", static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) >> 10),
                static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) >> 10));
    return 0;
}

/**
 * @brief `check`: runs the mic and speaker hardware check again (quiet room, about 2 s).
 * @param argc,argv Command line as given by the console.
 * @return 0 on success, 1 on a usage error or if the check did not finish.
 */
int cmd_check(int argc, char **argv) {
    if (!Args(argc, argv).check_count(0, 0)) { return 1; }
    std::printf("checking mics and speaker, keep the room quiet (about 2 s)...\n");
    if (!capture::run_check()) {
        std::printf("check did not finish\n");
        return 1;
    }
    return 0;
}

/**
 * @brief `loop [on|off] [both|left|right]`: mic -> speaker passthrough.
 *
 * - `loop` shows the state
 * - `loop left|right|both` picks the mic(s)
 * - `loop on [left|right|both]` and `loop off` switch it
 *
 * @param argc,argv Command line as given by the console.
 * @return 0 on success, 1 on a usage error.
 */
int cmd_loop(int argc, char **argv) {
    const Args args(argc, argv);
    if (!args.check_count(0, 2)) { return 1; }

    if (args.has(1)) {
        const auto first = args.get_choice(1, {"on", "off", "both", "left", "right"});
        if (!first) { return 1; }
        const bool is_switch = *first == "on" || *first == "off";

        std::optional<std::string_view> source = is_switch ? std::nullopt : first;
        if (is_switch && args.has(2)) {
            source = args.get_choice(2, {"both", "left", "right"});
            if (!source) { return 1; }
        } else if (!is_switch && args.has(2)) {
            std::printf("'%.*s' takes no further argument\n", static_cast<int>(first->size()), first->data());
            args.print_usage();
            return 1;
        }

        if (source) {
            speaker::loop_source.store(*source == "left"    ? speaker::Source::Left
                                       : *source == "right" ? speaker::Source::Right
                                                            : speaker::Source::Both);
        }
        if (*first == "on") {
            speaker::set_loopback(true);
            print_loop();
            std::printf("keep the speaker away from the mics or it will howl; loop switches itself off if it detects feedback\n");
            return 0;
        }
        if (*first == "off") { speaker::set_loopback(false); }
    }
    print_loop();
    return 0;
}

/**
 * @brief `volume [0-100]`: sets the master speaker volume, or shows it when no value is given.
 * @param argc,argv Command line as given by the console.
 * @return 0 on success, 1 on a usage error.
 */
int cmd_volume(int argc, char **argv) {
    const Args args(argc, argv);
    if (!args.check_count(0, 1)) { return 1; }
    if (args.has(1)) {
        const auto volume = args.get_int(1, 0, 100);
        if (!volume) { return 1; }
        speaker::volume.store(*volume);
    }
    std::printf("volume %d%%\n", speaker::volume.load());
    return 0;
}

/**
 * @brief `beep [freq_hz 20-8000] [ms 1-5000]`: plays a tone (default 1000 Hz, 200 ms).
 * @param argc,argv Command line as given by the console.
 * @return 0 on success, 1 on a usage error.
 */
int cmd_beep(int argc, char **argv) {
    const Args args(argc, argv);
    if (!args.check_count(0, 2)) { return 1; }
    const auto freq = args.get_int(1, 20, 8000, 1000);
    const auto ms   = args.get_int(2, 1, 5000, 200);
    if (!freq || !ms) { return 1; }
    speaker::tone(*freq, *ms);
    std::printf("beep %d Hz, %d ms\n", *freq, *ms);
    return 0;
}

/**
 * @brief `play [song]`: plays a melody from Assets (default twinkle); an unknown name lists the choices.
 * @param argc,argv Command line as given by the console.
 * @return 0 on success, 1 on a usage error or an unknown song.
 */
int cmd_play(int argc, char **argv) {
    const Args args(argc, argv);
    if (!args.check_count(0, 1)) { return 1; }
    const auto    name   = args.get_string(1, "twinkle");
    const Melody *melody = Assets::find(*name);
    if (melody == nullptr) {
        std::printf("unknown song '%.*s'; choose:", static_cast<int>(name->size()), name->data());
        for (const Melody &m : Assets::melodies) { std::printf(" %.*s", static_cast<int>(m.name.size()), m.name.data()); }
        std::printf("\n");
        return 1;
    }
    speaker::play(melody->notes, Config::speaker.note_gap_ms);
    std::printf("playing %.*s at volume %d%%\n", static_cast<int>(name->size()), name->data(), speaker::volume.load());
    return 0;
}

/**
 * @brief `plot [on|off]`: starts or stops the CSV level stream (default on).
 * @param argc,argv Command line as given by the console.
 * @return 0 on success, 1 on a usage error.
 */
int cmd_plot(int argc, char **argv) {
    const Args args(argc, argv);
    if (!args.check_count(0, 1)) { return 1; }
    const auto state = args.get_choice(1, {"on", "off"}, "on");
    if (!state) { return 1; }
    meter::set_streaming(*state == "on");
    return 0;
}

/**
 * @brief `restart`: reboots the chip.
 * @param argc,argv Command line as given by the console.
 * @return Does not return on success; 1 on a usage error.
 */
int cmd_restart(int argc, char **argv) {
    if (!Args(argc, argv).check_count(0, 0)) { return 1; }
    esp_restart();
}

/**
 * @struct Command
 * @brief One entry of the command table.
 *
 * The one list of commands: `help` shows the name, syntax and help text, and Args prints the syntax in usage errors.
 */
struct Command {
    const char            *name;
    const char            *syntax;  // arguments after the name, "" when there are none
    const char            *help;
    esp_console_cmd_func_t func;
};

constexpr Command kCommands[] = {
    {"hello", "[name]", "Say hello", cmd_hello},
    {"info", "", "Chip, flash and memory info", cmd_info},
    {"status", "", "Overview: audio loop, volume, last hardware check, memory", cmd_status},
    {"check", "", "Run the mic + speaker hardware check (quiet room, about 2 s)", cmd_check},
    {"loop", "[on|off] [both|left|right]", "Mic -> speaker passthrough; with no argument shows the state", cmd_loop},
    {"volume", "[0-100]", "Master speaker volume; with no argument shows it", cmd_volume},
    {"beep", "[freq_hz 20-8000] [ms 1-5000]", "Play a tone (default 1000 Hz, 200 ms)", cmd_beep},
    {"play", "[song]", "Play a song (default twinkle); an unknown name lists the choices", cmd_play},
    {"plot", "[on|off]", "Stream mic levels as CSV for plotting (default on)", cmd_plot},
    {"restart", "", "Reboot the chip", cmd_restart},
};

/**
 * @brief Looks up the argument syntax of a command in the table.
 * @param command Command name.
 * @return The syntax text, or "" when the command takes no arguments or is unknown.
 */
const char *syntax_of(std::string_view command) {
    for (const Command &c : kCommands) {
        if (command == c.name) { return c.syntax; }
    }
    return "";
}

/** @brief Registers every command and starts the REPL on USB (or UART, depending on the console setting). */
void start() {
    for (const Command &c : kCommands) {
        // Zero-initialised so fields we don't use (argtable, context, ...) stay empty under -Werror
        esp_console_cmd_t cmd = {};
        cmd.command           = c.name;
        cmd.hint              = c.syntax[0] != '\0' ? c.syntax : nullptr;
        cmd.help              = c.help;
        cmd.func              = c.func;
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }
    ESP_ERROR_CHECK(esp_console_register_help_command());

    esp_console_repl_t       *repl        = nullptr;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt                    = Config::console.prompt;

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t dev_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&dev_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_UART_DEFAULT || CONFIG_ESP_CONSOLE_UART_CUSTOM
    esp_console_dev_uart_config_t dev_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&dev_config, &repl_config, &repl));
#else
#error "Unsupported console type"
#endif

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

}  // namespace console

#pragma endregion  // Console
/* --------- END: Console ----------------------------------------------------------------------- */


/* --------- BEGIN: Entry Point ----------------------------------------------------------------- */
#pragma region Entry Point

/**
 * @brief Firmware entry point, called by ESP-IDF's main task.
 *
 * The console starts first because its task needs internal RAM, which the audio tasks would otherwise use up. The capture task
 * then runs the hardware check and streams audio.
 */
extern "C" void app_main() {
    rgb_led::off();
    console::start();  // first: its task needs internal RAM
    meter::start();
    speaker::init();
    mic::init();
    capture::start();  // runs the hardware check, then streams audio
    log_heap("ready");
}

#pragma endregion  // Entry Point
/* --------- END: Entry Point ------------------------------------------------------------------- */
