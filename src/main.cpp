// Toy robot firmware: ESP32-S3-N16R8, 2x INMP441 mics, MAX98357 amp.
// "Hi ESP" wake word + English voice commands (esp-sr), sound direction (DOA), speaker cues, USB console.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <numbers>

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
#include "freertos/queue.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"

extern "C" {
#include "esp_afe_doa.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_mn_speech_commands.h"
#include "esp_wn_iface.h"
#include "model_path.h"
}

namespace {

constexpr int kSampleRate = 16000;

// ---------------------------------------------------------------------------------------------------------------------
// Mic: two INMP441s on I2S0 (L/R=GND -> left, L/R=3V3 -> right)
// ---------------------------------------------------------------------------------------------------------------------
namespace mic {

constexpr int kChannels = 2;

i2s_chan_handle_t rx = nullptr;

void init() {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, nullptr, &rx));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(kSampleRate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg =
            {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = static_cast<gpio_num_t>(CONFIG_MIC_I2S_SCK_GPIO),
                .ws = static_cast<gpio_num_t>(CONFIG_MIC_I2S_WS_GPIO),
                .dout = I2S_GPIO_UNUSED,
                .din = static_cast<gpio_num_t>(CONFIG_MIC_I2S_SD_GPIO),
                .invert_flags = {},
            },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx));
}

// Blocks until `frames` stereo frames are read into `out` as interleaved L,R 16-bit samples
void read(int16_t *out, size_t frames) {
    const size_t samples = frames * kChannels;

    // AFE chunk size is fixed, so this only allocates once
    static std::unique_ptr<int32_t[]> raw;
    static size_t raw_len = 0;
    if (raw_len < samples) {
        raw.reset(new int32_t[samples]);
        raw_len = samples;
    }

    size_t bytes_read = 0;
    ESP_ERROR_CHECK(i2s_channel_read(rx, raw.get(), samples * sizeof(int32_t), &bytes_read, portMAX_DELAY));

    // INMP441 sends 24-bit samples left-aligned in 32-bit slots
    for (size_t i = 0; i < samples; ++i) {
        const int32_t s = raw[i] >> CONFIG_MIC_GAIN_SHIFT;
        out[i] = static_cast<int16_t>(std::clamp<int32_t>(s, INT16_MIN, INT16_MAX));
    }
}

}  // namespace mic

// ---------------------------------------------------------------------------------------------------------------------
// Speaker: MAX98357 on I2S1, tones played from a queue so callers never block
// ---------------------------------------------------------------------------------------------------------------------
namespace speaker {

struct Tone {
    uint16_t freq_hz;  // 0 = silence
    uint16_t duration_ms;
};

constexpr size_t kBlock = 256;    // samples per I2S write (16 ms)
constexpr int kFadeSamples = 80;  // 5 ms ramp in/out to avoid clicks

i2s_chan_handle_t tx = nullptr;
QueueHandle_t queue = nullptr;

void write(const int16_t *buf, size_t samples) {
    size_t written = 0;
    i2s_channel_write(tx, buf, samples * sizeof(int16_t), &written, portMAX_DELAY);
}

void play(const Tone &t) {
    static int16_t buf[kBlock];
    constexpr float kTwoPi = 2.0f * std::numbers::pi_v<float>;
    const float amp = 32767.0f * CONFIG_SPK_VOLUME / 100.0f;
    const float step = kTwoPi * t.freq_hz / kSampleRate;
    const int total = kSampleRate * t.duration_ms / 1000;

    float phase = 0.0f;
    for (int done = 0; done < total;) {
        const int n = std::min<int>(kBlock, total - done);
        for (int i = 0; i < n; ++i, ++done) {
            const int edge = std::min(done, total - 1 - done);
            const float env = edge < kFadeSamples ? static_cast<float>(edge) / kFadeSamples : 1.0f;
            buf[i] = t.freq_hz == 0 ? 0 : static_cast<int16_t>(amp * env * std::sin(phase));
            phase += step;
            if (phase > kTwoPi) {
                phase -= kTwoPi;
            }
        }
        write(buf, n);
    }
}

void task(void *) {
    static const int16_t silence[kBlock] = {};
    Tone t;
    while (true) {
        if (xQueueReceive(queue, &t, portMAX_DELAY) == pdTRUE) {
            play(t);
            // Flush the DMA buffers so the amp doesn't hold the last sample
            write(silence, kBlock);
        }
    }
}

void init() {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx, nullptr));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(kSampleRate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg =
            {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = static_cast<gpio_num_t>(CONFIG_SPK_I2S_BCLK_GPIO),
                .ws = static_cast<gpio_num_t>(CONFIG_SPK_I2S_LRC_GPIO),
                .dout = static_cast<gpio_num_t>(CONFIG_SPK_I2S_DIN_GPIO),
                .din = I2S_GPIO_UNUSED,
                .invert_flags = {},
            },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx));

    queue = xQueueCreate(8, sizeof(Tone));
    xTaskCreatePinnedToCore(task, "speaker", 4 * 1024, nullptr, 4, nullptr, 0);
}

void tone(uint16_t freq_hz, uint16_t duration_ms) {
    const Tone t{freq_hz, duration_ms};
    xQueueSend(queue, &t, 0);
}

void chirp() {  // wake word heard
    tone(880, 70);
    tone(1320, 90);
}

void confirm() {  // command accepted
    tone(1760, 60);
}

}  // namespace speaker

// ---------------------------------------------------------------------------------------------------------------------
// DOA: direction of the loudest sound from the two raw mic channels (SRP-PHAT)
// ---------------------------------------------------------------------------------------------------------------------
namespace doa {

struct Reading {
    float angle_deg;  // 0..180, 90 = straight ahead (broadside to the mic pair)
    float level_db;   // loudness of the frame the angle came from, dBFS
    int64_t time_us;  // esp_timer time of the reading; 0 = none yet
};

constexpr int kFrames = 1024;       // 64 ms per estimate
constexpr float kSmoothing = 0.4f;  // weight of the newest reading
constexpr size_t kFrameBytes = kFrames * mic::kChannels * sizeof(int16_t);

StreamBufferHandle_t stream = nullptr;
portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
Reading current{};

float level_dbfs(const int16_t *interleaved) {
    int64_t sum = 0;
    for (int i = 0; i < kFrames * mic::kChannels; ++i) {
        sum += static_cast<int32_t>(interleaved[i]) * interleaved[i];
    }
    const float rms = std::sqrt(static_cast<float>(sum) / (kFrames * mic::kChannels));
    return rms > 0.0f ? 20.0f * std::log10(rms / 32768.0f) : -120.0f;
}

void task(void *) {
    afe_doa_handle_t *h =
        afe_doa_create("MM", kSampleRate, CONFIG_DOA_RESOLUTION_DEG, CONFIG_MIC_SPACING_MM / 1000.0f, kFrames);
    configASSERT(h);
    auto buf = std::make_unique<int16_t[]>(kFrames * mic::kChannels);

    while (true) {
        xStreamBufferReceive(stream, buf.get(), kFrameBytes, portMAX_DELAY);

        // Skip quiet frames so background noise doesn't move the estimate
        const float level = level_dbfs(buf.get());
        if (level < CONFIG_DOA_MIN_LEVEL_DB) {
            continue;
        }

        const float angle = afe_doa_process(h, buf.get());
        const int64_t now = esp_timer_get_time();

        taskENTER_CRITICAL(&lock);
        // Restart smoothing after a quiet gap so a new source isn't dragged by the old one
        const bool fresh = current.time_us == 0 || now - current.time_us > 500'000;
        current.angle_deg = fresh ? angle : current.angle_deg + kSmoothing * (angle - current.angle_deg);
        current.level_db = level;
        current.time_us = now;
        taskEXIT_CRITICAL(&lock);
    }
}

void start() {
    // Room for two estimates; the trigger level makes Receive wait for a full frame
    stream = xStreamBufferCreate(2 * kFrameBytes, kFrameBytes);
    configASSERT(stream);
    xTaskCreatePinnedToCore(task, "doa", 6 * 1024, nullptr, 3, nullptr, 0);
}

// Never blocks; drops audio if the DOA task falls behind
void push(const int16_t *interleaved, size_t frames) {
    const size_t bytes = frames * mic::kChannels * sizeof(int16_t);
    if (xStreamBufferSpacesAvailable(stream) >= bytes) {
        xStreamBufferSend(stream, interleaved, bytes, 0);
    }
}

Reading latest() {
    taskENTER_CRITICAL(&lock);
    const Reading r = current;
    taskEXIT_CRITICAL(&lock);
    return r;
}

}  // namespace doa

// ---------------------------------------------------------------------------------------------------------------------
// Speech: AFE (2-mic BSS + NS) -> WakeNet "Hi ESP" -> MultiNet7 English commands
// ---------------------------------------------------------------------------------------------------------------------
namespace speech {

constexpr const char *TAG = "speech";
constexpr int kCommandTimeoutMs = 6000;

enum class Command : int {
    Forward = 1,
    Backward,
    Left,
    Right,
    Stop,
};

struct Phrase {
    Command cmd;
    const char *text;
};

// Several phrasings per command; MultiNet7 English takes plain text (no phonemes needed)
constexpr Phrase kPhrases[] = {
    {Command::Forward, "move forward"},
    {Command::Forward, "go forward"},
    {Command::Forward, "go straight"},
    {Command::Backward, "move back"},
    {Command::Backward, "go back"},
    {Command::Left, "turn left"},
    {Command::Left, "go left"},
    {Command::Right, "turn right"},
    {Command::Right, "go right"},
    {Command::Stop, "stop"},
    {Command::Stop, "halt"},
};

const char *name(Command cmd) {
    switch (cmd) {
        case Command::Forward: return "forward";
        case Command::Backward: return "backward";
        case Command::Left: return "left";
        case Command::Right: return "right";
        case Command::Stop: return "stop";
    }
    return "unknown";
}

using Handler = void (*)(Command cmd);

const esp_afe_sr_iface_t *afe = nullptr;
esp_afe_sr_data_t *afe_data = nullptr;
srmodel_list_t *models = nullptr;
Handler handler = nullptr;

void feed_task(void *) {
    const int chunk = afe->get_feed_chunksize(afe_data);
    configASSERT(afe->get_feed_channel_num(afe_data) == mic::kChannels);
    auto buf = std::make_unique<int16_t[]>(chunk * mic::kChannels);

    while (true) {
        mic::read(buf.get(), chunk);
        doa::push(buf.get(), chunk);
        afe->feed(afe_data, buf.get());
    }
}

void detect_task(void *) {
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_ENGLISH);
    esp_mn_iface_t *multinet = esp_mn_handle_from_name(mn_name);
    model_iface_data_t *mn = multinet->create(mn_name, kCommandTimeoutMs);
    ESP_LOGI(TAG, "multinet: %s", mn_name);

    ESP_ERROR_CHECK(esp_mn_commands_alloc(multinet, mn));
    for (const auto &p : kPhrases) {
        ESP_ERROR_CHECK(esp_mn_commands_add(static_cast<int>(p.cmd), p.text));
    }
    if (esp_mn_error_t *err = esp_mn_commands_update(); err != nullptr) {
        for (int i = 0; i < err->num; ++i) {
            ESP_LOGE(TAG, "rejected phrase: %s", err->phrases[i]->string);
        }
    }
    multinet->print_active_speech_commands(mn);

    bool awake = false;
    ESP_LOGI(TAG, "say \"Hi ESP\"");
    while (true) {
        afe_fetch_result_t *res = afe->fetch(afe_data);
        if (res == nullptr || res->ret_value == ESP_FAIL) {
            ESP_LOGE(TAG, "afe fetch failed");
            continue;
        }

        // With 2 mics, wait until the AFE has picked the channel the speaker is on
        if (res->wakeup_state == WAKENET_CHANNEL_VERIFIED) {
            ESP_LOGI(TAG, "wake word on channel %d from %.0f deg, listening...", res->trigger_channel_id,
                     doa::latest().angle_deg);
            multinet->clean(mn);
            speaker::chirp();
            awake = true;
        }
        if (!awake) {
            continue;
        }

        switch (multinet->detect(mn, res->data)) {
            case ESP_MN_STATE_DETECTED: {
                esp_mn_results_t *r = multinet->get_results(mn);
                const auto cmd = static_cast<Command>(r->command_id[0]);
                ESP_LOGI(TAG, "command: %s (\"%s\", p=%.2f)", name(cmd), r->string, r->prob[0]);
                speaker::confirm();
                if (handler != nullptr) {
                    handler(cmd);
                }
                break;
            }
            case ESP_MN_STATE_TIMEOUT:
                ESP_LOGI(TAG, "timeout, say \"Hi ESP\" again");
                afe->enable_wakenet(afe_data);
                awake = false;
                break;
            default:
                break;
        }
    }
}

// Loads models, then runs mic feed on core 0 and detection on core 1
void start(Handler on_command) {
    handler = on_command;

    // WakeNet/MultiNet/VADNet live in the "model" partition
    models = esp_srmodel_init("model");
    mic::init();
    doa::start();

    // "MM" = two mic channels, no playback reference; HIGH_PERF enables full BSS + noise suppression
    afe_config_t *cfg = afe_config_init("MM", models, AFE_TYPE_SR, AFE_MODE_HIGH_PERF);
    afe = esp_afe_handle_from_config(cfg);
    afe_data = afe->create_from_config(cfg);
    afe_config_free(cfg);

    xTaskCreatePinnedToCore(detect_task, "sr_detect", 8 * 1024, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(feed_task, "sr_feed", 8 * 1024, nullptr, 5, nullptr, 0);
}

}  // namespace speech

// ---------------------------------------------------------------------------------------------------------------------
// Console: REPL over the S3's native USB (or UART if configured)
// ---------------------------------------------------------------------------------------------------------------------
namespace console {

int cmd_hello(int argc, char **argv) {
    std::printf("hello, %s!\n", argc > 1 ? argv[1] : "robot");
    return 0;
}

int cmd_info(int, char **) {
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

int cmd_beep(int argc, char **argv) {
    const int freq = argc > 1 ? std::atoi(argv[1]) : 1000;
    const int ms = argc > 2 ? std::atoi(argv[2]) : 200;
    speaker::tone(freq, ms);
    return 0;
}

void print_doa() {
    const doa::Reading r = doa::latest();
    if (r.time_us == 0) {
        std::printf("no sound above threshold yet\n");
        return;
    }
    const float age_s = (esp_timer_get_time() - r.time_us) / 1e6f;
    std::printf("direction %5.1f deg  level %5.1f dBFS  (%.1fs ago)\n", r.angle_deg, r.level_db, age_s);
}

int cmd_doa(int argc, char **argv) {
    const int seconds = argc > 1 ? std::atoi(argv[1]) : 0;
    print_doa();
    for (int i = 0; i < seconds * 5; ++i) {
        vTaskDelay(pdMS_TO_TICKS(200));
        print_doa();
    }
    return 0;
}

int cmd_restart(int, char **) {
    esp_restart();
}

void start() {
    const esp_console_cmd_t cmds[] = {
        {.command = "hello", .help = "Say hello: hello [name]", .hint = nullptr, .func = cmd_hello},
        {.command = "info", .help = "Show chip, flash and memory info", .hint = nullptr, .func = cmd_info},
        {.command = "beep", .help = "Play a tone: beep [freq_hz] [ms]", .hint = nullptr, .func = cmd_beep},
        {.command = "doa", .help = "Sound direction: doa [watch_seconds]", .hint = nullptr, .func = cmd_doa},
        {.command = "restart", .help = "Reboot the chip", .hint = nullptr, .func = cmd_restart},
    };
    for (const auto &cmd : cmds) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }
    ESP_ERROR_CHECK(esp_console_register_help_command());

    esp_console_repl_t *repl = nullptr;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "robot> ";

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

// ---------------------------------------------------------------------------------------------------------------------
// Robot behaviour
// ---------------------------------------------------------------------------------------------------------------------
void on_command(speech::Command cmd) {
    // TODO: drive motors; doa::latest() says which way the speaker is
    ESP_LOGI("robot", "-> %s (speaker at %.0f deg)", speech::name(cmd), doa::latest().angle_deg);
}

}  // namespace

extern "C" void app_main() {
    speaker::init();
    speech::start(on_command);
    console::start();
}
