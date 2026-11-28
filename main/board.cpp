#include "board.hpp"
#include "ui.hpp"

#include "driver/gpio.h"
#include "driver/i2s.h"
#include "driver/ledc.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstring>

namespace go {
namespace {

constexpr const char* Tag = "arduboy-go";
constexpr auto Bus = SPI2_HOST;
constexpr auto LcdCs = GPIO_NUM_5;
constexpr auto LcdDc = GPIO_NUM_21;
constexpr auto LcdPwm = GPIO_NUM_14;
constexpr auto SdCs = GPIO_NUM_22;
constexpr auto SpeakerEnable = GPIO_NUM_25;
constexpr auto XAxis = ADC1_CHANNEL_6;
constexpr auto YAxis = ADC1_CHANNEL_7;
constexpr auto Battery = ADC1_CHANNEL_0;
constexpr auto Attenuation = ADC_ATTEN_DB_12;
constexpr int StripRows = 12;
constexpr unsigned DacHz = 32000;
constexpr size_t DacChunk = 128;
constexpr size_t AudioTargetFill = 768;
constexpr const char* MountPoint = "/sd";

struct DigitalButton { gpio_num_t pin; Key key; bool pullup; };
constexpr DigitalButton Buttons[] = {
    {GPIO_NUM_32, A, true}, {GPIO_NUM_33, B, true},
    {GPIO_NUM_13, Menu, true}, {GPIO_NUM_0, Volume, false},
    {GPIO_NUM_27, Select, true}, {GPIO_NUM_39, Start, false},
};

void output(gpio_num_t pin, bool high) {
    ESP_ERROR_CHECK(gpio_set_level(pin, high));
    gpio_config_t cfg{};
    cfg.pin_bit_mask = 1ULL << pin;
    cfg.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&cfg));
}

}

void Board::begin() {
    output(SdCs, true);
    output(LcdCs, true);
    output(LcdDc, true);
    output(SpeakerEnable, false);

    spi_bus_config_t bus{};
    bus.mosi_io_num = GPIO_NUM_23;
    bus.miso_io_num = GPIO_NUM_19;
    bus.sclk_io_num = GPIO_NUM_18;
    bus.quadwp_io_num = GPIO_NUM_NC;
    bus.quadhd_io_num = GPIO_NUM_NC;
    bus.data4_io_num = GPIO_NUM_NC;
    bus.data5_io_num = GPIO_NUM_NC;
    bus.data6_io_num = GPIO_NUM_NC;
    bus.data7_io_num = GPIO_NUM_NC;
    bus.max_transfer_sz = ScreenW * StripRows * sizeof(uint16_t);
    bus.isr_cpu_id = ESP_INTR_CPU_AFFINITY_0;
    ESP_ERROR_CHECK(spi_bus_initialize(Bus, &bus, SPI_DMA_CH_AUTO));

    // Put the card in SPI mode before initializing the LCD on the shared bus.
    mountStorage();
    startDisplay();
    startInput();
#ifdef ARDUGO_QUIET_TEST
    output(SpeakerEnable, false);
#else
    startAudio();
#endif
}

void Board::startDisplay() {
    ledc_timer_config_t timer{};
    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.duty_resolution = LEDC_TIMER_10_BIT;
    timer.timer_num = LEDC_TIMER_0;
    timer.freq_hz = 12000;
    timer.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    ledc_channel_config_t pwm{};
    pwm.gpio_num = LcdPwm;
    pwm.speed_mode = LEDC_LOW_SPEED_MODE;
    pwm.channel = LEDC_CHANNEL_0;
    pwm.timer_sel = LEDC_TIMER_0;
    ESP_ERROR_CHECK(ledc_channel_config(&pwm));

    spi_device_interface_config_t config{};
    config.clock_speed_hz = 40 * 1000 * 1000;
    config.spics_io_num = LcdCs;
    config.queue_size = 1;
    config.flags = SPI_DEVICE_HALFDUPLEX;
    ESP_ERROR_CHECK(spi_bus_add_device(Bus, &config, &lcd_));

    const auto send = [&](uint8_t cmd, std::initializer_list<uint8_t> data) {
        lcdCommand(cmd, data.begin(), data.size());
    };
    lcdCommand(0x01);
    vTaskDelay(pdMS_TO_TICKS(20));
    send(0x3a, {0x55});
    send(0xcf, {0x00, 0xc3, 0x30});
    send(0xed, {0x64, 0x03, 0x12, 0x81});
    send(0xe8, {0x85, 0x00, 0x78});
    send(0xcb, {0x39, 0x2c, 0x00, 0x34, 0x02});
    send(0xf7, {0x20});
    send(0xea, {0x00, 0x00});
    send(0xc0, {0x1b});
    send(0xc1, {0x12});
    send(0xc5, {0x32, 0x3c});
    send(0xc7, {0x91});
    send(0x36, {0xa8}); // Landscape, BGR panel.
    send(0xb1, {0x00, 0x10});
    send(0xb6, {0x0a, 0xa2});
    send(0xf6, {0x01, 0x30});
    send(0xf2, {0x00});
    send(0x26, {0x01});
    send(0xe0, {0x0f, 0x31, 0x2b, 0x0c, 0x0e, 0x08, 0x4e, 0xf1,
                0x37, 0x07, 0x10, 0x03, 0x0e, 0x09, 0x00});
    send(0xe1, {0x00, 0x0e, 0x14, 0x03, 0x11, 0x07, 0x31, 0xc1,
                0x48, 0x08, 0x0f, 0x0c, 0x31, 0x36, 0x0f});
    lcdCommand(0x11);
    vTaskDelay(pdMS_TO_TICKS(120));

    frame_ = static_cast<uint16_t*>(heap_caps_malloc(ScreenW * ScreenH * sizeof(uint16_t),
                                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    strip_ = static_cast<uint16_t*>(heap_caps_malloc(ScreenW * StripRows * sizeof(uint16_t),
                                                    MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    displayFree_ = xSemaphoreCreateBinary();
    ESP_ERROR_CHECK(frame_ && strip_ && displayFree_ ? ESP_OK : ESP_ERR_NO_MEM);
    std::memset(strip_, 0, ScreenW * StripRows * sizeof(uint16_t));
    const uint8_t columns[] = {0, 0, uint8_t((ScreenW - 1) >> 8), uint8_t(ScreenW - 1)};
    const uint8_t rows[] = {0, 0, uint8_t((ScreenH - 1) >> 8), uint8_t(ScreenH - 1)};
    lcdCommand(0x2a, columns, sizeof(columns));
    lcdCommand(0x2b, rows, sizeof(rows));
    lcdCommand(0x2c);
    for (int y = 0; y < ScreenH; y += StripRows) {
        const int n = std::min(StripRows, ScreenH - y) * ScreenW;
        spi_transaction_t transfer{};
        transfer.length = n * sizeof(*strip_) * 8;
        transfer.tx_buffer = strip_;
        ESP_ERROR_CHECK(spi_device_polling_transmit(lcd_, &transfer));
    }
    lcdCommand(0x29);
    xSemaphoreGive(displayFree_);
    ESP_ERROR_CHECK(xTaskCreatePinnedToCore([](void* self) { static_cast<Board*>(self)->renderDisplay(); },
                    "lcd", 3072, this, 4, &displayTask_, 0) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

void Board::lcdCommand(uint8_t command, const uint8_t* data, size_t size) {
    gpio_set_level(LcdDc, false);
    spi_transaction_t transfer{};
    transfer.flags = SPI_TRANS_USE_TXDATA;
    transfer.length = 8;
    transfer.tx_data[0] = command;
    ESP_ERROR_CHECK(spi_device_polling_transmit(lcd_, &transfer));
    gpio_set_level(LcdDc, true);
    if (size) lcdData(data, size);
}

void Board::lcdData(const void* data, size_t size) {
    spi_transaction_t transfer{};
    transfer.length = size * 8;
    if (size <= sizeof(transfer.tx_data)) {
        transfer.flags = SPI_TRANS_USE_TXDATA;
        std::memcpy(transfer.tx_data, data, size);
    } else transfer.tx_buffer = data;
    ESP_ERROR_CHECK(spi_device_polling_transmit(lcd_, &transfer));
}

void Board::present(const uint16_t* pixels, Rect region) {
    xSemaphoreTake(displayFree_, portMAX_DELAY);
    queueDisplay(pixels, region);
}

bool Board::tryPresent(const uint16_t* pixels, Rect region) {
    if (xSemaphoreTake(displayFree_, 0) != pdTRUE) return false;
    queueDisplay(pixels, region);
    return true;
}

bool Board::tryPresentGame(Canvas&, const uint8_t* image, Scale scale, unsigned palette) {
    if (xSemaphoreTake(displayFree_, 0) != pdTRUE) return false;
    std::memcpy(gameFrame_, image, sizeof(gameFrame_));
    region_ = viewport(scale);
    renderGameFrame_ = true;
    gamePalette_ = palette;
    xTaskNotifyGive(displayTask_);
    return true;
}

void Board::playbackStats(unsigned fps, unsigned avrPercent) {
    ESP_LOGI(Tag, "Gameplay: %u fps, AVR %u%%", fps, avrPercent);
}

#ifdef ARDUGO_QUIET_TEST
void Board::beginLcdMeasurement(bool fullFrames) {
    xSemaphoreTake(displayFree_, portMAX_DELAY);
    benchmarkFullFrames_ = fullFrames;
    lcdStats_ = {};
    displayedGameBox_ = {};
    xSemaphoreGive(displayFree_);
}

Board::LcdStats Board::endLcdMeasurement() {
    xSemaphoreTake(displayFree_, portMAX_DELAY);
    const auto stats = lcdStats_;
    benchmarkFullFrames_ = false;
    xSemaphoreGive(displayFree_);
    return stats;
}

void Board::probeLcdThroughput() {
    static std::array<uint8_t, GameW * GameH> pixels;
    Canvas canvas;
    const auto submit = [&](Scale scale, unsigned palette = 0) {
        ESP_ERROR_CHECK(tryPresentGame(canvas, pixels.data(), scale, palette) ? ESP_OK : ESP_FAIL);
        xSemaphoreTake(displayFree_, portMAX_DELAY);
        xSemaphoreGive(displayFree_);
    };
    for (const auto scale : {Scale::Native, Scale::Double, Scale::Fit}) {
        const auto box = viewport(scale);
        for (const bool full : {true, false}) {
            pixels.fill(0);
            beginLcdMeasurement(full);
            submit(scale);
            const unsigned frameBytes = box.w * box.h * sizeof(uint16_t);
            ESP_ERROR_CHECK(lcdStats_.bytes == frameBytes ? ESP_OK : ESP_FAIL);

            // Toggle an 8x8 sprite at an odd source row to exercise fractional scaling.
            constexpr int SpriteY = 21;
            constexpr int SpriteX = 59;
            const auto start = esp_timer_get_time();
            for (unsigned frame = 0; frame < 80; ++frame) {
                const unsigned color = (frame & 1) ? 0 : 255;
                for (int y = SpriteY; y < SpriteY + 8; ++y)
                    std::fill_n(pixels.data() + y * GameW + SpriteX, 8, color);
                submit(scale);
            }
            const auto elapsed = esp_timer_get_time() - start;
            const auto stats = endLcdMeasurement();
            const unsigned rows = full ? box.h : 8 * box.h / GameH;
            const unsigned bytes = stats.bytes - frameBytes;
            const unsigned windows = stats.windows - 1;
            ESP_ERROR_CHECK(bytes == 80 * box.w * rows * 2 && windows == 80 ? ESP_OK : ESP_FAIL);
            ESP_LOGI("bench", "LCD sprite %dx%d mode=%s frames=80 wall_us=%lld bytes=%u windows=%u",
                     box.w, box.h, full ? "full" : "rows", static_cast<long long>(elapsed),
                     bytes, windows);
        }

        beginLcdMeasurement(false);
        submit(scale);
        submit(scale);
        canvas.fill(0);
        present(canvas.data(), FpsCorner);
        xSemaphoreTake(displayFree_, portMAX_DELAY);
        xSemaphoreGive(displayFree_);
        submit(scale);
        ESP_ERROR_CHECK(lcdStats_.windows == 2 ? ESP_OK : ESP_FAIL);
        pixels.front() = 255;
        pixels.back() = 255;
        submit(scale);
        ESP_ERROR_CHECK(lcdStats_.windows == 4 ? ESP_OK : ESP_FAIL);
        submit(scale, 1);
        present(canvas.data(), box);
        xSemaphoreTake(displayFree_, portMAX_DELAY);
        xSemaphoreGive(displayFree_);
        submit(scale, 1);
        ESP_ERROR_CHECK(endLcdMeasurement().windows == 7 ? ESP_OK : ESP_FAIL);
    }
    ESP_LOGI("bench", "LCD partial update checks passed for all three scales");
}
#endif

void Board::queueDisplay(const uint16_t* pixels, Rect region) {
    for (int y = 0; y < region.h; ++y) {
        const auto* row = pixels + (region.y + y) * ScreenW + region.x;
        std::copy_n(row, region.w, frame_ + y * region.w);
    }
    renderGameFrame_ = false;
    region_ = region;
    xTaskNotifyGive(displayTask_);
}

void Board::writeDisplayRows(Rect box, int first, int end, const GamePixels* gamePixels) {
    const int right = box.x + box.w - 1;
    const int top = box.y + first;
    const int bottom = box.y + end - 1;
    const uint8_t columns[] = {uint8_t(box.x >> 8), uint8_t(box.x), uint8_t(right >> 8), uint8_t(right)};
    const uint8_t rows[] = {uint8_t(top >> 8), uint8_t(top), uint8_t(bottom >> 8), uint8_t(bottom)};
    lcdCommand(0x2a, columns, sizeof(columns));
    lcdCommand(0x2b, rows, sizeof(rows));
    lcdCommand(0x2c);
#ifdef ARDUGO_QUIET_TEST
    lcdStats_.bytes += (end - first) * box.w * sizeof(*strip_);
    ++lcdStats_.windows;
#endif
    for (int y = first; y < end; y += StripRows) {
        const int rows = std::min(StripRows, end - y);
        const int n = rows * box.w;
        if (gamePixels) {
            int lastSourceRow = -1;
            for (int i = 0; i < rows; ++i) {
                const int sourceRow = (y + i) * GameH / box.h;
                auto* row = strip_ + i * box.w;
                if (sourceRow == lastSourceRow) std::copy_n(row - box.w, box.w, row);
                else gamePixels->row(row, gameFrame_ + sourceRow * GameW);
                lastSourceRow = sourceRow;
            }
        } else {
            const auto* src = frame_ + y * box.w;
            for (int i = 0; i < n; ++i) strip_[i] = __builtin_bswap16(src[i]);
        }
        // Interrupt-driven DMA allows the idle and audio tasks to run during the transfer.
        spi_transaction_t transfer{};
        transfer.length = n * sizeof(*strip_) * 8;
        transfer.tx_buffer = strip_;
        ESP_ERROR_CHECK(spi_device_transmit(lcd_, &transfer));
    }
}

void Board::renderGame(Rect box) {
    const GamePixels pixels(box.w, gamePalette_, true);
    bool fullFrame = box.w != displayedGameBox_.w || gamePalette_ != displayedPalette_;
#ifdef ARDUGO_QUIET_TEST
    fullFrame |= benchmarkFullFrames_;
#endif
    if (fullFrame) {
        writeDisplayRows(box, 0, box.h, &pixels);
    } else {
        const auto changed = [this](int row) {
            return std::memcmp(gameFrame_ + row * GameW, displayedGame_ + row * GameW, GameW) != 0;
        };
        const auto scaled = [box](int row) {
            // Each source row begins at the first output row that maps to it.
            return (row * box.h + GameH - 1) / GameH;
        };
        for (int row = 0; row < GameH;) {
            if (!changed(row)) {
                ++row;
                continue;
            }
            const int first = row++;
            while (row < GameH && changed(row)) ++row;
            writeDisplayRows(box, scaled(first), scaled(row), &pixels);
        }
    }
    std::memcpy(displayedGame_, gameFrame_, sizeof(displayedGame_));
    displayedGameBox_ = box;
    displayedPalette_ = gamePalette_;
}

void Board::renderDisplay() {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const auto box = region_;
        ESP_ERROR_CHECK(spi_device_acquire_bus(lcd_, portMAX_DELAY));
        if (renderGameFrame_) {
            renderGame(box);
        } else {
            writeDisplayRows(box, 0, box.h);
            const auto game = displayedGameBox_;
            if (box.x < game.x + game.w && game.x < box.x + box.w &&
                box.y < game.y + game.h && game.y < box.y + box.h)
                displayedGameBox_ = {};
        }
        spi_device_release_bus(lcd_);
        xSemaphoreGive(displayFree_);
    }
}

void Board::brightness(unsigned percent) {
    const unsigned duty = (percent * percent * 1023 + 9999) / 10000;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

bool Board::mountStorage() {
    if (mounted_) return true;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = Bus;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;
    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.host_id = Bus;
    slot.gpio_cs = SdCs;
    esp_vfs_fat_mount_config_t options{};
    options.max_files = 14;
    sdmmc_card_t* card = nullptr;
    esp_err_t result = esp_vfs_fat_sdspi_mount(MountPoint, &host, &slot, &options, &card);
    if (result != ESP_OK) {
        output(SdCs, true);
        ESP_LOGW(Tag, "SD mount: %s", esp_err_to_name(result));
    } else ESP_LOGI(Tag, "SD mounted");
    mounted_ = result == ESP_OK;
    return mounted_;
}

void Board::startInput() {
    for (const auto& button : Buttons) {
        gpio_config_t config{};
        config.pin_bit_mask = 1ull << button.pin;
        config.mode = GPIO_MODE_INPUT;
        config.pull_up_en = button.pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
        ESP_ERROR_CHECK(gpio_config(&config));
    }
    auto result = adc1_config_width(ADC_WIDTH_BIT_12);
    for (auto channel : {XAxis, YAxis, Battery}) {
        if (result == ESP_OK) result = adc1_config_channel_atten(channel, Attenuation);
    }
    if (result != ESP_OK) {
        ESP_LOGW(Tag, "ADC: %s", esp_err_to_name(result));
        return;
    }
    esp_adc_cal_characterize(ADC_UNIT_1, Attenuation, ADC_WIDTH_BIT_12, 1100, &calibration_);
    adcReady_ = true;
}

uint16_t Board::input() {
    uint16_t keys = 0;
    for (const auto& button : Buttons) if (!gpio_get_level(button.pin)) keys |= button.key;
    int x = 0, y = 0;
    if (adcReady_) {
        x = adc1_get_raw(XAxis);
        y = adc1_get_raw(YAxis);
    }
    const auto axis = [&](int value, Key high, Key mid) {
        if (value >= (rawKeys_ & high ? 2940 : 3072)) keys |= high;
        else if (value >= (rawKeys_ & mid ? 900 : 1024)) keys |= mid;
    };
    axis(x, Left, Right);
    axis(y, Up, Down);
    const auto now = esp_timer_get_time();
    if (rawKeys_ != keys) {
        rawKeys_ = keys;
        keyChange_ = now;
    } else if (now - keyChange_ >= 7000) stableKeys_ = keys;
    return stableKeys_;
}

unsigned Board::battery() {
    if (!adcReady_) return 101;
    int sum = 0;
    for (int i = 0; i < 8; ++i) {
        const int value = adc1_get_raw(Battery);
        if (value < 0) return 101;
        sum += value;
    }
    const int mV = esp_adc_cal_raw_to_voltage(sum / 8, &calibration_);
    if (mV < 1400) return 101;
    return std::clamp((2 * mV - 3500) * 100 / 700, 0, 100);
}

void Board::startAudio() {
    i2s_config_t config{};
#ifdef ARDUGO_QUIET_TEST
    config.mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX);
#else
    config.mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN);
#endif
    config.sample_rate = DacHz;
    config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
    config.communication_format = I2S_COMM_FORMAT_STAND_MSB;
    config.dma_desc_num = 6;
    config.dma_frame_num = DacChunk;
    config.tx_desc_auto_clear = true;
    auto result = i2s_driver_install(I2S_NUM_0, &config, 0, nullptr);
    if (result == ESP_OK) {
#ifdef ARDUGO_QUIET_TEST
        result = i2s_set_dac_mode(I2S_DAC_CHANNEL_DISABLE);
#else
        result = i2s_set_dac_mode(I2S_DAC_CHANNEL_LEFT_EN); // DAC2 on GPIO26; GPIO25 controls /SD.
#endif
        if (result != ESP_OK) i2s_driver_uninstall(I2S_NUM_0);
    }
    if (result != ESP_OK) {
        ESP_LOGW(Tag, "Speaker unavailable: %s", esp_err_to_name(result));
        return;
    }
    output(SpeakerEnable, false);
    audioReady_ = true;
    ESP_ERROR_CHECK(xTaskCreatePinnedToCore([](void* self) { static_cast<Board*>(self)->renderAudio(); },
                    "sound", 3072, this, 5, nullptr, 0) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

void Board::renderAudio() {
    std::array<uint8_t, DacChunk> data{};
    struct DacFrame { uint16_t left, right; };
    std::array<DacFrame, DacChunk> stereo{};
    for (;;) {
        size_t count = 0;
        uint32_t gen = 0;
        portENTER_CRITICAL(&samplesLock_);
        gen = audioGen_;
        size_t avail = ringCount_;
        if (!audioPrimed_ && avail > 0 && avail < DacChunk * 3) {
            for (int wait = 0; wait < 4 && avail > 0 && avail < DacChunk * 3; ++wait) {
                portEXIT_CRITICAL(&samplesLock_);
                vTaskDelay(pdMS_TO_TICKS(2));
                portENTER_CRITICAL(&samplesLock_);
                gen = audioGen_;
                avail = ringCount_;
            }
            if (avail > 0) audioPrimed_ = true;
        } else if (!audioPrimed_ && avail >= DacChunk * 3) {
            audioPrimed_ = true;
        }
        if (audioPrimed_ && avail > 0) {
            count = std::min(data.size(), avail);
            for (size_t i = 0; i < count; ++i) {
                data[i] = audioRing_[ringHead_];
                ringHead_ = (ringHead_ + 1) & (audioRing_.size() - 1);
            }
            ringCount_ -= count;
            audioEmptyChunks_ = 0;
        } else if (audioPrimed_) {
            if (++audioEmptyChunks_ >= 12) audioPrimed_ = false;
        }
        portEXIT_CRITICAL(&samplesLock_);

        if (gen != renderGen_) {
            renderGen_ = gen;
            lastAudioSample_ = 128;
            silentRenderSamples_ = 4800;
        }

        if (count > 0) lastAudioSample_ = data[count - 1];
        // Fade an underrun to the DAC midpoint without repeating an old
        // waveform. Repetition introduces extra tones in polyphonic music.
        for (size_t i = count; i < data.size(); ++i) {
            const int difference = int(lastAudioSample_) - 128;
            lastAudioSample_ = uint8_t(128 + difference * 63 / 64);
            data[i] = lastAudioSample_;
        }
        bool hasSignal = false;
        for (size_t i = 0; i < data.size(); ++i) {
            if (data[i] != 128) {
                hasSignal = true;
                break;
            }
        }
        silentRenderSamples_ = hasSignal ? 0 : silentRenderSamples_ + data.size();
        portENTER_CRITICAL(&samplesLock_);
        const bool enable = audioActive_ && silentRenderSamples_ < 4800 && gen == audioGen_;
        if (speakerPinOn_ != enable) {
            speakerPinOn_ = enable;
#ifndef ARDUGO_QUIET_TEST
            gpio_set_level(SpeakerEnable, enable);
#endif
        }
        portEXIT_CRITICAL(&samplesLock_);
        for (size_t i = 0; i < stereo.size(); ++i) {
            const uint16_t dac = uint16_t(data[i] << 8);
            stereo[i] = {dac, dac};
        }
        size_t written = 0;
        if (i2s_write(I2S_NUM_0, stereo.data(), sizeof(stereo), &written, pdMS_TO_TICKS(50)) != ESP_OK ||
            written != sizeof(stereo)) {
            vTaskDelay(1);
            continue;
        }
#ifdef ARDUGO_QUIET_TEST
        nonzeroDmaSamples_.fetch_add(std::count_if(data.begin(), data.begin() + count,
            [](uint8_t sample) { return sample != 128; }), std::memory_order_relaxed);
#endif
    }
}

#ifdef ARDUGO_QUIET_TEST
void Board::probeAudioDma() {
    // The DAC remains disabled. This exercises the game sample queue and DMA.
    startAudio();
    std::array<int16_t, AudioHz / 250> wave{};
    for (unsigned i = 0; i < wave.size(); ++i) wave[i] = i % 50 < 25 ? 1000 : -1000;
    const auto started = esp_timer_get_time();
    int64_t resampleUs = 0;
    for (unsigned i = 0; i < 40; ++i) {
        const auto before = esp_timer_get_time();
        audio(wave.data(), wave.size(), 50, 100);
        resampleUs += esp_timer_get_time() - before;
        vTaskDelay(pdMS_TO_TICKS(4));
    }
    ESP_LOGI("bench", "PCM producer: %lld us across %lld us at 50 kHz",
             static_cast<long long>(resampleUs), static_cast<long long>(esp_timer_get_time() - started));
    vTaskDelay(pdMS_TO_TICKS(30));
    silence();
    vTaskDelay(pdMS_TO_TICKS(30));
    const auto count = nonzeroDmaSamples_.load(std::memory_order_relaxed);
    ESP_LOGI("bench", "Silent I2S DMA: %u nonzero samples submitted, DAC disabled", count);
    wave.fill(0);
    for (unsigned i = 0; i < 100; ++i) {
        audio(wave.data(), wave.size(), 50, 100);
        vTaskDelay(pdMS_TO_TICKS(4));
    }
    portENTER_CRITICAL(&samplesLock_);
    const bool muted = !speakerPinOn_;
    portEXIT_CRITICAL(&samplesLock_);
    ESP_ERROR_CHECK(muted ? ESP_OK : ESP_FAIL);
    silence();
    ESP_LOGI("bench", "Continuous silent samples correctly mute the amplifier");
}
#endif

void Board::audio(const int16_t* samples, size_t count, unsigned volume, unsigned speed) {
    if (!audioReady_ || count == 0) return;
    if (volume == 0) {
        if (audioActive_) silence();
        return;
    }
    const uint32_t nominalRate = std::max<uint32_t>(1, AudioHz * speed / 100);
    const int64_t nowUs = esp_timer_get_time();
    if (!audioActive_) {
        audioActive_ = true;
        audioPhase_ = 0;
        audioAcc_ = 0;
        audioRate_ = nominalRate;
        paceWindowUs_ = nowUs;
        paceSamples_ = 0;
        dac_.reset();
    } else {
        paceSamples_ += count;
        const int64_t dt = nowUs - paceWindowUs_;
        if (dt >= 80000) {
            if (dt <= 300000) {
                size_t avail = 0;
                portENTER_CRITICAL(&samplesLock_);
                avail = ringCount_;
                portEXIT_CRITICAL(&samplesLock_);
                // Preserve true musical pitch: never transpose audio down to follow FPS drops.
                // Allow at most +-0.5% micro-trim for clock crystal drift when the ring is active.
                int32_t trim = 0;
                if (avail >= DacChunk * 2) {
                    const int32_t fillErr = int32_t(avail) - int32_t(AudioTargetFill);
                    const int32_t maxTrim = std::max<int32_t>(1, int32_t(nominalRate / 200));
                    trim = std::clamp<int32_t>(fillErr * int32_t(nominalRate) / 65536,
                                               -maxTrim, maxTrim);
                }
                const uint32_t target = uint32_t(int32_t(nominalRate) + trim);
                audioRate_ = (audioRate_ * 3u + target) / 4u;
            }
            paceWindowUs_ = nowUs;
            paceSamples_ = 0;
        }
    }
    const uint32_t rate = std::max<uint32_t>(1, audioRate_);
    std::array<uint8_t, DacChunk> out{};
    size_t filled = 0;
    auto pushChunk = [&](const uint8_t* src, size_t n) {
        if (n == 0) return;
        portENTER_CRITICAL(&samplesLock_);
        const size_t space = audioRing_.size() - ringCount_;
        const size_t take = std::min(n, space);
        for (size_t k = 0; k < take; ++k) {
            audioRing_[ringTail_] = src[k];
            ringTail_ = (ringTail_ + 1) & (audioRing_.size() - 1);
        }
        ringCount_ += take;
        portEXIT_CRITICAL(&samplesLock_);
    };
    for (size_t i = 0; i < count; ++i) {
        uint32_t rem = DacHz;
        const int64_t sample = samples[i];
        while (audioPhase_ + rem >= rate) {
            const uint32_t take = rate - audioPhase_;
            audioAcc_ += sample * int64_t(take);
            const int32_t avg = int32_t(audioAcc_ / int64_t(rate));
            const uint8_t dacVal = dac_.convert(avg, volume);
            out[filled++] = dacVal;
            if (filled == out.size()) {
                pushChunk(out.data(), filled);
                filled = 0;
            }
            audioAcc_ = 0;
            audioPhase_ = 0;
            rem -= take;
        }
        audioPhase_ += rem;
        audioAcc_ += sample * int64_t(rem);
    }
    pushChunk(out.data(), filled);
}

void Board::silence() {
    audioActive_ = false;
    audioRate_ = AudioHz;
    audioPhase_ = 0;
    audioAcc_ = 0;
    paceWindowUs_ = 0;
    paceSamples_ = 0;
    dac_.reset();
    portENTER_CRITICAL(&samplesLock_);
#ifndef ARDUGO_QUIET_TEST
    gpio_set_level(SpeakerEnable, false);
#endif
    ringHead_ = 0;
    ringTail_ = 0;
    ringCount_ = 0;
    audioPrimed_ = false;
    speakerPinOn_ = false;
    audioEmptyChunks_ = 0;
    ++audioGen_;
    portEXIT_CRITICAL(&samplesLock_);
}

bool Board::canExit() {
    const auto* running = esp_ota_get_running_partition();
    return running && running->subtype != ESP_PARTITION_SUBTYPE_APP_FACTORY &&
        esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, nullptr);
}

void Board::exit() {
    const auto* factory = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, nullptr);
    if (!factory) return;
    esp_err_t result = esp_ota_set_boot_partition(factory);
    if (result == ESP_OK) esp_restart();
    ESP_LOGW(Tag, "Launcher: %s", esp_err_to_name(result));
}

}
