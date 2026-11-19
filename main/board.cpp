#include "board.hpp"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_cali_scheme.h"
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
constexpr auto XAxis = ADC_CHANNEL_6;
constexpr auto YAxis = ADC_CHANNEL_7;
constexpr auto Battery = ADC_CHANNEL_0;
constexpr auto Attenuation = ADC_ATTEN_DB_12;
constexpr int StripRows = 12;
constexpr unsigned DacHz = 32000;
constexpr size_t DacChunk = 128;
constexpr const char* MountPoint = "/sd";

struct DigitalButton { gpio_num_t pin; Key key; bool pullup; };
constexpr DigitalButton Buttons[] = {
    {GPIO_NUM_32, A, true}, {GPIO_NUM_33, B, true},
    {GPIO_NUM_13, Menu, true}, {GPIO_NUM_0, Volume, false},
    {GPIO_NUM_27, Select, true}, {GPIO_NUM_39, Start, false},
};

void output(gpio_num_t pin, bool high) {
    ESP_ERROR_CHECK(gpio_set_level(pin, high));
    ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_OUTPUT));
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
    ESP_ERROR_CHECK(spi_bus_initialize(Bus, &bus, SPI_DMA_CH_AUTO));

    // Put the card in SPI mode before initializing the LCD on the shared bus.
    mountStorage();
    startDisplay();
    startInput();
    startAudio();
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
    lcdCommand(0x29);

    frame_ = static_cast<uint16_t*>(heap_caps_malloc(ScreenW * ScreenH * sizeof(uint16_t),
                                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    strip_ = static_cast<uint16_t*>(heap_caps_malloc(ScreenW * StripRows * sizeof(uint16_t),
                                                    MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    displayFree_ = xSemaphoreCreateBinary();
    ESP_ERROR_CHECK(frame_ && strip_ && displayFree_ ? ESP_OK : ESP_ERR_NO_MEM);
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
    for (int y = 0; y < region.h; ++y) {
        const auto* row = pixels + (region.y + y) * ScreenW + region.x;
        std::copy_n(row, region.w, frame_ + y * region.w);
    }
    region_ = region;
    xTaskNotifyGive(displayTask_);
}

void Board::renderDisplay() {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const auto box = region_;
        const int right = box.x + box.w - 1;
        const int bottom = box.y + box.h - 1;
        const uint8_t columns[] = {uint8_t(box.x >> 8), uint8_t(box.x), uint8_t(right >> 8), uint8_t(right)};
        const uint8_t rows[] = {uint8_t(box.y >> 8), uint8_t(box.y), uint8_t(bottom >> 8), uint8_t(bottom)};
        ESP_ERROR_CHECK(spi_device_acquire_bus(lcd_, portMAX_DELAY));
        lcdCommand(0x2a, columns, sizeof(columns));
        lcdCommand(0x2b, rows, sizeof(rows));
        lcdCommand(0x2c);
        for (int y = 0; y < box.h; y += StripRows) {
            const int n = std::min(StripRows, box.h - y) * box.w;
            const auto* src = frame_ + y * box.w;
            for (int i = 0; i < n; ++i) strip_[i] = __builtin_bswap16(src[i]);
            // Interrupt-driven DMA allows the idle and audio tasks to run during the transfer.
            spi_transaction_t transfer{};
            transfer.length = n * sizeof(*strip_) * 8;
            transfer.tx_buffer = strip_;
            ESP_ERROR_CHECK(spi_device_transmit(lcd_, &transfer));
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
    adc_oneshot_unit_init_cfg_t unit{};
    unit.unit_id = ADC_UNIT_1;
    esp_err_t result = adc_oneshot_new_unit(&unit, &adc_);
    if (result != ESP_OK) {
        ESP_LOGW(Tag, "ADC: %s", esp_err_to_name(result));
        return;
    }
    adc_oneshot_chan_cfg_t config{};
    config.atten = Attenuation;
    config.bitwidth = ADC_BITWIDTH_12;
    for (auto channel : {XAxis, YAxis, Battery}) adc_oneshot_config_channel(adc_, channel, &config);
    adc_cali_line_fitting_config_t cal{};
    cal.unit_id = ADC_UNIT_1;
    cal.atten = Attenuation;
    cal.bitwidth = ADC_BITWIDTH_12;
    cal.default_vref = 1100;
    adc_cali_create_scheme_line_fitting(&cal, &calibration_);
}

uint16_t Board::input() {
    uint16_t keys = 0;
    for (const auto& button : Buttons) if (!gpio_get_level(button.pin)) keys |= button.key;
    int x = 0, y = 0;
    if (adc_) {
        adc_oneshot_read(adc_, XAxis, &x);
        adc_oneshot_read(adc_, YAxis, &y);
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
    if (!adc_ || !calibration_) return 101;
    int sum = 0;
    for (int i = 0; i < 8; ++i) {
        int value = 0;
        if (adc_oneshot_read(adc_, Battery, &value) != ESP_OK) return 101;
        sum += value;
    }
    int mV = 0;
    if (adc_cali_raw_to_voltage(calibration_, sum / 8, &mV) != ESP_OK || mV < 1400) return 101;
    return std::clamp((2 * mV - 3500) * 100 / 700, 0, 100);
}

void Board::startAudio() {
    dac_continuous_config_t config{};
    config.chan_mask = DAC_CHANNEL_MASK_CH1; // DAC2, GPIO26.
    config.desc_num = 4;
    config.buf_size = DacChunk * 2;
    config.freq_hz = DacHz;
    config.clk_src = DAC_DIGI_CLK_SRC_DEFAULT;
    config.chan_mode = DAC_CHANNEL_MODE_SIMUL;
    esp_err_t result = dac_continuous_new_channels(&config, &dac_);
    if (result != ESP_OK) {
        ESP_LOGW(Tag, "Speaker unavailable: %s", esp_err_to_name(result));
        return;
    }
    dacEvents_ = xQueueCreate(config.desc_num, sizeof(dac_event_data_t));
    samples_ = xStreamBufferCreate(2048, 1);
    ESP_ERROR_CHECK(dacEvents_ && samples_ ? ESP_OK : ESP_ERR_NO_MEM);
    dac_event_callbacks_t callbacks{};
    callbacks.on_convert_done = audioReady;
    ESP_ERROR_CHECK(dac_continuous_register_event_callback(dac_, &callbacks, this));
    ESP_ERROR_CHECK(dac_continuous_enable(dac_));
    ESP_ERROR_CHECK(dac_continuous_start_async_writing(dac_));
    ESP_ERROR_CHECK(xTaskCreatePinnedToCore([](void* self) { static_cast<Board*>(self)->renderAudio(); },
                    "sound", 3072, this, 5, nullptr, 0) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

bool Board::audioReady(dac_continuous_handle_t, const dac_event_data_t* event, void* board) {
    auto& self = *static_cast<Board*>(board);
    BaseType_t awakened = pdFALSE;
    if (xQueueSendFromISR(self.dacEvents_, event, &awakened) != pdPASS) {
        dac_event_data_t ignored{};
        xQueueReceiveFromISR(self.dacEvents_, &ignored, &awakened);
        xQueueSendFromISR(self.dacEvents_, event, &awakened);
    }
    return awakened == pdTRUE;
}

void Board::renderAudio() {
    std::array<uint8_t, DacChunk> data{};
    for (;;) {
        dac_event_data_t event{};
        if (xQueueReceive(dacEvents_, &event, portMAX_DELAY) != pdPASS) continue;
        const auto count = xStreamBufferReceive(samples_, data.data(), data.size(), 0);
        std::fill(data.begin() + count, data.end(), 128);
        dac_continuous_write_asynchronously(dac_, static_cast<uint8_t*>(event.buf), event.buf_size, data.data(), data.size(), nullptr);
    }
}

void Board::audio(const int16_t* samples, size_t count, unsigned volume, unsigned speed) {
    if (!dac_) return;
    if (volume == 0) {
        if (speakerOn_) silence();
        return;
    }
    if (!speakerOn_) {
        gpio_set_level(SpeakerEnable, true);
        speakerOn_ = true;
    }
    std::array<uint8_t, 256> out{};
    size_t filled = 0;
    const unsigned threshold = AudioHz * speed;
    for (size_t i = 0; i < count; ++i) {
        audioPhase_ += DacHz * 100;
        while (audioPhase_ >= threshold) {
            audioPhase_ -= threshold;
            out[filled++] = std::clamp(128 + samples[i] * int(volume) / 2400, 0, 255);
            if (filled == out.size()) {
                xStreamBufferSend(samples_, out.data(), filled, 0);
                filled = 0;
            }
        }
    }
    if (filled) xStreamBufferSend(samples_, out.data(), filled, 0);
}

void Board::silence() {
    gpio_set_level(SpeakerEnable, false);
    speakerOn_ = false;
    audioPhase_ = 0;
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
