#pragma once

#include "types.hpp"

#include "driver/dac_continuous.h"
#include "driver/spi_master.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"

namespace go {

class Board final : public Platform {
public:
    void begin();
    uint16_t input();
    void present(const uint16_t* pixels, Rect region) override;
    bool tryPresent(const uint16_t* pixels, Rect region) override;
    void brightness(unsigned percent) override;
    void audio(const int16_t* samples, size_t count, unsigned volume, unsigned speed) override;
    void silence() override;
    unsigned battery() override;
    bool mountStorage() override;
    bool canExit() override;
    void exit() override;

private:
    void startDisplay();
    void startInput();
    void startAudio();
    void lcdCommand(uint8_t command, const uint8_t* data = nullptr, size_t size = 0);
    void lcdData(const void* data, size_t size);
    void queueDisplay(const uint16_t* pixels, Rect region);
    void renderDisplay();
    void renderAudio();
    static bool audioReady(dac_continuous_handle_t, const dac_event_data_t*, void*);

    spi_device_handle_t lcd_ = nullptr;
    uint16_t* frame_ = nullptr;
    uint16_t* strip_ = nullptr;
    Rect region_{};
    SemaphoreHandle_t displayFree_ = nullptr;
    TaskHandle_t displayTask_ = nullptr;

    adc_oneshot_unit_handle_t adc_ = nullptr;
    adc_cali_handle_t calibration_ = nullptr;
    uint16_t rawKeys_ = 0;
    uint16_t stableKeys_ = 0;
    int64_t keyChange_ = 0;

    dac_continuous_handle_t dac_ = nullptr;
    QueueHandle_t dacEvents_ = nullptr;
    StreamBufferHandle_t samples_ = nullptr;
    portMUX_TYPE samplesLock_ = portMUX_INITIALIZER_UNLOCKED;
    int64_t audioWindow_ = 0;
    size_t producedSamples_ = 0;
    unsigned audioRate_ = 0;
    unsigned audioPhase_ = 0;
    bool audioPrimed_ = false;
    bool speakerOn_ = false;
    bool mounted_ = false;
};

}
