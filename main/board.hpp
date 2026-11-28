#pragma once

#include "types.hpp"
#include "audio.hpp"

#include "driver/spi_master.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <array>
#include <atomic>

namespace go {

struct GamePixels;

class Board final : public Platform {
public:
    void begin();
    uint16_t input();
    void present(const uint16_t* pixels, Rect region) override;
    bool tryPresent(const uint16_t* pixels, Rect region) override;
    bool tryPresentGame(Canvas& canvas, const uint8_t* image, Scale scale, unsigned palette) override;
    void playbackStats(unsigned fps, unsigned avrPercent) override;
    void brightness(unsigned percent) override;
    void audio(const int16_t* samples, size_t count, unsigned volume, unsigned speed) override;
    void silence() override;
    unsigned battery() override;
    bool mountStorage() override;
    bool canExit() override;
    void exit() override;
#ifdef ARDUGO_QUIET_TEST
    void probeAudioDma();
    void probeLcdThroughput();
    struct LcdStats { unsigned bytes, windows; };
    void beginLcdMeasurement(bool fullFrames);
    LcdStats endLcdMeasurement();
#endif

private:
    void startDisplay();
    void startInput();
    void startAudio();
    void lcdCommand(uint8_t command, const uint8_t* data = nullptr, size_t size = 0);
    void lcdData(const void* data, size_t size);
    void queueDisplay(const uint16_t* pixels, Rect region);
    void writeDisplayRows(Rect box, int first, int end, const GamePixels* gamePixels = nullptr);
    void renderGame(Rect box);
    void renderDisplay();
    void renderAudio();

    spi_device_handle_t lcd_ = nullptr;
    uint16_t* frame_ = nullptr;
    uint16_t* strip_ = nullptr;
    uint8_t gameFrame_[GameW * GameH]{};
    uint8_t displayedGame_[GameW * GameH]{};
    unsigned gamePalette_ = 0;
    unsigned displayedPalette_ = 0;
    Rect displayedGameBox_{};
    bool renderGameFrame_ = false;
#ifdef ARDUGO_QUIET_TEST
    bool benchmarkFullFrames_ = false;
    LcdStats lcdStats_{};
#endif
    Rect region_{};
    SemaphoreHandle_t displayFree_ = nullptr;
    TaskHandle_t displayTask_ = nullptr;

    esp_adc_cal_characteristics_t calibration_{};
    bool adcReady_ = false;
    uint16_t rawKeys_ = 0;
    uint16_t stableKeys_ = 0;
    int64_t keyChange_ = 0;

    static constexpr size_t AudioRingCap = 4096;
    std::array<uint8_t, AudioRingCap> audioRing_{};
    size_t ringHead_ = 0;
    size_t ringTail_ = 0;
    size_t ringCount_ = 0;
    portMUX_TYPE samplesLock_ = portMUX_INITIALIZER_UNLOCKED;
    uint32_t audioRate_ = AudioHz;
    uint32_t audioPhase_ = 0;
    int64_t audioAcc_ = 0;
    int64_t paceWindowUs_ = 0;
    size_t paceSamples_ = 0;
    DacConverter dac_;
    size_t silentRenderSamples_ = 0;
    uint32_t audioGen_ = 0;
    uint32_t renderGen_ = 0;
    uint8_t lastAudioSample_ = 128;
    uint8_t audioEmptyChunks_ = 0;
    bool audioReady_ = false;
    std::atomic<bool> audioActive_{false};
    bool audioPrimed_ = false;
    bool speakerPinOn_ = false;
#ifdef ARDUGO_QUIET_TEST
    std::atomic<unsigned> nonzeroDmaSamples_{0};
#endif
    bool mounted_ = false;
};

}
