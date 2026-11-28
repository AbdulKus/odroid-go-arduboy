#include "app.hpp"
#include "board.hpp"
#include "absim_jit_xtensa.hpp"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"

#ifdef ARDUGO_RUN_BENCHMARKS
#include "absim.hpp"
#include <algorithm>
#include <array>
#include <cstring>

namespace {

void measureGame(go::Board& board, go::Canvas& canvas, const go::Game& game,
                 const std::string& saveDir, const char* name, unsigned warmupSlices = 300) {
    go::Core core;
    const auto error = core.load(game, saveDir);
    if (!error.empty()) {
        ESP_LOGW("bench", "Load %s: %s", name, error.c_str());
        return;
    }
    ESP_LOGI("bench", "Measuring %s", name);
    std::array<uint8_t, go::GameW * go::GameH> lastFrame{};
    for (unsigned i = 0; i < warmupSlices; ++i) {
        const bool pressA = (i >= 250 && i < 280) ||
            (warmupSlices > 300 && i >= 1000 && i < warmupSlices - 125 && i % 250 < 50);
        core.run(4000, 100, pressA ? go::A : 0, false);
        if (i % 8 == 0) vTaskDelay(1);
    }

    board.beginLcdMeasurement(false);
    const auto start = esp_timer_get_time();
    const auto startCycles = core.cycles();
    core.clearBenchmark();
    auto nextFrame = start;
    int64_t coreTime = 0;
    unsigned frames = 0, nonzeroAudioSlices = 0;
    for (unsigned i = 0; i < 500; ++i) {
        const auto before = esp_timer_get_time();
        const uint16_t buttons = (i / 75) % 2 ? go::A | go::Up | go::Right : go::A | go::Up;
        core.run(4000, 100, buttons, false);
        const auto now = esp_timer_get_time();
        coreTime += now - before;
        const auto& samples = core.samples();
        nonzeroAudioSlices += std::any_of(samples.begin(), samples.end(), [](int v) { return v != 0; });
        const auto* pixels = core.pixels();
        if (now >= nextFrame && std::memcmp(pixels, lastFrame.data(), lastFrame.size()) &&
            board.tryPresentGame(canvas, pixels, go::Scale::Fit, 0)) {
            nextFrame = now + 16667;
            std::memcpy(lastFrame.data(), pixels, lastFrame.size());
            ++frames;
        }
        if (i % 8 == 0) vTaskDelay(1);
    }
    const auto stats = board.endLcdMeasurement();
    core.reportBenchmark();
    ESP_LOGI("bench", "game rows %s cycles=%llu core_us=%lld total_us=%lld frames=%u bytes=%u windows=%u nonzero_audio_slices=%u",
             name, static_cast<unsigned long long>(core.cycles() - startCycles),
             static_cast<long long>(coreTime), static_cast<long long>(esp_timer_get_time() - start),
             frames, stats.bytes, stats.windows, nonzeroAudioSlices);
}

void runQuietBenchmarks(go::Board& board) {
    if (!board.mountStorage()) return;
    go::Catalog catalog("/sd");
    if (!catalog.scan()) return;

    go::Canvas canvas;
    const char* const suffixes[] = {
        "/'Ard Drivin'.arduboy", "/ArduRacerFx.arduboy", "/Dazzle Dash.arduboy",
        "/Pong.arduboy", "/Tetris.arduboy",
    };
    for (const char* suffix : suffixes) {
        const std::string name = suffix;
        const auto game = std::find_if(catalog.games.begin(), catalog.games.end(), [&](const go::Game& item) {
            return item.path.size() >= name.size() &&
                item.path.compare(item.path.size() - name.size(), name.size(), name) == 0;
        });
        if (game == catalog.games.end()) {
            ESP_LOGW("bench", "ROM unavailable: %s", suffix);
            continue;
        }
        measureGame(board, canvas, *game, catalog.stateDir, suffix + 1);
    }
    const char* const reportedGames[] = {
        "/CyberHack.arduboy", "/DefenderFX.arduboy",
        "/Epic Cratres of Mass Destruction.arduboy", "/Evade 2.arduboy",
        "/Starduino Turbo.arduboy", "/The Icarus Device.arduboy",
    };
    for (const char* suffix : reportedGames) {
        const auto game = std::find_if(catalog.games.begin(), catalog.games.end(), [&](const go::Game& item) {
            const std::string name = suffix;
            return item.path.size() >= name.size() &&
                item.path.compare(item.path.size() - name.size(), name.size(), name) == 0;
        });
        if (game != catalog.games.end())
            measureGame(board, canvas, *game, catalog.stateDir, suffix + 1, 3250);
    }
}

}
#endif

extern "C" void app_main() {
    absim::jit::jit_preallocate();
    ESP_LOGI("arduboy-go", "Arduboy Go starting, PSRAM free: %u",
             unsigned(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
    static go::Board board;
    board.begin();
#ifdef ARDUGO_RUN_BENCHMARKS
    ESP_LOGI("bench", "Audio output disabled");
    ESP_LOGI("bench", "Internal RAM free=%u largest=%u core_state=%u",
             unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
             unsigned(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
             unsigned(sizeof(absim::arduboy_core_state_t)));
    board.probeLcdThroughput();
    runQuietBenchmarks(board);
    board.probeAudioDma();
    ESP_LOGI("bench", "Quiet benchmark finished (main_stack_hwm=%u)",
             unsigned(uxTaskGetStackHighWaterMark(nullptr)));
#endif
    static go::App app(board, "/sd");
    app.start();

    for (;;) {
        app.tick(esp_timer_get_time(), board.input());
        vTaskDelay(1);
    }
}
