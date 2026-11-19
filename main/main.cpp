#include "app.hpp"
#include "board.hpp"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"

extern "C" void app_main() {
    ESP_LOGI("arduboy-go", "Arduboy Go starting, PSRAM free: %u",
             unsigned(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
    static go::Board board;
    board.begin();
    static go::App app(board, "/sd");
    app.start();

    for (;;) {
        app.tick(esp_timer_get_time(), board.input());
        vTaskDelay(1);
    }
}
