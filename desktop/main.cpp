#include "app.hpp"
#include <SDL.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>

namespace {

constexpr unsigned HostAudioHz = 32000;

struct Desktop final : go::Platform {
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* texture = nullptr;
    SDL_AudioDeviceID device = 0;
    unsigned light = 100, phase = 0;

    void present(const uint16_t* pixels, go::Rect r) override {
        const SDL_Rect rect{r.x, r.y, r.w, r.h};
        SDL_UpdateTexture(texture, &rect, pixels + r.y * go::ScreenW + r.x, go::ScreenW * sizeof(*pixels));
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);
    }

    void brightness(unsigned percent) override {
        light = 40 + 60 * percent / 100;
        SDL_SetTextureColorMod(texture, 255 * light / 100, 255 * light / 100, 255 * light / 100);
    }

    void audio(const int16_t* samples, size_t count, unsigned volume, unsigned speed) override {
        if (!device || !volume || SDL_GetQueuedAudioSize(device) >= HostAudioHz / 8) return;
        std::array<int16_t, 256> output{};
        size_t filled = 0;
        const unsigned threshold = go::AudioHz * speed;
        for (size_t i = 0; i < count; ++i) {
            phase += HostAudioHz * 100;
            while (phase >= threshold) {
                phase -= threshold;
                output[filled++] = samples[i] * int(volume) / 20;
                if (filled == output.size()) {
                    SDL_QueueAudio(device, output.data(), filled * sizeof(int16_t));
                    filled = 0;
                }
            }
        }
        if (filled) SDL_QueueAudio(device, output.data(), filled * sizeof(int16_t));
    }

    void silence() override {
        if (device) SDL_ClearQueuedAudio(device);
        phase = 0;
    }
};

uint16_t buttons() {
    const auto* keys = SDL_GetKeyboardState(nullptr);
    return (keys[SDL_SCANCODE_UP] ? go::Up : 0) |
           (keys[SDL_SCANCODE_DOWN] ? go::Down : 0) |
           (keys[SDL_SCANCODE_LEFT] ? go::Left : 0) |
           (keys[SDL_SCANCODE_RIGHT] ? go::Right : 0) |
           (keys[SDL_SCANCODE_Z] ? go::A : 0) |
           (keys[SDL_SCANCODE_X] ? go::B : 0) |
           (keys[SDL_SCANCODE_ESCAPE] ? go::Menu : 0) |
           (keys[SDL_SCANCODE_TAB] ? go::Volume : 0) |
           (keys[SDL_SCANCODE_BACKSPACE] ? go::Select : 0) |
           (keys[SDL_SCANCODE_RETURN] ? go::Start : 0);
}

}

int main(int argc, char** argv) {
    if (argc != 2 || !std::filesystem::is_directory(argv[1])) {
        std::fprintf(stderr, "Usage: %s PATH_TO_SD_CARD_ROOT\n", argv[0]);
        return 2;
    }
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::fprintf(stderr, "%s\n", SDL_GetError());
        return 1;
    }
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "0");
    SDL_Window* window = SDL_CreateWindow("Arduboy Go", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                         960, 720, SDL_WINDOW_RESIZABLE);
    SDL_Renderer* renderer = window ? SDL_CreateRenderer(window, -1, 0) : nullptr;
    SDL_Texture* texture = renderer ? SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB565,
        SDL_TEXTUREACCESS_STREAMING, go::ScreenW, go::ScreenH) : nullptr;
    if (!texture) {
        std::fprintf(stderr, "%s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }
    SDL_RenderSetLogicalSize(renderer, go::ScreenW, go::ScreenH);
    SDL_AudioSpec audio{};
    audio.freq = HostAudioHz;
    audio.format = AUDIO_S16SYS;
    audio.channels = 1;
    audio.samples = 512;

    Desktop platform;
    platform.renderer = renderer;
    platform.texture = texture;
    if (SDL_InitSubSystem(SDL_INIT_AUDIO) == 0)
        platform.device = SDL_OpenAudioDevice(nullptr, 0, &audio, nullptr, 0);
    if (platform.device) SDL_PauseAudioDevice(platform.device, 0);
    else std::fprintf(stderr, "Audio unavailable: %s\n", SDL_GetError());

    go::App app(platform, argv[1]);
    app.start();
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    bool active = true;
    while (active) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) active = false;
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_EXPOSED) {
                SDL_RenderClear(renderer);
                SDL_RenderCopy(renderer, texture, nullptr, nullptr);
                SDL_RenderPresent(renderer);
            }
        }
        const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - start);
        app.tick(elapsed.count() + 1, buttons());
        SDL_Delay(2);
    }
    const bool saved = app.flushProgress();
    if (!saved) std::fprintf(stderr, "Couldn't write game progress. Check the SD card path.\n");
    if (platform.device) SDL_CloseAudioDevice(platform.device);
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return saved ? 0 : 1;
}
