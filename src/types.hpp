#pragma once

#include <cstddef>
#include <cstdint>

namespace go {

constexpr int ScreenW = 320;
constexpr int ScreenH = 240;
constexpr int GameW = 128;
constexpr int GameH = 64;
constexpr int AudioHz = 50000;

enum Key : uint16_t {
    Up = 1 << 0, Down = 1 << 1, Left = 1 << 2, Right = 1 << 3,
    A = 1 << 4, B = 1 << 5, Menu = 1 << 6, Volume = 1 << 7,
    Select = 1 << 8, Start = 1 << 9,
};

enum class Scale : uint8_t { Native, Double, Fit };

struct Settings {
    uint8_t brightness = 80;
    uint8_t volume = 50;
    Scale scale = Scale::Double;
    uint8_t speed = 100;
    uint8_t fps = 30;
    uint8_t palette = 0;
    bool oledFilter = false;
};

struct Rect { int x, y, w, h; };

constexpr Rect viewport(Scale scale) {
    int w = scale == Scale::Native ? 128 : scale == Scale::Double ? 256 : 320;
    int h = w / 2;
    return {(ScreenW - w) / 2, (ScreenH - h) / 2, w, h};
}

struct Platform {
    virtual ~Platform() = default;
    virtual void present(const uint16_t* pixels, Rect region) = 0;
    virtual void brightness(unsigned percent) = 0;
    virtual void audio(const int16_t* samples, size_t count, unsigned volume, unsigned speed) = 0;
    virtual void silence() = 0;
    virtual unsigned battery() { return 101; }
    virtual bool mountStorage() { return true; }
    virtual bool canExit() { return false; }
    virtual void exit() {}
};

}
