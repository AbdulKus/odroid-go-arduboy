#pragma once

#include "catalog.hpp"

#include <string_view>

namespace go {

constexpr int VisibleRows = 7;
constexpr int PauseRows = 7;
constexpr int SettingsRows = 7;

using Color = uint16_t;
constexpr Color rgb(unsigned r, unsigned g, unsigned b) {
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
}

class Canvas {
public:
    Canvas();
    const Color* data() const { return pixels_.data(); }

    void fill(Color color);
    void rect(Rect box, Color color);
    void rounded(Rect box, Color color);
    void text(int x, int y, std::string_view value, Color color, int width = ScreenW, bool large = false);
    void preview(int x, int y, const Preview& pixels);
    void frame(const uint8_t* image, Scale scale, unsigned palette);

private:
    std::vector<Color> pixels_;

    void pixel(int x, int y, Color color);
    int glyph(int x, int y, unsigned code, Color color, bool large);
};

void drawLibrary(Canvas& canvas, const Catalog& catalog, const std::vector<size_t>& games,
                 size_t selected, size_t top, bool favorites, const Preview* preview, unsigned battery);
void drawPause(Canvas& canvas, std::string_view title, unsigned selected, unsigned slot, bool hasSave);
void drawSettings(Canvas& canvas, const Settings& settings, unsigned selected);
void drawNotice(Canvas& canvas, std::string_view title, std::string_view detail, bool confirm = false);
void drawStorageError(Canvas& canvas);
void drawActivity(Canvas& canvas, std::string_view title, std::string_view status);

}
