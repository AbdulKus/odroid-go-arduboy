#include "ui.hpp"

#include <algorithm>
#include <array>
#include <cstring>

namespace go {
namespace {

constexpr Color Background = rgb(15, 23, 34);
constexpr Color Surface = rgb(25, 37, 53);
constexpr Color Selected = rgb(32, 57, 69);
constexpr Color Line = rgb(52, 71, 86);
constexpr Color Text = rgb(240, 246, 247);
constexpr Color Muted = rgb(165, 183, 196);
constexpr Color Accent = rgb(114, 233, 207);
constexpr Color Gold = rgb(251, 196, 104);
constexpr Color Screen = rgb(7, 13, 21);

struct Glyph { uint8_t advance; uint16_t rows[21]; };
#include "font.inc"

const Glyph& lookup(unsigned code, bool large) {
    if (code >= 32 && code <= 126) return large ? headingGlyphs[code - 32] : regularGlyphs[code - 32];
    if (!large && code >= 0x400 && code <= 0x45f) return regularGlyphs[95 + code - 0x400];
    return large ? headingGlyphs['?' - 32] : regularGlyphs['?' - 32];
}

unsigned nextCode(std::string_view& text) {
    const auto first = static_cast<uint8_t>(text.front());
    text.remove_prefix(1);
    if (first < 0x80) return first;
    unsigned code;
    unsigned continuation;
    if (first >= 0xc2 && first <= 0xdf) { code = first & 0x1f; continuation = 1; }
    else if (first <= 0xef && first >= 0xe0) { code = first & 0x0f; continuation = 2; }
    else if (first <= 0xf4 && first >= 0xf0) { code = first & 7; continuation = 3; }
    else return '?';
    while (continuation--) {
        if (text.empty() || (uint8_t(text.front()) & 0xc0) != 0x80) return '?';
        code = (code << 6) | (uint8_t(text.front()) & 0x3f);
        text.remove_prefix(1);
    }
    return code;
}

void footer(Canvas& c, std::string_view controls) {
    c.rect({0, 216, 320, 24}, Surface);
    c.rect({0, 216, 320, 1}, Line);
    c.text(10, 222, controls, Muted);
}

void icon(Canvas& c, int x, int y, Color color) {
    c.rounded({x, y, 20, 20}, color);
    c.rect({x + 4, y + 4, 12, 7}, Background);
    c.rect({x + 5, y + 14, 5, 1}, Background);
    c.rect({x + 7, y + 12, 1, 5}, Background);
    c.rect({x + 13, y + 13, 2, 2}, Background);
}

void header(Canvas& c, std::string_view title) {
    c.fill(Background);
    icon(c, 12, 13, Accent);
    c.text(41, 11, title, Text, 250, true);
    c.rect({12, 40, 296, 1}, Line);
}

void navRow(Canvas& c, unsigned row, unsigned selected, std::string_view label, std::string_view value = {}) {
    const int y = 46 + row * 21;
    if (row == selected) {
        c.rounded({12, y, 296, 21}, Selected);
        c.rect({12, y + 4, 2, 13}, Accent);
    } else {
        c.rect({22, y + 20, 276, 1}, Surface);
    }
    c.text(23, y + 4, label, Text, 174);
    c.text(204, y + 4, value, row == selected ? Accent : Muted, 98);
}

struct PaletteSpec {
    const char* name;
    uint8_t stops[4][3];
};

constexpr PaletteSpec Palettes[PaletteCount] = {
    {"White",       {{  0,   0,   0}, { 82,  84,  86}, {165, 168, 171}, {245, 250, 252}}},
    {"Mint",        {{  0,   0,   0}, { 28, 118, 128}, { 72, 202, 165}, {135, 252, 195}}},
    {"Amber",       {{  0,   0,   0}, {120,  42,   8}, {215, 120,  30}, {255, 212, 115}}},
    {"GB DMG",      {{155, 188,  15}, {139, 172,  15}, { 48,  98,  48}, { 15,  56,  15}}},
    {"GB Pocket",   {{196, 207, 161}, {139, 149, 109}, { 77,  83,  60}, { 31,  31,  31}}},
    {"GB Light",    {{  0, 210, 184}, {  0, 160, 145}, {  8,  96,  92}, {  8,  36,  44}}},
    {"GBC Classic", {{248, 240, 216}, {240, 168,  64}, {120, 168,  64}, { 24,  32,  48}}},
    {"GBC Pastel",  {{248, 236, 228}, {240, 144, 168}, {112, 188, 200}, { 36,  32,  76}}},
    {"GBC Dark",    {{  0,   0,   0}, { 56, 112, 232}, {120, 224, 144}, {252, 244, 210}}},
    {"GBC Ember",   {{  0,   0,   0}, {176,  36,  48}, {248, 144,  32}, {255, 244, 176}}},
    {"Virtual Boy", {{  0,   0,   0}, {104,   0,   0}, {200,  16,   8}, {255,  56,  28}}},
    {"Ice Blue",    {{  0,   0,   0}, { 36,  96, 176}, { 96, 184, 244}, {216, 244, 255}}},
};

}

Canvas::Canvas() : pixels_(ScreenW * ScreenH) {}

void Canvas::fill(Color color) { std::fill(pixels_.begin(), pixels_.end(), color); }

void Canvas::pixel(int x, int y, Color color) {
    if (x >= 0 && x < ScreenW && y >= 0 && y < ScreenH) pixels_[y * ScreenW + x] = color;
}

void Canvas::rect(Rect box, Color color) {
    const int left = std::max(0, box.x);
    const int right = std::min(ScreenW, box.x + box.w);
    if (left >= right) return;
    for (int y = std::max(0, box.y); y < std::min(ScreenH, box.y + box.h); ++y)
        std::fill_n(pixels_.data() + y * ScreenW + left, right - left, color);
}

void Canvas::rounded(Rect box, Color color) {
    rect({box.x + 3, box.y, box.w - 6, box.h}, color);
    rect({box.x + 1, box.y + 1, box.w - 2, box.h - 2}, color);
    rect({box.x, box.y + 3, box.w, box.h - 6}, color);
}

int Canvas::glyph(int x, int y, unsigned code, Color color, bool large) {
    const auto& g = lookup(code, large);
    for (int row = 0; row < (large ? 21 : 14); ++row) {
        unsigned bits = g.rows[row];
        while (bits) {
            int column = __builtin_ctz(bits);
            pixel(x + column, y + row, color);
            bits &= bits - 1;
        }
    }
    return g.advance;
}

void Canvas::text(int x, int y, std::string_view value, Color color, int width, bool large) {
    const int end = std::min(ScreenW, x + width);
    const int ellipsis = lookup('.', large).advance * 3;
    int length = 0;
    for (auto rest = value; !rest.empty();) length += lookup(nextCode(rest), large).advance;
    const bool clipped = length > end - x;
    while (!value.empty()) {
        const unsigned code = nextCode(value);
        const auto advance = lookup(code, large).advance;
        if (x + advance > end - (clipped ? ellipsis : 0)) {
            for (int i = 0; i < 3 && x + lookup('.', large).advance <= end; ++i)
                x += glyph(x, y, '.', color, large);
            break;
        }
        x += glyph(x, y, code, color, large);
    }
}

void Canvas::preview(int x, int y, const Preview& image) {
    for (unsigned row = 0; row < PreviewH; ++row)
        std::copy_n(image.data() + row * PreviewW, PreviewW, pixels_.data() + (y + row) * ScreenW + x);
}

GamePixels::GamePixels(int width, unsigned palette, bool swapBytes) : width(width) {
    const auto& stops = Palettes[palette % PaletteCount].stops;
    for (unsigned i = 0; i < colors.size(); ++i) {
        const unsigned seg = std::min<unsigned>(i * 3u / 255u, 2u);
        const unsigned lo = seg * 85u;
        const unsigned t = i - lo;
        const auto chan = [&](int c) {
            const int a = stops[seg][c];
            const int b = stops[seg + 1][c];
            return unsigned(a + (b - a) * int(t) / 85);
        };
        const auto color = rgb(chan(0), chan(1), chan(2));
        colors[i] = swapBytes ? __builtin_bswap16(color) : color;
    }
    for (int x = 0; x < width; ++x) columns[x] = x * GameW / width;
}

void GamePixels::row(uint16_t* destination, const uint8_t* source) const {
    for (int x = 0; x < width; ++x) destination[x] = colors[source[columns[x]]];
}

void Canvas::frame(const uint8_t* image, Scale scale, unsigned palette) {
    const auto view = viewport(scale);
    const GamePixels pixels(view.w, palette);
    int lastRow = -1;
    for (int y = 0; y < view.h; ++y) {
        auto* row = pixels_.data() + (view.y + y) * ScreenW + view.x;
        const int sourceY = y * GameH / view.h;
        if (sourceY == lastRow) std::copy_n(row - ScreenW, view.w, row);
        else {
            pixels.row(row, image + sourceY * GameW);
            lastRow = sourceY;
        }
    }
}

bool Platform::tryPresentGame(Canvas& canvas, const uint8_t* image, Scale scale, unsigned palette) {
    canvas.frame(image, scale, palette);
    return tryPresent(canvas.data(), viewport(scale));
}

void drawLibrary(Canvas& c, const Catalog& catalog, const std::vector<size_t>& games,
                 size_t selected, size_t top, bool favorites, const Preview* preview, unsigned battery) {
    c.fill(Background);
    const bool favorite = !games.empty() && catalog.games[games[selected]].favorite;
    constexpr std::string_view controls[] = {"Settings", "Tab", "Favorite", "Rescan"};
    for (int i = 0; i < 4; ++i) {
        const int x = i * 80 + 4;
        const bool active = (i == 1 && favorites) || (i == 2 && favorite);
        const Color label = i == 2 && games.empty() ? Muted : active ? Accent : Text;
        c.rounded({x, 3, 72, 21}, active ? Selected : Surface);
        int width = 0;
        for (auto code : controls[i]) width += lookup(code, false).advance;
        c.text(x + (72 - width) / 2, 7, controls[i], label, width);
    }
    icon(c, 12, 34, Accent);
    c.text(41, 32, "Arduboy Go", Text, 218, true);
    if (battery <= 100) {
        c.rounded({264, 33, 44, 21}, Surface);
        c.text(270, 37, std::to_string(battery) + "%", battery < 15 ? Gold : Muted, 33);
    }
    c.rect({12, 60, 296, 1}, Line);
    c.rounded({12, 66, 42, 17}, favorites ? Surface : Selected);
    c.text(22, 68, "All", favorites ? Muted : Accent);
    c.rounded({59, 66, 65, 17}, favorites ? Selected : Surface);
    c.text(66, 68, "Favorites", favorites ? Accent : Muted);
    if (games.empty()) {
        c.rounded({12, 89, 296, 139}, Surface);
        c.text(30, 113, favorites ? "No favorites yet" : "Your library is empty", Text, 255, true);
        c.text(31, 144, favorites ? "Switch to All and mark a game." : "Copy .hex or .arduboy files to", Muted);
        if (!favorites) c.text(31, 162, "roms/arduboy", Accent);
        return;
    }

    for (size_t i = top; i < games.size() && i < top + VisibleRows; ++i) {
        const auto& game = catalog.games[games[i]];
        const int y = 87 + int(i - top) * 20;
        if (i == selected) {
            c.rounded({12, y, 148, 19}, Selected);
            c.rect({12, y + 3, 2, 13}, Accent);
        }
        c.text(20, y + 3, game.title, i == selected ? Text : Muted, game.favorite ? 119 : 133);
        if (game.favorite) c.text(144, y + 3, "*", Gold, 10);
    }
    if (games.size() > VisibleRows) {
        constexpr int track = 138;
        const int thumb = std::max(8, int(track * VisibleRows / games.size()));
        const int scroll = (track - thumb) * top / (games.size() - VisibleRows);
        c.rect({163, 87, 2, track}, Line);
        c.rect({163, 87 + scroll, 2, thumb}, Accent);
    }

    c.rounded({171, 66, 137, 160}, Surface);
    c.rect({175, 71, 129, 65}, Screen);
    if (preview) c.preview(175, 71, *preview);
    else {
        icon(c, 229, 83, Line);
        c.text(209, 110, "No preview", Muted, 88);
    }
    const auto& game = catalog.games[games[selected]];
    c.text(179, 143, game.title, Text, 122);
    c.text(179, 162, game.packaged ? "ARDUBOY" : "INTEL HEX", Accent, 122);
    if (game.favorite) c.text(179, 181, "* Favorite", Gold, 122);
    std::string count = std::to_string(selected + 1) + " / " + std::to_string(games.size());
    if (catalog.truncated) count += "  (partial)";
    c.text(179, 202, count, Muted, 122);
}

void drawPause(Canvas& c, std::string_view title, unsigned selected, unsigned slot, bool hasSave) {
    header(c, "Paused");
    c.text(132, 18, title, Muted, 174);
    const std::string slotName = std::to_string(slot + 1) + (hasSave ? "   Saved" : "   Empty");
    navRow(c, 0, selected, "Resume");
    navRow(c, 1, selected, "Save slot   <  >", slotName);
    navRow(c, 2, selected, "Save state");
    navRow(c, 3, selected, "Load state", hasSave ? "" : "Empty");
    navRow(c, 4, selected, "Settings");
    navRow(c, 5, selected, "Restart game");
    navRow(c, 6, selected, "Return to library");
    footer(c, "A  Select    B / MENU  Resume");
}

void drawSettings(Canvas& c, const Settings& s, unsigned selected) {
    header(c, "Settings");
    constexpr const char* scales[] = {"1:1  /  128x64", "2x  /  256x128", "Fit  /  320x160"};
    navRow(c, 0, selected, "Brightness", std::to_string(s.brightness) + "%");
    navRow(c, 1, selected, "Volume", s.volume ? std::to_string(s.volume) + "%" : "Muted");
    navRow(c, 2, selected, "Game scale", scales[static_cast<unsigned>(s.scale)]);
    navRow(c, 3, selected, "Emulation speed", std::to_string(s.speed) + "%");
    navRow(c, 4, selected, "Screen updates", std::to_string(s.fps) + " fps");
    navRow(c, 5, selected, "OLED smoothing", s.oledFilter ? "On" : "Off");
    navRow(c, 6, selected, "Game palette", Palettes[s.palette % PaletteCount].name);
    navRow(c, 7, selected, "Show FPS", s.showFps ? "On" : "Off");
    footer(c, "LEFT / RIGHT  Change    B  Back");
}

void drawFps(Canvas& c, unsigned fps) {
    c.rect(FpsCorner, 0);
    c.text(4, 2, std::to_string(fps), rgb(255, 222, 24), FpsCorner.w - 4);
}

void drawNotice(Canvas& c, std::string_view title, std::string_view detail, bool confirm) {
    c.rounded({15, 74, 290, 92}, Line);
    c.rounded({17, 76, 286, 88}, Surface);
    c.text(27, 86, title, Text, 266, true);
    c.text(28, 116, detail, Muted, 262);
    c.text(28, 143, confirm ? "A  Confirm    B  Cancel" : "A / B  Dismiss", Accent, 260);
}

void drawStorageError(Canvas& c) {
    header(c, "Arduboy Go");
    c.rounded({12, 62, 296, 112}, Surface);
    c.text(26, 81, "SD card unavailable", Text, 266, true);
    c.text(27, 113, "Insert a FAT32 microSD card.", Muted);
    c.text(27, 135, "Press A or START to retry.", Accent);
    footer(c, "A  Retry    START  Retry");
}

void drawActivity(Canvas& c, std::string_view title, std::string_view status) {
    header(c, "Arduboy Go");
    c.text(17, 79, title, Text, 285, true);
    c.text(18, 109, status, Accent);
}

}
