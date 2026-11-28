#include "app.hpp"
#include "files.hpp"
#include "audio.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace fs = std::filesystem;

namespace {

void require(bool condition, const std::string& message) {
    if (condition) return;
    std::cerr << message << '\n';
    std::exit(1);
}

struct Display final : go::Platform {
    std::array<uint16_t, go::ScreenW * go::ScreenH> frame{};
    unsigned frames = 0, audioCalls = 0, silences = 0, brightnessLevel = 0;

    void present(const uint16_t* data, go::Rect r) override {
        for (int y = r.y; y < r.y + r.h; ++y)
            std::copy_n(data + y * go::ScreenW + r.x, r.w, frame.data() + y * go::ScreenW + r.x);
        ++frames;
    }

    void brightness(unsigned percent) override { brightnessLevel = percent; }

    void audio(const int16_t*, size_t, unsigned, unsigned) override { ++audioCalls; }

    void silence() override { ++silences; }

    unsigned battery() override { return 79; }
};

void ppm(const fs::path& path, const Display& display) {
    std::ofstream file(path, std::ios::binary);
    file << "P6\n" << go::ScreenW << " " << go::ScreenH << "\n255\n";
    for (auto p : display.frame) {
        file.put(char(((p >> 11) & 31) * 255 / 31));
        file.put(char(((p >> 5) & 63) * 255 / 63));
        file.put(char((p & 31) * 255 / 31));
    }
    require(bool(file), "Screenshot write failed");
}

void run(go::Core& core, unsigned milliseconds, uint16_t keys = 0) {
    for (unsigned i = 0; i < milliseconds; i += 10) core.run(10000, 100, keys, false);
}

size_t litPixels(const go::Core& core) {
    const auto* pixels = core.pixels();
    return std::count_if(pixels, pixels + go::GameW * go::GameH, [](int p) { return p > 48; });
}

void testCore(const fs::path& sd) {
    go::Catalog catalog(sd.string());
    require(catalog.scan(), "Can't open ROM library");
    auto it = std::find_if(catalog.games.begin(), catalog.games.end(),
                           [](const go::Game& game) { return game.title == "Orbit Signal"; });
    require(it != catalog.games.end() && it->packaged, "Missing package fixture");
    go::Preview preview{};
    require(catalog.loadPreview(*it, preview), "Embedded PNG preview failed");
    require(it->title == "Orbit Signal: Flight Test", "Package title was not read");
    require(std::any_of(preview.begin(), preview.end(), [](auto x) { return x; }), "Preview is empty");

    go::Core core;
    auto result = core.load(*it, catalog.stateDir);
    require(result.empty(), result);
    run(core, 690);
    require(core.cycles() > 10000000, "AVR clock didn't advance");
    const auto base = litPixels(core);
    require(base > 160 && base < 1400, "SSD1306 didn't render the expected scene: " + std::to_string(base));
    require(std::any_of(core.samples().begin(), core.samples().end(), [](auto p) { return p != 0; }),
            "No audio from AVR Timer3");

    require(core.save(0).empty() && core.hasSlot(0), "Couldn't save slot 1");
    run(core, 500, go::A);
    require(litPixels(core) > base + 30, "A button didn't reach the AVR");
    require(core.restore(0).empty(), "Couldn't restore slot 1");
    require(litPixels(core) == base, "State didn't restore the OLED frame");

    run(core, 480, go::B);
    run(core, 300);
    require(core.flush(), "Couldn't flush the Arduboy EEPROM");
    require(litPixels(core) == base + 3, "B did not update in-game progress");
    require(core.reset(), "Reset failed");
    run(core, 700);
    require(litPixels(core) == base + 3, "Reset didn't restore EEPROM");
    core.close();
    require(core.load(*it, catalog.stateDir).empty(), "Couldn't re-open game");
    run(core, 700);
    require(litPixels(core) == base + 3, "EEPROM did not persist after reopening");

    require(core.save(0).empty(), "Couldn't refresh state slot");
    for (auto& f : fs::directory_iterator(catalog.stateDir))
        if (f.path().extension() == ".slot1") std::ofstream(f.path(), std::ios::trunc) << "corrupt";
    require(core.restore(0).empty(), "Corrupt primary slot didn't fall back to the backup");
    require(litPixels(core) == base, "Wrong backup state");
    core.close();

    it = std::find_if(catalog.games.begin(), catalog.games.end(),
                      [](const go::Game& game) { return game.title == "Broken"; });
    require(it != catalog.games.end() && !core.load(*it, catalog.stateDir).empty() && !core,
            "Broken ROM should report an error before running");
}

void testLargeCatalog(const fs::path& sd) {
    go::Catalog catalog(sd.string());
    require(catalog.scan() && catalog.games.size() == 5000 && !catalog.truncated,
            "Large nested catalog did not index all 5,000 files");
    require(catalog.toggle(catalog.games.back()), "Couldn't write favorite");
    require(catalog.scan() && catalog.games.back().favorite, "Favorite did not survive rescan");
    require(catalog.toggle(catalog.games.back()), "Couldn't remove favorite");
    require(catalog.scan() && !catalog.games.back().favorite, "Removed favorite reappeared");
}

void testUi(const fs::path& sd, const fs::path& capture) {
    Display display;
    go::App app(display, sd.string());
    app.start();
    uint64_t now = 10000;
    auto tick = [&](uint16_t keys = 0) {
        app.tick(now, keys);
        now += 10000;
    };
    auto tap = [&](uint16_t key) { tick(key); tick(); };
    for (int i = 0; i < 20; ++i) tick();
    require(display.frames >= 2 && display.brightnessLevel == 80, "Library did not start");
    tap(go::Select);
    if (!capture.empty()) ppm(capture / "library.ppm", display);
    tap(go::Volume);
    if (!capture.empty()) {
        for (int i = 0; i < 15; ++i) tick();
        ppm(capture / "favorites.ppm", display);
    }
    tap(go::Volume);
    tap(go::A);
    for (int i = 0; i < 100; ++i) tick();
    require(display.audioCalls > 50, "Library launch did not start gameplay");
    require(display.frame[0] == 0 && display.frame[120 * go::ScreenW] == 0, "2x viewport isn't centered");
    if (!capture.empty()) ppm(capture / "game.ppm", display);

    tap(go::Menu);
    require(display.silences == 1, "Pause didn't mute audio");
    unsigned audioCalls = display.audioCalls;
    for (int i = 0; i < 50; ++i) tick();
    require(display.audioCalls == audioCalls, "Pause ran the emulator");
    if (!capture.empty()) ppm(capture / "pause.ppm", display);
    tap(go::Down);
    tap(go::Right);
    tap(go::Down);
    tap(go::A);
    tap(go::A); // dismiss save notice
    tap(go::Down);
    tap(go::Down);
    tap(go::A);
    if (!capture.empty()) ppm(capture / "settings.ppm", display);
    tap(go::Right);
    require(display.brightnessLevel == 90, "Backlight control didn't update");
    tap(go::Down);
    tap(go::Down);
    tap(go::Right); // 2x -> Fit
    for (int i = 0; i < 5; ++i) tap(go::Down);
    tap(go::Right); // FPS counter
    if (!capture.empty()) ppm(capture / "settings.ppm", display);
    tap(go::B);
    tap(go::B);
    require(display.audioCalls > audioCalls, "Resume didn't restart emulation");
    const auto settings = go::readSettings((sd / "saves/arduboy/settings.dat").string());
    require(settings.scale == go::Scale::Fit && settings.showFps, "Video settings did not persist");
    for (int i = 0; i < 125; ++i) tick(i < 50 ? go::A : 0);
    bool yellow = false;
    for (int y = 0; y < go::FpsCorner.h; ++y) {
        const auto row = display.frame.data() + y * go::ScreenW;
        yellow |= std::find(row, row + go::FpsCorner.w, go::rgb(255, 222, 24)) != row + go::FpsCorner.w;
    }
    require(yellow, "FPS counter not rendered in the screen corner");
    if (!capture.empty()) ppm(capture / "fps.ppm", display);
    require(app.flushProgress(), "App progress flush failed");
}

void testViewport() {
    go::Canvas canvas;
    std::array<uint8_t, go::GameW * go::GameH> pixels{};
    pixels.fill(255);
    for (const auto scale : {go::Scale::Native, go::Scale::Double, go::Scale::Fit}) {
        const auto box = go::viewport(scale);
        canvas.fill(0);
        canvas.frame(pixels.data(), scale, 0);
        const auto lit = std::count_if(canvas.data(), canvas.data() + go::ScreenW * go::ScreenH, [](auto x) { return x; });
        require(lit == box.w * box.h && box.w == 2 * box.h &&
                box.x * 2 + box.w == go::ScreenW && box.y * 2 + box.h == go::ScreenH, "Incorrect viewport geometry");
    }
    for (unsigned p = 0; p < go::PaletteCount; ++p) {
        const go::GamePixels gp(go::GameW, p, false);
        require(gp.colors[0] != gp.colors[255], "Palette background and foreground collided");
        if (p >= 6 && p <= 9) {
            require(gp.colors[85] != gp.colors[0] && gp.colors[170] != gp.colors[255] &&
                    gp.colors[85] != gp.colors[170], "GBC halftone stops must be distinct");
        }
    }
}

void extraRoms(const fs::path& folder) {
    go::Catalog catalog(folder.string());
    require(catalog.scan() && !catalog.games.empty(), "Missing compatibility ROMs");
    for (const auto& item : catalog.games) {
        go::Core core;
        const auto before = std::chrono::steady_clock::now();
        const auto error = core.load(item, catalog.stateDir);
        require(error.empty(), item.title + ": " + error);
        run(core, 2400);
        require(litPixels(core) > 20, item.title + " produced no video");
        const auto time = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - before);
        std::cout << item.title << " displayed " << litPixels(core) << " pixels in " << time.count() << " ms on host\n";
    }
}

}

void testSettingsMigration(const fs::path& dir) {
    fs::create_directories(dir);
    const fs::path path = dir / "settings.dat";
    std::array<uint8_t, 12> ags1{'A', 'G', 'S', '1', 80, 75, 1, 100, 60, 0, 0x02, 0};
    for (size_t i = 4; i < 11; ++i) ags1[11] ^= ags1[i];
    {
        std::ofstream out(path, std::ios::binary);
        out.write(reinterpret_cast<const char*>(ags1.data()), ags1.size());
    }
    const auto migrated = go::readSettings(path.string());
    require(migrated.oledFilter && migrated.showFps && migrated.fps == 60,
        "Legacy AGS1 settings should enable oledFilter");

    go::Settings explicitOff = migrated;
    explicitOff.oledFilter = false;
    require(go::writeSettings(path.string(), explicitOff), "writeSettings AGS2 failed");
    const auto roundtrip = go::readSettings(path.string());
    require(!roundtrip.oledFilter && roundtrip.showFps && roundtrip.fps == 60,
        "AGS2 should preserve explicit oledFilter=false");
    for (uint8_t rate : {uint8_t(40), uint8_t(45)}) {
        go::Settings fpsTest = roundtrip;
        fpsTest.fps = rate;
        fpsTest.palette = go::PaletteCount - 1;
        require(go::writeSettings(path.string(), fpsTest), "writeSettings 40/45 fps failed");
        const auto reloaded = go::readSettings(path.string());
        require(reloaded.fps == rate && reloaded.palette == fpsTest.palette,
            "40/45 fps or extended palette did not roundtrip");
    }
}

void testDacConversion() {
    go::DacConverter dac;
    int64_t quietEnergy = 0, loudEnergy = 0;
    for (unsigned volume : {5u, 50u, 100u}) {
        dac.reset();
        int64_t energy = 0;
        for (unsigned i = 0; i < 32000; ++i) {
            // Two mixed square-wave channels, with a maximum magnitude of 2000.
            const int sample = (i % 32 < 16 ? 1000 : -1000) +
                               (i % 50 < 25 ? 1000 : -1000);
            const auto value = dac.convert(sample, volume);
            require(value > 0 && value < 255, "Mixed sound saturated the DAC");
            const int level = int(value) - 128;
            energy += int64_t(level) * level;
        }
        if (volume == 5) quietEnergy = energy;
        if (volume == 50) loudEnergy = energy;
        for (unsigned i = 0; i < 16000; ++i) {
            const auto value = dac.convert(0, volume);
            if (i >= 12000) require(value == 128, "DAC error feedback generated noise during silence");
        }
    }
    require(quietEnergy > 0 && quietEnergy * 100 < loudEnergy,
            "Minimum volume lost its quiet dynamic range");
    dac.reset();
    for (unsigned i = 0; i < 16000; ++i) {
        const auto value = dac.convert(2000, 100);
        if (i >= 12000) require(value == 128, "DAC retained DC noise");
    }
    require(dac.convert(2000, 0) == 128, "Zero volume should produce the DAC midpoint");
}

int main(int argc, char** argv) {
    if (argc == 3 && std::string_view(argv[1]) == "--roms") {
        extraRoms(argv[2]);
        return 0;
    }
    if (argc != 3) return 2;
    testDacConversion();
    testCore(fs::path(argv[1]) / "core");
    testLargeCatalog(fs::path(argv[1]) / "large");
    testSettingsMigration(fs::path(argv[1]) / "settings-migration");
    if (std::string_view(argv[2]) != "-") fs::create_directories(argv[2]);
    testUi(fs::path(argv[1]) / "ui", std::string_view(argv[2]) == "-" ? fs::path{} : fs::path(argv[2]));
    testViewport();
    std::cout << "ROM formats, AVR, OLED, audio, input, saves, 5000 games, viewport and UI passed\n";
}
