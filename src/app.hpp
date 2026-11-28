#pragma once

#include "core.hpp"
#include "ui.hpp"

namespace go {

class App {
public:
    App(Platform& platform, std::string storageRoot);
    void start();
    void tick(uint64_t nowUs, uint16_t buttons);
    bool flushProgress() { return core_.flush(); }

private:
    enum class Screen { Storage, Library, Playing, Paused, Settings };
    enum class Confirm { None, Restart, Leave, Exit };
    static constexpr Rect FullScreen{0, 0, ScreenW, ScreenH};

    void draw();
    void drawGameUpdates(uint64_t nowUs);
    void rebuild(uint64_t keep = 0);
    void mount();
    bool scan(uint64_t keep = 0);
    void changeSelection(int delta);
    void openGame();
    void changeSetting(int delta);
    void closeSettings();
    void finishConfirm();
    void leaveGame();
    void show(Screen screen);
    void notice(std::string title, std::string detail);
    Game* game();

    Platform& platform_;
    Canvas canvas_;
    Catalog catalog_;
    Core core_;
    Settings settings_;
    Settings settingsBefore_;
    std::vector<size_t> visible_;
    Preview preview_{};
    std::array<uint8_t, GameW * GameH> lastFrame_{};
    std::string noticeTitle_, noticeDetail_;
    Screen screen_ = Screen::Storage;
    Screen settingsFrom_ = Screen::Library;
    Confirm confirm_ = Confirm::None;
    size_t selected_ = 0, top_ = 0;
    unsigned pauseRow_ = 0, settingsRow_ = 0, slot_ = 0;
    bool favorites_ = false;
    bool previewReady_ = false, previewPending_ = false, dirty_ = true;
    bool batteryRead_ = false;
    uint16_t previousKeys_ = 0, repeatKeys_ = 0;
    uint64_t now_ = 0, lastTick_ = 0, nextRepeat_ = 0, nextPreview_ = 0;
    uint64_t nextBattery_ = 0, nextFrame_ = 0, nextFlush_ = 0;
    uint64_t fpsSince_ = 0;
    uint64_t fpsStartCycles_ = 0;
    unsigned submittedFrames_ = 0, measuredFps_ = 0;
    bool fpsDirty_ = false;
    unsigned battery_ = 101;
};

}
