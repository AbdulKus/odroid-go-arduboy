#include "app.hpp"

#include <algorithm>
#include <cstring>
#include <exception>
#include <utility>

namespace go {
namespace {

constexpr uint16_t Directions = Up | Down | Left | Right;
constexpr unsigned Slots = 3;

unsigned wrap(int value, int count) {
    return (value + count) % count;
}

std::string settingsPath(const Catalog& catalog) { return catalog.stateDir + "/settings.dat"; }

}

App::App(Platform& platform, std::string storageRoot)
    : platform_(platform), catalog_(std::move(storageRoot)) {}

void App::start() {
    platform_.brightness(settings_.brightness);
    mount();
    draw();
}

void App::mount() {
    if (!platform_.mountStorage() || !scan()) {
        show(Screen::Storage);
        return;
    }
    settings_ = readSettings(settingsPath(catalog_));
    platform_.brightness(settings_.brightness);
    show(Screen::Library);
}

bool App::scan(uint64_t keep) {
    drawActivity(canvas_, "Library", "Scanning games...");
    platform_.present(canvas_.data(), FullScreen);
    if (!catalog_.scan()) return false;
    rebuild(keep);
    return true;
}

Game* App::game() {
    return visible_.empty() ? nullptr : &catalog_.games[visible_[selected_]];
}

void App::rebuild(uint64_t keep) {
    visible_.clear();
    visible_.reserve(catalog_.games.size());
    selected_ = top_ = 0;
    for (size_t i = 0; i < catalog_.games.size(); ++i) {
        const auto& item = catalog_.games[i];
        if (favorites_ && !item.favorite) continue;
        if (item.id == keep) selected_ = visible_.size();
        visible_.push_back(i);
    }
    changeSelection(0);
}

void App::changeSelection(int delta) {
    if (!visible_.empty()) {
        const int count = static_cast<int>(visible_.size());
        const int position = static_cast<int>(selected_) + delta;
        if (delta == 1 && position == count) selected_ = 0;
        else if (delta == -1 && position < 0) selected_ = count - 1;
        else selected_ = std::clamp(position, 0, count - 1);
        if (selected_ < top_) top_ = selected_;
        if (selected_ >= top_ + VisibleRows) top_ = selected_ - VisibleRows + 1;
        previewPending_ = true;
        nextPreview_ = now_ + 140000;
    }
    previewReady_ = false;
    dirty_ = true;
}

void App::openGame() {
    auto* item = game();
    if (!item) return;
    drawActivity(canvas_, item->title, "Loading game...");
    platform_.present(canvas_.data(), FullScreen);
    std::string error;
    try {
        error = core_.load(*item, catalog_.stateDir);
    } catch (const std::bad_alloc&) {
        error = "Insufficient emulator memory";
    }
    if (!error.empty()) {
        core_.close();
        notice("Unable to start", std::move(error));
        return;
    }
    item->title = core_.title();
    item->metadataRead = true;
    pauseRow_ = slot_ = 0;
    nextFlush_ = now_ + 3000000;
    show(Screen::Playing);
}

void App::show(Screen screen) {
    if (screen == Screen::Playing) nextFrame_ = 0;
    if (screen_ == Screen::Playing && screen != Screen::Playing) platform_.silence();
    screen_ = screen;
    dirty_ = true;
}

void App::notice(std::string title, std::string detail) {
    noticeTitle_ = std::move(title);
    noticeDetail_ = std::move(detail);
    dirty_ = true;
}

void App::changeSetting(int delta) {
    const auto stepped = [delta](int current, int min, int max) {
        return static_cast<uint8_t>(std::clamp(current + 10 * delta, min, max));
    };
    switch (settingsRow_) {
    case 0:
        settings_.brightness = stepped(settings_.brightness, 10, 100);
        platform_.brightness(settings_.brightness);
        break;
    case 1: settings_.volume = stepped(settings_.volume, 0, 100); break;
    case 2: settings_.scale = static_cast<Scale>(wrap(int(settings_.scale) + delta, 3)); break;
    case 3: settings_.speed = std::clamp(int(settings_.speed) + delta * 25, 50, 125); break;
    case 4: {
        const unsigned index = settings_.fps == 20 ? 0 : settings_.fps == 30 ? 1 : 2;
        constexpr uint8_t rates[] = {20, 30, 60};
        settings_.fps = rates[wrap(int(index) + delta, 3)];
        break;
    }
    case 5: settings_.oledFilter = !settings_.oledFilter; break;
    case 6: settings_.palette = wrap(int(settings_.palette) + delta, 3); break;
    }
    dirty_ = true;
}

void App::closeSettings() {
    show(settingsFrom_);
    const auto& old = settingsBefore_;
    if (old.brightness == settings_.brightness && old.volume == settings_.volume &&
        old.scale == settings_.scale && old.speed == settings_.speed &&
        old.fps == settings_.fps && old.palette == settings_.palette &&
        old.oledFilter == settings_.oledFilter) return;
    if (!writeSettings(settingsPath(catalog_), settings_))
        notice("Settings not saved", "Check available space on the SD card.");
}

void App::leaveGame() {
    if (!core_.flush()) {
        notice("Progress not saved", "Check the SD card and try again.");
        return;
    }
    if (const auto* item = game()) catalog_.capture(*item, core_.pixels());
    core_.close();
    changeSelection(0);
    show(Screen::Library);
}

void App::finishConfirm() {
    auto choice = std::exchange(confirm_, Confirm::None);
    dirty_ = true;
    if (choice == Confirm::Restart) {
        if (core_.reset()) show(Screen::Playing);
        else notice("Progress not saved", "Check the SD card and try again.");
    } else if (choice == Confirm::Leave) leaveGame();
    else if (choice == Confirm::Exit) platform_.exit();
}

void App::tick(uint64_t nowUs, uint16_t buttons) {
    now_ = nowUs;
    unsigned elapsed = lastTick_ ? std::min<uint64_t>(nowUs - lastTick_, 40000) : 0;
    lastTick_ = nowUs;
    uint16_t pressed = buttons & ~previousKeys_;
    previousKeys_ = buttons;
    const uint16_t direction = buttons & Directions;
    if (direction != repeatKeys_) {
        repeatKeys_ = direction;
        nextRepeat_ = nowUs + 310000;
    } else if (direction && nowUs >= nextRepeat_) {
        pressed |= direction;
        nextRepeat_ = nowUs + 85000;
    }

    if (!noticeTitle_.empty()) {
        if (pressed & (A | B | Menu)) {
            noticeTitle_.clear();
            noticeDetail_.clear();
            dirty_ = true;
        }
        if (dirty_) draw();
        return;
    }
    if (confirm_ != Confirm::None) {
        if (pressed & (B | Menu)) { confirm_ = Confirm::None; dirty_ = true; }
        else if (pressed & A) finishConfirm();
        if (dirty_) draw();
        return;
    }

    switch (screen_) {
    case Screen::Storage:
        if (pressed & (A | Start)) mount();
        break;
    case Screen::Library: {
        if (!batteryRead_ || nowUs >= nextBattery_) {
            battery_ = platform_.battery();
            batteryRead_ = true;
            nextBattery_ = nowUs + 12000000;
            dirty_ = true;
        }
        if (pressed & Menu) {
            settingsFrom_ = Screen::Library;
            settingsBefore_ = settings_;
            settingsRow_ = 0;
            show(Screen::Settings);
        } else if (pressed & Start) {
            auto keep = game() ? game()->id : 0;
            if (!scan(keep)) show(Screen::Storage);
        } else if ((pressed & B) && platform_.canExit()) {
            confirm_ = Confirm::Exit;
            dirty_ = true;
        } else if (pressed & Volume) {
            auto keep = game() ? game()->id : 0;
            favorites_ = !favorites_;
            rebuild(keep);
        } else if ((pressed & Select) && game()) {
            auto* item = game();
            if (!catalog_.toggle(*item)) notice("Favorite not saved", "Check the SD card.");
            else if (favorites_) rebuild(item->id);
            dirty_ = true;
        } else if (pressed & A) openGame();
        else if (pressed & Up) changeSelection(-1);
        else if (pressed & Down) changeSelection(1);
        else if (pressed & Left) changeSelection(-VisibleRows);
        else if (pressed & Right) changeSelection(VisibleRows);

        if (screen_ == Screen::Library && previewPending_ && nowUs >= nextPreview_) {
            previewPending_ = false;
            if (auto* item = game()) {
                try {
                    previewReady_ = catalog_.loadPreview(*item, preview_);
                } catch (const std::bad_alloc&) {
                    previewReady_ = false;
                }
            }
            dirty_ = true;
        }
        break;
    }
    case Screen::Playing:
        if (pressed & (Menu | Start | Volume)) {
            pauseRow_ = (pressed & Volume) ? 4 : 0;
            show(Screen::Paused);
            if (!core_.flush()) notice("Progress not saved", "Check the SD card.");
            if (pressed & Volume) {
                settingsFrom_ = Screen::Paused;
                settingsBefore_ = settings_;
                settingsRow_ = 1;
                show(Screen::Settings);
            }
            break;
        }
        if (elapsed) {
            core_.run(elapsed, settings_.speed, buttons, settings_.oledFilter);
            const auto& samples = core_.samples();
            platform_.audio(samples.data(), samples.size(), settings_.volume, settings_.speed);
        }
        if (nowUs >= nextFlush_) {
            core_.flush();
            nextFlush_ = nowUs + 3000000;
        }
        if (!dirty_ && nowUs >= nextFrame_) {
            nextFrame_ += 1000000 / settings_.fps;
            if (nextFrame_ <= nowUs) nextFrame_ = nowUs + 1000000 / settings_.fps;
            const auto* pixels = core_.pixels();
            if (std::memcmp(pixels, lastFrame_.data(), lastFrame_.size()) != 0) {
                canvas_.frame(pixels, settings_.scale, settings_.palette);
                platform_.present(canvas_.data(), viewport(settings_.scale));
                std::memcpy(lastFrame_.data(), pixels, lastFrame_.size());
            }
        }
        break;
    case Screen::Paused:
        if (pressed & (Menu | B | Start)) show(Screen::Playing);
        else if (pressed & (Up | Down)) {
            pauseRow_ = wrap(int(pauseRow_) + (pressed & Down ? 1 : -1), PauseRows);
            dirty_ = true;
        } else if (pauseRow_ == 1 && pressed & (Left | Right)) {
            slot_ = wrap(int(slot_) + (pressed & Right ? 1 : -1), Slots);
            dirty_ = true;
        } else if (pressed & A) {
            std::string error;
            switch (pauseRow_) {
            case 0: show(Screen::Playing); break;
            case 1: slot_ = (slot_ + 1) % Slots; dirty_ = true; break;
            case 2:
                error = core_.save(slot_);
                if (error.empty()) notice("State saved", "Slot " + std::to_string(slot_ + 1));
                break;
            case 3:
                error = core_.restore(slot_);
                if (error.empty()) show(Screen::Playing);
                break;
            case 4:
                settingsFrom_ = Screen::Paused;
                settingsBefore_ = settings_;
                settingsRow_ = 0;
                show(Screen::Settings);
                break;
            case 5: confirm_ = Confirm::Restart; dirty_ = true; break;
            case 6: confirm_ = Confirm::Leave; dirty_ = true; break;
            }
            if (!error.empty()) notice("Save state", std::move(error));
        }
        break;
    case Screen::Settings:
        if (pressed & (B | Menu)) closeSettings();
        else if (pressed & (Up | Down)) {
            settingsRow_ = wrap(int(settingsRow_) + (pressed & Down ? 1 : -1), SettingsRows);
            dirty_ = true;
        } else if (pressed & (Left | Right | A)) changeSetting(pressed & Left ? -1 : 1);
        break;
    }
    if (dirty_) draw();
}

void App::draw() {
    switch (screen_) {
    case Screen::Storage: drawStorageError(canvas_); break;
    case Screen::Library:
        drawLibrary(canvas_, catalog_, visible_, selected_, top_, favorites_,
                    previewReady_ ? &preview_ : nullptr, battery_);
        break;
    case Screen::Playing:
        canvas_.fill(0);
        canvas_.frame(core_.pixels(), settings_.scale, settings_.palette);
        std::memcpy(lastFrame_.data(), core_.pixels(), lastFrame_.size());
        nextFrame_ = now_ + 1000000 / settings_.fps;
        break;
    case Screen::Paused: drawPause(canvas_, core_.title(), pauseRow_, slot_, core_.hasSlot(slot_)); break;
    case Screen::Settings: drawSettings(canvas_, settings_, settingsRow_); break;
    }
    if (!noticeTitle_.empty()) drawNotice(canvas_, noticeTitle_, noticeDetail_);
    else if (confirm_ != Confirm::None) {
        const char* text = confirm_ == Confirm::Restart ? "Restart from the beginning?" :
            confirm_ == Confirm::Leave ? "Return to the game library?" : "Return to the ODROID-GO launcher?";
        drawNotice(canvas_, "Confirm", text, true);
    }
    platform_.present(canvas_.data(), FullScreen);
    dirty_ = false;
}

}
