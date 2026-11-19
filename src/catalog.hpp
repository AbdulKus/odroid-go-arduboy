#pragma once

#include "types.hpp"

#include <array>
#include <string>
#include <vector>

namespace go {

constexpr size_t MaxGames = 12000;
constexpr size_t MaxHexBytes = 192 * 1024;
constexpr size_t MaxFxBytes = 768 * 1024;
constexpr size_t PreviewW = 128;
constexpr size_t PreviewH = 64;
using Preview = std::array<uint16_t, PreviewW * PreviewH>;

struct Game {
    std::string path;
    std::string title;
    uint64_t id;
    bool packaged;
    bool favorite = false;
    bool metadataRead = false;
};

struct Rom {
    std::vector<uint8_t> hex;
    std::vector<uint8_t> fxData;
    std::vector<uint8_t> fxSave;
    std::string title;
    std::string device;
};

class Catalog {
public:
    explicit Catalog(std::string root);

    bool scan();
    bool toggle(Game& game);
    bool loadPreview(Game& game, Preview& pixels) const;
    bool capture(const Game& game, const uint8_t* pixels) const;

    std::vector<Game> games;
    bool truncated = false;
    const std::string root;
    const std::string romDir;
    const std::string stateDir;

private:
    void scanDirectory(const std::string& dir, unsigned depth);
    void readFavorites();
    std::vector<uint64_t> favorites_;
    size_t pathBytes_ = 0;
};

std::string readRom(const Game& game, Rom& rom);
Settings readSettings(const std::string& path);
bool writeSettings(const std::string& path, const Settings& settings);

}
