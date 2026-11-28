#include "catalog.hpp"
#include "files.hpp"
#include "ui.hpp"
#include "miniz.h"
#include "lodepng.h"
#include "yyjson/yyjson.h"

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <sys/stat.h>
#include <utility>

namespace go {
namespace {

constexpr size_t MaxArchiveBytes = 4 * 1024 * 1024;
constexpr size_t MaxImageBytes = 192 * 1024;
constexpr size_t MaxInfoBytes = 24 * 1024;
constexpr unsigned MaxDepth = 8;
constexpr size_t MaxCatalogPathBytes = 768 * 1024;

uint16_t rgb565(unsigned r, unsigned g, unsigned b) {
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
}

bool readVector(const std::string& path, std::vector<uint8_t>& data, size_t limit) {
    auto file = openFile(path, "rb");
    if (!file) return false;
    auto n = fileSize(file.get());
    if (n < 0 || static_cast<size_t>(n) > limit) return false;
    data.resize(n);
    return readBytes(file.get(), data.data(), data.size());
}

struct Package {
    mz_zip_archive zip{};
    bool ready = false;
    std::string title, hex, image, fxData, fxSave, device;

    explicit Package(const std::string& path) {
        auto file = openFile(path, "rb");
        if (!file) return;
        long n = fileSize(file.get());
        if (n < 0 || static_cast<size_t>(n) > MaxArchiveBytes) return;
        file.reset();
        ready = mz_zip_reader_init_file(&zip, path.c_str(), MZ_ZIP_FLAG_DO_NOT_SORT_CENTRAL_DIRECTORY);
        if (!ready) return;

        std::vector<uint8_t> info;
        if (extract("info.json", info, MaxInfoBytes)) {
            const char* text = reinterpret_cast<const char*>(info.data());
            size_t size = info.size();
            if (size >= 3 && std::memcmp(text, "\xef\xbb\xbf", 3) == 0) {
                text += 3;
                size -= 3;
            }
            std::unique_ptr<yyjson_doc, decltype(&yyjson_doc_free)> doc(
                yyjson_read(text, size, YYJSON_READ_ALLOW_COMMENTS | YYJSON_READ_ALLOW_TRAILING_COMMAS |
                                      YYJSON_READ_STOP_WHEN_DONE), yyjson_doc_free);
            auto* root = yyjson_doc_get_root(doc.get());
            const auto field = [](yyjson_val* obj, const char* key) -> std::string {
                auto* value = yyjson_obj_get(obj, key);
                const auto* p = yyjson_get_str(value);
                return p && yyjson_get_len(value) <= 255 ? p : "";
            };
            title = field(root, "title");
            auto* bin = yyjson_arr_get_first(yyjson_obj_get(root, "binaries"));
            hex = field(bin, "filename");
            fxData = field(bin, "flashdata");
            fxSave = field(bin, "flashsave");
            device = field(bin, "device");
            image = field(root, "banner");
            if (image.empty()) image = field(root, "cover");
            if (image.empty()) {
                auto* item = yyjson_arr_get_first(yyjson_obj_get(root, "screenshots"));
                if (const auto* p = yyjson_get_str(item); p && yyjson_get_len(item) <= 255) image = p;
            }
        }

        std::string fallbackImage;
        for (mz_uint i = 0; i < mz_zip_reader_get_num_files(&zip); ++i) {
            mz_zip_archive_file_stat entry{};
            if (!mz_zip_reader_file_stat(&zip, i, &entry) || entry.m_is_directory) continue;
            std::string name(entry.m_filename);
            auto ext = extension(name);
            if (hex.empty() && ext == ".hex") hex = name;
            if (image.empty() && ext == ".png") {
                if (stem(name) == "banner" || stem(name) == "cover") image = name;
                else if (fallbackImage.empty()) fallbackImage = name;
            }
            if (!hex.empty() && !image.empty()) break;
        }
        if (image.empty()) image = std::move(fallbackImage);
    }

    ~Package() { if (ready) mz_zip_reader_end(&zip); }

    bool extract(const std::string& name, std::vector<uint8_t>& data, size_t limit) {
        if (!ready || name.empty()) return false;
        int index = mz_zip_reader_locate_file(&zip, name.c_str(), nullptr, 0);
        mz_zip_archive_file_stat entry{};
        if (index < 0 || !mz_zip_reader_file_stat(&zip, index, &entry) ||
            entry.m_is_directory || entry.m_uncomp_size > limit || entry.m_uncomp_size == 0) return false;
        data.resize(static_cast<size_t>(entry.m_uncomp_size));
        return mz_zip_reader_extract_to_mem(&zip, index, data.data(), data.size(), 0);
    }
};

bool decodePreview(const std::vector<uint8_t>& data, Preview& pixels) {
    if (data.size() < 33) return false;
    LodePNGState state;
    lodepng_state_init(&state);
    unsigned w = 0, h = 0;
    unsigned error = lodepng_inspect(&w, &h, &state, data.data(), data.size());
    lodepng_state_cleanup(&state);
    if (error || w == 0 || h == 0 || w > 512 || h > 512) return false;
    unsigned char* raw = nullptr;
    error = lodepng_decode32(&raw, &w, &h, data.data(), data.size());
    std::unique_ptr<unsigned char, decltype(&std::free)> image(raw, std::free);
    if (error || !raw) return false;
    int dw = PreviewW;
    int dh = PreviewH;
    if (w * PreviewH >= h * PreviewW) dh = std::max<size_t>(1, h * PreviewW / w);
    else dw = std::max<size_t>(1, w * PreviewH / h);
    int left = (PreviewW - dw) / 2;
    int top = (PreviewH - dh) / 2;
    pixels.fill(0);
    for (int y = 0; y < dh; ++y) {
        for (int x = 0; x < dw; ++x) {
            const auto* p = raw + 4 * ((y * h / dh) * w + x * w / dw);
            pixels[(y + top) * PreviewW + x + left] =
                rgb565(p[0] * p[3] / 255, p[1] * p[3] / 255, p[2] * p[3] / 255);
        }
    }
    return true;
}

int compareNames(std::string_view a, std::string_view b) {
    const auto lower = [](unsigned char c) { return c >= 'A' && c <= 'Z' ? c + 'a' - 'A' : c; };
    const size_t n = std::min(a.size(), b.size());
    for (size_t i = 0; i < n; ++i) {
        if (int diff = lower(a[i]) - lower(b[i])) return diff;
    }
    return a.size() < b.size() ? -1 : a.size() != b.size();
}

bool readFavoriteFile(File file, std::vector<uint64_t>& favorites) {
    if (!file) return false;
    long size = fileSize(file.get());
    if (size < 8 || (size - 8) % 8 || size > long(8 + MaxGames * 8)) return false;
    std::array<uint8_t, 8> bytes{};
    if (!readBytes(file.get(), bytes.data(), bytes.size()) || std::memcmp(bytes.data(), "AGF1", 4)) return false;
    const uint32_t crc = readLe(bytes.data() + 4, 4);
    uint32_t actual = 0;
    std::vector<uint64_t> values;
    values.reserve((size - 8) / 8);
    for (long offset = 8; offset < size; offset += 8) {
        if (!readBytes(file.get(), bytes.data(), bytes.size())) return false;
        actual = mz_crc32(actual, bytes.data(), bytes.size());
        values.push_back(readLe(bytes.data(), 8));
    }
    if (actual != crc || !std::is_sorted(values.begin(), values.end())) return false;
    favorites = std::move(values);
    return true;
}

bool readSettingsFile(File file, Settings& result) {
    std::array<uint8_t, 12> data{};
    const bool okHeader = file && readBytes(file.get(), data.data(), data.size()) &&
        (std::memcmp(data.data(), "AGS1", 4) == 0 || std::memcmp(data.data(), "AGS2", 4) == 0);
    if (!okHeader) return false;
    const bool validFps = data[8] == 20 || data[8] == 30 || data[8] == 40 || data[8] == 45 || data[8] == 60;
    if (data[4] < 10 || data[4] > 100 || data[5] > 100 || data[6] > 2 ||
        (data[7] != 50 && data[7] != 75 && data[7] != 100 && data[7] != 125) ||
        !validFps || data[9] >= PaletteCount || data[10] > 3) return false;
    uint8_t checksum = 0;
    for (size_t i = 4; i < 11; ++i) checksum ^= data[i];
    if (checksum != data[11]) return false;
    // Migrate legacy AGS1 settings (saved when oledFilter defaulted to false) to enable OLED smoothing.
    const bool oledFilter = (data[3] == '1') ? true : bool(data[10] & 1);
    result = {data[4], data[5], static_cast<Scale>(data[6]), data[7],
              data[8], data[9], oledFilter, bool(data[10] & 2)};
    return true;
}

}

Catalog::Catalog(std::string base)
    : root(std::move(base)), romDir(root + "/roms/arduboy"), stateDir(root + "/saves/arduboy") {}

bool Catalog::scan() {
    games.clear();
    pathBytes_ = 0;
    truncated = false;
    makeDirectory(root + "/roms");
    makeDirectory(romDir);
    makeDirectory(root + "/saves");
    makeDirectory(stateDir);
    readFavorites();
    DIR* directory = opendir(romDir.c_str());
    if (!directory) return false;
    closedir(directory);
    scanDirectory(romDir, 0);
    std::sort(games.begin(), games.end(), [](const Game& a, const Game& b) {
        const int compare = compareNames(a.title, b.title);
        return compare != 0 ? compare < 0 : a.path < b.path;
    });
    return true;
}

void Catalog::scanDirectory(const std::string& dir, unsigned depth) {
    DIR* handle = opendir(dir.c_str());
    if (!handle) return;
    while (auto* item = readdir(handle)) {
        const char* name = item->d_name;
        if (name[0] == '.') continue;
        std::string path = dir + "/" + name;
        if (path.size() > 384) { truncated = true; continue; }
        bool isDirectory = item->d_type == DT_DIR;
        if (item->d_type == DT_UNKNOWN) {
            struct stat info{};
            if (stat(path.c_str(), &info) != 0) continue;
            isDirectory = S_ISDIR(info.st_mode);
        } else if (!isDirectory && item->d_type != DT_REG) {
            continue;
        }
        if (isDirectory) {
            if (depth < MaxDepth) scanDirectory(path, depth + 1);
            else truncated = true;
            continue;
        }

        const auto ext = extension(path);
        if (ext != ".arduboy" && ext != ".hex") continue;
        if (games.size() == MaxGames) { truncated = true; break; }
        auto id = pathId(std::string_view(path).substr(romDir.size() + 1));
        auto title = stem(path);
        const size_t bytes = path.size() + title.size() + 2;
        if (pathBytes_ + bytes > MaxCatalogPathBytes) { truncated = true; continue; }
        pathBytes_ += bytes;
        games.push_back({std::move(path), std::move(title), id, ext == ".arduboy",
                         std::binary_search(favorites_.begin(), favorites_.end(), id)});
    }
    closedir(handle);
}

void Catalog::readFavorites() {
    favorites_.clear();
    const auto path = stateDir + "/favorites.dat";
    if (!readFavoriteFile(openFile(path, "rb"), favorites_))
        readFavoriteFile(openBackup(path), favorites_);
}

bool Catalog::toggle(Game& game) {
    auto favorite = std::lower_bound(favorites_.begin(), favorites_.end(), game.id);
    if (game.favorite) {
        if (favorite == favorites_.end() || *favorite != game.id) return false;
        favorites_.erase(favorite);
    } else {
        if (favorites_.size() >= MaxGames) return false;
        favorites_.insert(favorite, game.id);
    }
    bool stored = replaceFile(stateDir + "/favorites.dat", [&](FILE* file) {
        std::array<uint8_t, 8> header{'A', 'G', 'F', '1'};
        uint32_t crc = 0;
        std::array<uint8_t, 8> bytes{};
        for (auto id : favorites_) {
            writeLe(bytes.data(), id, 8);
            crc = mz_crc32(crc, bytes.data(), bytes.size());
        }
        writeLe(header.data() + 4, crc, 4);
        if (!writeBytes(file, header.data(), header.size())) return false;
        for (auto id : favorites_) {
            writeLe(bytes.data(), id, 8);
            if (!writeBytes(file, bytes.data(), bytes.size())) return false;
        }
        return true;
    });
    if (stored) game.favorite = !game.favorite;
    else readFavorites();
    return stored;
}

bool Catalog::loadPreview(Game& game, Preview& pixels) const {
    std::vector<uint8_t> data;
    const auto base = game.path.substr(0, game.path.find_last_of('.'));
    bool preview = readVector(base + ".png", data, MaxImageBytes) && decodePreview(data, pixels);
    if (game.packaged && (!game.metadataRead || !preview)) {
        Package package(game.path);
        if (!package.title.empty()) game.title = package.title;
        game.metadataRead = true;
        if (!preview && package.extract(package.image, data, MaxImageBytes))
            preview = decodePreview(data, pixels);
    }
    if (preview) return true;
    if (!readVector(stateDir + "/" + hexId(game.id) + ".preview", data, PreviewW * PreviewH) ||
        data.size() != pixels.size()) return false;
    for (size_t i = 0; i < data.size(); ++i)
        pixels[i] = rgb565(data[i], data[i], data[i]);
    return true;
}

bool Catalog::capture(const Game& game, const uint8_t* pixels) const {
    const auto path = stateDir + "/" + hexId(game.id) + ".preview";
    return replaceFile(path, [&](FILE* file) { return writeBytes(file, pixels, PreviewW * PreviewH); });
}

std::string readRom(const Game& game, Rom& rom) {
    if (game.packaged) {
        Package package(game.path);
        if (!package.ready) return "Can't read .arduboy archive";
        if (!package.extract(package.hex, rom.hex, MaxHexBytes)) return "Missing or invalid HEX";
        rom.title = std::move(package.title);
        rom.device = std::move(package.device);
        if (!package.fxData.empty() && !package.extract(package.fxData, rom.fxData, MaxFxBytes))
            return "FX data missing or too large";
        if (!package.fxSave.empty() &&
            !package.extract(package.fxSave, rom.fxSave, MaxFxBytes - rom.fxData.size()))
            return "FX save missing or too large";
    } else {
        if (!readVector(game.path, rom.hex, MaxHexBytes) || rom.hex.empty())
            return "Can't read HEX file";
        const auto base = game.path.substr(0, game.path.find_last_of('.'));
        const auto data = base + ".bin";
        const auto save = base + ".save.bin";
        if (exists(data) && !readVector(data, rom.fxData, MaxFxBytes)) return "FX data too large or unreadable";
        if (exists(save) && !readVector(save, rom.fxSave, MaxFxBytes - rom.fxData.size()))
            return "FX save too large or unreadable";
    }
    if (rom.title.empty()) rom.title = game.title;
    return {};
}

Settings readSettings(const std::string& path) {
    Settings result;
    if (!readSettingsFile(openFile(path, "rb"), result)) readSettingsFile(openBackup(path), result);
    return result;
}

bool writeSettings(const std::string& path, const Settings& settings) {
    std::array<uint8_t, 12> data{'A', 'G', 'S', '2', settings.brightness, settings.volume,
        uint8_t(settings.scale), settings.speed, settings.fps, settings.palette,
        uint8_t(settings.oledFilter | (settings.showFps << 1))};
    for (size_t i = 4; i < 11; ++i) data[11] ^= data[i];
    return replaceFile(path, [&](FILE* file) { return writeBytes(file, data.data(), data.size()); });
}

}
