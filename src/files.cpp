#include "files.hpp"

#include <cerrno>
#include <sys/stat.h>
#include <unistd.h>

namespace go {

std::string extension(std::string_view path) {
    const auto dot = path.rfind('.');
    const auto slash = path.rfind('/');
    if (dot == path.npos || (slash != path.npos && slash > dot)) return {};
    std::string ext(path.substr(dot));
    for (char& ch : ext) {
        if (ch >= 'A' && ch <= 'Z') ch += 'a' - 'A';
    }
    return ext;
}

std::string stem(const std::string& path) {
    auto pos = path.find_last_of('/');
    auto name = path.substr(pos == path.npos ? 0 : pos + 1);
    auto dot = name.find_last_of('.');
    return name.substr(0, dot);
}

uint64_t pathId(std::string_view path) {
    uint64_t hash = 0xcbf29ce484222325ull;
    for (unsigned char byte : path) {
        if (byte >= 'A' && byte <= 'Z') byte += 'a' - 'A';
        hash = (hash ^ byte) * 0x100000001b3ull;
    }
    return hash;
}

std::string hexId(uint64_t id) {
    char buffer[17];
    std::snprintf(buffer, sizeof(buffer), "%016llx", static_cast<unsigned long long>(id));
    return buffer;
}

bool exists(const std::string& path) {
    struct stat info{};
    return stat(path.c_str(), &info) == 0;
}

bool makeDirectory(const std::string& path) {
    if (mkdir(path.c_str(), 0755) == 0) return true;
    struct stat info{};
    return errno == EEXIST && stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

bool replaceFile(const std::string& path, const std::function<bool(FILE*)>& write) {
    const std::string temp = path + ".tmp";
    const std::string backup = path + ".bak";
    auto file = openFile(temp, "wb");
    if (!file) return false;
    bool ok = write(file.get()) && std::fflush(file.get()) == 0 && fsync(fileno(file.get())) == 0;
    ok = std::fclose(file.release()) == 0 && ok;
    if (ok && exists(path)) {
        std::remove(backup.c_str());
        ok = std::rename(path.c_str(), backup.c_str()) == 0;
    }
    if (ok && std::rename(temp.c_str(), path.c_str()) == 0) return true;
    std::remove(temp.c_str());
    return false;
}

File openBackup(const std::string& path) {
    return openFile(path + ".bak", "rb");
}

long fileSize(FILE* file) {
    if (std::fseek(file, 0, SEEK_END) != 0) return -1;
    long size = std::ftell(file);
    return std::fseek(file, 0, SEEK_SET) == 0 ? size : -1;
}

bool writeBytes(FILE* file, const void* data, size_t size) {
    return std::fwrite(data, 1, size, file) == size;
}

bool readBytes(FILE* file, void* data, size_t size) {
    return std::fread(data, 1, size, file) == size;
}

void writeLe(uint8_t* dest, uint64_t value, unsigned count) {
    for (unsigned i = 0; i < count; ++i) dest[i] = value >> (8 * i);
}

uint64_t readLe(const uint8_t* src, unsigned count) {
    uint64_t value = 0;
    for (unsigned i = 0; i < count; ++i) value |= uint64_t(src[i]) << (8 * i);
    return value;
}

}
