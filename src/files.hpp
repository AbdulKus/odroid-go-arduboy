#pragma once

#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <string_view>

namespace go {

using File = std::unique_ptr<FILE, int(*)(FILE*)>;
inline File openFile(const std::string& path, const char* mode) {
    return File(std::fopen(path.c_str(), mode), std::fclose);
}

std::string extension(std::string_view path);
std::string stem(const std::string& path);
uint64_t pathId(std::string_view path);
std::string hexId(uint64_t id);
bool exists(const std::string& path);
bool makeDirectory(const std::string& path);
bool replaceFile(const std::string& path, const std::function<bool(FILE*)>& write);
File openBackup(const std::string& path);
long fileSize(FILE* file);
bool writeBytes(FILE* file, const void* data, size_t size);
bool readBytes(FILE* file, void* data, size_t size);
void writeLe(uint8_t* dest, uint64_t value, unsigned count);
uint64_t readLe(const uint8_t* src, unsigned count);

}
