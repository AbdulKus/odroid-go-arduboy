#include "core.hpp"
#include "absim.hpp"
#include "absim_jit_xtensa.hpp"
#include "absim_strstream.hpp"
#include "catalog.hpp"
#include "files.hpp"
#include "miniz.h"

#include <array>
#include <cstdio>
#include <cstring>
#include <sstream>

#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
#endif

namespace go {
namespace {

using Machine = absim::arduboy_t;
using Flash = absim::w25q128_t;
constexpr size_t MaxFxSave = 64 * 1024;
constexpr size_t MaxSavedata = 96 * 1024;
constexpr size_t MaxState = 256 * 1024;

bool writeSave(const std::string& path, const char (&magic)[5], uint64_t id, const std::string& data) {
    std::array<uint8_t, 20> header{};
    std::memcpy(header.data(), magic, 4);
    writeLe(header.data() + 4, id, 8);
    writeLe(header.data() + 12, data.size(), 4);
    writeLe(header.data() + 16, mz_crc32(0, reinterpret_cast<const uint8_t*>(data.data()), data.size()), 4);
    return replaceFile(path, [&](FILE* file) {
        return writeBytes(file, header.data(), header.size()) && writeBytes(file, data.data(), data.size());
    });
}

bool readSave(File file, const char (&magic)[5], uint64_t id, size_t maxSize, std::vector<char>& data) {
    std::array<uint8_t, 20> header{};
    if (!file || !readBytes(file.get(), header.data(), header.size()) ||
        std::memcmp(header.data(), magic, 4) != 0 || readLe(header.data() + 4, 8) != id)
        return false;
    const auto size = readLe(header.data() + 12, 4);
    if (size == 0 || size > maxSize) return false;
    data.resize(size);
    return readBytes(file.get(), data.data(), data.size()) && std::fgetc(file.get()) == EOF &&
        mz_crc32(0, reinterpret_cast<const uint8_t*>(data.data()), size) == readLe(header.data() + 16, 4);
}

bool hasProgram(const std::vector<uint8_t>& bytes) {
    // Check for a program record at reset and an EOF record. Ardens checks record lengths and checksums.
    bool resetVector = false;
    bool eof = false;
    for (size_t offset = 0; offset < bytes.size();) {
        size_t end = offset;
        while (end < bytes.size() && bytes[end] != '\n' && bytes[end] != '\r') ++end;
        if (end - offset >= 11 && bytes[offset] == ':') {
            const auto* line = bytes.data() + offset;
            if (std::memcmp(line + 3, "000000", 6) == 0 &&
                (line[1] != '0' || line[2] != '0')) resetVector = true;
            if (std::memcmp(line + 1, "00000001", 8) == 0) eof = true;
        }
        if (eof) break;
        offset = end + 1;
    }
    return resetVector && eof;
}

void setButtons(Machine& m, uint16_t keys) {
    using namespace absim;
    auto& data = m.core_state.cpu.data;
    data[reg::addr::PINB] = keys & B ? 0 : 1 << 4;
    data[reg::addr::PINE] = keys & A ? 0 : 1 << 6;
    data[reg::addr::PINF] = 0xf0 &
        ~((keys & Up ? 1 << 7 : 0) | (keys & Right ? 1 << 6 : 0) |
          (keys & Left ? 1 << 5 : 0) | (keys & Down ? 1 << 4 : 0));
}

}

Core::Core() = default;
Core::~Core() = default;

void Core::DeleteMachine::operator()(Machine* machine) const {
#ifdef ESP_PLATFORM
    machine->~arduboy_t();
    heap_caps_free(machine);
#else
    delete machine;
#endif
}

std::string Core::load(const Game& game, const std::string& saveDir) {
    close();
    Rom rom;
    auto error = readRom(game, rom);
    if (!error.empty()) return error;
    if (rom.fxSave.size() > MaxFxSave) return "FX save exceeds 64 KiB";
    if (!hasProgram(rom.hex)) return "HEX needs a reset vector and EOF record";
#ifdef ESP_PLATFORM
    // OLED history and sparse FX tables live in PSRAM. The core places hot AVR state separately.
    void* memory = heap_caps_malloc(sizeof(Machine), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!memory) return "Insufficient emulator memory";
    try {
        machine_.reset(new (memory) Machine());
    } catch (...) {
        heap_caps_free(memory);
        throw;
    }
#else
    machine_.reset(new Machine());
#endif
    auto& m = *machine_;
    m.program_state.cfg.bootloader = false;
    absim::istrstream input(reinterpret_cast<const char*>(rom.hex.data()), rom.hex.size());
    error = m.load_file("game.hex", input);
    if (!error.empty()) { close(); return error; }

    if (!rom.device.empty()) m.program_state.device_type = std::move(rom.device);
    using absim::reg::addr::PORTE;
    auto& cfg = m.program_state.cfg;
    // Standalone handheld play has no active USB host; enumerating EP0 SETUP on startup overflows tight stacks.
    cfg.usb_bus_state = absim::USB_BUS_DISCONNECTED;
    if (m.program_state.device_type == "ArduboyFXDevKit") cfg.fxport_mask = 1 << 2;
    if (m.program_state.device_type == "ArduboyMini") {
        cfg.fxport_reg = PORTE;
        cfg.fxport_mask = 1 << 2;
    }
    m.program_state.title = std::move(rom.title);
    m.program_state.fxdata = std::move(rom.fxData);
    m.program_state.fxsave = std::move(rom.fxSave);
    Flash::fx_data_save_layout_t layout{};
    Flash::make_data_save_layout(m.program_state.fxdata.size(), m.program_state.fxsave.size(), layout);
    m.peripherals.fx.save_start = layout.save_offset;
    m.reload_fx();
    m.reset();
    m.peripherals.display.enable_filter = false;
    m.peripherals.display.enable_current_limiting = false;
    m.core_state.cpu.sound_buffer.reserve(AudioHz / 6);
    savePath_ = saveDir + "/" + hexId(m.program_state.game_hash);
    restoreProgress();
    return {};
}

void Core::close() {
    machine_.reset();
    savePath_.clear();
}

void Core::run(unsigned elapsedUs, unsigned speed, uint16_t keys, bool filter) {
    if (!machine_) return;
    auto& m = *machine_;
    setButtons(m, keys);
    if (filter && !m.peripherals.display.enable_filter)
        m.peripherals.display.reset_filter_state(true);
    m.peripherals.display.enable_filter = filter;
    m.peripherals.display.enable_current_limiting = false;
    m.core_state.cpu.sound_buffer.clear();
    m.advance(uint64_t(elapsedUs) * 10000 * speed);
}

const uint8_t* Core::pixels() const {
    auto& display = machine_->peripherals.display;
    if (display.enable_filter) {
        if (display.filter_dirty) display.filter_pixels();
        return display.filtered_pixels.data();
    }
    return display.pixels[0].data();
}

const std::vector<int16_t>& Core::samples() const { return machine_->core_state.cpu.sound_buffer; }
uint64_t Core::cycles() const { return machine_->core_state.cpu.cycle_count; }
const std::string& Core::title() const { return machine_->program_state.title; }
#ifdef ARDUGO_QUIET_TEST
void Core::clearBenchmark() {
    machine_->core_state.cpu.benchmark = {};
    absim::jit::jit_reset_stats();
}
void Core::reportBenchmark() const {
    const auto& p = machine_->core_state.cpu.benchmark;
    const auto& j = absim::jit::jit_runtime();
    std::printf("bench: batches=%lu micros_blocks=%lu micros_timer_blocks=%lu micros_calls=%lu micros_wait_skipped=%lu sampled_queue=%llu sampled_instruction=%llu sampled_devices=%llu\n",
                static_cast<unsigned long>(p.batches), static_cast<unsigned long>(p.micros_blocks),
                static_cast<unsigned long>(p.micros_timer_blocks),
                static_cast<unsigned long>(p.micros_calls),
                static_cast<unsigned long>(p.micros_wait_iterations),
                p.queue_cycles, p.instruction_cycles, p.peripheral_cycles);
    std::printf("jit: failed=%d cap=%lu used=%lu tbl_int=%d compiles=%lu(%llukc) resets=%lu entries=%lu misses=%lu fallbacks=%lu\n",
                int(j.failed), static_cast<unsigned long>(j.code_cap), static_cast<unsigned long>(j.code_used),
                int(j.block_in_internal), static_cast<unsigned long>(j.stat_compiles),
                p.compile_cycles / 1000ULL,
                static_cast<unsigned long>(j.stat_resets), static_cast<unsigned long>(j.stat_entries),
                static_cast<unsigned long>(j.stat_misses), static_cast<unsigned long>(j.stat_fallbacks));
    std::printf("branches: jit=%lu fb=%lu short=%lu wake=%lu sleep=%lu\n",
                static_cast<unsigned long>(p.branch_jit), static_cast<unsigned long>(p.branch_jit_fb),
                static_cast<unsigned long>(p.branch_short), static_cast<unsigned long>(p.branch_waking),
                static_cast<unsigned long>(p.branch_sleep));
    static const char* const kPqNames[14] = {
        "DMY", "SPI", "TSYNC", "T1", "T3", "T4", "USB", "WDT", "EEP", "PLL", "ADC", "SPM", "TWI", "IRQ"
    };
    std::printf("pq_pops:");
    for (size_t i = 0; i < 14; ++i)
        if (p.pq_pops[i]) std::printf(" %s=%lu", kPqNames[i], static_cast<unsigned long>(p.pq_pops[i]));
    std::printf(" | short_pq:");
    for (size_t i = 0; i < 14; ++i)
        if (p.short_pq[i]) std::printf(" %s=%lu", kPqNames[i], static_cast<unsigned long>(p.short_pq[i]));
    std::printf("\n");
    static const char* const kHNames[8] = {
        "outSPDR", "inSPDR", "inSPSR", "outPORT", "inTCNT0", "pendIRQ", "macro", "timed"
    };
    bool anyH = false;
    for (size_t i = 0; i < 8; ++i) if (p.helper_calls[i]) anyH = true;
    if (anyH) {
        std::printf("jit_helpers:");
        for (size_t i = 0; i < 8; ++i)
            if (p.helper_calls[i])
                std::printf(" %s=%lu(%llukc)", kHNames[i],
                            static_cast<unsigned long>(p.helper_calls[i]),
                            p.helper_cycles[i] / 1000ULL);
        std::printf("\n");
        std::printf("timed_funcs:");
        for (size_t i = 0; i < 160; ++i)
            if (p.timed_func_hits[i])
                std::printf(" f%u=%lu", unsigned(i), static_cast<unsigned long>(p.timed_func_hits[i]));
        std::printf("\n");
    }
}
#endif

bool Core::restoreProgress() {
    auto& m = *machine_;
    const std::string path = savePath_ + ".eep";
    const auto tryRead = [&](File file) {
        std::vector<char> bytes;
        if (!readSave(std::move(file), "AGE1", m.program_state.game_hash, MaxSavedata, bytes)) return false;
        absim::istrstream input(bytes.data(), bytes.size());
        if (!m.load_savedata(input)) return false;
        for (const auto& [index, data] : m.save_data_state.savedata.fx_sectors) {
            if (index >= Flash::NUM_SECTORS || index * Flash::SECTOR_BYTES < m.peripherals.fx.save_start) continue;
            m.peripherals.fx.sectors_modified.set(index);
            m.peripherals.fx.sectors_modified_data[index] = std::make_unique<Flash::sector_t>(data);
        }
        return true;
    };
    return tryRead(openFile(path, "rb")) || tryRead(openBackup(path));
}

bool Core::flush() {
    if (!machine_ || !machine_->save_data_state.dirty) return true;
    auto& m = *machine_;
    std::ostringstream stream(std::ios::binary);
    m.save_savedata(stream);
    if (!stream || !writeSave(savePath_ + ".eep", "AGE1", m.program_state.game_hash, stream.str())) return false;
    m.save_data_state.dirty = false;
    return true;
}

bool Core::reset() {
    if (!flush()) return false;
    auto& m = *machine_;
    m.reset();
    m.reload_fx();
    m.save_data_state.savedata.clear();
    restoreProgress();
    return true;
}

std::string Core::slotPath(unsigned slot) const { return savePath_ + ".slot" + std::to_string(slot + 1); }

bool Core::hasSlot(unsigned slot) const {
    const auto path = slotPath(slot);
    return exists(path) || exists(path + ".bak");
}

std::string Core::save(unsigned slot) {
    auto& m = *machine_;
    std::ostringstream stream(std::ios::binary);
    auto error = m.save_savestate(stream);
    if (!error.empty()) return error;
    if (!stream || !writeSave(slotPath(slot), "AGQ1", m.program_state.game_hash, stream.str())) return "Save failed";
    return {};
}

void Core::syncFlash() {
    auto& m = *machine_;
    auto& cpu = m.core_state.cpu;
    if(cpu.usb.bus_state != absim::USB_BUS_DISCONNECTED) {
        // Older slots include an emulated USB host. Drop its pending
        // endpoint requests before continuing standalone play.
        cpu.usb.bus_state = absim::USB_BUS_DISCONNECTED;
        cpu.reset_usb();
        cpu.update_usb();
    }
    auto& fx = m.peripherals.fx;
    auto modified = fx.sectors_modified;
    auto minPage = fx.min_page;
    auto maxPage = fx.max_page;
    auto hash = m.program_state.game_hash;
    m.reload_fx();
    fx.min_page = minPage;
    fx.max_page = maxPage;
    fx.sectors_modified = modified;
    m.program_state.game_hash = hash;
    m.peripherals.display.reset_filter_state(false);
    // Slots restore the running game without rewriting its persistent progress.
    m.core_state.cpu.eeprom_dirty = false;
    fx.sectors_dirty = false;
    m.core_state.cpu.sound_buffer.clear();
}

std::string Core::restore(unsigned slot) {
    auto& m = *machine_;
    const auto path = slotPath(slot);
    if (!hasSlot(slot)) return "Save slot is empty";
    if (!flush()) return "Can't write game progress";
    auto error = std::string("Save is corrupt or incompatible");
    const auto tryRead = [&](File file) {
        std::vector<char> bytes;
        if (!readSave(std::move(file), "AGQ1", m.program_state.game_hash, MaxState, bytes)) return false;
        absim::istrstream input(bytes.data(), bytes.size());
        auto result = m.load_savestate(input);
        if (!result.empty()) { error = std::move(result); return false; }
        syncFlash();
        return true;
    };
    if (tryRead(openFile(path, "rb")) || tryRead(openBackup(path))) return {};
    return error;
}

}
