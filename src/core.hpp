#pragma once

#include "types.hpp"

#include <memory>
#include <string>
#include <vector>

namespace absim { struct arduboy_t; }

namespace go {

struct Game;

class Core {
public:
    Core();
    ~Core();

    std::string load(const Game& game, const std::string& saveDir);
    void close();
    void run(unsigned elapsedUs, unsigned speed, uint16_t keys, bool filter);
    const uint8_t* pixels() const;
    const std::vector<int16_t>& samples() const;
    uint64_t cycles() const;
    const std::string& title() const;

    bool flush();
    bool reset();
    bool hasSlot(unsigned slot) const;
    std::string save(unsigned slot);
    std::string restore(unsigned slot);
    explicit operator bool() const { return bool(machine_); }

private:
    struct DeleteMachine { void operator()(absim::arduboy_t* machine) const; };
    std::unique_ptr<absim::arduboy_t, DeleteMachine> machine_;
    std::string savePath_;

    bool restoreProgress();
    void syncFlash();
    std::string slotPath(unsigned slot) const;
};

}
