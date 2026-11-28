#include "absim.hpp"
#include "absim_jit_xtensa.hpp"

#include <array>
#include <cstdio>
#include <cstring>
#include <memory>
#include <random>

namespace {

using absim::atmega32u4_t;
using absim::INSTR_MAP;
namespace reg = absim::reg::addr;

template <size_t N>
void copyProgram(atmega32u4_t& cpu, const std::array<uint16_t, N>& words, size_t pc = 0) {
    for (size_t i = 0; i < N; ++i) {
        cpu.prog[2 * (pc + i)] = words[i];
        cpu.prog[2 * (pc + i) + 1] = words[i] >> 8;
    }
}

bool compareExecution(atmega32u4_t& reference, atmega32u4_t& fast, unsigned budget) {
    const auto start = fast.cycle_count;
    const auto executed = fast.execute_merged(budget);
    while (reference.cycle_count < start + executed) {
        const auto i = reference.decoded_prog[reference.pc];
        reference.cycle_count += INSTR_MAP[i.func](reference, i);
    }
    return fast.pc == reference.pc && fast.cycle_count == reference.cycle_count &&
        fast.data == reference.data && fast.spi_wcol_read == reference.spi_wcol_read &&
        fast.spsr_read_after_transmit == reference.spsr_read_after_transmit &&
        fast.spi_flag_cycle == reference.spi_flag_cycle &&
        fast.spi_done_cycle == reference.spi_done_cycle && fast.spi_busy == reference.spi_busy &&
        fast.spi_transmit_zero_cycle == reference.spi_transmit_zero_cycle &&
        fast.spi_data_latched == reference.spi_data_latched &&
        fast.spi_datain_byte == reference.spi_datain_byte &&
        fast.spi_data_byte == reference.spi_data_byte &&
        fast.peripheral_queue.next_cycle() == reference.peripheral_queue.next_cycle();
}

constexpr std::array<uint16_t, 37> MicrosProgram = {
    0xb73f, 0x94f8, 0x9180, 0x0720, 0x9190, 0x0721, 0x91a0, 0x0722, 0x91b0, 0x0723,
    0xb526, 0x9ba8, 0xc005, 0x3f2f, 0xf019, 0x9601, 0x1da1, 0x1db1, 0xbf3f,
    0x2fba, 0x2fa9, 0x2f98, 0x2788, 0x0f82, 0x1d91, 0x1da1, 0x1db1, 0x01bc, 0x01cd,
    0xe042, 0x0f66, 0x1f77, 0x1f88, 0x1f99, 0x954a, 0xf7d1, 0x9508,
};

bool checkMicrosWait() {
    // CALL micros, load a 16-bit deadline, subtract the sampled time, and loop while negative.
    constexpr std::array<uint16_t, 11> poll = {
        0x940e, 0x0040, 0x9140, 0x0730, 0x9150, 0x0731, 0x019a, 0x1b26, 0x0b37, 0xfd37, 0xcff5,
    };
    atmega32u4_t reference{}, fast{};
    for (auto* cpu : {&reference, &fast}) {
        cpu->reset();
        cpu->prog.fill(0);
        copyProgram(*cpu, poll);
        copyProgram(*cpu, MicrosProgram, 0x40);
        cpu->decode();
        cpu->st(reg::TCCR0A, 3);
        cpu->st(reg::TCCR0B, 3);
    }
    if (fast.merged_prog[0].func != absim::INSTR_MERGED_MICROS_CALL) return false;

    std::mt19937 random(9467);
    for (unsigned n = 0; n < 16000; ++n) {
        const uint64_t start = 1100ULL * n;
        for (auto* cpu : {&reference, &fast}) {
            cpu->cycle_count = start;
            cpu->update_sync_timers();
            cpu->pc = 0;
            cpu->data[reg::SPL] = 0xf0;
            cpu->data[reg::SPH] = 0x0a;
        }
        const unsigned budget = 1 + random() % 1000;
        if (fast.timer_sync.next_update_cycle <= start + budget + 4) continue;
        for (unsigned i = 0; i < 32; ++i) reference.data[i] = random();
        for (unsigned i = 0x720; i < 0x724; ++i) reference.data[i] = random();
        reference.data[0x730] = random();
        reference.data[0x731] = random();
        reference.data[1] = n % 2 ? 0 : random();
        reference.data[reg::SREG] = random();
        reference.data[reg::TIFR0] = random() & 7;
        reference.data[reg::TIMSK0] = n % 4 == 0 ? 1 : 0;
        reference.prev_sreg = fast.prev_sreg = reference.data[reg::SREG];
        fast.data = reference.data;
        if (!compareExecution(reference, fast, budget)) {
            std::fprintf(stderr, "micros() poll mismatch in iteration %u (budget %u)\n", n, budget);
            return false;
        }
    }
    return true;
}

bool checkArduinoDelay() {
    constexpr std::array<uint16_t, 19> poll = {
        0x14c1, 0x04d1, 0x04e1, 0x04f1, 0xf0f1, 0x940e, 0x0040,
        0x01dc, 0x01cb, 0x1988, 0x0999, 0x09aa, 0x09bb, 0x3e88,
        0x4093, 0x05a1, 0x05b1, 0xf370, 0xcfff,
    };
    atmega32u4_t reference{}, fast{};
    for (auto* cpu : {&reference, &fast}) {
        cpu->reset();
        cpu->prog.fill(0);
        copyProgram(*cpu, poll);
        copyProgram(*cpu, MicrosProgram, 0x40);
        cpu->decode();
        cpu->st(reg::TCCR0A, 3);
        cpu->st(reg::TCCR0B, 3);
    }
    constexpr uint16_t CallPc = 5;
    if (fast.merged_prog[CallPc].func != absim::INSTR_MERGED_MICROS_CALL ||
        fast.merged_prog[CallPc].m2 == 0) return false;

    std::mt19937 random(8839);
    for (unsigned n = 0; n < 4000; ++n) {
        const uint64_t cycle = 1100ULL * n;
        const unsigned budget = 1 + random() % 1000;
        for (auto* cpu : {&reference, &fast}) {
            cpu->cycle_count = cycle;
            cpu->update_sync_timers();
            cpu->pc = CallPc;
        }
        if (fast.timer_sync.next_update_cycle <= cycle + budget + 4) continue;
        for (unsigned i = 0; i < 32; ++i) reference.data[i] = random();
        reference.data[1] = n % 5 == 0 ? random() : 0;
        reference.data[reg::SPL] = 0xf0;
        reference.data[reg::SPH] = 0x0a;
        reference.data[reg::SREG] = random();
        reference.data[reg::TIFR0] = n % 7 == 0 ? 1 : 0;
        reference.data[reg::TIMSK0] = 0;
        const uint32_t overflows = n % 2 == 0 ? random() : 0;
        const uint32_t delayStart = (overflows << 10) + 4 * reference.timer0.tcnt - random() % 1200;
        for (unsigned i = 0; i < 4; ++i) {
            reference.data[0x720 + i] = overflows >> (8 * i);
            reference.data[8 + i] = delayStart >> (8 * i);
        }
        reference.prev_sreg = fast.prev_sreg = reference.data[reg::SREG];
        fast.data = reference.data;
        if (!compareExecution(reference, fast, budget)) {
            std::fprintf(stderr, "Arduino delay mismatch in iteration %u (budget %u)\n", n, budget);
            return false;
        }
    }
    return true;
}

bool checkArduinoYieldDelay16() {
    constexpr std::array<uint16_t, 35> compactMicros = {
        0xb73f, 0x94f8, 0x9180, 0x0720, 0x9190, 0x0721, 0x91a0, 0x0722, 0x91b0, 0x0723,
        0xb526, 0x9ba8, 0xc005, 0x3f2f, 0xf019, 0x9601, 0x1da1, 0x1db1, 0xbf3f, 0x2766,
        0x2f78, 0x2f89, 0x2f9a, 0x0f62, 0x1d71, 0x1d81, 0x1d91, 0xe042, 0x0f66, 0x1f77,
        0x1f88, 0x1f99, 0x954a, 0xf7d1, 0x9508,
    };
    constexpr std::array<uint16_t, 15> poll = {
        0x14c1, 0x04d1, 0x04e1, 0x04f1, 0xf051,
        0x940e, 0x0030,
        0x940e, 0x0040,
        0x1b6c, 0x0b7d, 0x3e68, 0x4073, 0xf390, 0xcfff,
    };
    atmega32u4_t reference{}, fast{};
    for (auto* cpu : {&reference, &fast}) {
        cpu->reset();
        cpu->prog.fill(0);
        copyProgram(*cpu, poll);
        cpu->prog[0x30 * 2] = 0x08;
        cpu->prog[0x30 * 2 + 1] = 0x95; // yield() -> ret
        copyProgram(*cpu, compactMicros, 0x40);
        cpu->decode();
        cpu->st(reg::TCCR0A, 3);
        cpu->st(reg::TCCR0B, 3);
    }
    constexpr uint16_t CallPc = 7;
    if (fast.merged_prog[CallPc].func != absim::INSTR_MERGED_MICROS_CALL ||
        fast.merged_prog[CallPc].m2 != 3) {
        std::fprintf(stderr, "YieldDelay16 failed merge: func=%u m2=%u\n",
                     fast.merged_prog[CallPc].func, fast.merged_prog[CallPc].m2);
        return false;
    }

    std::mt19937 random(44921);
    for (unsigned n = 0; n < 4000; ++n) {
        const uint64_t cycle = 1100ULL * n;
        const unsigned budget = 1 + random() % 1000;
        for (auto* cpu : {&reference, &fast}) {
            cpu->cycle_count = cycle;
            cpu->update_sync_timers();
            cpu->pc = CallPc;
        }
        if (fast.timer_sync.next_update_cycle <= cycle + budget + 4) continue;
        for (unsigned i = 0; i < 32; ++i) reference.data[i] = random();
        reference.data[1] = n % 5 == 0 ? random() : 0;
        reference.data[reg::SPL] = 0xf0;
        reference.data[reg::SPH] = 0x0a;
        reference.data[reg::SREG] = random();
        reference.data[reg::TIFR0] = n % 7 == 0 ? 1 : 0;
        reference.data[reg::TIMSK0] = 0;
        const uint32_t overflows = n % 2 == 0 ? random() : 0;
        const uint32_t delayStart = (overflows << 10) + 4 * reference.timer0.tcnt - random() % 1200;
        for (unsigned i = 0; i < 4; ++i)
            reference.data[0x720 + i] = overflows >> (8 * i);
        reference.data[28] = uint8_t(delayStart);
        reference.data[29] = uint8_t(delayStart >> 8);
        reference.prev_sreg = fast.prev_sreg = reference.data[reg::SREG];
        fast.data = reference.data;
        if (!compareExecution(reference, fast, budget)) {
            std::fprintf(stderr, "YieldDelay16 mismatch in iteration %u (budget %u)\n", n, budget);
            return false;
        }
    }
    return true;
}

bool checkSpiWait(bool readData, bool useBus) {
    const std::array<uint16_t, 4> poll = {
        0xb58e, 0xb40d, 0xfe07, uint16_t(readData ? 0xcffc : 0xcffd),
    };
    absim::arduboy_t referenceMachine, fastMachine;
    referenceMachine.reset();
    fastMachine.reset();
    auto& reference = referenceMachine.core_state.cpu;
    auto& fast = fastMachine.core_state.cpu;
    for (auto* cpu : {&reference, &fast}) {
        if (!useBus) cpu->spi_bus = nullptr;
        cpu->prog.fill(0);
        copyProgram(*cpu, poll);
        cpu->decode();
    }
    const uint16_t first = readData ? 0 : 1;
    if (fast.merged_prog[first].func != absim::INSTR_MERGED_SPI_POLL) return false;
    std::mt19937 random(2257);
    for (unsigned n = 0; n < 16000; ++n) {
        const unsigned budget = 1 + random() % 300;
        const uint64_t start = (1ULL << 32) - 64000 + 400ULL * n;
        const uint64_t flagCycle = n % 3 == 0 ? start + random() % (budget + 1) : UINT64_MAX;
        const bool busy = n % 2 == 0;
        const bool fastTransfer = n % 5 != 0;
        const auto zeroCycle = start + 1 + random() % 17;
        for (auto* cpu : {&reference, &fast}) {
            cpu->pc = first;
            cpu->cycle_count = start;
            cpu->peripheral_queue.clear();
            cpu->spi_flag_cycle = flagCycle;
            cpu->spi_busy = busy;
            cpu->spi_fast_transfer = fastTransfer;
            cpu->spi_transmit_zero_cycle = zeroCycle;
            cpu->spi_done_cycle = zeroCycle - fastTransfer;
            cpu->spi_data_latched = false;
            cpu->spi_datain_byte = 0xff;
            if (!useBus) {
                const auto next = busy ? std::min(flagCycle, cpu->spi_done_cycle) : flagCycle;
                if (next != UINT64_MAX) cpu->peripheral_queue.schedule(next, absim::PQ_SPI);
            }
        }
        reference.data[reg::SPDR] = random();
        reference.data[reg::SPSR] = random() & 0xc1;
        reference.spi_wcol_read = fast.spi_wcol_read = random() % 2;
        reference.spsr_read_after_transmit = fast.spsr_read_after_transmit = random() % 2;
        fast.data = reference.data;
        if (!compareExecution(reference, fast, budget)) {
            std::fprintf(stderr, "SPI poll mismatch in iteration %u (budget %u, read data %d, bus %d)\n",
                         n, budget, readData, useBus);
            return false;
        }
    }
    return true;
}

bool checkSpiBatch() {
    // Read SPDR, write a new byte, read SPSR, and continue through regular instructions.
    constexpr std::array<uint16_t, 5> code = {0xb58e, 0xbd9e, 0xb5ad, 0x278a, 0xcfff};
    atmega32u4_t reference{}, fast{};
    for (auto* cpu : {&reference, &fast}) {
        cpu->reset();
        cpu->prog.fill(0);
        copyProgram(*cpu, code);
        cpu->decode();
    }
    std::mt19937 random(1372);
    for (unsigned n = 0; n < 1000; ++n) {
        const uint64_t start = (1ULL << 32) - 64000 + 128ULL * n;
        const unsigned budget = 5 + random() % 60;
        for (auto* cpu : {&reference, &fast}) {
            cpu->pc = 0;
            cpu->cycle_count = start;
            cpu->peripheral_queue.clear();
            cpu->spi_flag_cycle = cpu->spi_done_cycle = cpu->spi_transmit_zero_cycle = UINT64_MAX;
            cpu->spi_busy = cpu->spi_data_latched = false;
        }
        reference.st(reg::SPCR, 0x50 | (n & 3));
        reference.st(reg::SPSR, n % 2);
        reference.data[reg::SPSR] |= random() & 0xc0;
        reference.data[reg::SPDR] = random();
        reference.data[25] = random();
        reference.spi_wcol_read = fast.spi_wcol_read = random() % 2;
        reference.spsr_read_after_transmit = fast.spsr_read_after_transmit = random() % 2;
        fast.spi_clock_cycles = reference.spi_clock_cycles;
        fast.data = reference.data;
        if (!compareExecution(reference, fast, budget) || fast.cycle_count >= fast.spi_done_cycle) {
            std::fprintf(stderr, "SPI batch mismatch in iteration %u (budget %u)\n", n, budget);
            return false;
        }
    }
    return true;
}

bool checkFxPins(uint8_t port, uint8_t mask) {
    absim::arduboy_t machine;
    machine.program_state.cfg.fxport_reg = port;
    machine.program_state.cfg.fxport_mask = mask;
    machine.reset();
    auto& cpu = machine.core_state.cpu;
    auto& flash = machine.peripherals.fx;
    cpu.peripheral_queue.clear();
    cpu.cycle_count = (1ULL << 32) - 16;
    flash.write_byte(0x000fff, 0xa5);
    flash.write_byte(0x001000, 0x5a);
    cpu.st(reg::PORTD, 0xc0);
    const auto select = [&](bool enabled) {
        cpu.st(port, enabled ? (cpu.data[port] & ~mask) : (cpu.data[port] | mask));
    };
    cpu.st(reg::SPCR, 0x50);
    cpu.st(reg::SPSR, 1);
    select(true);
    cpu.st(reg::SPDR, 0xab);
    cpu.cycle_count += 16;
    select(false);
    if (!flash.woken_up || flash.enabled || (cpu.ld(reg::SPSR) & 0x80)) return false;
    cpu.cycle_count += 2;
    if (!(cpu.ld(reg::SPSR) & 0x80)) return false;
    cpu.ld(reg::SPDR);

    select(true);
    const auto exchange = [&](uint8_t byte, uint8_t expected) {
        cpu.st(reg::SPDR, byte);
        cpu.cycle_count += 16;
        if (cpu.ld(reg::SPDR) != expected || (cpu.ld(reg::SPSR) & 0x80)) return false;
        cpu.cycle_count += 2;
        if (!(cpu.ld(reg::SPSR) & 0x80)) return false;
        return cpu.ld(reg::SPDR) == expected && !(cpu.ld(reg::SPSR) & 0x80);
    };
    return exchange(0x03, 0) && exchange(0x00, 0) && exchange(0x0f, 0) &&
        exchange(0xff, 0) && exchange(0, 0xa5) && exchange(0, 0x5a) &&
        cpu.peripheral_queue.next_cycle() == UINT64_MAX;
}

bool checkFusedInstructions() {
    constexpr std::array<uint16_t, 9> code = {
        0x0c64, 0x1c72, 0x1c82, 0xfc87, 0xc001, 0x2c97, 0xfe71, 0xc000, 0xcfff,
    };
    atmega32u4_t reference{}, fast{};
    for (auto* cpu : {&reference, &fast}) {
        cpu->reset();
        cpu->prog.fill(0);
        copyProgram(*cpu, code);
        cpu->decode();
    }
    if (fast.merged_prog[0].func != absim::INSTR_MERGED_ADD_ADC2 ||
        fast.merged_prog[3].func != absim::INSTR_MERGED_SKIP_RJMP ||
        fast.merged_prog[6].func != absim::INSTR_MERGED_SKIP_RJMP) return false;

    std::mt19937 random(23487);
    for (unsigned n = 0; n < 4000; ++n) {
        reference.pc = fast.pc = 0;
        reference.cycle_count = fast.cycle_count = uint64_t(n) << 20;
        for (unsigned i = 0; i < 32; ++i) reference.data[i] = random();
        reference.data[reg::SREG] = random();
        fast.data = reference.data;
        if (!compareExecution(reference, fast, 1 + random() % 64)) return false;
    }
    return true;
}

bool checkDelayCall() {
    constexpr std::array<uint16_t, 2> call = {0xd00f, 0xcfff};
    constexpr std::array<uint16_t, 6> delay = {0, 0xc000, 0x95c8, 0xc000, 0xc000, 0x9508};
    atmega32u4_t reference{}, fast{};
    for (auto* cpu : {&reference, &fast}) {
        cpu->reset();
        cpu->prog.fill(0);
        copyProgram(*cpu, call);
        copyProgram(*cpu, delay, 16);
        cpu->decode();
    }
    if (fast.merged_prog[0].func != absim::INSTR_MERGED_DELAY_CALL) return false;
    for (unsigned budget = 1; budget < 32; ++budget) {
        reference.pc = fast.pc = 0;
        reference.cycle_count = fast.cycle_count = uint64_t(budget) << 32;
        reference.data[reg::SPL] = 0xf0;
        reference.data[reg::SPH] = 0x0a;
        reference.data[30] = 32 + budget;
        reference.data[31] = 0;
        reference.data[0] = 0xaa;
        fast.data = reference.data;
        if (!compareExecution(reference, fast, budget)) return false;
    }
    return true;
}

bool checkReturnStack() {
    atmega32u4_t reference{}, fast{};
    reference.reset();
    fast.reset();
    constexpr unsigned End = atmega32u4_t::DATA_SIZE_BYTES;
    constexpr unsigned stacks[] = {0, 1, reg::SPL, reg::SPH, reg::SREG,
        0xff, 0x100, 0x101, 0x7ff, End - 1, End, 0xffff};
    for (const auto stack : stacks) {
        for (auto* cpu : {&reference, &fast}) {
            cpu->data[reg::SPL] = uint8_t(stack);
            cpu->data[reg::SPH] = uint8_t(stack >> 8);
            cpu->pc = 0x123;
        }
        fast.data = reference.data;
        reference.push(0x25);
        reference.push(1);
        if (INSTR_MAP[absim::INSTR_CALL](fast, {0x345, 0, 0, 0, 0, 0, 0}) != 4 ||
            fast.pc != 0x345 || fast.data != reference.data) return false;
        const unsigned high = reference.pop();
        const unsigned low = reference.pop();
        if (INSTR_MAP[absim::INSTR_RET](fast, {}) != 4 ||
            fast.pc != (high << 8 | low) || fast.data != reference.data) return false;
    }
    return true;
}

template <size_t N>
bool checkDisplayLoop(const std::array<uint16_t, N>& words, uint8_t operation) {
    absim::arduboy_t referenceMachine, fastMachine;
    for (auto* machine : {&referenceMachine, &fastMachine}) {
        machine->program_state.cfg.bootloader = false;
        machine->reset();
        auto& cpu = machine->core_state.cpu;
        cpu.prog.fill(0);
        copyProgram(cpu, words);
        cpu.prog[N * 2] = 0xff;
        cpu.prog[N * 2 + 1] = 0xcf;
        cpu.decode();
        for (unsigned i = 0; i < 128; ++i)
            machine->peripherals.fx.write_byte(0xff0 + i, uint8_t(i * 37 + 2));
    }
    auto& reference = referenceMachine.core_state.cpu;
    auto& fast = fastMachine.core_state.cpu;
    if (fast.merged_prog[0].func != operation) return false;

    std::mt19937 random(16007);
    for (unsigned trial = 0; trial < 256; ++trial) {
        for (auto& regValue : reference.data) regValue = random();
        const uint16_t cursor = (trial % 5) ? 0xc00 - 0x400 : 0x00fc;
        reference.data[26] = operation == absim::INSTR_MERGED_OLED_SEND ? trial : uint8_t(cursor);
        reference.data[27] = operation == absim::INSTR_MERGED_OLED_SEND ? 0 : uint8_t(cursor >> 8);
        reference.data[30] = trial % 4 ? 0 : 1;
        reference.data[31] = 8;
        reference.data[9] = trial;
        reference.data[7] = reference.data[25] = 1 + trial % 31;
        if (trial % 2 == 0) reference.data[24] = reference.data[1];
        const auto data = reference.data;
        for (auto* machine : {&referenceMachine, &fastMachine}) {
            machine->reset();
            auto& cpu = machine->core_state.cpu;
            cpu.data = data;
            cpu.pc = 0;
            cpu.cycle_count = (1ULL << 32) + 1000ULL * trial;
            cpu.st(reg::PORTD, trial % 2 ? 0x90 : 0x92);
            cpu.st(reg::SPCR, 0x50 | ((trial / 8) % 4));
            cpu.st(reg::SPSR, (trial / 4) % 2);
            auto& flash = machine->peripherals.fx;
            flash.woken_up = true;
            flash.processing_command = true;
            flash.reading = 4;
            flash.current_addr = 0xff0;
        }
        const unsigned budget = 1 + random() % 1000;
        if (!compareExecution(reference, fast, budget) ||
            referenceMachine.peripherals.display.ram != fastMachine.peripherals.display.ram ||
            referenceMachine.peripherals.fx.current_addr != fastMachine.peripherals.fx.current_addr) {
            std::fprintf(stderr, "Display loop %u mismatch in trial %u (budget %u)\n", operation, trial, budget);
            return false;
        }
    }
    return true;
}

bool checkDisplayLoops() {
    constexpr std::array<uint16_t, 9> oled = {
        0x8000, 0xbc0e, 0x1181, 0x2c01, 0x9711, 0xfda0, 0xcffd, 0x9201, 0xf7b9,
    };
    constexpr std::array<uint16_t, 13> blend = {
        0xb58e, 0xbc1e, 0x9f82, 0x906c, 0x226c, 0x2860, 0x926d,
        0x8060, 0x226d, 0x2861, 0x9261, 0x947a, 0xf799,
    };
    constexpr std::array<uint16_t, 47> masked = {
        0xb40e, 0xbc1e, 0xfc93, 0x2606, 0x2f86, 0xfc90, 0x2d80, 0xfc92,
        0x2400, 0x9e03, 0x01a0, 0xfa94, 0xf436, 0x95c8, 0x95c8, 0x2411,
        0xb40e, 0xbc1e, 0xfc90, 0x2e08, 0x9e03, 0x3000, 0xf034, 0x918c,
        0xfe91, 0x2748, 0x2140, 0x2748, 0x934c, 0x58a0, 0x4fbf, 0xfe97,
        0xc006, 0x918c, 0xfe91, 0x2758, 0x2151, 0x2758, 0x935c, 0x2411,
        0xfc95, 0x58a1, 0xfe95, 0x57af, 0x09b1, 0x959a, 0xf689,
    };
    return checkDisplayLoop(oled, absim::INSTR_MERGED_OLED_SEND) &&
        checkDisplayLoop(blend, absim::INSTR_MERGED_FX_BLEND) &&
        checkDisplayLoop(masked, absim::INSTR_MERGED_FX_MASKED_BLEND);
}

bool checkBatchInlineIoAndIrq() {
    // 1) Post-completion IN SPDR / SPSR + PORTD / SBI / CBI stay in one batch
    // without early exits, and remain cycle- and state-identical to unmerged stepping.
    absim::arduboy_t refArdu{}, fastArdu{};
    auto& ref = refArdu.core_state.cpu;
    auto& fast = fastArdu.core_state.cpu;
    refArdu.reset();
    fastArdu.reset();
    ref.spi_bus = &refArdu;
    fast.spi_bus = &fastArdu;

    // Program: nop; out PORTD, r17; out SPDR, r18; 18 nops;
    // in r0, SPDR; in r19, SPSR; cbi PORTD, 4; sbi PORTD, 4;
    // sbic PINF, 7; nop; sbis PINF, 7; lds r20, 0x0100; push r18; pop r21;
    // ldi r30, 35; ldi r31, 0; ijmp; rjmp -1
    constexpr std::array<uint16_t, 36> prog = {
        0x0000, // 0: nop
        0xb91b, // 1: out PORTD (0x2b), r17
        0xbe2e, // 2: out SPDR (0x4e), r18
        0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // 3..8
        0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // 9..14
        0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // 15..20
        0xb40e, // 21: in r0, SPDR (0x4e)
        0xb53d, // 22: in r19, SPSR (0x4d)
        0x985c, // 23: cbi PORTD (0x0b), 4
        0x9a5c, // 24: sbi PORTD (0x0b), 4
        0x997f, // 25: sbic PINF (0x0f), 7 (PINF=0 -> skips 26)
        0x0000, // 26: nop
        0x9b7f, // 27: sbis PINF (0x0f), 7 (not skipped)
        0x9140, 0x0100, // 28..29: lds r20, 0x0100
        0x932f, // 30: push r18
        0x915f, // 31: pop r21
        0xe2e3, // 32: ldi r30, 35
        0xe0f0, // 33: ldi r31, 0
        0x9409, // 34: ijmp
        0xcfff, // 35: rjmp -1
    };
    copyProgram(ref, prog);
    copyProgram(fast, prog);
    for (auto* c : {&ref, &fast}) {
        c->st(reg::SPCR, 0x50); // SPE|MSTR
        c->st(reg::SPSR, 0x01); // SPI2X (clk/2 -> 18 cycles per byte)
        c->data[17] = 0x10; // PORTD (OLED CS low, DC high -> OLED data mode)
        c->data[18] = 0xae;
        c->data[0x100] = 0x7b;
        c->data[reg::SPL] = 0xff;
        c->data[reg::SPH] = 0x0a;
        c->decode();
        c->merge_instrs();
    }
    ref.pc = fast.pc = 1;
    const auto cycles = fast.execute_merged(80);
    while (ref.cycle_count < cycles) {
        const auto insn = ref.decoded_prog[ref.pc];
        ref.cycle_count += INSTR_MAP[insn.func](ref, insn);
    }
    if (fast.pc != 35 || ref.pc != 35 || fast.cycle_count != ref.cycle_count ||
        fast.data[20] != 0x7b || fast.data[21] != 0xae ||
        fast.data[0] != ref.data[0] || fast.data[19] != ref.data[19] ||
        fast.data[reg::PORTD] != ref.data[reg::PORTD] ||
        fastArdu.peripherals.display.ram != refArdu.peripherals.display.ram) {
        std::fprintf(stderr, "T1 fail pc=%u/%u cyc=%llu/%llu r20=%u r21=%u r0=%u/%u r19=%u/%u portd=%u/%u\n",
            fast.pc, ref.pc, (unsigned long long)fast.cycle_count, (unsigned long long)ref.cycle_count,
            fast.data[20], fast.data[21], fast.data[0], ref.data[0], fast.data[19], ref.data[19],
            fast.data[reg::PORTD], ref.data[reg::PORTD]);
        return false;
    }

    // 2) SREG / cli / out SREG / sei 1-instruction shadow accuracy (with and without pending IRQ).
    constexpr std::array<uint16_t, 12> irqProg = {
        0xb72f, // 0: in r18, SREG
        0x94f8, // 1: cli
        0x9180, 0x0100, // 2..3: lds r24, 0x0100
        0xbf2f, // 4: out SREG, r18
        0x9583, // 5: inc r24 (must always execute in the 1-instruction shadow!)
        0x9380, 0x0100, // 6..7: sts 0x0100, r24
        0xcfff, // 8: rjmp -1
        0x9593, // 9: (timer0 OVF vector 0x2e is word 0x17? Use pc=23! Put vector at 0x2e/2=0x17=23)
        0x0000, 0x0000,
    };
    constexpr std::array<uint16_t, 2> isrProg = {
        0x9593, // 0x2e (word vector = TIMER0_OVF): inc r25
        0x9518, // 0x2f: reti
    };
    // Test A: no interrupt pending during out SREG -> whole sequence runs without PQ_INTERRUPT.
    absim::arduboy_t arduA{};
    auto& cpuA = arduA.core_state.cpu;
    arduA.reset();
    copyProgram(cpuA, irqProg, 0);
    copyProgram(cpuA, isrProg, 0x2e);
    cpuA.data[reg::SREG] = absim::reg::bit::SREG::I;
    cpuA.data[0x100] = 5;
    cpuA.data[reg::SPL] = 0xff;
    cpuA.data[reg::SPH] = 0x0a;
    cpuA.decode();
    cpuA.merge_instrs();
    cpuA.pc = 0;
    for (int step = 0; step < 5; ++step) arduA.cycle();
    if (cpuA.data[0x100] != 6 || cpuA.data[25] != 0) {
        std::fprintf(stderr, "TA fail pc=%u r24=%u m100=%u r25=%u\n", cpuA.pc, cpuA.data[24], cpuA.data[0x100], cpuA.data[25]);
        return false;
    }

    // Test B: TOV0 pending when out SREG restores I -> inc r24 MUST run before TIMER0_OVF vector (0x2e)!
    absim::arduboy_t arduB{};
    auto& cpuB = arduB.core_state.cpu;
    arduB.reset();
    copyProgram(cpuB, irqProg, 0);
    copyProgram(cpuB, isrProg, 0x2e);
    cpuB.data[reg::SREG] = absim::reg::bit::SREG::I;
    cpuB.data[0x100] = 5;
    cpuB.data[reg::SPL] = 0xff;
    cpuB.data[reg::SPH] = 0x0a;
    cpuB.data[reg::TIMSK0] = absim::reg::bit::TIMSK0::TOIE0;
    cpuB.data[reg::TIFR0] = absim::reg::bit::TIFR0::TOV0;
    cpuB.decode();
    cpuB.merge_instrs();
    cpuB.pc = 1;
    cpuB.data[18] = absim::reg::bit::SREG::I;
    cpuB.data[reg::SREG] = absim::reg::bit::SREG::I;
    cpuB.prev_sreg = absim::reg::bit::SREG::I;
    for (int step = 0; step < 30 && (cpuB.data[25] == 0 || cpuB.pc != 8); ++step)
        arduB.cycle();
    if (cpuB.data[24] != 6 || cpuB.data[25] != 1 || cpuB.data[0x100] != 6 || cpuB.pc != 8) {
        std::fprintf(stderr, "TB fail pc=%u r24=%u r25=%u m100=%u sreg=%u\n", cpuB.pc, cpuB.data[24], cpuB.data[25], cpuB.data[0x100], cpuB.data[reg::SREG]);
        return false;
    }

    return true;
}

bool checkIdleTimer1() {
    using namespace absim::reg::bit::TIFR1;
    atmega32u4_t cpu{};
    cpu.reset();
    cpu.st(reg::TCCR1B, 0x09); // CTC with TOP=OCR1A=0, prescaler 1.
    cpu.cycle_count = 2;
    cpu.update_sync_timers();
    const uint8_t compareFlags = OCF1A | OCF1B | OCF1C;
    if ((cpu.ld(reg::TIFR1) & compareFlags) != compareFlags ||
        cpu.timer1.next_update_cycle != UINT64_MAX) return false;

    cpu.cycle_count = 4096;
    if (cpu.ld(reg::TCNT1L) != 0 || cpu.ld(reg::TCNT1H) != 0) return false;
    cpu.st(reg::TIFR1, OCF1A);
    if ((cpu.ld(reg::TIFR1) & compareFlags) != (OCF1B | OCF1C) ||
        cpu.timer1.next_update_cycle != UINT64_MAX) return false;
    cpu.cycle_count = 4097;
    cpu.update_sync_timers();
    if ((cpu.ld(reg::TIFR1) & compareFlags) != compareFlags ||
        cpu.timer1.next_update_cycle != UINT64_MAX) return false;

    cpu.cycle_count = 8192;
    cpu.peripheral_queue.clear();
    cpu.st(reg::TIMSK1, absim::reg::bit::TIMSK1::OCIE1A);
    const auto next = cpu.peripheral_queue.next();
    if (cpu.timer1.next_update_cycle != 8193 ||
        next.type != absim::PQ_INTERRUPT || next.cycle != cpu.cycle_count) return false;
    cpu.st(reg::TIMSK1, 0);
    cpu.st(reg::OCR1AH, 0);
    cpu.st(reg::OCR1AL, 3);
    if (cpu.timer1.top != 3 || cpu.timer1.next_update_cycle != UINT64_MAX) return false;
    cpu.st(reg::TIMSK1, absim::reg::bit::TIMSK1::OCIE1A);
    return cpu.timer1.next_update_cycle != UINT64_MAX;
}

bool checkIdleTimer4() {
    atmega32u4_t cpu{};
    cpu.reset();
    cpu.st(reg::DDRC, 0xc0);
    cpu.st(reg::OCR4C, 0x7f);
    cpu.st(reg::OCR4A, 0x59);
    cpu.st(reg::TCCR4A, 0x42);
    cpu.st(reg::TCCR4B, 0x01);
    cpu.st(reg::TCCR4C, 0x01);
    cpu.st(reg::TCCR4D, 0x01);
    cpu.st(reg::TCCR4E, 0x40);
    // Clear initial start_delay / compare_block ticks.
    for (int i = 1; i <= 32; ++i) {
        cpu.cycle_count = uint64_t(i);
        cpu.update_timer4();
    }
    if (!cpu.sound_pwm || cpu.timer4.next_update_cycle != UINT64_MAX) return false;

    // Advance far without PQ_TIMER4 events and check lazy TCNT4 / TIFR4 catchup matches period modulo.
    cpu.cycle_count = 32 + 128000;
    const uint8_t tcnt_before_clear = cpu.ld(reg::TCNT4);
    const uint8_t tifr = cpu.ld(reg::TIFR4);
    if ((tifr & absim::reg::bit::TIFR4::TOV4) == 0 ||
        (tifr & absim::reg::bit::TIFR4::OCF4A) == 0 ||
        tcnt_before_clear > 0x7f)
        return false;

    // Enabling TIMSK4 resumes PQ_TIMER4 scheduling immediately.
    cpu.st(reg::TIMSK4, absim::reg::bit::TIMSK4::TOIE4);
    if (cpu.timer4.next_update_cycle == UINT64_MAX) return false;
    cpu.st(reg::TIMSK4, 0);
    return cpu.timer4.next_update_cycle == UINT64_MAX;
}

bool checkXtensaJitCompiler() {
    using namespace absim::jit;
    // 1) Verify core Xtensa LX6 instruction word encodings.
    {
        XtensaWriter w(0);
        w.entry32();
        w.retw_n();
        w.nop_n();
        w.mov_n(10, 2);
        const auto& b = w.bytes();
        auto u24 = [&](size_t off) {
            return uint32_t(b[off]) | (uint32_t(b[off + 1]) << 8) | (uint32_t(b[off + 2]) << 16);
        };
        auto u16 = [&](size_t off) {
            return uint16_t(b[off]) | (uint16_t(b[off + 1]) << 8);
        };
        if (b.size() != 9 || u24(0) != 0x004136u || u16(3) != 0xf01du ||
            u16(5) != 0xf03du || u16(7) != 0x02adu) {
            std::fprintf(stderr, "Xtensa encoding mismatch: %06x %04x %04x %04x\n",
                u24(0), u16(3), u16(5), u16(7));
            return false;
        }
    }

    // 2) Compile representative AVR blocks (fused branches, dense LDD/STD, calls, I/O, LDS < 256, RJMP -1).
    atmega32u4_t cpu{};
    // Build test program directly before decode() to avoid flash hex overhead.
    const std::array<uint16_t, 36> prog = {
        // 0..3: LDD/ADC/CP/BRNE loop back to 0
        0x8189, // ldd r24, Y+1
        0x1f89, // adc r24, r25
        0x1789, // cp r24, r25
        0xf7e1, // brne -4 (to 0)
        // 4..8: Dense LDD/ADD/ADC/STD sequence exercising 8-bit slow-branch budget trimming
        0x818a, // ldd r24, Y+2
        0x0f89, // add r24, r25
        0x1f89, // adc r24, r25
        0x838a, // std Y+2, r24
        0x819b, // ldd r25, Y+3
        0x0f98, // add r25, r24
        0x1f98, // adc r25, r24
        0x839b, // std Y+3, r25
        0x9191, // ld r25, Z+
        0x939d, // st X+, r25
        // 14..21: Stack, call, ret, reti, I/O, SREG
        0x938f, // push r24
        0x918f, // pop r24
        0xd004, // rcall +4 (to 21)
        0x9478, // sei (BSET 7)
        0x94f8, // cli (BCLR 7)
        0xb586, // in r24, 0x26 (TCNT0 = 0x46 -> 0x26+32=0x46)
        0xc002, // rjmp +2 (to 23)
        // 21..22: callee: lds r24, 0x005f (SREG, 2 words -> timed stub!), ret
        0x9508, // ret (21)
        0x9518, // reti (22)
        // 23..26: SPDR / PORTD out, cpi, brlo to 0
        0xbd8e, // out SPDR(0x2e), r24
        0xb98b, // out PORTD(0x0b), r24
        0x3085, // cpi r24, 5
        0xf008, // brlo +1 (to 28)
        // 27..29: two-word LDS < 256 (0x005f) at PC=27
        0x9180, 0x005f, // lds r24, 0x005f
        0x9508, // ret
        // 30: RJMP -1 self idle loop (must return false)
        0xcfff,
        // 31..35: nop padding
        0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    };
    copyProgram(cpu, prog);
    cpu.decode();
    cpu.reset();
    cpu.spi_bus = reinterpret_cast<decltype(cpu.spi_bus)>(1); // allow PORTD native check if needed
    cpu.jit_decode_gen = 1;

    JitRuntime rt;
    const JitOffsets off = compute_jit_offsets(cpu);
    jit_reset_runtime(rt, cpu, off);
    if (rt.failed || !rt.code_buf || rt.entry_off == 0) {
        std::fprintf(stderr, "jit_reset_runtime failed\n");
        return false;
    }

    // Compile all interesting entry points and verify every one succeeds cleanly.
    const uint16_t entries[] = {0, 4, 14, 17, 21, 22, 23, 27};
    for (uint16_t ep : entries) {
        if (!jit_compile_block(rt, cpu, off, ep) || rt.block_off[ep] == 0) {
            std::fprintf(stderr, "jit_compile_block failed at pc=%u\n", ep);
            return false;
        }
    }

    // RJMP -1 self loop MUST return false so execute_merged handles idle wait.
    if (jit_compile_block(rt, cpu, off, 30)) {
        std::fprintf(stderr, "jit_compile_block unexpectedly compiled RJMP -1\n");
        return false;
    }

    // Backpatching should have resolved jump patches to compiled blocks (e.g. target 0 and 21).
    for (const auto& p : rt.patches) {
        if (rt.block_off[p.target_pc] != 0) {
            std::fprintf(stderr, "Unresolved patch to compiled block pc=%u\n", p.target_pc);
            return false;
        }
    }

    jit_invalidate_runtime(rt, cpu);
    if (rt.code_used != rt.stubs_end || rt.block_off[0] != 0 || !rt.patches.empty()) {
        std::fprintf(stderr, "jit_invalidate_runtime did not clear runtime\n");
        return false;
    }

    // 3) Verify batched PUSH/POP/LDD_Y/STD_Y, shared call/ret stubs, and multi-arena spill/reset.
    const std::array<uint16_t, 20> batch_prog = {
        0x922f, // push r2
        0x923f, // push r3
        0x924f, // push r4
        0x925f, // push r5
        0x8389, // std Y+1, r24
        0x839a, // std Y+2, r25
        0x81cb, // ldd r28, Y+3  (overwrites Y low -> splits LDD_Y batch safely)
        0x8199, // ldd r25, Y+1
        0x940e, 0x000e, // call 14
        0x905f, // pop r5
        0x904f, // pop r4
        0x903f, // pop r3
        0x902f, // pop r2
        0x9508, // ret (pc=14)
        0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    };
    copyProgram(cpu, batch_prog);
    cpu.decode();
    cpu.spi_bus = nullptr;
    cpu.reset();
    cpu.jit_decode_gen = 2;

    JitRuntime m_rt;
    m_rt.host_code.assign(8192 / 4, 0);
    // Two separated 2 KiB arenas inside host_code: [0..2048) and [4096..6144).
    m_rt.arenas[0] = {m_rt.host_code.data(), reinterpret_cast<uintptr_t>(m_rt.host_code.data()), 0, 2048, 0};
    m_rt.arenas[1] = {m_rt.host_code.data() + 4096 / 4,
                      reinterpret_cast<uintptr_t>(m_rt.host_code.data() + 4096 / 4), 4096, 2048, 0};
    m_rt.arena_count = 2;
    m_rt.code_buf = m_rt.arenas[0].buf;
    m_rt.code_base_addr = m_rt.arenas[0].base_addr;
    m_rt.code_cap = 4096;
    m_rt.block_off = m_rt.host_blocks.data();
    m_rt.block_in_internal = true;

    jit_reset_runtime(m_rt, cpu, off);
    if (m_rt.failed || m_rt.call1_off == 0 || m_rt.call2_off == 0 || m_rt.ret_off == 0) {
        std::fprintf(stderr, "Multi-arena stub setup failed\n");
        return false;
    }
    if (!jit_compile_block(m_rt, cpu, off, 0) || m_rt.block_off[0] == 0) {
        std::fprintf(stderr, "Batched prologue+CALL compile failed\n");
        return false;
    }
    const uint32_t single_block_bytes = m_rt.code_used - m_rt.stubs_end;
    if (single_block_bytes > 240) {
        std::fprintf(stderr, "Batched block unexpectedly large: %u bytes\n", single_block_bytes);
        return false;
    }
    // Artificially fill arena 0 close to cap and verify next compile spills into arena 1.
    m_rt.arenas[0].used = 2000;
    if (!jit_compile_block(m_rt, cpu, off, 10) || !jit_compile_block(m_rt, cpu, off, 14)) {
        std::fprintf(stderr, "Arena 1 spill compile failed\n");
        return false;
    }
    if (m_rt.cur_arena != 1 || (uint32_t(m_rt.block_off[10]) << 2) < 4096) {
        std::fprintf(stderr, "Expected spill into arena 1 (>=4096), cur=%u off=%u\n",
                     m_rt.cur_arena, uint32_t(m_rt.block_off[10]) << 2);
        return false;
    }
    // Fill arena 1 as well and confirm single-arena eviction resets arena 0 while preserving arena 1.
    m_rt.arenas[1].used = 2000;
    if (!jit_compile_block(m_rt, cpu, off, 15) || m_rt.stat_resets != 1 ||
        m_rt.cur_arena != 0 || m_rt.block_off[10] == 0) {
        std::fprintf(stderr, "Multi-arena rolling eviction failed (resets=%u cur=%u blk10=%u)\n",
                     m_rt.stat_resets, m_rt.cur_arena, m_rt.block_off[10]);
        return false;
    }

    // 4) Compiling a prefix block before an already-compiled suffix stops at the suffix and shares it.
    jit_reset_runtime(m_rt, cpu, off);
    if (!jit_compile_block(m_rt, cpu, off, 2)) {
        std::fprintf(stderr, "Suffix compile failed\n");
        return false;
    }
    const uint32_t used_after_suffix = m_rt.code_used;
    if (!jit_compile_block(m_rt, cpu, off, 0) || m_rt.block_off[0] == 0) {
        std::fprintf(stderr, "Prefix compile failed\n");
        return false;
    }
    if ((m_rt.code_used - used_after_suffix) > 100) {
        std::fprintf(stderr, "Prefix block unexpectedly large: %u bytes\n",
                     m_rt.code_used - used_after_suffix);
        return false;
    }
    return true;
}

bool checkIdleTimer3() {
    auto a = std::make_unique<absim::arduboy_t>();
    a->reset();
    auto& cpu = a->core_state.cpu;
    cpu.st(absim::reg::addr::PRR0, 0xff);
    cpu.st(absim::reg::addr::PRR1, 0x00);
    cpu.st(absim::reg::addr::TCCR0B, 0);
    cpu.st(absim::reg::addr::TCCR1B, 0);
    cpu.st(absim::reg::addr::TCCR4B, 0);
    // Replicate Dazzle Dash fast PWM (WGM3=15, TOP=max(3, OCR3A=0)=3, div=8, TIMSK3=0).
    cpu.st(absim::reg::addr::OCR3AH, 0x00);
    cpu.st(absim::reg::addr::OCR3AL, 0x00);
    cpu.st(absim::reg::addr::TIMSK3, 0x00);
    cpu.st(absim::reg::addr::TCCR3A, 0x03);
    cpu.st(absim::reg::addr::TCCR3B, 0x1a);
    cpu.st(absim::reg::addr::TCNT3H, 0x00);
    cpu.st(absim::reg::addr::TCNT3L, 0x00);
    cpu.st(absim::reg::addr::TIFR3, 0xff);

    for (size_t pc = 0; pc < 16384; ++pc)
        cpu.decoded_prog[pc] = {0, 0, 0, absim::INSTR_NOP, 0, 0, 0};
    cpu.decoded_prog[200] = {uint16_t(-201), 0, 0, absim::INSTR_RJMP, 0, 0, 0};
    cpu.merged_prog = cpu.decoded_prog;
    cpu.pc = 0;
    cpu.active = true;

    const uint64_t start = cpu.cycle_count;
    while (cpu.cycle_count - start < 16024) {
        a->cycle();
        if (cpu.peripheral_queue.next().type == absim::PQ_TIMER_SYNC &&
            cpu.peripheral_queue.next_cycle() != UINT64_MAX &&
            cpu.cycle_count - start > 16) {
            std::fprintf(stderr, "Idle Timer3 unexpectedly scheduled PQ_TIMER_SYNC\n");
            return false;
        }
    }
    const uint64_t elapsed_ticks = (cpu.cycle_count - start) / 8;
    const uint8_t tifr = cpu.ld(absim::reg::addr::TIFR3);
    const uint8_t tcnt = cpu.ld(absim::reg::addr::TCNT3L);
    if ((tifr & (absim::reg::bit::TIFR3::TOV3 | absim::reg::bit::TIFR3::OCF3A)) !=
        (absim::reg::bit::TIFR3::TOV3 | absim::reg::bit::TIFR3::OCF3A)) {
        std::fprintf(stderr, "Idle Timer3 did not latch TIFR3 flags on read (0x%02x)\n", tifr);
        return false;
    }
    if (tcnt != uint8_t(elapsed_ticks % 4)) {
        std::fprintf(stderr, "Idle Timer3 TCNT3 mismatch: got=%u expected=%u\n",
                     tcnt, unsigned(elapsed_ticks % 4));
        return false;
    }

    cpu.st(absim::reg::addr::TIFR3, 0xff);
    cpu.peripheral_queue.clear();
    cpu.st(absim::reg::addr::TIMSK3, absim::reg::bit::TIMSK3::TOIE3);
    if (cpu.timer3.next_update_cycle == UINT64_MAX) {
        std::fprintf(stderr, "Timer3 did not schedule next_update_cycle when TIMSK3 enabled\n");
        return false;
    }
    a->cycle();
    if (cpu.peripheral_queue.next().type != absim::PQ_TIMER_SYNC) {
        std::fprintf(stderr, "Timer3 did not resume PQ_TIMER_SYNC when TIMSK3 enabled (next=%d)\n",
                     int(cpu.peripheral_queue.next().type));
        return false;
    }
    return true;
}

}

bool checkSoundPwmBoxFilter() {
    auto cpu = std::make_unique<absim::atmega32u4_t>();
    cpu->reset();
    cpu->st(reg::DDRC, 0x40);
    cpu->sound_buffer.clear();

    // 75% duty cycle across one 320-cycle sample interval (+1000 for 240 cycles, -1000 for 80 cycles -> +500).
    cpu->st(reg::PORTC, 0x40);
    cpu->cycle_count += 240;
    cpu->st(reg::PORTC, 0x00);
    cpu->cycle_count += 80;
    cpu->update_sound();
    if (cpu->sound_buffer.size() != 1 || cpu->sound_buffer[0] != 500)
        return false;

    // Dual push-pull pins (PC6 high / PC7 low for 80 cycles, inverted for 240 cycles -> -1000).
    cpu->st(reg::DDRC, 0xc0);
    cpu->sound_buffer.clear();
    cpu->st(reg::PORTC, 0x40);
    cpu->cycle_count += 80;
    cpu->st(reg::PORTC, 0x80);
    cpu->cycle_count += 240;
    cpu->update_sound();
    return cpu->sound_buffer.size() == 1 && cpu->sound_buffer[0] == -1000;
}

bool checkIdleTimer4NonPwm() {
    auto ref = std::make_unique<absim::arduboy_t>();
    auto fast = std::make_unique<absim::arduboy_t>();
    for (auto* ab : {ref.get(), fast.get()}) {
        ab->reset();
        auto& cpu = ab->core_state.cpu;
        cpu.prog.fill(0); // All NOPs (1 cycle each) across flash
        cpu.decode();
        cpu.st(reg::TCCR4A, 0x00);
        cpu.st(reg::OCR4C, 0x7f);
        cpu.st(reg::TCCR4B, 0x01);
    }
    auto& refCpu = ref->core_state.cpu;
    auto& fastCpu = fast->core_state.cpu;
    refCpu.st(reg::TIMSK4, absim::reg::bit::TIMSK4::TOIE4);
    fastCpu.st(reg::TIMSK4, 0x00);

    size_t fastSteps = 0;
    while (fastCpu.cycle_count < 6000) {
        fast->cycle();
        ++fastSteps;
    }
    size_t refSteps = 0;
    while (refCpu.cycle_count < fastCpu.cycle_count) {
        ref->cycle();
        ++refSteps;
    }

    refCpu.update_all();
    fastCpu.update_all();
    return fastSteps < refSteps &&
           !fastCpu.sound_pwm &&
           fastCpu.timer4.com4a == 0 &&
           fastCpu.cycle_count == refCpu.cycle_count &&
           fastCpu.timer4.tcnt == refCpu.timer4.tcnt &&
           (fastCpu.data[reg::TIFR4] & absim::reg::bit::TIFR4::TOV4) != 0 &&
           fastCpu.data[reg::TIFR4] == refCpu.data[reg::TIFR4];
}

template <size_t N, class InitRegs>
bool runDivModCase(const std::array<uint16_t, N>& words, uint8_t expectedFunc, uint8_t initSreg,
                   int32_t budget, InitRegs&& initRegs) {
    auto ref = std::make_unique<absim::atmega32u4_t>();
    auto fast = std::make_unique<absim::atmega32u4_t>();
    for (auto* cpu : {ref.get(), fast.get()}) {
        cpu->reset();
        cpu->prog.fill(0);
        auto put = [&](size_t pc, uint16_t w) {
            cpu->prog[2 * pc] = uint8_t(w & 0xff);
            cpu->prog[2 * pc + 1] = uint8_t(w >> 8);
        };
        put(0, 0xd00f); // rcall 0x10
        put(1, 0x9588); // sleep (stops execute_merged at pc == 2)
        for (size_t i = 0; i < N; ++i) put(0x10 + i, words[i]);
        cpu->decode();
        cpu->pc = 0;
        cpu->data[absim::reg::addr::SMCR] = 0x1;
        cpu->data[reg::SPL] = 0xf0;
        cpu->data[reg::SPH] = 0x0a;
        cpu->data[reg::SREG] = initSreg;
        initRegs(*cpu);
    }
    fast->merge_instrs();
    if (fast->merged_prog[0x10].func != expectedFunc)
        return false;
    for (int s = 0; s < 5000 && ref->pc != 2; ++s) {
        const auto insn = ref->decoded_prog[ref->pc];
        ref->cycle_count += absim::INSTR_MAP[insn.func](*ref, insn);
    }
    for (int s = 0; s < 5000 && fast->pc != 2 && fast->active; ++s)
        fast->execute_merged(budget);
    if (ref->pc != 2 || fast->pc != 2 || fast->cycle_count != ref->cycle_count) {
        std::fprintf(stderr, "DivMod timing/pc mismatch (N=%zu, sreg=%02x, b=%d): ref pc=%x cyc=%llu, fast pc=%x cyc=%llu\n",
            N, initSreg, budget, ref->pc, (unsigned long long)ref->cycle_count,
            fast->pc, (unsigned long long)fast->cycle_count);
        return false;
    }
    if (fast->data[reg::SREG] != ref->data[reg::SREG] ||
        fast->data[reg::SPL] != ref->data[reg::SPL] ||
        fast->data[reg::SPH] != ref->data[reg::SPH]) {
        std::fprintf(stderr, "DivMod SREG/SP mismatch (N=%zu, init=%02x, b=%d): ref sreg=%02x sp=%02x%02x, fast sreg=%02x sp=%02x%02x\n",
            N, initSreg, budget, ref->data[reg::SREG], ref->data[reg::SPH], ref->data[reg::SPL],
            fast->data[reg::SREG], fast->data[reg::SPH], fast->data[reg::SPL]);
        return false;
    }
    for (int r = 0; r < 32; ++r) {
        if (fast->data[r] != ref->data[r]) {
            std::fprintf(stderr, "DivMod r%d mismatch (N=%zu, b=%d): ref=%02x fast=%02x\n",
                r, N, budget, ref->data[r], fast->data[r]);
            return false;
        }
    }
    return true;
}

bool checkDivModExact() {
    static constexpr std::array<uint16_t, 34> udivmodsi4 = {
        0xe2a1, 0x2e1a, 0x1baa, 0x1bbb, 0x01fd, 0xc00d,
        0x1faa, 0x1fbb, 0x1fee, 0x1fff, 0x17a2, 0x07b3, 0x07e4, 0x07f5, 0xf020,
        0x1ba2, 0x0bb3, 0x0be4, 0x0bf5, 0x1f66, 0x1f77, 0x1f88, 0x1f99,
        0x941a, 0xf769, 0x9560, 0x9570, 0x9580, 0x9590,
        0x019b, 0x01ac, 0x01bd, 0x01cf, 0x9508,
    };
    static constexpr std::array<uint16_t, 20> udivmodhi4 = {
        0x1baa, 0x1bbb, 0xe151, 0xc007,
        0x1faa, 0x1fbb, 0x17a6, 0x07b7, 0xf010,
        0x1ba6, 0x0bb7, 0x1f88, 0x1f99, 0x955a, 0xf7a9,
        0x9580, 0x9590, 0x01bc, 0x01cd, 0x9508,
    };

    const std::array<std::pair<uint32_t, uint32_t>, 9> cases32 = {{
        {0u, 0u}, {0x12345678u, 0u}, {0u, 0x80000001u}, {0xffffffffu, 1u},
        {0x10000000u, 1u}, {0xffffffffu, 0x80000001u}, {0x12345678u, 0x9abcu},
        {0x80000000u, 2u}, {1u, 0xffffffffu},
    }};
    for (const auto [dividend, divisor] : cases32) {
        auto init32 = [&](absim::atmega32u4_t& cpu) {
            for (int r = 0; r < 4; ++r) {
                cpu.data[22 + r] = uint8_t((dividend >> (8 * r)) & 0xff);
                cpu.data[18 + r] = uint8_t((divisor >> (8 * r)) & 0xff);
            }
        };
        if (!runDivModCase(udivmodsi4, absim::INSTR_MERGED_UDIVMOD4, 0x00, 2000, init32) ||
            !runDivModCase(udivmodsi4, absim::INSTR_MERGED_UDIVMOD4, 0xff, 2000, init32) ||
            !runDivModCase(udivmodsi4, absim::INSTR_MERGED_UDIVMOD4, 0x83, 10, init32))
            return false;
    }

    const std::array<std::pair<uint16_t, uint16_t>, 10> cases16 = {{
        {0u, 0u}, {0x1234u, 0u}, {0u, 0xffffu}, {0xffffu, 1u}, {0x1000u, 1u},
        {0xffffu, 7u}, {0xffffu, 0x8001u}, {0x1234u, 0x56u}, {0x8000u, 2u}, {1u, 0xffffu},
    }};
    for (const auto [dividend, divisor] : cases16) {
        auto init16 = [&](absim::atmega32u4_t& cpu) {
            cpu.data[24] = uint8_t(dividend & 0xff);
            cpu.data[25] = uint8_t(dividend >> 8);
            cpu.data[22] = uint8_t(divisor & 0xff);
            cpu.data[23] = uint8_t(divisor >> 8);
        };
        if (!runDivModCase(udivmodhi4, absim::INSTR_MERGED_UDIVMOD2, 0x00, 1000, init16) ||
            !runDivModCase(udivmodhi4, absim::INSTR_MERGED_UDIVMOD2, 0xff, 1000, init16) ||
            !runDivModCase(udivmodhi4, absim::INSTR_MERGED_UDIVMOD2, 0x83, 10, init16))
            return false;
    }
    return true;
}

bool checkDisplayMuxZeroFilter() {
    auto ab = std::make_unique<absim::arduboy_t>();
    ab->reset();
    ab->core_state.cpu.prog.fill(0);
    ab->core_state.cpu.decode();
    ab->core_state.oled_in_reset = false;
    auto& disp = ab->peripherals.display;
    disp.type = absim::display_t::SSD1306;
    disp.enable_filter = true;
    disp.send_command(0x8d);
    disp.send_command(0x14);
    disp.send_command(0xaf);
    disp.send_command(0x20);
    disp.send_command(0x00);

    auto writePlane = [&](uint8_t val) {
        for (int i = 0; i < 1024; ++i)
            ab->core_state.oled.ram[i] = (i & 1) ? val : uint8_t(val ^ 0xff);
    };
    auto stepRows = [&](int rows) {
        const uint64_t rowPs = disp.ps_per_clk * disp.cycles_per_row;
        for (int i = 0; i < rows; ++i)
            ab->advance(absim::arduboy_t::PS_BUFFER + rowPs);
    };

    // Simulate DefenderFX 3-plane scan with 0xA8 0x3F / 0x00 row reset.
    std::array<uint8_t, 8192> snap{};
    for (int cycle = 0; cycle < 17; ++cycle) {
        const uint8_t v = (cycle % 3 == 0) ? 0xff : (cycle % 3 == 1 ? 0x0f : 0x00);
        writePlane(v);
        disp.send_command(0xa8);
        disp.send_command(0x3f);
        for (int s = 0; s < 256; ++s) {
            const uint8_t prev = disp.row;
            stepRows(1);
            if (disp.row < prev) break;
        }
        // Switch mux=0 between planes: once row reaches 0, it stays pinned without vsync.
        disp.send_command(0xa8);
        disp.send_command(0x00);
        for (int s = 0; s < 32 && disp.row != 0; ++s)
            stepRows(1);
        if (disp.row != 0) {
            std::fprintf(stderr, "DisplayMuxZero failed to reach row 0 at cyc %d: row=%u\n",
                cycle, disp.row);
            return false;
        }
        stepRows(4);
        if (disp.row != 0 || disp.vsync) {
            std::fprintf(stderr, "DisplayMuxZero mux=0 failed at cyc %d: row=%u vsync=%d\n",
                cycle, disp.row, int(disp.vsync));
            return false;
        }
        disp.send_command(0xa8);
        disp.send_command(0x3f);
        if (cycle == 4) snap = disp.filtered_pixels;
        if (cycle == 16 && snap != disp.filtered_pixels) {
            uint8_t maxDiff = 0;
            for (size_t i = 0; i < snap.size(); ++i) {
                const int d = int(snap[i]) - int(disp.filtered_pixels[i]);
                maxDiff = std::max<uint8_t>(maxDiff, uint8_t(d < 0 ? -d : d));
            }
            std::fprintf(stderr, "DisplayMuxZero diff c4 vs c7 maxDiff=%u anyLit0=%u\n",
                maxDiff, snap[1]);
            return false;
        }
    }
    const bool anyLit = std::any_of(
        disp.filtered_pixels.begin(), disp.filtered_pixels.end(), [](uint8_t p) { return p > 0; });
    const bool anyPartial = std::any_of(
        disp.filtered_pixels.begin(), disp.filtered_pixels.end(), [](uint8_t p) { return p > 0 && p < 255; });
    if (!anyLit || !anyPartial) {
        std::fprintf(stderr, "DisplayMuxZero pixels: anyLit=%d anyPartial=%d p0=%u p1=%u\n",
            int(anyLit), int(anyPartial), disp.filtered_pixels[0], disp.filtered_pixels[1]);
    }
    return anyLit && anyPartial;
}

int main() {
    auto check = [](const char* name, bool ok) {
        if (!ok) std::fprintf(stderr, "FAIL: %s\n", name);
        return ok;
    };
    return check("MicrosWait", checkMicrosWait()) &&
        check("ArduinoDelay", checkArduinoDelay()) &&
        check("ArduinoYieldDelay16", checkArduinoYieldDelay16()) &&
        check("SpiWait00", checkSpiWait(false, false)) &&
        check("SpiWait10", checkSpiWait(true, false)) &&
        check("SpiWait01", checkSpiWait(false, true)) &&
        check("SpiWait11", checkSpiWait(true, true)) &&
        check("SpiBatch", checkSpiBatch()) &&
        check("FxD1", checkFxPins(reg::PORTD, 1 << 1)) &&
        check("FxE2", checkFxPins(reg::PORTE, 1 << 2)) &&
        check("Fused", checkFusedInstructions()) &&
        check("DelayCall", checkDelayCall()) &&
        check("ReturnStack", checkReturnStack()) &&
        check("DisplayLoops", checkDisplayLoops()) &&
        check("BatchInlineIoAndIrq", checkBatchInlineIoAndIrq()) &&
        check("IdleTimer1", checkIdleTimer1()) &&
        check("IdleTimer3", checkIdleTimer3()) &&
        check("IdleTimer4", checkIdleTimer4()) &&
        check("IdleTimer4NonPwm", checkIdleTimer4NonPwm()) &&
        check("SoundPwmBoxFilter", checkSoundPwmBoxFilter()) &&
        check("DivModExact", checkDivModExact()) &&
        check("DisplayMuxZeroFilter", checkDisplayMuxZeroFilter()) &&
        check("XtensaJitCompiler", checkXtensaJitCompiler()) ? 0 : 1;
}
