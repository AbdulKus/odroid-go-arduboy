#include "absim.hpp"

// possibilities:

//     mul[s][u] + movw rN, r0 + eor r1, r1
//     ldi rC, N + add rA, rA + adc rB, rB + dec RC + brne
//     ldi rC, N + lsr rA, rA + ror rB, rB + dec RC + brne
//     ldi rC, N + asr rA, rA + ror rB, rB + dec RC + brne
//     lsr rN + lsr rN + ... + lsr rN
//     add + adc
//     subi + sbci
//     eor rN, rN + dec rN
//     and rN, rN + brbc
//     and rN, rN + brbs
//     and + or (used in sprite drawing)
//     sbiw rN, 1 + brne

// ymask:
//     ldi  r19, 0x01
//     sbrc r22, 1
//     ldi  r19, 0x04
//     sbrc r22, 0
//     add  r19, r19
//     sbrc r22, 2
//     swap r19

namespace absim
{

#ifdef ARDUGO_EMBEDDED
static uint16_t micros_timer_address(const atmega32u4_t& cpu, size_t pc)
{
    constexpr std::array<uint16_t, 19> pattern = {
        0xb73f, 0x94f8, 0x9180, 0, 0x9190, 0, 0x91a0, 0, 0x91b0, 0,
        0xb526, 0x9ba8, 0xc005, 0x3f2f, 0xf019, 0x9601, 0x1da1, 0x1db1, 0xbf3f,
    };
    if(pc + pattern.size() > cpu.prog.size() / 2) return 0;
    const auto word = [&](size_t offset) {
        return uint16_t(cpu.prog[2 * (pc + offset)] | (uint16_t(cpu.prog[2 * (pc + offset) + 1]) << 8));
    };
    const uint16_t base = word(3);
    if(base < 256 || size_t(base) + 4 > cpu.data.size()) return 0;
    for(size_t i = 0; i < pattern.size(); ++i)
    {
        if(pattern[i] != 0)
        {
            if(word(i) != pattern[i]) return 0;
        }
        else if(word(i) != base + (i - 3) / 2)
            return 0;
    }
    return base;
}

static uint16_t micros_wait_address(const atmega32u4_t& cpu, size_t pc)
{
    // LDS r20, deadline; LDS r21, deadline+1; MOVW r18, r20; SUB/SBC;
    // SBRC r19, 7; RJMP back to CALL. Both loads must address ordinary SRAM.
    constexpr std::array<uint16_t, 9> pattern = {
        0x9140, 0, 0x9150, 0, 0x019a, 0x1b26, 0x0b37, 0xfd37, 0xcff5,
    };
    pc += 2;
    if(pc + pattern.size() > cpu.prog.size() / 2) return 0;
    const auto word = [&](size_t offset) {
        const auto* bytes = cpu.prog.data() + 2 * (pc + offset);
        return uint16_t(bytes[0] | uint16_t(bytes[1]) << 8);
    };
    const auto address = word(1);
    if(address < 256 || size_t(address) + 2 > cpu.data.size() || word(3) != address + 1)
        return 0;
    for(size_t i = 0; i < pattern.size(); ++i)
        if(pattern[i] && word(i) != pattern[i]) return 0;
    return address;
}

static bool is_arduino_delay_poll(const atmega32u4_t& cpu, size_t call_pc)
{
    // The Arduino delay() loop tests the millisecond count in r12-r15,
    // calls micros(), then compares elapsed microseconds with 1000.
    constexpr std::array<uint16_t, 5> head = {
        0x14c1, 0x04d1, 0x04e1, 0x04f1, 0xf0f1,
    };
    constexpr std::array<uint16_t, 11> tail = {
        0x01dc, 0x01cb, 0x1988, 0x0999, 0x09aa, 0x09bb,
        0x3e88, 0x4093, 0x05a1, 0x05b1, 0xf370,
    };
    if(call_pc < head.size() || call_pc + 2 + tail.size() > cpu.prog.size() / 2)
        return false;
    const auto matches = [&](size_t pc, const auto& words) {
        for(const uint16_t word : words)
        {
            const auto* bytes = cpu.prog.data() + 2 * pc++;
            if(word != (bytes[0] | uint16_t(bytes[1]) << 8)) return false;
        }
        return true;
    };
    return matches(call_pc - head.size(), head) && matches(call_pc + 2, tail);
}

static bool is_arduino_delay_poll_legacy(const atmega32u4_t& cpu, size_t call_pc)
{
    // Older avr-gcc delay(): `cp r12,r1..cpc r15,r1; breq exit; call micros;
    // sub r22,r8; sbc r23,r9; sbc r24,r10; sbc r25,r11; cpi r22,0xe8; sbci r23,3;
    // cpc r24,r1; cpc r25,r1; brcs head`.
    constexpr std::array<uint16_t, 4> head = {
        0x14c1, 0x04d1, 0x04e1, 0x04f1,
    };
    constexpr std::array<uint16_t, 8> tail = {
        0x1968, 0x0979, 0x098a, 0x099b,
        0x3e68, 0x4073, 0x0581, 0x0591,
    };
    const size_t prog_words = cpu.decoded_prog.size();
    if(call_pc < 5 || call_pc + 2 + tail.size() >= prog_words)
        return false;
    const auto word = [&](size_t offset) {
        const auto* bytes = cpu.prog.data() + 2 * offset;
        return uint16_t(bytes[0] | uint16_t(bytes[1]) << 8);
    };
    for(size_t i = 0; i < head.size(); ++i)
        if(word(call_pc - 5 + i) != head[i]) return false;
    const auto& br_exit = cpu.decoded_prog[call_pc - 1];
    if(br_exit.func != INSTR_BRBS || br_exit.src != 1 || int16_t(br_exit.word) <= 0)
        return false;
    for(size_t i = 0; i < tail.size(); ++i)
        if(word(call_pc + 2 + i) != tail[i]) return false;
    const auto& br_back = cpu.decoded_prog[call_pc + 2 + tail.size()];
    return br_back.func == INSTR_BRBS && br_back.src == 0 &&
           (call_pc + 2 + tail.size() + 1 + size_t(int16_t(br_back.word))) == (call_pc - 5);
}

struct delay16_poll_t
{
    uint8_t loop_cycles = 0;
    uint8_t start_reg = 0;
};

static delay16_poll_t match_arduino_delay16_poll(
    const atmega32u4_t& cpu, size_t call_pc, bool compact_micros)
{
    const auto prog_words = cpu.decoded_prog.size();
    const auto word = [&](size_t offset) {
        const auto* bytes = cpu.prog.data() + 2 * offset;
        return uint16_t(bytes[0] | uint16_t(bytes[1]) << 8);
    };
    if(call_pc + 8 >= prog_words || call_pc < 5) return {};

    // Check optional yield() stub call (`call <ret>` / `rcall <ret>`) immediately before `call micros`.
    size_t head_end = call_pc;
    uint8_t yield_cycles = 0;
    if(call_pc >= 7 && cpu.decoded_prog[call_pc - 2].func == INSTR_CALL)
    {
        const uint16_t y = cpu.decoded_prog[call_pc - 2].word;
        if(y < prog_words && cpu.decoded_prog[y].func == INSTR_RET)
        {
            head_end = call_pc - 2;
            yield_cycles = 8;
        }
    }
    else if(call_pc >= 6 && cpu.decoded_prog[call_pc - 1].func == INSTR_RCALL)
    {
        const uint16_t y = uint16_t(call_pc + int16_t(cpu.decoded_prog[call_pc - 1].word)) & 0x3fff;
        if(y < prog_words && cpu.decoded_prog[y].func == INSTR_RET)
        {
            head_end = call_pc - 1;
            yield_cycles = 7;
        }
    }
    if(head_end < 5) return {};
    const size_t head_pc = head_end - 5;
    if(word(head_pc + 0) != 0x14c1 || word(head_pc + 1) != 0x04d1 ||
       word(head_pc + 2) != 0x04e1 || word(head_pc + 3) != 0x04f1)
        return {};
    const auto& br_exit = cpu.decoded_prog[head_pc + 4];
    if(br_exit.func != INSTR_BRBS || br_exit.src != 1 || int16_t(br_exit.word) <= 0)
        return {};
    const size_t exit_pc = head_pc + 5 + size_t(int16_t(br_exit.word));
    if(exit_pc > prog_words || call_pc + 6 > exit_pc) return {};

    // Match 16-bit elapsed check after `call micros`:
    //   sub r22, rS ; sbc r23, rS+1 ; cpi r22, 0xe8 ; [ldi rT, 3;] (cpc r23, rT | sbci r23, 3) ; (brcs head | brcc +k ; rjmp head)
    size_t p = call_pc + 2;
    const auto& s0 = cpu.decoded_prog[p];
    const auto& s1 = cpu.decoded_prog[p + 1];
    const auto& s2 = cpu.decoded_prog[p + 2];
    if(s0.func != INSTR_SUB || s0.dst != 22 ||
       s1.func != INSTR_SBC || s1.dst != 23 || s1.src != uint8_t(s0.src + 1))
        return {};
    const uint8_t start_reg = s0.src;
    if(start_reg >= 12 && start_reg <= 15) return {};
    if(start_reg + 1 >= 12 && start_reg <= 15) return {};
    if(start_reg >= 18 && start_reg <= 27) return {};
    if(s2.func != INSTR_CPI || s2.dst != 22 || s2.src != 0xe8) return {};
    p += 3;
    uint8_t post_cycles = 3;

    uint8_t const3_reg = 0xff;
    if(p < exit_pc && cpu.decoded_prog[p].func == INSTR_LDI && cpu.decoded_prog[p].src == 3)
    {
        const3_reg = cpu.decoded_prog[p].dst;
        ++p;
        ++post_cycles;
    }
    if(p >= exit_pc) return {};
    const auto& cmp_hi = cpu.decoded_prog[p];
    if(cmp_hi.func == INSTR_SBCI && cmp_hi.dst == 23 && cmp_hi.src == 3)
    {
        ++p;
        ++post_cycles;
    }
    else if(cmp_hi.func == INSTR_CPC && cmp_hi.dst == 23)
    {
        bool ok = (cmp_hi.src == const3_reg);
        if(!ok)
        {
            // Support hoisted `ldi rX, 3` before head_pc into a callee-saved register.
            const uint8_t rt = cmp_hi.src;
            const bool callee_saved = rt < 18 || rt >= 28;
            if(callee_saved && !(rt >= 12 && rt <= 15) &&
               rt != start_reg && rt != uint8_t(start_reg + 1))
            {
                for(size_t b = 1; b <= 10 && head_pc >= b; ++b)
                {
                    const auto& prev = cpu.decoded_prog[head_pc - b];
                    if(prev.func == INSTR_LDI && prev.dst == rt && prev.src == 3)
                    {
                        ok = true;
                        break;
                    }
                    if(prev.dst == rt || prev.func == INSTR_CALL || prev.func == INSTR_RCALL ||
                       prev.func == INSTR_ICALL || prev.func == INSTR_RET || prev.func == INSTR_RETI)
                        break;
                }
            }
        }
        if(!ok) return {};
        ++p;
        ++post_cycles;
    }
    else
        return {};

    if(p >= exit_pc) return {};
    const auto& br_back = cpu.decoded_prog[p];
    if(br_back.func == INSTR_BRBS && br_back.src == 0)
    {
        const size_t target = p + 1 + size_t(int16_t(br_back.word));
        if(target != head_pc) return {};
        post_cycles += 2;
    }
    else if(br_back.func == INSTR_BRBC && br_back.src == 0 && p + 1 < exit_pc &&
            cpu.decoded_prog[p + 1].func == INSTR_RJMP)
    {
        const size_t skip_target = p + 1 + size_t(int16_t(br_back.word));
        const size_t rjmp_target = p + 2 + size_t(int16_t(cpu.decoded_prog[p + 1].word));
        if(skip_target < p + 2 || rjmp_target != head_pc) return {};
        post_cycles += 3;
    }
    else
        return {};

    const uint8_t micros_cycles = compact_micros ? 45 : 47;
    const uint32_t total = 5u + yield_cycles + micros_cycles + post_cycles;
    if(total < 16 || total > 255) return {};
    return {uint8_t(total), start_reg};
}

static bool merge_spi_poll(avr_instr_t& first, const std::vector<avr_instr_t>& program, size_t pc)
{
    using namespace reg::addr;
    if(first.func != INSTR_MERGED_IN || (first.src != SPDR - 32 && first.src != SPSR - 32))
        return false;
    const bool read_data = first.src == SPDR - 32;
    const size_t status_pc = pc + read_data;
    if(status_pc + 2 >= program.size()) return false;
    const auto& status = program[status_pc];
    const auto& skip = program[status_pc + 1];
    const auto& jump = program[status_pc + 2];
    if(status.func != INSTR_IN || status.src != SPSR - 32 ||
       skip.func != INSTR_SBRS || skip.dst != status.dst || skip.src != reg::bit::SPSR::SPIF ||
       jump.func != INSTR_RJMP || int16_t(jump.word) != -(3 + int(read_data)))
        return false;
    first.func = INSTR_MERGED_SPI_POLL;
    first.m0 = status.dst;
    return true;
}

static void merge_delay_call(avr_instr_t& call, const std::vector<avr_instr_t>& program, size_t pc)
{
    const bool relative = call.func == INSTR_RCALL;
    if(!relative && call.func != INSTR_CALL) return;
    const uint16_t target = relative ? uint16_t(pc + 1 + int16_t(call.word)) : call.word;
    unsigned cycles = relative ? 3 : 4;
    bool reads_flash = false;
    for(size_t step = 0; step < 8 && target + step < program.size(); ++step)
    {
        const auto& op = program[target + step];
        if(op.func == INSTR_RET && step >= 3)
        {
            call.func = INSTR_MERGED_DELAY_CALL;
            call.m0 = relative ? 1 : 2;
            call.m1 = uint8_t(cycles + 4);
            call.m2 = reads_flash;
            return;
        }
        if(op.func == INSTR_NOP) ++cycles;
        else if(op.func == INSTR_RJMP && op.word == 0) cycles += 2;
        else if(op.func == INSTR_LPM && op.word != 1 && op.dst == 0 && !reads_flash)
        {
            reads_flash = true;
            cycles += 3;
        }
        else return;
    }
}

static bool merge_display_loop(const atmega32u4_t& cpu, avr_instr_t& first, size_t pc)
{
    // Arduboy2's SPI-paced display upload and an ArduboyFX shifted bitmap
    // loop. Match all instructions, including the backwards branch.
    constexpr std::array<uint16_t, 9> oled = {
        0x8000, 0xbc0e, 0x1181, 0x2c01, 0x9711, 0xfda0, 0xcffd, 0x9201, 0xf7b9,
    };
    constexpr std::array<uint16_t, 13> blend = {
        0xb58e, 0xbc1e, 0x9f82, 0x906c, 0x226c, 0x2860, 0x926d,
        0x8060, 0x226d, 0x2861, 0x9261, 0x947a, 0xf799,
    };
    constexpr std::array<uint16_t, 13> blend_alt = {
        0xb50e, 0xbc1e, 0x9f04, 0x909c, 0x209c, 0x2890, 0x929d,
        0x8090, 0x209d, 0x2891, 0x9291, 0x94ea, 0xf799,
    };
    constexpr std::array<uint16_t, 47> masked_blend = {
        0xb40e, 0xbc1e, 0xfc93, 0x2606, 0x2f86, 0xfc90, 0x2d80, 0xfc92,
        0x2400, 0x9e03, 0x01a0, 0xfa94, 0xf436, 0x95c8, 0x95c8, 0x2411,
        0xb40e, 0xbc1e, 0xfc90, 0x2e08, 0x9e03, 0x3000, 0xf034, 0x918c,
        0xfe91, 0x2748, 0x2140, 0x2748, 0x934c, 0x58a0, 0x4fbf, 0xfe97,
        0xc006, 0x918c, 0xfe91, 0x2758, 0x2151, 0x2758, 0x935c, 0x2411,
        0xfc95, 0x58a1, 0xfe95, 0x57af, 0x09b1, 0x959a, 0xf689,
    };
    const auto matches = [&](const auto& words) {
        if(pc + words.size() > cpu.prog.size() / 2) return false;
        const auto* bytes = cpu.prog.data() + 2 * pc;
        for(const auto word : words)
        {
            if(word != (uint16_t(bytes[1]) << 8 | bytes[0])) return false;
            bytes += 2;
        }
        return true;
    };

    uint8_t operation;
    if(first.func == INSTR_MERGED_LDD_Z && first.src == 0 && first.dst == 0 && matches(oled))
        operation = INSTR_MERGED_OLED_SEND;
    else if(first.func == INSTR_MERGED_IN && first.src == reg::addr::SPDR - 32 && first.dst == 24 && matches(blend))
    {
        first.m1 = 0;
        operation = INSTR_MERGED_FX_BLEND;
    }
    else if(first.func == INSTR_MERGED_IN && first.src == reg::addr::SPDR - 32 && first.dst == 16 && matches(blend_alt))
    {
        first.m1 = 1;
        operation = INSTR_MERGED_FX_BLEND;
    }
    else if(first.func == INSTR_MERGED_IN && first.src == reg::addr::SPDR - 32 && first.dst == 0 && matches(masked_blend))
        operation = INSTR_MERGED_FX_MASKED_BLEND;
    else return false;
    first.m2 = first.func;
    first.func = operation;
    return true;
}

static bool merge_divmod(const atmega32u4_t& cpu, avr_instr_t& first, size_t pc)
{
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
    const auto matches = [&](const auto& words) {
        if(pc + words.size() > cpu.prog.size() / 2) return false;
        const auto* bytes = cpu.prog.data() + 2 * pc;
        for(const auto word : words)
        {
            if(word != (uint16_t(bytes[1]) << 8 | bytes[0])) return false;
            bytes += 2;
        }
        return true;
    };

    uint8_t op = 0;
    if(first.func == INSTR_LDI && first.dst == 26 && first.src == 33 && matches(udivmodsi4))
        op = INSTR_MERGED_UDIVMOD4;
    else if((first.func == INSTR_SUB || first.func == INSTR_MERGED_SUB_SBC) &&
            first.dst == 26 && first.src == 26 && matches(udivmodhi4))
        op = INSTR_MERGED_UDIVMOD2;
    else
        return false;

    first.m2 = cpu.decoded_prog[pc].func;
    first.func = op;
    return true;
}
#endif

static uint32_t instr_is_delay(atmega32u4_t const& cpu, size_t n)
{
    auto i = cpu.decoded_prog[n];
    switch(i.func)
    {
    case INSTR_NOP:
        return 1;
    case INSTR_RJMP:
    {
        if(i.word == 0)
            return 2;
        break;
    }
    default:
        return 0;
    }
    return 0;
}

#if defined(ARDUGO_EMBEDDED) && defined(ESP_PLATFORM)
template<size_t Slots>
void atmega32u4_t::instruction_cache_t<Slots>::fill(
    uint16_t block, size_t slot, const avr_instr_t* program)
{
    memcpy(entries[slot].data(), program + block * INSTR_CACHE_BLOCK, sizeof(entries[slot]));
    tags[slot] = block;
}

template struct atmega32u4_t::instruction_cache_t<atmega32u4_t::MERGED_CACHE_SLOTS>;
template struct atmega32u4_t::instruction_cache_t<atmega32u4_t::DECODED_CACHE_SLOTS>;
#endif

void atmega32u4_t::merge_instrs()
{
#if defined(ARDUGO_EMBEDDED) && defined(ESP_PLATFORM)
    merged_cache.tags.fill(UINT16_MAX);
    decoded_cache.tags.fill(UINT16_MAX);
#endif
    memcpy(merged_prog.data(), decoded_prog.data(), array_bytes(merged_prog));

    for(auto& i : merged_prog)
    {
        switch(i.func)
        {
        case INSTR_OUT     : i.func = INSTR_MERGED_OUT     ; break;
        case INSTR_IN      : i.func = INSTR_MERGED_IN      ; break;
        case INSTR_LDS     : i.func = INSTR_MERGED_LDS     ; break;
        case INSTR_STS     : i.func = INSTR_MERGED_STS     ; break;
        case INSTR_LDD_Y   : i.func = INSTR_MERGED_LDD_Y   ; break;
        case INSTR_LDD_Z   : i.func = INSTR_MERGED_LDD_Z   ; break;
        case INSTR_STD_Y   : i.func = INSTR_MERGED_STD_Y   ; break;
        case INSTR_STD_Z   : i.func = INSTR_MERGED_STD_Z   ; break;
        case INSTR_LD_ST   : i.func = INSTR_MERGED_LD_ST   ; break;
        case INSTR_LD_X    : i.func = INSTR_MERGED_LD_X    ; break;
        case INSTR_LD_Y    : i.func = INSTR_MERGED_LD_Y    ; break;
        case INSTR_LD_Z    : i.func = INSTR_MERGED_LD_Z    ; break;
        case INSTR_LD_X_INC: i.func = INSTR_MERGED_LD_X_INC; break;
        case INSTR_LD_Y_INC: i.func = INSTR_MERGED_LD_Y_INC; break;
        case INSTR_LD_Z_INC: i.func = INSTR_MERGED_LD_Z_INC; break;
        case INSTR_LD_X_DEC: i.func = INSTR_MERGED_LD_X_DEC; break;
        case INSTR_LD_Y_DEC: i.func = INSTR_MERGED_LD_Y_DEC; break;
        case INSTR_LD_Z_DEC: i.func = INSTR_MERGED_LD_Z_DEC; break;
        case INSTR_ST_X    : i.func = INSTR_MERGED_ST_X    ; break;
        case INSTR_ST_Y    : i.func = INSTR_MERGED_ST_Y    ; break;
        case INSTR_ST_Z    : i.func = INSTR_MERGED_ST_Z    ; break;
        case INSTR_ST_X_INC: i.func = INSTR_MERGED_ST_X_INC; break;
        case INSTR_ST_Y_INC: i.func = INSTR_MERGED_ST_Y_INC; break;
        case INSTR_ST_Z_INC: i.func = INSTR_MERGED_ST_Z_INC; break;
        case INSTR_ST_X_DEC: i.func = INSTR_MERGED_ST_X_DEC; break;
        case INSTR_ST_Y_DEC: i.func = INSTR_MERGED_ST_Y_DEC; break;
        case INSTR_ST_Z_DEC: i.func = INSTR_MERGED_ST_Z_DEC; break;
        case INSTR_SBI     : i.func = INSTR_MERGED_SBI     ; break;
        case INSTR_CBI     : i.func = INSTR_MERGED_CBI     ; break;
        case INSTR_SBIS    : i.func = INSTR_MERGED_SBIS    ; break;
        case INSTR_SBIC    : i.func = INSTR_MERGED_SBIC    ; break;
        default: break;
        }
    }

    for(size_t n = 0; n + 1 < merged_prog.size(); ++n)
    {
        auto& i0 = merged_prog[n + 0];
        auto  i1 = decoded_prog[n + 1];

#ifdef ARDUGO_EMBEDDED
        if(merge_spi_poll(i0, decoded_prog, n))
            continue;
        if(merge_display_loop(*this, i0, n))
            continue;
        if(merge_divmod(*this, i0, n))
            continue;
        merge_delay_call(i0, decoded_prog, n);
        if((i0.func == INSTR_SBRC || i0.func == INSTR_SBRS) && i1.func == INSTR_RJMP)
        {
            i0.m0 = i0.func == INSTR_SBRS;
            i0.word = i1.word;
            i0.func = INSTR_MERGED_SKIP_RJMP;
            continue;
        }
        if(i0.func == INSTR_ADD && i1.func == INSTR_ADC &&
           n + 2 < decoded_prog.size() && i1.dst == i0.dst + 1 &&
           i1.src != i0.dst && i1.src != i1.dst)
        {
            const auto& i2 = decoded_prog[n + 2];
            if(i2.func == INSTR_ADC && i2.dst == i1.dst + 1 && i2.src == i1.src)
            {
                i0.m0 = i1.src;
                i0.func = INSTR_MERGED_ADD_ADC2;
                continue;
            }
        }
        if(i0.func == INSTR_MERGED_IN && i0.dst == 19 && i0.src == 0x3f)
        {
            const uint16_t timer_address = micros_timer_address(*this, n);
            if(timer_address != 0)
            {
                i0.func = INSTR_MERGED_MICROS_TIMER;
                i0.word = timer_address;
                continue;
            }
        }

        // The standard Arduino micros() implementation scales its timer and
        // overflow counter by four with a two-iteration register shift. Start
        // after the first MOV, which can execute in the interrupt-enable delay.
        constexpr std::array<uint8_t, 32> micros_result = {
            0xa9, 0x2f, 0x98, 0x2f, 0x88, 0x27, 0x82, 0x0f,
            0x91, 0x1d, 0xa1, 0x1d, 0xb1, 0x1d, 0xbc, 0x01, 0xcd, 0x01,
            0x42, 0xe0, 0x66, 0x0f, 0x77, 0x1f, 0x88, 0x1f, 0x99, 0x1f,
            0x4a, 0x95, 0xd1, 0xf7,
        };
        const size_t address = n * 2;
        if(i0.func == INSTR_MOV && i0.dst == 26 && i0.src == 25 &&
           address + micros_result.size() <= prog.size() &&
           memcmp(prog.data() + address, micros_result.data(), micros_result.size()) == 0)
        {
            i0.func = INSTR_MERGED_MICROS_RESULT;
            i0.word = 0;
            continue;
        }
        constexpr std::array<uint8_t, 28> micros_result_compact = {
            0x78, 0x2f, 0x89, 0x2f, 0x9a, 0x2f,
            0x62, 0x0f, 0x71, 0x1d, 0x81, 0x1d, 0x91, 0x1d,
            0x42, 0xe0, 0x66, 0x0f, 0x77, 0x1f, 0x88, 0x1f, 0x99, 0x1f,
            0x4a, 0x95, 0xd1, 0xf7,
        };
        if(i0.func == INSTR_MOV && i0.dst == 23 && i0.src == 24 &&
           address + micros_result_compact.size() <= prog.size() &&
           memcmp(prog.data() + address, micros_result_compact.data(), micros_result_compact.size()) == 0)
        {
            i0.func = INSTR_MERGED_MICROS_RESULT;
            i0.word = 1;
            continue;
        }
        constexpr std::array<uint8_t, 32> micros_result_legacy_postadd = {
            0xa9, 0x2f, 0x98, 0x2f, 0x88, 0x27, 0xbc, 0x01, 0xcd, 0x01,
            0x62, 0x0f, 0x71, 0x1d, 0x81, 0x1d, 0x91, 0x1d,
            0x42, 0xe0, 0x66, 0x0f, 0x77, 0x1f, 0x88, 0x1f, 0x99, 0x1f,
            0x4a, 0x95, 0xd1, 0xf7,
        };
        if(i0.func == INSTR_MOV && i0.dst == 26 && i0.src == 25 &&
           address + micros_result_legacy_postadd.size() <= prog.size() &&
           memcmp(prog.data() + address, micros_result_legacy_postadd.data(), micros_result_legacy_postadd.size()) == 0)
        {
            i0.func = INSTR_MERGED_MICROS_RESULT;
            i0.word = 2;
            continue;
        }
        constexpr std::array<uint8_t, 32> micros_result_legacy_preadd = {
            0xa9, 0x2f, 0x98, 0x2f, 0x88, 0x27,
            0x82, 0x0f, 0x91, 0x1d, 0xa1, 0x1d, 0xb1, 0x1d,
            0xbc, 0x01, 0xcd, 0x01,
            0x42, 0xe0, 0x66, 0x0f, 0x77, 0x1f, 0x88, 0x1f, 0x99, 0x1f,
            0x4a, 0x95, 0xd1, 0xf7,
        };
        if(i0.func == INSTR_MOV && i0.dst == 26 && i0.src == 25 &&
           address + micros_result_legacy_preadd.size() <= prog.size() &&
           memcmp(prog.data() + address, micros_result_legacy_preadd.data(), micros_result_legacy_preadd.size()) == 0)
        {
            i0.func = INSTR_MERGED_MICROS_RESULT;
            i0.word = 3;
            continue;
        }
#endif

        if(i0.func == INSTR_LDI && i1.func == INSTR_LDI)
        {
            i0.func = INSTR_MERGED_LDI2;
            i0.m0 = i1.dst;
            i0.m1 = i1.src;
            continue;
        }

        if(i0.func == INSTR_DEC && i1.func == INSTR_BRBC && i1.src == 1)
        {
            i0.func = INSTR_MERGED_DEC_BRNE;
            i0.word = i1.word;
            continue;
        }

        if(i0.func == INSTR_ADD &&
            i1.func == INSTR_ADC &&
            i0.dst + 1 == i1.dst &&
            i0.src + 1 == i1.src &&
            i1.src != i0.dst)
        {
            i0.func = INSTR_MERGED_ADD_ADC;
            continue;
        }

        if(i0.func == INSTR_SUB &&
            i1.func == INSTR_SBC &&
            i0.dst + 1 == i1.dst &&
            i0.src + 1 == i1.src &&
            i1.src != i0.dst)
        {
            i0.func = INSTR_MERGED_SUB_SBC;
            continue;
        }

        if(i0.func == INSTR_CP &&
            i1.func == INSTR_CPC &&
            i0.dst + 1 == i1.dst &&
            i0.src + 1 == i1.src)
        {
            i0.func = INSTR_MERGED_CP_CPC;
            continue;
        }

        if(i0.func == INSTR_SUBI &&
            i1.func == INSTR_SBCI &&
            i0.dst != i1.dst)
        {
            i0.func = INSTR_MERGED_SUBI_SBCI;
            i0.word = i0.src + i1.src * 256;
            i0.src = i1.dst;
            continue;
        }

        {
            uint32_t delay_cycles = 0;
            uint32_t delay_words = 0;

            for(size_t m = n; delay_words < 254 && m < decoded_prog.size();)
            {
                uint32_t t = instr_is_delay(*this, m);
                if(t == 0 || delay_cycles + t > MAX_INSTR_CYCLES)
                    break;

                uint32_t words =
                    instr_is_two_words(decoded_prog[m]) ? 2u : 1u;

                delay_cycles += t;
                delay_words += words;
                m += words;
            }
            if(delay_cycles > 1)
            {
                i0.func = INSTR_MERGED_DELAY;
                i0.src = (uint8_t)delay_words;
                i0.word = (uint16_t)delay_cycles;
            }
        }
    }
#ifdef ARDUGO_EMBEDDED
    for(size_t pc = 0; pc < merged_prog.size(); ++pc)
    {
        auto& call = merged_prog[pc];
        if(call.func != INSTR_CALL) continue;
        const size_t target = call.word;
        if(target + 34 >= merged_prog.size() ||
           merged_prog[target].func != INSTR_MERGED_MICROS_TIMER ||
           merged_prog[target + 20].func != INSTR_MERGED_MICROS_RESULT)
            continue;
        const bool compact = (merged_prog[target + 20].word == 1);
        const auto& insn19 = decoded_prog[target + 19];
        bool ok = false;
        if(!compact)
        {
            ok = target + 36 < merged_prog.size() &&
                 decoded_prog[target + 36].func == INSTR_RET &&
                 insn19.func == INSTR_MOV && insn19.dst == 27 && insn19.src == 26;
        }
        else
        {
            ok = decoded_prog[target + 34].func == INSTR_RET &&
                 ((insn19.func == INSTR_LDI && insn19.dst == 22 && insn19.src == 0) ||
                  (insn19.func == INSTR_CLR && insn19.dst == 22) ||
                  (insn19.func == INSTR_EOR && insn19.dst == 22 && insn19.src == 22));
        }
        if(!ok) continue;
        const auto counter = merged_prog[target].word;
        const auto deadline = micros_wait_address(*this, pc);
        call.func = INSTR_MERGED_MICROS_CALL;
        call.m0 = uint8_t(counter);
        call.m1 = uint8_t(counter >> 8);
        if(!compact && is_arduino_delay_poll(*this, pc))
        {
            call.src = uint8_t(deadline);
            call.dst = uint8_t(deadline >> 8);
            call.m2 = 1;
        }
        else if(!compact && is_arduino_delay_poll_legacy(*this, pc))
        {
            call.src = uint8_t(deadline);
            call.dst = uint8_t(deadline >> 8);
            call.m2 = 5;
        }
        else if(const auto d16 = match_arduino_delay16_poll(*this, pc, compact); d16.loop_cycles != 0)
        {
            call.src = d16.loop_cycles;
            call.dst = d16.start_reg;
            call.m2 = compact ? 3 : 4;
        }
        else
        {
            call.src = uint8_t(deadline);
            call.dst = uint8_t(deadline >> 8);
            call.m2 = compact ? 2 : 0;
        }
    }
#endif
}

}
