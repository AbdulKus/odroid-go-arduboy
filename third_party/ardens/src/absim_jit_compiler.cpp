#include "absim_jit_xtensa.hpp"

#include "absim.hpp"
#include "absim_instructions.hpp"

#include <algorithm>
#include <cstdio>

#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
#endif

namespace absim
{
namespace jit
{

namespace
{

// Register allocation in generated Xtensa blocks (ABI preserved across callx8):
//   a2 = atmega32u4_t* cpu
//   a3 = uint8_t* data (cpu.data.data())
//   a4 = int32_t rem_budget
//   a5 = uint32_t sreg (0..255)
//   a6 = uintptr_t code_base
//   a7 = const uint16_t* block_off
// Scratch: a8..a15 (callx8 args in a10..a12, return in a10).

constexpr uint8_t R_CPU = 2;
constexpr uint8_t R_DATA = 3;
constexpr uint8_t R_REM = 4;
constexpr uint8_t R_SREG = 5;
constexpr uint8_t R_BASE = 6;
constexpr uint8_t R_TBL = 7;

constexpr uint16_t SREG_ADDR = reg::addr::SREG;
constexpr uint16_t SPL_ADDR = reg::addr::SPL;
constexpr uint16_t SPH_ADDR = reg::addr::SPH;

constexpr uint8_t FLAG_C = reg::bit::SREG::C;
constexpr uint8_t FLAG_Z = reg::bit::SREG::Z;
constexpr uint8_t FLAG_N = reg::bit::SREG::N;
constexpr uint8_t FLAG_V = reg::bit::SREG::V;
constexpr uint8_t FLAG_S = reg::bit::SREG::S;
constexpr uint8_t FLAG_H = reg::bit::SREG::H;
constexpr uint8_t FLAG_T = reg::bit::SREG::T;
constexpr uint8_t FLAG_I = reg::bit::SREG::I;

constexpr uint8_t FLAGS_HSVNZC = FLAG_H | FLAG_S | FLAG_V | FLAG_N | FLAG_Z | FLAG_C;
constexpr uint8_t FLAGS_SVNZC = FLAG_S | FLAG_V | FLAG_N | FLAG_Z | FLAG_C;
constexpr uint8_t FLAGS_SVNZ = FLAG_S | FLAG_V | FLAG_N | FLAG_Z;

struct SlowExitStub
{
    uint32_t branch_off;
    uint8_t kind; // 0=bgeui5, 1=beqz, 2=bnez, 3=bltz(bri12), 4=j
    uint8_t s;
    uint16_t target_pc;
    uint16_t cyc_off;
};

struct InsnMeta
{
    uint8_t reads;
    uint8_t writes;
};

bool is_macro_op(uint8_t func)
{
    switch(func)
    {
    case INSTR_MERGED_OLED_SEND:
    case INSTR_MERGED_FX_BLEND:
    case INSTR_MERGED_FX_MASKED_BLEND:
    case INSTR_MERGED_UDIVMOD4:
    case INSTR_MERGED_UDIVMOD2:
    case INSTR_MERGED_MICROS_CALL:
    case INSTR_MERGED_MICROS_RESULT:
    case INSTR_MERGED_MICROS_TIMER:
    case INSTR_MERGED_SPI_POLL:
        return true;
    default:
        return false;
    }
}

ARDENS_FORCEINLINE InsnMeta get_flag_meta(const avr_instr_t& insn)
{
    switch(insn.func)
    {
    case INSTR_NOP:
    case INSTR_MOV:
    case INSTR_MOVW:
    case INSTR_LDI:
    case INSTR_SWAP:
    case INSTR_LDS:
    case INSTR_STS:
    case INSTR_LDD_Y:
    case INSTR_LDD_Z:
    case INSTR_STD_Y:
    case INSTR_STD_Z:
    case INSTR_LD_ST:
    case INSTR_LD_X:
    case INSTR_LD_Y:
    case INSTR_LD_Z:
    case INSTR_LD_X_INC:
    case INSTR_LD_Y_INC:
    case INSTR_LD_Z_INC:
    case INSTR_LD_X_DEC:
    case INSTR_LD_Y_DEC:
    case INSTR_LD_Z_DEC:
    case INSTR_ST_X:
    case INSTR_ST_Y:
    case INSTR_ST_Z:
    case INSTR_ST_X_INC:
    case INSTR_ST_Y_INC:
    case INSTR_ST_Z_INC:
    case INSTR_ST_X_DEC:
    case INSTR_ST_Y_DEC:
    case INSTR_ST_Z_DEC:
    case INSTR_PUSH:
    case INSTR_POP:
    case INSTR_LPM:
    case INSTR_SBI:
    case INSTR_CBI:
    case INSTR_SBIS:
    case INSTR_SBIC:
    case INSTR_SBRS:
    case INSTR_SBRC:
    case INSTR_CPSE:
    case INSTR_RJMP:
    case INSTR_JMP:
    case INSTR_RCALL:
    case INSTR_CALL:
    case INSTR_MERGED_DELAY_CALL:
        return {0, 0};

    case INSTR_IN:
        return {uint8_t((insn.src + 32 == SREG_ADDR) ? 0xff : 0), 0};
    case INSTR_OUT:
        return {0, uint8_t((insn.dst + 32 == SREG_ADDR) ? 0xff : 0)};

    case INSTR_ADD:
    case INSTR_SUB:
    case INSTR_SUBI:
    case INSTR_CP:
    case INSTR_CPI:
    case INSTR_NEG:
        return {0, FLAGS_HSVNZC};

    case INSTR_ADC:
        return {FLAG_C, FLAGS_HSVNZC};

    case INSTR_SBC:
    case INSTR_SBCI:
    case INSTR_CPC:
        return {uint8_t(FLAG_C | FLAG_Z), FLAGS_HSVNZC};

    case INSTR_AND:
    case INSTR_ANDI:
    case INSTR_OR:
    case INSTR_ORI:
    case INSTR_EOR:
    case INSTR_CLR:
    case INSTR_INC:
    case INSTR_DEC:
        return {0, FLAGS_SVNZ};

    case INSTR_COM:
    case INSTR_LSR:
    case INSTR_ASR:
    case INSTR_ADIW:
    case INSTR_SBIW:
        return {0, FLAGS_SVNZC};

    case INSTR_ROR:
        return {FLAG_C, FLAGS_SVNZC};

    case INSTR_MUL:
    case INSTR_MULS:
    case INSTR_MULSU:
    case INSTR_FMUL:
    case INSTR_FMULS:
    case INSTR_FMULSU:
        return {0, uint8_t(FLAG_C | FLAG_Z)};

    case INSTR_BLD:
        return {FLAG_T, 0};
    case INSTR_BST:
        return {0, FLAG_T};
    case INSTR_BSET:
        return {0, insn.dst};
    case INSTR_BCLR:
        return {0, uint8_t(~insn.dst)};
    case INSTR_BRBS:
    case INSTR_BRBC:
        return {uint8_t(1u << insn.src), 0};

    default:
        return {0xff, 0xff};
    }
}

uint8_t peek_live_flags(const avr_instr_t* prog, uint16_t pc, uint32_t max_steps = 24)
{
    uint8_t live = 0xff;
    uint16_t cur = pc;
    for(uint32_t s = 0; s < max_steps; ++s)
    {
        if(cur >= atmega32u4_t::PROG_SIZE_BYTES / 2) break;
        const auto& insn = prog[cur];
        if(insn.func == INSTR_RJMP)
        {
            if(insn.word == UINT16_MAX) break;
            cur = uint16_t(cur + int16_t(insn.word) + 1) & 0x3fff;
            continue;
        }
        if(insn.func == INSTR_JMP)
        {
            cur = insn.word & 0x3fff;
            continue;
        }
        if(insn.func == INSTR_RET || insn.func == INSTR_RETI || insn.func == INSTR_IJMP ||
           insn.func == INSTR_ICALL || insn.func == INSTR_CALL || insn.func == INSTR_RCALL)
            break;

        const auto meta = get_flag_meta(insn);
        if((live & meta.reads) != 0)
            break;
        live &= uint8_t(~meta.writes);
        if((live & FLAGS_HSVNZC) == 0)
            break;

        const uint16_t next_pc = cur + (instr_is_two_words(insn) ? 2 : 1);
        if(insn.func == INSTR_CPSE || insn.func == INSTR_SBRS || insn.func == INSTR_SBRC ||
           insn.func == INSTR_SBIS || insn.func == INSTR_SBIC ||
           insn.func == INSTR_BRBS || insn.func == INSTR_BRBC)
            break;
        cur = next_pc;
    }
    return live;
}

bool is_native_supported(const atmega32u4_t& cpu, const avr_instr_t& insn)
{
    switch(insn.func)
    {
    case INSTR_NOP:
    case INSTR_MOV:
    case INSTR_MOVW:
    case INSTR_LDI:
    case INSTR_SWAP:
    case INSTR_CLR:
    case INSTR_EOR:
    case INSTR_AND:
    case INSTR_ANDI:
    case INSTR_OR:
    case INSTR_ORI:
    case INSTR_ADD:
    case INSTR_ADC:
    case INSTR_SUB:
    case INSTR_SBC:
    case INSTR_SUBI:
    case INSTR_SBCI:
    case INSTR_CP:
    case INSTR_CPC:
    case INSTR_CPI:
    case INSTR_INC:
    case INSTR_DEC:
    case INSTR_COM:
    case INSTR_NEG:
    case INSTR_ASR:
    case INSTR_LSR:
    case INSTR_ROR:
    case INSTR_ADIW:
    case INSTR_SBIW:
    case INSTR_MUL:
    case INSTR_MULS:
    case INSTR_MULSU:
    case INSTR_FMUL:
    case INSTR_FMULS:
    case INSTR_FMULSU:
    case INSTR_BLD:
    case INSTR_BST:
    case INSTR_BSET:
    case INSTR_BCLR:
    case INSTR_LPM:
    case INSTR_LDS:
    case INSTR_STS:
    case INSTR_LDD_Y:
    case INSTR_LDD_Z:
    case INSTR_STD_Y:
    case INSTR_STD_Z:
    case INSTR_LD_ST:
    case INSTR_LD_X:
    case INSTR_LD_Y:
    case INSTR_LD_Z:
    case INSTR_LD_X_INC:
    case INSTR_LD_Y_INC:
    case INSTR_LD_Z_INC:
    case INSTR_LD_X_DEC:
    case INSTR_LD_Y_DEC:
    case INSTR_LD_Z_DEC:
    case INSTR_ST_X:
    case INSTR_ST_Y:
    case INSTR_ST_Z:
    case INSTR_ST_X_INC:
    case INSTR_ST_Y_INC:
    case INSTR_ST_Z_INC:
    case INSTR_ST_X_DEC:
    case INSTR_ST_Y_DEC:
    case INSTR_ST_Z_DEC:
    case INSTR_PUSH:
    case INSTR_POP:
    case INSTR_RJMP:
    case INSTR_JMP:
    case INSTR_IJMP:
    case INSTR_RCALL:
    case INSTR_CALL:
    case INSTR_ICALL:
    case INSTR_RET:
    case INSTR_RETI:
    case INSTR_BRBS:
    case INSTR_BRBC:
    case INSTR_SBRS:
    case INSTR_SBRC:
    case INSTR_CPSE:
        return true;

    case INSTR_IN:
    {
        const uint16_t addr = insn.src + 32;
        return !cpu.ld_handlers[addr] ||
               addr == reg::addr::SPDR || addr == reg::addr::SPSR ||
               addr == reg::addr::TCNT0;
    }
    case INSTR_OUT:
    {
        const uint16_t addr = insn.dst + 32;
        if(!cpu.st_handlers[addr] || addr == reg::addr::SPDR) return true;
#ifdef ARDUGO_EMBEDDED
        if(cpu.spi_bus &&
           ((cpu.st_handlers[addr] == atmega32u4_t::st_handle_port && !cpu.twi_adapter) ||
            cpu.st_handlers[addr] == atmega32u4_t::st_handle_pin))
            return true;
#endif
        return false;
    }
    case INSTR_SBI:
    case INSTR_CBI:
    {
        const uint16_t addr = insn.dst + 32;
        if(cpu.ld_handlers[addr]) return false;
        if(!cpu.st_handlers[addr]) return true;
#ifdef ARDUGO_EMBEDDED
        if(cpu.spi_bus && cpu.st_handlers[addr] == atmega32u4_t::st_handle_port && !cpu.twi_adapter)
            return true;
#endif
        return false;
    }
    case INSTR_SBIS:
    case INSTR_SBIC:
        return !cpu.ld_handlers[insn.dst + 32];

    default:
        return false;
    }
}

bool is_terminating(const avr_instr_t& insn)
{
    switch(insn.func)
    {
    case INSTR_RJMP:
        return insn.word != 0;
    case INSTR_JMP:
    case INSTR_IJMP:
    case INSTR_RCALL:
    case INSTR_CALL:
    case INSTR_ICALL:
    case INSTR_RET:
    case INSTR_RETI:
    case INSTR_BRBS:
    case INSTR_BRBC:
    case INSTR_SBRS:
    case INSTR_SBRC:
    case INSTR_SBIS:
    case INSTR_SBIC:
    case INSTR_CPSE:
        return true;
    default:
        return false;
    }
}

bool is_native_supported_clean(const atmega32u4_t& cpu, const avr_instr_t& insn)
{
    if((insn.func == INSTR_LDS || insn.func == INSTR_STS) &&
       insn.word >= atmega32u4_t::DATA_SIZE_BYTES)
        return false;
    if(insn.func == INSTR_LDS && insn.word < 256)
    {
        const uint16_t addr = insn.word;
        if(addr == SREG_ADDR) return true;
        return !cpu.ld_handlers[addr] ||
               addr == reg::addr::SPDR || addr == reg::addr::SPSR ||
               addr == reg::addr::TCNT0;
    }
    if(insn.func == INSTR_STS && insn.word < 256)
    {
        const uint16_t addr = insn.word;
        if(!cpu.st_handlers[addr] || addr == reg::addr::SPDR) return true;
#ifdef ARDUGO_EMBEDDED
        if(cpu.spi_bus &&
           ((cpu.st_handlers[addr] == atmega32u4_t::st_handle_port && !cpu.twi_adapter) ||
            cpu.st_handlers[addr] == atmega32u4_t::st_handle_pin))
            return true;
#endif
        return false;
    }
    return is_native_supported(cpu, insn);
}

bool has_br8_slow_check(const avr_instr_t& insn)
{
    switch(insn.func)
    {
    case INSTR_MERGED_DELAY_CALL:
    case INSTR_LDD_Y:
    case INSTR_LDD_Z:
    case INSTR_STD_Y:
    case INSTR_STD_Z:
    case INSTR_LD_ST:
    case INSTR_LD_X:
    case INSTR_LD_Y:
    case INSTR_LD_Z:
    case INSTR_LD_X_INC:
    case INSTR_LD_Y_INC:
    case INSTR_LD_Z_INC:
    case INSTR_LD_X_DEC:
    case INSTR_LD_Y_DEC:
    case INSTR_LD_Z_DEC:
    case INSTR_ST_X:
    case INSTR_ST_Y:
    case INSTR_ST_Z:
    case INSTR_ST_X_INC:
    case INSTR_ST_Y_INC:
    case INSTR_ST_Z_INC:
    case INSTR_ST_X_DEC:
    case INSTR_ST_Y_DEC:
    case INSTR_ST_Z_DEC:
    case INSTR_PUSH:
    case INSTR_POP:
    case INSTR_RETI:
        return true;
    default:
        return false;
    }
}

uint8_t base_cycles(const avr_instr_t& insn)
{
    switch(insn.func)
    {
    case INSTR_MERGED_DELAY_CALL: return insn.m1;
    case INSTR_LPM: return 3;
    case INSTR_CALL: return 4;
    case INSTR_RCALL:
    case INSTR_ICALL:
    case INSTR_JMP: return 3;
    case INSTR_RET:
    case INSTR_RETI: return 4;
    case INSTR_ADIW:
    case INSTR_SBIW:
    case INSTR_MUL:
    case INSTR_MULS:
    case INSTR_MULSU:
    case INSTR_FMUL:
    case INSTR_FMULS:
    case INSTR_FMULSU:
    case INSTR_LDS:
    case INSTR_STS:
    case INSTR_LDD_Y:
    case INSTR_LDD_Z:
    case INSTR_STD_Y:
    case INSTR_STD_Z:
    case INSTR_LD_ST:
    case INSTR_LD_X:
    case INSTR_LD_Y:
    case INSTR_LD_Z:
    case INSTR_LD_X_INC:
    case INSTR_LD_Y_INC:
    case INSTR_LD_Z_INC:
    case INSTR_LD_X_DEC:
    case INSTR_LD_Y_DEC:
    case INSTR_LD_Z_DEC:
    case INSTR_ST_X:
    case INSTR_ST_Y:
    case INSTR_ST_Z:
    case INSTR_ST_X_INC:
    case INSTR_ST_Y_INC:
    case INSTR_ST_Z_INC:
    case INSTR_ST_X_DEC:
    case INSTR_ST_Y_DEC:
    case INSTR_ST_Z_DEC:
    case INSTR_PUSH:
    case INSTR_POP:
    case INSTR_RJMP:
    case INSTR_IJMP:
    case INSTR_SBI:
    case INSTR_CBI:
        return 2;
    default:
        return 1;
    }
}

// Emit 32-bit words into code_buf safely (ESP32 IRAM / D-IRAM requires 32-bit aligned writes).
void write_aligned_words(uint32_t* dst_words, uint32_t byte_off, const uint8_t* src, size_t nbytes)
{
    const size_t nwords = nbytes >> 2;
    uint32_t* out = dst_words + (byte_off >> 2);
    for(size_t i = 0; i < nwords; ++i)
    {
        uint32_t w = uint32_t(src[4 * i + 0]) | (uint32_t(src[4 * i + 1]) << 8) |
                     (uint32_t(src[4 * i + 2]) << 16) | (uint32_t(src[4 * i + 3]) << 24);
        out[i] = w;
    }
}

void patch24_iram(uint32_t* buf_words, uint32_t byte_off, uint32_t insn24)
{
    const uint32_t wi = byte_off >> 2;
    const uint32_t shift = (byte_off & 3u) * 8u;
    if((byte_off & 3u) <= 1u)
    {
        const uint32_t mask = ~(0x00ffffffu << shift);
        buf_words[wi] = (buf_words[wi] & mask) | ((insn24 & 0x00ffffffu) << shift);
    }
    else
    {
        const uint32_t lo_bits = 32u - shift;
        const uint32_t lo_mask = (1u << shift) - 1u;
        buf_words[wi] = (buf_words[wi] & lo_mask) | ((insn24 & ((1u << lo_bits) - 1u)) << shift);
        const uint32_t hi_bits = 24u - lo_bits;
        const uint32_t hi_mask = ~((1u << hi_bits) - 1u);
        buf_words[wi + 1] = (buf_words[wi + 1] & hi_mask) | ((insn24 >> lo_bits) & ((1u << hi_bits) - 1u));
    }
}

void patch24_rt(JitRuntime& rt, uint32_t code_off, uint32_t insn24)
{
    for(uint8_t i = 0; i < rt.arena_count; ++i)
    {
        const auto& a = rt.arenas[i];
        if(code_off >= a.base_off && code_off + 3 <= a.base_off + a.cap)
        {
            patch24_iram(a.buf, code_off - a.base_off, insn24);
            return;
        }
    }
}

void write_block_rt(JitRuntime& rt, CodeArena& arena, uint32_t rel_off, const uint8_t* src, size_t nbytes, uint16_t start_pc)
{
    write_aligned_words(arena.buf, rel_off, src, nbytes);
    arena.used = rel_off + uint32_t(nbytes);
    uint32_t total_used = 0;
    for(uint8_t i = 0; i < rt.arena_count; ++i)
        total_used += rt.arenas[i].used;
    rt.code_used = total_used;
    rt.block_off[start_pc] = uint16_t((arena.base_off + rel_off) >> 2);
}

bool in_j_range(uint32_t from_off, uint32_t target_off)
{
    const int64_t disp = int64_t(target_off) - int64_t(from_off) - 4;
    return disp >= -131072 && disp <= 131068;
}

void sync_icache()
{
#ifdef ESP_PLATFORM
    asm volatile("memw; isync");
#endif
}

// Emit 3-cycle exact unsigned range check [256, 2816) on ptr register `r_ptr`.
// Clobbers a13, a14. Emits placeholder bgeui branch.
uint32_t emit_sram_bounds_check(XtensaWriter& w, uint8_t r_ptr)
{
    w.addmi(13, r_ptr, -256);
    w.srli(14, 13, 9);
    return w.bgeui_ph(14, XtensaWriter::b4constu_enc(5));
}

void emit_set_logical_flags(XtensaWriter& w, uint8_t r_res, uint8_t live, bool is_clr)
{
    const uint8_t need = live & FLAGS_SVNZ;
    if(need == 0) return;
    w.load_imm(11, 0xe1);
    w.and_(R_SREG, R_SREG, 11);
    if(need & FLAG_Z)
    {
        if(is_clr)
            w.addi(R_SREG, R_SREG, 2);
        else
        {
            w.addi(11, R_SREG, 2);
            w.moveqz(R_SREG, 11, r_res);
        }
    }
    const uint8_t ns = need & (FLAG_N | FLAG_S);
    if(!is_clr && ns != 0)
    {
        w.extui(11, r_res, 7, 1);
        if(ns == (FLAG_N | FLAG_S))
        {
            w.addx4(11, 11, 11);
            w.slli(11, 11, 2);
        }
        else if(ns == FLAG_N)
            w.slli(11, 11, 2);
        else
            w.slli(11, 11, 4);
        w.or_(R_SREG, R_SREG, 11);
    }
}

// Inputs: r8 = dst, r9 = src, r10 = full_sum/diff, r11 = res8 (0..255).
void emit_add_sub_flags(const JitRuntime& rt, XtensaWriter& w, bool is_sub, bool preserve_z, uint8_t live)
{
    const uint8_t need = live & FLAGS_HSVNZC;
    if(need == 0) return;

    const uint32_t stub = is_sub
        ? (preserve_z ? rt.flag_sbc_off : rt.flag_sub_off)
        : rt.flag_add_off;
    if((need & (FLAG_H | FLAG_V | FLAG_S)) != 0 && stub != 0 &&
       w.pos() + 14 < 524288u && in_j_range(w.pos() + 11, stub))
    {
        const uint32_t ret_off = w.pos() + 14;
        const int32_t lo = int8_t(ret_off & 0xffu);
        const int32_t hi = (int32_t(ret_off) - lo) >> 8;
        w.movi(15, hi);
        w.slli(15, 15, 8);
        w.addi24(15, 15, int8_t(lo));
        w.add(15, R_BASE, 15);
        w.j(stub);
        return;
    }

    if(preserve_z && (need & FLAG_Z))
        w.extui(14, R_SREG, 1, 1); // old Z in a14 (0 or 1)

    w.srli(R_SREG, R_SREG, 6);
    w.slli(R_SREG, R_SREG, 6);

    if(need & FLAG_C)
    {
        w.extui(12, 10, 8, 1);
        w.or_(R_SREG, R_SREG, 12);
    }
    if(need & FLAG_Z)
    {
        if(preserve_z)
        {
            w.slli(14, 14, 1);
            w.add(12, R_SREG, 14);
            w.moveqz(R_SREG, 12, 11);
        }
        else
        {
            w.addi(12, R_SREG, 2);
            w.moveqz(R_SREG, 12, 11);
        }
    }
    if(need & FLAG_H)
    {
        w.xor_(12, 8, 9);
        w.xor_(12, 12, 11);
        w.extui(12, 12, 4, 1);
        w.slli(12, 12, 5);
        w.or_(R_SREG, R_SREG, 12);
    }
    if(need & (FLAG_N | FLAG_V | FLAG_S))
    {
        if(need & (FLAG_V | FLAG_S))
        {
            if(is_sub)
            {
                w.xor_(12, 8, 9);
                w.xor_(13, 8, 11);
                w.and_(12, 12, 13);
            }
            else
            {
                w.xor_(12, 8, 11);
                w.xor_(13, 9, 11);
                w.and_(12, 12, 13);
            }
            if(need & FLAG_V)
            {
                w.extui(13, 12, 7, 1);
                w.slli(13, 13, 3);
                w.or_(R_SREG, R_SREG, 13);
            }
            if(need & FLAG_N)
            {
                w.extui(13, 11, 7, 1);
                w.slli(13, 13, 2);
                w.or_(R_SREG, R_SREG, 13);
            }
            if(need & FLAG_S)
            {
                w.xor_(12, 12, 11);
                w.extui(12, 12, 7, 1);
                w.slli(12, 12, 4);
                w.or_(R_SREG, R_SREG, 12);
            }
        }
        else
        {
            w.extui(13, 11, 7, 1);
            w.slli(13, 13, 2);
            w.or_(R_SREG, R_SREG, 13);
        }
    }
}

void emit_inc_dec_flags(XtensaWriter& w, bool is_dec, uint8_t live)
{
    const uint8_t need = live & FLAGS_SVNZ;
    if(need == 0) return;
    w.load_imm(12, 0xe1);
    w.and_(R_SREG, R_SREG, 12);
    if(need & FLAG_Z)
    {
        w.addi(12, R_SREG, 2);
        w.moveqz(R_SREG, 12, 11);
    }
    if(need & (FLAG_N | FLAG_V | FLAG_S))
    {
        if(need & (FLAG_V | FLAG_S))
        {
            w.xor_(12, 8, 11);
            w.and_(12, 12, is_dec ? 8 : 11);
            if(need & FLAG_V)
            {
                w.extui(13, 12, 7, 1);
                w.slli(13, 13, 3);
                w.or_(R_SREG, R_SREG, 13);
            }
            if(need & FLAG_N)
            {
                w.extui(13, 11, 7, 1);
                w.slli(13, 13, 2);
                w.or_(R_SREG, R_SREG, 13);
            }
            if(need & FLAG_S)
            {
                w.xor_(12, 12, 11);
                w.extui(12, 12, 7, 1);
                w.slli(12, 12, 4);
                w.or_(R_SREG, R_SREG, 12);
            }
        }
        else
        {
            w.extui(13, 11, 7, 1);
            w.slli(13, 13, 2);
            w.or_(R_SREG, R_SREG, 13);
        }
    }
}

void emit_adiw_sbiw_flags(XtensaWriter& w, bool is_sub, uint8_t live)
{
    const uint8_t need = live & FLAGS_SVNZC;
    if(need == 0) return;
    w.load_imm(12, 0xe0);
    w.and_(R_SREG, R_SREG, 12);
    if(need & FLAG_C)
    {
        w.extui(12, 10, 16, 1);
        w.or_(R_SREG, R_SREG, 12);
    }
    if(need & FLAG_Z)
    {
        w.addi(12, R_SREG, 2);
        w.moveqz(R_SREG, 12, 11);
    }
    if(need & (FLAG_N | FLAG_V | FLAG_S))
    {
        if(need & (FLAG_V | FLAG_S))
        {
            w.xor_(12, 8, 11);
            w.and_(12, 12, is_sub ? 8 : 11);
            if(need & FLAG_V)
            {
                w.extui(13, 12, 15, 1);
                w.slli(13, 13, 3);
                w.or_(R_SREG, R_SREG, 13);
            }
            if(need & FLAG_N)
            {
                w.extui(13, 11, 15, 1);
                w.slli(13, 13, 2);
                w.or_(R_SREG, R_SREG, 13);
            }
            if(need & FLAG_S)
            {
                w.xor_(12, 12, 11);
                w.extui(12, 12, 15, 1);
                w.slli(12, 12, 4);
                w.or_(R_SREG, R_SREG, 12);
            }
        }
        else
        {
            w.extui(13, 11, 15, 1);
            w.slli(13, 13, 2);
            w.or_(R_SREG, R_SREG, 13);
        }
    }
}

void emit_shift_flags(XtensaWriter& w, uint8_t live)
{
    // r8 = old_dst, r11 = res8
    const uint8_t need = live & FLAGS_SVNZC;
    if(need == 0) return;
    w.load_imm(12, 0xe0);
    w.and_(R_SREG, R_SREG, 12);
    if(need & FLAG_C)
    {
        w.extui(12, 8, 0, 1);
        w.or_(R_SREG, R_SREG, 12);
    }
    if(need & FLAG_Z)
    {
        w.addi(12, R_SREG, 2);
        w.moveqz(R_SREG, 12, 11);
    }
    if(need & FLAG_N)
    {
        w.extui(12, 11, 7, 1);
        w.slli(12, 12, 2);
        w.or_(R_SREG, R_SREG, 12);
    }
    if(need & FLAG_V)
    {
        w.srli(12, 11, 7);
        w.xor_(12, 12, 8);
        w.extui(12, 12, 0, 1);
        w.slli(12, 12, 3);
        w.or_(R_SREG, R_SREG, 12);
    }
    if(need & FLAG_S)
    {
        // For LSR/ASR/ROR, V = N ^ C => S = N ^ V = C.
        w.extui(12, 8, 0, 1);
        w.slli(12, 12, 4);
        w.or_(R_SREG, R_SREG, 12);
    }
}

void reset_patches(JitRuntime& rt)
{
    rt.patches.clear();
    rt.patch_head.fill(-1);
}

void link_committed_patches(JitRuntime& rt, size_t start_idx)
{
    const size_t n = rt.patches.size();
    for(size_t i = start_idx; i < n; ++i)
    {
        auto& p = rt.patches[i];
        const uint16_t b = p.target_pc & 511u;
        const int16_t head = rt.patch_head[b];
        p.prev = -1;
        p.next = head;
        if(head >= 0) rt.patches[head].prev = int16_t(i);
        rt.patch_head[b] = int16_t(i);
    }
}

void evict_one_arena(JitRuntime& rt)
{
    if(rt.arena_count == 0) return;
    // Reclaim arena[0] once after boot/menu code spills into secondary arenas so cold boot
    // code does not cause ongoing thrashing inside the smaller secondary arenas.
    const bool early_full_reset =
        !rt.boot_reclaimed && rt.secondary_evicts >= 2u && rt.stat_compiles >= 1200u;
    const uint8_t sec_limit = (rt.stat_resets >= 4u) ? 48u : 24u;
    if(rt.arena_count >= 3 && (rt.secondary_evicts >= sec_limit || early_full_reset))
    {
        if(rt.block_off)
            std::memset(rt.block_off, 0, 16384 * sizeof(uint16_t));
        rt.arenas[0].used = rt.stubs_end;
        for(uint8_t i = 1; i < rt.arena_count; ++i)
            rt.arenas[i].used = 0;
        rt.code_used = rt.stubs_end;
        reset_patches(rt);
        rt.cur_arena = 0;
        rt.next_evict_arena = 1;
        rt.secondary_evicts = 0;
        rt.last_full_reset_compiles = rt.stat_compiles;
        rt.boot_reclaimed = true;
        ++rt.stat_resets;
        return;
    }
    uint8_t v = 0;
    if(rt.arena_count >= 3)
    {
        v = (rt.next_evict_arena >= 1 && rt.next_evict_arena < rt.arena_count)
                ? rt.next_evict_arena
                : uint8_t(1);
        rt.next_evict_arena = (v + 1 < rt.arena_count) ? uint8_t(v + 1) : uint8_t(1);
        ++rt.secondary_evicts;
    }
    else
    {
        v = rt.next_evict_arena % rt.arena_count;
        rt.next_evict_arena = uint8_t((v + 1) % rt.arena_count);
    }
    CodeArena& va = rt.arenas[v];
    const uint32_t v_lo_bytes = va.base_off + (v == 0 ? rt.stubs_end : 0u);
    const uint32_t v_hi_bytes = va.base_off + va.cap;
    const uint16_t v_lo_w = static_cast<uint16_t>(v_lo_bytes >> 2);
    const uint16_t v_hi_w = static_cast<uint16_t>(v_hi_bytes >> 2);
    for(size_t pc = 0; pc < 16384; ++pc)
    {
        const uint16_t off_w = rt.block_off[pc];
        if(off_w >= v_lo_w && off_w < v_hi_w)
            rt.block_off[pc] = 0;
    }
    size_t keep = 0;
    for(size_t i = 0; i < rt.patches.size(); ++i)
    {
        const uint32_t co = rt.patches[i].code_off;
        if(co >= v_lo_bytes && co + 3 <= v_hi_bytes)
            continue;
        rt.patches[keep++] = rt.patches[i];
    }
    rt.patches.resize(keep);
    rt.patch_head.fill(-1);
    link_committed_patches(rt, 0);
    va.used = (v == 0) ? rt.stubs_end : 0u;
    uint32_t total_used = 0;
    for(uint8_t i = 0; i < rt.arena_count; ++i)
        total_used += rt.arenas[i].used;
    rt.code_used = total_used;
    rt.cur_arena = v;
    ++rt.stat_resets;
}

void emit_direct_jump(JitRuntime& rt, XtensaWriter& w, uint16_t target_pc, uint16_t block_start_pc, uint32_t block_start_off)
{
    target_pc &= 0x3fff;
    if(target_pc == block_start_pc)
    {
        w.j(block_start_off);
        return;
    }
    const auto& cur_arena = rt.arenas[rt.cur_arena];
    const uint32_t a_lo = cur_arena.base_off + (rt.cur_arena == 0 ? rt.stubs_end : 0u);
    const uint32_t a_hi = cur_arena.base_off + cur_arena.cap;
    const auto& a0 = rt.arenas[0];
    const uint32_t a0_lo = a0.base_off + rt.stubs_end;
    const uint32_t a0_hi = a0.base_off + a0.cap;
    if(rt.block_off && rt.block_off[target_pc] != 0)
    {
        const uint32_t tgt_off = uint32_t(rt.block_off[target_pc]) << 2;
        const bool jump_a0 = (rt.arena_count >= 3) && (tgt_off >= a0_lo && tgt_off < a0_hi);
        if(((tgt_off >= a_lo && tgt_off < a_hi) || jump_a0) &&
           in_j_range(w.pos(), tgt_off))
        {
            w.j(tgt_off);
            return;
        }
        w.load_u14(8, target_pc);
        w.j(rt.dispatch_off);
        return;
    }
    uint32_t patch_at = w.pos();
    w.load_u14(8, target_pc);
    w.j(rt.dispatch_off);
    if(rt.patches.size() < 32760)
        rt.patches.push_back({patch_at, target_pc, -1, -1});
}

void emit_call_helper(XtensaWriter& w, const JitOffsets& off, uint8_t slot)
{
    w.l32i(9, R_CPU, uint16_t(off.helpers + slot * 4));
    w.callx8(9);
}

void load_u16(XtensaWriter& w, uint8_t d, uint16_t v)
{
    if(v <= 2047)
    {
        w.load_u14(d, v);
        return;
    }
    if(v <= 32767)
    {
        w.load_imm(d, int32_t(v));
        return;
    }
    const uint8_t lo = uint8_t(v & 0xffu);
    if(lo <= 127)
    {
        w.movi(d, int32_t(v >> 8));
        w.slli(d, d, 8);
        if(lo) w.addi(d, d, int8_t(lo));
    }
    else
    {
        // Round up upper bits so the low byte becomes negative (-128..-1) for one addi.
        w.movi(d, int32_t((v >> 8) + 1u));
        w.slli(d, d, 8);
        w.addi(d, d, int8_t(int32_t(lo) - 256));
    }
}

void load_u32(XtensaWriter& w, uint8_t d, uint32_t v)
{
    if(v <= 65535u)
    {
        load_u16(w, d, uint16_t(v));
        return;
    }
    // Split v as (hi16 << 16) + (b1 << 8) + b0 with signed b0, b1 in [-128, 127].
    const int32_t b0 = static_cast<int8_t>(v & 0xffu);
    const uint32_t t1 = static_cast<uint32_t>(static_cast<uint64_t>(v) - static_cast<int64_t>(b0));
    const int32_t b1 = static_cast<int8_t>((t1 >> 8) & 0xffu);
    const uint32_t t2 = static_cast<uint32_t>(static_cast<uint64_t>(t1) - (static_cast<int64_t>(b1) << 8));
    const uint16_t hi16 = static_cast<uint16_t>(t2 >> 16);
    load_u16(w, d, hi16);
    if(hi16 != 0) w.slli(d, d, 16);
    if(b1 != 0) w.addmi(d, d, b1 * 256);
    if(b0 != 0) w.addi(d, d, static_cast<int8_t>(b0));
}

bool build_shared_stubs(JitRuntime& rt, const JitOffsets& off)
{
    if(rt.arena_count == 0) return false;
    auto& a0 = rt.arenas[0];
    XtensaWriter w(a0.base_off);

    // jit_exit_miss (a8 = target_pc) -> returns (1 << 16) | (rem & 0xffff)
    rt.miss_off = w.pos();
    w.s8i(R_SREG, R_DATA, SREG_ADDR);
    w.s16i(8, R_CPU, off.pc);
    w.movi_n(9, 1);
    w.slli(9, 9, 16);
    w.or_(2, R_REM, 9);
    w.retw_n();

    // jit_exit_slow (a8 = target_pc) -> returns (0 << 16) | (rem & 0xffff)
    w.align4();
    rt.slow_off = w.pos();
    w.s8i(R_SREG, R_DATA, SREG_ADDR);
    w.s16i(8, R_CPU, off.pc);
    w.mov_n(2, R_REM);
    w.retw_n();

    // Entry point from C++: (a2=cpu, a3=data, a4=rem, a5=sreg, a6=code_base, a7=block_off)
    w.align4();
    rt.entry_off = w.pos();
    w.entry32();
    w.l16ui(8, R_CPU, off.pc);
    // Fall straight into jit_dispatch_a8!

    // jit_dispatch_a8 (a8 = target_pc)
    rt.dispatch_off = w.pos();
    w.extui(8, 8, 0, 14);
    w.addx2(9, 8, R_TBL);
    w.l16ui(9, 9, 0);
    w.beqz(9, rt.miss_off);
    w.slli(9, 9, 2);
    w.add(9, 9, R_BASE);
    w.jx(9);

    // Shared CALL stub:
    //   call2_off (CALL): a8 = call_pc, a10 = target_pc -> insn_words = 2 (4 cycles)
    //   call1_off (RCALL/ICALL): a8 = call_pc, a10 = target_pc -> insn_words = 1 (3 cycles)
    w.align4();
    rt.call2_off = w.pos();
    w.movi_n(12, 2);
    const uint32_t call_skip_ph = w.emit_j_placeholder();
    rt.call1_off = w.pos();
    w.movi_n(12, 1);
    w.patch_j(call_skip_ph, w.pos());
    w.l8ui(11, R_DATA, SPL_ADDR);
    w.l8ui(9, R_DATA, SPH_ADDR);
    w.slli(9, 9, 8);
    w.or_(11, 11, 9);               // a11 = SP
    w.addmi(13, 11, -256);          // a13 = SP - 256
    w.beqz(13, rt.slow_off);        // SP == 256 -> slow_off
    w.srli(14, 13, 9);
    const uint32_t call_ok_ph = w.bltui_ph(14, XtensaWriter::b4constu_enc(5));
    w.j(rt.slow_off);
    w.patch_br8(call_ok_ph, uint8_t(XtensaWriter::b4constu_enc(5)), 14, 11, w.pos(), 6);
    w.sub(R_REM, R_REM, 12);
    w.addi(R_REM, R_REM, -2);       // 3 cycles for 1-word call, 4 cycles for 2-word CALL
    w.add(12, 8, 12);               // ret_pc = (call_pc + words) & 0x3fff
    w.extui(12, 12, 0, 14);
    w.add(9, R_DATA, 11);
    w.s8i(12, 9, 0);
    w.srli(13, 12, 8);
    w.addi_n(9, 9, -1);
    w.s8i(13, 9, 0);
    w.addi(11, 11, -2);
    w.s8i(11, R_DATA, SPL_ADDR);
    w.srli(9, 11, 8);
    w.s8i(9, R_DATA, SPH_ADDR);
    w.mov_n(8, 10);
    w.j(rt.dispatch_off);

    // Shared RET stub: a8 = ret_insn_pc
    w.align4();
    rt.ret_off = w.pos();
    w.l8ui(11, R_DATA, SPL_ADDR);
    w.l8ui(9, R_DATA, SPH_ADDR);
    w.slli(9, 9, 8);
    w.or_(11, 11, 9);
    w.addi(12, 11, 2);
    w.extui(12, 12, 0, 16);         // new_sp = (SP + 2) & 0xffff
    w.addmi(13, 12, -256);
    w.beqz(13, rt.slow_off);        // new_sp == 256 -> slow_off
    w.srli(14, 13, 9);
    const uint32_t ret_ok_ph = w.bltui_ph(14, XtensaWriter::b4constu_enc(5));
    w.j(rt.slow_off);
    w.patch_br8(ret_ok_ph, uint8_t(XtensaWriter::b4constu_enc(5)), 14, 11, w.pos(), 6);
    w.add(9, R_DATA, 12);
    w.l8ui(10, 9, 0);
    w.addi_n(9, 9, -1);
    w.l8ui(8, 9, 0);
    w.slli(8, 8, 8);
    w.or_(8, 8, 10);                // a8 = ret_pc
    w.s8i(12, R_DATA, SPL_ADDR);
    w.srli(9, 12, 8);
    w.s8i(9, R_DATA, SPH_ADDR);
    w.addi(R_REM, R_REM, -4);
    w.j(rt.dispatch_off);

    // Shared full HSVNZC flag stubs (inputs: a8=dst, a9=src, a10=full, a11=res8, a15=return addr).
    w.align4();
    rt.flag_add_off = w.pos();
    w.srli(R_SREG, R_SREG, 6);
    w.slli(R_SREG, R_SREG, 6);
    w.extui(12, 10, 8, 1);
    w.or_(R_SREG, R_SREG, 12);
    w.addi(12, R_SREG, 2);
    w.moveqz(R_SREG, 12, 11);
    w.xor_(12, 8, 9);
    w.xor_(12, 12, 11);
    w.extui(12, 12, 4, 1);
    w.slli(12, 12, 5);
    w.or_(R_SREG, R_SREG, 12);
    w.xor_(12, 8, 11);
    w.xor_(13, 9, 11);
    w.and_(12, 12, 13);
    w.extui(13, 12, 7, 1);
    w.slli(13, 13, 3);
    w.or_(R_SREG, R_SREG, 13);
    w.extui(13, 11, 7, 1);
    w.slli(13, 13, 2);
    w.or_(R_SREG, R_SREG, 13);
    w.xor_(12, 12, 11);
    w.extui(12, 12, 7, 1);
    w.slli(12, 12, 4);
    w.or_(R_SREG, R_SREG, 12);
    w.jx(15);

    w.align4();
    rt.flag_sbc_off = w.pos();
    w.extui(14, R_SREG, 1, 1);
    const uint32_t sbc_join_ph = w.emit_j_placeholder();
    rt.flag_sub_off = w.pos();
    w.movi_n(14, 1);
    w.patch_j(sbc_join_ph, w.pos());
    w.srli(R_SREG, R_SREG, 6);
    w.slli(R_SREG, R_SREG, 6);
    w.extui(12, 10, 8, 1);
    w.or_(R_SREG, R_SREG, 12);
    w.slli(14, 14, 1);
    w.add(12, R_SREG, 14);
    w.moveqz(R_SREG, 12, 11);
    w.xor_(12, 8, 9);
    w.xor_(13, 12, 11);
    w.extui(13, 13, 4, 1);
    w.slli(13, 13, 5);
    w.or_(R_SREG, R_SREG, 13);
    w.xor_(13, 8, 11);
    w.and_(12, 12, 13);
    w.extui(13, 12, 7, 1);
    w.slli(13, 13, 3);
    w.or_(R_SREG, R_SREG, 13);
    w.extui(13, 11, 7, 1);
    w.slli(13, 13, 2);
    w.or_(R_SREG, R_SREG, 13);
    w.xor_(12, 12, 11);
    w.extui(12, 12, 7, 1);
    w.slli(12, 12, 4);
    w.or_(R_SREG, R_SREG, 12);
    w.jx(15);

    w.align4();
    const uint32_t stubs_size = uint32_t(w.bytes().size());
    if(stubs_size > a0.cap) return false;
    write_aligned_words(a0.buf, 0, w.bytes().data(), stubs_size);
    a0.used = stubs_size;
    rt.cur_arena = 0;
    rt.stubs_end = stubs_size;
    rt.code_used = stubs_size;
    sync_icache();
    return true;
}

} // namespace

JitOffsets compute_jit_offsets(const atmega32u4_t& cpu)
{
    auto off_of = [&](const void* member) -> uint16_t {
        return static_cast<uint16_t>(reinterpret_cast<const char*>(member) -
                                     reinterpret_cast<const char*>(&cpu));
    };
    return {
        off_of(&cpu.cycle_count),
        off_of(&cpu.jit_batch_end),
        off_of(&cpu.jit_helpers),
        off_of(&cpu.jit_prog_ptr),
        off_of(&cpu.jit_irq_enable_rem),
        off_of(&cpu.jit_spm_active),
        off_of(&cpu.pc),
        off_of(&cpu.prev_sreg),
        off_of(&cpu.active),
        off_of(&cpu.io_reg_accessed),
    };
}

void jit_invalidate_runtime(JitRuntime& rt, const atmega32u4_t& cpu)
{
    if(rt.owner_cpu == reinterpret_cast<uintptr_t>(&cpu))
    {
        if(rt.block_off)
            std::memset(rt.block_off, 0, 16384 * sizeof(uint16_t));
        if(rt.miss_counts)
            std::memset(rt.miss_counts, 0, 16384);
        rt.cur_arena = 0;
        if(rt.arena_count > 0) rt.arenas[0].used = rt.stubs_end;
        for(uint8_t i = 1; i < rt.arena_count; ++i)
            rt.arenas[i].used = 0;
        rt.code_used = rt.stubs_end;
        reset_patches(rt);
        rt.secondary_evicts = 0;
        rt.last_full_reset_compiles = rt.stat_compiles;
        rt.boot_reclaimed = false;
        rt.owner_gen = cpu.jit_decode_gen;
    }
}

void jit_preallocate_runtime(JitRuntime& rt)
{
    if(rt.code_buf || rt.failed) return;
#ifdef ESP_PLATFORM
    // Keep ~4 KiB of internal D/IRAM free for system/stack allocations while claiming
    // available EXEC IRAM and D/IRAM regions into multi-arena chunks.
    void* dram_guard = nullptr;
    const uint32_t d8_largest = static_cast<uint32_t>(
        heap_caps_get_largest_free_block(MALLOC_CAP_EXEC | MALLOC_CAP_8BIT));
    if(d8_largest > 24 * 1024)
        dram_guard = heap_caps_malloc(4096, MALLOC_CAP_EXEC | MALLOC_CAP_8BIT);

    std::array<CodeArena, JitRuntime::kMaxArenas> chunks{};
    uint8_t nchunks = 0;
    while(nchunks < JitRuntime::kMaxArenas)
    {
        const uint32_t avail = static_cast<uint32_t>(
            heap_caps_get_largest_free_block(MALLOC_CAP_EXEC | MALLOC_CAP_32BIT));
        uint32_t want = (avail > 4352) ? std::min<uint32_t>(120 * 1024, (avail - 256) & ~255u) : 0u;
        uint32_t* buf = nullptr;
        while(want >= 4096)
        {
            buf = static_cast<uint32_t*>(
                heap_caps_malloc(want, MALLOC_CAP_EXEC | MALLOC_CAP_32BIT));
            if(buf) break;
            want = (want > 5120) ? (want - 1024) : 0u;
        }
#ifdef ARDUGO_QUIET_TEST
        std::printf("jit_prealloc: d8=%u avail=%u want=%u buf=%p\n",
                    unsigned(d8_largest), unsigned(avail), unsigned(want), static_cast<void*>(buf));
#endif
        if(!buf) break;
        chunks[nchunks++] = {buf, reinterpret_cast<uintptr_t>(buf), 0, want, 0};
    }
    if(dram_guard) heap_caps_free(dram_guard);

    if(nchunks > 0)
    {
        uint8_t best = 0;
        for(uint8_t i = 1; i < nchunks; ++i)
            if(chunks[i].cap > chunks[best].cap) best = i;
        if(best != 0) std::swap(chunks[0], chunks[best]);
        for(uint8_t i = 1; i < nchunks; ++i)
            for(uint8_t j = uint8_t(i + 1); j < nchunks; ++j)
                if(chunks[j].cap > chunks[i].cap) std::swap(chunks[i], chunks[j]);

        std::array<CodeArena, JitRuntime::kMaxArenas> kept{};
        uint8_t nkept = 0;
        kept[nkept++] = chunks[0];
        uintptr_t min_base = chunks[0].base_addr;
        uintptr_t max_end = chunks[0].base_addr + chunks[0].cap;
        const uintptr_t stubs_lo = chunks[0].base_addr;
        const uintptr_t stubs_hi = chunks[0].base_addr + 1024;

        for(uint8_t i = 1; i < nchunks; ++i)
        {
            const auto& c = chunks[i];
            const uintptr_t cand_min = std::min<uintptr_t>(min_base, c.base_addr);
            const uintptr_t cand_max = std::max<uintptr_t>(max_end, c.base_addr + uintptr_t(c.cap));
            const int64_t span = int64_t(cand_max - cand_min);
            const int64_t j_lo = int64_t(stubs_lo) - int64_t(c.base_addr + c.cap);
            const int64_t j_hi = int64_t(stubs_hi) - int64_t(c.base_addr) - 4;
            if(span <= 262140 && j_lo >= -131072 && j_hi <= 131068)
            {
                kept[nkept++] = c;
                min_base = cand_min;
                max_end = cand_max;
            }
            else
            {
#ifdef ARDUGO_QUIET_TEST
                std::printf("jit_prealloc: drop arena base=0x%lx cap=%u span=%ld\n",
                            (unsigned long)c.base_addr, unsigned(c.cap), long(span));
#endif
                heap_caps_free(c.buf);
            }
        }
        rt.arena_count = nkept;
        for(uint8_t i = 0; i < nkept; ++i)
            rt.arenas[i] = kept[i];
        rt.code_base_addr = min_base - 4;
        rt.code_cap = 0;
        for(uint8_t i = 0; i < rt.arena_count; ++i)
        {
            rt.arenas[i].base_off = static_cast<uint32_t>(rt.arenas[i].base_addr - rt.code_base_addr);
            rt.code_cap += rt.arenas[i].cap;
#ifdef ARDUGO_QUIET_TEST
            std::printf("jit_prealloc: arena[%u] addr=0x%lx off=%u cap=%u\n",
                        unsigned(i), (unsigned long)rt.arenas[i].base_addr,
                        unsigned(rt.arenas[i].base_off), unsigned(rt.arenas[i].cap));
#endif
        }
        rt.code_buf = rt.arenas[0].buf;
    }
    rt.block_off = static_cast<uint16_t*>(
        heap_caps_malloc(16384 * sizeof(uint16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    rt.block_in_internal = (rt.block_off != nullptr);
    if(!rt.block_off)
        rt.block_off = static_cast<uint16_t*>(
            heap_caps_malloc(16384 * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if(!rt.miss_counts)
        rt.miss_counts = static_cast<uint8_t*>(
            heap_caps_malloc(16384, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
#else
    rt.host_code.assign(64 * 1024 / 4, 0);
    rt.arenas[0] = {rt.host_code.data(), reinterpret_cast<uintptr_t>(rt.host_code.data()),
                    0, uint32_t(rt.host_code.size() * 4), 0};
    rt.arena_count = 1;
    rt.cur_arena = 0;
    rt.code_buf = rt.arenas[0].buf;
    rt.code_cap = rt.arenas[0].cap;
    rt.code_base_addr = rt.arenas[0].base_addr;
    rt.block_off = rt.host_blocks.data();
    rt.block_in_internal = true;
#endif
    if(!rt.code_buf || !rt.block_off)
    {
        rt.failed = true;
        return;
    }
}

void jit_reset_runtime(JitRuntime& rt, atmega32u4_t& cpu, const JitOffsets& off)
{
    if(rt.failed) return;
    if(!rt.code_buf)
    {
        jit_preallocate_runtime(rt);
        if(rt.failed) return;
    }
    if(rt.stubs_end == 0)
    {
        if(!build_shared_stubs(rt, off))
        {
            rt.failed = true;
            return;
        }
    }
    std::memset(rt.block_off, 0, 16384 * sizeof(uint16_t));
    if(rt.miss_counts)
        std::memset(rt.miss_counts, 0, 16384);
    rt.cur_arena = 0;
    rt.next_evict_arena = (rt.arena_count >= 3) ? 1 : 0;
    rt.secondary_evicts = 0;
    rt.last_full_reset_compiles = rt.stat_compiles;
    rt.boot_reclaimed = false;
    rt.arenas[0].used = rt.stubs_end;
    for(uint8_t i = 1; i < rt.arena_count; ++i)
        rt.arenas[i].used = 0;
    rt.code_used = rt.stubs_end;
    reset_patches(rt);
    rt.owner_cpu = reinterpret_cast<uintptr_t>(&cpu);
    rt.owner_gen = cpu.jit_decode_gen;
}

template <typename T, size_t Cap>
struct FixedVec
{
    T data_[Cap]{};
    size_t n_ = 0;
    void clear() { n_ = 0; }
    bool empty() const { return n_ == 0; }
    size_t size() const { return n_; }
    void push_back(const T& v) { if(n_ < Cap) data_[n_++] = v; }
    void pop_back() { if(n_ > 0) --n_; }
    T& back() { return data_[n_ - 1]; }
    const T& back() const { return data_[n_ - 1]; }
    T& operator[](size_t i) { return data_[i]; }
    const T& operator[](size_t i) const { return data_[i]; }
    const T* begin() const { return data_; }
    const T* end() const { return data_ + n_; }
};

bool jit_compile_block(JitRuntime& rt, const atmega32u4_t& cpu, const JitOffsets& off, uint16_t start_pc)
{
    if(rt.failed || rt.arena_count == 0 || !rt.block_off) return false;
    start_pc &= 0x3fff;
    if(rt.block_off[start_pc] != 0) return true;

    const auto& decoded = cpu.decoded_prog;
    const auto& merged = cpu.merged_prog;

    // 1) Collect either a macro stub, a single timed stub, or a native basic block.
    const auto& first_m = merged[start_pc];
    const auto& first_d = decoded[start_pc];

    struct BlockInsn { uint16_t pc; uint8_t cyc_off; avr_instr_t insn; uint8_t live; };
    struct Scratch {
        XtensaWriter w;
        FixedVec<BlockInsn, 28> ops;
        FixedVec<SlowExitStub, 64> slow_stubs;
    };
    static Scratch scratch;
    auto& w = scratch.w;

    ++rt.stat_compiles;
    uint32_t block_off_bytes = 0;
    const size_t saved_patch_count = rt.patches.size();

    uint8_t scanned = 0;
    uint8_t evicts = 0;
    for(int arena_try = 0; arena_try <= int(rt.arena_count) * 2 + 2; ++arena_try)
    {
        while(scanned < rt.arena_count &&
              ((rt.arenas[rt.cur_arena].used + 3u) & ~3u) + 64 > rt.arenas[rt.cur_arena].cap)
        {
            rt.cur_arena = uint8_t((rt.cur_arena + 1) % rt.arena_count);
            ++scanned;
        }
        if(scanned >= rt.arena_count)
        {
            if(evicts >= rt.arena_count)
                return false;
            evict_one_arena(rt);
            ++evicts;
            scanned = 0;
        }

        CodeArena& arena = rt.arenas[rt.cur_arena];
        const uint32_t rel_off = (arena.used + 3u) & ~3u;
        block_off_bytes = arena.base_off + rel_off;
        w.reset(block_off_bytes);

        if(is_macro_op(first_m.func))
        {
            w.s8i(R_SREG, R_DATA, SREG_ADDR);
            w.mov_n(10, R_CPU);
            uint8_t slot = 6;
            switch(first_m.func)
            {
            case INSTR_MERGED_OLED_SEND:
                slot = 8;
                w.load_u14(11, start_pc);
                w.mov_n(12, R_REM);
                break;
            case INSTR_MERGED_FX_BLEND:
                slot = 9;
                load_u32(w, 11, uint32_t(start_pc) | (uint32_t(first_m.m1 & 1u) << 14));
                w.mov_n(12, R_REM);
                break;
            case INSTR_MERGED_FX_MASKED_BLEND:
                slot = 10;
                w.load_u14(11, start_pc);
                w.mov_n(12, R_REM);
                break;
            case INSTR_MERGED_UDIVMOD4:
            case INSTR_MERGED_UDIVMOD2:
                slot = 16;
                load_u32(w, 11, uint32_t(start_pc) |
                                (uint32_t(first_m.func == INSTR_MERGED_UDIVMOD4 ? 1u : 0u) << 14));
                w.mov_n(12, R_REM);
                break;
            case INSTR_MERGED_MICROS_CALL:
                slot = 11;
                load_u32(w, 11, uint32_t(start_pc) | (uint32_t(first_m.word & 0x3fffu) << 14) |
                                (uint32_t(first_m.m2 & 0x7u) << 28));
                load_u32(w, 12, uint32_t(first_m.m0) | (uint32_t(first_m.m1) << 8) |
                                (uint32_t(first_m.src) << 16) | (uint32_t(first_m.dst) << 24));
                w.mov_n(13, R_REM);
                break;
            case INSTR_MERGED_MICROS_RESULT:
                slot = 12;
                load_u32(w, 11, uint32_t(start_pc) | (uint32_t(first_m.word & 3u) << 16));
                w.mov_n(12, R_REM);
                break;
            case INSTR_MERGED_MICROS_TIMER:
                slot = 13;
                load_u32(w, 11, uint32_t(start_pc) | (uint32_t(first_m.word) << 16));
                w.mov_n(12, R_REM);
                break;
            case INSTR_MERGED_SPI_POLL:
                slot = 14;
                load_u32(w, 11, uint32_t(start_pc) | (uint32_t(first_m.dst & 31u) << 14) |
                                (uint32_t(first_m.m0 & 31u) << 19) |
                                (uint32_t(first_m.src == reg::addr::SPDR - 32 ? 1u : 0u) << 24));
                w.mov_n(12, R_REM);
                break;
            default:
                w.load_u14(11, start_pc);
                w.mov_n(12, R_REM);
                break;
            }
            emit_call_helper(w, off, slot);
            w.l8ui(R_SREG, R_DATA, SREG_ADDR);
            w.extui(R_REM, 10, 0, 16);
            auto exit_ph = w.emit_bri12_placeholder(2, 10); // bltz a10 -> exit
            w.l16ui(8, R_CPU, off.pc);
            w.j(rt.dispatch_off);
            const uint32_t exit_lbl = w.pos();
            w.patch_bri12(exit_ph, 2, 10, exit_lbl);
            w.mov_n(2, R_REM);
            w.retw_n();

            w.align4();
            if(w.overflow() || rel_off + w.bytes().size() > arena.cap)
            {
                arena.used = arena.cap;
                rt.cur_arena = uint8_t((rt.cur_arena + 1) % rt.arena_count);
                ++scanned;
                continue;
            }
            write_block_rt(rt, arena, rel_off, w.bytes().data(), w.bytes().size(), start_pc);
            break;
        }
        else if(!is_native_supported_clean(cpu, first_d))
        {
            // Timed fallback block (executes single instruction via INSTR_MAP).
            w.addi(8, R_REM, -atmega32u4_t::MAX_INSTR_CYCLES);
            auto entry_exit_ph = w.emit_bri12_placeholder(2, 8);
            w.s8i(R_SREG, R_DATA, SREG_ADDR);
            w.mov_n(10, R_CPU);
            w.load_u14(11, start_pc);
            w.mov_n(12, R_REM);
            emit_call_helper(w, off, 7); // jit_helper_timed_insn
            w.l8ui(R_SREG, R_DATA, SREG_ADDR);
            w.extui(R_REM, 10, 0, 16);
            auto timed_exit_ph = w.emit_bri12_placeholder(2, 10);
            w.l16ui(8, R_CPU, off.pc);
            w.j(rt.dispatch_off);

            const uint32_t timed_exit_lbl = w.pos();
            w.patch_bri12(timed_exit_ph, 2, 10, timed_exit_lbl);
            w.mov_n(2, R_REM);
            w.retw_n();

            const uint32_t entry_exit_lbl = w.pos();
            w.patch_bri12(entry_exit_ph, 2, 8, entry_exit_lbl);
            w.load_u14(8, start_pc);
            w.j(rt.slow_off);

            w.align4();
            if(w.overflow() || rel_off + w.bytes().size() > arena.cap)
            {
                arena.used = arena.cap;
                rt.cur_arena = uint8_t((rt.cur_arena + 1) % rt.arena_count);
                ++scanned;
                continue;
            }
            write_block_rt(rt, arena, rel_off, w.bytes().data(), w.bytes().size(), start_pc);
            break;
        }
        else
        {
            // Collect a basic block up to 24 instructions / 32 cycles.
            auto& ops = scratch.ops;
            ops.clear();

            uint16_t cur = start_pc;
            uint32_t lin_cycles = 0;
            uint32_t br8_ops = 0;
            bool terminated = false;
            auto continues_batch = [](const avr_instr_t& prev, const avr_instr_t& next) -> bool {
                if(prev.func == INSTR_PUSH && next.func == INSTR_PUSH) return true;
                if(prev.func == INSTR_POP && next.func == INSTR_POP) return true;
                if(prev.func == INSTR_LDD_Y && (prev.src == 28 || prev.src == 29)) return false;
                if(prev.func == INSTR_LDD_Z && (prev.src == 30 || prev.src == 31)) return false;
                const bool py = (prev.func == INSTR_LDD_Y || prev.func == INSTR_STD_Y);
                const bool ny = (next.func == INSTR_LDD_Y || next.func == INSTR_STD_Y);
                if(py && ny) return true;
                const bool pz = (prev.func == INSTR_LDD_Z || prev.func == INSTR_STD_Z);
                const bool nz = (next.func == INSTR_LDD_Z || next.func == INSTR_STD_Z);
                return pz && nz;
            };
            while(ops.size() < 28 && lin_cycles < 44 && cur < atmega32u4_t::PROG_SIZE_BYTES / 2)
            {
                if(ops.size() > 0 && (is_macro_op(merged[cur].func) || rt.block_off[cur] != 0))
                    break;
                const auto& m_op = merged[cur];
                const auto& d_op = decoded[cur];
                const bool is_delay_call = (m_op.func == INSTR_MERGED_DELAY_CALL);
                const auto& op = is_delay_call ? m_op : d_op;
                if(!is_delay_call && !is_native_supported_clean(cpu, op))
                    break;
                if(op.func == INSTR_RJMP && op.word == UINT16_MAX)
                {
                    if(ops.empty()) return false; // self idle loop handled cleanly by execute_merged
                    break;
                }
                const bool uses_br8 = has_br8_slow_check(op);
                const bool in_batch = !ops.empty() && !is_delay_call && continues_batch(ops.back().insn, op);
                if(!ops.empty() && uses_br8 && !in_batch && br8_ops >= 8)
                    break;
                const uint8_t words = is_delay_call ? op.m0 : (instr_is_two_words(op) ? 2 : 1);
                if(cur + words > atmega32u4_t::PROG_SIZE_BYTES / 2) break;
                const uint8_t bc = base_cycles(op);
                if(!ops.empty() && lin_cycles + bc > 48) break;

                ops.push_back({cur, uint8_t(lin_cycles), op, 0xff});
                if(uses_br8 && !in_batch) ++br8_ops;
                if(is_terminating(op))
                {
                    if(op.func == INSTR_BRBS || op.func == INSTR_BRBC ||
                       op.func == INSTR_SBRS || op.func == INSTR_SBRC ||
                       op.func == INSTR_SBIS || op.func == INSTR_SBIC ||
                       op.func == INSTR_CPSE)
                    {
                        // Taken / skip adds up to +2 or +3 cycles
                    }
                    else
                        lin_cycles += bc;
                    terminated = true;
                    cur += words;
                    break;
                }
                lin_cycles += bc;
                cur += words;
            }
            if(ops.empty()) return false;

        const size_t try_patch_count = std::min(saved_patch_count, rt.patches.size());
        bool need_next_arena = false;
        while(!ops.empty())
        {
            rt.patches.resize(try_patch_count);
            w.reset(block_off_bytes);

            const auto& last_op = ops.back().insn;
            const uint8_t last_words = (last_op.func == INSTR_MERGED_DELAY_CALL)
                ? last_op.m0
                : (instr_is_two_words(last_op) ? 2 : 1);
            cur = uint16_t(ops.back().pc + last_words);
            terminated = is_terminating(last_op);
            lin_cycles = ops.back().cyc_off;
            if(!terminated ||
               (last_op.func != INSTR_BRBS && last_op.func != INSTR_BRBC &&
                last_op.func != INSTR_SBRS && last_op.func != INSTR_SBRC &&
                last_op.func != INSTR_SBIS && last_op.func != INSTR_SBIC &&
                last_op.func != INSTR_CPSE))
            {
                lin_cycles += base_cycles(last_op);
            }

            uint8_t max_cycles = uint8_t(lin_cycles);
            if(terminated)
            {
                if(last_op.func == INSTR_BRBS || last_op.func == INSTR_BRBC)
                    max_cycles = uint8_t(lin_cycles + 2);
                else if(last_op.func == INSTR_SBRS || last_op.func == INSTR_SBRC ||
                        last_op.func == INSTR_SBIS || last_op.func == INSTR_SBIC ||
                        last_op.func == INSTR_CPSE)
                {
                    const uint8_t skip_cyc = last_op.m2 ? last_op.m2
                        : uint8_t(2 + (ops.back().pc + 1 < decoded.size() && instr_is_two_words(decoded[ops.back().pc + 1])));
                    max_cycles = uint8_t(lin_cycles + skip_cyc);
                }
            }

            // Backward dead-flag liveness pass over `ops`.
            uint8_t live = 0xff;
            uint8_t after_br_live = 0xff;
            if(terminated && (last_op.func == INSTR_BRBS || last_op.func == INSTR_BRBC))
            {
                const uint16_t taken_pc = uint16_t(ops.back().pc + int16_t(last_op.word) + 1) & 0x3fff;
                const uint16_t fall_pc = uint16_t(ops.back().pc + 1) & 0x3fff;
                after_br_live = uint8_t(peek_live_flags(decoded.data(), taken_pc) |
                                        peek_live_flags(decoded.data(), fall_pc));
                live = uint8_t(after_br_live | (1u << last_op.src));
            }
            else if(!terminated)
            {
                live = peek_live_flags(decoded.data(), cur & 0x3fff);
            }
            else if(last_op.func == INSTR_RJMP)
            {
                const uint16_t target_pc = uint16_t(ops.back().pc + int16_t(last_op.word) + 1) & 0x3fff;
                live = peek_live_flags(decoded.data(), target_pc);
            }
            else if(last_op.func == INSTR_JMP)
            {
                live = peek_live_flags(decoded.data(), last_op.word & 0x3fff);
            }

            for(int idx = int(ops.size()) - 1; idx >= 0; --idx)
            {
                ops[idx].live = live;
                const auto& insn = ops[idx].insn;
                const auto meta = get_flag_meta(insn);
                uint8_t reads = meta.reads;
                // SBC/SBCI/CPC only consume previous Z if their own output Z is live.
                if((insn.func == INSTR_SBC || insn.func == INSTR_SBCI || insn.func == INSTR_CPC) &&
                   (live & FLAG_Z) == 0)
                {
                    reads &= uint8_t(~FLAG_Z);
                }
                live = uint8_t((live & ~meta.writes) | reads);
            }

            // Check for fused ALU + BRBS/BRBC at block tail when post-branch flags are dead.
            bool fused_tail = false;
            if(terminated && ops.size() >= 2 &&
               (last_op.func == INSTR_BRBS || last_op.func == INSTR_BRBC))
            {
                const auto& prev = ops[ops.size() - 2].insn;
                const uint8_t br_bit = last_op.src;
                if((after_br_live & FLAGS_HSVNZC) == 0)
                {
                    if((prev.func == INSTR_CP || prev.func == INSTR_CPI) &&
                       (br_bit == 1 || br_bit == 0 || br_bit == 4))
                        fused_tail = true;
                    else if((prev.func == INSTR_AND || prev.func == INSTR_ANDI ||
                             prev.func == INSTR_OR || prev.func == INSTR_ORI ||
                             prev.func == INSTR_EOR || prev.func == INSTR_INC ||
                             prev.func == INSTR_DEC || prev.func == INSTR_ADD ||
                             prev.func == INSTR_SUB || prev.func == INSTR_SUBI) &&
                            br_bit == 1)
                        fused_tail = true;
                    else if((prev.func == INSTR_ADIW || prev.func == INSTR_SBIW) && br_bit == 1)
                        fused_tail = true;
                }
            }

            // Entry budget guard (BRI12 reaches +/-2047 bytes).
            w.addi(8, R_REM, -int8_t(max_cycles));
            const uint32_t entry_guard_ph = w.emit_bri12_placeholder(2, 8);

            auto& slow_stubs = scratch.slow_stubs;
            slow_stubs.clear();
            auto add_slow_branch = [&](uint32_t br_off, uint8_t kind, uint8_t s, uint16_t pc, uint16_t cyc_off) {
                slow_stubs.push_back({br_off, kind, s, pc, cyc_off});
            };
            bool checked_spm = false;

            for(size_t idx = 0; idx < ops.size(); ++idx)
            {
                const auto& bi = ops[idx];
                const auto& op = bi.insn;
                const bool is_fused_alu = fused_tail && (idx + 2 == ops.size());
                const uint8_t live = is_fused_alu ? 0 : bi.live;

            switch(op.func)
            {
            case INSTR_NOP:
                break;

            case INSTR_MOV:
                w.l8ui(8, R_DATA, op.src);
                w.s8i(8, R_DATA, op.dst);
                break;

            case INSTR_MOVW:
                w.l16ui(8, R_DATA, uint16_t(op.src * 2));
                w.s16i(8, R_DATA, uint16_t(op.dst * 2));
                break;

            case INSTR_LDI:
                w.load_imm(8, op.src);
                w.s8i(8, R_DATA, op.dst);
                break;

            case INSTR_CLR:
                w.movi_n(8, 0);
                w.s8i(8, R_DATA, op.dst);
                emit_set_logical_flags(w, 8, live, true);
                break;

            case INSTR_SWAP:
                w.l8ui(8, R_DATA, op.dst);
                w.extui(9, 8, 4, 4);
                w.slli(8, 8, 4);
                w.or_(8, 8, 9);
                w.s8i(8, R_DATA, op.dst);
                break;

            case INSTR_AND:
            case INSTR_OR:
            case INSTR_EOR:
                w.l8ui(8, R_DATA, op.dst);
                w.l8ui(9, R_DATA, op.src);
                if(op.func == INSTR_AND) w.and_(10, 8, 9);
                else if(op.func == INSTR_OR) w.or_(10, 8, 9);
                else w.xor_(10, 8, 9);
                w.s8i(10, R_DATA, op.dst);
                if(is_fused_alu) w.mov_n(11, 10);
                emit_set_logical_flags(w, 10, live, false);
                break;

            case INSTR_ANDI:
            case INSTR_ORI:
                w.l8ui(8, R_DATA, op.dst);
                w.load_imm(9, op.src);
                if(op.func == INSTR_ANDI) w.and_(10, 8, 9);
                else w.or_(10, 8, 9);
                w.s8i(10, R_DATA, op.dst);
                if(is_fused_alu) w.mov_n(11, 10);
                emit_set_logical_flags(w, 10, live, false);
                break;

            case INSTR_ADD:
            case INSTR_ADC:
                w.l8ui(8, R_DATA, op.dst);
                if(op.src == op.dst)
                {
                    if((live & (FLAG_H | FLAG_V | FLAG_S)) != 0)
                        w.mov_n(9, 8);
                    w.slli(10, 8, 1);
                }
                else
                {
                    w.l8ui(9, R_DATA, op.src);
                    w.add(10, 8, 9);
                }
                if(op.func == INSTR_ADC)
                {
                    w.extui(12, R_SREG, 0, 1);
                    w.add(10, 10, 12);
                }
                w.extui(11, 10, 0, 8);
                w.s8i(11, R_DATA, op.dst);
                emit_add_sub_flags(rt, w, false, false, live);
                break;

            case INSTR_SUB:
            case INSTR_SBC:
                w.l8ui(8, R_DATA, op.dst);
                w.l8ui(9, R_DATA, op.src);
                w.sub(10, 8, 9);
                if(op.func == INSTR_SBC)
                {
                    w.extui(12, R_SREG, 0, 1);
                    w.sub(10, 10, 12);
                }
                w.extui(11, 10, 0, 8);
                w.s8i(11, R_DATA, op.dst);
                emit_add_sub_flags(rt, w, true, op.func == INSTR_SBC, live);
                break;

            case INSTR_SUBI:
            case INSTR_SBCI:
                w.l8ui(8, R_DATA, op.dst);
                w.load_imm(9, op.src);
                w.sub(10, 8, 9);
                if(op.func == INSTR_SBCI)
                {
                    w.extui(12, R_SREG, 0, 1);
                    w.sub(10, 10, 12);
                }
                w.extui(11, 10, 0, 8);
                w.s8i(11, R_DATA, op.dst);
                emit_add_sub_flags(rt, w, true, op.func == INSTR_SBCI, live);
                break;

            case INSTR_CP:
            case INSTR_CPC:
                w.l8ui(8, R_DATA, op.dst);
                w.l8ui(9, R_DATA, op.src);
                if(!is_fused_alu || ops.back().insn.src == 1)
                    w.sub(10, 8, 9);
                if(op.func == INSTR_CPC)
                {
                    w.extui(12, R_SREG, 0, 1);
                    w.sub(10, 10, 12);
                }
                if(!is_fused_alu || ops.back().insn.src == 1)
                    w.extui(11, 10, 0, 8);
                emit_add_sub_flags(rt, w, true, op.func == INSTR_CPC, live);
                break;

            case INSTR_CPI:
                w.l8ui(8, R_DATA, op.dst);
                w.load_imm(9, op.src);
                if(!is_fused_alu || ops.back().insn.src == 1)
                {
                    w.sub(10, 8, 9);
                    w.extui(11, 10, 0, 8);
                }
                emit_add_sub_flags(rt, w, true, false, live);
                break;

            case INSTR_INC:
            case INSTR_DEC:
                w.l8ui(8, R_DATA, op.dst);
                w.addi_n(11, 8, (op.func == INSTR_DEC) ? -1 : 1);
                w.extui(11, 11, 0, 8);
                w.s8i(11, R_DATA, op.dst);
                emit_inc_dec_flags(w, op.func == INSTR_DEC, live);
                break;

            case INSTR_COM:
                w.l8ui(8, R_DATA, op.dst);
                w.movi_n(9, -1);
                w.xor_(10, 8, 9);
                w.extui(10, 10, 0, 8);
                w.s8i(10, R_DATA, op.dst);
                if(live & FLAGS_SVNZC)
                {
                    w.load_imm(11, 0xe0);
                    w.and_(R_SREG, R_SREG, 11);
                    w.addi_n(R_SREG, R_SREG, 1); // C = 1
                    if(live & FLAG_Z) { w.addi(11, R_SREG, 2); w.moveqz(R_SREG, 11, 10); }
                    const uint8_t ns = live & (FLAG_N | FLAG_S);
                    if(ns)
                    {
                        w.extui(11, 10, 7, 1);
                        if(ns == (FLAG_N | FLAG_S)) { w.addx4(11, 11, 11); w.slli(11, 11, 2); }
                        else if(ns == FLAG_N) w.slli(11, 11, 2);
                        else w.slli(11, 11, 4);
                        w.or_(R_SREG, R_SREG, 11);
                    }
                }
                break;

            case INSTR_NEG:
                w.movi_n(8, 0);
                w.l8ui(9, R_DATA, op.dst);
                w.sub(10, 8, 9);
                w.extui(11, 10, 0, 8);
                w.s8i(11, R_DATA, op.dst);
                // NEG flags match 0 - old except H=(res|old)[3] and C=(res!=0) (which is exact borrow 0-old[8]!)
                // Wait: in sub_flags for 0 - old: H = (0 ^ old ^ res)[4] vs (res | old)[3]!
                // Check exact H: for 0x08 -> res=0xF8 -> (0^0x08^0xF8)&0x10 = 0x10 (borrow4=1)! (res|old)&8 = 8! Match!
                // What about 0x05 -> res=0xFB -> 0^5^0xFB=0xFE -> bit4=1! (res|old)&8=8! Wait: 0x00 -> res=0: bit4=0, (res|old)&8=0!
                // Every value x has bit4 of (0 ^ x ^ (-x)) == bit3 of (x | (-x))! Verify: borrow into bit 4 iff low nibble != 0!
                emit_add_sub_flags(rt, w, true, false, live);
                break;

            case INSTR_LSR:
            case INSTR_ASR:
            case INSTR_ROR:
                w.l8ui(8, R_DATA, op.dst);
                if(op.func == INSTR_LSR)
                    w.srli(11, 8, 1);
                else if(op.func == INSTR_ASR)
                {
                    w.sext(11, 8, 7);
                    w.srai(11, 11, 1);
                    w.extui(11, 11, 0, 8);
                }
                else
                {
                    w.srli(11, 8, 1);
                    w.extui(12, R_SREG, 0, 1);
                    w.slli(12, 12, 7);
                    w.or_(11, 11, 12);
                }
                w.s8i(11, R_DATA, op.dst);
                emit_shift_flags(w, live);
                break;

            case INSTR_ADIW:
            case INSTR_SBIW:
                w.l16ui(8, R_DATA, op.dst);
                w.add_const(10, 8, (op.func == INSTR_SBIW) ? -int32_t(op.src) : int32_t(op.src));
                w.extui(11, 10, 0, 16);
                w.s16i(11, R_DATA, op.dst);
                emit_adiw_sbiw_flags(w, op.func == INSTR_SBIW, live);
                break;

            case INSTR_MUL:
            case INSTR_MULS:
            case INSTR_MULSU:
            case INSTR_FMUL:
            case INSTR_FMULS:
            case INSTR_FMULSU:
            {
                w.l8ui(8, R_DATA, op.dst);
                w.l8ui(9, R_DATA, op.src);
                if(op.func == INSTR_MULS || op.func == INSTR_MULSU ||
                   op.func == INSTR_FMULS || op.func == INSTR_FMULSU)
                    w.sext(8, 8, 7);
                if(op.func == INSTR_MULS || op.func == INSTR_FMULS)
                    w.sext(9, 9, 7);
                w.mull(10, 8, 9);
                w.extui(11, 10, 0, 16);
                const bool is_fmul = (op.func == INSTR_FMUL || op.func == INSTR_FMULS || op.func == INSTR_FMULSU);
                if(is_fmul)
                {
                    w.slli(12, 11, 1);
                    w.extui(12, 12, 0, 16);
                    w.s16i(12, R_DATA, 0);
                }
                else
                    w.s16i(11, R_DATA, 0);
                if(live & (FLAG_C | FLAG_Z))
                {
                    w.load_imm(12, 0xfc);
                    w.and_(R_SREG, R_SREG, 12);
                    if(live & FLAG_C)
                    {
                        w.extui(12, 11, 15, 1);
                        w.or_(R_SREG, R_SREG, 12);
                    }
                    if(live & FLAG_Z)
                    {
                        w.addi(12, R_SREG, 2);
                        w.moveqz(R_SREG, 12, 11);
                    }
                }
                break;
            }

            case INSTR_BLD:
            {
                uint8_t bit_idx = 0;
                while(bit_idx < 7 && (op.src >> bit_idx) != 1) ++bit_idx;
                w.l8ui(8, R_DATA, op.dst);
                w.load_imm(9, uint8_t(~op.src));
                w.and_(8, 8, 9);
                w.extui(9, R_SREG, 6, 1);
                w.slli(9, 9, bit_idx);
                w.or_(8, 8, 9);
                w.s8i(8, R_DATA, op.dst);
                break;
            }

            case INSTR_BST:
            {
                uint8_t bit_idx = 0;
                while(bit_idx < 7 && (op.src >> bit_idx) != 1) ++bit_idx;
                w.l8ui(8, R_DATA, op.dst);
                w.load_imm(9, uint8_t(~FLAG_T));
                w.and_(R_SREG, R_SREG, 9);
                w.extui(8, 8, bit_idx, 1);
                w.slli(8, 8, 6);
                w.or_(R_SREG, R_SREG, 8);
                break;
            }

            case INSTR_BSET:
                if((op.dst & FLAG_I) == 0)
                {
                    w.load_imm(8, op.dst);
                    w.or_(R_SREG, R_SREG, 8);
                }
                else
                {
                    w.extui(13, R_SREG, 7, 1);
                    w.load_imm(8, FLAG_I);
                    w.or_(R_SREG, R_SREG, 8);
                    const uint32_t already_ph = w.emit_bri12_placeholder(1, 13);
                    w.mov_n(10, R_CPU);
                    emit_call_helper(w, off, 5);
                    w.movi_n(8, -1);
                    w.s32i(8, R_CPU, off.irq_enable_rem);
                    add_slow_branch(w.emit_bri12_placeholder(1, 10), 2, 10,
                                    uint16_t((bi.pc + 1) & 0x3fff), uint16_t(bi.cyc_off + 1));
                    w.add_const(8, R_REM, -int32_t(bi.cyc_off + 1));
                    w.s32i(8, R_CPU, off.irq_enable_rem);
                    w.patch_bri12(already_ph, 1, 13, w.pos());
                }
                break;

            case INSTR_BCLR:
                if((op.dst & FLAG_I) == 0)
                {
                    w.extui(R_SREG, R_SREG, 0, 7);
                    w.l8ui(8, R_CPU, off.prev_sreg);
                    w.extui(8, 8, 0, 7);
                    w.s8i(8, R_CPU, off.prev_sreg);
                    w.movi_n(8, -1);
                    w.s32i(8, R_CPU, off.irq_enable_rem);
                }
                else
                {
                    w.load_imm(8, op.dst);
                    w.and_(R_SREG, R_SREG, 8);
                }
                break;

            case INSTR_LDS:
                if(op.word < 256)
                {
                    const uint16_t addr = op.word;
                    if(addr == SREG_ADDR)
                        w.s8i(R_SREG, R_DATA, op.dst);
                    else if(!cpu.ld_handlers[addr])
                    {
                        w.l8ui(8, R_DATA, addr);
                        w.s8i(8, R_DATA, op.dst);
                    }
                    else if(addr == reg::addr::SPDR || addr == reg::addr::SPSR)
                    {
                        w.mov_n(10, R_CPU);
                        w.add_const(11, R_REM, -int32_t(bi.cyc_off));
                        emit_call_helper(w, off, addr == reg::addr::SPDR ? 1 : 2);
                        w.s8i(10, R_DATA, op.dst);
#ifndef ESP_PLATFORM
                        w.extui(11, 10, 8, 1);
                        add_slow_branch(w.emit_bri12_placeholder(1, 11), 2, 11,
                                        uint16_t((bi.pc + 2) & 0x3fff), uint16_t(bi.cyc_off + 2));
#endif
                    }
                    else if(addr == reg::addr::TCNT0)
                    {
                        w.mov_n(10, R_CPU);
                        w.add_const(11, R_REM, -int32_t(bi.cyc_off));
                        emit_call_helper(w, off, 4);
                        w.extui(11, 10, 8, 1);
                        add_slow_branch(w.emit_bri12_placeholder(1, 11), 2, 11, bi.pc, bi.cyc_off);
                        w.s8i(10, R_DATA, op.dst);
                    }
                    break;
                }
                w.load_imm(8, op.word);
                w.add(8, R_DATA, 8);
                w.l8ui(8, 8, 0);
                w.s8i(8, R_DATA, op.dst);
                break;

            case INSTR_STS:
                if(op.word < 256)
                {
                    const uint16_t addr = op.word;
                    if(!cpu.st_handlers[addr])
                    {
                        w.l8ui(8, R_DATA, op.src);
                        w.s8i(8, R_DATA, addr);
                    }
                    else if(addr == reg::addr::SPDR)
                    {
                        w.mov_n(10, R_CPU);
                        w.l8ui(11, R_DATA, op.src);
                        w.add_const(12, R_REM, -int32_t(bi.cyc_off));
                        emit_call_helper(w, off, 0);
                    }
                    else
                    {
                        const uint32_t is_pin = (cpu.st_handlers[addr] == atmega32u4_t::st_handle_pin) ? 1u : 0u;
                        w.mov_n(10, R_CPU);
                        w.l8ui(13, R_DATA, op.src);
                        load_u32(w, 11, (is_pin << 24) | (uint32_t(addr) << 8));
                        w.or_(11, 11, 13);
                        w.add_const(12, R_REM, -int32_t(bi.cyc_off));
                        emit_call_helper(w, off, 3);
                    }
                    break;
                }
                w.l8ui(8, R_DATA, op.src);
                w.load_imm(9, op.word);
                w.add(9, R_DATA, 9);
                w.s8i(8, 9, 0);
                break;

            case INSTR_LDD_Y:
            case INSTR_LDD_Z:
            case INSTR_STD_Y:
            case INSTR_STD_Z:
            {
                const bool is_y = (op.func == INSTR_LDD_Y || op.func == INSTR_STD_Y);
                const uint16_t ptr_off = is_y ? 28 : 30;
                auto same_ptr = [&](uint8_t f) {
                    return is_y ? (f == INSTR_LDD_Y || f == INSTR_STD_Y)
                                : (f == INSTR_LDD_Z || f == INSTR_STD_Z);
                };
                auto op_is_load = [](uint8_t f) {
                    return f == INSTR_LDD_Y || f == INSTR_LDD_Z;
                };

                size_t batch_end = idx;
                uint16_t min_q = op.dst;
                uint16_t max_q = op.dst;
                while(batch_end < ops.size() && same_ptr(ops[batch_end].insn.func))
                {
                    const auto& cur_op = ops[batch_end].insn;
                    min_q = std::min<uint16_t>(min_q, cur_op.dst);
                    max_q = std::max<uint16_t>(max_q, cur_op.dst);
                    ++batch_end;
                    if(op_is_load(cur_op.func) &&
                       (cur_op.src == ptr_off || cur_op.src == ptr_off + 1))
                        break;
                }

                w.l16ui(8, R_DATA, ptr_off);
                if(min_q != 0)
                {
                    w.add_const(8, 8, min_q);
                    w.extui(8, 8, 0, 16);
                }
                w.addmi(13, 8, -256);
                w.srli(14, 13, 9);
                add_slow_branch(w.bgeui_ph(14, XtensaWriter::b4constu_enc(5)), 0, 14, bi.pc, bi.cyc_off);
                if(max_q > min_q)
                {
                    w.addi(12, 13, int8_t(max_q - min_q));
                    w.srli(14, 12, 9);
                    add_slow_branch(w.bgeui_ph(14, XtensaWriter::b4constu_enc(5)), 0, 14, bi.pc, bi.cyc_off);
                }
                w.add(9, R_DATA, 8);
                for(size_t m = idx; m < batch_end; ++m)
                {
                    const auto& cur_op = ops[m].insn;
                    const uint16_t rel = uint16_t(cur_op.dst - min_q);
                    if(op_is_load(cur_op.func))
                    {
                        w.l8ui(8, 9, rel);
                        w.s8i(8, R_DATA, cur_op.src);
                    }
                    else
                    {
                        w.l8ui(8, R_DATA, cur_op.src);
                        w.s8i(8, 9, rel);
                    }
                }
                idx = batch_end - 1;
                break;
            }

            case INSTR_LD_ST:
            case INSTR_LD_X: case INSTR_LD_Y: case INSTR_LD_Z:
            case INSTR_LD_X_INC: case INSTR_LD_Y_INC: case INSTR_LD_Z_INC:
            case INSTR_LD_X_DEC: case INSTR_LD_Y_DEC: case INSTR_LD_Z_DEC:
            case INSTR_ST_X: case INSTR_ST_Y: case INSTR_ST_Z:
            case INSTR_ST_X_INC: case INSTR_ST_Y_INC: case INSTR_ST_Z_INC:
            case INSTR_ST_X_DEC: case INSTR_ST_Y_DEC: case INSTR_ST_Z_DEC:
            {
                uint16_t ptr_off = 26;
                if(op.func == INSTR_LD_ST)
                {
                    if(op.dst <= 2) ptr_off = 30;
                    else if(op.dst <= 10) ptr_off = 28;
                }
                else
                {
                    const int m = (op.func >= INSTR_ST_X) ? (op.func - INSTR_ST_X) : (op.func - INSTR_LD_X);
                    const int r = m % 3;
                    ptr_off = (r == 0) ? 26 : (r == 1 ? 28 : 30);
                }
                const bool is_load = (op.func == INSTR_LD_ST) ? (op.word == 0) : (op.func < INSTR_ST_X);
                const bool pre_dec = (op.dst & 0x2) != 0;
                const bool post_inc = (op.dst & 0x1) != 0;
                w.l16ui(8, R_DATA, ptr_off);
                if(pre_dec)
                {
                    w.addi_n(8, 8, -1);
                    w.extui(8, 8, 0, 16);
                }
                auto slow_ph = emit_sram_bounds_check(w, 8);
                add_slow_branch(slow_ph, 0, 14, bi.pc, bi.cyc_off);
                w.add(9, R_DATA, 8);
                if(is_load)
                {
                    w.l8ui(10, 9, 0);
                    w.s8i(10, R_DATA, op.src);
                }
                else
                {
                    w.l8ui(10, R_DATA, op.src);
                    w.s8i(10, 9, 0);
                }
                if(post_inc)
                    w.addi_n(8, 8, 1);
                if(pre_dec || post_inc)
                    w.s16i(8, R_DATA, ptr_off);
                break;
            }

            case INSTR_LPM:
            {
                if(!checked_spm)
                {
                    w.l8ui(9, R_CPU, off.spm_active);
                    add_slow_branch(w.emit_bri12_placeholder(1, 9), 2, 9, bi.pc, bi.cyc_off);
                    checked_spm = true;
                }
                w.l16ui(8, R_DATA, 30);
                w.extui(9, 8, 15, 1);
                w.movi_n(10, 0);
                const uint32_t oob_ph = w.emit_bri12_placeholder(1, 9);
                w.l32i(11, R_CPU, off.prog_ptr);
                w.add(11, 11, 8);
                w.l8ui(10, 11, 0);
                w.patch_bri12(oob_ph, 1, 9, w.pos());
                const uint8_t dst_reg = (op.word == 2) ? 0 : op.dst;
                w.s8i(10, R_DATA, dst_reg);
                if(op.word == 1)
                {
                    w.addi_n(8, 8, 1);
                    w.s16i(8, R_DATA, 30);
                }
                break;
            }

            case INSTR_PUSH:
            case INSTR_POP:
            {
                const bool is_push = (op.func == INSTR_PUSH);
                size_t batch_end = idx + 1;
                while(batch_end < ops.size() && ops[batch_end].insn.func == op.func)
                    ++batch_end;
                const uint32_t count = uint32_t(batch_end - idx);

                w.l8ui(8, R_DATA, SPL_ADDR);
                w.l8ui(9, R_DATA, SPH_ADDR);
                w.slli(9, 9, 8);
                w.or_(8, 8, 9);
                if(!is_push)
                {
                    w.addi_n(8, 8, 1);
                    w.extui(8, 8, 0, 16);
                }
                w.addmi(13, 8, -256);
                w.srli(14, 13, 9);
                add_slow_branch(w.bgeui_ph(14, XtensaWriter::b4constu_enc(5)), 0, 14, bi.pc, bi.cyc_off);
                if(is_push)
                {
                    if(count > 1)
                    {
                        w.addi(12, 13, -int8_t(count - 1));
                        add_slow_branch(w.emit_bri12_placeholder(2, 12), 3, 12, bi.pc, bi.cyc_off);
                    }
                    w.add(9, R_DATA, 8);
                    if(count > 1)
                        w.addi(9, 9, -int8_t(count - 1));
                    for(uint32_t m = 0; m < count; ++m)
                    {
                        w.l8ui(10, R_DATA, ops[idx + m].insn.src);
                        w.s8i(10, 9, uint16_t((count - 1) - m));
                    }
                    w.add_const(8, 8, -int32_t(count));
                }
                else
                {
                    if(count > 1)
                    {
                        w.addi(12, 13, int8_t(count - 1));
                        w.srli(14, 12, 9);
                        add_slow_branch(w.bgeui_ph(14, XtensaWriter::b4constu_enc(5)), 0, 14, bi.pc, bi.cyc_off);
                    }
                    w.add(9, R_DATA, 8);
                    for(uint32_t m = 0; m < count; ++m)
                    {
                        w.l8ui(10, 9, uint16_t(m));
                        w.s8i(10, R_DATA, ops[idx + m].insn.src);
                    }
                    if(count > 1)
                        w.add_const(8, 8, int32_t(count - 1));
                }
                w.s8i(8, R_DATA, SPL_ADDR);
                w.srli(9, 8, 8);
                w.s8i(9, R_DATA, SPH_ADDR);
                idx = batch_end - 1;
                break;
            }

            case INSTR_IN:
            {
                const uint16_t addr = op.src + 32;
                if(addr == SREG_ADDR)
                {
                    w.s8i(R_SREG, R_DATA, op.dst);
                }
                else if(!cpu.ld_handlers[addr])
                {
                    w.l8ui(8, R_DATA, addr);
                    w.s8i(8, R_DATA, op.dst);
                }
                else if(addr == reg::addr::SPDR || addr == reg::addr::SPSR)
                {
#ifdef ARDUGO_EMBEDDED
                    if(addr == reg::addr::SPDR && idx + 1 < ops.size() &&
                       ops[idx + 1].insn.func == INSTR_OUT &&
                       (ops[idx + 1].insn.dst + 32) == reg::addr::SPDR &&
                       ops[idx + 1].insn.src != op.dst)
                    {
                        w.mov_n(10, R_CPU);
                        w.l8ui(11, R_DATA, ops[idx + 1].insn.src);
                        w.add_const(12, R_REM, -int32_t(bi.cyc_off));
                        emit_call_helper(w, off, 15);
                        w.s8i(10, R_DATA, op.dst);
                        ++idx;
                        break;
                    }
#endif
                    w.mov_n(10, R_CPU);
                    w.add_const(11, R_REM, -int32_t(bi.cyc_off));
                    emit_call_helper(w, off, addr == reg::addr::SPDR ? 1 : 2);
                    w.s8i(10, R_DATA, op.dst);
#ifndef ESP_PLATFORM
                    w.extui(11, 10, 8, 1);
                    add_slow_branch(w.emit_bri12_placeholder(1, 11), 2, 11,
                                    uint16_t((bi.pc + 1) & 0x3fff), uint16_t(bi.cyc_off + 1));
#endif
                }
                else if(addr == reg::addr::TCNT0)
                {
                    w.mov_n(10, R_CPU);
                    w.add_const(11, R_REM, -int32_t(bi.cyc_off));
                    emit_call_helper(w, off, 4);
                    w.extui(11, 10, 8, 1);
                    add_slow_branch(w.emit_bri12_placeholder(1, 11), 2, 11, bi.pc, bi.cyc_off);
                    w.s8i(10, R_DATA, op.dst);
                }
                break;
            }

            case INSTR_OUT:
            {
                const uint16_t addr = op.dst + 32;
                if(addr == SREG_ADDR)
                {
                    w.extui(13, R_SREG, 7, 1); // old_i
                    w.l8ui(R_SREG, R_DATA, op.src);
                    w.extui(14, R_SREG, 7, 1); // new_i
                    w.xor_(8, 13, 14);
                    const uint32_t unchanged_ph = w.emit_bri12_placeholder(0, 8);
                    const uint32_t disable_ph = w.emit_bri12_placeholder(0, 14);
                    // 0 -> 1
                    w.mov_n(10, R_CPU);
                    emit_call_helper(w, off, 5);
                    w.movi_n(8, -1);
                    w.s32i(8, R_CPU, off.irq_enable_rem);
                    add_slow_branch(w.emit_bri12_placeholder(1, 10), 2, 10,
                                    uint16_t((bi.pc + 1) & 0x3fff), uint16_t(bi.cyc_off + 1));
                    w.add_const(8, R_REM, -int32_t(bi.cyc_off + 1));
                    w.s32i(8, R_CPU, off.irq_enable_rem);
                    const uint32_t skip_dis_ph = w.emit_j_placeholder();
                    w.patch_bri12(disable_ph, 0, 14, w.pos());
                    // 1 -> 0
                    w.l8ui(8, R_CPU, off.prev_sreg);
                    w.extui(8, 8, 0, 7);
                    w.s8i(8, R_CPU, off.prev_sreg);
                    w.movi_n(8, -1);
                    w.s32i(8, R_CPU, off.irq_enable_rem);
                    w.patch_j(skip_dis_ph, w.pos());
                    w.patch_bri12(unchanged_ph, 0, 8, w.pos());
                }
                else if(!cpu.st_handlers[addr])
                {
                    w.l8ui(8, R_DATA, op.src);
                    w.s8i(8, R_DATA, addr);
                }
                else if(addr == reg::addr::SPDR)
                {
                    w.mov_n(10, R_CPU);
                    w.l8ui(11, R_DATA, op.src);
                    w.add_const(12, R_REM, -int32_t(bi.cyc_off));
                    emit_call_helper(w, off, 0);
                }
                else
                {
                    const uint32_t is_pin = (cpu.st_handlers[addr] == atmega32u4_t::st_handle_pin) ? 1u : 0u;
                    w.mov_n(10, R_CPU);
                    w.l8ui(13, R_DATA, op.src);
                    load_u32(w, 11, (is_pin << 24) | (uint32_t(addr) << 8));
                    w.or_(11, 11, 13);
                    w.add_const(12, R_REM, -int32_t(bi.cyc_off));
                    emit_call_helper(w, off, 3);
                }
                break;
            }

            case INSTR_SBI:
            case INSTR_CBI:
            {
                const uint16_t addr = op.dst + 32;
                w.l8ui(8, R_DATA, addr);
                if(op.func == INSTR_SBI)
                {
                    w.load_imm(9, op.src);
                    w.or_(8, 8, 9);
                }
                else
                {
                    w.load_imm(9, uint8_t(~op.src));
                    w.and_(8, 8, 9);
                }
                if(!cpu.st_handlers[addr])
                {
                    w.s8i(8, R_DATA, addr);
                }
                else
                {
                    w.mov_n(10, R_CPU);
                    w.load_imm(11, int32_t(addr << 8));
                    w.or_(11, 11, 8);
                    w.add_const(12, R_REM, -int32_t(bi.cyc_off));
                    emit_call_helper(w, off, 3);
                }
                break;
            }

            case INSTR_MERGED_DELAY_CALL:
            {
                if(op.m2 && !checked_spm)
                {
                    w.l8ui(9, R_CPU, off.spm_active);
                    add_slow_branch(w.emit_bri12_placeholder(1, 9), 2, 9, bi.pc, bi.cyc_off);
                    checked_spm = true;
                }
                w.l8ui(8, R_DATA, SPL_ADDR);
                w.l8ui(9, R_DATA, SPH_ADDR);
                w.slli(9, 9, 8);
                w.or_(8, 8, 9);
                w.addmi(13, 8, -256);
                add_slow_branch(w.emit_bri12_placeholder(0, 13), 1, 13, bi.pc, bi.cyc_off);
                w.srli(14, 13, 9);
                add_slow_branch(w.bgeui_ph(14, XtensaWriter::b4constu_enc(5)), 0, 14, bi.pc, bi.cyc_off);

                const uint16_t ret_pc = uint16_t((bi.pc + op.m0) & 0x3fffu);
                w.add(9, R_DATA, 8);
                w.load_u14(10, ret_pc);
                w.s8i(10, 9, 0);
                w.srli(11, 10, 8);
                w.addi_n(9, 9, -1);
                w.s8i(11, 9, 0);

                if(op.m2)
                {
                    w.l16ui(8, R_DATA, 30);
                    w.extui(9, 8, 15, 1);
                    w.movi_n(10, 0);
                    const uint32_t oob_ph = w.emit_bri12_placeholder(1, 9);
                    w.l32i(11, R_CPU, off.prog_ptr);
                    w.add(11, 11, 8);
                    w.l8ui(10, 11, 0);
                    w.patch_bri12(oob_ph, 1, 9, w.pos());
                    w.s8i(10, R_DATA, 0);
                }
                break;
            }

            case INSTR_RJMP:
            case INSTR_JMP:
            {
                if(op.func == INSTR_RJMP && op.word == 0)
                    break;
                const uint16_t tgt = (op.func == INSTR_RJMP)
                    ? uint16_t(bi.pc + int16_t(op.word) + 1) & 0x3fff
                    : (op.word & 0x3fff);
                w.add_const(R_REM, R_REM, -int32_t(max_cycles));
                emit_direct_jump(rt, w, tgt, start_pc, block_off_bytes);
                break;
            }

            case INSTR_IJMP:
                w.l16ui(8, R_DATA, 30);
                w.add_const(R_REM, R_REM, -int32_t(max_cycles));
                w.j(rt.dispatch_off);
                break;

            case INSTR_RCALL:
            case INSTR_CALL:
            case INSTR_ICALL:
            {
                if(op.func == INSTR_ICALL)
                    w.l16ui(10, R_DATA, 30);
                if(bi.cyc_off != 0)
                    w.add_const(R_REM, R_REM, -int32_t(bi.cyc_off));
                w.load_u14(8, bi.pc);
                if(op.func != INSTR_ICALL)
                {
                    const uint16_t tgt = (op.func == INSTR_RCALL)
                        ? uint16_t(bi.pc + int16_t(op.word) + 1) & 0x3fff
                        : (op.word & 0x3fff);
                    w.load_u14(10, tgt);
                }
                w.j(op.func == INSTR_CALL ? rt.call2_off : rt.call1_off);
                break;
            }

            case INSTR_RET:
            {
                if(bi.cyc_off != 0)
                    w.add_const(R_REM, R_REM, -int32_t(bi.cyc_off));
                w.load_u14(8, bi.pc);
                w.j(rt.ret_off);
                break;
            }

            case INSTR_RETI:
            {
                w.l8ui(8, R_DATA, SPL_ADDR);
                w.l8ui(9, R_DATA, SPH_ADDR);
                w.slli(9, 9, 8);
                w.or_(8, 8, 9);
                w.addi_n(12, 8, 2);
                w.extui(12, 12, 0, 16);
                w.addmi(13, 12, -256);
                add_slow_branch(w.emit_bri12_placeholder(0, 13), 1, 13, bi.pc, bi.cyc_off);
                w.srli(14, 13, 9);
                add_slow_branch(w.bgeui_ph(14, XtensaWriter::b4constu_enc(5)), 0, 14, bi.pc, bi.cyc_off);

                w.add(9, R_DATA, 12);
                w.l8ui(10, 9, 0);
                w.addi_n(9, 9, -1);
                w.l8ui(15, 9, 0);
                w.slli(15, 15, 8);
                w.or_(15, 15, 10); // ret_pc in a15
                w.s8i(12, R_DATA, SPL_ADDR);
                w.srli(9, 12, 8);
                w.s8i(9, R_DATA, SPH_ADDR);

                w.extui(13, R_SREG, 7, 1);
                w.load_imm(9, FLAG_I);
                w.or_(R_SREG, R_SREG, 9);
                const uint32_t already_on_ph = w.emit_bri12_placeholder(1, 13);
                w.s16i(15, R_CPU, off.pc); // callx8 clobbers a8..a15; preserve ret_pc across helper call
                w.mov_n(10, R_CPU);
                emit_call_helper(w, off, 5);
                w.movi_n(8, -1);
                w.s32i(8, R_CPU, off.irq_enable_rem);
                const uint32_t irq_pend_ph = w.emit_bri12_placeholder(1, 10);
                w.add_const(8, R_REM, -int32_t(max_cycles));
                w.s32i(8, R_CPU, off.irq_enable_rem);
                w.l16ui(15, R_CPU, off.pc);
                const uint32_t skip_pend_ph = w.emit_j_placeholder();
                w.patch_bri12(irq_pend_ph, 1, 10, w.pos());
                w.l16ui(8, R_CPU, off.pc);
                w.add_const(R_REM, R_REM, -int32_t(max_cycles));
                w.j(rt.slow_off);
                w.patch_j(skip_pend_ph, w.pos());
                w.patch_bri12(already_on_ph, 1, 13, w.pos());
                w.add_const(R_REM, R_REM, -int32_t(max_cycles));
                w.mov_n(8, 15);
                w.j(rt.dispatch_off);
                break;
            }

            case INSTR_BRBS:
            case INSTR_BRBC:
            {
                const uint16_t taken_pc = uint16_t(bi.pc + int16_t(op.word) + 1) & 0x3fff;
                const uint16_t fall_pc = uint16_t(bi.pc + 1) & 0x3fff;
                const bool brbs = (op.func == INSTR_BRBS);
                uint32_t taken_ph = 0;
                uint8_t br_kind = 0;
                uint8_t b_r = 0, b_s = 0, b_t = 0;

                if(fused_tail)
                {
                    const uint8_t bit = op.src;
                    if(bit == 1)
                    {
                        br_kind = 1;
                        b_r = brbs ? 0 : 1;
                        b_s = 11;
                        taken_ph = w.emit_bri12_placeholder(b_r, b_s);
                    }
                    else if(bit == 0)
                    {
                        br_kind = 0;
                        b_r = brbs ? 3 : 11;
                        b_s = 8;
                        b_t = 9;
                        taken_ph = brbs ? w.bltu_ph(8, 9) : w.bgeu_ph(8, 9);
                    }
                    else
                    {
                        w.sext(8, 8, 7);
                        w.sext(9, 9, 7);
                        br_kind = 0;
                        b_r = brbs ? 2 : 10;
                        b_s = 8;
                        b_t = 9;
                        taken_ph = brbs ? w.blt_ph(8, 9) : w.bge_ph(8, 9);
                    }
                }
                else
                {
                    br_kind = 0;
                    b_r = uint8_t((brbs ? 14 : 6) | ((op.src >> 4) & 1));
                    b_s = R_SREG;
                    b_t = op.src & 0xf;
                    taken_ph = brbs ? w.bbsi_ph(R_SREG, op.src) : w.bbci_ph(R_SREG, op.src);
                }

                w.add_const(R_REM, R_REM, -int32_t(lin_cycles + 1));
                emit_direct_jump(rt, w, fall_pc, start_pc, block_off_bytes);

                if(br_kind == 1)
                    w.patch_bri12(taken_ph, b_r, b_s, w.pos());
                else
                    w.patch_br8(taken_ph, b_r, b_s, b_t, w.pos(), 7);
                w.add_const(R_REM, R_REM, -int32_t(lin_cycles + 2));
                emit_direct_jump(rt, w, taken_pc, start_pc, block_off_bytes);
                break;
            }

            case INSTR_SBRS:
            case INSTR_SBRC:
            case INSTR_SBIS:
            case INSTR_SBIC:
            case INSTR_CPSE:
            {
                const uint8_t skip_words = op.m2 ? op.m2
                    : uint8_t(2 + (bi.pc + 1 < decoded.size() && instr_is_two_words(decoded[bi.pc + 1])));
                const uint16_t skip_pc = uint16_t(bi.pc + skip_words) & 0x3fff;
                const uint16_t fall_pc = uint16_t(bi.pc + 1) & 0x3fff;
                uint32_t skip_ph = 0;
                uint8_t br_r = 0, br_s = 0, br_t = 0;
                if(op.func == INSTR_CPSE)
                {
                    w.l8ui(8, R_DATA, op.dst);
                    w.l8ui(9, R_DATA, op.src);
                    br_r = 1; br_s = 8; br_t = 9;
                    skip_ph = w.beq_ph(8, 9);
                }
                else
                {
                    const bool is_io = (op.func == INSTR_SBIS || op.func == INSTR_SBIC);
                    const bool on_set = (op.func == INSTR_SBRS || op.func == INSTR_SBIS);
                    uint8_t bit_idx = 0;
                    while(bit_idx < 7 && (op.src >> bit_idx) != 1) ++bit_idx;
                    w.l8ui(8, R_DATA, is_io ? (op.dst + 32) : op.dst);
                    br_r = uint8_t(on_set ? 14 : 6);
                    br_s = 8;
                    br_t = bit_idx;
                    skip_ph = on_set ? w.bbsi_ph(8, bit_idx) : w.bbci_ph(8, bit_idx);
                }
                w.add_const(R_REM, R_REM, -int32_t(lin_cycles + 1));
                emit_direct_jump(rt, w, fall_pc, start_pc, block_off_bytes);

                w.patch_br8(skip_ph, br_r, br_s, br_t, w.pos(), 7);
                w.add_const(R_REM, R_REM, -int32_t(lin_cycles + skip_words));
                emit_direct_jump(rt, w, skip_pc, start_pc, block_off_bytes);
                break;
            }

            default:
                break;
            }
        }

            if(!terminated)
            {
                w.add_const(R_REM, R_REM, -int32_t(lin_cycles));
                emit_direct_jump(rt, w, cur & 0x3fff, start_pc, block_off_bytes);
            }

            // Emit mid-block slow exit stubs before entry guard to minimize 8-bit branch distance.
            bool branch_ok = true;
            for(const auto& stub : slow_stubs)
            {
                const uint32_t lbl = w.pos();
                const int32_t disp = int32_t(lbl) - int32_t(stub.branch_off) - 4;
                if(stub.kind == 0)
                {
                    if(disp < -128 || disp > 127) { branch_ok = false; break; }
                    w.patch_br8(stub.branch_off, uint8_t(XtensaWriter::b4constu_enc(5)), stub.s, 15, lbl, 6);
                }
                else
                {
                    if(disp < -2048 || disp > 2047) { branch_ok = false; break; }
                    w.patch_bri12(stub.branch_off, uint8_t(stub.kind - 1), stub.s, lbl);
                }
                if(stub.cyc_off != 0)
                    w.add_const(R_REM, R_REM, -int32_t(stub.cyc_off));
                w.load_u14(8, stub.target_pc);
                w.j(rt.slow_off);
            }

            const int32_t guard_disp = int32_t(w.pos()) - int32_t(entry_guard_ph) - 4;
            if(w.overflow() || !branch_ok || guard_disp < -2048 || guard_disp > 2047)
            {
                ops.pop_back();
                continue;
            }

            // Entry budget guard failure stub (rem_budget < max_cycles)
            w.patch_bri12(entry_guard_ph, 2, 8, w.pos());
            w.load_u14(8, start_pc);
            w.j(rt.slow_off);

            w.align4();
            if(w.overflow() || rel_off + w.bytes().size() > arena.cap)
            {
                rt.patches.resize(try_patch_count);
                arena.used = arena.cap;
                rt.cur_arena = uint8_t((rt.cur_arena + 1) % rt.arena_count);
                ++scanned;
                need_next_arena = true;
                break;
            }
            write_block_rt(rt, arena, rel_off, w.bytes().data(), w.bytes().size(), start_pc);
            link_committed_patches(rt, try_patch_count);
            break;
        }
        if(need_next_arena)
            continue;
        if(ops.empty())
        {
            rt.patches.resize(try_patch_count);
            return false;
        }
        break;
    }
    }

    if(rt.block_off[start_pc] == 0)
        return false;

    // Backpatch pending jumps targeting start_pc in O(1) bucket time (same arena only).
    const auto& dst_arena = rt.arenas[rt.cur_arena];
    const uint32_t da_lo = dst_arena.base_off + (rt.cur_arena == 0 ? rt.stubs_end : 0u);
    const uint32_t da_hi = dst_arena.base_off + dst_arena.cap;
    const uint32_t target_off = block_off_bytes;
    int16_t cur_patch = rt.patch_head[start_pc & 511u];
    while(cur_patch >= 0)
    {
        int16_t next_patch = rt.patches[cur_patch].next;
        if(rt.patches[cur_patch].target_pc == start_pc)
        {
            const uint32_t code_off = rt.patches[cur_patch].code_off;
            if(code_off >= da_lo && code_off + 3 <= da_hi &&
               in_j_range(code_off, target_off))
                patch24_rt(rt, code_off, XtensaWriter::encode_j(code_off, target_off));

            auto& p = rt.patches[cur_patch];
            const uint16_t b = p.target_pc & 511u;
            if(p.prev >= 0) rt.patches[p.prev].next = p.next;
            else rt.patch_head[b] = p.next;
            if(p.next >= 0) rt.patches[p.next].prev = p.prev;

            const int16_t last = int16_t(rt.patches.size() - 1);
            if(cur_patch != last)
            {
                rt.patches[cur_patch] = rt.patches[last];
                auto& moved = rt.patches[cur_patch];
                if(moved.prev >= 0) rt.patches[moved.prev].next = cur_patch;
                else rt.patch_head[moved.target_pc & 511u] = cur_patch;
                if(moved.next >= 0) rt.patches[moved.next].prev = cur_patch;
                if(next_patch == last)
                    next_patch = cur_patch;
            }
            rt.patches.pop_back();
        }
        cur_patch = next_patch;
    }
    sync_icache();
    return true;
}

} // namespace jit
} // namespace absim
