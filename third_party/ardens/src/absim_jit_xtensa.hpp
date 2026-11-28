#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

namespace absim
{

struct atmega32u4_t;

namespace jit
{

struct JitOffsets
{
    uint16_t cycle_count;
    uint16_t batch_end;
    uint16_t helpers;
    uint16_t prog_ptr;
    uint16_t irq_enable_rem;
    uint16_t spm_active;
    uint16_t pc;
    uint16_t prev_sreg;
    uint16_t active;
    uint16_t io_reg_accessed;
};

struct JitPatch
{
    uint32_t code_off;
    uint16_t target_pc;
    int16_t prev;
    int16_t next;
};

struct CodeArena
{
    uint32_t* buf = nullptr;
    uintptr_t base_addr = 0;
    uint32_t base_off = 0;
    uint32_t cap = 0;
    uint32_t used = 0;
};

struct JitRuntime
{
    using EnterFn = uint32_t (*)(atmega32u4_t* cpu, uint8_t* data, int32_t budget,
                                 uint32_t sreg, uintptr_t code_base, const uint16_t* block_table);
    static constexpr uint8_t kMaxArenas = 8;

    CodeArena arenas[kMaxArenas]{};
    uint8_t arena_count = 0;
    uint8_t cur_arena = 0;
    uint8_t next_evict_arena = 0;
    uint8_t secondary_evicts = 0;
    uint32_t* code_buf = nullptr;
    uintptr_t code_base_addr = 0;
    uint16_t* block_off = nullptr;
    uint8_t* miss_counts = nullptr;
    uint32_t code_cap = 0;
    uint32_t code_used = 0;
    uint32_t stubs_end = 0;
    uint32_t entry_off = 0;
    uint32_t dispatch_off = 0;
    uint32_t miss_off = 0;
    uint32_t slow_off = 0;
    uint32_t call1_off = 0;
    uint32_t call2_off = 0;
    uint32_t ret_off = 0;
    uint32_t flag_add_off = 0;
    uint32_t flag_sub_off = 0;
    uint32_t flag_sbc_off = 0;
    uint32_t owner_gen = 0;
    uintptr_t owner_cpu = 0;
    uint32_t stat_compiles = 0;
    uint32_t stat_resets = 0;
    uint32_t stat_entries = 0;
    uint32_t stat_misses = 0;
    uint32_t stat_fallbacks = 0;
    uint32_t last_full_reset_compiles = 0;
    bool boot_reclaimed = false;
    bool block_in_internal = false;
    bool failed = false;
    std::array<int16_t, 512> patch_head{};
    std::vector<JitPatch> patches;
#ifndef ESP_PLATFORM
    std::vector<uint32_t> host_code;
    std::array<uint16_t, 16384> host_blocks{};
#endif

    EnterFn entry_fn() const
    {
        return reinterpret_cast<EnterFn>(code_base_addr + entry_off);
    }
};

class XtensaCodeBuf
{
public:
    void clear() { len_ = 0; overflow_ = false; }
    void push_back(uint8_t b)
    {
        if(len_ < sizeof(data_))
            data_[len_++] = b;
        else
            overflow_ = true;
    }
    [[gnu::always_inline]] void push16(uint16_t v)
    {
        const uint32_t n = len_;
        if(n <= sizeof(data_) - 2)
        {
            data_[n + 0] = uint8_t(v & 0xff);
            data_[n + 1] = uint8_t(v >> 8);
            len_ = n + 2;
        }
        else
            overflow_ = true;
    }
    [[gnu::always_inline]] void push24(uint32_t v)
    {
        const uint32_t n = len_;
        if(n <= sizeof(data_) - 3)
        {
            data_[n + 0] = uint8_t(v & 0xff);
            data_[n + 1] = uint8_t((v >> 8) & 0xff);
            data_[n + 2] = uint8_t((v >> 16) & 0xff);
            len_ = n + 3;
        }
        else
            overflow_ = true;
    }
    uint32_t size() const { return len_; }
    const uint8_t* data() const { return data_; }
    uint8_t operator[](size_t i) const { return data_[i]; }
    uint8_t& operator[](size_t i) { return data_[i]; }
    bool overflow() const { return overflow_; }

private:
    uint8_t data_[2048]{};
    uint32_t len_ = 0;
    bool overflow_ = false;
};

class XtensaWriter
{
public:
    explicit XtensaWriter(uint32_t base_off = 0) : base_off_(base_off) {}

    void reset(uint32_t base_off)
    {
        base_off_ = base_off;
        buf_.clear();
    }

    uint32_t pos() const { return base_off_ + buf_.size(); }
    uint32_t rel_pos() const { return buf_.size(); }
    const XtensaCodeBuf& bytes() const { return buf_; }
    bool overflow() const { return buf_.overflow(); }

    [[gnu::always_inline]] void emit16(uint16_t v)
    {
        buf_.push16(v);
    }

    [[gnu::always_inline]] void emit24(uint32_t v)
    {
        buf_.push24(v);
    }

    void patch24_at(uint32_t abs_off, uint32_t v)
    {
        const size_t i = abs_off - base_off_;
        buf_[i + 0] = uint8_t(v & 0xff);
        buf_[i + 1] = uint8_t((v >> 8) & 0xff);
        buf_[i + 2] = uint8_t((v >> 16) & 0xff);
    }

    void align4()
    {
        if((buf_.size() & 3u) == 2u)
            nop_n();
        while((buf_.size() & 3u) != 0u)
            buf_.push_back(0);
    }

    // Core Xtensa helpers verified against xtensa-esp32-elf-as / objdump.
    [[gnu::always_inline]] static uint32_t encode_rrr(uint8_t op2, uint8_t op1, uint8_t r, uint8_t s, uint8_t t)
    {
        return (uint32_t(op2 & 0xf) << 20) | (uint32_t(op1 & 0xf) << 16) |
               (uint32_t(r & 0xf) << 12) | (uint32_t(s & 0xf) << 8) |
               (uint32_t(t & 0xf) << 4) | 0u;
    }

    [[gnu::always_inline]] static uint32_t encode_rri8(uint8_t r, uint8_t s, uint8_t t, uint8_t imm8, uint8_t op0 = 2)
    {
        return (uint32_t(imm8) << 16) | (uint32_t(r & 0xf) << 12) |
               (uint32_t(s & 0xf) << 8) | (uint32_t(t & 0xf) << 4) | (op0 & 0xf);
    }

    static int b4const_enc(int32_t val)
    {
        static constexpr int32_t kTable[16] = {
            -1, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 16, 32, 64, 128, 256
        };
        for(int i = 0; i < 16; ++i)
            if(kTable[i] == val) return i;
        return -1;
    }

    static int b4constu_enc(uint32_t val)
    {
        static constexpr uint32_t kTable[16] = {
            32768, 65536, 2, 3, 4, 5, 6, 7, 8, 10, 12, 16, 32, 64, 128, 256
        };
        for(int i = 0; i < 16; ++i)
            if(kTable[i] == val) return i;
        return -1;
    }

    // 16-bit Narrow Instructions
    void retw_n() { emit16(0xf01d); }
    void nop_n() { emit16(0xf03d); }
    void mov_n(uint8_t d, uint8_t s) { emit16(0x000d | ((s & 0xf) << 8) | ((d & 0xf) << 4)); }
    void add_n(uint8_t d, uint8_t s, uint8_t t)
    {
        emit16(0x000a | ((d & 0xf) << 12) | ((s & 0xf) << 8) | ((t & 0xf) << 4));
    }
    void movi_n(uint8_t t, int8_t simm7)
    {
        // simm7 range [-32, 95]
        uint16_t u = uint8_t(simm7) & 0x7f;
        emit16(0x000c | ((u & 0xf) << 12) | ((t & 0xf) << 8) | (((u >> 4) & 0x7) << 4));
    }
    void addi_n(uint8_t d, uint8_t s, int8_t imm)
    {
        // imm in {-1, 1..15}; -1 encoded as 0
        if(imm != -1 && (imm < 1 || imm > 15))
        {
            emit24(encode_rri8(12, s, d, uint8_t(imm)));
            return;
        }
        uint8_t enc = (imm == -1) ? 0 : uint8_t(imm);
        emit16(0x000b | ((d & 0xf) << 12) | ((s & 0xf) << 8) | ((enc & 0xf) << 4));
    }
    void l32i_n(uint8_t t, uint8_t s, uint8_t byte_off)
    {
        emit16(0x0008 | (((byte_off >> 2) & 0xf) << 12) | ((s & 0xf) << 8) | ((t & 0xf) << 4));
    }
    void s32i_n(uint8_t t, uint8_t s, uint8_t byte_off)
    {
        emit16(0x0009 | (((byte_off >> 2) & 0xf) << 12) | ((s & 0xf) << 8) | ((t & 0xf) << 4));
    }

    // Window / System
    void entry32() { emit24(0x004136); } // entry a1, 32
    void jx(uint8_t s) { emit24(0x0000a0u | (uint32_t(s & 0xf) << 8)); }
    void callx8(uint8_t s) { emit24(0x0000e0u | (uint32_t(s & 0xf) << 8)); }
    void memw() { emit24(0x0020c0); }
    void isync() { emit24(0x002000); }

    // 16/24-bit ALU & Moves
    void add(uint8_t d, uint8_t s, uint8_t t) { add_n(d, s, t); }
    void sub(uint8_t d, uint8_t s, uint8_t t) { emit24(encode_rrr(12, 0, d, s, t)); }
    void addx2(uint8_t d, uint8_t s, uint8_t t) { emit24(encode_rrr(9, 0, d, s, t)); }
    void addx4(uint8_t d, uint8_t s, uint8_t t) { emit24(encode_rrr(10, 0, d, s, t)); }
    void addx8(uint8_t d, uint8_t s, uint8_t t) { emit24(encode_rrr(11, 0, d, s, t)); }
    void subx2(uint8_t d, uint8_t s, uint8_t t) { emit24(encode_rrr(13, 0, d, s, t)); }
    void and_(uint8_t d, uint8_t s, uint8_t t) { emit24(encode_rrr(1, 0, d, s, t)); }
    void or_(uint8_t d, uint8_t s, uint8_t t) { emit24(encode_rrr(2, 0, d, s, t)); }
    void xor_(uint8_t d, uint8_t s, uint8_t t) { emit24(encode_rrr(3, 0, d, s, t)); }
    void mull(uint8_t d, uint8_t s, uint8_t t) { emit24(encode_rrr(8, 2, d, s, t)); }
    void moveqz(uint8_t d, uint8_t s, uint8_t t) { emit24(encode_rrr(8, 3, d, s, t)); }
    void movnez(uint8_t d, uint8_t s, uint8_t t) { emit24(encode_rrr(9, 3, d, s, t)); }
    void movltz(uint8_t d, uint8_t s, uint8_t t) { emit24(encode_rrr(10, 3, d, s, t)); }
    void movgez(uint8_t d, uint8_t s, uint8_t t) { emit24(encode_rrr(11, 3, d, s, t)); }

    void slli(uint8_t d, uint8_t s, uint8_t sa)
    {
        if(sa == 0) { if(d != s) mov_n(d, s); return; }
        uint8_t sa5 = uint8_t(32 - sa);
        emit24(encode_rrr(1, (sa5 >> 4) & 1, d, s, sa5 & 0xf));
    }
    void srli(uint8_t d, uint8_t t, uint8_t sa)
    {
        if(sa == 0) { if(d != t) mov_n(d, t); return; }
        if(sa >= 16) { extui(d, t, sa, uint8_t(32 - sa)); return; }
        emit24(encode_rrr(4, 1, d, sa & 0xf, t));
    }
    void srai(uint8_t d, uint8_t t, uint8_t sa)
    {
        emit24(encode_rrr(2 | ((sa >> 4) & 1), 1, d, sa & 0xf, t));
    }
    void extui(uint8_t d, uint8_t t, uint8_t shift, uint8_t maskw)
    {
        emit24(encode_rrr(uint8_t((maskw - 1) & 0xf), uint8_t(4 | ((shift >> 4) & 1)),
                          d, uint8_t(shift & 0xf), t));
    }
    void sext(uint8_t d, uint8_t s, uint8_t imm)
    {
        emit24(encode_rrr(2, 3, d, s, uint8_t(imm - 7)));
    }
    void ssa8l(uint8_t s) { emit24(encode_rrr(4, 0, 2, s, 0)); }
    void sll(uint8_t d, uint8_t s) { emit24(encode_rrr(10, 1, d, s, 0)); }
    void srl(uint8_t d, uint8_t t) { emit24(encode_rrr(9, 1, d, 0, t)); }
    void sra(uint8_t d, uint8_t t) { emit24(encode_rrr(11, 1, d, 0, t)); }

    // Memory & Immediates
    void l8ui(uint8_t t, uint8_t s, uint16_t off) { emit24(encode_rri8(0, s, t, uint8_t(off))); }
    void l16ui(uint8_t t, uint8_t s, uint16_t off) { emit24(encode_rri8(1, s, t, uint8_t(off >> 1))); }
    void l16si(uint8_t t, uint8_t s, uint16_t off) { emit24(encode_rri8(9, s, t, uint8_t(off >> 1))); }
    void l32i(uint8_t t, uint8_t s, uint16_t off)
    {
        if(off <= 60 && (off & 3) == 0) { l32i_n(t, s, uint8_t(off)); return; }
        emit24(encode_rri8(2, s, t, uint8_t(off >> 2)));
    }
    void s8i(uint8_t t, uint8_t s, uint16_t off) { emit24(encode_rri8(4, s, t, uint8_t(off))); }
    void s16i(uint8_t t, uint8_t s, uint16_t off) { emit24(encode_rri8(5, s, t, uint8_t(off >> 1))); }
    void s32i(uint8_t t, uint8_t s, uint16_t off)
    {
        if(off <= 60 && (off & 3) == 0) { s32i_n(t, s, uint8_t(off)); return; }
        emit24(encode_rri8(6, s, t, uint8_t(off >> 2)));
    }

    void addi(uint8_t t, uint8_t s, int8_t simm8)
    {
        if(simm8 == -1 || (simm8 >= 1 && simm8 <= 15)) { addi_n(t, s, simm8); return; }
        emit24(encode_rri8(12, s, t, uint8_t(simm8)));
    }
    void addi24(uint8_t t, uint8_t s, int8_t simm8)
    {
        emit24(encode_rri8(12, s, t, uint8_t(simm8)));
    }
    void addmi(uint8_t t, uint8_t s, int32_t simm8_x256)
    {
        emit24(encode_rri8(13, s, t, uint8_t(simm8_x256 >> 8)));
    }
    void movi(uint8_t t, int32_t simm12)
    {
        emit24(encode_rri8(10, uint8_t((simm12 >> 8) & 0xf), t, uint8_t(simm12 & 0xff)));
    }

    void add_const(uint8_t d, uint8_t s, int32_t delta)
    {
        if(delta == 0) { if(d != s) mov_n(d, s); return; }
        if(delta >= -128 && delta <= 127) { addi(d, s, int8_t(delta)); return; }
        int32_t hi = (delta + 128) & ~255;
        int32_t lo = delta - hi;
        addmi(d, s, hi);
        if(lo != 0) addi(d, d, int8_t(lo));
    }

    void load_imm(uint8_t d, int32_t val)
    {
        if(val >= -32 && val <= 95) { movi_n(d, int8_t(val)); return; }
        if(val >= -2048 && val <= 2047) { movi(d, val); return; }
        int32_t lo = int8_t(val & 0xff);
        int32_t hi = (val - lo);
        movi(d, lo);
        addmi(d, d, hi);
    }

    // Load 0..16383 always in fixed 3 or 6 bytes cleanly (useful for constant PCs).
    void load_u14(uint8_t d, uint16_t val)
    {
        if(val <= 95)
            movi_n(d, int8_t(val));
        else if(val <= 2047)
            movi(d, val);
        else
        {
            int32_t lo = int8_t(val & 0xff);
            int32_t hi = int32_t(val) - lo;
            movi(d, lo);
            addmi(d, d, hi);
        }
    }

    // Jumps & Branches
    static uint32_t encode_j(uint32_t from_off, uint32_t target_off)
    {
        int32_t disp = int32_t(target_off) - int32_t(from_off) - 4;
        return ((uint32_t(disp) & 0x3ffffu) << 6) | 0x06u;
    }

    void j(uint32_t target_off)
    {
        emit24(encode_j(pos(), target_off));
    }

    uint32_t emit_j_placeholder()
    {
        uint32_t p = pos();
        emit24(0x06u);
        return p;
    }

    void patch_j(uint32_t at_off, uint32_t target_off)
    {
        patch24_at(at_off, encode_j(at_off, target_off));
    }

    // BRI12: beqz(0), bnez(1), bltz(2), bgez(3) -> range [-2048, 2047] from pc + 4
    static uint32_t encode_bri12(uint8_t m, uint8_t s, uint32_t from_off, uint32_t target_off)
    {
        int32_t disp = int32_t(target_off) - int32_t(from_off) - 4;
        return ((uint32_t(disp) & 0xfffu) << 12) | (uint32_t(s & 0xf) << 8) |
               (uint32_t(m & 3) << 6) | 0x16u;
    }
    uint32_t emit_bri12_placeholder(uint8_t m, uint8_t s)
    {
        uint32_t p = pos();
        emit24(encode_bri12(m, s, p, p + 4));
        return p;
    }
    void patch_bri12(uint32_t at_off, uint8_t m, uint8_t s, uint32_t target_off)
    {
        patch24_at(at_off, encode_bri12(m, s, at_off, target_off));
    }
    void beqz(uint8_t s, uint32_t target) { emit24(encode_bri12(0, s, pos(), target)); }
    void bnez(uint8_t s, uint32_t target) { emit24(encode_bri12(1, s, pos(), target)); }
    void bltz(uint8_t s, uint32_t target) { emit24(encode_bri12(2, s, pos(), target)); }
    void bgez(uint8_t s, uint32_t target) { emit24(encode_bri12(3, s, pos(), target)); }

    // RRI8 branches (range [-128, 127] from pc + 4)
    static uint32_t encode_br8(uint8_t r, uint8_t s, uint8_t t, uint32_t from_off, uint32_t target_off, uint8_t op0 = 7)
    {
        int32_t disp = int32_t(target_off) - int32_t(from_off) - 4;
        return (uint32_t(uint8_t(disp)) << 16) | (uint32_t(r & 0xf) << 12) |
               (uint32_t(s & 0xf) << 8) | (uint32_t(t & 0xf) << 4) | (op0 & 0xf);
    }

    uint32_t emit_br8_placeholder(uint8_t r, uint8_t s, uint8_t t, uint8_t op0 = 7)
    {
        uint32_t p = pos();
        emit24(encode_br8(r, s, t, p, p + 4, op0));
        return p;
    }
    void patch_br8(uint32_t at_off, uint8_t r, uint8_t s, uint8_t t, uint32_t target_off, uint8_t op0 = 7)
    {
        patch24_at(at_off, encode_br8(r, s, t, at_off, target_off, op0));
    }

    // Immediate/Register branches
    uint32_t beq_ph(uint8_t s, uint8_t t) { return emit_br8_placeholder(1, s, t, 7); }
    uint32_t bne_ph(uint8_t s, uint8_t t) { return emit_br8_placeholder(9, s, t, 7); }
    uint32_t blt_ph(uint8_t s, uint8_t t) { return emit_br8_placeholder(2, s, t, 7); }
    uint32_t bge_ph(uint8_t s, uint8_t t) { return emit_br8_placeholder(10, s, t, 7); }
    uint32_t bltu_ph(uint8_t s, uint8_t t) { return emit_br8_placeholder(3, s, t, 7); }
    uint32_t bgeu_ph(uint8_t s, uint8_t t) { return emit_br8_placeholder(11, s, t, 7); }
    uint32_t bbci_ph(uint8_t s, uint8_t bit) { return emit_br8_placeholder(uint8_t(6 | ((bit >> 4) & 1)), s, bit & 0xf, 7); }
    uint32_t bbsi_ph(uint8_t s, uint8_t bit) { return emit_br8_placeholder(uint8_t(14 | ((bit >> 4) & 1)), s, bit & 0xf, 7); }
    uint32_t beqi_ph(uint8_t s, int enc) { return emit_br8_placeholder(uint8_t(enc), s, 2, 6); }
    uint32_t bnei_ph(uint8_t s, int enc) { return emit_br8_placeholder(uint8_t(enc), s, 6, 6); }
    uint32_t blti_ph(uint8_t s, int enc) { return emit_br8_placeholder(uint8_t(enc), s, 10, 6); }
    uint32_t bgei_ph(uint8_t s, int enc) { return emit_br8_placeholder(uint8_t(enc), s, 14, 6); }
    uint32_t bltui_ph(uint8_t s, int enc) { return emit_br8_placeholder(uint8_t(enc), s, 11, 6); }
    uint32_t bgeui_ph(uint8_t s, int enc) { return emit_br8_placeholder(uint8_t(enc), s, 15, 6); }

    void bbci(uint8_t s, uint8_t bit, uint32_t target)
    {
        emit24(encode_br8(uint8_t(6 | ((bit >> 4) & 1)), s, bit & 0xf, pos(), target, 7));
    }
    void bbsi(uint8_t s, uint8_t bit, uint32_t target)
    {
        emit24(encode_br8(uint8_t(14 | ((bit >> 4) & 1)), s, bit & 0xf, pos(), target, 7));
    }
    void bgeui(uint8_t s, uint32_t imm, uint32_t target)
    {
        emit24(encode_br8(uint8_t(b4constu_enc(imm)), s, 15, pos(), target, 6));
    }

private:
    uint32_t base_off_ = 0;
    XtensaCodeBuf buf_;
};

JitOffsets compute_jit_offsets(const atmega32u4_t& cpu);
void jit_reset_runtime(JitRuntime& rt, atmega32u4_t& cpu, const JitOffsets& off);
void jit_invalidate_runtime(JitRuntime& rt, const atmega32u4_t& cpu);
bool jit_compile_block(JitRuntime& rt, const atmega32u4_t& cpu, const JitOffsets& off, uint16_t start_pc);
const JitRuntime& jit_runtime();
void jit_reset_stats();
void jit_preallocate();
void jit_preallocate_runtime(JitRuntime& rt);

} // namespace jit
} // namespace absim
