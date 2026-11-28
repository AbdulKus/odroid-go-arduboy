#include "absim.hpp"

#include "absim_instructions.hpp"
#include "absim_jit_xtensa.hpp"

// TODO: clean up remaining flags logic

namespace absim
{

ARDENS_IRAM ARDENS_FORCEINLINE static uint32_t instr_merged_micros_result(atmega32u4_t& cpu, avr_instr_t i);
ARDENS_IRAM static uint32_t instr_merged_micros_timer(atmega32u4_t& cpu, avr_instr_t i);
static uint32_t instr_merged_skip_rjmp(atmega32u4_t& cpu, avr_instr_t i);
static uint32_t instr_merged_add_adc2(atmega32u4_t& cpu, avr_instr_t i);
static uint32_t instr_delay_call_fallback(atmega32u4_t& cpu, avr_instr_t i)
{
    return i.m0 == 1 ? instr_rcall(cpu, i) : instr_call(cpu, i);
}
static uint32_t instr_display_loop_fallback(atmega32u4_t& cpu, avr_instr_t i)
{
    return INSTR_MAP[i.m2](cpu, i);
}

instr_func_t const INSTR_MAP[NUM_INSTR] =
{
    instr_unknown,
    instr_rcall,
    instr_call,
    instr_icall,
    instr_ret,
    instr_reti,
    instr_movw,
    instr_mov,
    instr_and,
    instr_or,
    instr_eor,
    instr_clr,
    instr_add,
    instr_adc,
    instr_sub,
    instr_sbc,
    instr_cpi,
    instr_cp,
    instr_cpc,
    instr_out,
    instr_in,
    instr_ldi,
    instr_lpm,
    instr_brbs,
    instr_brbc,
    instr_lds,
    instr_sts,
    instr_ldd_y,
    instr_ldd_z,
    instr_std_y,
    instr_std_z,
    instr_ld_st,
    instr_ld_x,
    instr_ld_y,
    instr_ld_z,
    instr_ld_x_inc,
    instr_ld_y_inc,
    instr_ld_z_inc,
    instr_ld_x_dec,
    instr_ld_y_dec,
    instr_ld_z_dec,
    instr_st_x,
    instr_st_y,
    instr_st_z,
    instr_st_x_inc,
    instr_st_y_inc,
    instr_st_z_inc,
    instr_st_x_dec,
    instr_st_y_dec,
    instr_st_z_dec,
    instr_push,
    instr_pop,
    instr_cpse,
    instr_subi,
    instr_sbci,
    instr_ori,
    instr_andi,
    instr_adiw,
    instr_sbiw,
    instr_bset,
    instr_bclr,
    instr_sbi,
    instr_cbi,
    instr_sbis,
    instr_sbic,
    instr_sbrs,
    instr_sbrc,
    instr_bld,
    instr_bst,
    instr_com,
    instr_neg,
    instr_swap,
    instr_inc,
    instr_dec,
    instr_asr,
    instr_lsr,
    instr_ror,
    instr_sleep,
    instr_mul,
    instr_muls,
    instr_mulsu,
    instr_fmul,
    instr_fmuls,
    instr_fmulsu,
    instr_nop,
    instr_rjmp,
    instr_jmp,
    instr_ijmp,
    instr_wdr,
    instr_spm,
    instr_break,

    // merged instrs

    instr_merged_out,
    instr_merged_in,
    instr_merged_lds,
    instr_merged_sts,
    instr_merged_ldd_y,
    instr_merged_ldd_z,
    instr_merged_std_y,
    instr_merged_std_z,
    instr_merged_ld_st,
    instr_merged_ld_x,
    instr_merged_ld_y,
    instr_merged_ld_z,
    instr_merged_ld_x_inc,
    instr_merged_ld_y_inc,
    instr_merged_ld_z_inc,
    instr_merged_ld_x_dec,
    instr_merged_ld_y_dec,
    instr_merged_ld_z_dec,
    instr_merged_st_x,
    instr_merged_st_y,
    instr_merged_st_z,
    instr_merged_st_x_inc,
    instr_merged_st_y_inc,
    instr_merged_st_z_inc,
    instr_merged_st_x_dec,
    instr_merged_st_y_dec,
    instr_merged_st_z_dec,
    instr_merged_sbi,
    instr_merged_cbi,
    instr_merged_sbis,
    instr_merged_sbic,

    instr_merged_ldi2,
    instr_merged_dec_brne,
    instr_merged_add_adc,
    instr_merged_sub_sbc,
    instr_merged_cp_cpc,
    instr_merged_subi_sbci,
    instr_merged_delay,
    instr_merged_micros_result,
    instr_merged_micros_timer,
    instr_call, // Boundary fallback for INSTR_MERGED_MICROS_CALL.
    instr_merged_in, // Boundary fallback for INSTR_MERGED_SPI_POLL.
    instr_merged_skip_rjmp,
    instr_merged_add_adc2,
    instr_delay_call_fallback,
    instr_display_loop_fallback,
    instr_display_loop_fallback,
    instr_display_loop_fallback,
    instr_display_loop_fallback,
    instr_display_loop_fallback,
};

ARDENS_FORCEINLINE static bool next_instr_is_two_words(atmega32u4_t const& cpu)
{
    if(cpu.pc + 1u >= cpu.decoded_prog.size())
        return false;
    return instr_is_two_words(cpu.decoded_prog[(cpu.pc + 1) % cpu.decoded_prog.size()]);
}

uint32_t instr_nop(atmega32u4_t& cpu, avr_instr_t i)
{
    (void)i;
    cpu.pc += 1;
    return 1;
}

uint32_t instr_unknown(atmega32u4_t& cpu, avr_instr_t i)
{
    (void)cpu;
    (void)i;
    return 1;
}

uint32_t instr_wdr(atmega32u4_t& cpu, avr_instr_t i)
{
    (void)i;
    cpu.watchdog_divider_cycle = 0;
    cpu.watchdog_prev_cycle = cpu.cycle_count;
    cpu.watchdog_next_cycle = cpu.cycle_count + cpu.watchdog_divider;
    cpu.peripheral_queue.schedule(cpu.watchdog_next_cycle, PQ_WATCHDOG);
    cpu.pc += 1;
    return 1;
}

uint32_t instr_spm(atmega32u4_t& cpu, avr_instr_t i)
{
    (void)i;
    cpu.execute_spm();
    cpu.pc += 1;
    return 1;
}

uint32_t instr_break(atmega32u4_t& cpu, avr_instr_t i)
{
    (void)i;
    cpu.pc += 1;
    cpu.autobreak(AB_BREAK);
    return 1;
}

ARDENS_FORCEINLINE static void set_flag(atmega32u4_t& cpu, uint8_t mask, uint32_t x)
{
    if(x) cpu.data[reg::addr::SREG] |= mask;
    else cpu.data[reg::addr::SREG] &= ~mask;
}

ARDENS_FORCEINLINE static uint8_t flag_s(uint8_t sreg)
{
    // S |= N ^ V
    sreg |= (((sreg ^ (sreg >> 1)) & 0x4) << 2);
    return sreg;
}

ARDENS_FORCEINLINE static void set_flags_hcv(atmega32u4_t& cpu, uint8_t h, uint8_t c, uint8_t v)
{
    uint8_t sreg = cpu.data[reg::addr::SREG] & ~(reg::bit::SREG::H | reg::bit::SREG::C | reg::bit::SREG::V);

    if(h) sreg |= reg::bit::SREG::H;
    if(c) sreg |= reg::bit::SREG::C;
    if(v) sreg |= reg::bit::SREG::V;

    cpu.data[reg::addr::SREG] = sreg;
}

ARDENS_FORCEINLINE static uint8_t flags_nzs(uint8_t sreg, uint32_t x)
{
    sreg |= ((x & 0x80) >> 5); // N
    if(x == 0) sreg |= reg::bit::SREG::Z; // Z
    sreg = flag_s(sreg);       // S
    return sreg;
}

ARDENS_FORCEINLINE static uint8_t flags_nzs16(uint8_t sreg, uint32_t x)
{
    sreg |= ((x & 0x8000) >> 13); // N
    if(x == 0) sreg |= reg::bit::SREG::Z; // Z
    sreg = flag_s(sreg);       // S
    return sreg;
}

ARDENS_FORCEINLINE static void set_flags_nzs(atmega32u4_t& cpu, uint32_t x)
{
    uint8_t sreg = cpu.data[reg::addr::SREG] & ~(reg::bit::SREG::N | reg::bit::SREG::Z | reg::bit::SREG::S);
    sreg = flags_nzs(sreg, x);
    cpu.data[reg::addr::SREG] = sreg;
}

uint32_t instr_rjmp(atmega32u4_t& cpu, avr_instr_t i)
{
    cpu.pc += (int16_t)i.word + 1;
    return 2;
}

uint32_t instr_jmp(atmega32u4_t& cpu, avr_instr_t i)
{
    cpu.pc = i.word;
    return 3;
}

uint32_t instr_ijmp(atmega32u4_t& cpu, avr_instr_t i)
{
    (void)i;
    uint16_t z = cpu.z_word();
    if(z >= cpu.last_addr / 2)
    {
        cpu.autobreak(AB_OOB_IJMP);
        if(cpu.enabled_autobreaks.test(AB_OOB_IJMP))
        {
            // cancel instruction so user has a chance for introspection
            return 2;
        }
    }
    cpu.pc = z;
    return 2;
}

ARDENS_FORCEINLINE static bool return_address_fits(const atmega32u4_t& cpu, unsigned high)
{
    return high >= 256 && high < cpu.data.size() - 1;
}

ARDENS_FORCEINLINE static void push_return_address(atmega32u4_t& cpu, uint16_t address)
{
#if defined(ARDUGO_EMBEDDED) && defined(ARDENS_NO_DEBUGGER)
    const unsigned sp = cpu.sp();
    if(sp && return_address_fits(cpu, sp - 1))
    {
        cpu.data[sp] = uint8_t(address);
        cpu.data[sp - 1] = uint8_t(address >> 8);
        cpu.data[reg::addr::SPL] = uint8_t(sp - 2);
        cpu.data[reg::addr::SPH] = uint8_t((sp - 2) >> 8);
        cpu.pushed_at_least_once = true;
        return;
    }
#endif
    cpu.push(uint8_t(address));
    cpu.push(uint8_t(address >> 8));
}

ARDENS_FORCEINLINE static uint16_t pop_return_address(atmega32u4_t& cpu)
{
#if defined(ARDUGO_EMBEDDED) && defined(ARDENS_NO_DEBUGGER)
    const unsigned sp = cpu.sp();
    if(return_address_fits(cpu, sp + 1))
    {
        const auto address = uint16_t(cpu.data[sp + 2] | (uint16_t(cpu.data[sp + 1]) << 8));
        cpu.data[reg::addr::SPL] = uint8_t(sp + 2);
        cpu.data[reg::addr::SPH] = uint8_t((sp + 2) >> 8);
        return address;
    }
#endif
    const uint16_t hi = cpu.pop();
    const uint16_t lo = cpu.pop();
    return lo | (hi << 8);
}

uint32_t instr_rcall(atmega32u4_t& cpu, avr_instr_t i)
{
    uint16_t ret_addr = cpu.pc + 1;
    cpu.pc += (int16_t)i.word + 1;
    if(i.word != 0)
        cpu.push_stack_frame(ret_addr);
    push_return_address(cpu, ret_addr);
    return 3;
}

ARDENS_FORCEINLINE static uint32_t exec_call_inline(atmega32u4_t& cpu, avr_instr_t i)
{
    uint16_t ret_addr = cpu.pc + 2;
    cpu.pc = i.word;
    cpu.push_stack_frame(ret_addr);
    push_return_address(cpu, ret_addr);
    return 4;
}

uint32_t instr_call(atmega32u4_t& cpu, avr_instr_t i)
{
    return exec_call_inline(cpu, i);
}

uint32_t instr_icall(atmega32u4_t& cpu, avr_instr_t i)
{
    uint16_t ret_addr = cpu.pc + 1;
    cpu.pc = cpu.z_word();
    cpu.push_stack_frame(ret_addr);
    push_return_address(cpu, ret_addr);
    return 3;
}

ARDENS_FORCEINLINE static uint32_t exec_ret_inline(atmega32u4_t& cpu)
{
    cpu.pc = pop_return_address(cpu);
    cpu.pop_stack_frame();
    return 4;
}

uint32_t instr_ret(atmega32u4_t& cpu, avr_instr_t i)
{
    (void)i;
    return exec_ret_inline(cpu);
}

uint32_t instr_reti(atmega32u4_t& cpu, avr_instr_t i)
{
    cpu.pc = pop_return_address(cpu);
    cpu.data[reg::addr::SREG] |= reg::bit::SREG::I;
    cpu.io_reg_accessed = true;
    cpu.just_written = 0x5f;
    cpu.pop_stack_frame();
    return 4;
}

uint32_t instr_movw(atmega32u4_t& cpu, avr_instr_t i)
{
#if 1
    // endianness doesn't matter here
    uint16_t* dst = reinterpret_cast<uint16_t*>(&cpu.data[0]) + i.dst;
    uint16_t* src = reinterpret_cast<uint16_t*>(&cpu.data[0]) + i.src;
    *dst = *src;
#else
    cpu.gpr(i.dst * 2 + 0) = cpu.gpr(i.src * 2 + 0);
    cpu.gpr(i.dst * 2 + 1) = cpu.gpr(i.src * 2 + 1);
#endif
    cpu.pc += 1;
    return 1;
}

uint32_t instr_mov(atmega32u4_t& cpu, avr_instr_t i)
{
    cpu.gpr(i.dst) = cpu.gpr(i.src);
    cpu.pc += 1;
    return 1;
}

uint32_t instr_and(atmega32u4_t& cpu, avr_instr_t i)
{
    uint32_t dst = cpu.gpr(i.dst);
    uint32_t res = dst & cpu.gpr(i.src);
    cpu.gpr(i.dst) = res;
    uint8_t sreg = cpu.data[reg::addr::SREG] & ~(reg::bit::SREG::V | reg::bit::SREG::N | reg::bit::SREG::Z | reg::bit::SREG::S);
    if(res == 0) sreg |= reg::bit::SREG::Z;
    if(res & 0x80) sreg |= (reg::bit::SREG::N | reg::bit::SREG::S);
    cpu.data[reg::addr::SREG] = sreg;
    cpu.pc += 1;
    return 1;
}

uint32_t instr_or(atmega32u4_t& cpu, avr_instr_t i)
{
    uint32_t dst = cpu.gpr(i.dst);
    uint32_t res = dst | cpu.gpr(i.src);
    cpu.gpr(i.dst) = res;
    uint8_t sreg = cpu.data[reg::addr::SREG] & ~(reg::bit::SREG::V | reg::bit::SREG::N | reg::bit::SREG::Z | reg::bit::SREG::S);
    sreg = flags_nzs(sreg, res);
    cpu.data[reg::addr::SREG] = sreg;
    cpu.pc += 1;
    return 1;
}

uint32_t instr_eor(atmega32u4_t& cpu, avr_instr_t i)
{
    uint32_t dst = cpu.gpr(i.dst);
    uint32_t res = dst ^ cpu.gpr(i.src);
    cpu.gpr(i.dst) = res;
    uint8_t sreg = cpu.data[reg::addr::SREG] & ~(reg::bit::SREG::V | reg::bit::SREG::N | reg::bit::SREG::Z | reg::bit::SREG::S);
    if(res == 0) sreg |= reg::bit::SREG::Z;
    if(res & 0x80) sreg |= (reg::bit::SREG::N | reg::bit::SREG::S);
    cpu.data[reg::addr::SREG] = sreg;
    cpu.pc += 1;
    return 1;
}

uint32_t instr_clr(atmega32u4_t& cpu, avr_instr_t i)
{
    cpu.gpr(i.dst) = 0;
    uint8_t sreg = cpu.data[reg::addr::SREG];
    sreg &= ~0x1c;
    sreg |= 0x02;
    cpu.data[reg::addr::SREG] = sreg;
    cpu.pc += 1;
    return 1;
}

uint32_t instr_add(atmega32u4_t& cpu, avr_instr_t i)
{
    unsigned dst = cpu.gpr(i.dst);
    unsigned src = cpu.gpr(i.src);
    unsigned res = (dst + src) & 0xff;
    cpu.gpr(i.dst) = (uint8_t)res;

    unsigned dst_xor_src = dst ^ src;
    unsigned hc = (dst | src) ^ (res & dst_xor_src);
    unsigned v = ~dst_xor_src & (dst ^ res);
    unsigned sreg = cpu.data[reg::addr::SREG] & ~reg::bit::SREG::HSVNZC;
    sreg |= (hc & 0x08) << 2;    // H flag
    sreg |= hc >> 7;             // C flag
    sreg |= (v & 0x80) >> 4;     // V flag
    sreg = flags_nzs(sreg, res);
    cpu.data[reg::addr::SREG] = (uint8_t)sreg;

    cpu.pc += 1;
    return 1;
}

ARDENS_FORCEINLINE uint32_t instr_adc(atmega32u4_t& cpu, avr_instr_t i)
{
    unsigned dst = cpu.gpr(i.dst);
    unsigned src = cpu.gpr(i.src);
    unsigned c = cpu.data[reg::addr::SREG] & 1;
    unsigned res = (dst + src + c) & 0xff;
    cpu.gpr(i.dst) = (uint8_t)res;

    unsigned dst_xor_src = dst ^ src;
    unsigned hc = (dst | src) ^ (res & dst_xor_src);
    unsigned v = ~dst_xor_src & (dst ^ res);
    unsigned sreg = cpu.data[reg::addr::SREG] & ~reg::bit::SREG::HSVNZC;
    sreg |= (hc & 0x08) << 2;    // H flag
    sreg |= hc >> 7;             // C flag
    sreg |= (v & 0x80) >> 4;     // V flag
    sreg = flags_nzs(sreg, res);
    cpu.data[reg::addr::SREG] = (uint8_t)sreg;

    cpu.pc += 1;
    return 1;
}

ARDENS_FORCEINLINE static void sub_flags(atmega32u4_t& cpu, unsigned res, unsigned dst, unsigned src)
{
    unsigned sreg = cpu.data[reg::addr::SREG] & ~reg::bit::SREG::HSVNZC;
    unsigned res_xor_src = res ^ src;
    unsigned hc = (res | src) ^ (dst & res_xor_src);
    unsigned v = ~res_xor_src & (res ^ dst);
    sreg |= (hc & 0x08) << 2;    // H flag
    sreg |= hc >> 7;             // C flag
    sreg |= (v & 0x80) >> 4;     // V flag
    sreg = flags_nzs(sreg, res);
    cpu.data[reg::addr::SREG] = (uint8_t)sreg;
}

ARDENS_FORCEINLINE static void sub_imm(atmega32u4_t& cpu, unsigned rdst, unsigned imm, unsigned c)
{
    unsigned dst = cpu.gpr(rdst);
    unsigned src = imm;
    unsigned res = (dst - src - c) & 0xff;
    cpu.gpr(rdst) = (uint8_t)res;
    sub_flags(cpu, res, dst, src);
}

uint32_t instr_sub(atmega32u4_t& cpu, avr_instr_t i)
{
    sub_imm(cpu, i.dst, cpu.gpr(i.src), 0);
    cpu.pc += 1;
    return 1;
}

uint32_t instr_sbc(atmega32u4_t& cpu, avr_instr_t i)
{
    unsigned sreg = cpu.data[reg::addr::SREG];
    unsigned dst = cpu.gpr(i.dst);
    unsigned src = cpu.gpr(i.src);
    unsigned res = (dst - src - (sreg & reg::bit::SREG::C)) & 0xff;
    cpu.gpr(i.dst) = (uint8_t)res;

    unsigned res_xor_src = res ^ src;
    unsigned hc = (res | src) ^ (dst & res_xor_src);
    unsigned v = ~res_xor_src & (res ^ dst);
    unsigned z = ~sreg & reg::bit::SREG::Z;
    sreg &= ~reg::bit::SREG::HSVNZC;
    sreg |= (hc & 0x08) << 2;    // H flag
    sreg |= hc >> 7;             // C flag
    sreg |= (v & 0x80) >> 4;     // V flag
    res |= z;
    sreg = flags_nzs(sreg, res);
    cpu.data[reg::addr::SREG] = (uint8_t)sreg;

    cpu.pc += 1;
    return 1;
}

ARDENS_FORCEINLINE uint32_t instr_cpi(atmega32u4_t& cpu, avr_instr_t i)
{
    unsigned dst = cpu.gpr(i.dst);
    unsigned src = i.src;
    unsigned res = (dst - src) & 0xff;
    sub_flags(cpu, res, dst, src);
    cpu.pc += 1;
    return 1;
}

ARDENS_FORCEINLINE uint32_t instr_cp(atmega32u4_t& cpu, avr_instr_t i)
{
    unsigned dst = cpu.gpr(i.dst);
    unsigned src = cpu.gpr(i.src);
    unsigned res = (dst - src) & 0xff;
    sub_flags(cpu, res, dst, src);
    cpu.pc += 1;
    return 1;
}

uint32_t instr_cpc(atmega32u4_t& cpu, avr_instr_t i)
{
    unsigned sreg = cpu.data[reg::addr::SREG];
    unsigned dst = cpu.gpr(i.dst);
    unsigned src = cpu.gpr(i.src);
    unsigned res = (dst - src - (sreg & reg::bit::SREG::C)) & 0xff;

    unsigned res_xor_src = res ^ src;
    unsigned hc = (res | src) ^ (dst & res_xor_src);
    unsigned v = ~res_xor_src & (res ^ dst);
    unsigned z = ~sreg & reg::bit::SREG::Z;
    sreg &= ~reg::bit::SREG::HSVNZC;
    sreg |= (hc & 0x08) << 2;    // H flag
    sreg |= hc >> 7;             // C flag
    sreg |= (v & 0x80) >> 4;     // V flag
    res |= z;
    sreg = flags_nzs(sreg, res);
    cpu.data[reg::addr::SREG] = (uint8_t)sreg;

    cpu.pc += 1;
    return 1;

}

template<bool merged>
ARDENS_FORCEINLINE static uint32_t instr_out(atmega32u4_t& cpu, avr_instr_t i)
{
    cpu.st_ior<merged>(i.dst, cpu.gpr(i.src));
    cpu.pc += 1;
    return 1;
}
ARDENS_IRAM uint32_t instr_out(atmega32u4_t& cpu, avr_instr_t i) { return instr_out<false>(cpu, i); }
uint32_t instr_merged_out(atmega32u4_t& cpu, avr_instr_t i) { return instr_out<true>(cpu, i); }

template<bool merged>
ARDENS_FORCEINLINE static uint32_t instr_in(atmega32u4_t& cpu, avr_instr_t i)
{
    cpu.gpr(i.dst) = cpu.ld_ior<merged>(i.src);
    cpu.pc += 1;
    return 1;
}
ARDENS_IRAM uint32_t instr_in(atmega32u4_t& cpu, avr_instr_t i) { return instr_in<false>(cpu, i); }
uint32_t instr_merged_in(atmega32u4_t& cpu, avr_instr_t i) { return instr_in<true>(cpu, i); }

uint32_t instr_ldi(atmega32u4_t& cpu, avr_instr_t i)
{
    cpu.gpr(i.dst) = i.src;
    cpu.pc += 1;
    return 1;
}

uint32_t instr_lpm(atmega32u4_t& cpu, avr_instr_t i)
{
    // TODO: handle RWW errors (0x0000 - 0x37ff while RWWSB is set in SPMCSR)
    uint16_t z = cpu.z_word();
    uint8_t res = 0x00;
    if(z < cpu.prog.size())
        res = cpu.prog[z];
    if(cpu.spm_en_cycles != 0)
    {
        if(cpu.spm_op == cpu.SPM_OP_BLB_SET)
        {
            // reading fuse or lock bits
            switch(z)
            {
            case 0x0000: res = cpu.fuse_lo; break;
            case 0x0001: res = cpu.lock; break;
            case 0x0002: res = cpu.fuse_ext; break;
            case 0x0003: res = cpu.fuse_hi; break;
            default:
                break;
            }
        }
        else if(cpu.spm_op == cpu.SPM_OP_SIG_READ)
        {
            switch(z)
            {
            case 0x0000: res = 0x1e; break;
            case 0x0002: res = 0x95; break;
            case 0x0004: res = 0x87; break;
            case 0x0001: res = 0x6d; break;
            default:
                break;
            }
        }
    }
    cpu.gpr(i.dst) = res;
    cpu.pc += 1;
    if(i.word == 1)
    {
        // post increment z pointer
        ++z;
        cpu.gpr(30) = uint8_t(z >> 0);
        cpu.gpr(31) = uint8_t(z >> 8);
    }
    return 3;
}

uint32_t instr_brbs(atmega32u4_t& cpu, avr_instr_t i)
{
    if(cpu.data[reg::addr::SREG] & (1 << i.src))
    {
        cpu.pc += (int16_t)i.word + 1;
        return 2;
    }
    cpu.pc += 1;
    return 1;
}

uint32_t instr_brbc(atmega32u4_t& cpu, avr_instr_t i)
{
    if(!(cpu.data[reg::addr::SREG] & (1 << i.src)))
    {
        cpu.pc += (int16_t)i.word + 1;
        return 2;
    }
    cpu.pc += 1;
    return 1;
}

template<bool merged>
ARDENS_FORCEINLINE static uint32_t instr_lds(atmega32u4_t& cpu, avr_instr_t i)
{
    cpu.gpr(i.dst) = cpu.ld<merged>(i.word);
    cpu.pc += 2;
    return 2;
}
uint32_t instr_lds(atmega32u4_t& cpu, avr_instr_t i) { return instr_lds<false>(cpu, i); }
uint32_t instr_merged_lds(atmega32u4_t& cpu, avr_instr_t i) { return instr_lds<true>(cpu, i); }

template<bool merged>
ARDENS_FORCEINLINE static uint32_t instr_sts(atmega32u4_t& cpu, avr_instr_t i)
{
    cpu.st<merged>(i.word, cpu.gpr(i.src));
    cpu.pc += 2;
    return 2;
}
uint32_t instr_sts(atmega32u4_t& cpu, avr_instr_t i) { return instr_sts<false>(cpu, i); }
uint32_t instr_merged_sts(atmega32u4_t& cpu, avr_instr_t i) { return instr_sts<true>(cpu, i); }

template<bool merged>
ARDENS_FORCEINLINE static uint32_t instr_ldd_y(atmega32u4_t& cpu, avr_instr_t i)
{
    uint16_t ptr = cpu.y_word();
    if(ptr == 0)
        cpu.autobreak(AB_NULL_REL_DEREF);
    ptr += i.dst;
    cpu.gpr(i.src) = cpu.ld<merged>(ptr);
    cpu.pc += 1;
    return 2;
}
uint32_t instr_ldd_y(atmega32u4_t& cpu, avr_instr_t i) { return instr_ldd_y<false>(cpu, i); }
uint32_t instr_merged_ldd_y(atmega32u4_t& cpu, avr_instr_t i) { return instr_ldd_y<true>(cpu, i); }

template<bool merged>
ARDENS_FORCEINLINE static uint32_t instr_ldd_z(atmega32u4_t& cpu, avr_instr_t i)
{
    uint16_t ptr = cpu.z_word();
    if(ptr == 0)
        cpu.autobreak(AB_NULL_REL_DEREF);
    ptr += i.dst;
    cpu.gpr(i.src) = cpu.ld<merged>(ptr);
    cpu.pc += 1;
    return 2;
}
uint32_t instr_ldd_z(atmega32u4_t& cpu, avr_instr_t i) { return instr_ldd_z<false>(cpu, i); }
uint32_t instr_merged_ldd_z(atmega32u4_t& cpu, avr_instr_t i) { return instr_ldd_z<true>(cpu, i); }

template<bool merged>
ARDENS_FORCEINLINE static uint32_t instr_std_y(atmega32u4_t& cpu, avr_instr_t i)
{
    uint16_t ptr = cpu.y_word();
    if(ptr == 0)
        cpu.autobreak(AB_NULL_REL_DEREF);
    ptr += i.dst;
    cpu.st<merged>(ptr, cpu.gpr(i.src));
    cpu.pc += 1;
    return 2;
}
uint32_t instr_std_y(atmega32u4_t& cpu, avr_instr_t i) { return instr_std_y<false>(cpu, i); }
uint32_t instr_merged_std_y(atmega32u4_t& cpu, avr_instr_t i) { return instr_std_y<true>(cpu, i); }

template<bool merged>
ARDENS_FORCEINLINE static uint32_t instr_std_z(atmega32u4_t& cpu, avr_instr_t i)
{
    uint16_t ptr = cpu.z_word();
    if(ptr == 0)
        cpu.autobreak(AB_NULL_REL_DEREF);
    ptr += i.dst;
    cpu.st<merged>(ptr, cpu.gpr(i.src));
    cpu.pc += 1;
    return 2;
}
uint32_t instr_std_z(atmega32u4_t& cpu, avr_instr_t i) { return instr_std_z<false>(cpu, i); }
uint32_t instr_merged_std_z(atmega32u4_t& cpu, avr_instr_t i) { return instr_std_z<true>(cpu, i); }

template<bool merged>
ARDENS_FORCEINLINE static uint32_t instr_ld_st(atmega32u4_t& cpu, avr_instr_t i)
{
    uint16_t ptr;

    if(i.dst <= 2) ptr = cpu.z_word();
    else if(i.dst <= 10) ptr = cpu.y_word();
    else ptr = cpu.x_word();

    // pre-decrement
    if(i.dst & 0x2)
        --ptr;

    if(!i.word)
        cpu.gpr(i.src) = cpu.ld<merged>(ptr);
    else
        cpu.st<merged>(ptr, cpu.gpr(i.src));

    // post-increment
    if(i.dst & 0x1)
        ++ptr;

    if(i.dst & 0x3)
    {
        // store back
        if(i.dst <= 2)
        {
            cpu.gpr(30) = uint8_t(ptr >> 0);
            cpu.gpr(31) = uint8_t(ptr >> 8);
        }
        else if(i.dst <= 10)
        {
            cpu.gpr(28) = uint8_t(ptr >> 0);
            cpu.gpr(29) = uint8_t(ptr >> 8);
        }
        else
        {
            cpu.gpr(26) = uint8_t(ptr >> 0);
            cpu.gpr(27) = uint8_t(ptr >> 8);
        }
    }

    cpu.pc += 1;
    return 2;
}
uint32_t instr_ld_st(atmega32u4_t& cpu, avr_instr_t i) { return instr_ld_st<false>(cpu, i); }
uint32_t instr_merged_ld_st(atmega32u4_t& cpu, avr_instr_t i) { return instr_ld_st<true>(cpu, i); }

template<bool merged, bool ld, int reg, bool inc, bool dec>
ARDENS_FORCEINLINE static uint32_t instr_super_ld_st(atmega32u4_t& cpu, avr_instr_t i)
{
    uint16_t ptr;

    if(reg == 2) ptr = cpu.z_word();
    else if(reg == 1) ptr = cpu.y_word();
    else ptr = cpu.x_word();

    // pre-decrement
    if(dec)
        --ptr;

    if(ld)
        cpu.gpr(i.src) = cpu.ld<merged>(ptr);
    else
        cpu.st<merged>(ptr, cpu.gpr(i.src));

    // post-increment
    if(inc)
        ++ptr;

    if(inc || dec)
    {
        // store back
        if(reg == 2)
        {
            cpu.gpr(30) = uint8_t(ptr >> 0);
            cpu.gpr(31) = uint8_t(ptr >> 8);
        }
        else if(reg == 1)
        {
            cpu.gpr(28) = uint8_t(ptr >> 0);
            cpu.gpr(29) = uint8_t(ptr >> 8);
        }
        else
        {
            cpu.gpr(26) = uint8_t(ptr >> 0);
            cpu.gpr(27) = uint8_t(ptr >> 8);
        }
    }

    cpu.pc += 1;
    return 2;
}

uint32_t instr_ld_x    (atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<0, 1, 0, 0, 0>(cpu, i); }
uint32_t instr_ld_y    (atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<0, 1, 1, 0, 0>(cpu, i); }
uint32_t instr_ld_z    (atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<0, 1, 2, 0, 0>(cpu, i); }
uint32_t instr_ld_x_inc(atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<0, 1, 0, 1, 0>(cpu, i); }
uint32_t instr_ld_y_inc(atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<0, 1, 1, 1, 0>(cpu, i); }
uint32_t instr_ld_z_inc(atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<0, 1, 2, 1, 0>(cpu, i); }
uint32_t instr_ld_x_dec(atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<0, 1, 0, 0, 1>(cpu, i); }
uint32_t instr_ld_y_dec(atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<0, 1, 1, 0, 1>(cpu, i); }
uint32_t instr_ld_z_dec(atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<0, 1, 2, 0, 1>(cpu, i); }
uint32_t instr_st_x    (atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<0, 0, 0, 0, 0>(cpu, i); }
uint32_t instr_st_y    (atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<0, 0, 1, 0, 0>(cpu, i); }
uint32_t instr_st_z    (atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<0, 0, 2, 0, 0>(cpu, i); }
uint32_t instr_st_x_inc(atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<0, 0, 0, 1, 0>(cpu, i); }
uint32_t instr_st_y_inc(atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<0, 0, 1, 1, 0>(cpu, i); }
uint32_t instr_st_z_inc(atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<0, 0, 2, 1, 0>(cpu, i); }
uint32_t instr_st_x_dec(atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<0, 0, 0, 0, 1>(cpu, i); }
uint32_t instr_st_y_dec(atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<0, 0, 1, 0, 1>(cpu, i); }
uint32_t instr_st_z_dec(atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<0, 0, 2, 0, 1>(cpu, i); }

uint32_t instr_merged_ld_x    (atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<1, 1, 0, 0, 0>(cpu, i); }
uint32_t instr_merged_ld_y    (atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<1, 1, 1, 0, 0>(cpu, i); }
uint32_t instr_merged_ld_z    (atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<1, 1, 2, 0, 0>(cpu, i); }
uint32_t instr_merged_ld_x_inc(atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<1, 1, 0, 1, 0>(cpu, i); }
uint32_t instr_merged_ld_y_inc(atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<1, 1, 1, 1, 0>(cpu, i); }
uint32_t instr_merged_ld_z_inc(atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<1, 1, 2, 1, 0>(cpu, i); }
uint32_t instr_merged_ld_x_dec(atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<1, 1, 0, 0, 1>(cpu, i); }
uint32_t instr_merged_ld_y_dec(atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<1, 1, 1, 0, 1>(cpu, i); }
uint32_t instr_merged_ld_z_dec(atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<1, 1, 2, 0, 1>(cpu, i); }
uint32_t instr_merged_st_x    (atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<1, 0, 0, 0, 0>(cpu, i); }
uint32_t instr_merged_st_y    (atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<1, 0, 1, 0, 0>(cpu, i); }
uint32_t instr_merged_st_z    (atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<1, 0, 2, 0, 0>(cpu, i); }
uint32_t instr_merged_st_x_inc(atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<1, 0, 0, 1, 0>(cpu, i); }
uint32_t instr_merged_st_y_inc(atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<1, 0, 1, 1, 0>(cpu, i); }
uint32_t instr_merged_st_z_inc(atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<1, 0, 2, 1, 0>(cpu, i); }
uint32_t instr_merged_st_x_dec(atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<1, 0, 0, 0, 1>(cpu, i); }
uint32_t instr_merged_st_y_dec(atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<1, 0, 1, 0, 1>(cpu, i); }
uint32_t instr_merged_st_z_dec(atmega32u4_t& cpu, avr_instr_t i) { return instr_super_ld_st<1, 0, 2, 0, 1>(cpu, i); }

uint32_t instr_push(atmega32u4_t& cpu, avr_instr_t i)
{
    cpu.push(cpu.gpr(i.src));
    cpu.pc += 1;
    return 2;
}

uint32_t instr_pop(atmega32u4_t& cpu, avr_instr_t i)
{
    cpu.gpr(i.src) = cpu.pop();
    cpu.pc += 1;
    return 2;
}

uint32_t instr_cpse(atmega32u4_t& cpu, avr_instr_t i)
{
    uint16_t dst = cpu.gpr(i.dst);
    uint16_t src = cpu.gpr(i.src);
    if(src != dst)
    {
        cpu.pc += 1;
        return 1;
    }
    const uint32_t cycles = i.m2 ? i.m2 : 2 + next_instr_is_two_words(cpu);
    cpu.pc += cycles;
    return cycles;
}

uint32_t instr_subi(atmega32u4_t& cpu, avr_instr_t i)
{
    sub_imm(cpu, i.dst, i.src, 0);
    cpu.pc += 1;
    return 1;
}

ARDENS_FORCEINLINE uint32_t instr_sbci(atmega32u4_t& cpu, avr_instr_t i)
{
    unsigned sreg = cpu.data[reg::addr::SREG];
    unsigned dst = cpu.gpr(i.dst);
    unsigned src = i.src;
    unsigned res = (dst - src - (sreg & reg::bit::SREG::C)) & 0xff;
    cpu.gpr(i.dst) = (uint8_t)res;

    unsigned res_xor_src = res ^ src;
    unsigned hc = (res | src) ^ (dst & res_xor_src);
    unsigned v = ~res_xor_src & (res ^ dst);
    unsigned z = ~sreg & reg::bit::SREG::Z;
    sreg &= ~reg::bit::SREG::HSVNZC;
    sreg |= (hc & 0x08) << 2;    // H flag
    sreg |= hc >> 7;             // C flag
    sreg |= (v & 0x80) >> 4;     // V flag
    res |= z;
    sreg = flags_nzs(sreg, res);
    cpu.data[reg::addr::SREG] = (uint8_t)sreg;

    cpu.pc += 1;
    return 1;
}

uint32_t instr_ori(atmega32u4_t& cpu, avr_instr_t i)
{
    uint32_t dst = cpu.gpr(i.dst);
    uint32_t res = dst | i.src;
    cpu.gpr(i.dst) = res;
    uint8_t sreg = cpu.data[reg::addr::SREG] & ~(reg::bit::SREG::V | reg::bit::SREG::N | reg::bit::SREG::Z | reg::bit::SREG::S);
    if(res == 0) sreg |= reg::bit::SREG::Z;
    if(res & 0x80) sreg |= (reg::bit::SREG::N | reg::bit::SREG::S);
    cpu.data[reg::addr::SREG] = sreg;
    cpu.pc += 1;
    return 1;
}

uint32_t instr_andi(atmega32u4_t& cpu, avr_instr_t i)
{
    uint32_t dst = cpu.gpr(i.dst);
    uint32_t res = dst & i.src;
    cpu.gpr(i.dst) = res;
    uint8_t sreg = cpu.data[reg::addr::SREG] & ~(reg::bit::SREG::V | reg::bit::SREG::N | reg::bit::SREG::Z | reg::bit::SREG::S);
    if(res == 0) sreg |= reg::bit::SREG::Z;
    if(res & 0x80) sreg |= (reg::bit::SREG::N | reg::bit::SREG::S);
    cpu.data[reg::addr::SREG] = sreg;
    cpu.pc += 1;
    return 1;
}

uint32_t instr_adiw(atmega32u4_t& cpu, avr_instr_t i)
{
    uint16_t src = i.src;
    uint16_t dst = cpu.gpr_word(i.dst);
    uint16_t res = dst + src;
    cpu.gpr(i.dst + 0) = uint8_t(res >> 0);
    cpu.gpr(i.dst + 1) = uint8_t(res >> 8);

    uint8_t sreg = cpu.data[reg::addr::SREG] & ~(reg::bit::SREG::Z | reg::bit::SREG::V | reg::bit::SREG::C | reg::bit::SREG::N | reg::bit::SREG::S);

    if(res == 0) sreg |= reg::bit::SREG::Z;         // Z
    sreg |= (~dst & res & 0x8000) >> 12; // V
    sreg |= (~res & dst & 0x8000) >> 15; // C
    sreg |= (res & 0x8000) >> 13;        // N
    sreg = flag_s(sreg);                 // S

    cpu.data[reg::addr::SREG] = sreg;

    //set_flag(cpu, reg::bit::SREG::Z, res == 0);
    //set_flag(cpu, reg::bit::SREG::V, ~dst & res & 0x8000);
    //set_flag(cpu, reg::bit::SREG::C, ~res & dst & 0x8000);
    //set_flag(cpu, reg::bit::SREG::N, res & 0x8000);
    //set_flag_s(cpu);

    cpu.pc += 1;
    return 2;
}

uint32_t instr_sbiw(atmega32u4_t& cpu, avr_instr_t i)
{
    uint16_t src = i.src;
    uint16_t dst = cpu.gpr_word(i.dst);
    uint16_t res = dst - src;
    cpu.gpr(i.dst + 0) = uint8_t(res >> 0);
    cpu.gpr(i.dst + 1) = uint8_t(res >> 8);

    // in the AVR instruction set manual (seemingly a typo):
    //set_flag(cpu, reg::bit::SREG::V, res & ~dst & 0x8000);

    uint8_t sreg = cpu.data[reg::addr::SREG] & ~(reg::bit::SREG::Z | reg::bit::SREG::V | reg::bit::SREG::C | reg::bit::SREG::N | reg::bit::SREG::S);

    if(res == 0) sreg |= reg::bit::SREG::Z;         // Z
    sreg |= (dst & ~res & 0x8000) >> 12; // V
    sreg |= (res & ~dst & 0x8000) >> 15; // C
    sreg |= (res & 0x8000) >> 13;        // N
    sreg = flag_s(sreg);                 // S

    cpu.data[reg::addr::SREG] = sreg;

    //set_flag(cpu, reg::bit::SREG::Z, res == 0);
    //set_flag(cpu, reg::bit::SREG::V, dst & ~res & 0x8000);
    //set_flag(cpu, reg::bit::SREG::C, res & ~dst & 0x8000);
    //set_flag(cpu, reg::bit::SREG::N, res & 0x8000);
    //set_flag_s(cpu);

    cpu.pc += 1;
    return 2;
}

uint32_t instr_bset(atmega32u4_t& cpu, avr_instr_t i)
{
    // src: bit, dst: mask
    cpu.data[reg::addr::SREG] |= i.dst;
    cpu.io_reg_accessed = true;
    cpu.pc += 1;
    return 1;
}

uint32_t instr_bclr(atmega32u4_t& cpu, avr_instr_t i)
{
    // src: bit, dst: mask
    cpu.data[reg::addr::SREG] &= i.dst;
    cpu.io_reg_accessed = true;
    cpu.pc += 1;
    return 1;
}

template<bool merged>
ARDENS_FORCEINLINE static uint32_t instr_sbi(atmega32u4_t& cpu, avr_instr_t i)
{
    cpu.st_ior<merged>(i.dst, cpu.ld_ior<merged>(i.dst) | i.src);
    cpu.pc += 1;
    return 2;
}
uint32_t instr_sbi(atmega32u4_t& cpu, avr_instr_t i) { return instr_sbi<false>(cpu, i); }
uint32_t instr_merged_sbi(atmega32u4_t& cpu, avr_instr_t i) { return instr_sbi<true>(cpu, i); }

template<bool merged>
ARDENS_FORCEINLINE static uint32_t instr_cbi(atmega32u4_t& cpu, avr_instr_t i)
{
    cpu.st_ior<merged>(i.dst, cpu.ld_ior<merged>(i.dst) & ~i.src);
    cpu.pc += 1;
    return 2;
}
uint32_t instr_cbi(atmega32u4_t& cpu, avr_instr_t i) { return instr_cbi<false>(cpu, i); }
uint32_t instr_merged_cbi(atmega32u4_t& cpu, avr_instr_t i) { return instr_cbi<true>(cpu, i); }

template<bool merged>
ARDENS_FORCEINLINE static uint32_t instr_sbis(atmega32u4_t& cpu, avr_instr_t i)
{
    if(cpu.ld_ior<merged>(i.dst) & i.src)
    {
        if(next_instr_is_two_words(cpu))
        {
            cpu.pc += 3;
            return 3;
        }
        cpu.pc += 2;
        return 2;
    }
    cpu.pc += 1;
    return 1;
}
uint32_t instr_sbis(atmega32u4_t& cpu, avr_instr_t i) { return instr_sbis<false>(cpu, i); }
uint32_t instr_merged_sbis(atmega32u4_t& cpu, avr_instr_t i) { return instr_sbis<true>(cpu, i); }

template<bool merged>
ARDENS_FORCEINLINE static uint32_t instr_sbic(atmega32u4_t& cpu, avr_instr_t i)
{
    if(!(cpu.ld_ior<merged>(i.dst) & i.src))
    {
        if(next_instr_is_two_words(cpu))
        {
            cpu.pc += 3;
            return 3;
        }
        cpu.pc += 2;
        return 2;
    }
    cpu.pc += 1;
    return 1;
}
uint32_t instr_sbic(atmega32u4_t& cpu, avr_instr_t i) { return instr_sbic<false>(cpu, i); }
uint32_t instr_merged_sbic(atmega32u4_t& cpu, avr_instr_t i) { return instr_sbic<true>(cpu, i); }

uint32_t instr_sbrs(atmega32u4_t& cpu, avr_instr_t i)
{
    if(cpu.gpr(i.dst) & i.src)
    {
        const uint32_t cycles = i.m2 ? i.m2 : 2 + next_instr_is_two_words(cpu);
        cpu.pc += cycles;
        return cycles;
    }
    cpu.pc += 1;
    return 1;
}

uint32_t instr_sbrc(atmega32u4_t& cpu, avr_instr_t i)
{
    if(!(cpu.gpr(i.dst) & i.src))
    {
        const uint32_t cycles = i.m2 ? i.m2 : 2 + next_instr_is_two_words(cpu);
        cpu.pc += cycles;
        return cycles;
    }
    cpu.pc += 1;
    return 1;
}

uint32_t instr_bld(atmega32u4_t& cpu, avr_instr_t i)
{
    if(cpu.data[reg::addr::SREG] & reg::bit::SREG::T)
        cpu.gpr(i.dst) |= i.src;
    else
        cpu.gpr(i.dst) &= ~i.src;
    cpu.pc += 1;
    return 1;
}

uint32_t instr_bst(atmega32u4_t& cpu, avr_instr_t i)
{
    set_flag(cpu, reg::bit::SREG::T, cpu.gpr(i.dst) & i.src);
    cpu.pc += 1;
    return 1;
}

uint32_t instr_com(atmega32u4_t& cpu, avr_instr_t i)
{
    uint8_t src = cpu.gpr(i.dst);
    uint8_t res = ~src;
    cpu.gpr(i.dst) = res;
    set_flag(cpu, reg::bit::SREG::V, 0);
    set_flag(cpu, reg::bit::SREG::C, 1);
    set_flags_nzs(cpu, res);
    cpu.pc += 1;
    return 1;
}

uint32_t instr_neg(atmega32u4_t& cpu, avr_instr_t i)
{
    uint8_t src = cpu.gpr(i.dst);
    uint8_t res = uint8_t(-src);
    cpu.gpr(i.dst) = res;

    set_flag(cpu, reg::bit::SREG::H, (res | src) & 0x8);
    set_flag(cpu, reg::bit::SREG::V, res == 0x80);
    set_flag(cpu, reg::bit::SREG::C, res != 0x00);
    set_flags_nzs(cpu, res);
    
    cpu.pc += 1;
    return 1;
}

uint32_t instr_swap(atmega32u4_t& cpu, avr_instr_t i)
{
    uint8_t dst = cpu.gpr(i.dst);
    dst = (dst >> 4) | (dst << 4);
    cpu.gpr(i.dst) = dst;
    cpu.pc += 1;
    return 1;
}

uint32_t instr_inc(atmega32u4_t& cpu, avr_instr_t i)
{
    uint8_t dst = cpu.gpr(i.dst);
    uint8_t res = dst + 1;
    cpu.gpr(i.dst) = res;
    set_flag(cpu, reg::bit::SREG::V, res == 0x80);
    set_flags_nzs(cpu, res);
    cpu.pc += 1;
    return 1;
}

uint32_t instr_dec(atmega32u4_t& cpu, avr_instr_t i)
{
    uint8_t dst = cpu.gpr(i.dst);
    uint8_t res = dst - 1;
    cpu.gpr(i.dst) = res;
    set_flag(cpu, reg::bit::SREG::V, dst == 0x80);
    set_flags_nzs(cpu, res);
    cpu.pc += 1;
    return 1;
}

uint32_t instr_asr(atmega32u4_t& cpu, avr_instr_t i)
{
    uint8_t dst = cpu.gpr(i.dst);
    uint8_t res = (dst >> 1) | (dst & 0x80);
    cpu.gpr(i.dst) = res;
    set_flag(cpu, reg::bit::SREG::C, dst & 0x1);
    set_flag(cpu, reg::bit::SREG::V, (dst >> 7) ^ (dst & 1));
    set_flags_nzs(cpu, res);
    cpu.pc += 1;
    return 1;
}

uint32_t instr_lsr(atmega32u4_t& cpu, avr_instr_t i)
{
    uint8_t dst = cpu.gpr(i.dst);
    uint8_t res = (dst >> 1);
    cpu.gpr(i.dst) = res;
    set_flag(cpu, reg::bit::SREG::C, dst & 0x1);
    set_flag(cpu, reg::bit::SREG::V, dst & 0x1);
    set_flag(cpu, reg::bit::SREG::S, dst & 0x1);
    set_flag(cpu, reg::bit::SREG::N, 0);
    set_flag(cpu, reg::bit::SREG::Z, res == 0);
    cpu.pc += 1;
    return 1;
}

ARDENS_FORCEINLINE uint32_t instr_ror(atmega32u4_t& cpu, avr_instr_t i)
{
    uint8_t dst = cpu.gpr(i.dst);
    uint8_t res = dst >> 1;
    if(cpu.data[reg::addr::SREG] & reg::bit::SREG::C)
        res |= 0x80;
    cpu.gpr(i.dst) = res;
    set_flag(cpu, reg::bit::SREG::C, dst & 0x1);
    set_flag(cpu, reg::bit::SREG::V, (res >> 7) ^ (dst & 0x1));
    set_flags_nzs(cpu, res);
    cpu.pc += 1;
    return 1;
}

uint32_t instr_sleep(atmega32u4_t& cpu, avr_instr_t i)
{
    if(cpu.data[reg::addr::SMCR] & 0x1)
    {
        cpu.active = false;
        if(cpu.spi_busy)
            cpu.peripheral_queue.schedule(cpu.spi_done_cycle, PQ_SPI);
        if(cpu.spi_flag_cycle != UINT64_MAX)
            cpu.peripheral_queue.schedule(cpu.spi_flag_cycle, PQ_SPI);
    }
    cpu.pc += 1;
    return 1;
}

uint32_t instr_mul(atmega32u4_t& cpu, avr_instr_t i)
{
    uint16_t src = cpu.gpr(i.src);
    uint16_t dst = cpu.gpr(i.dst);
    uint16_t res = src * dst;
    cpu.gpr(0) = uint8_t(res >> 0);
    cpu.gpr(1) = uint8_t(res >> 8);
    set_flag(cpu, reg::bit::SREG::Z, res == 0);
    set_flag(cpu, reg::bit::SREG::C, res & 0x8000);
    cpu.pc += 1;
    return 2;
}

uint32_t instr_muls(atmega32u4_t& cpu, avr_instr_t i)
{
    int16_t src = (int8_t)cpu.gpr(i.src);
    int16_t dst = (int8_t)cpu.gpr(i.dst);
    int16_t res = src * dst;
    cpu.gpr(0) = uint8_t((uint16_t)res >> 0);
    cpu.gpr(1) = uint8_t((uint16_t)res >> 8);
    set_flag(cpu, reg::bit::SREG::Z, res == 0);
    set_flag(cpu, reg::bit::SREG::C, res & 0x8000);
    cpu.pc += 1;
    return 2;
}

uint32_t instr_mulsu(atmega32u4_t& cpu, avr_instr_t i)
{
    uint16_t src = cpu.gpr(i.src);
    int16_t dst = (int8_t)cpu.gpr(i.dst);
    int16_t res = src * dst;
    cpu.gpr(0) = uint8_t((uint16_t)res >> 0);
    cpu.gpr(1) = uint8_t((uint16_t)res >> 8);
    set_flag(cpu, reg::bit::SREG::Z, res == 0);
    set_flag(cpu, reg::bit::SREG::C, res & 0x8000);
    cpu.pc += 1;
    return 2;
}

uint32_t instr_fmul(atmega32u4_t& cpu, avr_instr_t i)
{
    uint16_t src = cpu.gpr(i.src);
    uint16_t dst = cpu.gpr(i.dst);
    uint16_t res = src * dst;
    cpu.gpr(0) = uint8_t((uint16_t)res << 1);
    cpu.gpr(1) = uint8_t((uint16_t)res >> 7);
    set_flag(cpu, reg::bit::SREG::Z, res == 0);
    set_flag(cpu, reg::bit::SREG::C, res & 0x8000);
    cpu.pc += 1;
    return 2;
}

uint32_t instr_fmuls(atmega32u4_t& cpu, avr_instr_t i)
{
    int16_t src = (int8_t)cpu.gpr(i.src);
    int16_t dst = (int8_t)cpu.gpr(i.dst);
    int16_t res = src * dst;
    cpu.gpr(0) = uint8_t((uint16_t)res << 1);
    cpu.gpr(1) = uint8_t((uint16_t)res >> 7);
    set_flag(cpu, reg::bit::SREG::Z, res == 0);
    set_flag(cpu, reg::bit::SREG::C, res & 0x8000);
    cpu.pc += 1;
    return 2;
}

uint32_t instr_fmulsu(atmega32u4_t& cpu, avr_instr_t i)
{
    uint16_t src = cpu.gpr(i.src);
    int16_t dst = (int8_t)cpu.gpr(i.dst);
    int16_t res = src * dst;
    cpu.gpr(0) = uint8_t((uint16_t)res << 1);
    cpu.gpr(1) = uint8_t((uint16_t)res >> 7);
    set_flag(cpu, reg::bit::SREG::Z, res == 0);
    set_flag(cpu, reg::bit::SREG::C, res & 0x8000);
    cpu.pc += 1;
    return 2;
}

uint32_t instr_merged_ldi2(atmega32u4_t& cpu, avr_instr_t i)
{
    cpu.gpr(i.dst) = i.src;
    cpu.gpr(i.m0) = i.m1;
    cpu.pc += 2;
    return 2;
}

uint32_t instr_merged_dec_brne(atmega32u4_t& cpu, avr_instr_t i)
{
    uint8_t dst = cpu.gpr(i.dst);
    uint8_t res = dst - 1;
    cpu.gpr(i.dst) = res;
    unsigned sreg = cpu.data[reg::addr::SREG] & ~(reg::bit::SREG::V | reg::bit::SREG::N | reg::bit::SREG::Z | reg::bit::SREG::S);
    if(dst == 0x80) sreg |= reg::bit::SREG::V;
    sreg = flags_nzs(sreg, res);
    cpu.data[reg::addr::SREG] = sreg;

    if(res != 0)
    {
        cpu.pc += (int16_t)i.word + 2;
        return 3;
    }

    cpu.pc += 2;
    return 2;
}

ARDENS_FORCEINLINE uint32_t instr_merged_add_adc(atmega32u4_t& cpu, avr_instr_t i)
{
    unsigned dst = cpu.gpr(i.dst) + cpu.gpr(i.dst + 1) * 256;
    unsigned src = cpu.gpr(i.src) + cpu.gpr(i.src + 1) * 256;
    unsigned res = (dst + src) & 0xffff;
    cpu.gpr(i.dst + 0) = (uint8_t)(res >> 0);
    cpu.gpr(i.dst + 1) = (uint8_t)(res >> 8);

    unsigned dst_xor_src = dst ^ src;
    unsigned hc = (dst | src) ^ (res & dst_xor_src);
    unsigned v = ~dst_xor_src & (dst ^ res);
    unsigned sreg = cpu.data[reg::addr::SREG] & ~reg::bit::SREG::HSVNZC;
    sreg |= (hc & 0x0800) >> 6;  // H flag
    sreg |= hc >> 15;            // C flag
    sreg |= (v & 0x8000) >> 12;  // V flag
    sreg = flags_nzs(sreg, uint8_t(res >> 8));
    cpu.data[reg::addr::SREG] = (uint8_t)sreg;

    cpu.pc += 2;
    return 2;
}

ARDENS_FORCEINLINE static uint32_t exec_merged_sub_sbc_inline(atmega32u4_t& cpu, avr_instr_t i)
{
    unsigned dst = cpu.gpr(i.dst) + cpu.gpr(i.dst + 1) * 256;
    unsigned src = cpu.gpr(i.src) + cpu.gpr(i.src + 1) * 256;
    unsigned res = (dst - src) & 0xffff;
    cpu.gpr(i.dst + 0) = (uint8_t)(res >> 0);
    cpu.gpr(i.dst + 1) = (uint8_t)(res >> 8);

    unsigned sreg = cpu.data[reg::addr::SREG] & ~reg::bit::SREG::HSVNZC;
    unsigned hc = (~dst & src) | (src & res) | (res & ~dst);
    unsigned v = (dst & ~src & ~res) | (~dst & src & res);
    sreg |= (hc & 0x0800) >> 6;  // H flag
    sreg |= hc >> 15;            // C flag
    sreg |= (v & 0x8000) >> 12;  // V flag
    sreg = flags_nzs16(sreg, res);
    cpu.data[reg::addr::SREG] = (uint8_t)sreg;

    cpu.pc += 2;
    return 2;
}

uint32_t instr_merged_sub_sbc(atmega32u4_t& cpu, avr_instr_t i)
{
    return exec_merged_sub_sbc_inline(cpu, i);
}

ARDENS_FORCEINLINE uint32_t instr_merged_cp_cpc(atmega32u4_t& cpu, avr_instr_t i)
{
    unsigned dst = cpu.gpr(i.dst) + cpu.gpr(i.dst + 1) * 256;
    unsigned src = cpu.gpr(i.src) + cpu.gpr(i.src + 1) * 256;
    unsigned res = (dst - src) & 0xffff;

    unsigned sreg = cpu.data[reg::addr::SREG] & ~reg::bit::SREG::HSVNZC;
    unsigned hc = (~dst & src) | (src & res) | (res & ~dst);
    unsigned v = (dst & ~src & ~res) | (~dst & src & res);
    sreg |= (hc & 0x0800) >> 6;  // H flag
    sreg |= hc >> 15;            // C flag
    sreg |= (v & 0x8000) >> 12;  // V flag
    sreg = flags_nzs16(sreg, res);
    cpu.data[reg::addr::SREG] = (uint8_t)sreg;

    cpu.pc += 2;
    return 2;
}

ARDENS_FORCEINLINE uint32_t instr_merged_subi_sbci(atmega32u4_t& cpu, avr_instr_t i)
{
    unsigned dst = cpu.gpr(i.dst) + cpu.gpr(i.src) * 256;
    unsigned src = i.word;
    unsigned res = (dst - src) & 0xffff;
    cpu.gpr(i.dst) = (uint8_t)(res >> 0);
    cpu.gpr(i.src) = (uint8_t)(res >> 8);

    unsigned sreg = cpu.data[reg::addr::SREG] & ~reg::bit::SREG::HSVNZC;
    unsigned hc = (~dst & src) | (src & res) | (res & ~dst);
    unsigned v = (dst & ~src & ~res) | (~dst & src & res);
    sreg |= (hc & 0x0800) >> 6;  // H flag
    sreg |= hc >> 15;            // C flag
    sreg |= (v & 0x8000) >> 12;  // V flag
    sreg = flags_nzs16(sreg, res);
    cpu.data[reg::addr::SREG] = (uint8_t)sreg;

    cpu.pc += 2;
    return 2;
}

ARDENS_IRAM
uint32_t instr_merged_delay(atmega32u4_t& cpu, avr_instr_t i)
{
    cpu.pc += i.src;
    return i.word;
}

ARDENS_FORCEINLINE static uint32_t instr_merged_skip_rjmp(atmega32u4_t& cpu, avr_instr_t i)
{
    const bool set = (cpu.gpr(i.dst) & i.src) != 0;
    if(set == (i.m0 != 0))
    {
        cpu.pc += 2;
        return 2;
    }
    cpu.pc += 2 + int16_t(i.word);
    return 3;
}

ARDENS_FORCEINLINE static uint32_t instr_merged_add_adc2(atmega32u4_t& cpu, avr_instr_t i)
{
    const unsigned dst = cpu.gpr(i.dst) | (unsigned(cpu.gpr(i.dst + 1)) << 8);
    const unsigned src = cpu.gpr(i.src) | (unsigned(cpu.gpr(i.m0)) << 8);
    const unsigned sum = dst + src;
    cpu.gpr(i.dst) = uint8_t(sum);
    cpu.gpr(i.dst + 1) = uint8_t(sum >> 8);
    auto& flags = cpu.data[reg::addr::SREG];
    flags = (flags & ~reg::bit::SREG::C) | (sum >> 16);
    cpu.pc += 2;
    instr_adc(cpu, {0, i.m0, uint8_t(i.dst + 2), INSTR_ADC, 0, 0, 0});
    return 3;
}

ARDENS_FORCEINLINE static uint32_t instr_merged_micros_result(atmega32u4_t& cpu, avr_instr_t i)
{
    auto& r = cpu.data;
    const uint8_t variant = static_cast<uint8_t>(i.word);
    const bool compact = (variant == 1);
    const uint32_t timer = compact
        ? ((uint32_t(r[26]) << 24) | (uint32_t(r[25]) << 16) | (uint32_t(r[24]) << 8) | r[18])
        : ((uint32_t(r[27]) << 24) | (uint32_t(r[25]) << 16) | (uint32_t(r[24]) << 8) | r[18]);
    const uint32_t sum = timer + (compact ? uint32_t(r[22]) : 0u) + uint32_t(r[1]) * 0x01010100u;
    const uint32_t result = sum << 2;
    r[22] = result;
    r[23] = result >> 8;
    r[24] = result >> 16;
    r[25] = result >> 24;
    if(variant == 0 || variant == 3)
    {
        r[26] = sum >> 16;
        r[27] = sum >> 24;
    }
    else if(variant == 2)
    {
        r[26] = uint8_t(timer >> 16);
    }
    r[20] = 0;

    // The last DEC clears N, V and S and sets Z. H and C come from the
    // preceding shift of the most significant byte.
    const uint8_t top = uint8_t(sum >> 23);
    auto& sreg = cpu.data[reg::addr::SREG];
    using namespace reg::bit::SREG;
    sreg = (sreg & (T | I)) | Z | ((top & 0x08) << 2) | (top >> 7);
    cpu.pc += compact ? 14 : 16;
    return compact ? 21 : 23;
}

#ifdef ARDUGO_EMBEDDED
ARDENS_IRAM
static uint32_t capture_micros_timer(atmega32u4_t& cpu, uint16_t counter, uint8_t timer_count)
{
    auto& r = cpu.data;
    r[19] = r[reg::addr::SREG];
    r[18] = timer_count;
    r[24] = r[counter + 0];
    r[25] = r[counter + 1];
    r[26] = r[counter + 2];
    r[27] = r[counter + 3];
    uint32_t cycles = 15;
    if(r[reg::addr::TIFR0] & reg::bit::TIFR0::TOV0)
    {
        cycles = 17;
        if(timer_count < 255)
        {
            const uint32_t low = (uint32_t(r[25]) << 8 | r[24]) + 1u;
            const uint32_t high = (uint32_t(r[27]) << 8 | r[26]) + 257u * r[1] + (low >> 16);
            r[24] = low;
            r[25] = low >> 8;
            r[26] = high;
            r[27] = high >> 8;
            cycles = 20;
        }
    }
    // OUT restores the saved SREG after CLI, so normal IRQ-enable delay applies.
    cpu.prev_sreg &= ~reg::bit::SREG::I;
    cpu.pc += 19;
    return cycles;
}
#endif

static uint32_t instr_merged_micros_timer(atmega32u4_t& cpu, avr_instr_t i)
{
#ifdef ARDUGO_EMBEDDED
    uint8_t timer_count;
    if(!cpu.peek_timer0(cpu.cycle_count + 10, timer_count)) return 0;
    return capture_micros_timer(cpu, i.word, timer_count);
#else
    return instr_merged_in(cpu, i);
#endif
}

#ifdef ARDUGO_EMBEDDED
ARDENS_IRAM
static bool micros_call_may_take_interrupt(const atmega32u4_t& cpu)
{
    // The combined call bypasses the IRQ check following OUT SREG in micros().
    // Defer to the interpreter whenever check_all_interrupts() could take an IRQ.
    using namespace reg::addr;
    const auto& r = cpu.data;
    constexpr auto twi = reg::bit::TWCR::TWINT | reg::bit::TWCR::TWIE;
    return (r[SREG] & reg::bit::SREG::I) != 0 &&
        ((r[UDINT] & r[UDIEN]) || (r[USBINT] & r[USBCON]) || r[UEINT] ||
         (r[WDTCSR] & reg::bit::WDTCSR::WDIF) ||
         (r[TIFR0] & r[TIMSK0]) || (r[TIFR1] & r[TIMSK1]) ||
         (r[TIFR3] & r[TIMSK3]) || (r[TIFR4] & r[TIMSK4]) ||
         (r[TWCR] & twi) == twi);
}

#ifdef ESP_PLATFORM
#pragma GCC push_options
#pragma GCC optimize("Os")
#endif
ARDENS_IRAM [[gnu::noinline]]
static uint32_t skip_micros_wait(atmega32u4_t& cpu, uint16_t counter, uint16_t deadline,
    uint16_t sp, int32_t budget, uint8_t mode, uint8_t loop_cycles, uint8_t start_reg)
{
    // With TOV0 clear and r1 zero, each taken poll has fixed timing.
    // Every iteration in the group must fit with the same worst-case reserve.
    const bool delay_ms = (mode == 1 || mode == 3 || mode == 4 || mode == 5);
    const bool delay_16 = (mode == 3 || mode == 4);
    const uint32_t iteration_cycles = delay_16 ? loop_cycles
        : (mode == 5 ? 66u : (mode == 1 ? 64u : ((mode == 2) ? 55u : 57u)));
    uint32_t iterations = (iteration_cycles > 0 && budget > 5)
        ? uint32_t(budget - 5) / iteration_cycles
        : 0u;
    auto const& r = cpu.data;
    if((!deadline && !delay_ms) || iterations < 2 || r[1] != 0 ||
       (r[reg::addr::TIFR0] & reg::bit::TIFR0::TOV0))
        return 0;
    const auto untouched_by_call = [sp](uint16_t address, unsigned count) {
        return sp < address || unsigned(sp - 1) >= unsigned(address) + count;
    };
    if(!untouched_by_call(counter, 4) || (deadline && !untouched_by_call(deadline, 2)))
        return 0;

    uint8_t first, last;
    if(!cpu.peek_timer0(cpu.cycle_count + 14, first))
        return 0;

    uint32_t safe_ticks = 255u - first;
    uint32_t cur_elapsed = 0;
    uint16_t target = 0, base = 0;
    const auto word = [&r](unsigned offset) {
        return uint32_t(r[offset]) | (uint32_t(r[offset + 1]) << 8) |
            (uint32_t(r[offset + 2]) << 16) | (uint32_t(r[offset + 3]) << 24);
    };
    const auto taken = [&target, &base](uint8_t ticks) {
        return (uint16_t(target - base - 4u * ticks) & 0x8000) != 0;
    };

    if(delay_ms)
    {
        if(word(12) == 0) return 0;
        cur_elapsed = delay_16
            ? uint16_t((uint16_t(word(counter)) << 10) -
                (uint16_t(r[start_reg]) | (uint16_t(r[start_reg + 1]) << 8)))
            : ((word(counter) << 10) - word(8));
        if(cur_elapsed + 4u * first >= 1000u)
            return 0;
        safe_ticks = std::min(safe_ticks, (999u - (cur_elapsed + 4u * first)) / 4u);
    }
    else
    {
        target = r[deadline] | uint16_t(r[deadline + 1]) << 8;
        base = uint16_t(uint32_t(r[counter]) << 10);
        if(!taken(first))
            return 0;
        const uint16_t gap = uint16_t(target - base - 4u * first);
        safe_ticks = std::min(safe_ticks, uint32_t((gap - 0x8000u) / 4u));
    }

    uint32_t skipped = (iterations - 1) * iteration_cycles;
    auto endpoint_ok = [&](uint8_t t_last) {
        if(t_last < first) return false;
        return delay_ms ? (cur_elapsed + 4u * t_last < 1000u) : taken(t_last);
    };
    if(!cpu.peek_timer0(cpu.cycle_count + skipped + 14, last) || !endpoint_ok(last))
    {
        // Clamp to safe Timer0 headroom instead of bailing across a deadline or TOV0 boundary.
        const uint32_t margin_ticks = (safe_ticks > 1u) ? (safe_ticks - 1u) : 0u;
        const uint32_t clamped = 1u + (margin_ticks * 64u) / iteration_cycles;
        if(clamped < 2u || clamped >= iterations)
            return 0;
        iterations = clamped;
        skipped = (iterations - 1) * iteration_cycles;
        if(!cpu.peek_timer0(cpu.cycle_count + skipped + 14, last) || !endpoint_ok(last))
            return 0;
    }

    // The counter can move by <=255 ticks inside this event window. If both
    // endpoints branch, every intermediate iteration takes the same branch.
    if(delay_ms)
    {
        // The last skipped iteration compares the 32-bit delay count to zero.
        // Its flags are visible to the next micros() call via IN r19, SREG.
        using namespace reg::bit::SREG;
        cpu.data[reg::addr::SREG] = (r[reg::addr::SREG] & (I | T)) |
            ((r[15] & 0x80) ? (N | S) : 0);
    }
#ifdef ARDUGO_QUIET_TEST
    cpu.benchmark.micros_wait_iterations += iterations - 1;
#endif
    // Execute the last poll normally to leave the same registers and stack.
    return skipped;
}

ARDENS_IRAM
static uint32_t execute_micros_call(atmega32u4_t& cpu, avr_instr_t i, int32_t budget)
{
    constexpr int32_t max_call_cycles = 4 + 20 + 24 + 4;
    constexpr int32_t max_wait_cycles = 10;
    const auto sp = cpu.sp();
    if(budget < max_call_cycles || sp <= 256 || sp >= cpu.data.size() ||
       micros_call_may_take_interrupt(cpu))
        return 0;

    const auto counter = uint16_t(i.m0 | uint16_t(i.m1) << 8);
    const uint8_t mode = i.m2;
    const bool compact = (mode == 2 || mode == 3);
    const uint16_t result_variant = compact ? 1u
        : ((uint32_t(i.word) * 2u + 46u < cpu.prog.size() &&
            cpu.prog[uint32_t(i.word) * 2u + 46u] == 0xbc) ? 2u : 0u);
    const auto deadline = (mode == 0 || mode == 2) ? uint16_t(i.src | uint16_t(i.dst) << 8) : uint16_t(0);
    const uint64_t start = cpu.cycle_count;
    const uint16_t call_pc = cpu.pc;
    const uint8_t saved_prev_sreg = cpu.prev_sreg;
    uint32_t elapsed = skip_micros_wait(cpu, counter, deadline, sp, budget, mode, i.src, i.dst);

    while(budget - static_cast<int32_t>(elapsed) >= max_call_cycles)
    {
        uint8_t timer_count;
        if(!cpu.peek_timer0(start + elapsed + 14, timer_count)) break;
        const bool finish_wait = deadline && budget - static_cast<int32_t>(elapsed) >=
            max_call_cycles + max_wait_cycles;
        elapsed += exec_call_inline(cpu, i);
        elapsed += capture_micros_timer(cpu, counter, timer_count);
        cpu.data[compact ? 22 : 27] = compact ? uint8_t(0) : cpu.data[26];
        ++cpu.pc;
        ++elapsed;
        elapsed += instr_merged_micros_result(cpu, avr_instr_t{result_variant});
        elapsed += exec_ret_inline(cpu);
#ifdef ARDUGO_QUIET_TEST
        ++cpu.benchmark.micros_calls;
#endif
        if(!finish_wait) break;

        auto& r = cpu.data;
        r[18] = r[20] = r[deadline];
        r[19] = r[21] = r[deadline + 1];
        exec_merged_sub_sbc_inline(cpu, {0, 22, 18, 0, 0, 0, 0});
        if(!(r[19] & 0x80))
        {
            cpu.pc = call_pc + 11;
            elapsed += 9;
            break;
        }
        cpu.pc = call_pc;
        elapsed += max_wait_cycles;
    }
    cpu.prev_sreg = saved_prev_sreg;
    return elapsed;
}
#ifdef ESP_PLATFORM
#pragma GCC pop_options
#endif

ARDENS_IRAM
static uint32_t execute_spi_poll(atmega32u4_t& cpu, avr_instr_t i, int32_t budget)
{
    using namespace reg::addr;
    const bool read_data = i.src == SPDR - 32;
    const int32_t iteration = 4 + read_data;
    if(budget < iteration) return 0;
    const auto start = cpu.cycle_count;
    int32_t elapsed = 0;
    do
    {
        const auto poll = start + elapsed;
        const auto status_at = poll + read_data;
        auto event = cpu.spi_flag_cycle;
        if(cpu.spi_busy)
            event = std::min(event, cpu.spi_transmit_zero_cycle -
                (cpu.spi_fast_transfer && !cpu.spi_data_latched));
        if(!cpu.spi_bus && event < start + budget)
        {
            cpu.cycle_count = start;
            return elapsed;
        }
        if(read_data)
        {
            if(event <= poll)
            {
                cpu.cycle_count = poll;
                cpu.gpr(i.dst) = atmega32u4_t::spi_handle_ld_spdr(cpu, SPDR);
            }
            else
            {
                cpu.acknowledge_spi_flags();
                cpu.gpr(i.dst) = cpu.data[SPDR];
            }
        }
        uint8_t status;
        if(event <= status_at)
        {
            cpu.cycle_count = status_at;
            status = atmega32u4_t::spi_handle_ld_spsr(cpu, SPSR);
        }
        else
            status = cpu.read_spi_status();
        cpu.gpr(i.m0) = status;
        elapsed += iteration;
        if(status & reg::bit::SPSR::SPIF)
        {
            cpu.cycle_count = start;
            cpu.pc += 3 + read_data;
            return elapsed - 1; // SBRS skips RJMP in two cycles.
        }
        // Repeated polls leave the AVR unchanged until SPI advances or a
        // further SPDR read acknowledges a write collision.
        if(event > status_at && !(read_data && cpu.spi_wcol_read))
        {
            const auto until = std::min<uint64_t>(event - poll, budget - elapsed + iteration);
            elapsed += std::max<int32_t>(0, static_cast<int32_t>(until) / iteration - 1) * iteration;
        }
    } while(budget - elapsed >= iteration);

    cpu.cycle_count = start;
    return elapsed;
}

#ifdef ARDUGO_EMBEDDED
ARDENS_NOINLINE ARDENS_IRAM
static void core_send_oled_data(arduboy_core_state_t& state, uint8_t byte)
{
    display_send_data_common(state.oled, byte);
}

ARDENS_FORCEINLINE static uint8_t core_exchange_spi_byte_fast(
    atmega32u4_t& cpu, uint8_t byte, uint64_t at)
{
    auto* state = cpu.core_state;
    const uint8_t port = cpu.data[reg::addr::PORTD];
    if(state->fx_selected && (port & reg::bit::PORTD::PORTD6) && !state->fx_busy) [[likely]]
        return exchange_fx_spi_fast(*state, byte, at);
    if(!(port & reg::bit::PORTD::PORTD6) && (port & reg::bit::PORTD::PORTD4) &&
       !state->fx_selected && (state->oled_in_reset || at < state->oled_next_row))
    {
        core_send_oled_data(*state, byte);
        return 0;
    }
    return cpu.spi_bus->exchange_spi_byte(byte, at);
}

ARDENS_FORCEINLINE static void service_active_master_spi_inline(atmega32u4_t& cpu, uint64_t now)
{
    if(now >= cpu.spi_flag_cycle)
    {
        cpu.data[reg::addr::SPSR] |= reg::bit::SPSR::SPIF;
        cpu.spi_flag_cycle = UINT64_MAX;
    }
    if(cpu.spi_busy)
    {
        const uint64_t receive_cycle =
            cpu.spi_transmit_zero_cycle - (cpu.spi_fast_transfer ? 1 : 0);
        if(!cpu.spi_data_latched && now >= receive_cycle)
        {
            cpu.spi_datain_byte = core_exchange_spi_byte_fast(cpu, cpu.spi_data_byte, receive_cycle);
            cpu.data[reg::addr::SPDR] = cpu.spi_datain_byte;
            cpu.spi_data_latched = true;
        }
        if(now >= cpu.spi_transmit_zero_cycle)
        {
            cpu.spi_busy = false;
            cpu.spi_flag_cycle = cpu.spi_transmit_zero_cycle + (cpu.spi_fast_transfer ? 1 : 0);
            if(now >= cpu.spi_flag_cycle)
            {
                cpu.data[reg::addr::SPSR] |= reg::bit::SPSR::SPIF;
                cpu.spi_flag_cycle = UINT64_MAX;
            }
        }
    }
}

ARDENS_NOINLINE ARDENS_IRAM
static uint8_t spdr_ld_slow(atmega32u4_t& cpu)
{
    return atmega32u4_t::spi_handle_ld_spdr(cpu, reg::addr::SPDR);
}

ARDENS_NOINLINE ARDENS_IRAM
static void spdr_st_slow(atmega32u4_t& cpu, uint8_t val)
{
    atmega32u4_t::spi_handle_st_spdr(cpu, reg::addr::SPDR, val);
}

ARDENS_NOINLINE ARDENS_IRAM
static uint8_t spdr_ld_loop(atmega32u4_t& cpu)
{
    const uint64_t now = cpu.cycle_count;
    if(cpu.spi_bus && cpu.active &&
       !(cpu.data[reg::addr::SPCR] & reg::bit::SPCR::SPIE)) [[likely]]
    {
        service_active_master_spi_inline(cpu, now);
        cpu.acknowledge_spi_flags();
        return cpu.data[reg::addr::SPDR];
    }
    return spdr_ld_slow(cpu);
}

ARDENS_NOINLINE ARDENS_IRAM
static void spdr_st_loop(atmega32u4_t& cpu, uint8_t val)
{
    const uint64_t now = cpu.cycle_count;
    const uint8_t spcr = cpu.data[reg::addr::SPCR];
    constexpr uint8_t SPE = reg::bit::SPCR::SPE;
    constexpr uint8_t MSTR = reg::bit::SPCR::MSTR;
    constexpr uint8_t SPIE = reg::bit::SPCR::SPIE;
    constexpr uint8_t DORD = reg::bit::SPCR::DORD;
    if(cpu.spi_bus && cpu.active &&
       (spcr & (SPE | MSTR | SPIE | DORD)) == (SPE | MSTR) &&
       cpu.spi_clock_cycles >= 2) [[likely]]
    {
        service_active_master_spi_inline(cpu, now);
        cpu.acknowledge_spi_flags();
        if(!cpu.spi_busy && cpu.spi_transmit_zero_cycle != now) [[likely]]
        {
            cpu.spi_data_byte = val;
            cpu.spi_fast_transfer = (cpu.spi_clock_cycles == 2);
            cpu.spi_transmit_zero_cycle = now + cpu.spi_clock_cycles * 8 + 1;
            cpu.spi_done_cycle = cpu.spi_transmit_zero_cycle - (cpu.spi_fast_transfer ? 1 : 0);
            cpu.spi_data_latched = false;
            cpu.spi_busy = true;
            return;
        }
    }
    spdr_st_slow(cpu, val);
}
#endif

ARDENS_IRAM
static uint32_t execute_oled_send(atmega32u4_t& cpu, int32_t budget)
{
    if(!cpu.spi_bus || budget < 18) return 0;
    auto& r = cpu.data;
    const auto start = cpu.cycle_count;
    uint16_t cursor = cpu.z_word();
    uint16_t count = cpu.x_word();
    uint16_t last_sbiw_operand = count;
    uint8_t last_byte = r[0];
    const bool clear = r[24] != r[1];
    uint32_t elapsed = 0;
    do
    {
        if(cursor < 256 || cursor >= r.size() || budget - int32_t(elapsed) < 18) break;
        const uint8_t byte = r[cursor];
        cpu.cycle_count = start + elapsed + 2; // LD Z precedes OUT.
#ifdef ARDUGO_EMBEDDED
        spdr_st_loop(cpu, byte);
#else
        atmega32u4_t::spi_handle_st_spdr(cpu, reg::addr::SPDR, byte);
#endif
        last_byte = clear ? r[1] : byte;
        r[cursor++] = last_byte;

        const bool odd = (count & 1) != 0;
        last_sbiw_operand = uint16_t(count - !odd);
        count = uint16_t(last_sbiw_operand - 1);
        elapsed += (odd ? 13 : 18) - (count == 0);
    } while(count);
    if(!elapsed) return 0;

    // Only the last SBIW is observable. Intermediate branches depend on the
    // countdown bits and do not change the other AVR flags.
    const auto pc = cpu.pc;
    r[26] = uint8_t(last_sbiw_operand);
    r[27] = uint8_t(last_sbiw_operand >> 8);
    instr_sbiw(cpu, {0, 1, 26, INSTR_SBIW, 0, 0, 0});
    r[0] = last_byte;
    r[30] = uint8_t(cursor);
    r[31] = uint8_t(cursor >> 8);
    cpu.pc = count ? pc : pc + 9;
    return elapsed;
}

ARDENS_IRAM
static uint32_t execute_fx_blend(atmega32u4_t& cpu, int32_t budget, bool alt = false)
{
    if(!cpu.spi_bus || budget < 19) return 0;
    auto& r = cpu.data;
    const auto start = cpu.cycle_count;
    uint16_t x = cpu.x_word();
    uint16_t z = cpu.z_word();
    const uint8_t count_reg = alt ? 14 : 7;
    const uint8_t rx_reg = alt ? 16 : 24;
    const uint8_t mul_reg = alt ? 20 : 18;
    const uint8_t tmp_reg = alt ? 9 : 6;
    const uint8_t mx_reg = alt ? 12 : 28;
    const uint8_t mz_reg = alt ? 13 : 29;
    uint8_t count = r[count_reg], received = r[rx_reg], high = r[1], blended = r[tmp_reg];
    uint16_t product = 0;
    uint32_t elapsed = 0;
    do
    {
        if(x < 256 || z < 256 || x >= r.size() || z >= r.size() || budget - int32_t(elapsed) < 19)
            break;
        cpu.cycle_count = start + elapsed;
#ifdef ARDUGO_EMBEDDED
        received = spdr_ld_loop(cpu);
        ++cpu.cycle_count;
        spdr_st_loop(cpu, high);
#else
        received = atmega32u4_t::spi_handle_ld_spdr(cpu, reg::addr::SPDR);
        ++cpu.cycle_count;
        atmega32u4_t::spi_handle_st_spdr(cpu, reg::addr::SPDR, high);
#endif
        product = uint16_t(received) * r[mul_reg];
        high = uint8_t(product >> 8);
        r[x] = (r[x] & r[mx_reg]) | uint8_t(product);
        ++x;
        blended = (r[z] & r[mz_reg]) | high;
        r[z++] = blended;
        --count;
        elapsed += 19 - (count == 0);
    } while(count);
    if(!elapsed) return 0;

    // DEC overwrites the NZVS flags set by the logical ops. Carry comes from
    // the last MUL and H, T and I are preserved through the entire loop.
    auto& flags = r[reg::addr::SREG];
    flags = (flags & ~reg::bit::SREG::C) | (product >> 15);
    const auto pc = cpu.pc;
    r[count_reg] = uint8_t(count + 1);
    instr_dec(cpu, {0, 0, count_reg, INSTR_DEC, 0, 0, 0});
    r[0] = uint8_t(product);
    r[1] = high;
    r[tmp_reg] = blended;
    r[rx_reg] = received;
    r[26] = uint8_t(x);
    r[27] = uint8_t(x >> 8);
    r[30] = uint8_t(z);
    r[31] = uint8_t(z >> 8);
    cpu.pc = count ? pc : pc + 13;
    return elapsed;
}

ARDENS_IRAM
static uint32_t popcnt32(uint32_t v)
{
    v = v - ((v >> 1) & 0x55555555u);
    v = (v & 0x33333333u) + ((v >> 2) & 0x33333333u);
    v = (v + (v >> 4)) & 0x0f0f0f0fu;
    return (v * 0x01010101u) >> 24;
}

ARDENS_IRAM
static uint8_t udivmod_sreg(uint8_t prev_sreg, uint8_t q_msb)
{
    uint8_t sreg = (prev_sreg & (reg::bit::SREG::I | reg::bit::SREG::T)) | reg::bit::SREG::C;
    // The final loop step executes adc r25,r25 before dec/com, setting H from bit 4 of r25 (~q_msb).
    if((q_msb & 0x10u) == 0) sreg |= reg::bit::SREG::H;
    if(q_msb == 0) sreg |= reg::bit::SREG::Z;
    if(q_msb & 0x80u) sreg |= reg::bit::SREG::N | reg::bit::SREG::S;
    return sreg;
}

ARDENS_IRAM
static uint32_t execute_udivmod4(atmega32u4_t& cpu, int32_t budget, uint32_t* needed_cycles = nullptr)
{
    auto& r = cpu.data;
    const uint32_t num = uint32_t(r[22]) | (uint32_t(r[23]) << 8) |
                         (uint32_t(r[24]) << 16) | (uint32_t(r[25]) << 24);
    const uint32_t den = uint32_t(r[18]) | (uint32_t(r[19]) << 8) |
                         (uint32_t(r[20]) << 16) | (uint32_t(r[21]) << 24);
    const uint32_t q = (den != 0) ? (num / den) : 0xffffffffu;
    const uint32_t rem = (den != 0) ? (num - q * den) : num;
    const uint32_t cycles = 569u + 3u * popcnt32(q);
    const uint16_t sp_val = cpu.sp();
    if(budget < int32_t(cycles) || sp_val < 256 || size_t(sp_val) + 2 >= r.size())
    {
        if(needed_cycles && sp_val >= 256 && size_t(sp_val) + 2 < r.size())
            *needed_cycles = cycles;
        return 0;
    }

    const uint16_t next_sp = uint16_t(sp_val + 2u);
    const uint16_t ret_pc = uint16_t(r[next_sp]) | (uint16_t(r[next_sp - 1]) << 8);
    r[reg::addr::SPL] = uint8_t(next_sp);
    r[reg::addr::SPH] = uint8_t(next_sp >> 8);
    cpu.pc = ret_pc & 0x3fffu;

    r[1] = 0;
    r[18] = uint8_t(q);
    r[19] = uint8_t(q >> 8);
    r[20] = uint8_t(q >> 16);
    r[21] = uint8_t(q >> 24);
    r[22] = r[26] = uint8_t(rem);
    r[23] = r[27] = uint8_t(rem >> 8);
    r[24] = r[30] = uint8_t(rem >> 16);
    r[25] = r[31] = uint8_t(rem >> 24);

    r[reg::addr::SREG] = udivmod_sreg(r[reg::addr::SREG], uint8_t(q >> 24));
    return cycles;
}

ARDENS_IRAM
static uint32_t execute_udivmod2(atmega32u4_t& cpu, int32_t budget, uint32_t* needed_cycles = nullptr)
{
    auto& r = cpu.data;
    const uint32_t num = uint32_t(r[24]) | (uint32_t(r[25]) << 8);
    const uint32_t den = uint32_t(r[22]) | (uint32_t(r[23]) << 8);
    const uint16_t q = (den != 0) ? uint16_t(num / den) : 0xffffu;
    const uint16_t rem = (den != 0) ? uint16_t(num - uint32_t(q) * den) : uint16_t(num);
    const uint32_t cycles = 193u + popcnt32(uint32_t(q));
    const uint16_t sp_val = cpu.sp();
    if(budget < int32_t(cycles) || sp_val < 256 || size_t(sp_val) + 2 >= r.size())
    {
        if(needed_cycles && sp_val >= 256 && size_t(sp_val) + 2 < r.size())
            *needed_cycles = cycles;
        return 0;
    }

    const uint16_t next_sp = uint16_t(sp_val + 2u);
    const uint16_t ret_pc = uint16_t(r[next_sp]) | (uint16_t(r[next_sp - 1]) << 8);
    r[reg::addr::SPL] = uint8_t(next_sp);
    r[reg::addr::SPH] = uint8_t(next_sp >> 8);
    cpu.pc = ret_pc & 0x3fffu;

    r[21] = 0;
    r[22] = uint8_t(q);
    r[23] = uint8_t(q >> 8);
    r[24] = r[26] = uint8_t(rem);
    r[25] = r[27] = uint8_t(rem >> 8);

    r[reg::addr::SREG] = udivmod_sreg(r[reg::addr::SREG], uint8_t(q >> 8));
    return cycles;
}

#ifdef ESP_PLATFORM
#pragma GCC push_options
#pragma GCC optimize("Os")
#endif
ARDENS_IRAM
static uint32_t execute_fx_masked_blend(atmega32u4_t& cpu, int32_t budget)
{
    if(!cpu.spi_bus) return 0;
    auto& r = cpu.data;
    const uint8_t mode = r[9];
    const bool extra_byte = (mode & 0x10) != 0;
    const bool low_visible = (r[16] & 0x80) == 0;
    const bool high_visible = (mode & 0x80) != 0;
    const unsigned stride = mode & 0x20 ? 0x81 : 0x7f;
    const int32_t iteration_cycles = (extra_byte ? 25 : 16) + 2 +
        (low_visible ? 10 : 3) + 2 + (high_visible ? 10 : 3) + 9;
    const auto start = cpu.cycle_count;
    uint16_t cursor = cpu.x_word();
    uint8_t count = r[25], scratch = r[24], outgoing = r[1];
    uint8_t mask_low = 0, mask_high = 0, result_low = 0;
    bool borrow = false, half_borrow = false;
    uint32_t elapsed = 0;
    do
    {
        const auto upper_row = uint16_t(cursor + 128);
        if(budget - int32_t(elapsed) < iteration_cycles ||
           (low_visible && (cursor < 256 || cursor >= r.size())) ||
           (high_visible && (upper_row < 256 || upper_row >= r.size())))
            break;

        cpu.cycle_count = start + elapsed;
#ifdef ARDUGO_EMBEDDED
        uint8_t pixels = spdr_ld_loop(cpu);
        ++cpu.cycle_count;
        spdr_st_loop(cpu, outgoing);
#else
        uint8_t pixels = atmega32u4_t::spi_handle_ld_spdr(cpu, reg::addr::SPDR);
        ++cpu.cycle_count;
        atmega32u4_t::spi_handle_st_spdr(cpu, reg::addr::SPDR, outgoing);
#endif
        if(mode & 8) pixels ^= r[22];
        scratch = (mode & 1) ? pixels : r[22];
        const auto mask = uint16_t((mode & 4) ? 0 : pixels) * r[19];
        mask_low = uint8_t(mask);
        mask_high = uint8_t(mask >> 8);

        pixels = scratch;
        if(extra_byte)
        {
            cpu.cycle_count = start + elapsed + 21; // Two LPMs and CLR pad the next SPI read.
#ifdef ARDUGO_EMBEDDED
            const auto next = spdr_ld_loop(cpu);
            ++cpu.cycle_count;
            spdr_st_loop(cpu, 0);
#else
            const auto next = atmega32u4_t::spi_handle_ld_spdr(cpu, reg::addr::SPDR);
            ++cpu.cycle_count;
            atmega32u4_t::spi_handle_st_spdr(cpu, reg::addr::SPDR, 0);
#endif
            if(!(mode & 1)) pixels = next;
        }

        const auto product = uint16_t(pixels) * r[19];
        result_low = uint8_t(product);
        if(low_visible)
        {
            scratch = r[cursor];
            if(!(mode & 2)) mask_low ^= scratch;
            r[cursor] = mask_low = (mask_low & result_low) ^ scratch;
        }
        if(high_visible)
        {
            scratch = r[upper_row];
            if(!(mode & 2)) mask_high ^= scratch;
            r[upper_row] = mask_high = (mask_high & uint8_t(product >> 8)) ^ scratch;
        }

        const auto hi = uint8_t(upper_row >> 8);
        const bool low_borrow = uint8_t(upper_row) < stride;
        borrow = low_borrow && hi == 0;
        half_borrow = low_borrow && (hi & 15) == 0;
        cursor = uint16_t(upper_row - stride);
        outgoing = 0;
        --count;
        elapsed += iteration_cycles - (count == 0);
    } while(count);
    if(!elapsed) return 0;

    using namespace reg::bit::SREG;
    auto& flags = r[reg::addr::SREG];
    flags = (flags & ~(H | T | C)) | (borrow ? C : 0) | (half_borrow ? H : 0) | (extra_byte ? T : 0);
    const auto pc = cpu.pc;
    r[25] = uint8_t(count + 1);
    instr_dec(cpu, {0, 0, 25, INSTR_DEC, 0, 0, 0});
    r[0] = result_low;
    r[1] = 0;
    r[20] = mask_low;
    r[21] = mask_high;
    r[24] = scratch;
    r[26] = uint8_t(cursor);
    r[27] = uint8_t(cursor >> 8);
    cpu.pc = count ? pc : pc + 47;
    return elapsed;
}
#ifdef ESP_PLATFORM
#pragma GCC pop_options
#endif

uint32_t atmega32u4_t::execute_merged(int32_t budget, const uint16_t* jit_block_off)
{
    const uint64_t start = cycle_count;
    uint32_t elapsed = 0;
    io_reg_accessed = false;
    bool irq_delay = false;
#ifdef ESP_PLATFORM
    uint16_t cached_pc = UINT16_MAX;
    const avr_instr_t* cached_line = nullptr;
#endif

    do
    {
        if(pc >= PROG_SIZE_BYTES / 2)
            pc &= PROG_SIZE_BYTES / 2 - 1;
#ifdef ESP_PLATFORM
        const uint16_t block = pc & ~(INSTR_CACHE_BLOCK - 1);
        if(block != cached_pc)
        {
            cached_line = &merged_instruction(block);
            cached_pc = block;
        }
        const auto& i = cached_line[pc % INSTR_CACHE_BLOCK];
#else
        const auto i = merged_prog[pc];
#endif
        const bool clear_irq_delay = irq_delay;
        bool just_enabled_irq = false;
        bool force_exit = false;
        uint32_t cycles;
        // Pure register operations don't observe time or change the interrupt
        // enable bit. Commit the clock before running a general handler.
        switch(i.func)
        {
        case INSTR_ADC:             cycles = instr_adc(*this, i); break;
        case INSTR_ADD:             cycles = instr_add(*this, i); break;
        case INSTR_AND:             cycles = instr_and(*this, i); break;
        case INSTR_ANDI:            cycles = instr_andi(*this, i); break;
        case INSTR_ASR:             cycles = instr_asr(*this, i); break;
        case INSTR_ADIW:            cycles = instr_adiw(*this, i); break;
        case INSTR_BCLR:
        {
            const uint8_t old_i = data[reg::addr::SREG] & reg::bit::SREG::I;
            data[reg::addr::SREG] &= i.dst;
            ++pc;
            cycles = 1;
            if(old_i && !(i.dst & reg::bit::SREG::I))
            {
                prev_sreg &= ~reg::bit::SREG::I;
                irq_delay = false;
            }
            break;
        }
        case INSTR_BSET:
        {
            const uint8_t old_i = data[reg::addr::SREG] & reg::bit::SREG::I;
            data[reg::addr::SREG] |= i.dst;
            ++pc;
            cycles = 1;
            if(!old_i && (i.dst & reg::bit::SREG::I))
            {
                if(has_pending_interrupt())
                {
                    irq_delay = false;
                    force_exit = true;
                }
                else
                    irq_delay = true;
                just_enabled_irq = true;
            }
            break;
        }
        case INSTR_BRBC:            cycles = instr_brbc(*this, i); break;
        case INSTR_BRBS:            cycles = instr_brbs(*this, i); break;
        case INSTR_CLR:             cycles = instr_clr(*this, i); break;
        case INSTR_CP:              cycles = instr_cp(*this, i); break;
        case INSTR_CPC:             cycles = instr_cpc(*this, i); break;
        case INSTR_CPI:             cycles = instr_cpi(*this, i); break;
        case INSTR_CPSE:            cycles = instr_cpse(*this, i); break;
        case INSTR_DEC:             cycles = instr_dec(*this, i); break;
        case INSTR_EOR:             cycles = instr_eor(*this, i); break;
        case INSTR_IJMP:
        {
            const uint16_t z = z_word();
            if(z >= last_addr / 2) goto timed;
            pc = z;
            cycles = 2;
            break;
        }
        case INSTR_INC:             cycles = instr_inc(*this, i); break;
        case INSTR_JMP:             cycles = instr_jmp(*this, i); break;
        case INSTR_LDI:             cycles = instr_ldi(*this, i); break;
        case INSTR_LPM:             cycles = instr_lpm(*this, i); break;
        case INSTR_LSR:             cycles = instr_lsr(*this, i); break;
        case INSTR_MOV:             cycles = instr_mov(*this, i); break;
        case INSTR_MOVW:            cycles = instr_movw(*this, i); break;
        case INSTR_MUL:             cycles = instr_mul(*this, i); break;
        case INSTR_NOP:             cycles = instr_nop(*this, i); break;
        case INSTR_OR:              cycles = instr_or(*this, i); break;
        case INSTR_ORI:             cycles = instr_ori(*this, i); break;
#ifdef ARDENS_NO_DEBUGGER
        case INSTR_CALL:
        case INSTR_RCALL:
        case INSTR_ICALL:
            if(!return_address_fits(*this, unsigned(sp()) - 1)) goto timed;
            if(i.func == INSTR_RCALL) cycles = instr_rcall(*this, i);
            else if(i.func == INSTR_CALL) cycles = instr_call(*this, i);
            else cycles = instr_icall(*this, i);
            break;
        case INSTR_PUSH:
        {
            uint16_t tsp = sp();
            if(tsp < st_handlers.size() || tsp >= data.size()) goto timed;
            data[tsp] = gpr(i.src);
            --tsp;
            data[reg::addr::SPL] = uint8_t(tsp);
            data[reg::addr::SPH] = uint8_t(tsp >> 8);
            pushed_at_least_once = true;
            ++pc;
            cycles = 2;
            break;
        }
        case INSTR_POP:
        {
            const uint16_t tsp = uint16_t(sp() + 1);
            if(tsp < ld_handlers.size() || tsp >= data.size()) goto timed;
            gpr(i.src) = data[tsp];
            data[reg::addr::SPL] = uint8_t(tsp);
            data[reg::addr::SPH] = uint8_t(tsp >> 8);
            ++pc;
            cycles = 2;
            break;
        }
        case INSTR_RET:
            if(!return_address_fits(*this, unsigned(sp()) + 1)) goto timed;
            cycles = instr_ret(*this, i);
            break;
        case INSTR_RETI:
        {
            if(!return_address_fits(*this, unsigned(sp()) + 1)) goto timed;
            pc = pop_return_address(*this);
            const uint8_t old_i = data[reg::addr::SREG] & reg::bit::SREG::I;
            data[reg::addr::SREG] |= reg::bit::SREG::I;
            if(!old_i)
            {
                if(has_pending_interrupt())
                {
                    irq_delay = false;
                    force_exit = true;
                }
                else
                    irq_delay = true;
                just_enabled_irq = true;
            }
            cycles = 4;
            break;
        }
#endif
        case INSTR_RJMP:
            if(i.word == UINT16_MAX)
            {
                // A jump to itself has no observable effects until the next event.
                const int32_t left = budget - static_cast<int32_t>(elapsed);
                cycles = 2u * std::max<int32_t>(1, (left + 1) / 2);
                force_exit = true;
                break;
            }
            cycles = instr_rjmp(*this, i);
            break;
        case INSTR_ROR:             cycles = instr_ror(*this, i); break;
        case INSTR_SBC:             cycles = instr_sbc(*this, i); break;
        case INSTR_SBCI:            cycles = instr_sbci(*this, i); break;
        case INSTR_SBIW:            cycles = instr_sbiw(*this, i); break;
        case INSTR_SBRC:            cycles = instr_sbrc(*this, i); break;
        case INSTR_SBRS:            cycles = instr_sbrs(*this, i); break;
        case INSTR_SUB:             cycles = instr_sub(*this, i); break;
        case INSTR_SUBI:            cycles = instr_subi(*this, i); break;

        // SPI reads and writes advance SPI state inline without exiting the batch.
        case INSTR_MERGED_IN:
        handle_merged_in:
        {
            const uint16_t addr = i.src + 32;
            if(addr == reg::addr::SPDR || addr == reg::addr::SPSR)
            {
                if(spi_flag_cycle <= start + elapsed ||
                   (spi_busy && spi_transmit_zero_cycle -
                       (spi_fast_transfer && !spi_data_latched) <= start + elapsed))
                {
                    cycle_count = start + elapsed;
                    gpr(i.dst) = (addr == reg::addr::SPDR)
                        ? spi_handle_ld_spdr(*this, reg::addr::SPDR)
                        : spi_handle_ld_spsr(*this, reg::addr::SPSR);
#ifdef ARDUGO_EMBEDDED
                    if(!spi_bus)
#endif
                        if(const auto due = peripheral_queue.next_cycle(); due < start + budget + MAX_INSTR_CYCLES)
                            budget = due <= start + MAX_INSTR_CYCLES ? 0 :
                                static_cast<int32_t>(due - start - MAX_INSTR_CYCLES);
                }
                else if(addr == reg::addr::SPDR)
                {
                    acknowledge_spi_flags();
                    gpr(i.dst) = data[reg::addr::SPDR];
                }
                else
                    gpr(i.dst) = read_spi_status();
                ++pc;
                cycles = 1;
            }
            else if(!ld_handlers[addr])
            {
                gpr(i.dst) = data[addr];
                ++pc;
                cycles = 1;
            }
#ifdef ARDUGO_EMBEDDED
            else if(addr == reg::addr::TCNT0)
            {
                uint8_t value;
                if(!peek_timer0(start + elapsed, value)) goto timed;
                gpr(i.dst) = value;
                ++pc;
                cycles = 1;
            }
#endif
            else
                goto timed;
            break;
        }
        case INSTR_MERGED_OUT:
        {
            const uint16_t addr = i.dst + 32;
            if(addr == reg::addr::SPDR)
            {
                cycle_count = start + elapsed;
                spi_handle_st_spdr(*this, reg::addr::SPDR, gpr(i.src));
                ++pc;
#ifdef ARDUGO_EMBEDDED
                if(!spi_bus)
#endif
                    if(const auto due = peripheral_queue.next_cycle(); due < start + budget + MAX_INSTR_CYCLES)
                        budget = due <= start + MAX_INSTR_CYCLES ? 0 :
                            static_cast<int32_t>(due - start - MAX_INSTR_CYCLES);
                cycles = 1;
            }
            else if(addr == reg::addr::SREG)
            {
                const uint8_t old_i = data[reg::addr::SREG] & reg::bit::SREG::I;
                const uint8_t val = gpr(i.src);
                data[reg::addr::SREG] = val;
                const uint8_t new_i = val & reg::bit::SREG::I;
                ++pc;
                cycles = 1;
                if(new_i != old_i)
                {
                    if(new_i)
                    {
                        if(has_pending_interrupt())
                        {
                            irq_delay = false;
                            force_exit = true;
                        }
                        else
                            irq_delay = true;
                        just_enabled_irq = true;
                    }
                    else
                    {
                        prev_sreg &= ~reg::bit::SREG::I;
                        irq_delay = false;
                    }
                }
            }
            else if(!st_handlers[addr])
            {
                data[addr] = gpr(i.src);
                ++pc;
                cycles = 1;
            }
#ifdef ARDUGO_EMBEDDED
            else if(spi_bus && st_handlers[addr] == st_handle_port && !twi_adapter)
            {
                cycle_count = start + elapsed;
                st_handle_port(*this, addr, gpr(i.src));
                ++pc;
                cycles = 1;
            }
            else if(spi_bus && st_handlers[addr] == st_handle_pin)
            {
                cycle_count = start + elapsed;
                st_handle_pin(*this, addr, gpr(i.src));
                ++pc;
                cycles = 1;
            }
#endif
            else
                goto timed;
            break;
        }
        case INSTR_MERGED_SBI:
        case INSTR_MERGED_CBI:
        {
            const uint16_t addr = i.dst + 32;
            if(!ld_handlers[addr] &&
               (!st_handlers[addr]
#ifdef ARDUGO_EMBEDDED
                || (spi_bus && st_handlers[addr] == st_handle_port && !twi_adapter)
#endif
               ))
            {
                const uint8_t val = (i.func == INSTR_MERGED_SBI)
                    ? uint8_t(data[addr] | i.src)
                    : uint8_t(data[addr] & ~i.src);
                if(st_handlers[addr])
                {
                    cycle_count = start + elapsed;
                    st_handlers[addr](*this, addr, val);
                }
                else
                    data[addr] = val;
                ++pc;
                cycles = 2;
                break;
            }
            goto timed;
        }
        case INSTR_MERGED_SBIS:
        case INSTR_MERGED_SBIC:
        {
            const uint16_t addr = i.dst + 32;
            if(!ld_handlers[addr])
            {
                const bool set = (data[addr] & i.src) != 0;
                const bool skip = (i.func == INSTR_MERGED_SBIS) ? set : !set;
                if(skip)
                {
                    cycles = next_instr_is_two_words(*this) ? 3 : 2;
                    pc += cycles;
                }
                else
                {
                    ++pc;
                    cycles = 1;
                }
                break;
            }
            goto timed;
        }

        // SRAM doesn't depend on cycle_count. Execute common memory operations
        // directly, but use the timed handler for registers and peripheral I/O.
        case INSTR_MERGED_LDS:
            if(i.word < ld_handlers.size()) goto timed;
            cycles = instr_lds<true>(*this, i);
            break;
        case INSTR_MERGED_STS:
            if(i.word < st_handlers.size()) goto timed;
            cycles = instr_sts<true>(*this, i);
            break;
        case INSTR_MERGED_LDD_Y:
            if(uint16_t(y_word() + i.dst) < ld_handlers.size()) goto timed;
            cycles = instr_ldd_y<true>(*this, i);
            break;
        case INSTR_MERGED_LDD_Z:
            if(uint16_t(z_word() + i.dst) < ld_handlers.size()) goto timed;
            cycles = instr_ldd_z<true>(*this, i);
            break;
        case INSTR_MERGED_STD_Y:
            if(uint16_t(y_word() + i.dst) < st_handlers.size()) goto timed;
            cycles = instr_std_y<true>(*this, i);
            break;
        case INSTR_MERGED_STD_Z:
            if(uint16_t(z_word() + i.dst) < st_handlers.size()) goto timed;
            cycles = instr_std_z<true>(*this, i);
            break;
        case INSTR_MERGED_LD_X:
            if(x_word() < ld_handlers.size()) goto timed;
            cycles = instr_super_ld_st<true, true, 0, false, false>(*this, i);
            break;
        case INSTR_MERGED_LD_Z_INC:
            if(z_word() < ld_handlers.size()) goto timed;
            cycles = instr_super_ld_st<true, true, 2, true, false>(*this, i);
            break;
        case INSTR_MERGED_ST_X_INC:
            if(x_word() < st_handlers.size()) goto timed;
            cycles = instr_super_ld_st<true, false, 0, true, false>(*this, i);
            break;
        case INSTR_MERGED_ST_Z_INC:
            if(z_word() < st_handlers.size()) goto timed;
            cycles = instr_super_ld_st<true, false, 2, true, false>(*this, i);
            break;

        case INSTR_MERGED_ADD_ADC:  cycles = instr_merged_add_adc(*this, i); break;
        case INSTR_MERGED_CP_CPC:   cycles = instr_merged_cp_cpc(*this, i); break;
        case INSTR_MERGED_DEC_BRNE: cycles = instr_merged_dec_brne(*this, i); break;
        case INSTR_MERGED_DELAY:    cycles = instr_merged_delay(*this, i); break;
        case INSTR_MERGED_LDI2:     cycles = instr_merged_ldi2(*this, i); break;
        case INSTR_MERGED_MICROS_RESULT:
            // Interrupts and timer updates must still be able to land inside the sequence.
            if(budget - static_cast<int32_t>(elapsed) >= 23)
            {
#ifdef ARDUGO_QUIET_TEST
                ++benchmark.micros_blocks;
#endif
                cycles = instr_merged_micros_result(*this, i);
            }
            else
                cycles = instr_mov(*this, i);
            break;
        case INSTR_MERGED_MICROS_TIMER:
            cycle_count = start + elapsed;
            if(budget - static_cast<int32_t>(elapsed) >= 20)
            {
                cycles = instr_merged_micros_timer(*this, i);
                if(cycles != 0)
                {
#ifdef ARDUGO_QUIET_TEST
                    ++benchmark.micros_timer_blocks;
#endif
                    if(data[reg::addr::SREG] & reg::bit::SREG::I)
                    {
                        if(has_pending_interrupt())
                        {
                            elapsed += cycles;
                            cycle_count = start + elapsed;
                            return elapsed;
                        }
                        irq_delay = true;
                        just_enabled_irq = true;
                    }
                    break;
                }
            }
            cycles = instr_merged_in(*this, i);
            break;
        case INSTR_MERGED_MICROS_CALL:
            cycle_count = start + elapsed;
            cycles = execute_micros_call(*this, i, budget - static_cast<int32_t>(elapsed));
            if(cycles == 0) cycles = instr_call(*this, i);
            break;
        case INSTR_MERGED_SPI_POLL:
            cycle_count = start + elapsed;
            cycles = execute_spi_poll(*this, i, budget - static_cast<int32_t>(elapsed));
            if(cycles == 0)
            {
#ifdef ARDUGO_EMBEDDED
                if(spi_bus)
                    goto handle_merged_in;
#endif
                cycles = instr_merged_in(*this, i);
                force_exit = true;
            }
            break;
        case INSTR_MERGED_SUBI_SBCI: cycles = instr_merged_subi_sbci(*this, i); break;
        case INSTR_MERGED_SKIP_RJMP: cycles = instr_merged_skip_rjmp(*this, i); break;
        case INSTR_MERGED_ADD_ADC2: cycles = instr_merged_add_adc2(*this, i); break;
        case INSTR_MERGED_DELAY_CALL:
        {
            auto stack = sp();
            if(budget - static_cast<int32_t>(elapsed) < i.m1 || stack <= 256 || stack >= data.size())
                goto timed;
            const uint16_t return_pc = pc + i.m0;
            // CALL leaves both return bytes in SRAM after RET, with SP restored.
            data[stack] = uint8_t(return_pc);
            data[stack - 1] = uint8_t(return_pc >> 8);
            pushed_at_least_once = true;
            if(i.m2) instr_lpm(*this, {0, 0, 0, INSTR_LPM, 0, 0, 0});
            pc = return_pc;
            cycles = i.m1;
            break;
        }
        case INSTR_MERGED_OLED_SEND:
        case INSTR_MERGED_FX_BLEND:
        case INSTR_MERGED_FX_MASKED_BLEND:
        case INSTR_MERGED_UDIVMOD4:
        case INSTR_MERGED_UDIVMOD2:
            cycle_count = start + elapsed;
            if(i.func == INSTR_MERGED_OLED_SEND)
                cycles = execute_oled_send(*this, budget - static_cast<int32_t>(elapsed));
            else if(i.func == INSTR_MERGED_FX_BLEND)
                cycles = execute_fx_blend(*this, budget - static_cast<int32_t>(elapsed), i.m1 != 0);
            else if(i.func == INSTR_MERGED_FX_MASKED_BLEND)
                cycles = execute_fx_masked_blend(*this, budget - static_cast<int32_t>(elapsed));
            else if(i.func == INSTR_MERGED_UDIVMOD4)
                cycles = execute_udivmod4(*this, budget - static_cast<int32_t>(elapsed));
            else
                cycles = execute_udivmod2(*this, budget - static_cast<int32_t>(elapsed));
            if(!cycles) goto timed;
            break;
        default:
        timed:
        {
#ifdef ESP_PLATFORM
            // SPM can re-decode the program in a general handler.
            if(i.func == INSTR_SPM) cached_pc = UINT16_MAX;
#endif
            cycle_count = start + elapsed;
            const uint8_t old_i = data[reg::addr::SREG] & reg::bit::SREG::I;
            cycles = INSTR_MAP[i.func](*this, i);
            const uint8_t new_i = data[reg::addr::SREG] & reg::bit::SREG::I;
            if(new_i != old_i)
            {
                if(new_i)
                {
                    if(has_pending_interrupt())
                    {
                        irq_delay = false;
                        force_exit = true;
                    }
                    else
                        irq_delay = true;
                    just_enabled_irq = true;
                }
                else
                {
                    prev_sreg &= ~reg::bit::SREG::I;
                    irq_delay = false;
                }
            }
            if(!active || io_reg_accessed)
                force_exit = true;
            break;
        }
        }
        if(__builtin_expect(clear_irq_delay && !just_enabled_irq, 0))
        {
            if(data[reg::addr::SREG] & reg::bit::SREG::I)
                prev_sreg |= reg::bit::SREG::I;
            else
                prev_sreg &= ~reg::bit::SREG::I;
            irq_delay = false;
        }
        elapsed += cycles;
        if(__builtin_expect(force_exit, 0))
            break;
        if(jit_block_off && !irq_delay && jit_block_off[pc & 0x3fffu] != 0)
            break;
    } while(static_cast<int32_t>(elapsed) < budget || (jit_block_off && irq_delay));

    cycle_count = start + elapsed;
    return elapsed;
}
#endif

static jit::JitRuntime s_jit_rt;

ARDENS_NOINLINE
static uint32_t jit_helper_out_spdr_slow(atmega32u4_t* cpu, uint32_t val, uint64_t now)
{
    cpu->cycle_count = now;
    atmega32u4_t::spi_handle_st_spdr(*cpu, reg::addr::SPDR, static_cast<uint8_t>(val));
    return 0;
}

ARDENS_IRAM
static uint32_t jit_helper_out_spdr(atmega32u4_t* cpu, uint32_t val, int32_t rem)
{
    const uint64_t now = cpu->jit_batch_end - rem;
    cpu->cycle_count = now;
#ifdef ARDUGO_EMBEDDED
    const uint8_t spcr = cpu->data[reg::addr::SPCR];
    constexpr uint8_t SPE = reg::bit::SPCR::SPE;
    constexpr uint8_t MSTR = reg::bit::SPCR::MSTR;
    constexpr uint8_t SPIE = reg::bit::SPCR::SPIE;
    constexpr uint8_t DORD = reg::bit::SPCR::DORD;
    if(cpu->spi_bus && cpu->active &&
       (spcr & (SPE | MSTR | SPIE | DORD)) == (SPE | MSTR) &&
       cpu->spi_clock_cycles >= 2) [[likely]]
    {
        if(cpu->spi_busy || now >= cpu->spi_flag_cycle)
            service_active_master_spi_inline(*cpu, now);
        cpu->acknowledge_spi_flags();
        if(!cpu->spi_busy && cpu->spi_transmit_zero_cycle != now) [[likely]]
        {
            cpu->spi_data_byte = static_cast<uint8_t>(val);
            cpu->spi_fast_transfer = (cpu->spi_clock_cycles == 2);
            cpu->spi_transmit_zero_cycle = now + cpu->spi_clock_cycles * 8 + 1;
            cpu->spi_done_cycle = cpu->spi_transmit_zero_cycle - (cpu->spi_fast_transfer ? 1 : 0);
            cpu->spi_data_latched = false;
            cpu->spi_busy = true;
            return 0;
        }
    }
#endif
    return jit_helper_out_spdr_slow(cpu, val, now);
}

ARDENS_NOINLINE ARDENS_IRAM
static uint32_t jit_helper_in_spdr_slow(atmega32u4_t* cpu, uint64_t now)
{
    cpu->cycle_count = now;
    return atmega32u4_t::spi_handle_ld_spdr(*cpu, reg::addr::SPDR);
}

ARDENS_IRAM
static uint32_t jit_helper_in_spdr(atmega32u4_t* cpu, int32_t rem)
{
    const uint64_t now = cpu->jit_batch_end - rem;
#ifdef ARDUGO_EMBEDDED
    if(!(cpu->spi_bus && cpu->active && !(cpu->data[reg::addr::SPCR] & reg::bit::SPCR::SPIE))) [[unlikely]]
        return jit_helper_in_spdr_slow(cpu, now);
    if(now >= cpu->spi_flag_cycle)
    {
        cpu->data[reg::addr::SPSR] |= reg::bit::SPSR::SPIF;
        cpu->spi_flag_cycle = UINT64_MAX;
    }
    if(cpu->spi_busy)
    {
        const uint64_t receive_cycle = cpu->spi_transmit_zero_cycle - (cpu->spi_fast_transfer ? 1u : 0u);
        if(!cpu->spi_data_latched && now >= receive_cycle)
        {
            cpu->cycle_count = now;
            cpu->spi_datain_byte = core_exchange_spi_byte_fast(*cpu, cpu->spi_data_byte, receive_cycle);
            cpu->data[reg::addr::SPDR] = cpu->spi_datain_byte;
            cpu->spi_data_latched = true;
        }
        if(now >= cpu->spi_transmit_zero_cycle)
        {
            cpu->spi_busy = false;
            cpu->spi_flag_cycle = cpu->spi_transmit_zero_cycle + (cpu->spi_fast_transfer ? 1u : 0u);
            if(now >= cpu->spi_flag_cycle)
            {
                cpu->data[reg::addr::SPSR] |= reg::bit::SPSR::SPIF;
                cpu->spi_flag_cycle = UINT64_MAX;
            }
        }
    }
    cpu->acknowledge_spi_flags();
    return cpu->data[reg::addr::SPDR];
#else
    if(cpu->spi_flag_cycle <= now ||
       (cpu->spi_busy && cpu->spi_transmit_zero_cycle -
           (cpu->spi_fast_transfer && !cpu->spi_data_latched) <= now))
        return jit_helper_in_spdr_slow(cpu, now);
    cpu->acknowledge_spi_flags();
    return cpu->data[reg::addr::SPDR];
#endif
}

#ifdef ARDUGO_EMBEDDED
ARDENS_IRAM
static uint32_t jit_helper_in_out_spdr(atmega32u4_t* cpu, uint32_t out_val, int32_t rem)
{
    const uint64_t now_in = cpu->jit_batch_end - rem;
    const uint8_t spcr = cpu->data[reg::addr::SPCR];
    constexpr uint8_t SPE = reg::bit::SPCR::SPE;
    constexpr uint8_t MSTR = reg::bit::SPCR::MSTR;
    constexpr uint8_t SPIE = reg::bit::SPCR::SPIE;
    constexpr uint8_t DORD = reg::bit::SPCR::DORD;
    if(!(cpu->spi_bus && cpu->active &&
         (spcr & (SPE | MSTR | SPIE | DORD)) == (SPE | MSTR) &&
         cpu->spi_clock_cycles >= 2)) [[unlikely]]
    {
        const uint32_t received = jit_helper_in_spdr(cpu, rem);
        jit_helper_out_spdr(cpu, out_val, rem - 1);
        return received;
    }

    if(now_in >= cpu->spi_flag_cycle)
    {
        cpu->data[reg::addr::SPSR] |= reg::bit::SPSR::SPIF;
        cpu->spi_flag_cycle = UINT64_MAX;
    }
    if(cpu->spi_busy)
    {
        const uint64_t receive_cycle = cpu->spi_transmit_zero_cycle - (cpu->spi_fast_transfer ? 1u : 0u);
        if(!cpu->spi_data_latched && now_in >= receive_cycle)
        {
            cpu->cycle_count = now_in;
            cpu->spi_datain_byte = core_exchange_spi_byte_fast(*cpu, cpu->spi_data_byte, receive_cycle);
            cpu->data[reg::addr::SPDR] = cpu->spi_datain_byte;
            cpu->spi_data_latched = true;
        }
        if(now_in >= cpu->spi_transmit_zero_cycle)
        {
            cpu->spi_busy = false;
            cpu->spi_flag_cycle = cpu->spi_transmit_zero_cycle + (cpu->spi_fast_transfer ? 1u : 0u);
            if(now_in >= cpu->spi_flag_cycle)
            {
                cpu->data[reg::addr::SPSR] |= reg::bit::SPSR::SPIF;
                cpu->spi_flag_cycle = UINT64_MAX;
            }
        }
    }
    cpu->acknowledge_spi_flags();
    const uint32_t received = cpu->data[reg::addr::SPDR];

    const uint64_t now_out = now_in + 1;
    cpu->cycle_count = now_out;
    if(cpu->spi_busy || now_out >= cpu->spi_flag_cycle) [[unlikely]]
        service_active_master_spi_inline(*cpu, now_out);
    if(!cpu->spi_busy && cpu->spi_transmit_zero_cycle != now_out) [[likely]]
    {
        cpu->spi_data_byte = static_cast<uint8_t>(out_val);
        cpu->spi_fast_transfer = (cpu->spi_clock_cycles == 2);
        cpu->spi_transmit_zero_cycle = now_out + cpu->spi_clock_cycles * 8 + 1;
        cpu->spi_done_cycle = cpu->spi_transmit_zero_cycle - (cpu->spi_fast_transfer ? 1 : 0);
        cpu->spi_data_latched = false;
        cpu->spi_busy = true;
        return received;
    }
    jit_helper_out_spdr_slow(cpu, out_val, now_out);
    return received;
}
#endif

ARDENS_IRAM
static uint32_t jit_helper_in_spsr(atmega32u4_t* cpu, int32_t rem)
{
    const uint64_t now = cpu->jit_batch_end - rem;
    if(cpu->spi_flag_cycle <= now ||
       (cpu->spi_busy && cpu->spi_transmit_zero_cycle -
           (cpu->spi_fast_transfer && !cpu->spi_data_latched) <= now))
    {
        cpu->cycle_count = now;
        return atmega32u4_t::spi_handle_ld_spsr(*cpu, reg::addr::SPSR);
    }
    return cpu->read_spi_status();
}

ARDENS_NOINLINE ARDENS_IRAM
static uint32_t jit_helper_out_port_slow(atmega32u4_t* cpu, uint32_t packed, uint64_t now)
{
    const uint16_t addr = static_cast<uint16_t>((packed >> 8) & 0xffffu);
    const uint8_t val = static_cast<uint8_t>(packed & 0xffu);
    cpu->cycle_count = now;
    if(packed >> 24)
        atmega32u4_t::st_handle_pin(*cpu, addr, val);
    else
        atmega32u4_t::st_handle_port(*cpu, addr, val);
    return 0;
}

ARDENS_IRAM
static uint32_t jit_helper_out_port(atmega32u4_t* cpu, uint32_t packed, int32_t rem)
{
    const uint64_t now = cpu->jit_batch_end - rem;
#ifdef ARDUGO_EMBEDDED
    const uint16_t addr = static_cast<uint16_t>((packed >> 8) & 0xffffu);
    const uint8_t val = static_cast<uint8_t>(packed & 0xffu);
    const bool is_pin = (packed >> 24) != 0;
    const uint16_t port_addr = is_pin ? uint16_t(addr + 2u) : addr;
    if(cpu->spi_bus && cpu->active &&
       !(cpu->data[reg::addr::SPCR] & reg::bit::SPCR::SPIE) &&
       !cpu->twi_adapter) [[likely]]
    {
        auto* bus = cpu->spi_bus;
        auto& state = bus->core_state;
        const bool spi_pins = (port_addr == reg::addr::PORTD || port_addr == state.fx_port);
        if(spi_pins && (cpu->spi_busy || now >= cpu->spi_flag_cycle))
            service_active_master_spi_inline(*cpu, now);
        if(port_addr == reg::addr::PORTC)
        {
            cpu->cycle_count = now;
            cpu->update_sound();
        }
        const uint8_t next_val = is_pin ? uint8_t(cpu->data[port_addr] ^ val) : val;
        cpu->data[port_addr] = next_val;
        if(port_addr == reg::addr::PORTC)
            cpu->data[reg::addr::PINC] = next_val;
        if(spi_pins)
        {
            const bool selected = (cpu->data[state.fx_port] & state.fx_mask) == 0;
            const bool in_reset = !(cpu->data[reg::addr::PORTD] & reg::bit::PORTD::PORTD7);
            if(selected != state.fx_selected || in_reset != state.oled_in_reset)
            {
                cpu->cycle_count = now;
                bus->sync_spi_pins(now);
            }
        }
        return 0;
    }
#endif
    return jit_helper_out_port_slow(cpu, packed, now);
}

ARDENS_IRAM
static uint32_t jit_helper_in_tcnt0(atmega32u4_t* cpu, int32_t rem)
{
    uint8_t val = 0;
    if(!cpu->peek_timer0(cpu->jit_batch_end - rem, val))
        return 0x100u;
    return val;
}

ARDENS_IRAM
static uint32_t jit_helper_has_pending_irq(atmega32u4_t* cpu)
{
    return cpu->has_pending_interrupt() ? 1u : 0u;
}

ARDENS_NOINLINE
static uint32_t jit_helper_macro(atmega32u4_t* cpu, uint32_t pc, int32_t rem)
{
    const uint64_t batch_start = cpu->cycle_count;
    const uint64_t now = cpu->jit_batch_end - rem;
    cpu->pc = static_cast<uint16_t>(pc);
    cpu->cycle_count = now;
#ifdef ESP_PLATFORM
    const auto& i = (&cpu->merged_instruction(static_cast<uint16_t>(pc) & ~(atmega32u4_t::INSTR_CACHE_BLOCK - 1)))[pc % atmega32u4_t::INSTR_CACHE_BLOCK];
#else
    const auto i = cpu->merged_prog[static_cast<uint16_t>(pc)];
#endif
    uint32_t cycles = 0;
    bool force_exit = false;
    // Keep dispatch branch-based inside IRAM so -fjump-tables never emits a flash table here.
    if(i.func == INSTR_MERGED_OLED_SEND)
    {
        cycles = execute_oled_send(*cpu, rem);
        if(!cycles) goto timed_fallback;
    }
    else if(i.func == INSTR_MERGED_FX_BLEND)
    {
        cycles = execute_fx_blend(*cpu, rem, i.m1 != 0);
        if(!cycles) goto timed_fallback;
    }
    else if(i.func == INSTR_MERGED_FX_MASKED_BLEND)
    {
        cycles = execute_fx_masked_blend(*cpu, rem);
        if(!cycles) goto timed_fallback;
    }
    else if(i.func == INSTR_MERGED_UDIVMOD4 || i.func == INSTR_MERGED_UDIVMOD2)
    {
        uint32_t needed = 0;
        cycles = (i.func == INSTR_MERGED_UDIVMOD4)
            ? execute_udivmod4(*cpu, rem, &needed)
            : execute_udivmod2(*cpu, rem, &needed);
        if(!cycles)
        {
            const uint64_t next_event = cpu->peripheral_queue.next_cycle();
            if(needed != 0 && now > batch_start && next_event > now && (next_event - now) >= needed)
                return 0x80000000u | uint16_t(rem);
            goto timed_fallback;
        }
    }
    else if(i.func == INSTR_MERGED_MICROS_CALL)
    {
        cycles = execute_micros_call(*cpu, i, rem);
        if(!cycles) goto timed_fallback;
    }
    else if(i.func == INSTR_MERGED_MICROS_RESULT)
    {
        if(rem >= 23)
        {
#ifdef ARDUGO_QUIET_TEST
            ++cpu->benchmark.micros_blocks;
#endif
            cycles = instr_merged_micros_result(*cpu, i);
        }
        else
        {
            if(rem < 1) return 0x80000000u | uint16_t(rem);
            cycles = instr_mov(*cpu, i);
        }
    }
    else if(i.func == INSTR_MERGED_MICROS_TIMER)
    {
        if(rem >= 20)
        {
            cycles = instr_merged_micros_timer(*cpu, i);
            if(cycles)
            {
#ifdef ARDUGO_QUIET_TEST
                ++cpu->benchmark.micros_timer_blocks;
#endif
                if(cpu->data[reg::addr::SREG] & reg::bit::SREG::I)
                {
                    if(cpu->has_pending_interrupt())
                        force_exit = true;
                    else
                        cpu->jit_irq_enable_rem = rem - static_cast<int32_t>(cycles);
                }
            }
        }
        if(!cycles)
        {
            if(rem < 1) return 0x80000000u | uint16_t(rem);
            cycles = instr_merged_in(*cpu, i);
        }
    }
    else if(i.func == INSTR_MERGED_SPI_POLL)
    {
        cycles = execute_spi_poll(*cpu, i, rem);
        if(!cycles)
        {
            if(rem < 1) return 0x80000000u | uint16_t(rem);
#ifdef ARDUGO_EMBEDDED
            if(cpu->spi_bus)
            {
                const uint16_t addr = i.src + 32;
                if(addr == reg::addr::SPDR || addr == reg::addr::SPSR)
                {
                    if(cpu->spi_flag_cycle <= cpu->cycle_count ||
                       (cpu->spi_busy && cpu->spi_transmit_zero_cycle -
                           (cpu->spi_fast_transfer && !cpu->spi_data_latched) <= cpu->cycle_count))
                    {
                        cpu->gpr(i.dst) = (addr == reg::addr::SPDR)
                            ? atmega32u4_t::spi_handle_ld_spdr(*cpu, reg::addr::SPDR)
                            : atmega32u4_t::spi_handle_ld_spsr(*cpu, reg::addr::SPSR);
                    }
                    else if(addr == reg::addr::SPDR)
                    {
                        cpu->acknowledge_spi_flags();
                        cpu->gpr(i.dst) = cpu->data[reg::addr::SPDR];
                    }
                    else
                        cpu->gpr(i.dst) = cpu->read_spi_status();
                    ++cpu->pc;
                    cycles = 1;
                }
            }
#endif
            if(!cycles)
            {
                cycles = instr_merged_in(*cpu, i);
                force_exit = true;
            }
        }
    }
    else if(i.func == INSTR_MERGED_DELAY_CALL)
    {
        const auto stack = cpu->sp();
        if(rem < i.m1 || stack <= 256 || stack >= cpu->data.size())
            goto timed_fallback;
        const uint16_t return_pc = static_cast<uint16_t>(cpu->pc + i.m0);
        cpu->data[stack] = uint8_t(return_pc);
        cpu->data[stack - 1] = uint8_t(return_pc >> 8);
        cpu->pushed_at_least_once = true;
        if(i.m2) instr_lpm(*cpu, {0, 0, 0, INSTR_LPM, 0, 0, 0});
        cpu->pc = return_pc;
        cycles = i.m1;
    }
    else
    {
    timed_fallback:
        if(rem < atmega32u4_t::MAX_INSTR_CYCLES)
            return 0x80000000u | uint16_t(rem);
        cycles = INSTR_MAP[i.func](*cpu, i);
        if(!cpu->active || cpu->io_reg_accessed)
            force_exit = true;
    }
    const int32_t new_rem = rem - static_cast<int32_t>(cycles);
    if(new_rem <= 0) force_exit = true;
    return (force_exit ? 0x80000000u : 0u) | uint16_t(new_rem);
}

ARDENS_IRAM
static uint32_t jit_helper_oled_send(atmega32u4_t* cpu, uint32_t pc, int32_t rem)
{
    cpu->pc = static_cast<uint16_t>(pc);
    cpu->cycle_count = cpu->jit_batch_end - rem;
    bool force_exit = false;
    uint32_t cycles = execute_oled_send(*cpu, rem);
    if(!cycles)
    {
        if(rem < atmega32u4_t::MAX_INSTR_CYCLES)
            return 0x80000000u | uint16_t(rem);
        cycles = instr_merged_ldd_z(*cpu, {0, 0, 0, INSTR_MERGED_LDD_Z, 0, 0, 0});
        if(!cpu->active || cpu->io_reg_accessed)
            force_exit = true;
    }
    const int32_t new_rem = rem - static_cast<int32_t>(cycles);
    if(new_rem <= 0) force_exit = true;
    return (force_exit ? 0x80000000u : 0u) | uint16_t(new_rem);
}

ARDENS_IRAM
static uint32_t jit_helper_fx_blend(atmega32u4_t* cpu, uint32_t pc_alt, int32_t rem)
{
    const bool alt = (pc_alt >> 14) != 0;
    cpu->pc = static_cast<uint16_t>(pc_alt & 0x3fffu);
    cpu->cycle_count = cpu->jit_batch_end - rem;
    bool force_exit = false;
    uint32_t cycles = execute_fx_blend(*cpu, rem, alt);
    if(!cycles)
    {
        if(rem < atmega32u4_t::MAX_INSTR_CYCLES)
            return 0x80000000u | uint16_t(rem);
        cycles = instr_merged_in(*cpu, {0, uint8_t(reg::addr::SPDR - 32), uint8_t(alt ? 16 : 24), INSTR_MERGED_IN, 0, 0, 0});
        if(!cpu->active || cpu->io_reg_accessed)
            force_exit = true;
    }
    const int32_t new_rem = rem - static_cast<int32_t>(cycles);
    if(new_rem <= 0) force_exit = true;
    return (force_exit ? 0x80000000u : 0u) | uint16_t(new_rem);
}

ARDENS_IRAM
static uint32_t jit_helper_fx_masked_blend(atmega32u4_t* cpu, uint32_t pc, int32_t rem)
{
    cpu->pc = static_cast<uint16_t>(pc);
    cpu->cycle_count = cpu->jit_batch_end - rem;
    bool force_exit = false;
    uint32_t cycles = execute_fx_masked_blend(*cpu, rem);
    if(!cycles)
    {
        if(rem < atmega32u4_t::MAX_INSTR_CYCLES)
            return 0x80000000u | uint16_t(rem);
        cycles = instr_merged_in(*cpu, {0, uint8_t(reg::addr::SPDR - 32), 0, INSTR_MERGED_IN, 0, 0, 0});
        if(!cpu->active || cpu->io_reg_accessed)
            force_exit = true;
    }
    const int32_t new_rem = rem - static_cast<int32_t>(cycles);
    if(new_rem <= 0) force_exit = true;
    return (force_exit ? 0x80000000u : 0u) | uint16_t(new_rem);
}

ARDENS_IRAM
static uint32_t jit_helper_micros_call(atmega32u4_t* cpu, uint32_t pc_tgt_mode, uint32_t args, int32_t rem)
{
    const uint16_t pc = static_cast<uint16_t>(pc_tgt_mode & 0x3fffu);
    const uint16_t tgt = static_cast<uint16_t>((pc_tgt_mode >> 14) & 0x3fffu);
    const uint8_t m2 = static_cast<uint8_t>((pc_tgt_mode >> 28) & 0x7u);
    const avr_instr_t i{
        tgt,
        static_cast<uint8_t>((args >> 16) & 0xffu),
        static_cast<uint8_t>((args >> 24) & 0xffu),
        INSTR_MERGED_MICROS_CALL,
        static_cast<uint8_t>(args & 0xffu),
        static_cast<uint8_t>((args >> 8) & 0xffu),
        m2
    };
    cpu->pc = pc;
    cpu->cycle_count = cpu->jit_batch_end - rem;
    bool force_exit = false;
    uint32_t cycles = execute_micros_call(*cpu, i, rem);
    if(!cycles)
    {
        if(rem < atmega32u4_t::MAX_INSTR_CYCLES)
            return 0x80000000u | uint16_t(rem);
        cycles = instr_call(*cpu, i);
        if(!cpu->active || cpu->io_reg_accessed)
            force_exit = true;
    }
    const int32_t new_rem = rem - static_cast<int32_t>(cycles);
    if(new_rem <= 0) force_exit = true;
    return (force_exit ? 0x80000000u : 0u) | uint16_t(new_rem);
}

ARDENS_IRAM
static uint32_t jit_helper_micros_result(atmega32u4_t* cpu, uint32_t pc_compact, int32_t rem)
{
    cpu->pc = static_cast<uint16_t>(pc_compact & 0x3fffu);
    cpu->cycle_count = cpu->jit_batch_end - rem;
    const uint16_t variant = static_cast<uint16_t>((pc_compact >> 16) & 3u);
    const bool compact = (variant == 1);
    uint32_t cycles = 0;
    bool force_exit = false;
    if(rem >= 23)
    {
#ifdef ARDUGO_QUIET_TEST
        ++cpu->benchmark.micros_blocks;
#endif
        cycles = instr_merged_micros_result(*cpu, {variant});
    }
    else
    {
        if(rem < 1) return 0x80000000u | uint16_t(rem);
        cycles = instr_mov(*cpu, {0, uint8_t(compact ? 24 : 25), uint8_t(compact ? 23 : 26), INSTR_MOV, 0, 0, 0});
    }
    const int32_t new_rem = rem - static_cast<int32_t>(cycles);
    if(new_rem <= 0) force_exit = true;
    return (force_exit ? 0x80000000u : 0u) | uint16_t(new_rem);
}

ARDENS_IRAM
static uint32_t jit_helper_micros_timer(atmega32u4_t* cpu, uint32_t pc_counter, int32_t rem)
{
    cpu->pc = static_cast<uint16_t>(pc_counter & 0x3fffu);
    cpu->cycle_count = cpu->jit_batch_end - rem;
    const uint16_t counter = static_cast<uint16_t>(pc_counter >> 16);
    uint32_t cycles = 0;
    bool force_exit = false;
    if(rem >= 20)
    {
        cycles = instr_merged_micros_timer(*cpu, {counter});
        if(cycles)
        {
#ifdef ARDUGO_QUIET_TEST
            ++cpu->benchmark.micros_timer_blocks;
#endif
            if(cpu->data[reg::addr::SREG] & reg::bit::SREG::I)
            {
                if(cpu->has_pending_interrupt())
                    force_exit = true;
                else
                    cpu->jit_irq_enable_rem = rem - static_cast<int32_t>(cycles);
            }
        }
    }
    if(!cycles)
    {
        if(rem < 1) return 0x80000000u | uint16_t(rem);
        cycles = instr_merged_in(*cpu, {0, 0x3f, 19, INSTR_MERGED_IN, 0, 0, 0});
    }
    const int32_t new_rem = rem - static_cast<int32_t>(cycles);
    if(new_rem <= 0) force_exit = true;
    return (force_exit ? 0x80000000u : 0u) | uint16_t(new_rem);
}

ARDENS_IRAM
static uint32_t jit_helper_spi_poll(atmega32u4_t* cpu, uint32_t pc_args, int32_t rem)
{
    const uint16_t pc = static_cast<uint16_t>(pc_args & 0x3fffu);
    const uint8_t dst = static_cast<uint8_t>((pc_args >> 14) & 31u);
    const uint8_t m0 = static_cast<uint8_t>((pc_args >> 19) & 31u);
    const bool read_data = ((pc_args >> 24) & 1u) != 0;
    const uint8_t src_io = static_cast<uint8_t>((read_data ? reg::addr::SPDR : reg::addr::SPSR) - 32u);
    const avr_instr_t i{0, src_io, dst, INSTR_MERGED_SPI_POLL, m0, 0, 0};
    cpu->pc = pc;
    cpu->cycle_count = cpu->jit_batch_end - rem;
    bool force_exit = false;
    uint32_t cycles = execute_spi_poll(*cpu, i, rem);
    if(!cycles)
    {
        if(rem < 1) return 0x80000000u | uint16_t(rem);
#ifdef ARDUGO_EMBEDDED
        if(cpu->spi_bus)
        {
            const uint16_t addr = src_io + 32;
            if(cpu->spi_flag_cycle <= cpu->cycle_count ||
               (cpu->spi_busy && cpu->spi_transmit_zero_cycle -
                   (cpu->spi_fast_transfer && !cpu->spi_data_latched) <= cpu->cycle_count))
            {
                cpu->gpr(dst) = read_data
                    ? atmega32u4_t::spi_handle_ld_spdr(*cpu, reg::addr::SPDR)
                    : atmega32u4_t::spi_handle_ld_spsr(*cpu, reg::addr::SPSR);
            }
            else if(addr == reg::addr::SPDR)
            {
                cpu->acknowledge_spi_flags();
                cpu->gpr(dst) = cpu->data[reg::addr::SPDR];
            }
            else
                cpu->gpr(dst) = cpu->read_spi_status();
            ++cpu->pc;
            cycles = 1;
        }
#endif
        if(!cycles)
        {
            cycles = instr_merged_in(*cpu, i);
            force_exit = true;
        }
    }
    const int32_t new_rem = rem - static_cast<int32_t>(cycles);
    if(new_rem <= 0) force_exit = true;
    return (force_exit ? 0x80000000u : 0u) | uint16_t(new_rem);
}

ARDENS_IRAM
static uint32_t jit_helper_udivmod(atmega32u4_t* cpu, uint32_t pc_is4, int32_t rem)
{
    const uint64_t batch_start = cpu->cycle_count;
    const uint64_t now = cpu->jit_batch_end - rem;
    const uint16_t pc = static_cast<uint16_t>(pc_is4 & 0x3fffu);
    const bool is4 = (pc_is4 >> 14) != 0;
    cpu->pc = pc;
    cpu->cycle_count = now;
    uint32_t needed = 0;
    uint32_t cycles = is4
        ? execute_udivmod4(*cpu, rem, &needed)
        : execute_udivmod2(*cpu, rem, &needed);
    bool force_exit = false;
    if(!cycles)
    {
        const uint64_t next_event = cpu->peripheral_queue.next_cycle();
        if(needed != 0 && now > batch_start && next_event > now && (next_event - now) >= needed)
            return 0x80000000u | uint16_t(rem);
        if(rem < 1)
            return 0x80000000u | uint16_t(rem);
        if(is4)
        {
            cpu->data[26] = 33;
            cpu->pc = uint16_t(pc + 1u);
            cycles = 1;
        }
        else
        {
            cycles = instr_sub(*cpu, {0, 26, 26, INSTR_SUB, 0, 0, 0});
        }
    }
    const int32_t new_rem = rem - static_cast<int32_t>(cycles);
    if(new_rem <= 0) force_exit = true;
    return (force_exit ? 0x80000000u : 0u) | uint16_t(new_rem);
}

ARDENS_IRAM
static uint32_t jit_helper_timed_insn(atmega32u4_t* cpu, uint32_t pc, int32_t rem)
{
    cpu->pc = static_cast<uint16_t>(pc);
    cpu->cycle_count = cpu->jit_batch_end - rem;
    const uint8_t old_i = cpu->data[reg::addr::SREG] & reg::bit::SREG::I;
#if defined(ARDUGO_EMBEDDED) && defined(ESP_PLATFORM)
    const auto& i = cpu->decoded_cache.get(static_cast<uint16_t>(pc), cpu->decoded_prog.data());
#else
    const auto& i = cpu->decoded_prog[static_cast<uint16_t>(pc)];
#endif
    const uint32_t cycles = INSTR_MAP[i.func](*cpu, i);
    const int32_t new_rem = rem - static_cast<int32_t>(cycles);
    bool force_exit = (!cpu->active || cpu->io_reg_accessed || new_rem <= 0);
    const uint8_t new_i = cpu->data[reg::addr::SREG] & reg::bit::SREG::I;
    if(new_i != old_i)
    {
        if(new_i)
        {
            if(cpu->has_pending_interrupt())
            {
                cpu->jit_irq_enable_rem = -1;
                force_exit = true;
            }
            else
                cpu->jit_irq_enable_rem = new_rem;
        }
        else
        {
            cpu->prev_sreg &= ~reg::bit::SREG::I;
            cpu->jit_irq_enable_rem = -1;
        }
    }
    if(i.func == INSTR_SPM)
    {
        cpu->jit_spm_active = (cpu->spm_en_cycles != 0) ? 1 : 0;
        force_exit = true;
    }
    return (force_exit ? 0x80000000u : 0u) | uint16_t(new_rem);
}

void atmega32u4_t::jit_invalidate()
{
    jit::jit_invalidate_runtime(s_jit_rt, *this);
}

#if defined(ARDUGO_QUIET_TEST) && defined(ESP_PLATFORM)
static inline uint32_t read_ccount()
{
    uint32_t c;
    __asm__ __volatile__("rsr %0, ccount" : "=a"(c));
    return c;
}
template <size_t Idx, typename Fn, typename... Args>
ARDENS_IRAM
static uint32_t jit_helper_timed_wrap(Fn fn, atmega32u4_t* cpu, Args... args)
{
    const uint32_t t0 = read_ccount();
    const uint32_t res = fn(cpu, args...);
    const uint32_t dt = read_ccount() - t0;
    cpu->benchmark.helper_cycles[Idx] += dt;
    ++cpu->benchmark.helper_calls[Idx];
    return res;
}
ARDENS_IRAM static uint32_t wrap_h0(atmega32u4_t* c, uint32_t v, int32_t r) { return jit_helper_timed_wrap<0>(jit_helper_out_spdr, c, v, r); }
ARDENS_IRAM static uint32_t wrap_h1(atmega32u4_t* c, int32_t r) { return jit_helper_timed_wrap<1>(jit_helper_in_spdr, c, r); }
ARDENS_IRAM static uint32_t wrap_h2(atmega32u4_t* c, int32_t r) { return jit_helper_timed_wrap<2>(jit_helper_in_spsr, c, r); }
ARDENS_IRAM static uint32_t wrap_h3(atmega32u4_t* c, uint32_t p, int32_t r) { return jit_helper_timed_wrap<3>(jit_helper_out_port, c, p, r); }
ARDENS_IRAM static uint32_t wrap_h4(atmega32u4_t* c, int32_t r) { return jit_helper_timed_wrap<4>(jit_helper_in_tcnt0, c, r); }
ARDENS_IRAM static uint32_t wrap_h5(atmega32u4_t* c) { return jit_helper_timed_wrap<5>(jit_helper_has_pending_irq, c); }
ARDENS_IRAM static uint32_t wrap_h6(atmega32u4_t* c, uint32_t pc, int32_t r) { return jit_helper_timed_wrap<6>(jit_helper_macro, c, pc, r); }
ARDENS_IRAM static uint32_t wrap_h7(atmega32u4_t* c, uint32_t pc, int32_t r)
{
    const uint8_t fn = c->decoded_prog[pc & 0x3fff].func;
    if(fn < 160) ++c->benchmark.timed_func_hits[fn];
    return jit_helper_timed_wrap<7>(jit_helper_timed_insn, c, pc, r);
}
ARDENS_IRAM static uint32_t wrap_h8(atmega32u4_t* c, uint32_t pc, int32_t r) { return jit_helper_timed_wrap<6>(jit_helper_oled_send, c, pc, r); }
ARDENS_IRAM static uint32_t wrap_h9(atmega32u4_t* c, uint32_t pc, int32_t r) { return jit_helper_timed_wrap<6>(jit_helper_fx_blend, c, pc, r); }
ARDENS_IRAM static uint32_t wrap_h10(atmega32u4_t* c, uint32_t pc, int32_t r) { return jit_helper_timed_wrap<6>(jit_helper_fx_masked_blend, c, pc, r); }
ARDENS_IRAM static uint32_t wrap_h11(atmega32u4_t* c, uint32_t a1, uint32_t a2, int32_t r) { return jit_helper_timed_wrap<6>(jit_helper_micros_call, c, a1, a2, r); }
ARDENS_IRAM static uint32_t wrap_h12(atmega32u4_t* c, uint32_t a1, int32_t r) { return jit_helper_timed_wrap<6>(jit_helper_micros_result, c, a1, r); }
ARDENS_IRAM static uint32_t wrap_h13(atmega32u4_t* c, uint32_t a1, int32_t r) { return jit_helper_timed_wrap<6>(jit_helper_micros_timer, c, a1, r); }
ARDENS_IRAM static uint32_t wrap_h14(atmega32u4_t* c, uint32_t a1, int32_t r) { return jit_helper_timed_wrap<6>(jit_helper_spi_poll, c, a1, r); }
ARDENS_IRAM static uint32_t wrap_h15(atmega32u4_t* c, uint32_t o, int32_t r) { return jit_helper_timed_wrap<1>(jit_helper_in_out_spdr, c, o, r); }
ARDENS_IRAM static uint32_t wrap_h16(atmega32u4_t* c, uint32_t a1, int32_t r) { return jit_helper_timed_wrap<6>(jit_helper_udivmod, c, a1, r); }
#endif

ARDENS_IRAM
uint32_t atmega32u4_t::execute_jit(int32_t budget)
{
    if(budget <= 0)
        return 0;

    static const jit::JitOffsets s_jit_off = jit::compute_jit_offsets(*this);
    if(s_jit_rt.owner_cpu != reinterpret_cast<uintptr_t>(this) ||
       s_jit_rt.owner_gen != jit_decode_gen)
    {
        jit::jit_reset_runtime(s_jit_rt, *this, s_jit_off);
    }
    if(s_jit_rt.failed)
        return 0;

    if(!jit_helpers[0])
    {
#if defined(ARDUGO_QUIET_TEST) && defined(ESP_PLATFORM)
        jit_helpers[0] = reinterpret_cast<void*>(&wrap_h0);
        jit_helpers[1] = reinterpret_cast<void*>(&wrap_h1);
        jit_helpers[2] = reinterpret_cast<void*>(&wrap_h2);
        jit_helpers[3] = reinterpret_cast<void*>(&wrap_h3);
        jit_helpers[4] = reinterpret_cast<void*>(&wrap_h4);
        jit_helpers[5] = reinterpret_cast<void*>(&wrap_h5);
        jit_helpers[6] = reinterpret_cast<void*>(&wrap_h6);
        jit_helpers[7] = reinterpret_cast<void*>(&wrap_h7);
        jit_helpers[8] = reinterpret_cast<void*>(&wrap_h8);
        jit_helpers[9] = reinterpret_cast<void*>(&wrap_h9);
        jit_helpers[10] = reinterpret_cast<void*>(&wrap_h10);
        jit_helpers[11] = reinterpret_cast<void*>(&wrap_h11);
        jit_helpers[12] = reinterpret_cast<void*>(&wrap_h12);
        jit_helpers[13] = reinterpret_cast<void*>(&wrap_h13);
        jit_helpers[14] = reinterpret_cast<void*>(&wrap_h14);
        jit_helpers[15] = reinterpret_cast<void*>(&wrap_h15);
        jit_helpers[16] = reinterpret_cast<void*>(&wrap_h16);
#else
        jit_helpers[0] = reinterpret_cast<void*>(&jit_helper_out_spdr);
        jit_helpers[1] = reinterpret_cast<void*>(&jit_helper_in_spdr);
        jit_helpers[2] = reinterpret_cast<void*>(&jit_helper_in_spsr);
        jit_helpers[3] = reinterpret_cast<void*>(&jit_helper_out_port);
        jit_helpers[4] = reinterpret_cast<void*>(&jit_helper_in_tcnt0);
        jit_helpers[5] = reinterpret_cast<void*>(&jit_helper_has_pending_irq);
        jit_helpers[6] = reinterpret_cast<void*>(&jit_helper_macro);
        jit_helpers[7] = reinterpret_cast<void*>(&jit_helper_timed_insn);
        jit_helpers[8] = reinterpret_cast<void*>(&jit_helper_oled_send);
        jit_helpers[9] = reinterpret_cast<void*>(&jit_helper_fx_blend);
        jit_helpers[10] = reinterpret_cast<void*>(&jit_helper_fx_masked_blend);
        jit_helpers[11] = reinterpret_cast<void*>(&jit_helper_micros_call);
        jit_helpers[12] = reinterpret_cast<void*>(&jit_helper_micros_result);
        jit_helpers[13] = reinterpret_cast<void*>(&jit_helper_micros_timer);
        jit_helpers[14] = reinterpret_cast<void*>(&jit_helper_spi_poll);
        jit_helpers[15] = reinterpret_cast<void*>(&jit_helper_in_out_spdr);
        jit_helpers[16] = reinterpret_cast<void*>(&jit_helper_udivmod);
#endif
    }

    const uint64_t start = cycle_count;
    jit_batch_end = start + budget;
    jit_prog_ptr = prog.data();
    jit_spm_active = (spm_en_cycles != 0) ? 1 : 0;
    jit_irq_enable_rem = -1;
    io_reg_accessed = false;

#ifdef ESP_PLATFORM
    int32_t rem = budget;
    auto enter = s_jit_rt.entry_fn();
    while(rem > 0)
    {
        ++s_jit_rt.stat_entries;
        const uint32_t res = enter(
            this, data.data(), rem, data[reg::addr::SREG],
            s_jit_rt.code_base_addr, s_jit_rt.block_off);
        const int32_t rem_out = static_cast<int16_t>(res & 0xffffu);
        rem = rem_out;
        if((res >> 16) == 1)
        {
            ++s_jit_rt.stat_misses;
            const uint32_t resets = s_jit_rt.stat_resets;
            uint8_t threshold = 1;
            if(resets >= 8) threshold = 24;
            else if(resets >= 4) threshold = 12;
            else if(resets >= 2) threshold = 4;
            bool do_compile = (threshold <= 1) || !s_jit_rt.miss_counts;
            if(!do_compile)
            {
                uint8_t& cnt = s_jit_rt.miss_counts[pc & 0x3fffu];
                if(++cnt >= threshold)
                {
                    cnt = 0;
                    do_compile = true;
                }
            }
            if(!do_compile)
            {
                if(jit_irq_enable_rem >= 0 && rem < jit_irq_enable_rem)
                {
                    if(data[reg::addr::SREG] & reg::bit::SREG::I)
                        prev_sreg |= reg::bit::SREG::I;
                    jit_irq_enable_rem = -1;
                }
                if(rem <= MAX_INSTR_CYCLES)
                    break;
                cycle_count = start + static_cast<uint32_t>(budget - rem);
                const int32_t cold_budget = std::min<int32_t>(rem - MAX_INSTR_CYCLES, 48);
                const uint32_t cold_elapsed = execute_merged(cold_budget, s_jit_rt.block_off);
                rem -= static_cast<int32_t>(cold_elapsed);
                if(cold_elapsed == 0 || !active || io_reg_accessed)
                    break;
                continue;
            }
#if defined(ARDUGO_QUIET_TEST) && defined(ESP_PLATFORM)
            const uint32_t tc0 = read_ccount();
            const bool compiled = jit::jit_compile_block(s_jit_rt, *this, s_jit_off, pc);
            benchmark.compile_cycles += read_ccount() - tc0;
#else
            const bool compiled = jit::jit_compile_block(s_jit_rt, *this, s_jit_off, pc);
#endif
            if(!compiled)
            {
                ++s_jit_rt.stat_fallbacks;
                break;
            }
            continue;
        }
        break;
    }
    if(jit_irq_enable_rem >= 0 && rem < jit_irq_enable_rem)
    {
        if(data[reg::addr::SREG] & reg::bit::SREG::I)
            prev_sreg |= reg::bit::SREG::I;
    }
    const uint32_t elapsed = static_cast<uint32_t>(budget - rem);
    cycle_count = start + elapsed;
    return elapsed;
#else
    (void)start;
    return 0;
#endif
}

namespace jit
{
const JitRuntime& jit_runtime()
{
    return s_jit_rt;
}
void jit_reset_stats()
{
    s_jit_rt.stat_compiles = 0;
    s_jit_rt.stat_resets = 0;
    s_jit_rt.stat_entries = 0;
    s_jit_rt.stat_misses = 0;
    s_jit_rt.stat_fallbacks = 0;
}
void jit_preallocate()
{
    jit_preallocate_runtime(s_jit_rt);
}
}

}

