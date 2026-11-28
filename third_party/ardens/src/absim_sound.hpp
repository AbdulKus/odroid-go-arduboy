#include "absim.hpp"

namespace absim
{

void atmega32u4_t::sound_st_handler_ddrc(atmega32u4_t& cpu, uint16_t ptr, uint8_t x)
{
    if(ptr == reg::addr::DDRC)
    {
        // Finish the old sample interval before changing the output pins.
        cpu.update_sound();
        uint32_t e = 0;
        if(x & reg::bit::DDRC::DDC6)
            e |= (1 << 0);
        if(x & reg::bit::DDRC::DDC7)
            e |= (1 << 1);
        cpu.sound_enabled = e;
    }
    cpu.data[ptr] = x;
    if(ptr == reg::addr::DDRC)
        cpu.data[reg::addr::PINC] = cpu.data[reg::addr::PORTC];
}
    
ARDENS_IRAM ARDENS_NOINLINE void atmega32u4_t::update_sound()
{
    uint32_t cycles = uint32_t(cycle_count - sound_prev_cycle);
    sound_prev_cycle = cycle_count;
    if(cycles == 0)
        return;

    const auto pins = sound_enabled;
    int16_t x = 0;
    if(pins != 0)
    {
        x = sound_pwm_val;
        if(!sound_pwm)
        {
            x = 0;
            const uint8_t portc = data[reg::addr::PINC];
            if(pins & (1 << 0))
                x += (portc & reg::bit::PINC::PINC6) ? SOUND_GAIN / 2 : -SOUND_GAIN / 2;
            if(pins & (1 << 1))
                x += (portc & reg::bit::PINC::PINC7) ? -SOUND_GAIN / 2 : SOUND_GAIN / 2;
        }
    }

    const uint32_t c = sound_cycle + cycles;
    if(c < SOUND_CYCLES)
    {
        sound_cycle = c;
        sound_acc += int32_t(cycles) * int32_t(x);
        return;
    }

    // Box-integrate the active transition bucket so >25 kHz PORTC PWM/PDM does not alias.
    const uint32_t take0 = SOUND_CYCLES - sound_cycle;
    const int32_t sum0 = sound_acc + int32_t(take0) * int32_t(x);
    sound_buffer.push_back(int16_t(sum0 / SOUND_CYCLES));

    const uint32_t rem = cycles - take0;
    const uint32_t full = rem / SOUND_CYCLES;
    const uint32_t tail = rem % SOUND_CYCLES;
    for(uint32_t i = 0; i < full; ++i)
        sound_buffer.push_back(x);

    sound_cycle = tail;
    sound_acc = int32_t(tail) * int32_t(x);
}
    
}
