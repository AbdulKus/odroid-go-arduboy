#include "absim.hpp"

#include <cmath>
#include <cstring>

#if defined(ARDENS_SSE2)
#include <emmintrin.h>
#endif

namespace absim
{

void display_t::send_command(uint8_t byte)
{
    if(!processing_command)
    {
        command_byte_index = 0;
        current_command = byte;
        processing_command = true;
    }
    switch(current_command)
    {
    case 0x00: case 0x01: case 0x02: case 0x03:
    case 0x04: case 0x05: case 0x06: case 0x07:
    case 0x08: case 0x09: case 0x0a: case 0x0b:
    case 0x0c: case 0x0d: case 0x0e: case 0x0f:
        data_col &= 0xf0;
        data_col |= current_command;
        processing_command = false;
        break;

    case 0x10: case 0x11: case 0x12: case 0x13:
    case 0x14: case 0x15: case 0x16: case 0x17:
    case 0x18: case 0x19: case 0x1a: case 0x1b:
    case 0x1c: case 0x1d: case 0x1e: case 0x1f:
        data_col &= 0x0f;
        data_col |= current_command << 4;
        if(type != SH1106)
            data_col &= 0x7f;
        processing_command = false;
        break;

    case 0x20:
        if(command_byte_index == 1)
        {
            switch(byte & 0x3)
            {
            case 0: addressing_mode = addr_mode::HORIZONTAL; break;
            case 1: addressing_mode = addr_mode::VERTICAL;   break;
            case 2: addressing_mode = addr_mode::PAGE;       break;
            case 3:                                          break;
            default: break;
            }
            processing_command = false;
        }
        break;

    case 0x21:
        if(command_byte_index == 1)
        {
            col_start = byte & 0x7f;
            data_col = col_start;
        }
        if(command_byte_index == 2)
        {
            col_end = byte & 0x7f;
            processing_command = false;
        }
        break;

    case 0x22:
        if(command_byte_index == 1)
        {
            page_start = byte & 0x7;
            data_page = page_start;
        }
        if(command_byte_index == 2)
        {
            page_end = byte & 0x7;
            processing_command = false;
        }
        break;

    case 0x26:
    case 0x27:
        // TODO: continuous horizontal scroll setup
        if(command_byte_index == 6)
            processing_command = false;
        break;

    case 0x29:
    case 0x2a:
        // TODO: continuous vertical and horizontal scroll setup
        if(command_byte_index == 5)
            processing_command = false;
        break;

    case 0x2e:
        // TODO: deactivate scroll
        processing_command = false;
        break;

    case 0x2f:
        // TODO: activate scroll
        processing_command = false;
        break;

    case 0x30:
    case 0x31:
    case 0x32:
    case 0x33:
        if(type == SH1106)
        {
            // TODO: set pump voltage value
            // 0: VPP = 6.4 V
            // 1: VPP = 7.4 V
            // 2: VPP = 8.0 V (power on)
            // 3: VPP = 9.0 V
        }
        processing_command = false;
        break;

    case 0x81:
        if(command_byte_index == 1)
        {
            contrast = byte;
            processing_command = false;
        }
        break;

    case 0x8d:
        if(type == SSD1309)
        {
            processing_command = false;
            break;
        }
        if(command_byte_index == 1)
        {
            enable_charge_pump = (byte == 0x14);
            processing_command = false;
        }
        break;

    case 0xb0: case 0xb1 : case 0xb2 : case 0xb3:
    case 0xb4: case 0xb5 : case 0xb6 : case 0xb7:
        data_page = byte & 0x7;
        processing_command = false;
        break;

    case 0xa0:
        segment_remap = false;
        processing_command = false;
        break;

    case 0xa1:
        segment_remap = true;
        processing_command = false;
        break;

    case 0xa4:
        entire_display_on = true;
        processing_command = false;
        break;

    case 0xa5:
        entire_display_on = false;
        processing_command = false;
        break;

    case 0xa6:
        inverse_display = false;
        processing_command = false;
        break;

    case 0xa7:
        inverse_display = true;
        processing_command = false;
        break;

    case 0xa8:
        if(command_byte_index == 1)
        {
            mux_ratio = byte & 0x3f;
            processing_command = false;
        }
        break;

    case 0xae:
        display_on = false;
        processing_command = false;
        break;

    case 0xaf:
        display_on = true;
        processing_command = false;
        break;

    case 0xc0:
        com_scan_direction = false;
        processing_command = false;
        break;

    case 0xc8:
        com_scan_direction = true;
        processing_command = false;
        break;

    case 0xd3:
        if(command_byte_index == 1)
        {
            display_offset = byte & 0x3f;
            processing_command = false;
        }
        break;

    case 0xd5:
        if(command_byte_index == 1)
        {
            divide_ratio = byte & 0xf;
            fosc_index = byte >> 4;
            update_clocking();
            processing_command = false;
        }
        break;

    case 0xda:
        if(command_byte_index == 1)
        {
            alternative_com = (byte & 0x10) != 0;
            com_remap = (byte & 0x20) != 0;
            processing_command = false;
        }
        break;

    case 0xd9:
        if(command_byte_index == 1)
        {
            phase_1 = byte & 0xf;
            phase_2 = byte >> 4;
            processing_command = false;
        }
        break;

    case 0xdb:
        if(command_byte_index == 1)
        {
            vcomh_deselect = (byte >> 4) & 0x7;
            processing_command = false;
        }
        break;

    case 0xe3:
        processing_command = false;
        break;

    default:

        if(current_command >= 0x40 && current_command < 0x80)
        {
            display_start = current_command & 0x3f;
            processing_command = false;
            break;
        }

        processing_command = false;
        break;
    }
    ++command_byte_index;
}

void display_t::send_data(uint8_t byte)
{
    display_send_data_common(*this, byte);
}

ARDENS_FORCEINLINE void display_t::update_pixels_row()
{
    uint8_t ram_row = row;
    ram_row += display_start;
    ram_row &= 63;

    uint8_t mask = 1 << (ram_row % 8);
    size_t rindex = (ram_row / 8) * 128;

    uint8_t out_row = row;
    out_row -= display_offset;
    if(com_scan_direction) out_row = mux_ratio - out_row;
    out_row &= 63;

    // the Arduboy's display is upside-down
    size_t pindex = (63 - out_row) * 128 + 128;

    if(!enable_filter)
    {
#if defined(ARDENS_SSE2)
        auto& parray = pixels[1];
        if((mux_ratio >= 16 && row == mux_ratio) || row >= 63)
        {
            pixels[0] = pixels[1];
            vsync = true;
        }
#else
        auto& parray = pixels[0];
        if((mux_ratio >= 16 && row == mux_ratio) || row >= 63)
            vsync = true;
#endif
    }

    constexpr float F = 0.65f;
    constexpr uint32_t FA = uint32_t(F * 256);
    constexpr uint32_t FB = 280 - FA;
    uint8_t tcontrast = (uint8_t)std::min<uint32_t>(255,
        contrast == 0 ? 0 : FA + contrast * FB / 256);
    uint8_t p0 = 0;
    uint8_t p1 = enable_charge_pump ? tcontrast : tcontrast >> 4;

    uint8_t const* const disp_ram = ram_ptr();

    // current limiting
    if(enable_current_limiting)
    {
        // count number of pixels on in the current row
        int num_pixels_on = 0;
        for(int i = 0; i < 128; ++i)
            if(disp_ram[rindex + i] & mask)
                ++num_pixels_on;
        if(inverse_display)
            num_pixels_on = 128 - num_pixels_on;

        float row_drive =
            ref_segment_current * (1.f / 255) * num_pixels_on * tcontrast;

        constexpr float DIFF = MAX_DRIVER_CURRENT * 0.5f;
        if(row != 0 && std::abs(row_drive - prev_row_drive) > DIFF)
        {
            constexpr float F = 0.35f;
            if(row_drive > prev_row_drive)
                row_drive = F * (prev_row_drive + DIFF) + (1.f - F) * row_drive;
            else
                row_drive = F * (prev_row_drive - DIFF) + (1.f - F) * row_drive;
        }

        if(row_drive > MAX_DRIVER_CURRENT)
        {
            float t = MAX_DRIVER_CURRENT / row_drive;
            t += current_limit_slope * (1.f - t);
            row_drive *= t;
        }

        if(num_pixels_on > 0)
        {
            float segment_drive = row_drive / num_pixels_on;
            float t = segment_drive / ref_segment_current * 255;
            if(t > tcontrast)
                t = float(tcontrast);
            p1 = uint8_t(t);
        }

        prev_row_drive = row_drive;
    }

    if(inverse_display) std::swap(p0, p1);

    if(enable_filter)
    {
        const uint8_t y = 63 - out_row;
        const uint8_t s = filter_slot;
        const uint8_t prev = uint8_t((s + 4u) % 5u);
        FilterRow& fr = filter_rows[y];
        uint8_t const* src = disp_ram + rindex;
        auto pack32 = [src, mask](int base) -> uint32_t {
            uint32_t w = 0;
            for(int k = 0; k < 32; ++k)
                if(src[base - k] & mask)
                    w |= (1u << k);
            return w;
        };
        const uint32_t b0 = pack32(127);
        const uint32_t b1 = pack32(95);
        const uint32_t b2 = pack32(63);
        const uint32_t b3 = pack32(31);
        const bool same_prev =
            fr.p0[prev] == p0 && fr.p1[prev] == p1 &&
            fr.bits[prev][0] == b0 && fr.bits[prev][1] == b1 &&
            fr.bits[prev][2] == b2 && fr.bits[prev][3] == b3;
        fr.p0[s] = p0;
        fr.p1[s] = p1;
        fr.bits[s][0] = b0;
        fr.bits[s][1] = b1;
        fr.bits[s][2] = b2;
        fr.bits[s][3] = b3;
        if(!same_prev)
            filtered_steady_mask &= ~(1ULL << y);

        if((mux_ratio >= 16 && row == mux_ratio) || row >= 63)
        {
            if(++filter_slot >= 5u)
                filter_slot = 0;
            filter_dirty = true;
            vsync = true;
        }
        return;
    }

#if defined(ARDENS_SSE2)
    {
        auto& parray = pixels[1];
        uint8_t const* src = disp_ram + rindex;
        uint8_t* dst = parray.data() + pindex;
        __m128i vp0 = _mm_set1_epi8((char)p0);
        __m128i vp1 = _mm_set1_epi8((char)p1);
        __m128i vm = _mm_set1_epi8((char)mask);

        for(int i = 0; i < 8; ++i)
        {
            dst -= 16;

            __m128i vs = _mm_loadu_si128((__m128i const*)src);
            vs = _mm_and_si128(vs, vm);
            vs = _mm_cmpeq_epi8(vs, _mm_setzero_si128());
            __m128i vt0 = _mm_and_si128(vs, vp0);
            __m128i vt1 = _mm_andnot_si128(vs, vp1);
            __m128i vp = _mm_or_si128(vt0, vt1);
            vp = _mm_shuffle_epi32(vp, _MM_SHUFFLE(0, 1, 2, 3));
            vp = _mm_shufflelo_epi16(vp, _MM_SHUFFLE(2, 3, 0, 1));
            vp = _mm_shufflehi_epi16(vp, _MM_SHUFFLE(2, 3, 0, 1));
            vp = _mm_or_si128(_mm_slli_epi16(vp, 8), _mm_srli_epi16(vp, 8));
            _mm_storeu_si128((__m128i*)dst, vp);

            src += 16;
        }
    }
    if(vsync)
        pixels[0] = pixels[1];
#else
    auto& parray = pixels[0];
    if((mux_ratio >= 16 && row == mux_ratio) || row >= 63)
        vsync = true;
    alignas(4) uint8_t rowbuf[128];
    uint8_t const* src = disp_ram + rindex;
    for(int i = 0; i < 128; ++i)
        rowbuf[127 - i] = (src[i] & mask) ? p1 : p0;
    pindex -= 128;
    std::memcpy(&parray[pindex], rowbuf, 128);
#endif
}

ARDENS_IRAM void display_t::reset_filter_state(bool seed_from_pixels0)
{
    filtered_steady_mask = 0;
    filter_slot = 0;
    pixel_history_index = 0;
    filter_dirty = seed_from_pixels0;
    if(!seed_from_pixels0)
    {
        memset(&filter_rows, 0, sizeof(filter_rows));
        return;
    }
    for(int y = 0; y < 64; ++y)
    {
        const uint8_t* src = &pixels[0][y * 128];
        uint8_t lo = 255;
        uint8_t hi = 0;
        for(int x = 0; x < 128; ++x)
        {
            lo = std::min(lo, src[x]);
            hi = std::max(hi, src[x]);
        }
        std::array<uint32_t, 4> words{};
        for(int w = 0; w < 4; ++w)
        {
            uint32_t bits = 0;
            for(int k = 0; k < 32; ++k)
                if(src[w * 32 + k] > lo)
                    bits |= (1u << k);
            words[w] = bits;
        }
        FilterRow& fr = filter_rows[y];
        for(int s = 0; s < 5; ++s)
        {
            fr.p0[s] = lo;
            fr.p1[s] = hi;
            fr.bits[s] = words;
        }
    }
    std::memcpy(filtered_pixels.data(), pixels[0].data(), filtered_pixels.size());
    filtered_steady_mask = ~0ULL;
    filter_dirty = false;
}

#if defined(ESP_PLATFORM) && defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize("Os")
#endif
ARDENS_IRAM ARDENS_NOINLINE void display_t::filter_pixels()
{
    filter_dirty = false;
    const uint8_t cur = filter_slot % 5u;
    const uint8_t s0 = uint8_t((cur + 1u) % 5u);
    const uint8_t s1 = uint8_t((cur + 2u) % 5u);
    const uint8_t s2 = uint8_t((cur + 3u) % 5u);
    const uint8_t s3 = uint8_t((cur + 4u) % 5u);

    static constexpr uint32_t EXPAND4[16] = {
        0x00000000u, 0x00000001u, 0x00000100u, 0x00000101u,
        0x00010000u, 0x00010001u, 0x00010100u, 0x00010101u,
        0x01000000u, 0x01000001u, 0x01000100u, 0x01000101u,
        0x01010000u, 0x01010001u, 0x01010100u, 0x01010101u,
    };

    uint8_t lvl[16]{};
    uint32_t cached_p0_key = 0xffffffffu, cached_p1_key = 0xffffffffu;
    bool have_lvl = false;
    alignas(4) uint32_t row_out[32];

    for(int y = 0; y < 64; ++y)
    {
        if((filtered_steady_mask >> y) & 1ULL)
            continue;

        const FilterRow& fr = filter_rows[y];
        const uint32_t p0_key =
            uint32_t(fr.p0[s0]) | (uint32_t(fr.p0[s1]) << 8) |
            (uint32_t(fr.p0[s2]) << 16) | (uint32_t(fr.p0[s3]) << 24);
        const uint32_t p1_key =
            uint32_t(fr.p1[s0]) | (uint32_t(fr.p1[s1]) << 8) |
            (uint32_t(fr.p1[s2]) << 16) | (uint32_t(fr.p1[s3]) << 24);
        if(!have_lvl || cached_p0_key != p0_key || cached_p1_key != p1_key)
        {
            cached_p0_key = p0_key;
            cached_p1_key = p1_key;
            const uint32_t p0_0 = fr.p0[s0], p0_1 = fr.p0[s1], p0_2 = fr.p0[s2], p0_3 = fr.p0[s3];
            const uint32_t p1_0 = fr.p1[s0], p1_1 = fr.p1[s1], p1_2 = fr.p1[s2], p1_3 = fr.p1[s3];
            for(uint32_t idx = 0; idx < 16; ++idx)
            {
                const uint32_t v0 = (idx & 1u) ? p1_0 : p0_0;
                const uint32_t v1 = (idx & 2u) ? p1_1 : p0_1;
                const uint32_t v2 = (idx & 4u) ? p1_2 : p0_2;
                const uint32_t v3 = (idx & 8u) ? p1_3 : p0_3;
                lvl[idx] = uint8_t((42u * (v0 + v3 + 2u * (v1 + v2))) >> 8);
            }
            have_lvl = true;
        }

        const auto& b0 = fr.bits[s0];
        const auto& b1 = fr.bits[s1];
        const auto& b2 = fr.bits[s2];
        const auto& b3 = fr.bits[s3];

        uint32_t diff =
            (p0_key ^ ((p0_key << 8) | (p0_key >> 24))) |
            (p1_key ^ ((p1_key << 8) | (p1_key >> 24)));
        for(int wi = 0; wi < 4; ++wi)
        {
            const uint32_t w0 = b0[wi], w1 = b1[wi], w2 = b2[wi], w3 = b3[wi];
            diff |= (w0 ^ w1) | (w1 ^ w2) | (w2 ^ w3);
            uint32_t* dstw = row_out + wi * 8;
            for(int nib = 0; nib < 8; ++nib)
            {
                const int sh = nib * 4;
                const uint32_t code =
                    EXPAND4[(w0 >> sh) & 0xfu] |
                    (EXPAND4[(w1 >> sh) & 0xfu] << 1) |
                    (EXPAND4[(w2 >> sh) & 0xfu] << 2) |
                    (EXPAND4[(w3 >> sh) & 0xfu] << 3);
                dstw[nib] =
                    uint32_t(lvl[code & 0xffu]) |
                    (uint32_t(lvl[(code >> 8) & 0xffu]) << 8) |
                    (uint32_t(lvl[(code >> 16) & 0xffu]) << 16) |
                    (uint32_t(lvl[code >> 24]) << 24);
            }
        }
        if(diff == 0)
            filtered_steady_mask |= (1ULL << y);
        std::memcpy(&filtered_pixels[y * 128], row_out, 128);
    }
}
#if defined(ESP_PLATFORM) && defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif

ARDENS_FORCEINLINE bool display_t::advance(uint64_t ps)
{
    vsync = false;
    ps += ps_rem;
    ps_rem = 0;

    if(ps < ps_per_clk)
    {
        ps_rem = ps;
        return false;
    }

    uint32_t ticks = 0;
    const uint64_t fast_limit = ps_per_clk << 2;
    if(ps < fast_limit)
    {
        while(ps >= ps_per_clk)
        {
            ps -= ps_per_clk;
            ++ticks;
        }
        ps_rem = ps;
    }
    else if(ps <= 0xffffffffULL && ps_per_clk <= 0xffffffffULL)
    {
        const uint32_t ps32 = uint32_t(ps);
        const uint32_t clk32 = uint32_t(ps_per_clk);
        ticks = ps32 / clk32;
        ps_rem = ps32 % clk32;
    }
    else
    {
        ticks = uint32_t(ps / ps_per_clk);
        ps_rem = ps % ps_per_clk;
    }
    uint32_t rc = row_cycle + ticks;
    while(rc >= cycles_per_row)
    {
        update_pixels_row();
        if(row == mux_ratio)
            row = 0;
        else
            row = (row + 1) % 64;
        rc -= cycles_per_row;
    }
    row_cycle = (uint8_t)rc;

    return vsync;
}

constexpr std::array<double, 16> FOSC =
{
    // mostly made up
    200, 224, 248, 272, 296, 320, 344, 368,
    392, 416, 440, 464, 488, 512, 536, 570,
};

ARDENS_FORCEINLINE void display_t::update_clocking()
{
    cycles_per_row = phase_1 + phase_2 + 50;
    ps_per_clk = (uint64_t)round(1e12 * (divide_ratio + 1) / fosc());
}

void display_t::reset()
{
    memset(&ram, 0, sizeof(ram));
    memset(&pixels, 0, sizeof(pixels));
    memset(&filtered_pixels, 0, sizeof(filtered_pixels));
    reset_filter_state(false);

    ref_segment_current = 0.195f;
    current_limit_slope = 0.75f;
    enable_current_limiting = true;
    prev_row_drive = 0;

    contrast = 0x7f;
    entire_display_on = false;
    inverse_display = false;
    display_on = false;
    enable_charge_pump = false;

    addressing_mode = addr_mode::PAGE;

    col_start = 0;
    col_end = (type == SH1106? 131 : 127);
    page_start = 0;
    page_end = 7;

    mux_ratio = 63;

    display_offset = 0;
    display_start = 0;

    com_scan_direction = false;
    alternative_com = true;
    com_remap = false;
    segment_remap = false;

    fosc_index = 8;
    divide_ratio = 0;
    phase_1 = 2;
    phase_2 = 2;
    vcomh_deselect = 2;

    row = 0;
    row_cycle = 0;
    cycles_per_row = 0;
    ps_per_clk = 0;

    ps_rem = 0;
    
    processing_command = false;
    current_command = 0;
    command_byte_index = 0;

    data_page = 0;
    data_col = 0;

    update_clocking();
}

double display_t::fosc() const
{
    double f = FOSC[fosc_index % 16] * 1000.0;
    if(type == SH1106)
        f *= 0.8f;
    return f;
}

double display_t::refresh_rate() const
{
    int D = divide_ratio + 1;
    int K = phase_1 + phase_2 + 50;
    int MUX = mux_ratio + 1;
    return fosc() / double(D * K * MUX);
}

}
