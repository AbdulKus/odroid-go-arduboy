#include "absim.hpp"

extern "C"
{
#include "boot/boot_flashcart.h"
}

namespace absim
{

void w25q128_t::erase_all_data()
{
#if defined(ARDUGO_EMBEDDED) && defined(ESP_PLATFORM)
    read_sectors.fill(nullptr);
    read_sector_indices.fill(NUM_SECTORS);
#endif
    for(auto& s : sectors) s.reset();
    write_bytes(0, ARDENS_BOOT_FLASHCART, sizeof(ARDENS_BOOT_FLASHCART));
    sectors_modified.reset();
    set_empty_page_range();
}

void w25q128_t::set_empty_page_range()
{
    min_page = EMPTY_PAGE_SENTINEL;
    max_page = EMPTY_PAGE_SENTINEL;
}

bool w25q128_t::has_empty_page_range() const
{
    return
        min_page == EMPTY_PAGE_SENTINEL &&
        max_page == EMPTY_PAGE_SENTINEL;
}

ARDENS_IRAM
uint8_t w25q128_t::read_byte(size_t addr)
{
    if(addr >= DATA_BYTES)
        return 0xff;

    size_t sector_index = addr / SECTOR_BYTES;
    size_t byte_index = addr % SECTOR_BYTES;
#if defined(ARDUGO_EMBEDDED) && defined(ESP_PLATFORM)
    const size_t slot = sector_index & 3u;
    if(!read_sectors[slot] || read_sector_indices[slot] != sector_index)
    {
        read_sectors[slot] = sectors[sector_index].get();
        read_sector_indices[slot] = sector_index;
    }
    return read_sectors[slot] ? (*read_sectors[slot])[byte_index] : 0xff;
#else
    auto& sector = sectors[sector_index];
    return sector ? (*sector)[byte_index] : 0xff;
#endif
}

ARDENS_NOINLINE ARDENS_IRAM
void w25q128_t::fill_cache64(size_t base, uint8_t* dst)
{
    if(base >= DATA_BYTES) [[unlikely]]
    {
        memset(dst, 0xff, 64);
        return;
    }
    const size_t sector_index = base / SECTOR_BYTES;
    const size_t slot = sector_index & 3u;
#if defined(ARDUGO_EMBEDDED) && defined(ESP_PLATFORM)
    if(!read_sectors[slot] || read_sector_indices[slot] != sector_index)
    {
        read_sectors[slot] = sectors[sector_index].get();
        read_sector_indices[slot] = sector_index;
    }
    const sector_t* sector = read_sectors[slot];
#else
    const sector_t* sector = sectors[sector_index].get();
#endif
    if(sector)
        memcpy(dst, &(*sector)[base & (SECTOR_BYTES - 1)], 64);
    else
        memset(dst, 0xff, 64);
}

void w25q128_t::write_byte(size_t addr, uint8_t data)
{
    if(addr >= DATA_BYTES)
        return;

    ++data_gen;
    size_t sector_index = addr / SECTOR_BYTES;
    size_t byte_index = addr % SECTOR_BYTES;
    auto& sector = sectors[sector_index];
    if(!sector)
    {
        sector = std::make_unique<sector_t>();
        memset(sector->data(), 0xff, SECTOR_BYTES);
    }
    (*sector)[byte_index] = data;
}

void w25q128_t::program_byte(size_t addr, uint8_t data)
{
    write_byte(addr, read_byte(addr) & data);
}

void w25q128_t::write_bytes(size_t addr, uint8_t const* data, size_t bytes)
{
    if(addr >= DATA_BYTES)
        return;
    bytes = std::min(bytes, DATA_BYTES - addr);
    ++data_gen;

    while(bytes > 0)
    {
        size_t sector_index = addr / SECTOR_BYTES;
        size_t byte_index = addr % SECTOR_BYTES;
        size_t num_bytes = std::min<size_t>(SECTOR_BYTES - byte_index, bytes);
        auto& sector = sectors[sector_index];
        if(!sector)
        {
            sector = std::make_unique<sector_t>();
            if(num_bytes < SECTOR_BYTES)
                memset(sector->data(), 0xff, SECTOR_BYTES);
        }
        memcpy(sector->data() + byte_index, data, num_bytes);
        bytes -= num_bytes;
        addr += num_bytes;
        data += num_bytes;
    }
}

void w25q128_t::reset()
{
    enabled = false;
    woken_up = false;
    write_enabled = false;
    reading_status = false;
    processing_command = false;
    command = CMD_NONE;

    reading = 0;
    programming = 0;
    erasing_sector = 0;
    releasing = 0;

    busy_ps_rem = 0;
    current_addr = 0;

    sectors_dirty = false;

    busy_error = false;
    ++data_gen;
#if defined(ARDUGO_EMBEDDED) && defined(ESP_PLATFORM)
    read_sectors.fill(nullptr);
    read_sector_indices.fill(NUM_SECTORS);
#endif
    for(auto& s : sectors_modified_data)
        s.reset();
}

ARDENS_IRAM
void w25q128_t::set_enabled(bool e)
{
    if(enabled != e)
    {
        enabled = e;
        if(!e)
        {
            reading_status = false;
            reading = 0;
            programming = 0;
            erasing_sector = 0;
            processing_command = false;
            releasing = 0;
            reading_jedec_id = 0;
            if(busy_ps_rem == 0)
                command = CMD_NONE;
        }
    }
}

ARDENS_FORCEINLINE void w25q128_t::advance(uint64_t ps)
{
    if(ps >= busy_ps_rem)
    {
        if(!enabled)
            command = CMD_NONE;
        busy_ps_rem = 0;
    }
    else
        busy_ps_rem -= ps;
}

ARDENS_FORCEINLINE void w25q128_t::track_page()
{
    current_addr &= 0xffffff;
    uint32_t page = current_addr / PAGE_BYTES;
    if(has_empty_page_range())
    {
        min_page = page;
        max_page = page;
    }
    else
    {
        min_page = std::min(min_page, page);
        max_page = std::max(max_page, page);
    }
}

ARDENS_FORCEINLINE
uint8_t w25q128_t::spi_transceive(uint8_t byte)
{
    if(!enabled) return 0;

    if(reading >= 4) [[likely]]
    {
        busy_error = false;
        const uint8_t data_to_send = read_byte(current_addr);
        current_addr = (current_addr + 1) & 0xffffffu;
        return data_to_send;
    }

    return spi_transceive_slow(byte);
}

ARDENS_IRAM
uint8_t w25q128_t::spi_transceive_slow(uint8_t byte)
{

    uint8_t data_to_send = 0;
    busy_error = false;

    if(reading)
    {
        if(reading <= 3)
        {
            current_addr = (current_addr << 8) | byte;
            current_addr &= 0xffffff;
            ++reading;
        }
    }
    else if(programming)
    {
        if(programming == 4)
        {
            track_page();
            uint32_t page = current_addr & 0xffff00;
#ifdef ARDUGO_EMBEDDED
            if(current_addr >= save_start)
#endif
            {
                sectors_modified.set(current_addr >> 12);
                sectors_dirty = true;
                program_byte(current_addr, byte);
            }
            ++current_addr;
            current_addr = page | (current_addr & 0xff);
            data_to_send = 0;
            busy_ps_rem = 700ull * 1000 * 1000; // 0.7 ms
        }
        else
        {
            current_addr = (current_addr << 8) | byte;
            ++programming;
        }
    }
    else if(reading_status)
    {
        if(busy_ps_rem != 0)
            data_to_send = 0x1;
    }
    else if(erasing_sector)
    {
        if(erasing_sector <= 3)
        {
            current_addr = (current_addr << 8) | byte;
            ++erasing_sector;
        }
        if(erasing_sector == 4)
        {
            current_addr &= 0xfff000;
            track_page();
#ifdef ARDUGO_EMBEDDED
            if(current_addr >= save_start)
#endif
            {
                ++data_gen;
                auto& sector = sectors[current_addr / SECTOR_BYTES];
                if(sector)
                    memset(sector->data(), 0xff, SECTOR_BYTES);
                sectors_modified.set(current_addr >> 12);
                sectors_dirty = true;
            }
            busy_ps_rem = 100ull * 1000 * 1000 * 1000; // 100 ms
            erasing_sector = 0;
        }
    }
    else if(releasing)
    {
        if(releasing == 1) data_to_send = 0x00;
        if(releasing == 2) data_to_send = 0x00;
        if(releasing == 3) data_to_send = 0x00;
        if(releasing <= 3)
            ++releasing;
        if(releasing == 4)
        {
            releasing = 0;
            command = CMD_NONE;
            processing_command = false;
            data_to_send = 0x00;
        }
    }
    else if(reading_jedec_id)
    {
        if(reading_jedec_id == 1) data_to_send = 0x40;
        if(reading_jedec_id == 2) data_to_send = 0x17;
        if(reading_jedec_id <= 2)
            ++reading_jedec_id;
        if(reading_jedec_id >= 3)
        {
            reading_jedec_id = 0;
            command = CMD_NONE;
            processing_command = false;
        }
    }
    else if(!woken_up)
    {
        if(byte == 0xab)
        {
            processing_command = true;
            command = CMD_RELEASE_POWER_DOWN;
            woken_up = true;
            releasing = 1;
        }
    }
    else if(!processing_command)
    {
        processing_command = true;
        if(busy_ps_rem != 0 && byte != 0x05 && byte != 0x06)
            busy_error = true;
        switch(byte)
        {
        case 0x02: // program page
            if(!write_enabled || busy_ps_rem != 0) break;
            command = CMD_PAGE_PROGRAM;
            programming = 1;
            current_addr = 0;
            break;
        case 0x03: // read data
            if(busy_ps_rem != 0) break;
            command = CMD_READ_DATA;
            reading = 1;
            current_addr = 0;
            break;
        case 0x04: // write disable
            if(busy_ps_rem != 0) break;
            command = CMD_WRITE_DISABLE;
            write_enabled = false;
            break;
        case 0x05: // read status register 1
            command = CMD_READ_STATUS_REGISTER_1;
            reading_status = true;
            if(busy_ps_rem != 0)
                data_to_send = 0x1;
            break;
        case 0x06: // write enable
            if(busy_ps_rem != 0) break;
            command = CMD_WRITE_ENABLE;
            write_enabled = true;
            break;
        case 0x20: // sector erase
            if(!write_enabled || busy_ps_rem != 0) break;
            command = CMD_SECTOR_ERASE;
            erasing_sector = 1;
            current_addr = 0;
            break;
        case 0x9f:
            command = CMD_JEDEC_ID;
            reading_jedec_id = 1;
            data_to_send = 0xef;
            break;
        case 0xb9:
            woken_up = false;
            break;
        default:
            command = CMD_UNKNOWN;
            processing_command = false;
            break;
        }
    }

    return data_to_send;
}

}
