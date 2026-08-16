/*
DingusPPC - The Experimental PowerPC Macintosh emulator
Copyright (C) 2018-26 The DingusPPC Development Team
          (See CREDITS.MD for more details)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

/** Boot ROM emulation with flash chip support. */

#ifndef BOOTROM_H
#define BOOTROM_H

#include <devices/common/mmiodevice.h>
#include <devices/memctrl/memctrlbase.h>
#include <devices/memctrl/flash.h>
#include <array>
#include <memory>

class BootRom : public MMIODevice, public FlashController {
public:
    BootRom(const std::string &dev_name, uint32_t size);
    virtual ~BootRom() = default;

    // HWComponent methods
    virtual int device_postinit() override;

    // MMIODevice methods
    virtual uint32_t read(uint32_t rgn_start, uint32_t offset, int size) override = 0;
    virtual void write(uint32_t rgn_start, uint32_t offset, uint32_t value, int size) override = 0;

    // BootRom methods
    virtual void set_rom_write_enable(const bool enable);
    virtual int set_data(const uint8_t* data, uint32_t size);
    virtual uint8_t* get_data() { return rom_entry->mem_ptr; };
    virtual void identify_rom();

    // FlashController methods
    virtual uint8_t rom_read(FlashChip *chip, uint32_t addr) override = 0;
    virtual void rom_write(FlashChip *chip, uint32_t addr, uint8_t value) override = 0;

    AddressMapEntry*    rom_entry = nullptr;

private:
    bool                rom_we = false;
    uint32_t            rom_size;
};

/** New World boot ROM (1MB, single flash chip). */
class BootRomNW : public BootRom {
public:
    BootRomNW();
    virtual ~BootRomNW() = default;

    static std::unique_ptr<HWComponent> create() {
        return std::unique_ptr<HWComponent>(new BootRomNW());
    }

    // MMIODevice methods
    virtual uint32_t read(uint32_t rgn_start, uint32_t offset, int size) override;
    virtual void write(uint32_t rgn_start, uint32_t offset, uint32_t value, int size) override;

    // FlashController methods
    virtual uint8_t rom_read(FlashChip *chip, uint32_t addr) override;
    virtual void rom_write(FlashChip *chip, uint32_t addr, uint8_t value) override;

    FlashChip*  flash_chip = nullptr;
};

#endif // BOOTROM_H
