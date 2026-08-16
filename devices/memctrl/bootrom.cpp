/*
DingusPPC - The Experimental PowerPC Macintosh emulator
Copyright (C) 2018-26 The DingusPPC Development Team
          (See CREDITS.MD for more details)

(You may also contact divingkxt or powermax2286 on Discord)

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

#include <devices/memctrl/bootrom.h>
#include <devices/deviceregistry.h>
#include <machines/machinefactory.h>
#include <loguru.hpp>

#include <cstring>

BootRom::BootRom(const std::string &dev_name, uint32_t size)
    : MMIODevice()
{
    this->name = dev_name;
    supports_types(HWCompType::MMIO_DEV | HWCompType::ROM);
    this->rom_size = size;
}

int BootRom::device_postinit() {
    return 0;
}

void BootRom::set_rom_write_enable(const bool enable)
{
    LOG_F(INFO, "%s: ROM write %s", this->name.c_str(), enable ? "enabled" : "disabled");
    if (this->rom_entry) {
        this->rom_entry->type = enable ? RT_MMIO : RT_ROM;
    }
    this->rom_we = enable;
}

void BootRom::identify_rom()
{
    MachineFactory::machine_name_from_rom((char *)get_data(), this->rom_size);
}

int BootRom::set_data(const uint8_t* data, uint32_t size)
{
    if (size > this->rom_size) {
        LOG_F(ERROR, "%s: ROM source is larger than expected.", this->name.c_str());
        return -1;
    }
    if (size < this->rom_size)
        LOG_F(WARNING, "%s: ROM source is smaller than expected.", this->name.c_str());
    std::memcpy(this->rom_entry->mem_ptr + this->rom_size - size, data, size);
    LOG_F(INFO, "%s: loaded %u bytes into ROM", this->name.c_str(), size);
    return 0;
}

/* New World BootRom - 1MB with single flash chip */

BootRomNW::BootRomNW()
    : BootRom("BootRomNW", 0x100000)
{
    LOG_F(INFO, "%s: New World boot ROM (1 MB)", this->name.c_str());
}

uint32_t BootRomNW::read(uint32_t rgn_start, uint32_t offset, int size)
{
    if (size != 4 || offset & 3) {
        LOG_F(ERROR, "%s: unexpected read size/offset @%06X.%c",
            this->name.c_str(), offset, SIZE_ARG(size));
        return 0;
    }

    if (!this->flash_chip) {
        LOG_F(ERROR, "%s: no flash chip attached", this->name.c_str());
        return 0;
    }

    uint32_t value = 0;
    for (int i = 0; i < 4; i++) {
        value = (value << 8) | this->flash_chip->read(offset + i);
    }

    return value;
}

void BootRomNW::write(uint32_t rgn_start, uint32_t offset, uint32_t value, int size)
{
    if (size != 4 || offset & 3) {
        LOG_F(ERROR, "%s: unexpected write size/offset @%06X.%c = %08X",
            this->name.c_str(), offset, SIZE_ARG(size), value);
        return;
    }

    if (!this->flash_chip) {
        LOG_F(ERROR, "%s: no flash chip attached", this->name.c_str());
        return;
    }

    for (int i = 0; i < 4; i++) {
        this->flash_chip->write(offset + i, (value >> ((3 - i) * 8)) & 0xFF);
    }
}

uint8_t BootRomNW::rom_read(FlashChip *chip, uint32_t addr)
{
    return this->get_data()[addr];
}

void BootRomNW::rom_write(FlashChip *chip, uint32_t addr, uint8_t value)
{
    this->get_data()[addr] = value;
}

/* Device registration */

static const DeviceDescription BootRomNW_Descriptor = {
    BootRomNW::create, {}, {},
    HWCompType::MMIO_DEV | HWCompType::ROM
};

REGISTER_DEVICE(BootRomNW, BootRomNW_Descriptor);
