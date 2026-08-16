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

/** Sharp LH28F008BJT flash chip emulation (Intel command set). */

#include <devices/memctrl/flash.h>
#include <loguru.hpp>

void FlashChip::set_controller(FlashController* c)
{
    this->controller = c;
}

SharpLH28F008BJT::SharpLH28F008BJT()
    : FlashChip("SharpLH28F008BJT")
{
    supports_types(HWCompType::FLASH);
    LOG_F(INFO, "%s: Sharp LH28F008BJT 8 Mbit flash chip (Mfr 0x%02X, Dev 0x%02X)",
        this->get_name().c_str(), this->manufacturer_id, this->device_id);
}

uint8_t SharpLH28F008BJT::read(uint32_t addr)
{
    uint8_t value;

    switch (this->state) {

    case Flash::Reset:
    case Flash::ReadArray:
        value = controller->rom_read(this, addr);
        break;

    case Flash::ReadIdentifier:
        switch (addr & 0xFF) {
        case 0:
            value = this->manufacturer_id;
            break;
        case 1:
            value = this->device_id;
            break;
        default:
            value = 0;
            break;
        }
        break;

    case Flash::ReadStatus:
        value = this->status_reg;
        break;

    case Flash::EraseSuspend:
        value = controller->rom_read(this, addr);
        break;

    default:
        value = 0;
        LOG_F(ERROR, "%s: unexpected read in state %d addr=%06X",
            this->get_name().c_str(), this->state, addr);
        break;
    }

    return value;
}

void SharpLH28F008BJT::write(uint32_t addr, uint8_t value)
{
    switch (this->state) {

    case Flash::ReadArray:
    case Flash::Reset:
        switch (value) {
        case 0xFF:  // Read Array / Reset
            this->state = Flash::Reset;
            break;
        case 0x90:  // Read Identifier
            this->state = Flash::ReadIdentifier;
            break;
        case 0x70:  // Read Status Register
            this->status_reg |= SR_WSM_READY;
            this->state = Flash::ReadStatus;
            break;
        case 0x50:  // Clear Status Register
            this->status_reg = SR_WSM_READY;
            this->state = Flash::ClearStatus;
            break;
        case 0x20:  // Block Erase Setup
            this->block_addr = addr;
            this->state = Flash::EraseSetup;
            break;
        case 0x30:  // Full Chip Erase Setup
            this->block_addr = addr;
            this->state = Flash::EraseSetup;
            break;
        case 0x40:  // Byte Write Setup
        case 0x10:  // Alternate Byte Write Setup
            this->write_addr = addr;
            this->state = Flash::ProgramSetup;
            break;
        case 0xB0:  // Erase Suspend
            this->state = Flash::EraseSuspend;
            break;
        case 0xD0:  // Erase Resume
            this->state = Flash::EraseSuspend;
            break;
        default:
            LOG_F(ERROR, "%s: ReadArray unexpected cmd %02X at %06X",
                this->get_name().c_str(), value, addr);
            break;
        }
        break;

    case Flash::ReadIdentifier:
        if (value == 0xFF) {
            this->state = Flash::Reset;
        }
        break;

    case Flash::ReadStatus:
        if (value == 0xFF) {
            this->state = Flash::Reset;
        } else if (value == 0x50) {
            this->status_reg = SR_WSM_READY;
            this->state = Flash::ClearStatus;
        }
        break;

    case Flash::ClearStatus:
        this->state = Flash::ReadArray;
        break;

    case Flash::EraseSetup:
        if (value == 0xD0) {
            // Erase confirm - erase the block
            this->status_reg |= SR_WSM_READY;
            LOG_F(INFO, "%s: erasing block at %06X",
                this->get_name().c_str(), this->block_addr);
            // Erase block (set all bytes to 0xFF)
            for (uint32_t i = 0; i < 0x10000; i++) {
                controller->rom_write(this, (this->block_addr & ~0xFFFF) + i, 0xFF);
            }
            this->state = Flash::ReadStatus;
        } else {
            LOG_F(ERROR, "%s: EraseSetup unexpected cmd %02X",
                this->get_name().c_str(), value);
            this->state = Flash::ReadArray;
        }
        break;

    case Flash::ProgramSetup:
        // Program the byte
        this->status_reg |= SR_WSM_READY;
        controller->rom_write(this, this->write_addr, value);
        LOG_F(INFO, "%s: program %06X = %02X",
            this->get_name().c_str(), this->write_addr, value);
        this->state = Flash::ReadStatus;
        break;

    case Flash::EraseSuspend:
        if (value == 0xD0) {
            // Erase resume
            this->state = Flash::ReadArray;
        } else if (value == 0xFF) {
            this->state = Flash::Reset;
        }
        break;

    default:
        LOG_F(ERROR, "%s: unexpected write %02X in state %d at %06X",
            this->get_name().c_str(), value, this->state, addr);
        break;
    }
}
