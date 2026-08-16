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

/** Flash chip emulation for Mac mini G4 and similar machines. */

#ifndef FLASH_H
#define FLASH_H

#include <devices/common/hwcomponent.h>
#include <cstdint>
#include <memory>
#include <string>

class FlashController;

namespace Flash {

    enum State : uint8_t {
        ReadArray,
        ReadIdentifier,
        ReadStatus,
        EraseSetup,
        EraseConfirm,
        ProgramSetup,
        ProgramConfirm,
        EraseSuspend,
        ByteWrite,
        ClearStatus,
        Reset,
    };

} // namespace Flash

/** Abstract base class for flash chips. */
class FlashChip : virtual public HWComponent {
public:
    FlashChip(const std::string &dev_name) : HWComponent(dev_name) {}
    virtual ~FlashChip() = default;

    virtual void set_controller(FlashController* controller);
    virtual uint8_t read(uint32_t addr) = 0;
    virtual void write(uint32_t addr, uint8_t value) = 0;

    FlashController *controller = nullptr;
};

/** Abstract base class for flash controllers (ROM devices). */
class FlashController {
public:
    FlashController() = default;
    virtual ~FlashController() = default;

    virtual uint8_t rom_read(FlashChip *chip, uint32_t addr) = 0;
    virtual void rom_write(FlashChip *chip, uint32_t addr, uint8_t value) = 0;
};

/** Sharp LH28F008BJT flash chip (Intel command set, 8-bit, 1MB). */
class SharpLH28F008BJT : public FlashChip {
public:
    SharpLH28F008BJT();
    virtual ~SharpLH28F008BJT() = default;

    static std::unique_ptr<HWComponent> create() {
        return std::unique_ptr<HWComponent>(new SharpLH28F008BJT());
    }

    // FlashChip methods
    virtual uint8_t read(uint32_t addr) override;
    virtual void write(uint32_t addr, uint8_t value) override;

private:
    // Status register bits
    enum StatusReg : uint8_t {
        SR_WSM_READY   = 0x80,  // SR.7: Write State Machine ready
        SR_ERASE_SUSP  = 0x40,  // SR.6: Erase suspended
        SR_ERASE_ERR   = 0x20,  // SR.5: Erase error
        SR_PROG_ERR    = 0x10,  // SR.4: Program error
        SR_VPP_ERR     = 0x08,  // SR.3: VPP low voltage
    };

    uint8_t     manufacturer_id = 0x89;  // Sharp/Intel
    uint8_t     device_id       = 0xA2;  // LH28F008BJT
    Flash::State state          = Flash::ReadArray;
    uint8_t     status_reg      = SR_WSM_READY;
    uint32_t    block_addr      = 0;     // address for erase/program operations
    uint32_t    write_addr      = 0;     // address for byte write
};

#endif // FLASH_H
