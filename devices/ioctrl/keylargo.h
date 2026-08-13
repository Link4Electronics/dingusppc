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

/** KeyLargo I/O controller definitions (Mac mini G4 / PowerMac10,1/10,2).
 *
 * The KeyLargo is the second-generation Apple I/O controller ("mac-io")
 * used on the Power Mac G4 (AGP Graphics), iBook G4, PowerBook G4 and the
 * Mac mini G4. On the Mac mini it sits on the main UniNorth PCI bus at
 * DEV_FUN(0x17,0), VID/DID 0x106b:0x003e, class 0xFF0000 and maps a 512 KiB
 * MMIO window at BAR0 (0x80000000). Sub-blocks are selected by bits 12..18
 * of the MMIO offset:
 *
 *   0x00 - control/GPIO registers
 *   0x08 - DBDMA
 *   0x10 - I2S (sound)
 *   0x12 - ESCC legacy addressing
 *   0x13 - ESCC MacRISC addressing
 *   0x15 - timer
 *   0x16 - VIA-PMU (registers span 0x16000..0x17FFF, i.e. sub-blocks 0x16
 *           and 0x17, one register per 0x200 bytes)
 *   0x18 - I2C
 *   0x20 - ATA-3
 *   0x40 - OpenPIC (interrupt controller)
 */

#ifndef KEYLARGO_H
#define KEYLARGO_H

#include <devices/common/hwinterrupt.h>
#include <devices/common/pci/pcidevice.h>
#include <devices/common/viapmu.h>

#include <cinttypes>
#include <memory>
#include <string>
#include <vector>

/** KeyLargo control/GPIO register offsets. */
enum KeyLargoReg : uint32_t {
    KL_MBCR            = 0x34, // media bay control/status
    KL_FCR0            = 0x38, // feature control register
    KL_FCR1            = 0x3C,
    KL_FCR2            = 0x40,
    KL_FCR3            = 0x44,
    KL_FCR4            = 0x48,
    KL_FCR5            = 0x4C, // Pangea only
    KL_GPIO_LEVELS0    = 0x50,
    KL_GPIO_LEVELS1    = 0x54,
    KL_GPIO_EXTINT_0   = 0x58, // 18 extint GPIO regs
    KL_GPIO_PMU_MSG_IRQ = 0x61, // GPIO_EXTINT_0 + 9
    KL_GPIO_0          = 0x6A, // 17 GPIO regs
};

/* KeyLargo's BAR0 window (0x80000000, 512 KiB) is mapped by the boot ROM
   and Open Firmware at the fixed device-tree address without going through
   PCI BAR assignment. The UniNorth boot-ROM "config window" (Intrepid,
   0x80008000-0x80008FFF) overlaps the middle of it, so we register the
   window as two chunks around that region. */
#define KL_BAR0_BASE          0x80000000
#define KL_BAR0_CHUNK1_SIZE   0x8000   /* 0x80000000..0x80007FFF */
#define KL_BAR0_CHUNK2_OFFSET 0x9000   /* 0x80009000..0x8007FFFF */
#define KL_BAR0_CHUNK2_SIZE   (0x80000 - KL_BAR0_CHUNK2_OFFSET)

/** KeyLargo MMIO sub-block selectors ((offset >> 12) & 0x7F). */
enum KeyLargoSubBlock : uint32_t {
    KL_SUB_CTRL    = 0x00,
    KL_SUB_DBDMA   = 0x08,
    KL_SUB_I2S     = 0x10,
    KL_SUB_ESCC_LEG= 0x12,
    KL_SUB_ESCC    = 0x13,
    KL_SUB_TIMER   = 0x15,
    KL_SUB_VIA_PMU = 0x16,
    KL_SUB_I2C     = 0x18,
    KL_SUB_ATA     = 0x20,
    KL_SUB_OPENPIC = 0x40,
};

/** KeyLargo I2C controller registers (offsets within the 0x18 sub-block). */
enum KeyLargoI2CReg : uint32_t {
    KL_I2C_CTRL   = 0x00,
    KL_I2C_MODE   = 0x10,
    KL_I2C_STATUS = 0x30, // status (read) / data (write)
    KL_I2C_ADDR   = 0x70,
};

class KeyLargo : public PCIDevice, public InterruptCtrl {
public:
    KeyLargo();
    ~KeyLargo() = default;

    static std::unique_ptr<HWComponent> create() {
        return std::unique_ptr<KeyLargo>(new KeyLargo());
    }

    int device_postinit() override;

    // MMIODevice methods
    uint32_t read(uint32_t rgn_start, uint32_t offset, int size) override;
    void write(uint32_t rgn_start, uint32_t offset, uint32_t value, int size) override;

    // InterruptCtrl methods
    uint64_t register_dev_int(IntSrc src_id) override;
    uint64_t register_dma_int(IntSrc src_id) override;
    void ack_int(uint64_t irq_id, uint8_t irq_line_state) override;
    void ack_dma_int(uint64_t irq_id, uint8_t irq_line_state) override;

private:
    void notify_bar_change(int bar_num);

    uint32_t ctrl_read(uint32_t offset, int size);
    void ctrl_write(uint32_t offset, uint32_t value, int size);
    uint8_t ctrl_read_byte(uint32_t offset);
    void ctrl_write_byte(uint32_t offset, uint8_t value);

    uint32_t i2c_read(uint32_t offset, int size);
    void i2c_write(uint32_t offset, uint32_t value, int size);

    ViaPmu* viapmu;

    uint32_t base_addr   = 0;
    int      iomem_size  = 0x80000;

    uint32_t mbcr   = 0;
    uint32_t fcr[6] = {};
    uint8_t  gpio_levels0 = 0;
    uint8_t  gpio_levels1 = 0;
    uint8_t  gpio_extint[18] = {};
    uint8_t  gpio[17] = {};

    uint8_t  i2c_ctrl = 0;
    uint8_t  i2c_mode = 0;
    uint8_t  i2c_addr = 0;
    uint8_t  i2c_data = 0;

    uint32_t unsupported_read_mask  = 0;
    uint32_t unsupported_write_mask = 0;
};

#endif // KEYLARGO_H
