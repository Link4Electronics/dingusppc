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

/** Intrepid (UniNorth 2) definitions.

    The UniNorth (U2/U3 "Intrepid") is the combined memory controller and
    multi-PCI-host bridge used in the Power Mac G4 (AGP Graphics), iBook G4,
    PowerBook G4 and Mac mini G4. It contains three PCI host bridges:

    - the main PCI bus
    - the AGP bus
    - the internal PCI bus (FireWire, GMAC, etc.)

    Each of the three buses has its own configuration window:

    Main bus:
      config addr 0xF2800000, config data 0xF2C00000, I/O space 0xF2000000
    AGP bus:
      config addr 0xF0800000, config data 0xF0C00000
    Internal bus:
      config addr 0xF4800000, config data 0xF4C00000

    The UniNorth control registers live at 0xF8000000 (UNI_N_VERSION at
    offset 0, must read 0x00D2 on the Intrepid/UniNorth 2 rev used in the
    Mac mini G4).

    This emulation only implements what is needed to boot the Mac mini G4
    boot ROM up to the point where Open Firmware can probe the PCI buses.
*/

#ifndef INTREPID_H
#define INTREPID_H

#include <devices/common/pci/pcidevice.h>
#include <devices/common/pci/pcihost.h>
#include <devices/memctrl/memctrlbase.h>
#include <machines/machinebase.h>

#include <cinttypes>
#include <memory>

class Intrepid;

/** UniNorth control register offsets. */
enum UniNReg : uint32_t {
    UNI_N_VERSION     = 0x0000, // chip version, 0x00D2 for Intrepid
    UNI_N_CLOCK_CNTL  = 0x0020, // clock control (PCI2, GMAC, FireWire, ATA-100)
    UNI_N_POWER_MGT   = 0x0030, // power management
    UNI_N_ARB_CTRL    = 0x0040, // arbitration control
    UNI_N_CPU_NUMBER  = 0x0050, // CPU number, read by the bootROM
    UNI_N_HWINIT_STATE= 0x0070, // hardware init state, read by the bootROM
    UNI_N_AACK_DELAY  = 0x0100, // AACK delay
};

/** PCI configuration window addresses of the UniNorth. */
enum UniNWindow : uint32_t {
    UNI_N_WINDOW_AGP      = 0, // AGP bus
    UNI_N_WINDOW_MAIN     = 1, // main PCI bus
    UNI_N_WINDOW_INTERNAL = 2, // internal PCI bus

    UNI_N_AGP_CONFIG_ADDR     = 0xF0800000,
    UNI_N_AGP_CONFIG_DATA     = 0xF0C00000,
    UNI_N_MAIN_IO_SPACE       = 0xF2000000,
    UNI_N_MAIN_CONFIG_ADDR    = 0xF2800000,
    UNI_N_MAIN_CONFIG_DATA    = 0xF2C00000,
    UNI_N_INTERNAL_CONFIG_ADDR = 0xF4800000,
    UNI_N_INTERNAL_CONFIG_DATA = 0xF4C00000,
    UNI_N_REGISTER_BLOCK      = 0xF8000000,

    // UniNorth I2C controller (i2c@f8001000 in the device tree).
    UNI_N_I2C_BLOCK           = 0xF8001000,

    // Memory controller clock/control registers.
    UNI_N_CLK_BLOCK           = 0xF8002000,

    // Memory mapped "config window" used by the boot ROM's early PCI
    // quiesce/init code. Two window bases exist at 0x80008000 and
    // 0x80008800; a single 4 KiB region covers both.
    UNI_N_CONFIG_WINDOW       = 0x80008000,
};

/** UniNorth I2C controller register offsets (within the 0xF8001000 block).

    Byte registers sit on byte lane 3 of the keywest-style register file,
    i.e. physical base + 0x10*n + 3 (AAPL,address = 0xF8001003,
    AAPL,address-step = 0x10); the two 32-bit clock/command registers sit
    at base + 0x90 / 0xA0. The boot ROM's I2C driver (0xfff8b51c/0xfff8b5b4/
    0xfff8b8d0) uses these offsets directly. */
enum UniNI2CReg : uint32_t {
    UNI_N_I2C_MODE       = 0x03, // channel + machine mode
    UNI_N_I2C_CTRL       = 0x13, // command register
    UNI_N_I2C_STATUS_ACK = 0x23, // transfer status; bit 1 = ack received
    UNI_N_I2C_STATUS     = 0x33, // controller status / ISR
    UNI_N_I2C_IER        = 0x43, // interrupt enable
    UNI_N_I2C_ADDR       = 0x53, // target device address
    UNI_N_I2C_SUBADDR    = 0x63, // sub-address
    UNI_N_I2C_DATA       = 0x73, // data byte
    UNI_N_I2C_SSADDR     = 0x83, // secondary address
    UNI_N_I2C_ADDR_CTL   = 0x90, // address control (32-bit)
    UNI_N_I2C_DATA2      = 0xA0, // data 2 (32-bit)
};

/** UniNorth memory controller clock register offsets (within 0xF8002000). */
enum UniNClkReg : uint32_t {
    UNI_N_CLK_BOOT       = 0x080, // 0xF8002080
    UNI_N_CLK_CTL        = 0x180, // 0xF8002180
    UNI_N_CLK_SUN        = 0x1C0, // 0xF80021C0 (4 banks, stride 0x40)
    UNI_N_CLK_SUN_STRIDE = 0x040,
    UNI_N_CLK_REG        = 0x500, // 0xF8002500 (CPU clock PLL, RMW)
};

/** PCI-facing part of the UniNorth host bridges on the AGP and internal buses. */
class IntrepidPciHostDevice : public PCIDevice {
public:
    IntrepidPciHostDevice(std::string name, int dev_id);
    ~IntrepidPciHostDevice() = default;

    static std::unique_ptr<HWComponent> create_agp() {
        return std::unique_ptr<IntrepidPciHostDevice>(new IntrepidPciHostDevice("UniNorth AGP", 0x0018));
    }

    static std::unique_ptr<HWComponent> create_internal() {
        return std::unique_ptr<IntrepidPciHostDevice>(new IntrepidPciHostDevice("UniNorth Internal PCI", 0x001E));
    }

    uint32_t pci_cfg_read(uint32_t reg_offs, AccessDetails &details) override;
    void pci_cfg_write(uint32_t reg_offs, uint32_t value, AccessDetails &details) override;
};

/** Intrepid memory controller + PCI host bridges. */
class Intrepid : public MemCtrlBase, public PCIDevice, public PCIHost {
public:
    Intrepid();
    ~Intrepid() = default;

    static std::unique_ptr<HWComponent> create() {
        return std::unique_ptr<Intrepid>(new Intrepid());
    }

    uint32_t read(uint32_t rgn_start, uint32_t offset, int size) override;
    void write(uint32_t rgn_start, uint32_t offset, uint32_t value, int size) override;

    int device_postinit() override;

    /** Assign the IRQ routing maps of the AGP and internal PCI buses. */
    void set_agp_irq_map(const std::vector<PciIrqMap>& irq_map) {
        this->agp_host.set_irq_map(irq_map);
    }
    void set_internal_irq_map(const std::vector<PciIrqMap>& irq_map) {
        this->internal_host.set_irq_map(irq_map);
    }

    /** Allocate RAM starting at address 0 (the UniNorth RAM configuration
        registers are not emulated yet). */
    void setup_ram(int capacity_megs);

    /** Config window handlers — called from KeyLargo's DBDMA handler because
     *  the config window at 0x80008000 overlaps the DBDMA register block. */
    uint32_t read_config_window(uint32_t offset, int size);
    void write_config_window(uint32_t offset, uint32_t value, int size);

protected:
    /* my own PCI configuration registers access (main bus host bridge) */
    uint32_t pci_cfg_read(uint32_t reg_offs, AccessDetails &details) override;
    void pci_cfg_write(uint32_t reg_offs, uint32_t value, AccessDetails &details) override;

private:
    uint32_t read_unin_register(uint32_t offset, int size);
    void write_unin_register(uint32_t offset, uint32_t value, int size);
    uint32_t read_i2c_register(uint32_t offset, int size);
    void write_i2c_register(uint32_t offset, uint32_t value, int size);
    uint32_t read_clk_register(uint32_t offset, int size);
    void write_clk_register(uint32_t offset, uint32_t value, int size);
    uint32_t config_read(int window, uint32_t offset, int size);
    void config_write(int window, uint32_t offset, uint32_t value, int size);

    PCIHost agp_host;       // secondary PCI host for the AGP bus
    PCIHost internal_host;  // secondary PCI host for the internal bus
    PCIHost *hosts[3];      // hosts[UNI_N_WINDOW_AGP/MAIN/INTERNAL]

    std::unique_ptr<IntrepidPciHostDevice> agp_dev;      // PCI device on the AGP bus
    std::unique_ptr<IntrepidPciHostDevice> internal_dev; // PCI device on the internal bus

    uint32_t config_addr[3] = {};

    uint32_t clock_cntl  = 0;
    uint32_t power_mgt   = 0;
    uint32_t arb_ctrl    = 0;
    uint32_t aack_delay = 0;
    uint32_t hwinit_state = 0; // 0 = cold boot, 1 = sleep, 2 = running

    uint32_t macrisc_addr_select = 0x00000001; // kMacRISCPCIAddressSelect @0x48

    // UniNorth I2C controller register file (0xF8001000 block)
    uint8_t  i2c_mode    = 0;
    uint8_t  i2c_ctrl    = 0;
    uint8_t  i2c_ier     = 0;
    uint8_t  i2c_addr    = 0;
    uint8_t  i2c_subaddr = 0;
    uint8_t  i2c_data    = 0xFF; // empty bus reads pulled high
    uint8_t  i2c_ssaddr  = 0;
    uint32_t i2c_addr_ctl = 0;
    uint32_t i2c_data2    = 0;

    // Memory controller clock register (0xF8002500), read-modify-write by
    // the boot ROM's clock setup (0xfff888c0) and DRAM timing code.
    uint32_t clk_reg = 0;

    // Per-bank memory config bytes read by the boot ROM's RAM-size probe
    // (0xfff88568) at 0xF80021C0 + bank*32: byte 0 at +0, byte 0x10 at +0x10.
    // byte 0x10's low 6 bits mark the bank present; its top nibble together
    // with byte 0's bit 0 select the DDR timing entry from the ROM's table.
    // Bank 0 = the installed 512 MB DDR DIMM, banks 1-7 empty.
    uint8_t mem_bank_cfg[8][2] = {};

    // Descriptor table pointer last written to each config window base
    // (0x80008000 and 0x80008800); used to dedupe the INFO log.
    uint32_t config_window_ptr[2] = {};
};

#endif // INTREPID_H
