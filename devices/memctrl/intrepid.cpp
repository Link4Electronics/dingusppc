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

/** Intrepid (UniNorth 2) emulation. */

#include <core/endianswap.h>
#include <devices/common/hwcomponent.h>
#include <devices/common/hwinterrupt.h>
#include <devices/deviceregistry.h>
#include <devices/memctrl/intrepid.h>
#include <loguru.hpp>

#include <algorithm>
#include <cinttypes>
#include <cstring>
#include <string>

/* Number of trailing zero bits; returns 32 for a zero value. */
static int ctz32(uint32_t val) {
    int n = 0;
    if (!val)
        return 32;
    while (!(val & 1)) {
        val >>= 1;
        n++;
    }
    return n;
}

/* Serial presence detect (SPD) data of the 512 MB DIMM in a Mac mini G4
   (PowerMac10,2), byte-for-byte from the device tree's
   /proc/device-tree/memory@0/dimm-info. The DIMM is a DDR SDRAM (type 7,
   byte 2), 13 rows / 11 cols / 4 banks, i.e. the (0x0D, 0x0B, 0x04) triple
   the boot ROM's memory-training table scan expects (0xfff883d8, "0xD0"
   version table). */
static const uint8_t spd_dimm_info[128] = {
    0x80, 0x08, 0x07, 0x0d, 0x0b, 0x02, 0x40, 0x00, 0x04, 0x50, 0x70, 0x00, 0x82, 0x08, 0x00, 0x01,
    0x0e, 0x04, 0x1c, 0x01, 0x02, 0x20, 0x00, 0x60, 0x70, 0x75, 0x70, 0x3c, 0x28, 0x3c, 0x28, 0x80,
    0x60, 0x60, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x37, 0x46, 0x28, 0x28, 0x50, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0xf3, 0x7f, 0x7f,
    0x7f, 0x7f, 0x7f, 0xe3, 0x00, 0x00, 0x00, 0x35, 0x31, 0x35, 0x31, 0x32, 0x36, 0x32, 0x31, 0x35,
    0x35, 0x38, 0x31, 0x34, 0x34, 0x32, 0x30, 0x30, 0x30, 0x41, 0x00, 0x09, 0x06, 0xf1, 0xfb, 0x08,
    0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

IntrepidPciHostDevice::IntrepidPciHostDevice(std::string name, int dev_id)
    : PCIDevice(name)
{
    supports_types(HWCompType::PCI_DEV);

    // populate PCI config header
    this->vendor_id   = PCI_VENDOR_APPLE;
    this->device_id   = dev_id;
    this->class_rev   = 0x06000000;
    this->cache_ln_sz = 8;
    this->lat_timer   = 0x10;
}

uint32_t IntrepidPciHostDevice::pci_cfg_read(uint32_t reg_offs, AccessDetails &details)
{
    if (reg_offs < 64) {
        return PCIDevice::pci_cfg_read(reg_offs, details);
    }
    LOG_READ_UNIMPLEMENTED_CONFIG_REGISTER();
    return 0; // PCI Spec §6.1
}

void IntrepidPciHostDevice::pci_cfg_write(uint32_t reg_offs, uint32_t value, AccessDetails &details)
{
    if (reg_offs < 64) {
        PCIDevice::pci_cfg_write(reg_offs, value, details);
        return;
    }
    LOG_WRITE_UNIMPLEMENTED_CONFIG_REGISTER();
}

Intrepid::Intrepid() : MemCtrlBase(), PCIDevice("Intrepid"), PCIHost()
{
    supports_types(HWCompType::MEM_CTRL | HWCompType::MMIO_DEV |
                   HWCompType::PCI_HOST | HWCompType::PCI_DEV);

    // populate PCI config header of the main bus host bridge (UniNorth PCI)
    this->vendor_id   = PCI_VENDOR_APPLE;
    this->device_id   = 0x0017;
    this->class_rev   = 0x06000000;
    this->cache_ln_sz = 8;
    this->lat_timer   = 0x10;

    // add memory mapped I/O region for the UniNorth control registers
    this->add_mmio_region(UNI_N_REGISTER_BLOCK, 0x1000, this);

    // add memory mapped I/O region for the UniNorth I2C controller
    // (i2c@f8001000) and the memory controller clock registers
    // (0xF8002000 block, incl. the 0xF8002500 CPU clock PLL register)
    this->add_mmio_region(UNI_N_I2C_BLOCK, 0x1000, this);
    this->add_mmio_region(UNI_N_CLK_BLOCK, 0x1000, this);

    // add memory mapped I/O region for the boot ROM's early PCI quiesce
    // config window (0x80008000 and 0x80008800 within a single 4 KiB range)
    this->add_mmio_region(UNI_N_CONFIG_WINDOW, 0x1000, this);

    // add memory mapped I/O regions for the PCI configuration windows.
    // Each window has a config addr register at base + 0x800000 and a
    // config data port at base + 0xC00000 (per linux setup_uninorth()).
    // The main bus additionally decodes an 8 MB PCI I/O space at 0xF2000000.
    this->add_mmio_region(UNI_N_AGP_CONFIG_ADDR,      0x1000, this);
    this->add_mmio_region(UNI_N_AGP_CONFIG_DATA,      0x1000, this);
    this->add_mmio_region(UNI_N_MAIN_IO_SPACE,        0x800000, this);
    this->add_mmio_region(UNI_N_MAIN_CONFIG_ADDR,     0x1000, this);
    this->add_mmio_region(UNI_N_MAIN_CONFIG_DATA,     0x1000, this);
    this->add_mmio_region(UNI_N_INTERNAL_CONFIG_ADDR, 0x1000, this);
    this->add_mmio_region(UNI_N_INTERNAL_CONFIG_DATA, 0x1000, this);

    // select the PCI host to use for each configuration window
    this->hosts[UNI_N_WINDOW_AGP]      = &this->agp_host;
    this->hosts[UNI_N_WINDOW_MAIN]     = this;
    this->hosts[UNI_N_WINDOW_INTERNAL] = &this->internal_host;

    // PCI devices that represent the AGP and internal host bridges on
    // their respective buses.
    this->agp_dev = std::unique_ptr<IntrepidPciHostDevice>(
        new IntrepidPciHostDevice("UniNorth AGP", 0x0018));
    this->internal_dev = std::unique_ptr<IntrepidPciHostDevice>(
        new IntrepidPciHostDevice("UniNorth Internal PCI", 0x001E));
}

int Intrepid::device_postinit()
{
    // register the three host bridges at device 11 of their own buses,
    // as seen on real UniNorth G4 hardware
    this->pci_register_device(DEV_FUN(0x0B, 0), this);
    this->agp_host.pci_register_device(DEV_FUN(0x0B, 0), this->agp_dev.get());
    this->internal_host.pci_register_device(DEV_FUN(0x0B, 0), this->internal_dev.get());

    this->pcihost_device_postinit();
    this->agp_host.pcihost_device_postinit();
    this->internal_host.pcihost_device_postinit();

    return 0;
}

void Intrepid::setup_ram(int capacity_megs)
{
    if (capacity_megs) {
        uint64_t ram_size = (uint64_t)capacity_megs << 20;
        if (ram_size > 0xFFFFFFFFULL) {
            LOG_F(ERROR, "Intrepid: requested RAM size %d MB too large", capacity_megs);
            return;
        }
        if (!this->add_ram_region(0, (uint32_t)ram_size)) {
            LOG_F(WARNING, "Intrepid: %d MB RAM allocation failed (maybe already exists?)",
                capacity_megs);
        }
    }
}

uint32_t Intrepid::read(uint32_t rgn_start, uint32_t offset, int size)
{
    switch (rgn_start) {
    case UNI_N_REGISTER_BLOCK:
        return this->read_unin_register(offset, size);

    case UNI_N_I2C_BLOCK:
        return this->read_i2c_register(offset, size);

    case UNI_N_CLK_BLOCK:
        return this->read_clk_register(offset, size);

    case UNI_N_CONFIG_WINDOW:
        return this->read_config_window(offset, size);

    case UNI_N_AGP_CONFIG_ADDR:
        return this->config_addr[UNI_N_WINDOW_AGP];

    case UNI_N_AGP_CONFIG_DATA:
        return this->config_read(UNI_N_WINDOW_AGP, offset, size);

    case UNI_N_MAIN_IO_SPACE:
        // PCI I/O space
        return pci_io_read_broadcast(offset, size);

    case UNI_N_MAIN_CONFIG_ADDR:
        return this->config_addr[UNI_N_WINDOW_MAIN];

    case UNI_N_MAIN_CONFIG_DATA:
        return this->config_read(UNI_N_WINDOW_MAIN, offset, size);

    case UNI_N_INTERNAL_CONFIG_ADDR:
        return this->config_addr[UNI_N_WINDOW_INTERNAL];

    case UNI_N_INTERNAL_CONFIG_DATA:
        return this->config_read(UNI_N_WINDOW_INTERNAL, offset, size);

    default:
        LOG_F(ERROR, "%s: read from unknown MMIO region 0x%08x @0x%08x.%c",
            this->name.c_str(), rgn_start, offset, SIZE_ARG(size));
    }

    return 0;
}

void Intrepid::write(uint32_t rgn_start, uint32_t offset, uint32_t value, int size)
{
    switch (rgn_start) {
    case UNI_N_REGISTER_BLOCK:
        this->write_unin_register(offset, value, size);
        return;

    case UNI_N_I2C_BLOCK:
        this->write_i2c_register(offset, value, size);
        return;

    case UNI_N_CLK_BLOCK:
        this->write_clk_register(offset, value, size);
        return;

    case UNI_N_CONFIG_WINDOW:
        this->write_config_window(offset, value, size);
        return;

    case UNI_N_AGP_CONFIG_ADDR:
        this->config_addr[UNI_N_WINDOW_AGP] = value;
        return;

    case UNI_N_AGP_CONFIG_DATA:
        this->config_write(UNI_N_WINDOW_AGP, offset, size, value);
        return;

    case UNI_N_MAIN_IO_SPACE:
        // PCI I/O space
        pci_io_write_broadcast(offset, size, value);
        return;

    case UNI_N_MAIN_CONFIG_ADDR:
        this->config_addr[UNI_N_WINDOW_MAIN] = value;
        return;

    case UNI_N_MAIN_CONFIG_DATA:
        this->config_write(UNI_N_WINDOW_MAIN, offset, size, value);
        return;

    case UNI_N_INTERNAL_CONFIG_ADDR:
        this->config_addr[UNI_N_WINDOW_INTERNAL] = value;
        return;

    case UNI_N_INTERNAL_CONFIG_DATA:
        this->config_write(UNI_N_WINDOW_INTERNAL, offset, value, size);
        return;

    default:
        LOG_F(ERROR, "%s: write to unknown MMIO region 0x%08x @0x%08x.%c = 0x%0*x",
            this->name.c_str(), rgn_start, offset, SIZE_ARG(size), size * 2,
            BYTESWAP_SIZED(value, size));
    }
}

uint32_t Intrepid::read_unin_register(uint32_t offset, int size)
{
    switch (offset & 0xFFF) {
    case UNI_N_VERSION:
        // Intrepid revision 0x00D2, as found in the Mac mini G4
        return 0x00D2;
    case UNI_N_VERSION + 3:
        // The boot ROM reads the version as a byte at offset 3, which is
        // big-endian lane 3 of the word at offset 0 (memory probe 0xfff88cac,
        // table scan 0xfff883e4, I2C bus select 0xfff8b4e8).
        return 0xD2;
    case UNI_N_CLOCK_CNTL:
        return this->clock_cntl;
    case UNI_N_POWER_MGT:
        return this->power_mgt;
    case UNI_N_ARB_CTRL:
        return this->arb_ctrl;
    case UNI_N_CPU_NUMBER:
        // CPU 0 reads zero, which tells the bootROM it is the boot CPU
        return 0;
    case UNI_N_HWINIT_STATE:
        return this->hwinit_state;
    case UNI_N_AACK_DELAY:
        return this->aack_delay;
    default:
        return 0;
    }
}

void Intrepid::write_unin_register(uint32_t offset, uint32_t value, int size)
{
    switch (offset & 0xFFF) {
    case UNI_N_CLOCK_CNTL:
        this->clock_cntl = value;
        return;
    case UNI_N_POWER_MGT:
        this->power_mgt = value;
        return;
    case UNI_N_ARB_CTRL:
        this->arb_ctrl = value;
        return;
    case UNI_N_AACK_DELAY:
        this->aack_delay = value;
        return;
    case UNI_N_HWINIT_STATE:
        this->hwinit_state = value;
        return;
    default:
        LOG_F(WARNING, "%s: write to unimplemented register 0x%03x.%c = 0x%0*x",
            this->name.c_str(), offset, SIZE_ARG(size), size * 2,
            BYTESWAP_SIZED(value, size));
    }
}

/* UniNorth I2C controller (0xF8001000 block).

   The boot ROM's I2C byte routines (0xfff8b51c read / 0xfff8b5b4 write /
   0xfff8b8d0 reset) poll the STATUS register and abort to an error path
   when STATUS_ACK bit 1 is clear. STATUS reads return all four "phase done"
   bits (0x0F) and STATUS_ACK reads return bit 1 (0x02), which lets every
   "wait until bit set" poll exit immediately. The only device attached is
   the DIMM SPD EEPROM at 7-bit address 0x50 (0xA0/0xA1); its content is
   served from the data register on reads. */
uint32_t Intrepid::read_i2c_register(uint32_t offset, int size)
{
    switch (offset) {
    case UNI_N_I2C_STATUS_ACK:
        return 0x02; // last transfer acknowledged
    case UNI_N_I2C_STATUS:
        return 0x0F; // address/data/stop phases all complete
    case UNI_N_I2C_MODE:
        return this->i2c_mode;
    case UNI_N_I2C_CTRL:
        return this->i2c_ctrl;
    case UNI_N_I2C_IER:
        return this->i2c_ier;
    case UNI_N_I2C_ADDR:
        return this->i2c_addr;
    case UNI_N_I2C_SUBADDR:
        return this->i2c_subaddr;
    case UNI_N_I2C_DATA:
        // The DIMM's SPD EEPROM sits at 7-bit address 0x50 (8-bit write
        // address 0xA0, read address 0xA1) on this bus. A data read after
        // the boot ROM's read transfer (0xfff8b51c) returns the selected
        // SPD byte.
        if ((this->i2c_addr & 0xFE) == 0xA0 && this->i2c_subaddr < 0x80)
            return spd_dimm_info[this->i2c_subaddr];
        return this->i2c_data;
    case UNI_N_I2C_SSADDR:
        return this->i2c_ssaddr;
    case UNI_N_I2C_ADDR_CTL:
        return this->i2c_addr_ctl;
    case UNI_N_I2C_DATA2:
        return this->i2c_data2;
    default:
        return 0;
    }
}

void Intrepid::write_i2c_register(uint32_t offset, uint32_t value, int size)
{
    uint8_t byte = value & 0xFF;

    switch (offset) {
    case UNI_N_I2C_STATUS_ACK:
    case UNI_N_I2C_STATUS:
        // status/ISR writes are transfer-control commands; no devices on
        // the bus, so ignore them
        return;
    case UNI_N_I2C_MODE:
        this->i2c_mode = byte;
        return;
    case UNI_N_I2C_CTRL:
        this->i2c_ctrl = byte;
        return;
    case UNI_N_I2C_IER:
        this->i2c_ier = byte;
        return;
    case UNI_N_I2C_ADDR:
        this->i2c_addr = byte;
        return;
    case UNI_N_I2C_SUBADDR:
        this->i2c_subaddr = byte;
        return;
    case UNI_N_I2C_DATA:
        this->i2c_data = byte;
        return;
    case UNI_N_I2C_SSADDR:
        this->i2c_ssaddr = byte;
        return;
    case UNI_N_I2C_ADDR_CTL:
        this->i2c_addr_ctl = value;
        return;
    case UNI_N_I2C_DATA2:
        this->i2c_data2 = value;
        return;
    default:
        LOG_F(9, "%s: i2c register write @0x%03x.%c = 0x%0*x",
            this->name.c_str(), offset, SIZE_ARG(size), size * 2,
            BYTESWAP_SIZED(value, size));
    }
}

/* Memory controller clock registers (0xF8002000 block).

   The boot ROM writes the SUN I2C mode bytes (0xF80021C0 + 0x40*n) and the
   clock control registers (0xF8002080, 0xF8002180), and read-modify-writes
   the CPU clock PLL register 0xF8002500. All but 0xF8002500 are write-only
   from the ROM's point of view. */
uint32_t Intrepid::read_clk_register(uint32_t offset, int size)
{
    if (offset == UNI_N_CLK_REG) {
        if (size == 1)
            return (this->clk_reg >> 24) & 0xFF; // big-endian byte lane 0
        return this->clk_reg;
    }
    return 0;
}

void Intrepid::write_clk_register(uint32_t offset, uint32_t value, int size)
{
    if (offset == UNI_N_CLK_REG) {
        if (size == 1)
            this->clk_reg = (this->clk_reg & 0x00FFFFFF) |
                            ((value & 0xFF) << 24);
        else
            this->clk_reg = value;
        return;
    }
    LOG_F(9, "%s: clock register write @0x%03x.%c = 0x%0*x",
        this->name.c_str(), offset, SIZE_ARG(size), size * 2,
        BYTESWAP_SIZED(value, size));
}

/* Boot ROM config window at 0x80008000 / 0x80008800.

   The Mac mini G4 boot ROM's early PCI quiesce code drives this window
   (via stwbrx/lwbrx, so register values arrive byte-swapped) as follows:

       [base+0x00] <- 0xFFFF0000       reset / set up the window
       [base+0x14] <- 0x00010001
       [base+0x0C] <- descriptor table pointer (in ROM)
       [base+0x00] <- 0x80008000       window base select
       [base+0x00] <- 0x00010001       GO: execute the descriptor table
       poll [base+0x04] bit 0x400      cleared when execution completes

   The descriptor tables (0xFFF02DB0 fast / 0xFFF02E20 slow) describe
   memory-controller calibration curves which our flat RAM emulation does
   not need; we only report the "done" state so the poll terminates. */
uint32_t Intrepid::read_config_window(uint32_t offset, int size)
{
    uint32_t reg = offset & 0x1F;

    switch (reg) {
    case 0x04: // status: bit 0x400 = busy, clear when the init completes
        return 0;
    default:
        return 0;
    }
}

void Intrepid::write_config_window(uint32_t offset, uint32_t value, int size)
{
    int window = (offset >> 11) & 1; // 0 = 0x80008000, 1 = 0x80008800
    uint32_t reg = offset & 0x1F;

    switch (reg) {
    case 0x00: // command register (values arrive byte-swapped from stwbrx)
        if (BYTESWAP_32(value) == 0x00010001) { // GO command
            LOG_F(INFO, "%s: config window %d GO, descriptor table @0x%08X",
                this->name.c_str(), window, this->config_window_ptr[window]);
        }
        break;
    case 0x0C: // descriptor table pointer
        this->config_window_ptr[window] = BYTESWAP_32(value);
        LOG_F(INFO, "%s: config window %d descriptor table @0x%08X",
            this->name.c_str(), window, this->config_window_ptr[window]);
        break;
    case 0x14: // secondary command
        break;
    default:
        LOG_F(WARNING, "%s: write to config window register 0x%03x.%c = 0x%0*x",
            this->name.c_str(), offset, SIZE_ARG(size), size * 2,
            BYTESWAP_SIZED(value, size));
    }
}

/* Translate a UniNorth config address into the standard PCI format.
   This implements the same logic as QEMU's unin_get_config_reg():

   - bit 31 set:      standard PCI format passed through (also produced by
                      the Apple MacRISCI CFA0 scheme)
   - bit 0 set:       standard PCI type 1 format (Apple MacRISCI CFA1)
   - otherwise:       Apple IDSEL-based scheme used by Open Firmware
   The byte lane within the config data port (addr & 7) is folded into
   the low bits of the result. */
static uint32_t unin_get_config_reg(uint32_t reg, uint32_t addr)
{
    uint32_t retval;

    if (reg & (1u << 31)) {
        retval = reg | (addr & 3);
    } else if (reg & 1) {
        retval = (reg & ~7u) | (addr & 7);
    } else {
        uint32_t slot = ctz32(reg & 0xFFFFF800U);
        if (slot == 32)
            slot = 0;
        uint32_t func = (reg >> 8) & 7;
        retval = (reg & (0xff - 7)) | (addr & 7);
        retval |= slot << 11;
        retval |= func << 8;
    }

    return retval;
}

uint32_t Intrepid::config_read(int window, uint32_t offset, int size)
{
    PCIHost *host = this->hosts[window];
    uint32_t config_addr = this->config_addr[window];
    uint32_t config_addr2 = unin_get_config_reg(config_addr, offset & 7);

    int bus_num = (config_addr2 >> 16) & 0xFF;
    int dev_num = (config_addr2 >> 11) & 0x1F;
    int fun_num = (config_addr2 >> 8) & 0x07;
    uint8_t reg_offs = config_addr2 & 0xFC;

    AccessDetails details;
    details.size = size;
    details.offset = config_addr2 & 3;
    details.flags = PCI_CONFIG_READ |
        (bus_num ? PCI_CONFIG_TYPE_1 : PCI_CONFIG_TYPE_0);

    PCIBase *device = bus_num ? host->pci_find_device(bus_num, dev_num, fun_num)
                              : host->pci_find_device(dev_num, fun_num);
    if (device) {
        uint32_t value = device->pci_cfg_read(reg_offs, details);
        // bytes 0 to 3 repeat
        return pci_conv_rd_data(value, value, details);
    }

    LOG_READ_NON_EXISTENT_PCI_DEVICE();
    return 0xFFFFFFFFUL; // PCI spec §6.1
}

void Intrepid::config_write(int window, uint32_t offset, uint32_t value, int size)
{
    PCIHost *host = this->hosts[window];
    uint32_t config_addr = this->config_addr[window];
    uint32_t config_addr2 = unin_get_config_reg(config_addr, offset & 7);

    int bus_num = (config_addr2 >> 16) & 0xFF;
    int dev_num = (config_addr2 >> 11) & 0x1F;
    int fun_num = (config_addr2 >> 8) & 0x07;
    uint8_t reg_offs = config_addr2 & 0xFC;

    AccessDetails details;
    details.size = size;
    details.offset = config_addr2 & 3;
    details.flags = PCI_CONFIG_WRITE |
        (bus_num ? PCI_CONFIG_TYPE_1 : PCI_CONFIG_TYPE_0);

    PCIBase *device = bus_num ? host->pci_find_device(bus_num, dev_num, fun_num)
                              : host->pci_find_device(dev_num, fun_num);
    if (device) {
        if (size == 4 && !details.offset) { // aligned DWORD writes -> fast path
            device->pci_cfg_write(reg_offs, BYTESWAP_32(value), details);
            return;
        }
        // otherwise perform necessary data transformations -> slow path
        uint32_t old_val = details.size == 4 ? 0 : device->pci_cfg_read(reg_offs, details);
        uint32_t new_val = pci_conv_wr_data(old_val, value, details);
        device->pci_cfg_write(reg_offs, new_val, details);
        return;
    }
    LOG_WRITE_NON_EXISTENT_PCI_DEVICE();
}

uint32_t Intrepid::pci_cfg_read(uint32_t reg_offs, AccessDetails &details)
{
    if (reg_offs < 64) {
        return PCIDevice::pci_cfg_read(reg_offs, details);
    }

    switch (reg_offs) {
    case 0x48: // kMacRISCPCIAddressSelect
        return this->macrisc_addr_select;
    default:
        LOG_READ_UNIMPLEMENTED_CONFIG_REGISTER();
    }

    return 0; // PCI Spec §6.1
}

void Intrepid::pci_cfg_write(uint32_t reg_offs, uint32_t value, AccessDetails &details)
{
    if (reg_offs < 64) {
        PCIDevice::pci_cfg_write(reg_offs, value, details);
        return;
    }

    switch (reg_offs) {
    case 0x48: // kMacRISCPCIAddressSelect
        this->macrisc_addr_select = value;
        return;
    default:
        LOG_WRITE_UNIMPLEMENTED_CONFIG_REGISTER();
    }
}

static const PropMap Intrepid_Properties = {
    {"pci_A1",
        new StrProperty("")},
    {"pci_B1",
        new StrProperty("")},
    {"pci_C1",
        new StrProperty("")},
    {"pci_D1",
        new StrProperty("")},
    {"pci_E1",
        new StrProperty("")},
    {"pci_F1",
        new StrProperty("")},
    {"pci_G1",
        new StrProperty("")},
};

static const DeviceDescription Intrepid_Descriptor = {
    Intrepid::create, {}, Intrepid_Properties,
    HWCompType::MEM_CTRL | HWCompType::MMIO_DEV | HWCompType::PCI_HOST | HWCompType::PCI_DEV
};

REGISTER_DEVICE(Intrepid, Intrepid_Descriptor);
