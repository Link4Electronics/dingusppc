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

#include <core/bitops.h>
#include <core/endianswap.h>
#include <core/memaccess.h>
#include <devices/common/hwcomponent.h>
#include <devices/common/pci/pcidevice.h>
#include <devices/deviceregistry.h>
#include <devices/video/atiradeon.h>
#include <loguru.hpp>

#include <cstring>
#include <map>

/* Radeon reference clock (crystal) frequency in Hz. */
#define RADEON_XTAL_CLK (27000000.0f)

/* Host-side constant for how many dwords of 2D FIFO we pretend to have. */
#define RADEON_CMDFIFO_ENTRIES 64

static bool radeon_clip_axis(int coord, int increment, uint32_t length,
                             int scissor_min, int scissor_max,
                             int& skip, uint32_t& clipped_length) {
    int first = increment > 0 ? scissor_min - coord : coord - scissor_max;
    int last  = increment > 0 ? scissor_max - coord : coord - scissor_min;

    first = std::max(first, 0);
    last  = std::min(last, int(length) - 1);
    if (first > last) {
        return false;
    }

    skip = first;
    clipped_length = last - first + 1;
    return true;
}

/* Human readable Radeon HW register names for easier debugging. */
static const std::map<uint32_t, std::string> radeon_reg_names = {
    #define one_reg_name(x) {RADEON_ ## x, #x}
    one_reg_name(MM_INDEX),
    one_reg_name(MM_DATA),
    one_reg_name(CLOCK_CNTL_INDEX),
    one_reg_name(CLOCK_CNTL_DATA),
    one_reg_name(GEN_INT_CNTL),
    one_reg_name(GEN_INT_STATUS),
    one_reg_name(CRTC_GEN_CNTL),
    one_reg_name(CRTC_EXT_CNTL),
    one_reg_name(DAC_CNTL),
    one_reg_name(GPIO_VGA_DDC),
    one_reg_name(GPIO_DVI_DDC),
    one_reg_name(GPIO_MONID),
    one_reg_name(PALETTE_INDEX),
    one_reg_name(PALETTE_DATA),
    one_reg_name(CNFG_CNTL),
    one_reg_name(GEN_RESET_CNTL),
    one_reg_name(CNFG_MEMSIZE),
    one_reg_name(CONFIG_APER_0_BASE),
    one_reg_name(CONFIG_APER_SIZE),
    one_reg_name(CONFIG_REG_1_BASE),
    one_reg_name(CONFIG_REG_APER_SIZE),
    one_reg_name(HOST_PATH_CNTL),
    one_reg_name(MC_STATUS),
    one_reg_name(CRTC_H_TOTAL_DISP),
    one_reg_name(CRTC_H_SYNC_STRT_WID),
    one_reg_name(CRTC_V_TOTAL_DISP),
    one_reg_name(CRTC_V_SYNC_STRT_WID),
    one_reg_name(CRTC_VLINE_CRNT_VLINE),
    one_reg_name(CRTC_OFFSET),
    one_reg_name(CRTC_PITCH),
    one_reg_name(CUR_OFFSET),
    one_reg_name(CUR_HORZ_VERT_POSN),
    one_reg_name(CUR_HORZ_VERT_OFF),
    one_reg_name(CUR_CLR0),
    one_reg_name(CUR_CLR1),
    one_reg_name(RBBM_STATUS),
    one_reg_name(DST_OFFSET),
    one_reg_name(DST_PITCH),
    one_reg_name(DST_WIDTH),
    one_reg_name(DST_HEIGHT),
    one_reg_name(SRC_X),
    one_reg_name(SRC_Y),
    one_reg_name(DST_X),
    one_reg_name(DST_Y),
    one_reg_name(SRC_PITCH_OFFSET),
    one_reg_name(DST_PITCH_OFFSET),
    one_reg_name(SRC_Y_X),
    one_reg_name(DST_Y_X),
    one_reg_name(DST_HEIGHT_WIDTH),
    one_reg_name(DP_GUI_MASTER_CNTL),
    one_reg_name(DP_BRUSH_BKGD_CLR),
    one_reg_name(DP_BRUSH_FRGD_CLR),
    one_reg_name(DST_WIDTH_X),
    one_reg_name(DST_HEIGHT_WIDTH_8),
    one_reg_name(SRC_X_Y),
    one_reg_name(DST_X_Y),
    one_reg_name(DST_WIDTH_HEIGHT),
    one_reg_name(DST_WIDTH_X_INCY),
    one_reg_name(DST_HEIGHT_Y),
    one_reg_name(SRC_OFFSET),
    one_reg_name(SRC_PITCH),
    one_reg_name(DST_HEIGHT_WIDTH_BW),
    one_reg_name(CLR_CMP_CNTL),
    one_reg_name(DP_SRC_FRGD_CLR),
    one_reg_name(DP_SRC_BKGD_CLR),
    one_reg_name(SC_LEFT),
    one_reg_name(SC_RIGHT),
    one_reg_name(SC_TOP),
    one_reg_name(SC_BOTTOM),
    one_reg_name(SRC_SC_RIGHT),
    one_reg_name(SRC_SC_BOTTOM),
    one_reg_name(DP_CNTL),
    one_reg_name(DP_DATATYPE),
    one_reg_name(DP_MIX),
    one_reg_name(DP_WRITE_MASK),
    one_reg_name(DEFAULT_PITCH_OFFSET),
    one_reg_name(DEFAULT_SC_BOTTOM_RIGHT),
    one_reg_name(SC_TOP_LEFT),
    one_reg_name(SC_BOTTOM_RIGHT),
    one_reg_name(SRC_SC_BOTTOM_RIGHT),
    one_reg_name(DST_TILE),
    one_reg_name(WAIT_UNTIL),
    one_reg_name(CACHE_CNTL),
    one_reg_name(GUI_STAT),
    one_reg_name(PC_GUI_MODE),
    one_reg_name(PC_GUI_CTLSTAT),
    one_reg_name(HOST_DATA0),
    one_reg_name(HOST_DATA_LAST),
};

ATIRadeon::ATIRadeon(uint16_t dev_id)
    : PCIDevice("ati-radeon"), VideoCtrlBase()
{
    uint8_t asic_id;

    supports_types(HWCompType::MMIO_DEV | HWCompType::PCI_DEV);

    this->vram_size = GET_INT_PROP("gfxmem_size") << 20; // convert MBs to bytes
    this->framebuffer_size = std::min(this->vram_size, 0x4000000U);

    // allocate video RAM
    this->vram_ptr = std::unique_ptr<uint8_t[]>(new uint8_t[this->vram_size]);

    switch (dev_id) {
    case ATI_RADEON_RV280_DEV_ID:
        asic_id = 0x81; // RV280 (Radeon 9200)
        this->cmd_fifo_size = RADEON_CMDFIFO_ENTRIES;
        break;
    default:
        asic_id = 0xDD;
        LOG_F(WARNING, "ATI Radeon: bogus ASIC ID assigned!");
    }

    // set up PCI configuration space header
    this->vendor_id   = PCI_VENDOR_ATI;
    this->device_id   = dev_id;
    this->subsys_vndr = PCI_VENDOR_ATI;
    this->subsys_id   = 0x5962; // adapter ID (matches the real mini GPU)
    this->class_rev   = (0x030000 << 8) | 0x01;
    this->min_gnt     = 8;
    this->irq_pin     = 1;
    this->cap_ptr     = 0x50; // PM capability, then AGP capability
    this->command     = 0x0007; // I/O + memory + bus master
    for (int i = 0; i < this->aperture_count; i++) {
        this->bars_cfg[i] = (uint32_t)(-this->aperture_size[i] | this->aperture_flag[i]);
    }
    // expansion ROM is present (128KB) but the Mac OS never enables it
    // because the display driver lives in the machine's boot ROM instead.
    this->exp_bar_cfg  = ~(0x20000 - 1);
    this->finish_config_bars();

    this->pci_notify_bar_change = [this](int bar_num) {
        this->notify_bar_change(bar_num);
    };

    // stuff default values into chip registers
    this->regs[RADEON_CNFG_MEMSIZE >> 2] = this->vram_size;
    this->regs[RADEON_CONFIG_APER_0_BASE >> 2] = this->aperture_size[0];
    this->regs[RADEON_CONFIG_APER_SIZE >> 2] = this->aperture_size[0];
    this->regs[RADEON_CONFIG_REG_1_BASE >> 2] = this->aperture_size[2];
    this->regs[RADEON_CONFIG_REG_APER_SIZE >> 2] = this->aperture_size[2];
    this->regs[RADEON_MC_STATUS >> 2] = 5;

    set_bit(regs[RADEON_CRTC_GEN_CNTL >> 2], 23); // CRTC_DISPLAY_DIS (blank)
    insert_bits<uint32_t>(this->regs[RADEON_GUI_STAT >> 2], 32, 0, 8);

    // initialize display identification
    this->disp_id = std::unique_ptr<DisplayID> (new DisplayID());

    uint8_t mon_code = this->disp_id->read_monitor_sense(0, 0);
    this->regs[RADEON_GPIO_MONID >> 2] = ((mon_code & 6) << 11) | ((mon_code & 1) << 8);
    this->regs[RADEON_GPIO_VGA_DDC >> 2] = ((mon_code & 6) << 11) | ((mon_code & 1) << 8);
    this->regs[RADEON_GPIO_DVI_DDC >> 2] = ((mon_code & 6) << 11) | ((mon_code & 1) << 8);

    this->draw_fb_is_dynamic = true;
}

void ATIRadeon::change_one_bar(uint32_t &aperture, uint32_t aperture_size,
                               uint32_t aperture_new, int bar_num) {
    if (aperture != aperture_new) {
        if (aperture)
            this->host_instance->pci_unregister_mmio_region(aperture,
                                                            aperture_size, this);

        aperture = aperture_new;
        if (aperture)
            this->host_instance->pci_register_mmio_region(aperture, aperture_size, this);

        LOG_F(INFO, "%s: aperture[%d] set to 0x%08X", this->name.c_str(),
              bar_num, aperture);
    }
}

void ATIRadeon::notify_bar_change(int bar_num)
{
    switch (bar_num) {
    case 0:
        change_one_bar(this->aperture_base[bar_num],
                       this->aperture_size[bar_num],
                       this->bars[bar_num] & ~15, bar_num);
        break;
    case 2:
        change_one_bar(this->aperture_base[bar_num],
                       this->aperture_size[bar_num],
                       this->bars[bar_num] & ~15, bar_num);
        break;
    case 1:
        this->aperture_base[1] = this->bars[bar_num] & ~3;
        LOG_F(INFO, "%s: I/O space address set to 0x%08X", this->name.c_str(),
              this->aperture_base[1]);
        break;
    }
}

uint32_t ATIRadeon::pci_cfg_read(uint32_t reg_offs, AccessDetails &details)
{
    if (reg_offs < 64) {
        return PCIDevice::pci_cfg_read(reg_offs, details);
    }

    switch (reg_offs) {
    case 0x40:
        return this->user_cfg;
    case 0x50:
        // PM capability, version 1.1, next cap at 0x58
        return (0x02 << 16) | (0x58 << 8) | 0x01;
    case 0x54:
        return 0; // PMCSR_BSE
    case 0x58:
        // AGP capability, version 2.0, last cap in the list
        return (0x02 << 16) | (0x00 << 8) | 0x02;
    case 0x5c:
        // AGP status: rates x1/x2/x4, SBA, fast-writes
        return 0x0000000e | (1 << 9) | (1 << 10);
    case 0x60:
        return 0; // AGP command
    default:
        LOG_READ_UNIMPLEMENTED_CONFIG_REGISTER();
    }

    return 0;
}

void ATIRadeon::pci_cfg_write(uint32_t reg_offs, uint32_t value, AccessDetails &details)
{
    if (reg_offs < 64) {
        PCIDevice::pci_cfg_write(reg_offs, value, details);
        return;
    }

    switch (reg_offs) {
    case 0x40:
        this->user_cfg = value;
        break;
    case 0x54: // PMCSR writes accepted, only D0 power state is supported
    case 0x60: // AGP command writes accepted, AGP stays disabled
        break;
    default:
        LOG_WRITE_UNIMPLEMENTED_CONFIG_REGISTER();
    }
}

const char* ATIRadeon::get_reg_name(uint32_t reg_num) {
    auto iter = radeon_reg_names.find(reg_num << 2);
    if (iter != radeon_reg_names.end()) {
        return iter->second.c_str();
    } else {
        return "unknown Radeon register";
    }
}

uint32_t ATIRadeon::read_reg(uint32_t reg_offset, uint32_t size) {
    uint32_t reg_num = reg_offset >> 2;
    uint32_t offset = reg_offset & 3;
    uint64_t result = 0;

    switch (reg_offset) {
    case RADEON_MM_DATA:
    case RADEON_MM_DATA + 1:
    case RADEON_MM_DATA + 2:
    case RADEON_MM_DATA + 3:
        // indexed access to registers or VRAM
        if (this->regs[RADEON_MM_INDEX >> 2] & 0x80000000) {
            uint32_t idx = (this->regs[RADEON_MM_INDEX >> 2] & ~0x80000000) +
                           (reg_offset - RADEON_MM_DATA);
            if (idx < this->vram_size - size)
                return read_mem(&this->vram_ptr[idx], size);
            return 0;
        }
        if (this->regs[RADEON_MM_INDEX >> 2] > 7) {
            return this->read_reg(this->regs[RADEON_MM_INDEX >> 2] +
                                  (reg_offset - RADEON_MM_DATA), size);
        }
        return 0;
    case RADEON_CLOCK_CNTL_DATA:
        result = this->plls[this->regs[RADEON_CLOCK_CNTL_INDEX >> 2] & 0x3f];
        break;
    case RADEON_CNFG_MEMSIZE:
        result = this->vram_size;
        break;
    case RADEON_CONFIG_APER_0_BASE:
    case RADEON_CONFIG_APER_1_BASE:
        result = this->aperture_base[0] & ~0xf;
        break;
    case RADEON_CONFIG_APER_SIZE:
        result = this->aperture_size[0] / 2;
        break;
    case RADEON_CONFIG_REG_1_BASE:
        result = this->aperture_base[2] & ~0xf;
        break;
    case RADEON_CONFIG_REG_APER_SIZE:
        result = this->aperture_size[2] / 2;
        break;
    case RADEON_HOST_PATH_CNTL:
        result = 1 << 23; // Radeon HDP_APER_CNTL
        break;
    case RADEON_MC_STATUS:
        result = 5; // memory controller initialized
        break;
    case RADEON_MEM_SDRAM_MODE_REG:
        result = (1 << 28) | (1 << 20);
        break;
    case RADEON_RBBM_STATUS:
    case RADEON_GUI_STAT:
        result = RADEON_CMDFIFO_ENTRIES; // free CMDFIFO entries
        break;
    case RADEON_CRTC_VLINE_CRNT_VLINE:
        result = 0;
        insert_bits<uint64_t>(result, this->regs[reg_num], 0, 12);
        break;
    case RADEON_PALETTE_INDEX:
        result = this->regs[reg_num];
        insert_bits<uint64_t>(result, this->dac_wr_index, 0, 8);
        insert_bits<uint64_t>(result, this->dac_rd_index, 16, 8);
        break;
    case RADEON_PALETTE_DATA:
        if (!this->comp_index) {
            uint8_t alpha;
            get_palette_color(this->dac_rd_index, color_buf[0],
                              color_buf[1], color_buf[2], alpha);
        }
        insert_bits<uint64_t>(result, color_buf[this->comp_index], 8, 8);
        if (++this->comp_index >= 3) {
            this->dac_rd_index++; // auto-increment reading index
            this->comp_index = 0; // reset color component index
        }
        break;
    case RADEON_DP_GUI_MASTER_CNTL:
        result = this->dp_gui_master_cntl |
                 ((this->dp_datatype & RADEON_DP_BRUSH_DATATYPE) >> 4) |
                 ((this->dp_datatype & RADEON_DP_DST_DATATYPE) << 8) |
                 ((this->dp_datatype & RADEON_DP_SRC_DATATYPE) >> 4) |
                 (this->dp_mix & RADEON_DP_ROP3) |
                 ((this->dp_mix & RADEON_DP_SRC_SOURCE) << 16);
        break;
    default:
        if (reg_offset >= 0xf00 && reg_offset < 0x1000) {
            // read-only mirror of the PCI configuration space
            AccessDetails cfg_details = { uint8_t(size), uint8_t(reg_offset & 3), 0 };
            return this->pci_cfg_read(reg_offset - 0xf00, cfg_details);
        }
        if (reg_offset < sizeof(this->regs))
            result = this->regs[reg_num];
        else
            return 0;
        break;
    }

    if (offset || size != 4) { // slow path
        if ((offset + size) > 4) {
            if (reg_offset < sizeof(this->regs))
                result |= (uint64_t)(this->regs[reg_num + 1]) << 32;
        }
        result = extract_bits<uint64_t>(result, offset * 8, size * 8);
    }

    return static_cast<uint32_t>(result);
}

void ATIRadeon::write_reg(uint32_t reg_offset, uint32_t value, uint32_t size) {
    uint32_t reg_num = reg_offset >> 2;
    uint32_t offset = reg_offset & 3;
    uint32_t old_value = this->regs[reg_num];
    uint32_t new_value;

    if (reg_offset >= RADEON_HOST_DATA0 && reg_offset <= RADEON_HOST_DATA_LAST) {
        this->write_host_data(value, size);
        return;
    }

    if (offset || size != 4) { // slow path
        if ((offset + size) > 4) {
            ABORT_F("%s: unaligned DWORD writes not implemented", this->name.c_str());
        }
        uint64_t val = old_value;
        insert_bits<uint64_t>(val, value, offset * 8, size * 8);
        value = static_cast<uint32_t>(val);
    }

    switch (reg_offset) {
    case RADEON_MM_INDEX:
        this->regs[reg_num] = value & ~3;
        return;
    case RADEON_MM_DATA:
    case RADEON_MM_DATA + 1:
    case RADEON_MM_DATA + 2:
    case RADEON_MM_DATA + 3:
        // indexed access to registers or VRAM
        if (this->regs[RADEON_MM_INDEX >> 2] & 0x80000000) {
            uint32_t idx = (this->regs[RADEON_MM_INDEX >> 2] & ~0x80000000) +
                           (reg_offset - RADEON_MM_DATA);
            if (idx < this->vram_size) {
                draw_fb = true;
                write_mem(&this->vram_ptr[idx], value, size);
            }
            return;
        }
        if (this->regs[RADEON_MM_INDEX >> 2] > 7) {
            this->write_reg(this->regs[RADEON_MM_INDEX >> 2] +
                            (reg_offset - RADEON_MM_DATA), value, size);
        }
        return;
    case RADEON_CLOCK_CNTL_INDEX:
        this->regs[reg_num] = value;
        return;
    case RADEON_CLOCK_CNTL_DATA:
        if (this->regs[RADEON_CLOCK_CNTL_INDEX >> 2] & RADEON_PLL_WR_EN) {
            uint8_t pll_addr = this->regs[RADEON_CLOCK_CNTL_INDEX >> 2] & 0x3f;
            this->plls[pll_addr] = value & 0xff;
            LOG_F(9, "%s: PLL #%d set to 0x%02X", this->name.c_str(), pll_addr, value & 0xff);
            this->crtc_update();
        }
        return;
    case RADEON_GEN_INT_CNTL:
        this->regs[reg_num] = value;
        this->update_interrupt();
        return;
    case RADEON_GEN_INT_STATUS:
        this->regs[reg_num] &= ~(value & 0xfc080eff);
        this->update_interrupt();
        return;
    case RADEON_CRTC_GEN_CNTL:
        new_value = value;
        if (bit_changed(old_value, new_value, 23)) { // CRTC_DISPLAY_DIS
            if (bit_set(new_value, 23)) {
                this->blank_on = true;
                this->blank_display();
            } else {
                this->blank_on = false;
            }
        }
        if (bit_changed(old_value, new_value, 25) || // CRTC_EN
            extract_bits<uint32_t>(old_value, 8, 3) !=
            extract_bits<uint32_t>(new_value, 8, 3)) {
            this->draw_fb = true;
            if (bit_set(new_value, 25) && !bit_set(new_value, 23)) {
                this->crtc_update();
            }
        }
        this->regs[reg_num] = new_value;
        return;
    case RADEON_CRTC_EXT_CNTL:
        new_value = value;
        if (bit_changed(old_value, new_value, 10)) { // CRT_CRTC_DISPLAY_DIS
            if (bit_set(new_value, 10)) {
                this->blank_on = true;
                this->blank_display();
            } else {
                this->blank_on = false;
                this->crtc_update();
            }
        }
        if (bit_changed(old_value, new_value, 15)) { // CRT_CRTC_ON
            if (bit_set(new_value, 15)) {
                this->crtc_update();
            }
        }
        this->regs[reg_num] = new_value;
        return;
    case RADEON_DAC_CNTL:
        this->regs[reg_num] = value & 0xffffe3ff;
        return;
    case RADEON_GPIO_VGA_DDC:
    case RADEON_GPIO_DVI_DDC:
    case RADEON_GPIO_MONID:
        new_value = value;
        if (offset <= 1 && offset + size > 1) {
            uint8_t gpio_levels = (new_value >> 8) & 0xFFU;
            gpio_levels = ((gpio_levels & 0x30) >> 3) | (gpio_levels & 1);
            gpio_levels ^= 7;
            uint8_t gpio_dirs = (new_value >> 16) & 0xFFU;
            gpio_dirs = ((gpio_dirs & 0x30) >> 3) | (gpio_dirs & 1);
            gpio_levels &= ~gpio_dirs;
            gpio_levels = this->disp_id->read_monitor_sense(gpio_levels, gpio_dirs);
            insert_bits<uint32_t>(new_value,
                                ((gpio_levels & 6) << 3) | (gpio_levels & 1), 8, 8);
        }
        this->regs[reg_num] = new_value;
        return;
    case RADEON_PALETTE_INDEX:
        this->dac_wr_index = value & 0xff;
        this->dac_rd_index = (value >> 16) & 0xff;
        this->comp_index = 0;
        this->regs[reg_num] = value;
        return;
    case RADEON_PALETTE_DATA:
        this->color_buf[this->comp_index] = (value >> 8) & this->dac_mask;
        if (++this->comp_index >= 3) {
            this->set_palette_color(this->dac_wr_index, color_buf[0],
                                    color_buf[1], color_buf[2], 0xFF);
            this->dac_wr_index++; // auto-increment color index
            this->comp_index = 0; // reset color component index
            draw_fb = true;
        }
        return;
    case RADEON_CNFG_CNTL:
        this->regs[reg_num] = value;
        return;
    case RADEON_GEN_RESET_CNTL:
        this->regs[reg_num] = value;
        if (value & RADEON_SOFT_RESET_GUI)
            LOG_F(9, "%s: reset GUI engine", this->name.c_str());
        if (value & (RADEON_SOFT_RESET_VCLK | RADEON_SOFT_RESET_PCLK |
                     RADEON_SOFT_RESET_ECP | RADEON_SOFT_RESET_DISPENG_XCLK))
            LOG_F(9, "%s: reset clock/display engine", this->name.c_str());
        return;
    case RADEON_CRTC_H_TOTAL_DISP:
        this->regs[reg_num] = value & 0x07ff07ff;
        this->crtc_update();
        return;
    case RADEON_CRTC_H_SYNC_STRT_WID:
        this->regs[reg_num] = value & 0x17bf1fff;
        return;
    case RADEON_CRTC_V_TOTAL_DISP:
        this->regs[reg_num] = value & 0x0fff0fff;
        this->crtc_update();
        return;
    case RADEON_CRTC_V_SYNC_STRT_WID:
        this->regs[reg_num] = value & 0x9f0fff;
        return;
    case RADEON_CRTC_VLINE_CRNT_VLINE:
        this->regs[reg_num] = (this->regs[reg_num] & 0xfffff000) | (value & 0xfff);
        return;
    case RADEON_CRTC_OFFSET:
        this->regs[reg_num] = value & 0x87fffff8;
        this->crtc_update();
        return;
    case RADEON_CRTC_PITCH:
        this->regs[reg_num] = value & 0x07ff07ff;
        this->crtc_update();
        return;
    case RADEON_CUR_OFFSET:
        this->regs[reg_num] = value & 0x87fffff0;
        this->cursor_dirty = true;
        draw_fb = true;
        return;
    case RADEON_CUR_HORZ_VERT_POSN:
        this->regs[reg_num] = value & 0x3fff0fff;
        draw_fb = true;
        return;
    case RADEON_CUR_HORZ_VERT_OFF:
        this->regs[reg_num] = value & 0x3f003f;
        this->cursor_dirty = true;
        draw_fb = true;
        return;
    case RADEON_CUR_CLR0:
    case RADEON_CUR_CLR1:
        this->regs[reg_num] = value & 0xffffff;
        this->cursor_dirty = true;
        draw_fb = true;
        return;
    case RADEON_DP_GUI_MASTER_CNTL:
        this->dp_gui_master_cntl = value & 0xf800000f;
        this->dp_datatype = ((value & 0x0f00) >> 8) | ((value & 0x30f0) << 4) |
                            ((value & 0x4000) << 16);
        this->dp_mix = (value & RADEON_ROP3_MASK) | ((value & 0x7000000) >> 16);

        if (!(value & RADEON_GMC_SRC_PITCH_OFFSET_CNTL)) {
            this->src_offset = this->default_offset;
            this->src_pitch = this->default_pitch;
        }
        if (!(value & RADEON_GMC_DST_PITCH_OFFSET_CNTL)) {
            this->dst_offset = this->default_offset;
            this->dst_pitch = this->default_pitch;
        }
        if (!(value & RADEON_GMC_SRC_CLIPPING)) {
            this->src_sc_right = this->sc_right;
            this->src_sc_bottom = this->sc_bottom;
        }
        if (!(value & RADEON_GMC_DST_CLIPPING)) {
            this->sc_left = 0;
            this->sc_top = 0;
            this->sc_right = this->default_sc_right;
            this->sc_bottom = this->default_sc_bottom;
        }
        this->regs[reg_num] = value & 0xf800000f;
        return;
    default:
        break;
    }

    if (reg_offset >= 0xf00 && reg_offset < 0x1000) {
        // read-only copy of PCI configuration space, ignore writes
        return;
    }

    switch (reg_offset) {
    case RADEON_DST_OFFSET:
        this->regs[reg_num] = value & 0xfffffff0;
        this->dst_offset = this->regs[reg_num];
        return;
    case RADEON_DST_PITCH:
        this->regs[reg_num] = value & 0x3fff;
        this->dst_pitch = this->regs[reg_num];
        return;
    case RADEON_DST_WIDTH:
        this->regs[reg_num] = value & 0x3fff;
        this->dst_width = this->regs[reg_num];
        this->draw_2d();
        return;
    case RADEON_DST_HEIGHT:
        this->regs[reg_num] = value & 0x3fff;
        this->dst_height = this->regs[reg_num];
        return;
    case RADEON_SRC_X:
        this->regs[reg_num] = value & 0x3fff;
        this->src_x = this->regs[reg_num];
        return;
    case RADEON_SRC_Y:
        this->regs[reg_num] = value & 0x3fff;
        this->src_y = this->regs[reg_num];
        return;
    case RADEON_DST_X:
        this->regs[reg_num] = value & 0x3fff;
        this->dst_x = this->regs[reg_num];
        return;
    case RADEON_DST_Y:
        this->regs[reg_num] = value & 0x3fff;
        this->dst_y = this->regs[reg_num];
        return;
    case RADEON_SRC_PITCH_OFFSET:
        this->regs[reg_num] = value;
        this->src_offset = (value & 0x3fffff) << 10;
        this->src_pitch = (value & 0x3fc00000) >> 16;
        return;
    case RADEON_DST_PITCH_OFFSET:
        this->regs[reg_num] = value;
        this->dst_offset = (value & 0x3fffff) << 10;
        this->dst_pitch = (value & 0x3fc00000) >> 16;
        return;
    case RADEON_SRC_Y_X:
        this->regs[reg_num] = value;
        this->src_x = value & 0x3fff;
        this->src_y = (value >> 16) & 0x3fff;
        return;
    case RADEON_DST_Y_X:
        this->regs[reg_num] = value;
        this->dst_x = value & 0x3fff;
        this->dst_y = (value >> 16) & 0x3fff;
        return;
    case RADEON_DST_HEIGHT_WIDTH:
        this->regs[reg_num] = value;
        this->dst_width = value & 0x3fff;
        this->dst_height = (value >> 16) & 0x3fff;
        this->draw_2d();
        return;
    case RADEON_DST_WIDTH_X:
        this->regs[reg_num] = value;
        this->dst_x = value & 0x3fff;
        this->dst_width = (value >> 16) & 0x3fff;
        this->draw_2d();
        return;
    case RADEON_DST_HEIGHT_WIDTH_8:
        this->regs[reg_num] = value;
        this->dst_width = value & 0x3fff;
        this->dst_height = (value >> 16) & 0x3fff;
        this->draw_2d();
        return;
    case RADEON_SRC_X_Y:
        this->regs[reg_num] = value;
        this->src_y = value & 0x3fff;
        this->src_x = (value >> 16) & 0x3fff;
        return;
    case RADEON_DST_X_Y:
        this->regs[reg_num] = value;
        this->dst_y = value & 0x3fff;
        this->dst_x = (value >> 16) & 0x3fff;
        return;
    case RADEON_DST_WIDTH_HEIGHT:
        this->regs[reg_num] = value;
        this->dst_height = value & 0x3fff;
        this->dst_width = (value >> 16) & 0x3fff;
        this->draw_2d();
        return;
    case RADEON_DST_WIDTH_X_INCY:
        this->regs[reg_num] = value;
        this->dst_x = value & 0x3fff;
        this->dst_width = (value >> 16) & 0x3fff;
        this->draw_2d();
        return;
    case RADEON_DST_HEIGHT_Y:
        this->regs[reg_num] = value;
        this->dst_y = value & 0x3fff;
        this->dst_height = (value >> 16) & 0x3fff;
        return;
    case RADEON_SRC_OFFSET:
        this->regs[reg_num] = value & 0xfffffff0;
        this->src_offset = this->regs[reg_num];
        return;
    case RADEON_SRC_PITCH:
        this->regs[reg_num] = value & 0x3fff;
        this->src_pitch = this->regs[reg_num];
        return;
    case RADEON_DST_HEIGHT_WIDTH_BW:
        this->regs[reg_num] = value;
        this->dst_width = value & 0x3fff;
        this->dst_height = (value >> 16) & 0x3fff;
        this->draw_2d();
        return;
    case RADEON_CLR_CMP_CNTL:
    case RADEON_CLR_CMP_CLR_SRC:
    case RADEON_CLR_CMP_CLR_DST:
    case RADEON_CLR_CMP_MASK:
        this->regs[reg_num] = value;
        return;
    case RADEON_DP_SRC_FRGD_CLR:
    case RADEON_DP_SRC_BKGD_CLR:
        this->regs[reg_num] = value;
        return;
    case RADEON_SC_LEFT:
        this->regs[reg_num] = value & 0x3fff;
        this->sc_left = this->regs[reg_num];
        return;
    case RADEON_SC_RIGHT:
        this->regs[reg_num] = value & 0x3fff;
        this->sc_right = this->regs[reg_num];
        return;
    case RADEON_SC_TOP:
        this->regs[reg_num] = value & 0x3fff;
        this->sc_top = this->regs[reg_num];
        return;
    case RADEON_SC_BOTTOM:
        this->regs[reg_num] = value & 0x3fff;
        this->sc_bottom = this->regs[reg_num];
        return;
    case RADEON_SRC_SC_RIGHT:
        this->regs[reg_num] = value & 0x3fff;
        this->src_sc_right = this->regs[reg_num];
        return;
    case RADEON_SRC_SC_BOTTOM:
        this->regs[reg_num] = value & 0x3fff;
        this->src_sc_bottom = this->regs[reg_num];
        return;
    case RADEON_DP_CNTL:
    case RADEON_DP_CNTL_XDIR_YDIR_YMAJOR:
        this->regs[reg_num] = value;
        this->dp_cntl = value;
        return;
    case RADEON_DP_DATATYPE:
        this->regs[reg_num] = value & 0xe0070f0f;
        this->dp_datatype = this->regs[reg_num];
        return;
    case RADEON_DP_MIX:
        this->regs[reg_num] = value & 0x00ff0700;
        this->dp_mix = this->regs[reg_num];
        return;
    case RADEON_DP_WRITE_MASK:
        this->regs[reg_num] = value;
        this->dp_write_mask = value;
        return;
    case RADEON_DEFAULT_PITCH_OFFSET:
        this->regs[reg_num] = value;
        this->default_offset = (value & 0x3fffff) << 10;
        this->default_pitch = (value & 0x3fc00000) >> 16;
        return;
    case RADEON_DEFAULT_SC_BOTTOM_RIGHT:
        this->regs[reg_num] = value & 0x3fff3fff;
        this->sc_right = this->regs[reg_num] & 0x3fff;
        this->sc_bottom = (this->regs[reg_num] >> 16) & 0x3fff;
        this->default_sc_right = this->sc_right;
        this->default_sc_bottom = this->sc_bottom;
        return;
    case RADEON_SC_TOP_LEFT:
        this->regs[reg_num] = value & 0x3fff3fff;
        this->sc_left = this->regs[reg_num] & 0x3fff;
        this->sc_top = (this->regs[reg_num] >> 16) & 0x3fff;
        return;
    case RADEON_SC_BOTTOM_RIGHT:
        this->regs[reg_num] = value & 0x3fff3fff;
        this->sc_right = this->regs[reg_num] & 0x3fff;
        this->sc_bottom = (this->regs[reg_num] >> 16) & 0x3fff;
        return;
    case RADEON_SRC_SC_BOTTOM_RIGHT:
        this->regs[reg_num] = value & 0x3fff3fff;
        this->src_sc_right = this->regs[reg_num] & 0x3fff;
        this->src_sc_bottom = (this->regs[reg_num] >> 16) & 0x3fff;
        return;
    case RADEON_DST_TILE:
        this->regs[reg_num] = value & 3;
        return;
    case RADEON_WAIT_UNTIL:
    case RADEON_CACHE_CNTL:
    case RADEON_PC_GUI_MODE:
    case RADEON_PC_GUI_CTLSTAT:
    case RADEON_GUI_STAT:
        this->regs[reg_num] = value;
        return;
    default:
        break;
    }

    if (reg_offset < sizeof(this->regs)) {
        this->regs[reg_num] = value;
        LOG_F(9, "%s: write %s %04x = %08x", this->name.c_str(),
              get_reg_name(reg_num), reg_offset, value);
    }
    else {
        LOG_F(WARNING, "%s: write  out of range register %04x = %08x",
              this->name.c_str(), reg_offset, value);
    }
}

bool ATIRadeon::io_access_allowed(uint32_t offset) {
    if (offset >= this->aperture_base[1] && offset < (this->aperture_base[1] + 0x100)) {
        if (this->command & 1) {
            return true;
        }
        LOG_F(WARNING, "ATI I/O space disabled in the command reg");
    }
    return false;
}

bool ATIRadeon::pci_io_read(uint32_t offset, uint32_t size, uint32_t* res) {
    if (!this->io_access_allowed(offset)) {
        return false;
    }

    *res = BYTESWAP_SIZED(this->read_reg(offset - this->aperture_base[1], size), size);
    return true;
}

bool ATIRadeon::pci_io_write(uint32_t offset, uint32_t value, uint32_t size) {
    if (!this->io_access_allowed(offset)) {
        return false;
    }

    this->write_reg(offset - this->aperture_base[1], BYTESWAP_SIZED(value, size), size);
    return true;
}

uint32_t ATIRadeon::read(uint32_t rgn_start, uint32_t offset, int size)
{
    if (rgn_start == this->aperture_base[0] && offset < this->aperture_size[0]) {
        if (offset < this->vram_size) { // little-endian VRAM region
            return read_mem(&this->vram_ptr[offset], size);
        }
        LOG_F(WARNING, "%s: read  unmapped aperture[0] region %08x.%c",
              this->name.c_str(), offset, SIZE_ARG(size));
        return 0;
    }

    if (rgn_start == this->aperture_base[2] && offset < this->aperture_size[2]) {
        return BYTESWAP_SIZED(this->read_reg(offset, size), size);
    }

    // memory mapped expansion ROM region
    if (rgn_start == this->exp_rom_addr) {
        if (this->exp_rom_data && offset < this->exp_rom_size) {
            return read_mem(&this->exp_rom_data[offset], size);
        }
        LOG_F(WARNING, "%s: read  unmapped ROM region %08x.%c",
            this->name.c_str(), offset, SIZE_ARG(size));
        return 0;
    }

    LOG_F(WARNING, "%s: read  unmapped aperture region %08x.%c",
          this->name.c_str(), offset, SIZE_ARG(size));
    return 0;
}

void ATIRadeon::write(uint32_t rgn_start, uint32_t offset, uint32_t value, int size)
{
    if (rgn_start == this->aperture_base[0] && offset < this->aperture_size[0]) {
        if (offset < this->vram_size) { // little-endian VRAM region
            draw_fb = true;
            write_mem(&this->vram_ptr[offset], value, size);
            return;
        }
        LOG_F(WARNING, "%s: write unmapped aperture[0] region %08x.%c = %0*x",
              this->name.c_str(), offset, SIZE_ARG(size), size * 2, value);
        return;
    }

    if (rgn_start == this->aperture_base[2] && offset < this->aperture_size[2]) {
        this->write_reg(offset, BYTESWAP_SIZED(value, size), size);
        return;
    }

    if (rgn_start == this->exp_rom_addr) {
        LOG_F(WARNING, "%s: write to expansion ROM ignored %08x.%c = %0*x",
              this->name.c_str(), offset, SIZE_ARG(size), size * 2, value);
        return;
    }

    LOG_F(WARNING, "%s: write unmapped aperture region %08x.%c = %0*x",
          this->name.c_str(), offset, SIZE_ARG(size), size * 2, value);
}

void ATIRadeon::crtc_update() {
    uint32_t new_width, new_height, new_htotal, new_vtotal;

    // check for unsupported modes and fail early
    if (!bit_set(this->regs[RADEON_CRTC_GEN_CNTL >> 2], 24)) // CRTC_EXT_DISP_EN
        return;

    if (!bit_set(this->regs[RADEON_CRTC_GEN_CNTL >> 2], 25)) // CRTC_EN
        return;

    bool need_recalc = false;

    new_width  = (extract_bits<uint32_t>(this->regs[RADEON_CRTC_H_TOTAL_DISP >> 2],
                                         16, 11) + 1) * 8;
    new_height = extract_bits<uint32_t>(this->regs[RADEON_CRTC_V_TOTAL_DISP >> 2],
                                        16, 12) + 1;

    // display not configured yet
    if (extract_bits<uint32_t>(this->regs[RADEON_CRTC_H_TOTAL_DISP >> 2], 16, 11) == 0 ||
        extract_bits<uint32_t>(this->regs[RADEON_CRTC_V_TOTAL_DISP >> 2], 16, 12) == 0) {
        new_width  = 640;
        new_height = 480;
    }

    if (new_width != this->active_width || new_height != this->active_height) {
        this->create_display_window(new_width, new_height);
        need_recalc = true;
    }

    new_htotal = (extract_bits<uint32_t>(this->regs[RADEON_CRTC_H_TOTAL_DISP >> 2],
                                         0, 11) + 1) * 8;
    new_vtotal = extract_bits<uint32_t>(this->regs[RADEON_CRTC_V_TOTAL_DISP >> 2],
                                        0, 12) + 1;

    if (new_htotal != this->hori_total || new_vtotal != this->vert_total) {
        this->hori_total = new_htotal;
        this->vert_total = new_vtotal;
        need_recalc = true;
    }

    uint32_t new_vert_blank = new_vtotal - new_height;
    if (new_vert_blank != this->vert_blank) {
        this->vert_blank = new_vert_blank;
        need_recalc = true;
    }

    uint32_t new_pixel_format = extract_bits<uint32_t>(this->regs[RADEON_CRTC_GEN_CNTL >> 2],
                                                       8, 3);
    if (new_pixel_format != this->pixel_format) {
        this->pixel_format = new_pixel_format;
        need_recalc = true;
    }

    static uint8_t bits_per_pixel[8] = {0, 4, 8, 16, 16, 24, 32, 0};

    int new_fb_pitch = (this->regs[RADEON_CRTC_PITCH >> 2] & 0x7ff) * 8;
    if (new_fb_pitch != this->fb_pitch) {
        this->fb_pitch = new_fb_pitch;
        need_recalc = true;
    }
    uint32_t fb_offset = this->regs[RADEON_CRTC_OFFSET >> 2] & 0x07ffffff;
    if (fb_offset > this->vram_size - (new_width * (bits_per_pixel[this->pixel_format] >> 3))) {
        fb_offset = 0;
    }
    uint8_t* new_fb_ptr = &this->vram_ptr[fb_offset];
    if (new_fb_ptr != this->fb_ptr) {
        this->fb_ptr = new_fb_ptr;
        need_recalc = true;
    }

    // calculate PPLL output frequency
    uint8_t  ppll_ref_div = this->plls[RADEON_PPLL_REF_DIV] & 0x3ff;
    uint32_t ppll_fb_div  = this->plls[RADEON_PPLL_DIV_3] & 0x7ff;
    uint32_t post_div     = 1 << ((this->plls[RADEON_PPLL_DIV_3] >> 16) & 3);

    float new_pixel_clock;
    if (ppll_ref_div) {
        new_pixel_clock = RADEON_XTAL_CLK * ppll_fb_div / (ppll_ref_div * post_div);
    } else {
        new_pixel_clock = 0;
    }
    if (new_pixel_clock != this->pixel_clock) {
        this->pixel_clock = new_pixel_clock;
        need_recalc = true;
    }

    if (!need_recalc)
        return;

    this->draw_fb = true;
    LOG_F(INFO, "%s: ppll_ref_div:%d ppll_fb_div:%d post_div:%d",
          this->name.c_str(), ppll_ref_div, ppll_fb_div, post_div);

    // calculate display refresh rate
    this->refresh_rate = pixel_clock / this->hori_total / this->vert_total;

    if (this->refresh_rate < 24 || this->refresh_rate > 120) {
        LOG_F(ERROR, "%s: Refresh rate is weird. Will try 60 Hz", this->name.c_str());
        this->refresh_rate = 60;
        this->pixel_clock = this->refresh_rate * this->hori_total * this->vert_total;
    }

    // set up frame buffer converter
    switch (this->pixel_format) {
    case 1: // CRTC_PIX_WIDTH_4BPP
        this->convert_fb_cb = [this](uint8_t *dst_buf, int dst_pitch) {
            draw_fb = false;
            this->convert_frame_4bpp_indexed(dst_buf, dst_pitch);
        };
        break;
    case 2: // CRTC_PIX_WIDTH_8BPP
        this->convert_fb_cb = [this](uint8_t *dst_buf, int dst_pitch) {
            draw_fb = false;
            this->convert_frame_8bpp_indexed(dst_buf, dst_pitch);
        };
        break;
    case 3: // CRTC_PIX_WIDTH_15BPP
        this->convert_fb_cb = [this](uint8_t *dst_buf, int dst_pitch) {
            draw_fb = false;
            this->convert_frame_15bpp<LE>(dst_buf, dst_pitch);
        };
        break;
    case 4: // CRTC_PIX_WIDTH_16BPP
        this->convert_fb_cb = [this](uint8_t *dst_buf, int dst_pitch) {
            draw_fb = false;
            this->convert_frame_16bpp<LE>(dst_buf, dst_pitch);
        };
        break;
    case 5: // CRTC_PIX_WIDTH_24BPP
        this->convert_fb_cb = [this](uint8_t *dst_buf, int dst_pitch) {
            draw_fb = false;
            this->convert_frame_24bpp(dst_buf, dst_pitch);
        };
        break;
    case 6: // CRTC_PIX_WIDTH_32BPP
        this->convert_fb_cb = [this](uint8_t *dst_buf, int dst_pitch) {
            draw_fb = false;
            this->convert_frame_32bpp<LE>(dst_buf, dst_pitch);
        };
        break;
    default:
        LOG_F(ERROR, "%s: unsupported pixel format %d", this->name.c_str(), this->pixel_format);
    }

    LOG_F(INFO, "%s: primary CRT controller enabled:", this->name.c_str());
    LOG_F(INFO, "Video width: %d pixels", this->active_width);
    LOG_F(INFO, "Video height: %d lines", this->active_height);
    LOG_F(INFO, "Vertical blank: %d lines", this->vert_blank);
    LOG_F(INFO, "Pixel (dot) clock: %f MHz", this->pixel_clock * 1e-6);
    LOG_F(INFO, "Refresh rate: %f Hz", this->refresh_rate);

    this->stop_refresh_task();
    this->start_refresh_task();

    this->crtc_on = true;
}

int ATIRadeon::device_postinit()
{
    this->vbl_cb = [this](uint8_t irq_line_state) {
        if (irq_line_state) {
            set_bit(this->regs[RADEON_GEN_INT_STATUS >> 2], 0); // CRTC_VBLANK_INT
            set_bit(this->regs[RADEON_GEN_INT_STATUS >> 2], 1); // CRTC_VLINE_INT
        } else {
            clear_bit(this->regs[RADEON_GEN_INT_STATUS >> 2], 0);
        }
        this->update_interrupt();
    };
    return 0;
}

void ATIRadeon::update_interrupt()
{
    uint32_t int_cntl = this->regs[RADEON_GEN_INT_CNTL >> 2];
    uint32_t int_status = this->regs[RADEON_GEN_INT_STATUS >> 2];
    bool new_pci_irq_line_state = !!(int_status & int_cntl);

    if (new_pci_irq_line_state != this->pci_irq_line_state) {
        this->pci_irq_line_state = new_pci_irq_line_state;
        this->pci_interrupt(this->pci_irq_line_state);
    }
}

// =================================== Draw Engine =====================================

int ATIRadeon::dst_bpp() const {
    switch (this->dp_datatype & RADEON_DP_DST_DATATYPE) {
    case RADEON_DST_8BPP:
        return 8;
    case RADEON_DST_15BPP:
    case RADEON_DST_16BPP:
        return 16;
    case RADEON_DST_24BPP:
        return 24;
    case RADEON_DST_32BPP:
        return 32;
    default:
        return 0;
    }
}

void ATIRadeon::draw_2d() {
    uint32_t src_source = this->dp_mix & RADEON_DP_SRC_SOURCE;
    int bpp = this->dst_bpp();

    if (!bpp) {
        LOG_F(WARNING, "%s: unsupported 2D dst datatype 0x%X", this->name.c_str(),
              this->dp_datatype & RADEON_DP_DST_DATATYPE);
        return;
    }

    if (src_source == RADEON_DP_SRC_HOST || src_source == RADEON_DP_SRC_HOST_BYTEALIGN) {
        // begin a HOST_DATA blit
        this->host_dst_width   = this->dst_width;
        this->host_dst_height  = this->dst_height;
        this->host_dst_col     = 0;
        this->host_dst_row     = 0;
        this->host_data_active = true;
        return;
    }

    bool left_to_right = this->dp_cntl & RADEON_DST_X_LEFT_TO_RIGHT;
    bool top_to_bottom = this->dp_cntl & RADEON_DST_Y_TOP_TO_BOTTOM;

    int dst_x = this->dst_x;
    int dst_y = this->dst_y;
    if (!left_to_right)
        dst_x = this->dst_x + 1 - int(this->dst_width);
    if (!top_to_bottom)
        dst_y = this->dst_y + 1 - int(this->dst_height);

    int bypp = bpp >> 3;
    // pitch fields (DST_PITCH_OFFSET [29:22] = line_bytes/64) are already
    // decoded into byte strides by the register writes
    int dst_stride = this->dst_pitch;
    int src_stride = this->src_pitch;

    uint32_t rop3 = this->dp_mix & RADEON_DP_ROP3;

    switch (rop3) {
    case RADEON_ROP3_SRCCOPY:
    {
        if (!src_stride) {
            LOG_F(WARNING, "%s: SRCCOPY blit with zero source pitch", this->name.c_str());
            return;
        }
        if (!this->host_data_active) {
            // bound check the source rect
            if (this->src_offset + dst_height * src_stride + this->dst_width * bypp >
                this->vram_size) {
                LOG_F(WARNING, "%s: SRCCOPY blit outside VRAM", this->name.c_str());
                return;
            }
        }

        int src_x = this->src_x;
        int src_y = this->src_y;
        if (!left_to_right)
            src_x = this->src_x + 1 - int(this->dst_width);
        if (!top_to_bottom)
            src_y = this->src_y + 1 - int(this->dst_height);

        // clip the destination rect to the scissor window
        int x_skip, y_skip;
        if (!radeon_clip_axis(dst_x, 1, this->dst_width, this->sc_left, this->sc_right,
                              x_skip, this->dst_width) ||
            !radeon_clip_axis(dst_y, 1, this->dst_height, this->sc_top, this->sc_bottom,
                              y_skip, this->dst_height)) {
            return;
        }
        dst_x += x_skip;
        dst_y += y_skip;

        // only support left-to-right, top-to-bottom blits for now
        if (!left_to_right || !top_to_bottom) {
            LOG_F(WARNING, "%s: unsupported SRCCOPY direction, DP_CNTL=0x%08X",
                  this->name.c_str(), this->dp_cntl);
            return;
        }

        for (uint32_t y = 0; y < this->dst_height; y++) {
            uint8_t* src_row = &this->vram_ptr[this->src_offset + (src_y + int(y)) * src_stride];
            uint8_t* dst_row = &this->vram_ptr[this->dst_offset + (dst_y + int(y)) * dst_stride];
            memmove(&dst_row[dst_x * bypp],
                    &src_row[(src_x + x_skip) * bypp],
                    this->dst_width * bypp);
        }
        this->draw_fb = true;
        break;
    }
    case RADEON_ROP3_PATCOPY:
    case RADEON_ROP3_BLACKNESS:
    case RADEON_ROP3_WHITENESS:
    {
        if (bpp == 24) {
            LOG_F(WARNING, "%s: fill blit unsupported in 24 bits", this->name.c_str());
            return;
        }

        uint32_t filler;
        switch (rop3) {
        case RADEON_ROP3_PATCOPY:
            filler = this->dp_src_frgd_clr;
            break;
        case RADEON_ROP3_BLACKNESS:
            filler = 0;
            break;
        default: // WHITENESS
            filler = 0xffffffff;
            break;
        }
        if (bypp == 2) {
            filler &= 0xffff;
            filler = (filler << 16) | filler;
        } else if (bypp == 1) {
            filler &= 0xff;
            filler = (filler << 24) | (filler << 16) | (filler << 8) | filler;
        }

        int x_skip, y_skip;
        if (!radeon_clip_axis(dst_x, 1, this->dst_width, this->sc_left, this->sc_right,
                              x_skip, this->dst_width) ||
            !radeon_clip_axis(dst_y, 1, this->dst_height, this->sc_top, this->sc_bottom,
                              y_skip, this->dst_height)) {
            return;
        }
        dst_x += x_skip;
        dst_y += y_skip;

        for (uint32_t y = 0; y < this->dst_height; y++) {
            uint8_t* row = &this->vram_ptr[this->dst_offset + (dst_y + int(y)) * dst_stride];
            if (bypp == 4) {
                uint32_t* d = (uint32_t*)&row[dst_x * 4];
                for (uint32_t x = 0; x < this->dst_width; x++)
                    d[x] = filler;
            } else if (bypp == 2) {
                uint16_t* d = (uint16_t*)&row[dst_x * 2];
                for (uint32_t x = 0; x < this->dst_width; x++)
                    d[x] = filler & 0xffff;
            } else {
                memset(&row[dst_x], filler & 0xff, this->dst_width);
            }
        }
        this->draw_fb = true;
        break;
    }
    default:
        LOG_F(WARNING, "%s: unimplemented 2D blit op 0x%X (DP_MIX=0x%08X)",
              this->name.c_str(), rop3, this->dp_mix);
        break;
    }
}

void ATIRadeon::write_host_data(uint32_t value, uint32_t size) {
    if (!this->host_data_active)
        return;

    uint32_t src_datatype = this->dp_datatype & RADEON_DP_SRC_DATATYPE;
    int bpp = this->dst_bpp();
    int bypp = bpp >> 3;

    if (src_datatype != RADEON_SRC_MONO_FRGD_BKGD &&
        src_datatype != RADEON_SRC_MONO_FRGD &&
        src_datatype != RADEON_SRC_COLOR) {
        LOG_F(WARNING, "%s: unsupported HOST_DATA src datatype 0x%X",
              this->name.c_str(), src_datatype);
        this->host_data_active = false;
        return;
    }

    bool left_to_right = this->dp_cntl & RADEON_DST_X_LEFT_TO_RIGHT;
    bool top_to_bottom = this->dp_cntl & RADEON_DST_Y_TOP_TO_BOTTOM;

    if (!left_to_right || !top_to_bottom || bpp == 24) {
        LOG_F(WARNING, "%s: unsupported HOST_DATA blit direction, DP_CNTL=0x%08X",
              this->name.c_str(), this->dp_cntl);
        this->host_data_active = false;
        return;
    }

    int dst_stride = this->dst_pitch; // pitch is decoded as a byte stride
    uint8_t write_mask = this->dp_write_mask;

    if (src_datatype == RADEON_SRC_COLOR) {
        uint32_t pix = BYTESWAP_32(value);
        for (uint32_t p = 0; p < (4 / bypp) && this->host_data_active; p++) {
            int x = this->dst_x + int(this->host_dst_col);
            int y = this->dst_y + int(this->host_dst_row);

            if (x >= int(this->sc_left) && x <= int(this->sc_right) &&
                y >= int(this->sc_top) && y <= int(this->sc_bottom) &&
                x >= 0 && y >= 0) {
                uint32_t pixel;
                if (bypp == 1) {
                    pixel = pix & 0xff;
                } else {
                    pixel = (pix >> 16) & 0xffff; // 16bpp: MSB pixel first
                }
                pixel &= write_mask;
                if (this->dst_offset + (uint32_t)y * dst_stride + (uint32_t)x * bypp < this->vram_size) {
                    uint8_t* dst = &this->vram_ptr[this->dst_offset + (uint32_t)y * dst_stride + (uint32_t)x * bypp];
                    for (int byte = 0; byte < bypp; byte++) {
                        dst[byte] = (dst[byte] & ~(write_mask >> (byte * 8))) |
                                    ((pixel >> (byte * 8)) & (write_mask >> (byte * 8)));
                    }
                    this->draw_fb = true;
                }
            }
            pix >>= bypp * 8;
            if (++this->host_dst_col >= this->host_dst_width) {
                this->host_dst_col = 0;
                if (++this->host_dst_row >= this->host_dst_height) {
                    this->host_data_active = false;
                }
            }
        }
        return;
    }

    // expand monochrome bits to fg/bg colored pixels
    uint32_t fg = this->dp_src_frgd_clr;
    uint32_t bg = this->dp_src_bkgd_clr;
    if (bypp == 2) {
        fg &= 0xffff;
        fg = (fg << 16) | fg;
        bg &= 0xffff;
        bg = (bg << 16) | bg;
    } else if (bypp == 1) {
        fg &= 0xff;
        fg = (fg << 24) | (fg << 16) | (fg << 8) | fg;
        bg &= 0xff;
        bg = (bg << 24) | (bg << 16) | (bg << 8) | bg;
    }

    bool msb_first = !(this->dp_datatype & RADEON_DP_BYTE_PIX_ORDER);
    uint32_t word = BYTESWAP_32(value);

    for (int byte = 0; byte < int(size) && this->host_data_active; byte++) {
        int shift = msb_first ? (3 - byte) * 8 : byte * 8;
        uint8_t data = (word >> shift) & 0xff;
        for (int bit = 0; bit < 8 && this->host_data_active; bit++) {
            bool is_fg = msb_first ? (data & (0x80 >> bit)) : (data & (1 << bit));
            int x = this->dst_x + int(this->host_dst_col);
            int y = this->dst_y + int(this->host_dst_row);

            if (x >= int(this->sc_left) && x <= int(this->sc_right) &&
                y >= int(this->sc_top) && y <= int(this->sc_bottom) &&
                x >= 0 && y >= 0) {
                uint32_t pixel = (is_fg ? fg : bg) & write_mask;
                uint32_t fb_offs = this->dst_offset + (uint32_t)y * dst_stride + (uint32_t)x * bypp;
                if (fb_offs < this->vram_size) {
                    uint8_t* dst = &this->vram_ptr[fb_offs];
                    for (int wbyte = 0; wbyte < bypp; wbyte++) {
                        uint8_t mask_byte = (write_mask >> (wbyte * 8)) & 0xff;
                        dst[wbyte] = (dst[wbyte] & ~mask_byte) |
                                     ((pixel >> (wbyte * 8)) & mask_byte);
                    }
                    this->draw_fb = true;
                }
            }

            if (++this->host_dst_col >= this->host_dst_width) {
                this->host_dst_col = 0;
                if (++this->host_dst_row >= this->host_dst_height) {
                    this->host_data_active = false;
                }
            }
        }
    }
}

// ================================== Device config ====================================

static const PropMap AtiRadeon_Properties = {
    {"gfxmem_size",
        new IntProperty(64, std::vector<uint32_t>({16, 32, 64, 128}))},
    {"mon_id",
        new StrProperty("")},
};

static const DeviceDescription AtiRadeonRV280_Descriptor = {
    ATIRadeon::create_rv280, {}, AtiRadeon_Properties,
    HWCompType::MMIO_DEV | HWCompType::PCI_DEV
};

REGISTER_DEVICE(AtiRadeonRV280, AtiRadeonRV280_Descriptor);
