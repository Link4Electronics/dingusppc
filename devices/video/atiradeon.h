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

#ifndef ATI_RADEON_H
#define ATI_RADEON_H

#include <devices/common/pci/pcidevice.h>
#include <devices/video/atiradeonregs.h>
#include <devices/video/displayid.h>
#include <devices/video/videoctrl.h>

#include <cinttypes>
#include <memory>

class ATIRadeon : public PCIDevice, public VideoCtrlBase {
public:
    ATIRadeon(uint16_t dev_id);
    ~ATIRadeon() = default;

    static std::unique_ptr<HWComponent> create_rv280() {
        return std::unique_ptr<ATIRadeon>(new ATIRadeon(ATI_RADEON_RV280_DEV_ID));
    }

    // HWComponent methods
    int device_postinit();

    // MMIODevice methods
    uint32_t read(uint32_t rgn_start, uint32_t offset, int size);
    void write(uint32_t rgn_start, uint32_t offset, uint32_t value, int size);

    // PCI device methods
    uint32_t pci_cfg_read(uint32_t reg_offs, AccessDetails &details);
    void pci_cfg_write(uint32_t reg_offs, uint32_t value, AccessDetails &details);

    // I/O space access methods
    bool pci_io_read(uint32_t offset, uint32_t size, uint32_t* res);
    bool pci_io_write(uint32_t offset, uint32_t value, uint32_t size);

protected:
    void notify_bar_change(int bar_num);
    const char* get_reg_name(uint32_t reg_num);
    bool io_access_allowed(uint32_t offset);
    uint32_t read_reg(uint32_t reg_offset, uint32_t size);
    void write_reg(uint32_t reg_offset, uint32_t value, uint32_t size);
    void crtc_update();
    void update_interrupt();

private:
    void change_one_bar(uint32_t &aperture, uint32_t aperture_size,
                        uint32_t aperture_new, int bar_num);
    void draw_2d();
    void start_host_rect();
    void write_host_data(uint32_t value, uint32_t size);

    // 64 KB of memory-mapped registers (the whole Radeon register space,
    // addressable either directly or via MM_INDEX/MM_DATA).
    uint32_t    regs[0x10000 / 4] = {};
    uint8_t     plls[64]  = {}; // PLL registers (selected via CLOCK_CNTL_INDEX)

    uint32_t    cmd_fifo_size = 0;
    bool        pci_irq_line_state = false;

    bool        host_data_active = false;
    uint32_t    host_dst_width   = 0;
    uint32_t    host_dst_height  = 0;
    uint32_t    host_dst_col     = 0;
    uint32_t    host_dst_row     = 0;

    // Video RAM variables
    std::unique_ptr<uint8_t[]>  vram_ptr;
    uint32_t    vram_size;
    uint32_t    framebuffer_size;

    // display identification
    std::unique_ptr<DisplayID>  disp_id;

    // config 0x40
    uint8_t     user_cfg = 8;

    // main aperture (128MB)
    // I/O region (256 bytes)
    // register aperture (64KB)
    uint32_t aperture_count = 3;
    uint32_t aperture_base[3] = { 0, 0, 0 };
    uint32_t aperture_size[3] = { 0x8000000, 0x100, 0x10000 };
    uint32_t aperture_flag[3] = { 0, 1, 0 };

    // DAC interface state
    uint8_t     dac_wr_index = 0;  // current DAC color index for writing
    uint8_t     dac_rd_index = 0;  // current DAC color index for reading
    uint8_t     dac_mask     = 0;  // current DAC mask
    int         comp_index   = 0;  // current color component index
    uint8_t     color_buf[3] = {}; // buffer for storing DAC color components

    // 2D engine state
    uint32_t    dst_offset = 0;
    uint32_t    dst_pitch  = 0;
    uint32_t    dst_width  = 0;
    uint32_t    dst_height = 0;
    uint32_t    dst_x      = 0;
    uint32_t    dst_y      = 0;
    uint32_t    src_offset = 0;
    uint32_t    src_pitch  = 0;
    uint32_t    src_x      = 0;
    uint32_t    src_y      = 0;
    uint32_t    default_offset = 0;
    uint32_t    default_pitch  = 0;
    uint32_t    sc_left   = 0;
    uint32_t    sc_right  = 0;
    uint32_t    sc_top    = 0;
    uint32_t    sc_bottom = 0;
    uint32_t    default_sc_right  = 0;
    uint32_t    default_sc_bottom = 0;
    uint32_t    src_sc_right  = 0;
    uint32_t    src_sc_bottom = 0;
    uint32_t    dp_gui_master_cntl = 0;
    uint32_t    dp_cntl   = 0;
    uint32_t    dp_datatype = 0;
    uint32_t    dp_mix    = 0;
    uint32_t    dp_write_mask = 0;
    uint32_t    dp_src_frgd_clr = 0;
    uint32_t    dp_src_bkgd_clr = 0;

    int         dst_bpp() const;
};

#endif // ATI_RADEON_H
