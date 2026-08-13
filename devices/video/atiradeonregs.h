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

#ifndef ATI_RADEON_DEFS_H
#define ATI_RADEON_DEFS_H

/* ATI PCI device IDs. */
enum {
    ATI_RADEON_RV100_DEV_ID  = 0x5159, // RV100 (QY, Radeon VE)
    ATI_RADEON_RV280_DEV_ID  = 0x5962, // RV280 (RockHopper2, Radeon 9200)
};

/* Radeon (R100/R200 family) register offsets. */
enum {
    RADEON_MM_INDEX              = 0x0000,
    RADEON_MM_DATA               = 0x0004,
    RADEON_CLOCK_CNTL_INDEX      = 0x0008,
    RADEON_CLOCK_CNTL_DATA       = 0x000c,
    RADEON_BIOS_0_SCRATCH        = 0x0010,
    RADEON_BIOS_7_SCRATCH        = 0x002c,
    RADEON_BUS_CNTL              = 0x0030,
    RADEON_BUS_CNTL1             = 0x0034,
    RADEON_GEN_INT_CNTL          = 0x0040,
    RADEON_GEN_INT_STATUS        = 0x0044,
    RADEON_CRTC_GEN_CNTL         = 0x0050,
    RADEON_CRTC_EXT_CNTL         = 0x0054,
    RADEON_DAC_CNTL              = 0x0058,
    RADEON_GPIO_VGA_DDC          = 0x0060,
    RADEON_GPIO_DVI_DDC          = 0x0064,
    RADEON_GPIO_MONID            = 0x0068,
    RADEON_I2C_CNTL_1            = 0x0094,
    RADEON_PALETTE_INDEX         = 0x00b0,
    RADEON_PALETTE_DATA          = 0x00b4,
    RADEON_PALETTE_30_DATA       = 0x00b8,
    RADEON_CNFG_CNTL             = 0x00e0,
    RADEON_GEN_RESET_CNTL        = 0x00f0,
    RADEON_CNFG_MEMSIZE          = 0x00f8,
    RADEON_CONFIG_APER_0_BASE    = 0x0100,
    RADEON_CONFIG_APER_1_BASE    = 0x0104,
    RADEON_CONFIG_APER_SIZE      = 0x0108,
    RADEON_CONFIG_REG_1_BASE     = 0x010c,
    RADEON_CONFIG_REG_APER_SIZE  = 0x0110,
    RADEON_HOST_PATH_CNTL        = 0x0130,
    RADEON_MEM_CNTL              = 0x0140,
    RADEON_MC_FB_LOCATION        = 0x0148,
    RADEON_MC_AGP_LOCATION       = 0x014c,
    RADEON_MC_STATUS             = 0x0150,
    RADEON_MEM_SDRAM_MODE_REG    = 0x0158,
    RADEON_MEM_POWER_MISC        = 0x015c,
    RADEON_AGP_BASE              = 0x0170,
    RADEON_AGP_CNTL              = 0x0174,
    RADEON_AGP_APER_OFFSET       = 0x0178,
    RADEON_PCI_GART_PAGE         = 0x017c,

    RADEON_CRTC_H_TOTAL_DISP     = 0x0200,
    RADEON_CRTC_H_SYNC_STRT_WID  = 0x0204,
    RADEON_CRTC_V_TOTAL_DISP     = 0x0208,
    RADEON_CRTC_V_SYNC_STRT_WID  = 0x020c,
    RADEON_CRTC_VLINE_CRNT_VLINE = 0x0210,
    RADEON_CRTC_CRNT_FRAME       = 0x0214,
    RADEON_CRTC_GUI_TRIG_VLINE   = 0x0218,
    RADEON_CRTC_OFFSET           = 0x0224,
    RADEON_CRTC_OFFSET_CNTL      = 0x0228,
    RADEON_CRTC_PITCH            = 0x022c,
    RADEON_OVR_CLR               = 0x0230,
    RADEON_OVR_WID_LEFT_RIGHT    = 0x0234,
    RADEON_OVR_WID_TOP_BOTTOM    = 0x0238,
    RADEON_CUR_OFFSET            = 0x0260,
    RADEON_CUR_HORZ_VERT_POSN    = 0x0264,
    RADEON_CUR_HORZ_VERT_OFF     = 0x0268,
    RADEON_CUR_CLR0              = 0x026c,
    RADEON_CUR_CLR1              = 0x0270,
    RADEON_LVDS_GEN_CNTL         = 0x02d0,

    RADEON_CRTC2_H_TOTAL_DISP    = 0x0300,
    RADEON_CRTC2_H_SYNC_STRT_WID = 0x0304,
    RADEON_CRTC2_V_TOTAL_DISP    = 0x0308,
    RADEON_CRTC2_V_SYNC_STRT_WID = 0x030c,
    RADEON_CRTC2_VLINE_CRNT_VLINE = 0x0310,
    RADEON_CRTC2_CRNT_FRAME      = 0x0314,
    RADEON_CRTC2_OFFSET          = 0x0324,
    RADEON_CRTC2_OFFSET_CNTL     = 0x0328,
    RADEON_CRTC2_PITCH           = 0x032c,
    RADEON_CRTC2_GEN_CNTL        = 0x03f8,
    RADEON_CRTC2_STATUS          = 0x03fc,

    RADEON_RBBM_STATUS           = 0x0e40,

    /* GUI (drawing engine) registers, 0x1400-0x1fff. */
    RADEON_DST_OFFSET            = 0x1404,
    RADEON_DST_PITCH             = 0x1408,
    RADEON_DST_WIDTH             = 0x140c,
    RADEON_DST_HEIGHT            = 0x1410,
    RADEON_SRC_X                 = 0x1414,
    RADEON_SRC_Y                 = 0x1418,
    RADEON_DST_X                 = 0x141c,
    RADEON_DST_Y                 = 0x1420,
    RADEON_SRC_PITCH_OFFSET      = 0x1428,
    RADEON_DST_PITCH_OFFSET      = 0x142c,
    RADEON_SRC_Y_X               = 0x1434,
    RADEON_DST_Y_X               = 0x1438,
    RADEON_DST_HEIGHT_WIDTH      = 0x143c,
    RADEON_DP_GUI_MASTER_CNTL    = 0x146c,
    RADEON_BRUSH_SCALE           = 0x1470,
    RADEON_BRUSH_Y_X             = 0x1474,
    RADEON_DP_BRUSH_BKGD_CLR     = 0x1478,
    RADEON_DP_BRUSH_FRGD_CLR     = 0x147c,
    RADEON_DST_WIDTH_X           = 0x1588,
    RADEON_DST_HEIGHT_WIDTH_8    = 0x158c,
    RADEON_SRC_X_Y               = 0x1590,
    RADEON_DST_X_Y               = 0x1594,
    RADEON_DST_WIDTH_HEIGHT      = 0x1598,
    RADEON_DST_WIDTH_X_INCY      = 0x159c,
    RADEON_DST_HEIGHT_Y          = 0x15a0,
    RADEON_DST_X_SUB             = 0x15a4,
    RADEON_DST_Y_SUB             = 0x15a8,
    RADEON_SRC_OFFSET            = 0x15ac,
    RADEON_SRC_PITCH             = 0x15b0,
    RADEON_DST_HEIGHT_WIDTH_BW   = 0x15b4,
    RADEON_CLR_CMP_CNTL          = 0x15c0,
    RADEON_CLR_CMP_CLR_SRC       = 0x15c4,
    RADEON_CLR_CMP_CLR_DST       = 0x15c8,
    RADEON_CLR_CMP_MASK          = 0x15cc,
    RADEON_DP_SRC_FRGD_CLR       = 0x15d8,
    RADEON_DP_SRC_BKGD_CLR       = 0x15dc,
    RADEON_SC_LEFT               = 0x1640,
    RADEON_SC_RIGHT              = 0x1644,
    RADEON_SC_TOP                = 0x1648,
    RADEON_SC_BOTTOM             = 0x164c,
    RADEON_SRC_SC_RIGHT          = 0x1654,
    RADEON_SRC_SC_BOTTOM         = 0x165c,
    RADEON_DP_CNTL               = 0x16c0,
    RADEON_DP_DATATYPE           = 0x16c4,
    RADEON_DP_MIX                = 0x16c8,
    RADEON_DP_WRITE_MASK         = 0x16cc,
    RADEON_DP_CNTL_XDIR_YDIR_YMAJOR = 0x16d0,
    RADEON_DEFAULT_PITCH_OFFSET  = 0x16e0,
    RADEON_DEFAULT_SC_BOTTOM_RIGHT = 0x16e8,
    RADEON_SC_TOP_LEFT           = 0x16ec,
    RADEON_SC_BOTTOM_RIGHT       = 0x16f0,
    RADEON_SRC_SC_BOTTOM_RIGHT   = 0x16f4,
    RADEON_DST_TILE              = 0x1700,
    RADEON_WAIT_UNTIL            = 0x1720,
    RADEON_CACHE_CNTL            = 0x1724,
    RADEON_GUI_STAT              = 0x1740,
    RADEON_PC_GUI_MODE           = 0x1744,
    RADEON_PC_GUI_CTLSTAT        = 0x1748,
    RADEON_HOST_DATA0            = 0x17c0,
    RADEON_HOST_DATA7            = 0x17dc,
    RADEON_HOST_DATA_LAST        = 0x17e0,
};

/* Radeon PLL register indices (via CLOCK_CNTL_INDEX). */
enum {
    RADEON_PPLL_CNTL     = 0x02,
    RADEON_PPLL_REF_DIV  = 0x03,
    RADEON_PPLL_DIV_0    = 0x04,
    RADEON_PPLL_DIV_1    = 0x05,
    RADEON_PPLL_DIV_2    = 0x06,
    RADEON_PPLL_DIV_3    = 0x07,
    RADEON_VCLK_ECP_CNTL = 0x08,
    RADEON_P2PLL_CNTL    = 0x2a,
    RADEON_P2PLL_REF_DIV = 0x2b,
};

/* CLOCK_CNTL_INDEX. */
#define RADEON_PLL_WR_EN  0x00000080

/* GEN_INT_CNTL / GEN_INT_STATUS. */
#define RADEON_CRTC_VBLANK_INT  0x00000001
#define RADEON_CRTC_VLINE_INT   0x00000002
#define RADEON_CRTC_VSYNC_INT   0x00000004

/* CNFG_CNTL. */
#define RADEON_APER_0_ENDIAN  0x00000003
#define RADEON_APER_1_ENDIAN  0x0000000c

/* CRTC_GEN_CNTL. */
#define RADEON_CRTC_CSYNC_EN         0x00000010
#define RADEON_CRTC_PIX_WIDTH_MASK   0x00000700
#define RADEON_CRTC_PIX_WIDTH_4BPP   0x00000100
#define RADEON_CRTC_PIX_WIDTH_8BPP   0x00000200
#define RADEON_CRTC_PIX_WIDTH_15BPP  0x00000300
#define RADEON_CRTC_PIX_WIDTH_16BPP  0x00000400
#define RADEON_CRTC_PIX_WIDTH_24BPP  0x00000500
#define RADEON_CRTC_PIX_WIDTH_32BPP  0x00000600
#define RADEON_CRTC2_DISPLAY_DIS     0x00800000
#define RADEON_CRTC2_EXT_DISP_EN     0x01000000
#define RADEON_CRTC2_EN              0x02000000
#define RADEON_CRTC2_CUR_EN          0x00010000

/* CRTC_EXT_CNTL. */
#define RADEON_CRT_CRTC_DISPLAY_DIS  0x00000400
#define RADEON_CRT_CRTC_ON           0x00008000

/* DAC_CNTL. */
#define RADEON_DAC_8BIT_EN  0x00000100
#define RADEON_DAC_MASK     0xff000000

/* GEN_RESET_CNTL. */
#define RADEON_SOFT_RESET_GUI  0x00000001
#define RADEON_SOFT_RESET_VCLK 0x00000100
#define RADEON_SOFT_RESET_PCLK 0x00000200
#define RADEON_SOFT_RESET_ECP  0x00000400
#define RADEON_SOFT_RESET_DISPENG_XCLK 0x00000800

/* DP_DATATYPE. */
#define RADEON_DST_8BPP        0x00000002
#define RADEON_DST_15BPP       0x00000003
#define RADEON_DST_16BPP       0x00000004
#define RADEON_DST_24BPP       0x00000005
#define RADEON_DST_32BPP       0x00000006
#define RADEON_DP_DST_DATATYPE 0x0000000f
#define RADEON_DP_BRUSH_DATATYPE 0x00000f00
#define RADEON_SRC_MONO_FRGD_BKGD 0x00000000
#define RADEON_SRC_MONO_FRGD      0x00010000
#define RADEON_SRC_COLOR          0x00030000
#define RADEON_DP_SRC_DATATYPE    0x00030000
#define RADEON_DP_BYTE_PIX_ORDER  0x40000000

/* DP_GUI_MASTER_CNTL. */
#define RADEON_GMC_SRC_PITCH_OFFSET_CNTL 0x00000001
#define RADEON_GMC_DST_PITCH_OFFSET_CNTL 0x00000002
#define RADEON_GMC_SRC_CLIPPING          0x00000004
#define RADEON_GMC_DST_CLIPPING          0x00000008
#define RADEON_GMC_ROP3_MASK             0x00ff0000
#define RADEON_ROP3_MASK                 0x00ff0000
#define RADEON_ROP3_BLACKNESS            0x00000000
#define RADEON_ROP3_SRCCOPY              0x00cc0000
#define RADEON_ROP3_PATCOPY              0x00f00000
#define RADEON_ROP3_WHITENESS            0x00ff0000

/* DP_MIX. */
#define RADEON_DP_SRC_RECT          0x00000200
#define RADEON_DP_SRC_HOST          0x00000300
#define RADEON_DP_SRC_HOST_BYTEALIGN 0x00000400
#define RADEON_DP_SRC_SOURCE        0x00000700
#define RADEON_DP_ROP3              0x00ff0000

/* DP_CNTL. */
#define RADEON_DST_X_RIGHT_TO_LEFT   0x00000000
#define RADEON_DST_X_LEFT_TO_RIGHT   0x00000001
#define RADEON_DST_Y_BOTTOM_TO_TOP   0x00000000
#define RADEON_DST_Y_TOP_TO_BOTTOM   0x00000002
#define RADEON_DST_X_MAJOR           0x00000000
#define RADEON_DST_Y_MAJOR           0x00000004
#define RADEON_DST_HOST_BIG_ENDIAN_EN 0x00000200

#endif // ATI_RADEON_DEFS_H
