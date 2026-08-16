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

/** @file UniNorth 2 GMAC (Sun GEM) ethernet controller emulation.

    Register-level port of QEMU's hw/net/sungem.c for the UniNorth 2
    "Intrepid" GMAC cell (internal PCI bus device 0x0F, BAR0 2 MiB,
    OpenPIC interrupt 41). All GEM registers are little-endian 32-bit
    words; the dingusppc MMIO framework uses big-endian byte-lane
    semantics, so registers are stored in little-endian byte order and
    byteswapped at the BAR boundary.

    There is no host network backend: TX descriptors are read from the
    ring and the frames discarded (completion is signalled via the
    GREG_STAT_TXNR field, TXDONE/TXALL status bits and the TXINTME
    interrupt), and RX never delivers packets. That is enough for the
    boot ROM / Open Firmware probe and for the Linux sungem driver to
    configure the interface and report a link. */

#include <core/endianswap.h>
#include <core/memaccess.h>
#include <cpu/ppc/ppcmmu.h>
#include <devices/common/hwcomponent.h>
#include <devices/common/hwinterrupt.h>
#include <devices/common/mmiodevice.h>
#include <devices/common/pci/pcidevice.h>
#include <devices/common/pci/pcihost.h>
#include <devices/deviceregistry.h>
#include <devices/ethernet/sungem.h>
#include <devices/memctrl/memctrlbase.h>

#include <cinttypes>
#include <loguru.hpp>

using namespace std;

// ---------------------------------------------------------------------------
// Register definitions (offsets relative to the BAR; QEMU hw/net/sungem.c and
// Linux drivers/net/ethernet/sun/sungem.h)
// ---------------------------------------------------------------------------

// Global registers (BAR + 0x0000)
constexpr uint32_t GREG_SEBSTATE  = 0x0000;
constexpr uint32_t GREG_STAT      = 0x000C;
constexpr uint32_t GREG_STAT_TXINTME   = 0x00000001;
constexpr uint32_t GREG_STAT_TXALL     = 0x00000002;
constexpr uint32_t GREG_STAT_TXDONE    = 0x00000004;
constexpr uint32_t GREG_STAT_RXDONE    = 0x00000010;
constexpr uint32_t GREG_STAT_RXNOBUF   = 0x00000020;
constexpr uint32_t GREG_STAT_RXTAGERR  = 0x00000040;
constexpr uint32_t GREG_STAT_TXMAC     = 0x00004000;
constexpr uint32_t GREG_STAT_RXMAC     = 0x00008000;
constexpr uint32_t GREG_STAT_MAC       = 0x00010000;
constexpr uint32_t GREG_STAT_TXNR      = 0xfff80000;
constexpr uint32_t GREG_STAT_TXNR_SHIFT = 19;
constexpr uint32_t GREG_STAT_LATCH     = GREG_STAT_TXALL | GREG_STAT_TXINTME |
                                         GREG_STAT_RXDONE | GREG_STAT_RXNOBUF |
                                         GREG_STAT_RXTAGERR;
constexpr uint32_t GREG_IMASK    = 0x0010;
constexpr uint32_t GREG_IACK     = 0x0014;
constexpr uint32_t GREG_STAT2    = 0x001C;
constexpr uint32_t GREG_PCIESTAT = 0x1000;
constexpr uint32_t GREG_PCIEMASK = 0x1004;
constexpr uint32_t GREG_BIFCFG   = 0x1008;
constexpr uint32_t GREG_BIFDIAG  = 0x100C;
constexpr uint32_t GREG_SWRST    = 0x1010;
constexpr uint32_t GREG_SWRST_TXRST  = 0x00000001;
constexpr uint32_t GREG_SWRST_RXRST  = 0x00000002;
constexpr uint32_t GREG_SWRST_RSTOUT = 0x00000004;

// TX DMA registers (BAR + 0x2000)
constexpr uint32_t TXDMA_KICK    = 0x0000;
constexpr uint32_t TXDMA_CFG     = 0x0004;
constexpr uint32_t TXDMA_CFG_ENABLE = 0x00000001;
constexpr uint32_t TXDMA_DBLOW   = 0x0008;
constexpr uint32_t TXDMA_DBHI    = 0x000C;
constexpr uint32_t TXDMA_PCNT    = 0x0024;
constexpr uint32_t TXDMA_SMACHINE = 0x0028;
constexpr uint32_t TXDMA_DPLOW   = 0x0030;
constexpr uint32_t TXDMA_DPHI    = 0x0034;
constexpr uint32_t TXDMA_TXDONE  = 0x0100;
constexpr uint32_t TXDMA_FTAG    = 0x0108;
constexpr uint32_t TXDMA_FSZ     = 0x0118;

// RX DMA registers (BAR + 0x4000)
constexpr uint32_t RXDMA_CFG     = 0x0000;
constexpr uint32_t RXDMA_CFG_ENABLE = 0x00000001;
constexpr uint32_t RXDMA_DBLOW   = 0x0004;
constexpr uint32_t RXDMA_DBHI    = 0x0008;
constexpr uint32_t RXDMA_PCNT    = 0x0018;
constexpr uint32_t RXDMA_SMACHINE = 0x001C;
constexpr uint32_t RXDMA_PTHRESH = 0x0020;
constexpr uint32_t RXDMA_DPLOW   = 0x0024;
constexpr uint32_t RXDMA_DPHI    = 0x0028;
constexpr uint32_t RXDMA_KICK    = 0x0100;
constexpr uint32_t RXDMA_DONE    = 0x0104;
constexpr uint32_t RXDMA_BLANK   = 0x0108;
constexpr uint32_t RXDMA_FTAG    = 0x0110;
constexpr uint32_t RXDMA_FSZ     = 0x0120;

// MAC registers (BAR + 0x6000)
constexpr uint32_t MAC_TXRST     = 0x0000;
constexpr uint32_t MAC_RXRST     = 0x0004;
constexpr uint32_t MAC_TXSTAT    = 0x0010;
constexpr uint32_t MAC_RXSTAT    = 0x0014;
constexpr uint32_t MAC_CSTAT     = 0x0018;
constexpr uint32_t MAC_CSTAT_PTR = 0xffff0000;
constexpr uint32_t MAC_TXMASK    = 0x0020;
constexpr uint32_t MAC_RXMASK    = 0x0024;
constexpr uint32_t MAC_MCMASK    = 0x0028;
constexpr uint32_t MAC_TXCFG     = 0x0030;
constexpr uint32_t MAC_TXCFG_ENAB = 0x00000001;
constexpr uint32_t MAC_RXCFG     = 0x0034;
constexpr uint32_t MAC_XIFCFG    = 0x003C;
constexpr uint32_t MAC_MINFSZ    = 0x0050;
constexpr uint32_t MAC_MAXFSZ    = 0x0054;
constexpr uint32_t MAC_ADDR0     = 0x0080;
constexpr uint32_t MAC_ADDR1     = 0x0084;
constexpr uint32_t MAC_ADDR2     = 0x0088;
constexpr uint32_t MAC_ADDR3     = 0x008C;
constexpr uint32_t MAC_ADDR4     = 0x0090;
constexpr uint32_t MAC_ADDR5     = 0x0094;
constexpr uint32_t MAC_HASH0     = 0x00C0;
constexpr uint32_t MAC_PATMPS    = 0x0114;
constexpr uint32_t MAC_SMACHINE  = 0x0134;

// MIF (MDIO) registers (BAR + 0x6200)
constexpr uint32_t MIF_FRAME     = 0x000C;
constexpr uint32_t MIF_FRAME_OP      = 0x30000000;
constexpr uint32_t MIF_FRAME_PHYAD   = 0x0f800000;
constexpr uint32_t MIF_FRAME_REGAD   = 0x007c0000;
constexpr uint32_t MIF_FRAME_TALSB   = 0x00010000;
constexpr uint32_t MIF_FRAME_DATA    = 0x0000ffff;
constexpr uint32_t MIF_CFG       = 0x0010;
constexpr uint32_t MIF_CFG_MDI0  = 0x00000100;
constexpr uint32_t MIF_CFG_MDI1  = 0x00000200;
constexpr uint32_t MIF_STATUS    = 0x0018;
constexpr uint32_t MIF_SMACHINE  = 0x001C;

// PCS registers (BAR + 0x9000)
constexpr uint32_t PCS_MIISTAT   = 0x0004;
constexpr uint32_t PCS_ISTAT     = 0x0018;
constexpr uint32_t PCS_SSTATE    = 0x005C;

// WOL block (BAR + 0x3000)
constexpr uint32_t SUNGEM_MMIO_WOL_SIZE = 0x14;

// TX descriptor
constexpr uint64_t TXDCTRL_BUFSZ = 0x0000000000007fffULL;
constexpr uint64_t TXDCTRL_SOF   = 0x0000000080000000ULL;
constexpr uint64_t TXDCTRL_EOF   = 0x0000000040000000ULL;
constexpr uint64_t TXDCTRL_INTME = 0x0000000100000000ULL;

constexpr auto SUNGEM_DEV_ID = 0x0032; // UniNorth 2 GMAC
constexpr uint32_t SUNGEM_BAR_SIZE = 0x200000; // 2 MiB

// Primitive BCM5201-style PHY on MDIO address 0 (link always up)
constexpr uint8_t  PHY_ADDR       = 0;
constexpr uint16_t PHY_ID1        = 0x0040;
constexpr uint16_t PHY_ID2        = 0x6210;
constexpr uint32_t MII_BMCR   = 0;
constexpr uint32_t MII_BMSR   = 1;
constexpr uint32_t MII_PHYID1 = 2;
constexpr uint32_t MII_PHYID2 = 3;
constexpr uint32_t MII_ANAR   = 4;
constexpr uint32_t MII_ANLPAR = 5;
constexpr uint16_t MII_BMSR_100TX_FD = 0x0080;
constexpr uint16_t MII_BMSR_AN_COMP  = 0x0020;
constexpr uint16_t MII_BMSR_AUTONEG  = 0x1000;
constexpr uint16_t MII_BMSR_LINK_ST  = 0x0004;
constexpr uint16_t MII_ANLPAR_TXFD   = 0x0020;

SunGEM::SunGEM() : PCIDevice("SunGEM")
{
    this->supports_types(HWCompType::MMIO_DEV | HWCompType::PCI_DEV);

    // PCI configuration space header (ethernet@f on the internal bus)
    this->vendor_id = this->subsys_vndr = PCI_VENDOR_APPLE;
    this->device_id = this->subsys_id   = SUNGEM_DEV_ID;
    this->class_rev = 0x02000050; // network controller, rev 0x50 (rev 80)
    this->irq_pin   = 1;
    this->min_gnt   = 0x40;
    this->max_lat   = 0x40;

    // BAR0: 2 MiB MMIO window (0xf5200000 on the Mac mini G4)
    this->bars_cfg[0] = 0xFFE00000;
    this->finish_config_bars();

    this->pci_notify_bar_change = [this](int bar_num) {
        this->notify_bar_change(bar_num);
    };

    this->reset_all(true);
}

int SunGEM::device_postinit()
{
    return 0;
}

void SunGEM::notify_bar_change(int bar_num)
{
    if (bar_num) // only BAR0 is supported
        return;

    uint32_t new_base = this->bars[bar_num] & 0xFFE00000UL;
    if (new_base != this->base_addr) {
        if (this->base_addr)
            this->host_instance->pci_unregister_mmio_region(this->base_addr,
                SUNGEM_BAR_SIZE, this);
        this->base_addr = new_base;
        if (new_base)
            this->host_instance->pci_register_mmio_region(new_base,
                SUNGEM_BAR_SIZE, this);
        LOG_F(INFO, "%s: BAR0 set to 0x%X", this->name.c_str(), new_base);
    }
}

// ---------------------------------------------------------------------------
// Interrupt / status handling
// ---------------------------------------------------------------------------

void SunGEM::eval_irq()
{
    uint32_t mask = this->gregs[GREG_IMASK >> 2];
    uint32_t stat = this->gregs[GREG_STAT >> 2] & ~GREG_STAT_TXNR;

    this->pci_interrupt((stat & ~mask) ? 1 : 0);
}

void SunGEM::update_status(uint32_t bits, bool val)
{
    if (val)
        this->gregs[GREG_STAT >> 2] |= bits;
    else
        this->gregs[GREG_STAT >> 2] &= ~bits;
    this->eval_irq();
}

void SunGEM::eval_cascade_irq()
{
    uint32_t mask = this->macregs[MAC_TXSTAT >> 2];
    uint32_t stat = this->macregs[MAC_TXMASK >> 2];
    this->update_status(GREG_STAT_TXMAC, (stat & ~mask) != 0);

    mask = this->macregs[MAC_RXSTAT >> 2];
    stat = this->macregs[MAC_RXMASK >> 2];
    this->update_status(GREG_STAT_RXMAC, (stat & ~mask) != 0);

    mask = this->macregs[MAC_MCMASK >> 2];
    stat = this->macregs[MAC_CSTAT >> 2] & ~MAC_CSTAT_PTR;
    this->update_status(GREG_STAT_MAC, (stat & ~mask) != 0);
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

void SunGEM::update_masks()
{
    uint32_t sz = 1U << (((this->rxdmaregs[RXDMA_CFG >> 2] & 0x1E) >> 1) + 5);
    this->rx_mask = sz - 1;

    sz = 1U << (((this->txdmaregs[TXDMA_CFG >> 2] & 0x1E) >> 1) + 5);
    this->tx_mask = sz - 1;
}

void SunGEM::reset_rx()
{
    this->rxdmaregs[RXDMA_FSZ >> 2]      = 0x140;
    this->rxdmaregs[RXDMA_DONE >> 2]     = 0;
    this->rxdmaregs[RXDMA_KICK >> 2]     = 0;
    this->rxdmaregs[RXDMA_CFG >> 2]      = 0x1000010;
    this->rxdmaregs[RXDMA_PTHRESH >> 2]  = 0xf8;
    this->rxdmaregs[RXDMA_BLANK >> 2]    = 0;

    this->update_masks();
}

void SunGEM::reset_tx()
{
    this->txdmaregs[TXDMA_FSZ >> 2]     = 0x90;
    this->txdmaregs[TXDMA_TXDONE >> 2]  = 0;
    this->txdmaregs[TXDMA_KICK >> 2]    = 0;
    this->txdmaregs[TXDMA_CFG >> 2]     = 0x118010;

    this->update_masks();

    this->tx_size = 0;
    this->tx_first_ctl = 0;
}

void SunGEM::reset_all(bool pci_reset)
{
    this->reset_rx();
    this->reset_tx();

    this->gregs[GREG_IMASK >> 2] = 0xFFFFFFF;
    this->gregs[GREG_STAT >> 2]  = 0;
    if (pci_reset) {
        this->gregs[GREG_SWRST >> 2] = 0;
        this->macregs[MAC_ADDR0 >> 2] = (this->mac_addr[4] << 8) | this->mac_addr[5];
        this->macregs[MAC_ADDR1 >> 2] = (this->mac_addr[2] << 8) | this->mac_addr[3];
        this->macregs[MAC_ADDR2 >> 2] = (this->mac_addr[0] << 8) | this->mac_addr[1];
    } else {
        this->gregs[GREG_SWRST >> 2] &= GREG_SWRST_RSTOUT;
    }
    this->mifregs[MIF_CFG >> 2] = MIF_CFG_MDI0;
}

// ---------------------------------------------------------------------------
// TX DMA
// ---------------------------------------------------------------------------

uint64_t SunGEM::dma_read_qword(uint32_t addr)
{
    MapDmaResult res = mmu_map_dma_mem(addr, 8, false);
    if (res.type == RT_NONE)
        return 0;

    uint32_t lo = READ_DWORD_LE_A(res.host_va);
    uint32_t hi = READ_DWORD_LE_A(res.host_va + 4);
    return ((uint64_t)hi << 32) | lo;
}

void SunGEM::tx_kick()
{
    uint32_t txdma_cfg = this->txdmaregs[TXDMA_CFG >> 2];
    uint32_t txmac_cfg = this->macregs[MAC_TXCFG >> 2];
    if (!(txdma_cfg & TXDMA_CFG_ENABLE) || !(txmac_cfg & MAC_TXCFG_ENAB)) {
        LOG_F(INFO, "%s: TX kick while disabled (cfg=%X mac=%X)", this->name.c_str(),
              txdma_cfg, txmac_cfg);
        return;
    }

    uint64_t dbase = ((uint64_t)this->txdmaregs[TXDMA_DBHI >> 2] << 32) |
                     this->txdmaregs[TXDMA_DBLOW >> 2];
    uint32_t comp = this->txdmaregs[TXDMA_TXDONE >> 2] & this->tx_mask;
    uint32_t kick = this->txdmaregs[TXDMA_KICK >> 2] & this->tx_mask;

    LOG_F(INFO, "%s: TX kick comp=%u kick=%u", this->name.c_str(), comp, kick);

    while (comp != kick) {
        uint64_t desc_addr = dbase + (uint64_t)comp * 16;
        uint64_t control_word = this->dma_read_qword(desc_addr);
        uint64_t buffer       = this->dma_read_qword(desc_addr + 8);

        // assemble (and discard) the current frame
        if (control_word & TXDCTRL_SOF) {
            if (this->tx_first_ctl)
                LOG_F(WARNING, "%s: TX frame was not finished", this->name.c_str());
            this->tx_size = 0;
            this->tx_first_ctl = control_word;
        }
        this->tx_size += control_word & TXDCTRL_BUFSZ;
        if (control_word & TXDCTRL_EOF) {
            LOG_F(INFO, "%s: TX frame size=%u (dropped, no NIC backend)",
                  this->name.c_str(), this->tx_size);
            this->tx_size = 0;
            this->tx_first_ctl = 0;
        }

        uint32_t ints = GREG_STAT_TXDONE;
        if (control_word & TXDCTRL_INTME)
            ints |= GREG_STAT_TXINTME;
        this->update_status(ints, true);

        comp = (comp + 1) & this->tx_mask;
        this->txdmaregs[TXDMA_TXDONE >> 2] = comp;
    }

    this->update_status(GREG_STAT_TXALL, true);
}

// ---------------------------------------------------------------------------
// MII (MDIO) PHY
// ---------------------------------------------------------------------------

uint16_t SunGEM::mii_read(uint8_t phy_addr, uint8_t reg_addr)
{
    if (phy_addr != PHY_ADDR)
        return 0xFFFF;

    switch (reg_addr) {
    case MII_BMCR:
        return 0;
    case MII_PHYID1:
        return PHY_ID1;
    case MII_PHYID2:
        return PHY_ID2;
    case MII_BMSR:
        return MII_BMSR_100TX_FD | MII_BMSR_AN_COMP |
               MII_BMSR_AUTONEG | MII_BMSR_LINK_ST;
    case MII_ANAR:
    case MII_ANLPAR:
        return MII_ANLPAR_TXFD;
    case 0x18: // BCM5201 aux status: 100FD
        return 3;
    default:
        return 0;
    }
}

void SunGEM::mii_write(uint8_t phy_addr, uint8_t reg_addr, uint16_t val)
{
    // no writable PHY state
}

uint32_t SunGEM::mii_op(uint32_t val)
{
    if ((val >> 30) != 1) { // not start-of-frame
        LOG_F(WARNING, "%s: invalid MII frame SOF=%u", this->name.c_str(), val >> 30);
        return 0xFFFF;
    }

    uint8_t phy_addr = (val & MIF_FRAME_PHYAD) >> 23;
    uint8_t reg_addr = (val & MIF_FRAME_REGAD) >> 18;
    uint8_t op       = (val & MIF_FRAME_OP) >> 28;

    switch (op) {
    case 1: // write
        this->mii_write(phy_addr, reg_addr, val & MIF_FRAME_DATA);
        return val | MIF_FRAME_TALSB;
    case 2: // read
        return this->mii_read(phy_addr, reg_addr) | MIF_FRAME_TALSB;
    default:
        LOG_F(WARNING, "%s: invalid MII opcode %u", this->name.c_str(), op);
        break;
    }
    return 0xFFFF | MIF_FRAME_TALSB;
}

// ---------------------------------------------------------------------------
// MMIO read/write
//
// All GEM registers are little-endian. The framework hands us / expects the
// value in the CPU's (big-endian) byte order with partial accesses aligned
// to the LSB, so:
//   - read:  BYTESWAP_32(reg) >> (8 * (4 - size))
//   - write: size >= 4 -> reg = BYTESWAP_32(value)
//            size <  4 -> the low 'size' bytes of 'value' are written in
//            byte-reversed order into the low bytes of the register.
// ---------------------------------------------------------------------------

static uint32_t le_merge(uint32_t reg, uint32_t value, int size)
{
    if (size >= 4)
        return BYTESWAP_32(value);

    uint32_t mask = (1U << (8 * size)) - 1;
    uint32_t merged = 0;
    for (int i = 0; i < size; i++)
        merged |= ((value >> (8 * i)) & 0xFF) << (8 * (size - 1 - i));
    return (reg & ~mask) | (merged & mask);
}

uint32_t SunGEM::read(uint32_t rgn_start, uint32_t offset, int size)
{
    uint32_t val = 0;

    if (offset < 0x2000) {
        // global registers
        if (offset < 0x20 || (offset >= 0x1000 && offset <= 0x1010)) {
            uint32_t idx = offset >> 2;
            val = this->gregs[idx];
            if (offset == GREG_STAT) {
                // reading clears the latched bits and injects the TX
                // completion index into the TXNR field
                this->gregs[idx] &= ~GREG_STAT_LATCH;
                this->eval_irq();
                val = (val & ~GREG_STAT_TXNR) |
                      ((this->txdmaregs[TXDMA_TXDONE >> 2] & this->tx_mask)
                       << GREG_STAT_TXNR_SHIFT);
            } else if (offset == GREG_STAT2) {
                // same, but no side effect
                val = (this->gregs[idx] & ~GREG_STAT_TXNR) |
                      ((this->txdmaregs[TXDMA_TXDONE >> 2] & this->tx_mask)
                       << GREG_STAT_TXNR_SHIFT);
            }
        }
    } else if (offset < 0x3000) {
        // TX DMA block
        uint32_t off = offset - 0x2000;
        if (off < 0x38 || (off >= 0x100 && off <= 0x118))
            val = this->txdmaregs[off >> 2];
    } else if (offset < 0x4000) {
        // WOL block: unsupported, read 0xFFFFFFFF like QEMU
        val = 0xFFFFFFFF;
    } else if (offset < 0x6000) {
        // RX DMA block
        uint32_t off = offset - 0x4000;
        if (off <= 0x28 || (off >= 0x100 && off <= 0x120))
            val = this->rxdmaregs[off >> 2];
    } else if (offset < 0x6200) {
        // MAC block
        uint32_t off = offset - 0x6000;
        if (off <= 0x134) {
            val = this->macregs[off >> 2];
            switch (off) {
            case MAC_TXSTAT:
                this->macregs[off >> 2] = 0;
                this->update_status(GREG_STAT_TXMAC, false);
                break;
            case MAC_RXSTAT:
                this->macregs[off >> 2] = 0;
                this->update_status(GREG_STAT_RXMAC, false);
                break;
            case MAC_CSTAT:
                this->macregs[off >> 2] &= MAC_CSTAT_PTR;
                this->update_status(GREG_STAT_MAC, false);
                break;
            }
        }
    } else if (offset < 0x6400) {
        // MIF block
        uint32_t off = offset - 0x6200;
        if (off <= 0x1C)
            val = this->mifregs[off >> 2];
    } else if (offset < 0x9000) {
        // unused gap
        val = 0;
    } else if (offset < 0x9060) {
        // PCS block
        uint32_t off = offset - 0x9000;
        if (off <= 0x18 || (off >= 0x50 && off <= 0x5C))
            val = this->pcsregs[off >> 2];
    } else {
        // rest of the 2 MiB BAR
        val = 0;
    }

    if (size >= 4)
        return BYTESWAP_32(val);
    return BYTESWAP_32(val) >> (8 * (4 - size));
}

void SunGEM::write(uint32_t rgn_start, uint32_t offset, uint32_t value, int size)
{
    if (offset < 0x2000) {
        // global registers
        if (!(offset < 0x20 || (offset >= 0x1000 && offset <= 0x1010)))
            return;
        switch (offset) {
        case GREG_SEBSTATE:
        case GREG_STAT:
        case GREG_STAT2:
        case GREG_PCIESTAT:
            return; // read-only
        case GREG_IACK:
            // writing a latched bit clears it
            value &= GREG_STAT_LATCH;
            this->gregs[GREG_STAT >> 2] &= ~value;
            this->eval_irq();
            return;
        case GREG_PCIEMASK:
            value &= 0x7;
            break;
        }
        this->gregs[offset >> 2] = le_merge(this->gregs[offset >> 2], value, size);
        switch (offset) {
        case GREG_IMASK:
            this->eval_irq();
            break;
        case GREG_SWRST:
            switch (value & (GREG_SWRST_TXRST | GREG_SWRST_RXRST)) {
            case GREG_SWRST_RXRST:
                this->reset_rx();
                break;
            case GREG_SWRST_TXRST:
                this->reset_tx();
                break;
            case GREG_SWRST_TXRST | GREG_SWRST_RXRST:
                this->reset_all(false);
                break;
            }
            break;
        }
        return;
    }

    if (offset < 0x3000) {
        // TX DMA block
        uint32_t off = offset - 0x2000;
        if (off < 0x38 || (off >= 0x100 && off <= 0x118)) {
            switch (off) {
            case TXDMA_TXDONE:
            case TXDMA_PCNT:
            case TXDMA_SMACHINE:
            case TXDMA_DPLOW:
            case TXDMA_DPHI:
            case TXDMA_FSZ:
            case TXDMA_FTAG:
                return; // read-only
            }
            this->txdmaregs[off >> 2] = le_merge(this->txdmaregs[off >> 2], value, size);
            switch (off) {
            case TXDMA_KICK:
                this->tx_kick();
                break;
            case TXDMA_CFG:
                this->update_masks();
                break;
            }
        }
        return;
    }

    if (offset < 0x4000) {
        // WOL block: unsupported
        LOG_F(WARNING, "%s: WOL write @0x%X = 0x%X (unsupported)",
              this->name.c_str(), offset, value);
        return;
    }

    if (offset < 0x6000) {
        // RX DMA block
        uint32_t off = offset - 0x4000;
        if (off <= 0x28 || (off >= 0x100 && off <= 0x120)) {
            switch (off) {
            case RXDMA_DONE:
            case RXDMA_PCNT:
            case RXDMA_SMACHINE:
            case RXDMA_DPLOW:
            case RXDMA_DPHI:
            case RXDMA_FSZ:
            case RXDMA_FTAG:
                return; // read-only
            }
            this->rxdmaregs[off >> 2] = le_merge(this->rxdmaregs[off >> 2], value, size);
            switch (off) {
            case RXDMA_CFG:
                this->update_masks();
                break;
            case RXDMA_KICK:
                LOG_F(INFO, "%s: RX kick = 0x%X", this->name.c_str(), value);
                break;
            }
        }
        return;
    }

    if (offset < 0x6200) {
        // MAC block
        uint32_t off = offset - 0x6000;
        if (off <= 0x134) {
            switch (off) {
            case MAC_TXRST:
            case MAC_RXRST:
            case MAC_TXSTAT:
            case MAC_RXSTAT:
            case MAC_CSTAT:
            case MAC_PATMPS:
            case MAC_SMACHINE:
                return; // read-only
            }
            if (off == MAC_MINFSZ)
                value &= 0x3ff;
            this->macregs[off >> 2] = le_merge(this->macregs[off >> 2], value, size);
            switch (off) {
            case MAC_TXMASK:
            case MAC_RXMASK:
            case MAC_MCMASK:
                this->eval_cascade_irq();
                break;
            case MAC_RXCFG:
                this->update_masks();
                break;
            }
        }
        return;
    }

    if (offset < 0x6400) {
        // MIF block
        uint32_t off = offset - 0x6200;
        if (off <= 0x1C) {
            switch (off) {
            case MIF_STATUS:
            case MIF_SMACHINE:
                return; // read-only
            }
            if (off == MIF_CFG) {
                // keep the RO MDIO-present bits advertising an MDIO0 PHY
                value &= ~MIF_CFG_MDI1;
                value |= MIF_CFG_MDI0;
            }
            this->mifregs[off >> 2] = le_merge(this->mifregs[off >> 2], value, size);
            if (off == MIF_FRAME)
                this->mifregs[off >> 2] = this->mii_op(this->mifregs[off >> 2]);
        }
        return;
    }

    if (offset < 0x9060) {
        // PCS block
        uint32_t off = offset - 0x9000;
        if (off <= 0x18 || (off >= 0x50 && off <= 0x5C)) {
            switch (off) {
            case PCS_MIISTAT:
            case PCS_ISTAT:
            case PCS_SSTATE:
                return; // read-only
            }
            this->pcsregs[off >> 2] = le_merge(this->pcsregs[off >> 2], value, size);
        }
    }
}

static const DeviceDescription SunGEM_Descriptor = {
    SunGEM::create, {}, {},
    HWCompType::MMIO_DEV | HWCompType::PCI_DEV
};

REGISTER_DEVICE(SunGEM, SunGEM_Descriptor);
