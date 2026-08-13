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

/** @file Construct the Q88 machine (Mac mini G4, PowerMac10,1/10,2). */

#include <cpu/ppc/ppcemu.h>
#include <devices/common/nvram.h>
#include <devices/deviceregistry.h>
#include <devices/memctrl/intrepid.h>
#include <machines/machine.h>
#include <machines/machinebase.h>
#include <machines/machinefactory.h>
#include <machines/machineproperties.h>

// UniNorth AGP bus IRQ mapping. IntSrc values map to OpenPIC source numbers
// (see OpenPic::source_number_for_intsrc): GPU = 48.
static const std::vector<PciIrqMap> intrepid_agp_irq_map = {
    {nullptr       , DEV_FUN(0x0B,0)                  }, // UniNorth AGP
    {"pci_AGP_GPU" , DEV_FUN(0x10,0), IntSrc::PCI_GPU }, // Radeon 9200
};

// UniNorth main PCI bus IRQ mapping. KeyLargo USB 27/28/29, NEC USB 63,
// wireless 52.
static const std::vector<PciIrqMap> intrepid_main_irq_map = {
    {nullptr        , DEV_FUN(0x0B,0)                  }, // UniNorth PCI
    {"pci_KeyLargo" , DEV_FUN(0x17,0), IntSrc::INT_UNKNOWN }, // KeyLargo mac-io
    {"pci_Wireless" , DEV_FUN(0x12,0), IntSrc::PCI_WIRELESS }, // BCM4318
    {"pci_USB"      , DEV_FUN(0x18,0), IntSrc::USB_KL0 }, // KeyLargo USB
    {"pci_USB2"     , DEV_FUN(0x19,0), IntSrc::USB_KL1 }, // KeyLargo USB
    {"pci_USB3"     , DEV_FUN(0x1A,0), IntSrc::USB_KL2 }, // KeyLargo USB
    {"pci_USB4"     , DEV_FUN(0x1B,0), IntSrc::NEC_USB }, // NEC USB
};

// UniNorth internal PCI bus IRQ mapping. ata-6 39, FireWire 40, GMAC 41.
static const std::vector<PciIrqMap> intrepid_internal_irq_map = {
    {nullptr        , DEV_FUN(0x0B,0)                  }, // UniNorth Internal PCI
    {"pci_ATA"      , DEV_FUN(0x0D,0), IntSrc::IDE0    }, // ata-6
    {"pci_FireWire" , DEV_FUN(0x0E,0), IntSrc::FIREWIRE }, // FireWire
    {"pci_GMAC"     , DEV_FUN(0x0F,0), IntSrc::ETHERNET }, // GMAC
};

class MachineQ88 : public Machine {
public:
    int initialize(const std::string &id);
};

int MachineQ88::initialize(const std::string &id) {
    LOG_F(INFO, "Building machine Q88 (Mac mini G4)...");

    // get pointer to the memory controller/primary PCI bridge object
    Intrepid* intrepid_obj = dynamic_cast<Intrepid*>(gMachineObj->get_comp_by_name("Intrepid"));
    intrepid_obj->set_irq_map(intrepid_main_irq_map);
    intrepid_obj->set_agp_irq_map(intrepid_agp_irq_map);
    intrepid_obj->set_internal_irq_map(intrepid_internal_irq_map);

    // register KeyLargo mac-io at 00:17.0 on the main PCI bus
    PCIBase* keylargo = dynamic_cast<PCIBase*>(gMachineObj->get_comp_by_name("KeyLargo"));
    if (!keylargo) {
        LOG_F(ERROR, "KeyLargo device not found!");
        return -1;
    }
    intrepid_obj->pci_register_device(DEV_FUN(0x17,0), keylargo);

    // allocate ROM region (New World: 1 MB at 0xFFF00000)
    if (!intrepid_obj->add_rom_region(0xFFF00000, 0x100000)) {
        LOG_F(ERROR, "Could not allocate ROM region!");
        return -1;
    }

    // allocate RAM
    intrepid_obj->setup_ram(GET_INT_PROP("rambank0_size"));

    // register NVRAM (nvram@fff04000, 8 KB, shadows the ROM window) as an
    // MMIO region; MemCtrlBase gives it precedence over the ROM region
    NVram* nvram = dynamic_cast<NVram*>(gMachineObj->get_comp_by_name("NVRAM"));
    if (!nvram) {
        LOG_F(ERROR, "NVRAM device not found!");
        return -1;
    }
    intrepid_obj->add_mmio_region(0xFFF04000, 0x2000, nvram);

    // configure CPU clocks (PowerMac10,2: 1.5 GHz core, 166 MHz bus)
    // NOTE: ROM delay() assumes timebase = r24 = 0x13A00008 = 329252872 Hz
    // (set at fff03020, not recalibrated for our 41.67 MHz TB), so delays run
    // 8x over their intended duration with the real timebase. Use the ROM's
    // assumed TB so delay(x) == x milliseconds, matching real-hardware timing.
    uint64_t core_freq     = 1500000000ULL;
    uint64_t bus_freq      = 166000000ULL;
    uint64_t timebase_freq = 329252872ULL;

    // initialize virtual CPU and request a G4 (MPC7447A) CPU
    ppc_cpu_init(intrepid_obj, {
        .version = PPC_VER::MPC7447A,
        .timebase_freq_hz = timebase_freq,
        .bus_freq_hz = bus_freq,
        .core_freq_hz = core_freq,
    });

    return 0;
}

static const PropMap q88_settings = {
    {"rambank0_size",
        new IntProperty(512, std::vector<uint32_t>({128, 256, 512, 1024}))},
    {"pci_AGP_GPU",
        new StrProperty("AtiRadeonRV280")},
    {"pci_KeyLargo",
        new StrProperty("")},
    {"pci_Wireless",
        new StrProperty("")},
    {"pci_USB",
        new StrProperty("")},
    {"pci_USB2",
        new StrProperty("")},
    {"pci_USB3",
        new StrProperty("")},
    {"pci_USB4",
        new StrProperty("")},
    {"pci_ATA",
        new StrProperty("")},
    {"pci_FireWire",
        new StrProperty("")},
    {"pci_GMAC",
        new StrProperty("")},
};

static std::vector<std::string> q88_devices = {
    "Intrepid",
    "KeyLargo",
    "NVRAM",
};

static const DeviceDescription MachineQ88_descriptor = {
    Machine::create<MachineQ88>, q88_devices, q88_settings
};

REGISTER_DEVICE(MachineQ88, MachineQ88_descriptor);

static const MachineDescription q88_descriptor = {
    .name = "Q88",
    .description = "Mac mini G4",
    .machine_root = "MachineQ88",
};

REGISTER_MACHINE(q88, q88_descriptor);
