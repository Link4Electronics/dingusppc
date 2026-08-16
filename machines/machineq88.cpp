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
#include <devices/memctrl/bootrom.h>
#include <devices/memctrl/intrepid.h>
#include <machines/machine.h>
#include <machines/machinebase.h>
#include <machines/machinefactory.h>
#include <machines/machineproperties.h>

#include <memory>

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

    // allocate ROM region with flash chip emulation (New World: 1 MB at 0xFFF00000)
    BootRomNW* bootrom = dynamic_cast<BootRomNW*>(gMachineObj->get_comp_by_name("BootRomNW"));
    if (!bootrom) {
        LOG_F(ERROR, "BootRomNW device not found!");
        return -1;
    }

    // create and attach the Sharp LH28F008BJT flash chip
    auto flash_chip = std::make_unique<SharpLH28F008BJT>();
    flash_chip->set_controller(bootrom);
    bootrom->flash_chip = flash_chip.release();

    // register the boot ROM as the ROM region device
    bootrom->rom_entry = intrepid_obj->add_rom_region(0xFFF00000, 0x100000, bootrom);
    if (!bootrom->rom_entry) {
        LOG_F(ERROR, "Could not allocate ROM region!");
        return -1;
    }

    // map the "rom" region (DT node rom@ff800000, ranges 0xFF800000 size
    // 0x800000, with boot-rom@fff00000 as the flash sub-node) as writable RAM
    // below the flash. Open Firmware relocates its image and builds its
    // control block (0xFF844C00) here on the real machine; without this the
    // ROM-side code branches into unmapped memory and faults.
    intrepid_obj->add_ram_region(0xFF800000, 0x700000);

    // allocate RAM (DT memory@0: 1 GiB, reg 0x00000000 0x40000000)
    intrepid_obj->setup_ram(GET_INT_PROP("rambank0_size"));

    // Note: NVRAM is now part of the flash chip (addresses 0x4000-0x5FFF and
    // 0x6000-0x7FFF within the 1MB flash). The NVRAM device is kept for file
    // persistence but not registered as MMIO.

    // configure CPU clocks (PowerMac10,2: 1.5 GHz core, 166 MHz bus, 41.67 MHz
    // timebase). The ROM calibrates its own values (bus/timebase) from the
    // mac-io timer@15000 counter, and delay() uses them directly.
    uint64_t core_freq     = 1500000000ULL;
    uint64_t bus_freq      = 166000000ULL;
    uint64_t timebase_freq = 41666666ULL;

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
        new IntProperty(1024, std::vector<uint32_t>({128, 256, 512, 1024}))},
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
        new StrProperty("KeyLargoUSB")},
    {"pci_USB4",
        new StrProperty("")},
    {"pci_ATA",
        new StrProperty("IntrepidAta")},
    {"pci_FireWire",
        new StrProperty("TSB43AB22")},
    {"pci_GMAC",
        new StrProperty("SunGEM")},
    {"hdd_config",
        new StrProperty("Ide0:0")},
    {"cdr_config",
        new StrProperty("Ide0:1")},
    {"serial_backend",
        new StrProperty("stdio", std::vector<std::string>({"null", "stdio", "socket"}))},
};

static std::vector<std::string> q88_devices = {
    "Intrepid",
    "KeyLargo",
    "BootRomNW",
    "NVRAM",
    "Ide0",
    "AtaHardDisk",
    "AtapiCdrom",
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
