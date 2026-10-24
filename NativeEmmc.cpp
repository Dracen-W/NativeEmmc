// SPDX-License-Identifier: BSD-2-Clause-Patent
// MediaTek MT6768 MSDC/eMMC bring-up for the Xiaomi Redmi Note 9 (merlinx).
//
// The first milestone is deliberately read-only and passive: it maps the
// controller UEFI left running and issues one CMD17 against the iPadOS
// userdata partition. Nothing is reset, initialized or written. The probe is
// off unless native-emmc-probe is present in the boot arguments.
#include <IOKit/IOService.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOMemoryMap.h>
#include <libkern/libkern.h>
#include <pexpert/pexpert.h>
#include <kern/debug.h>

extern "C" volatile uint32_t native_emmc_probe_calls;
extern "C" volatile uint32_t native_emmc_matched;
extern "C" volatile uint32_t native_emmc_probe_requested;

volatile uint32_t native_emmc_probe_calls = 0;
volatile uint32_t native_emmc_matched = 0;
volatile uint32_t native_emmc_probe_requested = 0;

namespace {

constexpr uint64_t kMsdcBase = 0x11230000ULL;
constexpr uint64_t kMsdcTop = 0x11CD0000ULL;
constexpr uint64_t kMsdcSize = 0x1000ULL;

constexpr uint32_t MSDC_CFG = 0x000;
constexpr uint32_t MSDC_PS = 0x008;
constexpr uint32_t MSDC_INT = 0x00c;
constexpr uint32_t MSDC_INTEN = 0x010;
constexpr uint32_t MSDC_FIFOCS = 0x014;
constexpr uint32_t MSDC_RXDATA = 0x01c;
constexpr uint32_t SDC_CFG = 0x030;
constexpr uint32_t SDC_CMD = 0x034;
constexpr uint32_t SDC_ARG = 0x038;
constexpr uint32_t SDC_STS = 0x03c;
constexpr uint32_t SDC_RESP0 = 0x040;
constexpr uint32_t SDC_BLK_NUM = 0x050;
constexpr uint32_t SDC_CSTS = 0x058;
constexpr uint32_t EMMC_CFG0 = 0x070;

constexpr uint32_t MSDC_CFG_MODE = 1U << 0;
constexpr uint32_t MSDC_CFG_PIO = 1U << 3;
constexpr uint32_t MSDC_INT_CMDRDY = 1U << 8;
constexpr uint32_t MSDC_INT_CMDTMO = 1U << 9;
constexpr uint32_t MSDC_INT_CMDCRCERR = 1U << 10;
constexpr uint32_t MSDC_INT_XFER_COMPL = 1U << 12;
constexpr uint32_t MSDC_INT_DATTMO = 1U << 14;
constexpr uint32_t MSDC_INT_DATCRCERR = 1U << 15;
constexpr uint32_t MSDC_INT_ERROR = MSDC_INT_CMDTMO | MSDC_INT_CMDCRCERR |
                                    MSDC_INT_DATTMO | MSDC_INT_DATCRCERR;
constexpr uint32_t MSDC_FIFOCS_CLR = 1U << 31;
constexpr uint32_t MSDC_FIFOCS_RX_MASK = 0xff;
constexpr uint32_t SDC_CMD_RSP_TYPE_SHIFT = 7;
constexpr uint32_t SDC_CMD_SINGLE_BLK = 1U << 11;
constexpr uint32_t SDC_CMD_BLK_SIZE_SHIFT = 16;
constexpr uint32_t SDC_STS_BUSY = 1U << 0;

constexpr uint32_t kBlockSize = 512;
// p45 starts at 20332544 512-byte sectors in the eMMC user area. The root
// manifest is at the beginning of the iPadOS userdata image.
constexpr uint64_t kUserdataLba = 20332544ULL;

static inline uint32_t read32(uintptr_t base, uint32_t offset)
{
  return *reinterpret_cast<volatile uint32_t *>(base + offset);
}

static inline void write32(uintptr_t base, uint32_t offset, uint32_t value)
{
  *reinterpret_cast<volatile uint32_t *>(base + offset) = value;
}

static uintptr_t mapPhys(uint64_t phys, uint64_t length, OSSharedPtr<IOMemoryMap> &keep)
{
  IODeviceMemory *memory = IODeviceMemory::withRange(phys, length);
  if (!memory) {
    return 0;
  }
  keep = OSSharedPtr<IOMemoryMap>(memory->map(kIOMapAnywhere | kIOMapInhibitCache), OSNoRetain);
  memory->release();
  return keep ? static_cast<uintptr_t>(keep->getVirtualAddress()) : 0;
}

struct Registers {
  uint32_t cfg;
  uint32_t ps;
  uint32_t intStatus;
  uint32_t fifo;
  uint32_t sdcCfg;
  uint32_t sdcSts;
  uint32_t csts;
  uint32_t emmcCfg0;
};

static Registers snapshot(uintptr_t base)
{
  Registers r;
  r.cfg = read32(base, MSDC_CFG);
  r.ps = read32(base, MSDC_PS);
  r.intStatus = read32(base, MSDC_INT);
  r.fifo = read32(base, MSDC_FIFOCS);
  r.sdcCfg = read32(base, SDC_CFG);
  r.sdcSts = read32(base, SDC_STS);
  r.csts = read32(base, SDC_CSTS);
  r.emmcCfg0 = read32(base, EMMC_CFG0);
  return r;
}

static bool waitCommand(uintptr_t base, uint32_t *status)
{
  for (unsigned i = 0; i < 200000; i++) {
    *status = read32(base, MSDC_INT);
    if (*status & (MSDC_INT_CMDRDY | MSDC_INT_ERROR)) {
      return true;
    }
    IODelay(1);
  }
  return false;
}

static void clearStatus(uintptr_t base, uint32_t bits)
{
  if (bits) {
    write32(base, MSDC_INT, bits);
  }
}

// Read one 512-byte block at an absolute eMMC user-area LBA. The controller is
// not reinitialized: this is the same PIO path UEFI used, with interrupts
// masked and completion polled.
static bool readBlock(uintptr_t base, uint64_t lba, uint8_t *out, uint32_t *failure)
{
  uint32_t status = 0;

  // A stale FIFO or command status would be read as the new block.
  write32(base, MSDC_INTEN, 0);
  clearStatus(base, 0xffffffffU);
  write32(base, MSDC_FIFOCS, MSDC_FIFOCS_CLR);
  write32(base, MSDC_CFG, read32(base, MSDC_CFG) | MSDC_CFG_MODE | MSDC_CFG_PIO);

  for (unsigned i = 0; i < 200000; i++) {
    if ((read32(base, SDC_STS) & SDC_STS_BUSY) == 0) {
      break;
    }
    IODelay(1);
    if (i == 199999) {
      *failure = 1;
      return false;
    }
  }

  write32(base, SDC_BLK_NUM, 1);
  write32(base, SDC_ARG, static_cast<uint32_t>(lba));
  write32(base, SDC_CMD,
          17U | (1U << SDC_CMD_RSP_TYPE_SHIFT) | SDC_CMD_SINGLE_BLK |
          (kBlockSize << SDC_CMD_BLK_SIZE_SHIFT));

  if (!waitCommand(base, &status)) {
    *failure = 2;
    return false;
  }
  if (status & MSDC_INT_ERROR) {
    *failure = 3;
    return false;
  }
  clearStatus(base, MSDC_INT_CMDRDY);

  uint32_t got = 0;
  unsigned idle = 0;
  while (got < kBlockSize) {
    status = read32(base, MSDC_INT);
    if (status & MSDC_INT_ERROR) {
      *failure = 4;
      return false;
    }
    uint32_t available = read32(base, MSDC_FIFOCS) & MSDC_FIFOCS_RX_MASK;
    while (available >= 4 && got < kBlockSize) {
      uint32_t word = read32(base, MSDC_RXDATA);
      out[got + 0] = static_cast<uint8_t>(word);
      out[got + 1] = static_cast<uint8_t>(word >> 8);
      out[got + 2] = static_cast<uint8_t>(word >> 16);
      out[got + 3] = static_cast<uint8_t>(word >> 24);
      got += 4;
      available -= 4;
    }
    if (got == kBlockSize) {
      break;
    }
    if (++idle > 200000) {
      *failure = 5;
      return false;
    }
    IODelay(1);
  }

  clearStatus(base, status & (MSDC_INT_XFER_COMPL | MSDC_INT_ERROR));
  *failure = 0;
  return true;
}

} // namespace

class NativeEmmc : public IOService {
  OSDeclareDefaultStructors(NativeEmmc)
public:
  IOService *probe(IOService *provider, SInt32 *score) override;
  bool start(IOService *provider) override;
  void stop(IOService *provider) override;
};

OSDefineMetaClassAndStructors(NativeEmmc, IOService)

IOService *NativeEmmc::probe(IOService *provider, SInt32 *score)
{
  native_emmc_probe_calls = native_emmc_probe_calls + 1;
  if (!OSDynamicCast(IOPlatformExpertDevice, provider)) {
    return nullptr;
  }
  native_emmc_matched = 1;
  if (score) {
    *score = 500;
  }
  return this;
}

bool NativeEmmc::start(IOService *provider)
{
  if (!super::start(provider)) {
    return false;
  }

  unsigned probe = 0;
  PE_parse_boot_argn("native-emmc-probe", &probe, sizeof(probe));
  if (!probe) {
    return true;
  }
  native_emmc_probe_requested = probe;

  OSSharedPtr<IOMemoryMap> baseMap;
  OSSharedPtr<IOMemoryMap> topMap;
  uintptr_t base = mapPhys(kMsdcBase, kMsdcSize, baseMap);
  uintptr_t top = mapPhys(kMsdcTop, kMsdcSize, topMap);
  if (!base || !top) {
    panic("NATIVE_EMMC map failed base=%p top=%p", (void *)base, (void *)top);
  }

  Registers r = snapshot(base);
  if (probe == 1) {
    panic("NATIVE_EMMC registers\n"
          "cfg=%08x ps=%08x int=%08x fifo=%08x\n"
          "sdccfg=%08x sdcsts=%08x csts=%08x emmccfg0=%08x\n"
          "top0=%08x top1=%08x lba=%llu",
          r.cfg, r.ps, r.intStatus, r.fifo, r.sdcCfg, r.sdcSts, r.csts, r.emmcCfg0,
          read32(top, 0), read32(top, 4), (unsigned long long)kUserdataLba);
  }

  uint8_t block[kBlockSize] = {};
  uint32_t failure = 0;
  bool ok = readBlock(base, kUserdataLba, block, &failure);
  if (!ok) {
    panic("NATIVE_EMMC read failed failure=%u\n"
          "cfg=%08x ps=%08x int=%08x fifo=%08x\n"
          "sdccfg=%08x sdcsts=%08x csts=%08x emmccfg0=%08x\n"
          "resp0=%08x lba=%llu",
          failure, r.cfg, r.ps, r.intStatus, r.fifo, r.sdcCfg, r.sdcSts, r.csts, r.emmcCfg0,
          read32(base, SDC_RESP0), (unsigned long long)kUserdataLba);
  }

  bool magic = true;
  const char expected[8] = { 'S', 'H', 'N', 'G', 'R', 'O', 'O', 'T' };
  for (unsigned i = 0; i < sizeof(expected); i++) {
    if (block[i] != static_cast<uint8_t>(expected[i])) {
      magic = false;
    }
  }
  panic("NATIVE_EMMC read ok magic=%u\n"
        "%02x %02x %02x %02x %02x %02x %02x %02x\n"
        "%02x %02x %02x %02x %02x %02x %02x %02x\n"
        "lba=%llu cfg=%08x fifo=%08x",
        magic ? 1U : 0U,
        block[0], block[1], block[2], block[3], block[4], block[5], block[6], block[7],
        block[8], block[9], block[10], block[11], block[12], block[13], block[14], block[15],
        (unsigned long long)kUserdataLba, r.cfg, r.fifo);
  return true;
}

void NativeEmmc::stop(IOService *provider)
{
  super::stop(provider);
}
