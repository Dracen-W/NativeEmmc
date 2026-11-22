// SPDX-License-Identifier: BSD-2-Clause-Patent
// MediaTek MT6768 MSDC/eMMC bring-up for the Xiaomi Redmi Note 9 (merlinx).
//
// The first milestone was a deliberately read-only and passive probe: it maps
// the controller UEFI left running and issues one CMD17 against the iPadOS
// userdata partition. Root mode now publishes the OS and SystemOS windows as
// read-only block devices. Nothing is reset, initialized, tuned or written.
#include <IOKit/IOService.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOBSD.h>
#include <IOKit/storage/IOBlockStorageDevice.h>
#include <sys/param.h>
#include <sys/types.h>
#include <libkern/c++/OSDictionary.h>
#include <libkern/c++/OSIterator.h>
#include <libkern/c++/OSNumber.h>
#include <libkern/c++/OSString.h>
#include <libkern/libkern.h>
#include <libkern/crypto/sha2.h>
#include <kern/thread.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>
#include <kern/debug.h>

extern "C" __attribute__((weak)) void native_gop_text(unsigned int line, const char *text);
extern "C" __attribute__((weak)) void native_musb_log_printf(const char *fmt, ...);
extern "C" volatile uint32_t native_emmc_probe_calls;
extern "C" volatile uint32_t native_emmc_matched;
extern "C" volatile uint32_t native_emmc_probe_requested;

extern "C" {
volatile uint32_t native_emmc_probe_calls = 0;
volatile uint32_t native_emmc_matched = 0;
volatile uint32_t native_emmc_probe_requested = 0;
volatile uint32_t native_emmc_blocks_published = 0;
volatile uint64_t native_emmc_block_reads = 0;
volatile uint64_t native_emmc_block_failures = 0;
volatile uint64_t native_emmc_blocks_read_total = 0;
volatile uint64_t native_emmc_read_us_total = 0;
}

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
constexpr uint32_t MSDC_CFG_RST = 1U << 2;
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
constexpr uint32_t SDC_CMD_MULTIPLE_BLK = 1U << 12;
constexpr uint32_t SDC_CMD_STOP_CMD = 1U << 14;
constexpr uint32_t SDC_CMD_BLK_SIZE_SHIFT = 16;
constexpr uint32_t SDC_STS_BUSY = 1U << 0;

// JEDEC eMMC R1 response error bits (Linux include/linux/mmc/mmc.h).
constexpr uint32_t kR1ErrorMask = 0xFDF9A080U;

constexpr uint32_t kBlockSize = 512;
constexpr uint32_t kTransferBlocks = 2048;
constexpr uint32_t kTransferBytes = kTransferBlocks * kBlockSize;
constexpr unsigned kSpinPolls = 4096;
constexpr unsigned kMaxPolls = 1000000;
// p45 starts at 20332544 512-byte sectors in the eMMC user area. The root
// manifest is at the beginning of the iPadOS userdata image.
constexpr uint64_t kUserdataLba = 20332544ULL;
constexpr uint64_t kUserdataBytes = 114597789696ULL;

static inline uint32_t read32(uintptr_t base, uint32_t offset)
{
  return *reinterpret_cast<volatile uint32_t *>(base + offset);
}

static inline void write32(uintptr_t base, uint32_t offset, uint32_t value)
{
  *reinterpret_cast<volatile uint32_t *>(base + offset) = value;
}

static inline uint64_t emmcNowUs()
{
  uint64_t now = 0, ns = 0;
  clock_get_uptime(&now);
  absolutetime_to_nanoseconds(now, &ns);
  return ns / 1000ULL;
}

static inline void pollPause(unsigned iteration)
{
  if (iteration < kSpinPolls) {
    __asm__ volatile("yield" ::: "memory");
  } else if ((iteration & 63U) == 0U) {
    IODelay(iteration < 65536U ? 1 : 10);
  } else {
    __asm__ volatile("yield" ::: "memory");
  }
}

static void resetController(uintptr_t base)
{
  write32(base, MSDC_CFG, read32(base, MSDC_CFG) | MSDC_CFG_RST);
  for (unsigned i = 0; i < 100000U && (read32(base, MSDC_CFG) & MSDC_CFG_RST); ++i) {
    IODelay(1);
  }
  write32(base, MSDC_FIFOCS, MSDC_FIFOCS_CLR);
  for (unsigned i = 0; i < 100000U && (read32(base, MSDC_FIFOCS) & MSDC_FIFOCS_CLR); ++i) {
    IODelay(1);
  }
  write32(base, MSDC_INT, 0xffffffffU);
  write32(base, MSDC_CFG, read32(base, MSDC_CFG) | MSDC_CFG_MODE | MSDC_CFG_PIO);
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
  for (unsigned i = 0; i < kMaxPolls; i++) {
    *status = read32(base, MSDC_INT);
    if (*status & (MSDC_INT_CMDRDY | MSDC_INT_ERROR)) {
      return true;
    }
    pollPause(i);
  }
  return false;
}

static void clearStatus(uintptr_t base, uint32_t bits)
{
  if (bits) {
    write32(base, MSDC_INT, bits);
  }
}

static bool waitNotBusy(uintptr_t base, uint32_t *failure)
{
  for (unsigned i = 0; i < kMaxPolls; i++) {
    if ((read32(base, SDC_STS) & SDC_STS_BUSY) == 0) {
      return true;
    }
    pollPause(i);
  }
  if (failure) {
    *failure = 1;
  }
  return false;
}

// Issue CMD12 to terminate an open-ended or failed multi-block transfer, clear
// any residual FIFO state, and wait for the card/controller busy state to drop.
static bool stopTransfer(uintptr_t base)
{
  uint32_t status = 0;
  for (unsigned i = 0; i < kMaxPolls; i++) {
    if ((read32(base, SDC_STS) & (SDC_STS_BUSY | (SDC_STS_BUSY << 1))) == 0) {
      break;
    }
    pollPause(i);
  }
  write32(base, MSDC_INTEN, 0);
  clearStatus(base, 0xffffffffU);
  write32(base, SDC_ARG, 0);
  write32(base, SDC_CMD, 12U | (7U << SDC_CMD_RSP_TYPE_SHIFT) | SDC_CMD_STOP_CMD);
  bool cmdOk = waitCommand(base, &status) && (status & MSDC_INT_ERROR) == 0;
  clearStatus(base, status & (MSDC_INT_CMDRDY | MSDC_INT_ERROR));
  write32(base, MSDC_FIFOCS, MSDC_FIFOCS_CLR);
  bool busyOk = waitNotBusy(base, nullptr);
  return cmdOk && busyOk;
}

// Read blocks via PIO (CMD17 for 1 block, CMD18 + CMD12 for multiple blocks),
// modeled on Linux drivers/mmc/host/mtk-sd.c msdc_pio_read().
static bool readBlocks(uintptr_t base, uint64_t lba, uint32_t blocks, uint8_t *out, uint32_t *failure)
{
  uint32_t status = 0;
  uint32_t total = blocks * kBlockSize;

  if (!blocks || !out || lba > 0xffffffffULL) {
    *failure = 1;
    return false;
  }

  // A stale FIFO or command status would be read as the new data.
  write32(base, MSDC_INTEN, 0);
  clearStatus(base, 0xffffffffU);
  write32(base, MSDC_FIFOCS, MSDC_FIFOCS_CLR);
  for (unsigned clrSpin = 0; clrSpin < 100000U && (read32(base, MSDC_FIFOCS) & MSDC_FIFOCS_CLR); ++clrSpin) {
  }
  /* MSDC_CFG_MODE | MSDC_CFG_PIO is latched at startRoot() and resetController(). */

  if (!waitNotBusy(base, failure)) {
    return false;
  }

  write32(base, SDC_BLK_NUM, blocks);
  write32(base, SDC_ARG, static_cast<uint32_t>(lba));
  uint32_t command = blocks == 1 ? 17U : 18U;
  uint32_t transfer = blocks == 1 ? SDC_CMD_SINGLE_BLK : SDC_CMD_MULTIPLE_BLK;
  write32(base, SDC_CMD,
          command | (1U << SDC_CMD_RSP_TYPE_SHIFT) | transfer |
          (kBlockSize << SDC_CMD_BLK_SIZE_SHIFT));

  if (!waitCommand(base, &status)) {
    if (blocks > 1) {
      (void)stopTransfer(base);
    }
    *failure = 2;
    return false;
  }
  if (status & MSDC_INT_ERROR) {
    clearStatus(base, status & (MSDC_INT_CMDRDY | MSDC_INT_ERROR));
    if (blocks > 1) {
      (void)stopTransfer(base);
    }
    *failure = 3;
    return false;
  }
  clearStatus(base, MSDC_INT_CMDRDY);

  // Verify the card's R1 status word in SDC_RESP0 before entering the data phase.
  if ((read32(base, SDC_RESP0) & kR1ErrorMask) != 0) {
    if (blocks > 1) {
      (void)stopTransfer(base);
    }
    *failure = 7;
    return false;
  }

  uint32_t got = 0;
  unsigned idle = 0;
  while (got < total) {
    uint32_t available = read32(base, MSDC_FIFOCS) & MSDC_FIFOCS_RX_MASK;
    if (available >= 4) {
      do {
        while (available >= 16 && got + 16 <= total) {
          uint32_t w0 = read32(base, MSDC_RXDATA);
          uint32_t w1 = read32(base, MSDC_RXDATA);
          uint32_t w2 = read32(base, MSDC_RXDATA);
          uint32_t w3 = read32(base, MSDC_RXDATA);
          __builtin_memcpy(out + got, &w0, 4);
          __builtin_memcpy(out + got + 4, &w1, 4);
          __builtin_memcpy(out + got + 8, &w2, 4);
          __builtin_memcpy(out + got + 12, &w3, 4);
          got += 16;
          available -= 16;
        }
        while (available >= 4 && got + 4 <= total) {
          uint32_t word = read32(base, MSDC_RXDATA);
          __builtin_memcpy(out + got, &word, sizeof(word));
          got += 4;
          available -= 4;
        }
        if (got >= total) {
          break;
        }
        available = read32(base, MSDC_FIFOCS) & MSDC_FIFOCS_RX_MASK;
      } while (available >= 4);
      idle = 0;
      continue;
    }

    status = read32(base, MSDC_INT);
    if (status & MSDC_INT_ERROR) {
      clearStatus(base, status & (MSDC_INT_XFER_COMPL | MSDC_INT_ERROR));
      if (blocks > 1) {
        (void)stopTransfer(base);
      }
      *failure = 4;
      return false;
    }
    if (++idle > kMaxPolls) {
      if (blocks > 1) {
        (void)stopTransfer(base);
      }
      *failure = 5;
      return false;
    }
    pollPause(idle);
  }

  // Require an explicit transfer-complete indication without controller errors
  // even after all expected bytes have been drained from the RX FIFO.
  bool completed = false;
  for (unsigned i = 0; i < kMaxPolls; i++) {
    status = read32(base, MSDC_INT);
    if (status & MSDC_INT_ERROR) {
      clearStatus(base, status & (MSDC_INT_XFER_COMPL | MSDC_INT_ERROR));
      if (blocks > 1) {
        (void)stopTransfer(base);
      }
      *failure = 4;
      return false;
    }
    if (status & MSDC_INT_XFER_COMPL) {
      clearStatus(base, MSDC_INT_XFER_COMPL);
      completed = true;
      break;
    }
    pollPause(i);
  }
  if (!completed) {
    if (blocks > 1) {
      (void)stopTransfer(base);
    }
    *failure = 5;
    return false;
  }

  if (blocks > 1 && !stopTransfer(base)) {
    *failure = 6;
    return false;
  }
  *failure = 0;
  return true;
}

static uint32_t le32(const uint8_t *at)
{
  return uint32_t(at[0]) | (uint32_t(at[1]) << 8) | (uint32_t(at[2]) << 16) | (uint32_t(at[3]) << 24);
}

static uint64_t le64(const uint8_t *at)
{
  return uint64_t(le32(at)) | (uint64_t(le32(at + 4)) << 32);
}

struct EmmcWindow {
  char name[32];
  uint64_t offset;
  uint64_t size;
  uint8_t sha256[32];
};

struct EmmcLayout {
  uint64_t imageBytes;
  uint32_t windows;
  EmmcWindow window[4];
  uint32_t manifestStatus;
  uint32_t magic0, magic1;
};

// Parse the SHNGROOT manifest at the start of the userdata partition. The
// manifest uses 4096-byte blocks and describes byte offsets inside the image.
static bool findEmmcLayout(uintptr_t base, EmmcLayout *layout)
{
  uint8_t *block = static_cast<uint8_t *>(IOMalloc(4096));
  if (!block) {
    return false;
  }
  bzero(layout, sizeof(*layout));
  uint32_t failure = 0;
  if (!readBlocks(base, kUserdataLba, 8, block, &failure)) {
    layout->manifestStatus = 8;
    layout->magic0 = failure;
    IOFree(block, 4096);
    return true;
  }
  layout->magic0 = le32(block);
  layout->magic1 = le32(block + 4);
  layout->manifestStatus = 1;
  if (layout->magic0 == 0x474e4853U && layout->magic1 == 0x544f4f52U) {
    uint32_t version = le32(block + 8);
    uint32_t count = le32(block + 12);
    uint64_t imageBytes = le64(block + 16);
    layout->manifestStatus = 2;
    if (version == 1 && count && count <= 4) {
      SHA256_CTX context;
      uint8_t digest[32];
      SHA256_Init(&context);
      SHA256_Update(&context, block, 4064);
      SHA256_Final(digest, &context);
      layout->manifestStatus = 3;
      if (memcmp(digest, block + 4064, 32) == 0) {
        layout->manifestStatus = 4;
        bool bounds = imageBytes >= 4096 && !(imageBytes & 4095) &&
                      imageBytes <= kUserdataBytes;
        for (uint32_t index = 0; index < count && bounds; index++) {
          const uint8_t *entry = block + 24 + 128 * index;
          uint64_t offset = le64(entry + 32), size = le64(entry + 40);
          if ((offset & 4095) || offset < 4096 || (size & 511) || !size ||
              offset >= imageBytes || size > imageBytes - offset ||
              ((size + 4095) & ~4095ULL) > imageBytes - offset) {
            bounds = false;
            break;
          }
          if (le32(entry + 48) != 512 || le32(entry + 52) != 1) {
            layout->manifestStatus = 7;
            bounds = false;
            break;
          }
          unsigned length = 0;
          while (length < 32 && entry[length] >= 0x20 && entry[length] <= 0x7e) {
            ++length;
          }
          if (!length || length == 32 || entry[length] != 0) {
            layout->manifestStatus = 5;
            bounds = false;
            break;
          }
          for (uint32_t prior = 0; prior < index; ++prior) {
            const EmmcWindow &other = layout->window[prior];
            if (!strcmp(reinterpret_cast<const char *>(entry), other.name)) {
              layout->manifestStatus = 5;
              bounds = false;
              break;
            }
            if (offset < other.offset + ((other.size + 4095) & ~4095ULL) &&
                other.offset < offset + ((size + 4095) & ~4095ULL)) {
              layout->manifestStatus = 6;
              bounds = false;
              break;
            }
          }
          if (!bounds) {
            break;
          }
          memcpy(layout->window[index].name, entry, 32);
          layout->window[index].name[31] = 0;
          layout->window[index].offset = offset;
          layout->window[index].size = size;
          memcpy(layout->window[index].sha256, entry + 56, 32);
        }
        if (bounds) {
          layout->windows = count;
          layout->imageBytes = imageBytes;
          layout->manifestStatus = 0;
        }
      }
    }
  }
  IOFree(block, 4096);
  return true;
}

static IOLock *gControllerLock = nullptr;
static uintptr_t gControllerBase = 0;
static uint64_t gWindowLba[4];
static uint64_t gWindowBytes[4];
static unsigned gWindowCount;

} // namespace

class NativeEmmcBlock : public IOBlockStorageDevice {
  OSDeclareDefaultStructors(NativeEmmcBlock)
  uint64_t absoluteLba = 0;
  uint64_t bytes = 0;
  char label[32] = {};
  uint8_t *bounce = nullptr;
  IOLock *windowLock = nullptr;
  bool online = false;
public:
  const char *windowLabel() const { return label; }
  bool isRootWindow() const { return !strcmp(label, "OS"); }
  bool isTrustedWindow() const { return isRootWindow() || !strcmp(label, "SystemOS"); }
  bool isOnline() const { return online; }
  bool configure(const char *name, uint64_t lba, uint64_t size);
  bool start(IOService *provider) override;
  void free() override;
  IOReturn doEjectMedia() override { return kIOReturnUnsupported; }
  IOReturn doFormatMedia(UInt64) override { return kIOReturnNotWritable; }
  UInt32 doGetFormatCapacities(UInt64 *, UInt32) const override { return 0; }
  char *getVendorString() override { return const_cast<char *>("Native"); }
  char *getProductString() override { return label; }
  char *getRevisionString() override { return const_cast<char *>("1.0"); }
  char *getAdditionalDeviceInfoString() override { return const_cast<char *>("eMMC userdata window, read-only"); }
  IOReturn reportBlockSize(UInt64 *value) override { *value = 512; return kIOReturnSuccess; }
  IOReturn reportEjectability(bool *value) override { *value = false; return kIOReturnSuccess; }
  IOReturn reportMaxValidBlock(UInt64 *value) override { *value = bytes / 512 - 1; return kIOReturnSuccess; }
  IOReturn reportMediaState(bool *present, bool *changed) override {
    *present = online;
    if (changed) {
      *changed = false;
    }
    return kIOReturnSuccess;
  }
  IOReturn reportRemovability(bool *value) override { *value = false; return kIOReturnSuccess; }
  IOReturn reportWriteProtection(bool *value) override { *value = true; return kIOReturnSuccess; }
  IOReturn doSynchronizeCache() override { return online ? kIOReturnSuccess : kIOReturnNotReady; }
  IOReturn doSynchronize(UInt64, UInt64, IOOptionBits) override { return doSynchronizeCache(); }
  IOReturn getWriteCacheState(bool *value) override { *value = false; return kIOReturnSuccess; }
  IOReturn setWriteCacheState(bool) override { return kIOReturnUnsupported; }
  IOReturn doAsyncReadWrite(IOMemoryDescriptor *, UInt64, UInt64, IOStorageAttributes *, IOStorageCompletion *) override;
};

OSDefineMetaClassAndStructors(NativeEmmcBlock, IOBlockStorageDevice)

static char native_emmc_uuid_fold(char value)
{
  return value >= 'a' && value <= 'z' ? value - ('a' - 'A') : value;
}

static bool native_emmc_uuid_equals(const char *left, const char *right)
{
  if (!left || !right) {
    return false;
  }
  while (*left && *right) {
    if (native_emmc_uuid_fold(*left++) != native_emmc_uuid_fold(*right++)) {
      return false;
    }
  }
  return !*left && !*right;
}

static const char *native_emmc_window_for_uuid(const char *uuid)
{
  if (native_emmc_uuid_equals(uuid, "04769D26-2CA0-4825-B443-E2E5F88300F2")) {
    return "OS";
  }
  if (native_emmc_uuid_equals(uuid, "8846FE78-2910-4297-AF97-D59BB6F8FC16")) {
    return "SystemOS";
  }
  return nullptr;
}

extern "C" int native_verified_block_volume_device(const char *uuid, dev_t *device, char *path, size_t size)
{
  if (!uuid || !device || !path || size < 6 || strlen(uuid) != 36) {
    return 0;
  }
  const char *expected = native_emmc_window_for_uuid(uuid);
  if (!expected) {
    return 0;
  }
  *device = NODEV;
  path[0] = 0;
  OSDictionary *matching = IOService::serviceMatching("IOMedia");
  OSDictionary *properties = OSDictionary::withCapacity(1);
  OSString *wanted = OSString::withCString(uuid);
  if (!matching || !properties || !wanted) {
    OSSafeReleaseNULL(wanted);
    OSSafeReleaseNULL(properties);
    OSSafeReleaseNULL(matching);
    return 0;
  }
  bool prepared = properties->setObject("UUID", wanted) &&
                  matching->setObject("IOPropertyMatch", properties);
  wanted->release();
  properties->release();
  if (!prepared) {
    matching->release();
    return 0;
  }
  OSIterator *iterator = IOService::getMatchingServices(matching);
  matching->release();
  if (!iterator) {
    return 0;
  }
  bool authenticated = false;
  unsigned matches = 0;
  dev_t candidate = NODEV;
  char candidatePath[64] = {};
  while (OSObject *object = iterator->getNextObject()) {
    IOService *media = OSDynamicCast(IOService, object);
    if (!media) {
      continue;
    }
    if (++matches != 1) {
      authenticated = false;
      break;
    }
    OSString *identity = OSDynamicCast(OSString, media->getProperty("UUID"));
    OSString *name = OSDynamicCast(OSString, media->getProperty(kIOBSDNameKey));
    OSNumber *majorNumber = OSDynamicCast(OSNumber, media->getProperty(kIOBSDMajorKey));
    OSNumber *minorNumber = OSDynamicCast(OSNumber, media->getProperty(kIOBSDMinorKey));
    if (!identity || !native_emmc_uuid_equals(identity->getCStringNoCopy(), uuid) || !name || !majorNumber ||
        !minorNumber || !name->getLength() || name->getLength() >= sizeof(candidatePath) - 6 ||
        name->getLength() >= size - 6 || majorNumber->unsigned64BitValue() > 255 ||
        minorNumber->unsigned64BitValue() > 0xffffff) {
      continue;
    }
    for (IOService *provider = media; provider; provider = provider->getProvider()) {
      NativeEmmcBlock *block = OSDynamicCast(NativeEmmcBlock, provider);
      if (!block) {
        continue;
      }
      authenticated = block->isOnline() && !strcmp(block->windowLabel(), expected);
      if (authenticated) {
        candidate = static_cast<dev_t>(makedev(majorNumber->unsigned32BitValue(),
                                               minorNumber->unsigned32BitValue()));
        snprintf(candidatePath, sizeof(candidatePath), "/dev/%s", name->getCStringNoCopy());
      }
      break;
    }
  }
  iterator->release();
  if (!authenticated || matches != 1) {
    return 0;
  }
  *device = candidate;
  strlcpy(path, candidatePath, size);
  return 1;
}

// The offline OS image cannot satisfy Apple's per-device root seal. Accept the root only
// when it was mounted from one of the read-only target windows this driver published.
extern "C" int native_verified_block_root_authorized(dev_t device)
{
  unsigned root = 0;
  PE_parse_boot_argn("native-emmc-root", &root, sizeof(root));
  if (!root) {
    return 0;
  }
  OSDictionary *matching = IOService::serviceMatching("IOMedia");
  OSDictionary *properties = OSDictionary::withCapacity(2);
  OSNumber *majorNumber = OSNumber::withNumber(static_cast<unsigned>(major(device)), 32);
  OSNumber *minorNumber = OSNumber::withNumber(static_cast<unsigned>(minor(device)), 32);
  if (!matching || !properties || !majorNumber || !minorNumber) {
    OSSafeReleaseNULL(minorNumber);
    OSSafeReleaseNULL(majorNumber);
    OSSafeReleaseNULL(properties);
    OSSafeReleaseNULL(matching);
    return 0;
  }
  bool prepared = properties->setObject(kIOBSDMajorKey, majorNumber) &&
                  properties->setObject(kIOBSDMinorKey, minorNumber) &&
                  matching->setObject("IOPropertyMatch", properties);
  minorNumber->release();
  majorNumber->release();
  properties->release();
  if (!prepared) {
    matching->release();
    return 0;
  }
  OSIterator *iterator = IOService::getMatchingServices(matching);
  matching->release();
  if (!iterator) {
    return 0;
  }
  bool authorized = false;
  unsigned matches = 0;
  while (OSObject *object = iterator->getNextObject()) {
    IOService *media = OSDynamicCast(IOService, object);
    if (!media || ++matches != 1) {
      authorized = false;
      break;
    }
    OSString *identity = OSDynamicCast(OSString, media->getProperty("UUID"));
    const char *expectedWindow = identity ? native_emmc_window_for_uuid(identity->getCStringNoCopy()) : nullptr;
    for (IOService *provider = media; provider; provider = provider->getProvider()) {
      NativeEmmcBlock *block = OSDynamicCast(NativeEmmcBlock, provider);
      if (block) {
        authorized = block->isTrustedWindow() && block->isOnline() &&
                     (!expectedWindow || !strcmp(block->windowLabel(), expectedWindow));
        break;
      }
    }
  }
  iterator->release();
  return authorized && matches == 1;
}

bool NativeEmmcBlock::configure(const char *name, uint64_t lba, uint64_t size)
{
  if (!name || !size || (size & 511) || !(lba + size / 512 <= 0x100000000ULL)) {
    return false;
  }
  strlcpy(label, name, sizeof(label));
  absoluteLba = lba;
  bytes = size;
  windowLock = IOLockAlloc();
  if (!windowLock) {
    return false;
  }
  bounce = static_cast<uint8_t *>(IOMalloc(kTransferBytes));
  return bounce != nullptr;
}

bool NativeEmmcBlock::start(IOService *provider)
{
  if (!bounce || !windowLock || !IOBlockStorageDevice::start(provider)) {
    return false;
  }
  online = true;
  registerService();
  __atomic_fetch_add(&native_emmc_blocks_published, 1, __ATOMIC_RELAXED);
  return true;
}

IOReturn NativeEmmcBlock::doAsyncReadWrite(IOMemoryDescriptor *buffer, UInt64 block, UInt64 count,
                                           IOStorageAttributes *, IOStorageCompletion *completion)
{
  if (!buffer || !completion || !completion->action || !count || block >= bytes / 512 ||
      count > bytes / 512 - block || buffer->getLength() < count * 512) {
    return kIOReturnBadArgument;
  }
  if (buffer->getDirection() != kIODirectionIn) {
    return kIOReturnNotWritable;
  }
  if (!online) {
    return kIOReturnNotReady;
  }

  IOReturn result = kIOReturnSuccess;
  uint64_t done = 0;
  IOLockLock(windowLock);
  while (done < count) {
    uint32_t n = static_cast<uint32_t>(count - done);
    if (n > kTransferBlocks) {
      n = kTransferBlocks;
    }
    uint32_t failure = 0;
    bool ok = false;
    const uint64_t t0 = emmcNowUs();
    IOLockLock(gControllerLock);
    for (unsigned attempt = 0; attempt < 4; ++attempt) {
      failure = 0;
      ok = readBlocks(gControllerBase, absoluteLba + block + done, n, bounce, &failure);
      if (ok) {
        break;
      }
      if (native_gop_text) {
        char errLine[64];
        snprintf(errLine, sizeof(errLine), "EMMC RD ERR f%u lba%llx n%u try%u",
                 failure, (unsigned long long)(absoluteLba + block + done), n, attempt + 1);
        native_gop_text(13, errLine);
      }
      (void)stopTransfer(gControllerBase);
      resetController(gControllerBase);
      IODelay(1000 * (attempt + 1));
    }
    IOLockUnlock(gControllerLock);
    __atomic_fetch_add(&native_emmc_read_us_total, emmcNowUs() - t0, __ATOMIC_RELAXED);
    if (!ok) {
      result = kIOReturnIOError;
      break;
    }
    const IOByteCount chunkBytes = static_cast<IOByteCount>(n) * kBlockSize;
    if (buffer->writeBytes(done * kBlockSize, bounce, chunkBytes) != chunkBytes) {
      result = kIOReturnIOError;
      break;
    }
    done += n;
    __atomic_fetch_add(&native_emmc_block_reads, 1, __ATOMIC_RELAXED);
    __atomic_fetch_add(&native_emmc_blocks_read_total, n, __ATOMIC_RELAXED);
  }
  IOLockUnlock(windowLock);
  if (result != kIOReturnSuccess) {
    __atomic_fetch_add(&native_emmc_block_failures, 1, __ATOMIC_RELAXED);
  }

  completion->action(completion->target, completion->parameter, result, done * 512);
  return kIOReturnSuccess;
}

void NativeEmmcBlock::free()
{
  if (bounce) {
    IOFree(bounce, kTransferBytes);
    bounce = nullptr;
  }
  if (windowLock) {
    IOLockFree(windowLock);
    windowLock = nullptr;
  }
  IOBlockStorageDevice::free();
}

class NativeEmmc : public IOService {
  OSDeclareDefaultStructors(NativeEmmc)
  OSSharedPtr<IOMemoryMap> fBaseMap;
  OSSharedPtr<IOMemoryMap> fTopMap;
  uintptr_t fBase = 0;
  uintptr_t fTop = 0;
  bool startRoot(IOService *provider);
public:
  IOService *probe(IOService *provider, SInt32 *score) override;
  bool start(IOService *provider) override;
  void stop(IOService *provider) override;
};

OSDefineMetaClassAndStructors(NativeEmmc, IOService)

static NativeEmmcBlock *gPendingDevice[4];
static unsigned gPublicationOrder[4];
static unsigned gPublicationCount;
static NativeEmmc *gPendingProvider;

static void reportMedia(unsigned delay)
{
  IOSleep(delay * 1000);
  char text[2200];
  int used = snprintf(text, sizeof(text),
                      "NATIVE_EMMC media report published=%u reads=%llu failures=%llu\n",
                      native_emmc_blocks_published,
                      (unsigned long long)native_emmc_block_reads,
                      (unsigned long long)native_emmc_block_failures);
  for (unsigned index = 0; index < gWindowCount; ++index) {
    uint8_t sector[1024] = {};
    uint32_t failure = 0;
    if (gControllerLock) {
      IOLockLock(gControllerLock);
    }
    bool ok = readBlocks(gControllerBase, gWindowLba[index], 2, sector, &failure);
    if (gControllerLock) {
      IOLockUnlock(gControllerLock);
    }
    used += snprintf(text + used, sizeof(text) - used,
                     "window %u lba=%llu bytes=%llu read=%u failure=%u "
                     "first=%02x%02x%02x%02x gpt=%02x%02x%02x%02x\n",
                     index, (unsigned long long)gWindowLba[index],
                     (unsigned long long)gWindowBytes[index], ok ? 1U : 0U, failure,
                     sector[0], sector[1], sector[2], sector[3],
                     sector[512], sector[513], sector[514], sector[515]);
  }
  OSDictionary *matching = IOService::serviceMatching("IOMedia");
  OSIterator *iterator = matching ? IOService::getMatchingServices(matching) : nullptr;
  unsigned count = 0;
  while (iterator && used < int(sizeof(text)) - 200) {
    IOService *media = OSDynamicCast(IOService, iterator->getNextObject());
    if (!media) {
      break;
    }
    OSString *uuid = OSDynamicCast(OSString, media->getProperty("UUID"));
    OSString *bsd = OSDynamicCast(OSString, media->getProperty(kIOBSDNameKey));
    OSString *content = OSDynamicCast(OSString, media->getProperty("Content"));
    OSNumber *size = OSDynamicCast(OSNumber, media->getProperty("Size"));
    bool whole = false;
    OSBoolean *wholeProperty = OSDynamicCast(OSBoolean, media->getProperty("Whole"));
    if (wholeProperty) {
      whole = wholeProperty->isTrue();
    }
    used += snprintf(text + used, sizeof(text) - used, "media %u %s %s size=%llx whole=%u uuid=%s\n",
                     count++, bsd ? bsd->getCStringNoCopy() : "-",
                     content ? content->getCStringNoCopy() : "-",
                     size ? size->unsigned64BitValue() : 0ULL, whole ? 1U : 0U,
                     uuid ? uuid->getCStringNoCopy() : "-");
  }
  OSSafeReleaseNULL(iterator);
  OSSafeReleaseNULL(matching);
  panic("%s", text);
}

static void publishThread(void *, wait_result_t)
{
  for (unsigned position = 0; position < gPublicationCount; ++position) {
    unsigned index = gPublicationOrder[position];
    NativeEmmcBlock *device = gPendingDevice[index];
    if (!device->start(gPendingProvider)) {
      panic("NATIVE_EMMC window start failed: %s", device->windowLabel());
    }
    IOLog("NativeEmmc: published %s\n", device->windowLabel());
  }
  unsigned media = 0;
  PE_parse_boot_argn("native-emmc-media", &media, sizeof(media));
  if (media) {
    unsigned delay = 2;
    PE_parse_boot_argn("native-emmc-media-delay", &delay, sizeof(delay));
    reportMedia(delay);
  }
  gPendingProvider->release();
}

static bool buildPublicationPlan(const EmmcLayout *layout)
{
  unsigned system = layout->windows, os = layout->windows;
  for (unsigned index = 0; index < layout->windows; ++index) {
    if (!strcmp(layout->window[index].name, "SystemOS")) {
      system = index;
    }
    if (!strcmp(layout->window[index].name, "OS")) {
      os = index;
    }
  }
  if (system == layout->windows || os == layout->windows) {
    return false;
  }
  gPublicationOrder[0] = system;
  gPublicationCount = 1;
  for (unsigned index = 0; index < layout->windows; ++index) {
    if (index != system && index != os) {
      gPublicationOrder[gPublicationCount++] = index;
    }
  }
  gPublicationOrder[gPublicationCount++] = os;
  return true;
}

bool NativeEmmc::startRoot(IOService *provider)
{
  write32(fBase, MSDC_CFG, read32(fBase, MSDC_CFG) | MSDC_CFG_MODE | MSDC_CFG_PIO);
  {
    Registers r = snapshot(fBase);
    uint32_t ckdiv = (r.cfg >> 8) & 0xfffU;
    uint32_t ckmod = (r.cfg >> 20) & 0x3U;
    uint32_t bwBits = (r.sdcCfg >> 16) & 0x3U;
    uint32_t busWidth = (bwBits == 2U) ? 8U : ((bwBits == 1U) ? 4U : 1U);
    uint32_t benchUs = 0;
    uint32_t benchKBs = 0;
    uint32_t benchFail = 0;
    uint8_t *benchBuf = static_cast<uint8_t *>(IOMalloc(kTransferBytes));
    if (benchBuf) {
      uint64_t t0 = emmcNowUs();
      bool ok = readBlocks(fBase, kUserdataLba, kTransferBlocks, benchBuf, &benchFail);
      uint64_t dt = emmcNowUs() - t0;
      benchUs = static_cast<uint32_t>(dt > 0xffffffffULL ? 0xffffffffULL : dt);
      if (ok && dt > 0) {
        benchKBs = static_cast<uint32_t>((static_cast<uint64_t>(kTransferBytes) * 1000ULL) / dt);
      }
      IOFree(benchBuf, kTransferBytes);
    }
    if (native_musb_log_printf) {
      native_musb_log_printf(
          "[EMMC] cfg=%08x(ckmod=%u ckdiv=%u) sdccfg=%08x(bw=%u) emmc0=%08x top0=%08x | 1M_pio=%uus (%u.%u MB/s fail=%u)\n",
          r.cfg, ckmod, ckdiv, r.sdcCfg, busWidth, r.emmcCfg0, read32(fTop, 0),
          benchUs, benchKBs / 1000U, (benchKBs % 1000U) / 100U, benchFail);
    }
    IOLog("NativeEmmc: cfg=%08x ckmod=%u ckdiv=%u bw=%u 1M_pio=%uus (%u.%u MB/s fail=%u)\n",
          r.cfg, ckmod, ckdiv, busWidth, benchUs, benchKBs / 1000U, (benchKBs % 1000U) / 100U, benchFail);
  }
  EmmcLayout *layout = static_cast<EmmcLayout *>(IOMalloc(sizeof(EmmcLayout)));
  if (!layout || !findEmmcLayout(fBase, layout)) {
    panic("NATIVE_EMMC root manifest read failed");
  }
  if (layout->manifestStatus != 0) {
    panic("NATIVE_EMMC root manifest rejected status=%u magic=%08x %08x windows=%u",
          layout->manifestStatus, layout->magic0, layout->magic1, layout->windows);
  }
  if (!buildPublicationPlan(layout)) {
    panic("NATIVE_EMMC root windows require distinct OS and SystemOS");
  }
  gControllerLock = IOLockAlloc();
  if (!gControllerLock) {
    panic("NATIVE_EMMC controller lock allocation failed");
  }
  gControllerBase = fBase;
  gWindowCount = layout->windows;
  gPendingProvider = this;
  for (unsigned index = 0; index < layout->windows; ++index) {
    NativeEmmcBlock *device = OSTypeAlloc(NativeEmmcBlock);
    uint64_t lba = kUserdataLba + layout->window[index].offset / 512;
    if (!device || !device->init(static_cast<OSDictionary *>(nullptr)) ||
        !device->configure(layout->window[index].name, lba, layout->window[index].size) ||
        !device->attach(this)) {
      panic("NATIVE_EMMC window configure failed: %s", layout->window[index].name);
    }
    gPendingDevice[index] = device;
    gWindowLba[index] = lba;
    gWindowBytes[index] = layout->window[index].size;
  }
  IOFree(layout, sizeof(EmmcLayout));

  retain();
  thread_t thread = nullptr;
  if (kernel_thread_start(publishThread, this, &thread) != KERN_SUCCESS) {
    release();
    panic("NATIVE_EMMC publisher thread creation failed");
  }
  thread_deallocate(thread);
  return true;
}

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
  if (!IOService::start(provider)) {
    return false;
  }

  unsigned probe = 0;
  unsigned root = 0;
  PE_parse_boot_argn("native-emmc-probe", &probe, sizeof(probe));
  PE_parse_boot_argn("native-emmc-root", &root, sizeof(root));
  if (!probe && !root) {
    return true;
  }
  native_emmc_probe_requested = probe ? probe : root;

  fBase = mapPhys(kMsdcBase, kMsdcSize, fBaseMap);
  fTop = mapPhys(kMsdcTop, kMsdcSize, fTopMap);
  if (!fBase || !fTop) {
    panic("NATIVE_EMMC map failed base=%p top=%p", (void *)fBase, (void *)fTop);
  }

  Registers r = snapshot(fBase);
  if (probe == 1) {
    panic("NATIVE_EMMC registers\n"
          "cfg=%08x ps=%08x int=%08x fifo=%08x\n"
          "sdccfg=%08x sdcsts=%08x csts=%08x emmccfg0=%08x\n"
          "top0=%08x top1=%08x lba=%llu",
          r.cfg, r.ps, r.intStatus, r.fifo, r.sdcCfg, r.sdcSts, r.csts, r.emmcCfg0,
          read32(fTop, 0), read32(fTop, 4), (unsigned long long)kUserdataLba);
  }

  if (probe == 2) {
    write32(fBase, MSDC_CFG, read32(fBase, MSDC_CFG) | MSDC_CFG_MODE | MSDC_CFG_PIO);
    uint8_t block[kBlockSize] = {};
    uint32_t failure = 0;
    bool ok = readBlocks(fBase, kUserdataLba, 1, block, &failure);
    if (!ok) {
      panic("NATIVE_EMMC read failed failure=%u\n"
            "cfg=%08x ps=%08x int=%08x fifo=%08x\n"
            "sdccfg=%08x sdcsts=%08x csts=%08x emmccfg0=%08x\n"
            "resp0=%08x lba=%llu",
            failure, r.cfg, r.ps, r.intStatus, r.fifo, r.sdcCfg, r.sdcSts, r.csts, r.emmcCfg0,
            read32(fBase, SDC_RESP0), (unsigned long long)kUserdataLba);
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
  }

  if (root) {
    return startRoot(provider);
  }
  return true;
}

void NativeEmmc::stop(IOService *provider)
{
  IOService::stop(provider);
}