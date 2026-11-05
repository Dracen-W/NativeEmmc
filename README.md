# NativeEmmc

Native XNU storage support for the MediaTek MT6768 MSDC controller, used by the
Xiaomi Redmi Note 9 (merlinx) UniXNU port.

The driver is intentionally developed in small, observable stages:

1. `native-emmc-probe=1`: map the controller UEFI left running and panic with
   its register state.
2. `native-emmc-probe=2`: issue one read-only `CMD17` for the first block of
   the iPadOS userdata partition and panic with the first 16 bytes. A successful
   read starts with `SHNGROOT`.
3. `native-emmc-root=1`: validate the SHNGROOT manifest and publish the OS and
   SystemOS windows as read-only 512-byte block devices, SystemOS first.

The driver does not reset, initialize, tune or write the controller or card. It
depends on the state left by the platform UEFI firmware. No Apple binaries or
other proprietary artifacts belong in this repository.

## Known Issue in Revisions `4980f1070b0c822dedb9eb350df710a57df28590` / `5b9407d178908138313b46e6c0b2dd8a73230dec` (Severe Read Latency)

> **Warning:** Revisions **`4980f1070b0c822dedb9eb350df710a57df28590`** and
> **`5b9407d178908138313b46e6c0b2dd8a73230dec`** exhibit **severe read latency**
> on hardware (booting through `dyld_shared_cache_arm64` validation and into
> `launchd` takes ~10–12 minutes). Do not pin new builds to those revisions.

Root causes in `4980f1070b0c822dedb9eb350df710a57df28590`:

1. **Unconditional `IODelay(1)` on every FIFO drain pass:** The MT6768 MSDC RX
   FIFO holds at most 128 bytes (`MSDC_FIFOCS_RX_MASK`). `readBlocks()` called
   `IODelay(1)` at the end of every outer loop iteration even when bytes were
   just drained from the FIFO (`got != before`), adding thousands of forced
   microsecond sleeps per megabyte read.
2. **Per-sector `IOMemoryDescriptor::writeBytes` splitting:** `doAsyncReadWrite`
   copied the bounce buffer into the caller's `IOMemoryDescriptor` in 128
   separate 512-byte `writeBytes()` calls per 64 KiB chunk instead of a single
   contiguous copy.
3. **Small 64 KiB transfer window (`kTransferBlocks = 128`):** Large sequential
   reads (`dyld_shared_cache_arm64` mappings and code-signature blobs) issued
   four times as many `CMD18` + `CMD12` stop transactions as necessary.

### Improvements after `4980f1070b0c822dedb9eb350df710a57df28590`

Following the Linux mainline MediaTek MSDC driver (`drivers/mmc/host/mtk-sd.c`,
`msdc_pio_read`) and the static audit in
[`UniXNU-gta4l` PR #3](https://github.com/gta4lunix/UniXNU-gta4l/pull/3):

- **Zero-delay continuous FIFO draining with adaptive spin-then-delay backoff:**
  `readBlocks()` drains all available 32-bit words continuously without
  sleeping whenever `got != before`, spins briefly while the bus fills the next
  FIFO word, and calls `IODelay(1)` only after sustained empty polls.
- **Single-call `IOMemoryDescriptor::writeBytes` and 256 KiB transfer chunks
  (`kTransferBlocks = 512`):** Each transfer chunk is copied to the caller's
  descriptor in one call and issues one `CMD18`/`CMD12` pair per 256 KiB.
- **Explicit `MSDC_INT_XFER_COMPL` verification:** Draining `total` bytes from
  `MSDC_RXDATA` no longer returns early before the controller asserts
  `MSDC_INT_XFER_COMPL` without `MSDC_INT_ERROR`.
- **Guaranteed `CMD12` (`stopTransfer`) recovery on multi-block errors:** Any
  data-phase error or timeout during a `CMD18` (`blocks > 1`) read now issues
  `CMD12`, clears the FIFO, and verifies that `SDC_STS_BUSY` clears so the card
  is never left stranded in the `Sending-Data` state.
- **R1 card-status error checking (`SDC_RESP0`):** Command responses are checked
  against the JEDEC eMMC R1 error mask (`0xFDF9A080`).
- **Root-authorization UUID cross-check:** `native_verified_block_root_authorized`
  verifies that the `IOMedia` UUID matches the trusted `NativeEmmcBlock` window
  label (`OS` or `SystemOS`).
- **License text alignment:** `LICENSE` includes the full `BSD-2-Clause-Patent`
  grant matching the `SPDX-License-Identifier` header in `NativeEmmc.cpp`.

## Integration

The UniXNU-merlinx repository includes this repository as
`drivers/NativeEmmc`, builds it with `scripts/build-arm-driver.py NativeEmmc`,
and links `libNativeEmmc.a` when `CONFIG_XNU_NATIVE_EMMC=y`.

# Documentation

Webpage at [https://unixnu.com/drivers/community-native-emmc](https://unixnu.com/drivers/community-native-emmc)

## Acknowledgments

Special thanks to [@DokuroMitsukai](https://github.com/DokuroMitsukai) and the
[`gta4lunix/UniXNU-gta4l` PR #3](https://github.com/gta4lunix/UniXNU-gta4l/pull/3)
static audit (`docs/native-emmc-reference-audit.md`) of revision
`4980f1070b0c822dedb9eb350df710a57df28590`, which identified the
`MSDC_INT_XFER_COMPL` completion requirement, multi-block error recovery
(`CMD12` stop handling), R1 response status verification, root-authorization
UUID cross-checking, and the `BSD-2-Clause-Patent` license text alignment.

## License

BSD-2-Clause-Patent. See [LICENSE](LICENSE).
