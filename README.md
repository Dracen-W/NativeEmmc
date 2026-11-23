# NativeEmmc (MediaTek MT6768 MSDC0)

Native XNU IOKit eMMC 5.1 block driver for MediaTek MT6768 (`merlinx`, Redmi Note 9).

## Current Status

- **iPadOS 17.7.11 (`21H461`)**: Confirmed working on hardware (boots root/SystemOS/Data via MSDC0 DMA/PIO), **not yet fully stress-tested**.
- **iOS 16.7.16 (`20H392`)**: Confirmed working on hardware (8-bit DDR, DMA ~41.2 MB/s, PIO ~34.4 MB/s, mounts OS + SystemOS + Data cleanly).

## Unified Configuration (`CONFIG_XNU_DW_NATIVEEMMC_*`)

This repository ships a single unified core (`NativeEmmc.cpp`) with no OS-specific branches.
See [`configs/example.config`](configs/example.config) for all supported parameters (values left blank by default).
When built inside `UniXNU-merlinx`, board/OS configs (such as `configs/merlinx-iOS16.7.config`) override these keys at compile time via `-DCONFIG_XNU_DW_NATIVEEMMC_*`:

| Key | Purpose | Default |
| --- | --- | --- |
| `CONFIG_XNU_DW_NATIVEEMMC_OS_UUID` | APFS OS volume UUID (falls back to `CONFIG_XNU_DW_APFS_OS_UUID`) | iPadOS 17.7.11 UUID |
| `CONFIG_XNU_DW_NATIVEEMMC_SYSTEMOS_UUID` | APFS SystemOS volume UUID (falls back to `CONFIG_XNU_DW_APFS_SYSTEMOS_UUID`) | iPadOS 17.7.11 UUID |
| `CONFIG_XNU_DW_NATIVEEMMC_EXTRA_OS_UUID` | Optional secondary OS volume UUID | unset |
| `CONFIG_XNU_DW_NATIVEEMMC_EXTRA_SYSTEMOS_UUID` | Optional secondary SystemOS volume UUID | unset |
| `CONFIG_XNU_DW_NATIVEEMMC_TRANSFER_BLOCKS` | Max 512B blocks per transfer | `2048` (1 MiB) |
| `CONFIG_XNU_DW_NATIVEEMMC_SPIN_POLLS` | Yield iterations before `IODelay` | `4096` |
| `CONFIG_XNU_DW_NATIVEEMMC_USERDATA_LBA` | Userdata partition start LBA | `20332544` |
| `CONFIG_XNU_DW_NATIVEEMMC_USERDATA_BYTES` | Userdata partition size in bytes | `114597789696` |
