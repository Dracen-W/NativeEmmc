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

## Integration

The UniXNU-merlinx repository includes this repository as
`drivers/NativeEmmc`, builds it with `scripts/build-arm-driver.py NativeEmmc`,
and links `libNativeEmmc.a` when `CONFIG_XNU_NATIVE_EMMC=y`.

# Documentation

Webpage at [https://unixnu.com/drivers/community-native-emmc](https://unixnu.com/drivers/community-native-emmc)

## License

BSD-2-Clause-Patent. See [LICENSE](LICENSE).
