# STERZStick v1.0-RC4 Release Notes

## BLE Reinitialization Fix

This release fixes BLE restart behavior when the device returns from low power mode. It is the last M5StickCPlus2-only release before the major v2.0 refactor and M5StickS3 support.

---

## Bug Fixes

### BLE Reinitialization After Low Power

**Issue:** When the device entered low power mode (BLE stopped) and later needed to resume BLE advertising, the NimBLE stack could fail to reinitialize cleanly. This led to unreliable BLE restart behavior after extended idle periods.

**Fix Applied:**

1. **`configureBLEStack()`** – New helper to rebuild the NimBLE stack from scratch when restarting BLE.
2. **`stopBLE()`** – Now fully deinitializes `BLEDevice` and clears globals so a subsequent start begins with a clean state.
3. **`startBLE()`** – Calls `configureBLEStack()` after controller enable to ensure a consistent stack state.
4. **`setup()`** – Uses `startBLE()` for initialization instead of duplicating stack setup logic.

**Impact:** BLE restarts reliably after low power mode, improving reconnection when the device wakes and resumes advertising.

---

## Board Support

- **M5StickC Plus2** – Supported (primary board for this release)
- **M5StickS3** – Not supported in v1.0-RC4. See v2.0-RC1 for multi-board support.

---

## Installation

### From main branch (includes v1.0-RC4)

```bash
git clone https://github.com/Felixrising/STERZStick.git
cd STERZStick
# main branch is v1.0-RC4
pio run --target upload
```

### Using the tag

```bash
git clone https://github.com/Felixrising/STERZStick.git
cd STERZStick
git checkout v1.0-RC4
pio run --target upload
```

---

## Changelog from v1.0-RC3

### Fixed

- BLE reinitialization failure when restarting after low power mode
- NimBLE stack state inconsistencies on BLE stop/start cycles

### Changed

- `stopBLE()` fully deinitializes BLEDevice and clears globals
- `startBLE()` uses `configureBLEStack()` for consistent init
- `setup()` delegates BLE init to `startBLE()` (removed duplicated logic)

---

## What's Next (v2.0)

v2.0-RC1 and later introduce:

- Modular architecture (`BleService`, `DisplayController`, `PowerManager`, `ImuService`)
- Shared runtime state in `app/AppState`
- Tentative M5StickS3 support with board-specific build environments
- Display UX redesign and deferred BLE startup

Use v1.0-RC4 if you need a stable, M5StickCPlus2-only build. Use `dev` or v2.0-RC1 for the refactored codebase and multi-board work.

---

## Support

For issues or questions: [GitHub Issues](https://github.com/Felixrising/STERZStick/issues)
