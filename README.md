# STERZStick v1.0-RC3

A high-performance Zwift steering device using the M5StickCPlus2 with advanced IMU/AHRS implementation, power management, and BLE connectivity.

> **Want your own M5StickC Plus2?** Get it here: [M5StickC Plus2 on AliExpress](https://s.click.aliexpress.com/e/_oocIGLh) *(affiliate link - supports further development)*

## About

This project transforms your M5StickCPlus2 into a professional-grade steering controller for Zwift cycling simulation. Based on the excellent work from [@stefaandesmet2003/ESP32Sterzo](https://github.com/stefaandesmet2003/ESP32Sterzo.git), this implementation adds significant enhancements including:

- **Mahony AHRS algorithm** for stable yaw calculation
- **Advanced power management** with adaptive CPU/IMU frequency scaling
- **Motion-based sleep system** with 2-minute low power and 5-minute BLE wait timeouts
- **Comprehensive calibration system** with visual feedback
- **1-degree steering precision** (vs 10-degree binning)
- **Drift compensation** and automatic centering
- **Professional button handling** with short/long press actions

## Recent Development Updates

The project has recently gone through a major internal refactor and behavior update:

- **Modular architecture** - `main.cpp` has been reduced to orchestration, with major logic moved into:
  - `modules/ble/BleService`
  - `modules/display/DisplayController`
  - `modules/power/PowerManager`
  - `modules/sensor/ImuService`
  - shared runtime state in `app/AppState`
- **Board abstraction** - board-specific pin mapping and compile-time variants are in place (`M5StickCPlus2` + `M5StickS3` environments).
- **Display UX redesign** - compact, pixel-aware layout with adaptive text fitting and cleaner steering presentation for small screens.
- **Deferred BLE startup** - BLE init is now delayed after boot for faster UI readiness.
- **Simplified power model** - shifted toward practical `ON -> OFF` behavior after inactivity, with board-specific shutdown behavior.

## Board Support Status

- **M5StickC Plus2** - primary supported board
- **M5StickS3** - active development support (build + runtime scaffolding in place)
  - PMIC-aware power-off path is implemented
  - further PMIC mode tuning and validation is ongoing

## Features

### Core Functionality
- **Zwift BLE Integration** - Full compatibility with Zwift steering protocol
- **Mahony AHRS Filter** - Superior stability compared to basic IMU readings
- **1° Steering Precision** - Fine-grained control for competitive cycling
- **Automatic Calibration** - Smart startup calibration with manual override
- **Drift Compensation** - Gradual centering force prevents long-term drift

### Power Management
- **Adaptive IMU Frequency** - 25Hz for optimal performance and power balance
- **CPU Frequency Scaling** - 80MHz for BLE compatibility and power efficiency
- **Deferred BLE Startup** - faster boot, BLE starts asynchronously after startup
- **Practical Auto Power-Off** - inactivity transitions to board-level off behavior
- **Smart Screen Control** - Auto off after 60s, motion detection keeps active

### User Interface
- **Real-time Display** - Yaw angle, steering bin, BLE status, power mode
- **Visual Limit Indicators** - Flashing arrows at ±40° steering limits
- **Battery Monitoring** - Voltage display with 0.01V precision
- **Calibration Progress** - Visual progress bars and audio feedback
- **Power Status** - Shows current power mode and sleep countdowns

### Button Controls
- **Button A (Front) Short (<1s)** - Wake screen / reset screen timer
- **Button A (Front) Long (≥1s)** - Quick yaw recenter with audio feedback
- **Button B (Top) Short (<2s)** - Quick yaw recenter with audio feedback  
- **Button B (Top) Long (≥2s)** - Full calibration sequence
- **Button C (Power) Short (<1s)** - Wake screen / reset screen timer
- **Button C (Power) Long (≥2s)** - Power off / enter deep sleep mode

## Hardware Requirements

<img src="images/4_0977f4c6-d95d-49bd-8dc0-aa02c9464fd8_1200x1200.webp" alt="M5StickC Plus2 Device" width="300" align="right" style="margin-left: 20px;">

- **M5StickCPlus2** - Main device with built-in IMU (MPU6886)
- **USB-C Cable** - For programming and charging
- **Battery** - Built-in 200mAh LiPo battery

The M5StickC Plus2 is a compact development board perfect for IoT projects. It features a built-in MPU6886 6-axis IMU, 1.14" color LCD display, and 200mAh battery - everything needed for this Zwift steering controller.

## Installation

### Prerequisites
- [PlatformIO](https://platformio.org/) installed
- [Git](https://git-scm.com/) for cloning

### Setup
```bash
# Clone the repository
git clone https://github.com/Felixrising/STERZStick.git
cd STERZStick

# Build and upload
pio run --target upload

# Monitor serial output (optional)
pio device monitor
```

### Dependencies
All dependencies are automatically managed by PlatformIO:
- `M5Unified` - M5StickCPlus2 hardware abstraction
- `ESP32 BLE Arduino` - Bluetooth Low Energy support
- `Preferences` - Non-volatile storage
- `Madgwick` - AHRS algorithm library (unused, kept for compatibility)

## Usage

### First Time Setup
1. **Power On** - Device performs automatic calibration on first boot
2. **Zwift Pairing** - Open Zwift → Settings → Connections → Search for "STERZO"
3. **Calibration** - Hold Button A or B for 2-3 seconds for manual calibration if needed

### Normal Operation
1. **Mount Device** - Attach to handlebars or hold in hand
2. **Connect to Zwift** - Device advertises as "STERZO" 
3. **Steering Range** - ±40° physical rotation = full Zwift steering
4. **Auto-Centering** - Device gradually returns to center when not actively steering

### Power Management
- **BLE Active Mode** - Full performance during Zwift sessions (80MHz CPU, 25Hz IMU)
- **BLE Waiting Mode** - Waiting for connection with inactivity timeout
- **Auto Power-Off** - Device powers down after inactivity to minimize drain
- **Wake Up** - Power button wake / cold boot behavior

## Technical Details

### IMU Configuration
- **Sensor** - MPU6886 6-axis IMU (gyroscope + accelerometer)
- **Algorithm** - Mahony AHRS for quaternion-based orientation
- **Sampling Rate** - 25Hz for optimal performance and power balance
- **Calibration** - Advanced gyro bias compensation and accelerometer calibration

### BLE Protocol
- **Service UUID** - `347b0001-7635-408b-8918-8ff3949ce592`
- **Steering Data** - Float32 angle in degrees (-40° to +40°)
- **Update Rate** - On-change notification (1° resolution)
- **Zwift Handshake** - Full protocol compatibility

### Power Specifications
- **BLE Active Current** - ~80-120mA (80MHz CPU, 25Hz IMU, BLE active)
- **BLE Waiting Current** - ~40-60mA (80MHz CPU, 25Hz IMU, BLE advertising)
- **Low Power Current** - ~20-30mA (80MHz CPU, 25Hz IMU, no BLE)
- **ULP Sleep Current** - ~5-10μA (ULP coprocessor monitoring buttons)
- **Battery Life** - 3-5 hours active use, weeks/months standby

## Calibration System

### Automatic Calibration (First Boot)
1. **Gyro Bias** - 3000 samples over 3 seconds (device stationary)
2. **Accelerometer** - 6-point calibration with audio/visual guidance
3. **Yaw Centering** - Sets current orientation as center position

### Manual Calibration
- **Quick Recenter** - Long press Button A (≥1s) or short press Button B (<2s)
- **Full Calibration** - Long press Button B (≥2s)
- **Progress Feedback** - Visual progress bars and audio beeps

## Troubleshooting

### Common Issues

**Device not connecting to Zwift:**
- Ensure Bluetooth is enabled on your device
- Restart Zwift and search for "STERZO" in connections
- Try power cycling the M5StickCPlus2

**Steering feels unstable:**
- Perform full calibration (long press Button B ≥2s)
- Perform quick recenter (long press Button A ≥1s or short press Button B <2s)
- Ensure device is mounted securely
- Check battery level (low battery affects performance)

**Device goes to sleep too quickly:**
- Motion detection requires actual movement/rotation
- BLE connection prevents automatic sleep
- Screen stays on for 60s after button press, indefinitely when connected to Zwift
- Device enters ULP sleep after 5min BLE wait + 2min low power (when disconnected)

**Yaw drift over time:**
- Built-in drift compensation should handle minor drift
- Perform quick recenter (long press Button A ≥1s or short press Button B <2s)
- Full recalibration may be needed for major drift (long press Button B ≥2s)

### Debug Information
Enable serial monitoring to see detailed debug output:
```bash
pio device monitor
```

Debug output includes:
- IMU readings and frequencies
- Power management status
- BLE connection events
- Calibration progress
- Button press detection

## Battery Optimization Tips

1. **Use during Zwift sessions only** - Device sleeps when not connected
2. **Regular charging** - Don't let battery fully discharge
3. **Stable mounting** - Reduces unnecessary motion detection
4. **Firmware updates** - Keep firmware updated for latest power optimizations

## Contributing

Contributions are welcome! Please feel free to submit issues, feature requests, or pull requests.

### Development Setup
```bash
# Clone with development dependencies
git clone https://github.com/Felixrising/STERZStick.git
cd STERZStick

# Build in debug mode
pio run -e debug

# Run tests (if available)
pio test
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- **[@stefaandesmet2003](https://github.com/stefaandesmet2003/ESP32Sterzo.git)** - Original ESP32Sterzo implementation
- **M5Stack** - M5StickCPlus2 hardware and libraries
- **Zwift** - Steering protocol documentation and support
- **Mahony & Madgwick** - AHRS algorithm implementations

## Project Status

- **Core Functionality** - Complete and tested
- **Power Management** - Optimized for battery life
- **Zwift Integration** - Fully compatible
- **Calibration System** - Comprehensive and user-friendly
- **Documentation** - Ongoing improvements
- **Testing** - Continuous validation with Zwift updates

## Changelog

### v1.0-RC3-dev (Current Development Branch)
**Refactor, Display Cleanup, and Multi-Board Progress**

- Refactored monolithic logic into dedicated modules (`ble`, `display`, `power`, `sensor`)
- Added shared runtime state container in `app/AppState`
- Added compile-time board variant structure for `M5StickCPlus2` and `M5StickS3`
- Implemented deferred BLE startup for faster startup responsiveness
- Updated power strategy toward practical inactivity shutdown behavior
- Improved dashboard layout for tiny screens with compact mode and pixel-aware overlays
- Updated steering direction display (`L`/`R`) and heading rendering behavior
- Continued PMIC-specific power optimization and validation for M5StickS3

---

### v1.0-RC3 (Current Release) CRITICAL UPDATE
**Critical Bug Fixes & IMU Stability Release**

**CRITICAL FIXES:**
- **FIXED: Power Management Bug** - BLE no longer incorrectly stops after 5 minutes during active use
  - Device now properly tracks user activity (motion, button presses)
  - BLE timeout resets when user is active
  - Safety checks prevent LOW_POWER mode during recent activity
  - Resolves heading display freeze, unresponsive steering, ESP_ERR_INVALID_STATE errors
  
- **FIXED: IMU Timing & Drift** - Stable heading tracking restored
  - Hybrid delay approach: vTaskDelay + delayMicroseconds for timing precision
  - Mahony filter now uses actual measured frequency (not target constant)
  - Drift reduced from 3-5°/min (RC2) to <0.5°/min
  - Smooth centering force behavior, no more wild swings

- **IMPROVED: Smart Display Updates**
  - 10 FPS when Zwift connected OR yaw changes OR motion detected
  - 5 FPS when BLE connected but idle
  - 1 FPS only when truly idle (no connection, no changes)
  - Responsive steering feel during active use

**Impact:**
- Heading display remains responsive after screen wake
- BLE stays active indefinitely during use
- Accurate, stable yaw tracking (minimal drift)
- Power efficiency from RC2 maintained
- **UPGRADE STRONGLY RECOMMENDED for all RC2 users**

**See:** [RELEASE_NOTES_v1.0-RC3.md](RELEASE_NOTES_v1.0-RC3.md) for complete technical details

---

### v1.0-RC2
**Major Power Optimization & Performance Release**

**Power Optimizations:**
- Reduced LOW_POWER_CPU_FREQ from 80MHz to 10MHz (significant power savings)
- LED breathing rate-limited to 20Hz (50ms intervals) for power savings
- Adaptive display refresh: 10 FPS active, 5 FPS idle BLE, 1 FPS disconnected
- Battery ADC reads now cached for 1 second instead of every frame
- Debug logging conditionally compiled with `#ifdef DEBUG_MODE`
- Replaced `delayMicroseconds()` busy-wait with `vTaskDelay()` for power-efficient sleep

**Performance Improvements:**
- BLE controller state checking prevents redundant init/enable calls
- Splash overlay uses char arrays instead of String objects (stack vs heap allocation)
- Smart display refresh only when content actually changes
- Button press duration constants now defined for maintainability

**Bug Fixes:**
- Fixed BLE controller conflicts when switching between power modes
- Corrected splash screen string handling for better memory efficiency
- Improved motion detection threshold accuracy

**Expected Impact:**
- 30-50% longer battery life in low power modes
- Smoother display updates during active use
- More reliable BLE connection management
- Reduced memory fragmentation

**See:** [RELEASE_NOTES_v1.0-RC2.md](RELEASE_NOTES_v1.0-RC2.md)

---

### v1.0-RC1
- Added LittleFS support for splash screen images
- Enhanced splash screen system with logo display
- Improved startup sequence
- Repository renamed from ZTEERStick to STERZStick

---

Built for the Zwift cycling community. 