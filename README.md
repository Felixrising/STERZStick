# STERZStick v1.0-RC1

A high-performance Zwift steering device using the M5StickCPlus2 with advanced IMU/AHRS implementation, power management, and BLE connectivity.

## 🎯 About

This project transforms your M5StickCPlus2 into a professional-grade steering controller for Zwift cycling simulation. Based on the excellent work from [@stefaandesmet2003/ESP32Sterzo](https://github.com/stefaandesmet2003/ESP32Sterzo.git), this implementation adds significant enhancements including:

- **Mahony AHRS algorithm** for stable yaw calculation
- **Advanced power management** with adaptive CPU/IMU frequency scaling
- **Motion-based sleep system** with intelligent timeout management
- **Comprehensive calibration system** with visual feedback
- **1-degree steering precision** (vs 10-degree binning)
- **Drift compensation** and automatic centering
- **Professional button handling** with short/long press actions

## 🛒 Get Your M5StickCPlus2

[![M5StickCPlus2](images/4_0977f4c6-d95d-49bd-8dc0-aa02c9464fd8_1200x1200.webp)](https://s.click.aliexpress.com/e/_oocIGLh)

**Support further development!** This is an [affiliate link](https://s.click.aliexpress.com/e/_oocIGLh) to purchase the M5StickCPlus2. Your purchase helps fund ongoing improvements to STERZStick.

## 🚀 Features

### Core Functionality
- ✅ **Zwift BLE Integration** - Full compatibility with Zwift steering protocol
- ✅ **Mahony AHRS Filter** - Superior stability compared to basic IMU readings
- ✅ **1° Steering Precision** - Fine-grained control for competitive cycling
- ✅ **Automatic Calibration** - Smart startup calibration with manual override
- ✅ **Drift Compensation** - Gradual centering force prevents long-term drift

### Power Management
- ⚡ **Optimized IMU Frequency** - 25Hz operation for balanced performance and power efficiency
- ⚡ **Efficient CPU Frequency** - 80MHz operation for optimal BLE compatibility and power savings
- ⚡ **Smart Sleep System** - Multiple sleep modes based on activity and connection status
- ⚡ **Motion Detection** - 30-second motion detection for intelligent wake/sleep decisions
- ⚡ **Adaptive Power States** - Automatic transitions between active, low-power, and sleep modes

### User Interface
- 📱 **Real-time Display** - Yaw angle, steering bin, BLE status, power mode
- 📱 **Visual Limit Indicators** - Flashing arrows at ±40° steering limits
- 📱 **Battery Monitoring** - Voltage display with 0.01V precision
- 📱 **Calibration Progress** - Visual progress bars and audio feedback
- 📱 **Power Status** - Shows current power mode and sleep countdowns

### Button Controls
- 🔘 **Button A Short** - Wake display and reset activity timer
- 🔘 **Button A Long (2s)** - Quick yaw recenter
- 🔘 **Button B Short** - Quick yaw recenter
- 🔘 **Button B Long (2s)** - Full calibration sequence
- 🔘 **Button C Short** - Wake display
- 🔘 **Button C Medium (2-3s)** - Wake display
- 🔘 **Button C Long (3s)** - Power off/sleep mode

## 🛠 Hardware Requirements

- **M5StickCPlus2** - Main device with built-in IMU (MPU6886)
- **USB-C Cable** - For programming and charging
- **Battery** - Built-in 200mAh LiPo battery

## 📦 Installation

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

## 🎮 Usage

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
- **Active Use** - Full performance during Zwift sessions (80MHz CPU, 25Hz IMU)
- **Low Power Mode** - Automatic power reduction after 2 minutes of inactivity
- **BLE Wait Mode** - 5-minute timeout waiting for Zwift connection
- **Motion Detection** - 30-second motion detection for smart wake/sleep decisions
- **Wake Up** - Move device or press any button to wake from sleep

## 🔧 Technical Details

### IMU Configuration
- **Sensor** - MPU6886 6-axis IMU (gyroscope + accelerometer)
- **Algorithm** - Mahony AHRS for quaternion-based orientation
- **Sampling Rate** - 25Hz for optimal performance and power efficiency
- **Calibration** - 6-point accelerometer + gyro bias compensation

### BLE Protocol
- **Service UUID** - `347b0001-7635-408b-8918-8ff3949ce592`
- **Steering Data** - Float32 angle in degrees (-40° to +40°)
- **Update Rate** - On-change notification (1° resolution)
- **Zwift Handshake** - Full protocol compatibility

### Power Specifications
- **Active Current** - ~50-80mA (80MHz CPU, 25Hz IMU, BLE active)
- **Low Power Current** - ~30-50mA (80MHz CPU, 25Hz IMU, reduced BLE)
- **Sleep Current** - ~5-10mA (timer sleep cycles)
- **Battery Life** - 3-5 hours active use, days/weeks standby

## 🔄 Calibration System

### Automatic Calibration (First Boot)
1. **Gyro Bias** - 3000 samples over 3 seconds (device stationary)
2. **Accelerometer** - 6-point calibration with audio/visual guidance
3. **Yaw Centering** - Sets current orientation as center position

### Manual Calibration
- **Quick Recenter** - Short press Button A or B
- **Full Calibration** - Long press Button A (2s) or Button B (2s)
- **Progress Feedback** - Visual progress bars and audio beeps

## 🐛 Troubleshooting

### Common Issues

**Device not connecting to Zwift:**
- Ensure Bluetooth is enabled on your device
- Restart Zwift and search for "STERZO" in connections
- Try power cycling the M5StickCPlus2

**Steering feels unstable:**
- Perform full calibration (long press Button A or B)
- Ensure device is mounted securely
- Check battery level (low battery affects performance)

**Device goes to sleep too quickly:**
- Motion detection requires actual movement/rotation
- BLE connection prevents sleep mode
- Adjust timeout values in code if needed

**Yaw drift over time:**
- Built-in drift compensation should handle minor drift
- Perform quick recenter (short press Button A or B)
- Full recalibration may be needed for major drift

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

## 🔋 Battery Optimization Tips

1. **Use during Zwift sessions only** - Device sleeps when not connected
2. **Regular charging** - Don't let battery fully discharge
3. **Stable mounting** - Reduces unnecessary motion detection
4. **Firmware updates** - Keep firmware updated for latest power optimizations

## 🤝 Contributing

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

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **[@stefaandesmet2003](https://github.com/stefaandesmet2003/ESP32Sterzo.git)** - Original ESP32Sterzo implementation
- **M5Stack** - M5StickCPlus2 hardware and libraries
- **Zwift** - Steering protocol documentation and support
- **Mahony & Madgwick** - AHRS algorithm implementations

## 📊 Project Status

- ✅ **Core Functionality** - Complete and tested
- ✅ **Power Management** - Optimized for battery life
- ✅ **Zwift Integration** - Fully compatible
- ✅ **Calibration System** - Comprehensive and user-friendly
- 🔄 **Documentation** - Ongoing improvements
- 🔄 **Testing** - Continuous validation with Zwift updates

---

**Built with ❤️ for the Zwift cycling community** 