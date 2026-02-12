# M5StickS3 Board Information

**SKU:** K150 | **Product:** StickS3

Compact ESP32-S3 IoT development kit with 1.14" display, speaker, MEMS mic, IR TX/RX, and 6-axis IMU. Uses M5PM1 for power management (no RTC; timer-based power control).

---

## Power Management

### M5PM1 Power Chip

The StickS3 uses **M5PM1** (not AXP2101) for power management. Key differences from M5StickC Plus2:

- No HOLD pin; power is controlled via M5PM1
- No RTC; M5PM1 provides a timer for timed power on/off/reboot
- `M5.Power.powerOff()` triggers PMIC-level shutdown for true power-off
- Power levels L0–L3B control peripheral power; L1 keeps IMU powered for motion wake

### Power On

- **Single press** power button (side) – power on
- **Download mode:** USB connected, long-press power button until green LED flashes

### Power Off

- **Double press** power button – power off (PMIC shutdown)
- **Software:** `M5.Power.powerOff()` for programmatic shutdown

### Power Consumption (4.2V typical)

| State | Current |
|-------|---------|
| Power off | 14.02 µA |
| L1 (IMU only) | 52.47 µA |
| L2 | 102.40 µA |
| L3A (active, low) | 36.69 mA |
| Full load | 519.02 mA |

### IMU Motion Wake

M5PM1 can stay at L1 (IMU + PMIC) while ESP32-S3 is off. IMU INT1 (PYG4) can wake M5PM1 on motion. IMU interrupt is connected to M5PM1 PYG4; PYG1_IRQ connects to ESP32-S3 G13 for chained wake-up.

---

## Hardware Components

### SoC: ESP32-S3-PICO-1-N8R8

- **CPU:** Dual-core Xtensa LX7, up to 240 MHz
- **Flash:** 8 MB QSPI
- **PSRAM:** 8 MB QSPI
- **Connectivity:** 2.4 GHz Wi-Fi 4, Bluetooth 5 LE + Mesh

### Buttons

| ESP32-S3 | Function |
|----------|----------|
| G11 | KEY1 (Button A) |
| G12 | KEY2 (Button B) |

**Power button (side):** Single press = power on, double press = power off, long press = download mode. Not a general-purpose GPIO.

### Display

| ESP32-S3 | ST7789P3 |
|----------|----------|
| G39 | MOSI |
| G40 | SCK |
| G45 | RS (DC) |
| G41 | CS |
| G21 | RST |
| G38 | BL (backlight) |

- **Driver:** ST7789P3
- **Resolution:** 135 x 240 pixels
- **Size:** 1.14" IPS

### IMU (BMI270)

| ESP32-S3 | BMI270 |
|----------|--------|
| G48 | SCL |
| G47 | SDA |

- **Sensor:** BMI270 (6-axis; different from MPU6886 on Plus2)
- **I2C address:** 0x68 (default)
- **Interrupt:** IMU INT1 → M5PM1 PYG4 (for motion wake)

### M5PM1 (Power Management)

| M5PM1 Pin | ESP32-S3 | Function |
|-----------|----------|----------|
| PYG0_CHG_STAT | G0 | Battery charge status |
| PYG1_IRQ | G1 | IRQ output (e.g. to G13 for wake) |
| PYG2 | – | L3B enable (LCD, MIC, SPK) |
| PYG3 | – | Speaker amp enable |
| PYG4_IMU_INT | G4 | IMU INT1 input (motion wake) |

M5PM1 shares I2C with BMI270: SCL=G48, SDA=G47.

### Audio

| ESP32-S3 | ES8311 |
|----------|--------|
| G18 | MCLK |
| G14 | DOUT |
| G17 | BCLK |
| G15 | LRCK |
| G16 | DIN |
| G48 | SCL |
| G47 | SDA |

- **Codec:** ES8311 (24-bit, I2S)
- **Mic:** MEMS, 65 dB SNR
- **Speaker:** 8 Ω / 1 W, AW8737 amplifier

**Note:** Disable speaker amp when using IR receive, or reception will fail.

### IR

| ESP32-S3 | Function |
|----------|----------|
| G46 | IR_TX |
| G42 | IR_RX |

Both TX and RX (unlike Plus2, which has TX only).

### Grove (HY2.0-4P PORT.A)

| Pin | Color | Function |
|-----|-------|----------|
| 1 | Black | GND |
| 2 | Red | 5V |
| 3 | Yellow | G9 |
| 4 | White | G10 |

### Hat2-Bus (2.54mm, 16-pin)

| Pin | Left | Right | Pin |
|-----|------|-------|-----|
| 1 | GND | 2 | G5 |
| 3 | EXT_5V | 4 | G4 |
| 5 | Boot | 6 | G6 |
| 7 | G1 | 8 | G7 |
| 9 | G8 | 10 | G43 |
| 11 | BAT | 12 | G44 |
| 13 | 3V3_L2 | 14 | G2 |
| 15 | 5V_IN | 16 | G3 |

---

## Physical Specifications

| Parameter | Value |
|-----------|-------|
| Size | 48.0 x 24.0 x 15.0 mm |
| Weight | 20.0 g |
| Battery | 250 mAh LiPo |
| Operating temp | 0–40 °C |
| Interface | USB Type-C (power, programming) |

---

## STERZStick Compatibility Notes

- **IMU:** StickS3 uses BMI270 instead of MPU6886; driver and calibration differ from Plus2.
- **Buttons:** 2 programmable keys (KEY1, KEY2) vs 3 on Plus2; power button is PMIC-managed.
- **Power:** Use `M5.Power.powerOff()` instead of HOLD-pin logic.
- **RTC:** No RTC; use M5PM1 timer for timed power on/off.
- **Extension:** Some Hat units (e.g. Hat Mini JoyC, Hat Mini EncoderC, Hat 18650C) are structurally incompatible.

---

## References

- [StickS3 Documentation](https://docs.m5stack.com/en/core/StickS3)
- [StickS3 M5PM1 Power Management](https://docs.m5stack.com/en/arduino/m5sticks3/m5pm1)
- [M5PM1 Datasheet](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1207/M5PM1_Datasheet_EN.pdf)
- [StickS3 Schematics PDF](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1207/K150_Stick_S3_PRJ_V0.6_20251111_2025_11_17_16_10_24.pdf)
- [M5PM1 Arduino Library](https://github.com/m5stack/M5PM1)
- [BMI270 Arduino Library (SparkFun)](https://github.com/sparkfun/SparkFun_BMI270_Arduino_Library)
