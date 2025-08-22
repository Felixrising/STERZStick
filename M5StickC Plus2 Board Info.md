# M5StickC Plus2 Board Information

## Power Management

### Power On
- Press **BUTTON C** for more than 2 seconds, OR
- Wake up via RTC IRQ signal
- **Important**: After wake-up, the program must set the HOLD pin (GPIO4) to HIGH (1) to keep power on, otherwise the device will shut down again.

### Power Off
- **Without USB power**: Press **BUTTON C** for more than 6 seconds, OR set HOLD (GPIO4) = 0 in the program
- **With USB connected**: Press **BUTTON C** for more than 6 seconds will turn off screen and enter sleep mode (not full power-off)

### Deep Sleep on Battery Power
- **CRITICAL**: When entering deep sleep on battery power, you must enable GPIO hold on the HOLD pin:
  ```cpp
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  rtc_gpio_hold_en(GPIO_NUM_4);  // Keep HOLD=1 while sleeping
  esp_deep_sleep_start();
  ```
- **Why**: Without `rtc_gpio_hold_en(GPIO_NUM_4)`, the HOLD pin will float during deep sleep, causing the device to lose power and not wake up properly
- **Wake-up**: After wake-up from deep sleep, immediately set HOLD pin HIGH again:
  ```cpp
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  rtc_gpio_hold_en(GPIO_NUM_4);  // Maintain hold state
  ```

## Hardware Components

### ESP32-PICO-V3-02 Microcontroller
- **GPIO19**: IR Emitter & Red LED
- **GPIO37**: Button A
- **GPIO39**: Button B  
- **GPIO35**: Button C
- **GPIO2**: Passive Buzzer

### Display
- **Driver IC**: ST7789V2
- **Resolution**: 135 x 240 pixels
- **Color**: TFT

#### TFT Display Pin Mapping
| ESP32 Pin | Function |
|-----------|----------|
| G15 | TFT_MOSI |
| G13 | TFT_CLK |
| G14 | TFT_DC |
| G12 | TFT_RST |
| G5 | TFT_CS |
| G27 | TFT_BL |

### Microphone (SPM1423)
| ESP32 Pin | Function |
|-----------|----------|
| G0 | CLK |
| G34 | DATA |

### Sensors
- **6-Axis IMU**: MPU6886
- **RTC**: BM8563
- **I2C Pins**: SCL=G22, SDA=G21

### IR Emitter & LED
- **Red LED**: TX=G19
- **IR Emitter**: TX=G19

### Extension Port (HY2.0-4P)
| Pin | Color | Function |
|-----|-------|----------|
| 1 | Black | GND |
| 2 | Red | 5V |
| 3 | Yellow | G32 |
| 4 | White | G33 |