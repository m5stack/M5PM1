# M5PM1

## Overview

**SKU: N/A**

M5PM1 is a dual-platform (ESP-IDF & Arduino) driver library for M5Stack PM1 Power Management IC. It provides comprehensive power management features including:

- Battery charging and monitoring
- Multiple power rails (DCDC 5V, LDO 3.3V)
- 5 GPIO pins with various functions (GPIO/IRQ/WAKE/PWM/ADC)
- PWM output (2 channels)
- ADC input (2 channels + temperature)
- NeoPixel LED control (up to 32 LEDs)
- Watchdog timer
- RTC RAM (32 bytes, retained in sleep)
- I2C auto-sleep/wake feature

## Features

- **Dual-Platform Support**: Works with both Arduino and ESP-IDF frameworks
- **Arduino-Style API**: Familiar `pinMode()`, `digitalWrite()`, `digitalRead()` functions
- **I2C Auto-Wake**: Automatically handles PM1 sleep mode wake-up
- **Comprehensive Power Management**: Battery charging, voltage monitoring, power hold

## Hardware

- **I2C Address**: 0x6E (default)
- **I2C Speed**: 100KHz (default), 400KHz (configurable)
- **GPIO Pins**: 5 (GPIO0-GPIO4)
- **PWM Channels**: 2 (GPIO3, GPIO4)
- **ADC Channels**: 2 (GPIO1, GPIO2) + Internal Temperature

## Pin Functions

| GPIO | Special Function | Description |
|------|-----------------|-------------|
| GPIO0 | LED_EN | NeoPixel LED enable |
| GPIO1 | ADC1 | Analog input channel 1 |
| GPIO2 | ADC2 | Analog input channel 2 |
| GPIO3 | PWM0 | PWM output channel 0 |
| GPIO4 | PWM1 | PWM output channel 1 |

## Usage

### Arduino

```cpp
#include <M5PM1.h>

M5PM1 pm1;

void setup() {
    Serial.begin(115200);
    Wire.begin(38, 39);  // SDA, SCL

    if (pm1.begin(&Wire) != M5PM1_OK) {
        Serial.println("PM1 init failed!");
        while (1) delay(100);
    }

    // Use Arduino-style API
    pm1.pinMode(0, OUTPUT);
    pm1.digitalWrite(0, HIGH);

    // Read battery voltage
    uint16_t vbat;
    if (pm1.readVbat(&vbat) == M5PM1_OK) {
        Serial.printf("Battery: %d mV\n", vbat);
    }
}

void loop() {
    // Toggle GPIO0
    pm1.digitalWrite(0, HIGH);
    delay(500);
    pm1.digitalWrite(0, LOW);
    delay(500);
}
```

### ESP-IDF

```cpp
#include "M5PM1.h"

M5PM1 pm1;

void app_main() {
    // Initialize with self-created I2C bus
    if (pm1.begin(I2C_NUM_0, M5PM1_DEFAULT_ADDR, 21, 22, 100000) != M5PM1_OK) {
        ESP_LOGE("PM1", "Init failed!");
        return;
    }

    // Or use existing i2c_master_bus_handle_t
    // pm1.begin(existing_bus_handle, M5PM1_DEFAULT_ADDR, 100000);

    // Configure GPIO
    pm1.gpioSetFunc(M5PM1_GPIO_NUM_0, M5PM1_GPIO_FUNC_GPIO);
    pm1.gpioSetMode(M5PM1_GPIO_NUM_0, M5PM1_GPIO_MODE_OUTPUT);
    pm1.gpioSetOutput(M5PM1_GPIO_NUM_0, 1);
}
```

## API Reference

### Initialization

| Method | Description |
|--------|-------------|
| `begin()` | Initialize the PM1 device |
| `setAutoWakeEnable()` | Enable/disable auto-wake feature |
| `sendWakeSignal()` | Manually send wake signal |

### GPIO Functions

| Method | Description |
|--------|-------------|
| `pinMode()` | Set GPIO mode (Arduino-style) |
| `digitalWrite()` | Set GPIO output (Arduino-style) |
| `digitalRead()` | Read GPIO input (Arduino-style) |
| `gpioSetFunc()` | Set GPIO function (GPIO/IRQ/WAKE/OTHER) |
| `gpioSetMode()` | Set GPIO direction |
| `gpioSetOutput()` | Set GPIO output level |
| `gpioGetInput()` | Get GPIO input level |
| `gpioSetPull()` | Set GPIO pull-up/pull-down |
| `gpioSetDrive()` | Set GPIO drive mode |

### Power Management

| Method | Description |
|--------|-------------|
| `getPowerSource()` | Get current power source |
| `getWakeSource()` | Get wake source flags |
| `setPowerConfig()` | Set power configuration |
| `setChargeEnable()` | Enable/disable battery charging |
| `setDcdcEnable()` | Enable/disable 5V DCDC |
| `setLdoEnable()` | Enable/disable 3.3V LDO |
| `shutdown()` | Shutdown the system |
| `reboot()` | Reboot the system |

### Voltage Reading

| Method | Description |
|--------|-------------|
| `readVref()` | Read reference voltage |
| `readVbat()` | Read battery voltage |
| `readVin()` | Read VIN voltage |
| `read5VInOut()` | Read 5VINOUT voltage |

### PWM Functions

| Method | Description |
|--------|-------------|
| `setPwmFrequency()` | Set PWM frequency |
| `getPwmFrequency()` | Get PWM frequency |
| `setPwmDuty()` | Set PWM duty (percentage) |
| `getPwmDuty()` | Get PWM duty (percentage) |
| `setPwmDuty12bit()` | Set PWM duty (12-bit) |
| `getPwmDuty12bit()` | Get PWM duty (12-bit) |
| `analogWrite()` | Arduino-compatible PWM output |

### ADC Functions

| Method | Description |
|--------|-------------|
| `analogRead()` | Read ADC value |
| `isAdcBusy()` | Check ADC busy status |
| `disableAdc()` | Disable ADC conversion |

### NeoPixel Functions

| Method | Description |
|--------|-------------|
| `setLeds()` | Configure and set all LEDs |
| `setLedCount()` | Set LED count |
| `setLedColor()` | Set LED color |
| `refreshLeds()` | Refresh LED display |
| `disableLeds()` | Disable LED output |

### Watchdog Functions

| Method | Description |
|--------|-------------|
| `wdtSet()` | Set watchdog timeout |
| `wdtFeed()` | Feed the watchdog |
| `wdtGetCount()` | Get watchdog countdown |

### RTC RAM Functions

| Method | Description |
|--------|-------------|
| `writeRtcRAM()` | Write to RTC RAM |
| `readRtcRAM()` | Read from RTC RAM |

## Related Link

- [Document & Datasheet](https://docs.m5stack.com/en/unit/pm1)

## Required Libraries

- None (standalone library)

## License

- [M5PM1 - MIT](LICENSE)
