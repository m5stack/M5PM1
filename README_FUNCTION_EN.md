# M5PM1 Function Overview (English)

Feature overview of the PM1 power management driver library for Arduino/ESP-IDF.

- **Version**: 1.0.1
- **Default I2C address**: `0x6E`
- **I2C speed**: Default 100 kHz, can switch to 400 kHz (the library rebuilds I2C internally after switching)

## Project Structure and Roles

- `src/M5PM1.h`: Public APIs, registers/enums, cache structure definitions.
- `src/M5PM1.cpp`: Driver implementation, conflict checks, snapshot and cache logic.
- `src/M5PM1_i2c_compat.h`: Arduino/ESP-IDF I2C compatibility wrapper.

## Examples

- `examples/basic_power_adc/basic_power_adc.ino`: Initialization, device info, VBAT/VIN/5V readings, ADC/temperature sampling, power rail control.
- `examples/gpio_pwm/gpio_pwm.ino`: GPIO output/input reads and PWM breathing LED demo.
- `examples/usb_interrupt_sleep/usb_interrupt_sleep.ino`: USB plug/unplug interrupt, timer wake, and shutdown demo.
- `examples/neopixel/neopixel.ino`: NeoPixel rainbow demo.

## Feature Categories

### 1) Initialization and Logging

- **Initialization**
  - Arduino: `begin(TwoWire*, addr, sda, scl, speed)`
  - ESP-IDF: `begin(i2c_port_t, addr, sda, scl, speed)`
  - ESP-IDF: `begin(i2c_master_bus_handle_t, addr, speed)`
  - ESP-IDF: `begin(i2c_bus_handle_t, addr, speed)`
- **Logging**
  - `setLogLevel`/`getLogLevel`

### 2) Device Information

- `getDeviceId`/`getDeviceModel`/`getHwVersion`/`getSwVersion`

### 3) GPIO Basics and Advanced Control

- **Arduino style**
  - `pinMode`/`digitalWrite`/`digitalRead`
- **With return status**
  - `pinModeWithRes`/`digitalWriteWithRes`/`digitalReadWithRes`
- **Advanced configuration**
  - `gpioSet`/`gpioSetFunc`/`gpioSetMode`/`gpioSetOutput`/`gpioGetInput`
  - `gpioSetPull`/`gpioSetDrive`/`ledEnSetDrive`
  - `gpioSetWakeEnable`/`gpioSetWakeEdge`
- **Pin status and validation**
  - `dumpPinStatus`/`verifyPinConfig`/`getPinStatus`/`getPinStatusArray`

### 4) Power Hold

- `gpioSetPowerHold`/`gpioGetPowerHold`
- `ldoSetPowerHold`/`ldoGetPowerHold`
- `dcdcSetPowerHold`/`dcdcGetPowerHold`

### 5) ADC and Temperature

- `analogRead`/`isAdcBusy`/`disableAdc`/`readTemperature`

### 6) PWM

- `setPwmFrequency`/`getPwmFrequency`
- `setPwmDuty`/`getPwmDuty` (0-100%)
- `setPwmDuty12bit`/`getPwmDuty12bit` (0-4095)
- `setPwmConfig`
- `analogWrite` (0-255)

### 7) Voltage Reading

- `readVref`/`getRefVoltage`/`readVbat`/`readVin`/`read5VInOut`

### 8) Power Management and Battery

- `getPowerSource`
- `getWakeSource`/`clearWakeSource`
- `setPowerConfig`/`getPowerConfig`/`clearPowerConfig`
- `setChargeEnable`/`setDcdcEnable`/`setLdoEnable`/`set5VInOutEnable`/`setLedEnLevel`
- `setBatteryLvp`

### 9) Watchdog and Timer

- `wdtSet`/`wdtFeed`/`wdtGetCount`
- `timerSet`/`timerClear`

### 10) Button

- `btnSetConfig`/`btnGetState`/`btnGetFlag`
- `setSingleResetDisable` (High risk)/`getSingleResetDisable`
- `setDoubleOffDisable` (High risk)/`getDoubleOffDisable`

### 11) Interrupts

- **Status read and clear**
  - `irqGetGpioStatus`/`irqClearGpioAll`
  - `irqGetSysStatus`/`irqClearSysAll`
  - `irqGetBtnStatus`/`irqClearBtnAll`
- **Enum-style read**
  - `irqGetGpioStatusEnum`/`irqGetSysStatusEnum`/`irqGetBtnStatusEnum`
- **Interrupt masks**
  - GPIO: `irqSetGpioMask(m5pm1_irq_gpio_t, m5pm1_irq_mask_ctrl_t)`/`irqGetGpioMask`/`irqSetGpioMaskAll(m5pm1_irq_mask_ctrl_t)`/`irqGetGpioMaskBits`
  - System: `irqSetSysMask(m5pm1_irq_sys_t, m5pm1_irq_mask_ctrl_t)`/`irqGetSysMask`/`irqSetSysMaskAll(m5pm1_irq_mask_ctrl_t)`/`irqGetSysMaskBits`
  - Button: `irqSetBtnMask(m5pm1_btn_irq_t, m5pm1_irq_mask_ctrl_t)`/`irqGetBtnMask`/`irqSetBtnMaskAll(m5pm1_irq_mask_ctrl_t)`/`irqGetBtnMaskBits`
  - Note: Single set interfaces do not support `ALL`/`NONE` enum values, use `xxxMaskAll()` for batch operations

### 12) System Commands

- `sysCmd`/`shutdown`/`reboot`/`enterDownloadMode`
- `setDownloadLock` (High risk)/`getDownloadLock`

### 13) NeoPixel

- `setLeds`/`setLedCount`/`setLedColor`/`refreshLeds`/`disableLeds`

### 14) AW8737A Pulse Control

- `setAw8737aPulse`/`refreshAw8737aPulse`

### 15) RTC RAM

- `writeRtcRAM`/`readRtcRAM`

### 16) I2C Configuration and Auto Wake

- `setI2cConfig`/`switchI2cSpeed`/`getI2cSpeed`
- `setI2cSleepTime`/`getI2cSleepTime`
- `setAutoWakeEnable`/`isAutoWakeEnabled`/`sendWakeSignal`
- Compatibility layer: `M5PM1_i2c_compat.h`

### 17) State Snapshot/Cache/Validation

- `setAutoSnapshot`/`isAutoSnapshotEnabled`/`updateSnapshot`/`verifySnapshot`
- `validateConfig`
- `getCachedPwmFrequency`/`getCachedPwmState`/`getCachedAdcState`
- `getCachedPowerConfig`/`getCachedButtonConfig`
- `getCachedIrqMasks`/`getCachedIrqStatus`

## Precautions

If using high-risk functions `setDownloadLock`, `setSingleResetDisable`, or `setDoubleOffDisable` results in the inability to enable download mode, single-click reset, or double-click shutdown, please disconnect/reconnect the battery (devices with built-in batteries may risk damage to the casing and components) or quickly short BAT and GND (this operation may damage the battery).

## Pin and Function Limits

| Area | Description | Notes |
| :--- | :--- | :--- |
| **WAKE mutual exclusion** | GPIO0/2 are mutually exclusive; GPIO3/4 are mutually exclusive | Only for WAKE function | 
| **Wake (WAKE)** | GPIO0/2/3/4 support wake | GPIO1 does not support WAKE |
| **ADC channels** | GPIO1=ADC1, GPIO2=ADC2 | Temperature is an internal channel |
| **PWM channels** | GPIO3=PWM0, GPIO4=PWM1 | Frequency shared across channels |
| **NeoPixel** | GPIO0 only; LED count 1-31; data area 0x60 - 0x9F | I2C briefly non-interruptible during refresh |
| **I2C sleep** | Idle timeout can enter low power | Sleep is ineffective when PWM is enabled or in download mode |
| **Register blocks** | 0x00 - 0x0C, 0x10 - 0x19, 0x20 - 0x2A, 0x30 - 0x35, 0x38 - 0x3D, 0x40 - 0x45, 0x48 - 0x4A, 0x50, 0x53, 0x60 - 0x9F, 0xA0 - 0xBF | Supports continuous read/write |

## Version and Dependencies

- **Version**: 1.0.1
- **Platforms**: Arduino / ESP-IDF (>=4.4)
- **Dependency**: espressif/i2c_bus
