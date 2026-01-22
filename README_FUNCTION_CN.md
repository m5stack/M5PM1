# M5PM1 功能总览（中文）

面向 Arduino/ESP-IDF 双平台的 PM1 电源管理驱动库功能分类说明。

- **版本**: 1.0.1
- **默认 I2C 地址**: `0x6E`
- **I2C 速度**: 默认 100 kHz，可切换 400 kHz（切换后库内部会处理 I2C 重建）

## 项目结构与角色

- `src/M5PM1.h`: 公共 API、寄存器/枚举、缓存结构定义。
- `src/M5PM1.cpp`: 驱动实现、冲突校验、快照与缓存逻辑。
- `src/M5PM1_i2c_compat.h`: Arduino/ESP-IDF I2C 读写兼容封装。

## 示例

- `examples/basic_power_adc/basic_power_adc.ino`: 初始化、设备信息、VBAT/VIN/5V 读数、ADC/温度采样、电源轨控制示例。
- `examples/gpio_pwm/gpio_pwm.ino`: GPIO 输出/输入读取与 PWM 呼吸灯示例。
- `examples/interrupt_button_sleep/interrupt_button_sleep.ino`: GPIO 中断、按钮 API、定时器/唤醒与关机示例。
- `examples/neopixel/neopixel.ino`: NeoPixel 彩虹渐变示例。

## 功能分类

### 1) 初始化与日志

- **初始化**
  - Arduino: `begin(TwoWire*, addr, sda, scl, speed)`
  - ESP-IDF: `begin(i2c_port_t, addr, sda, scl, speed)`
  - ESP-IDF: `begin(i2c_master_bus_handle_t, addr, speed)`
  - ESP-IDF: `begin(i2c_bus_handle_t, addr, speed)`
- **日志**
  - `setLogLevel`/`getLogLevel`

### 2) 设备信息

- `getDeviceId`/`getDeviceModel`/`getHwVersion`/`getSwVersion`

### 3) GPIO 基础与高级控制

- **Arduino 风格**
  - `pinMode`/`digitalWrite`/`digitalRead`
- **带返回值版本**
  - `pinModeWithRes`/`digitalWriteWithRes`/`digitalReadWithRes`
- **高级配置**
  - `gpioSet`/`gpioSetFunc`/`gpioSetMode`/`gpioSetOutput`/`gpioGetInput`
  - `gpioSetPull`/`gpioSetDrive`/`ledEnSetDrive`
  - `gpioSetWakeEnable`/`gpioSetWakeEdge`
- **引脚状态与校验**
  - `dumpPinStatus`/`verifyPinConfig`/`getPinStatus`/`getPinStatusArray`

### 4) 电源保持

- `gpioSetPowerHold`/`gpioGetPowerHold`
- `ldoSetPowerHold`/`ldoGetPowerHold`
- `dcdcSetPowerHold`/`dcdcGetPowerHold`

### 5) ADC 与温度

- `analogRead`/`isAdcBusy`/`disableAdc`/`readTemperature`

### 6) PWM

- `setPwmFrequency`/`getPwmFrequency`
- `setPwmDuty`/`getPwmDuty` (0-100%)
- `setPwmDuty12bit`/`getPwmDuty12bit` (0-4095)
- `setPwmConfig`
- `analogWrite` (0-255)

### 7) 电压读取

- `readVref`/`getRefVoltage`/`readVbat`/`readVin`/`read5VInOut`

### 8) 电源管理与电池

- `getPowerSource`
- `getWakeSource`/`clearWakeSource`
- `setPowerConfig`/`getPowerConfig`/`clearPowerConfig`
- `setChargeEnable`/`setDcdcEnable`/`setLdoEnable`/`set5VInOutEnable`/`setLedEnLevel`
- `setBatteryLvp`

### 9) 看门狗与定时器

- `wdtSet`/`wdtFeed`/`wdtGetCount`
- `timerSet`/`timerClear`

### 10) 按钮

- `btnSetConfig`/`btnGetState`/`btnGetFlag`
- `setSingleResetDisable`（高危）/`getSingleResetDisable`
- `setDoubleOffDisable`（高危）/`getDoubleOffDisable`

### 11) 中断

- **状态读取与清除**
  - `irqGetGpioStatus`/`irqClearGpioAll`
  - `irqGetSysStatus`/`irqClearSysAll`
  - `irqGetBtnStatus`/`irqClearBtnAll`
- **枚举式读取**
  - `irqGetGpioStatusEnum`/`irqGetSysStatusEnum`/`irqGetBtnStatusEnum`
- **中断屏蔽**
  - GPIO: `irqSetGpioMask(m5pm1_irq_gpio_t, m5pm1_irq_mask_ctrl_t)`/`irqGetGpioMask`/`irqSetGpioMaskAll(m5pm1_irq_mask_ctrl_t)`/`irqGetGpioMaskBits`
  - 系统: `irqSetSysMask(m5pm1_irq_sys_t, m5pm1_irq_mask_ctrl_t)`/`irqGetSysMask`/`irqSetSysMaskAll(m5pm1_irq_mask_ctrl_t)`/`irqGetSysMaskBits`
  - 按钮: `irqSetBtnMask(m5pm1_btn_irq_t, m5pm1_irq_mask_ctrl_t)`/`irqGetBtnMask`/`irqSetBtnMaskAll(m5pm1_irq_mask_ctrl_t)`/`irqGetBtnMaskBits`
  - 注意: 单个设置接口不支持 `ALL`/`NONE` 枚举值，批量操作请使用 `xxxMaskAll()` 函数

### 12) 系统命令

- `sysCmd`/`shutdown`/`reboot`/`enterDownloadMode`
- `setDownloadLock`（高危）/`getDownloadLock`

### 13) NeoPixel

- `setLeds`/`setLedCount`/`setLedColor`/`refreshLeds`/`disableLeds`

### 14) AW8737A 脉冲控制

- `setAw8737aPulse`/`refreshAw8737aPulse`

### 15) RTC RAM

- `writeRtcRAM`/`readRtcRAM`

### 16) I2C 配置与自动唤醒

- `setI2cConfig`/`switchI2cSpeed`/`getI2cSpeed`
- `setI2cSleepTime`/`getI2cSleepTime`
- `setAutoWakeEnable`/`isAutoWakeEnabled`/`sendWakeSignal`
- 兼容层: `M5PM1_i2c_compat.h`

### 17) 状态快照/缓存/校验

- `setAutoSnapshot`/`isAutoSnapshotEnabled`/`updateSnapshot`/`verifySnapshot`
- `validateConfig`
- `getCachedPwmFrequency`/`getCachedPwmState`/`getCachedAdcState`
- `getCachedPowerConfig`/`getCachedButtonConfig`
- `getCachedIrqMasks`/`getCachedIrqStatus`

## 注意事项

如果使用 `setDownloadLock`、`setSingleResetDisable`、`setDoubleOffDisable` 高危函数导致无法启用烧录、单击复位、双击关断，请拔插电池（内置电池设备可能损坏壳体和器件）或快速短接 BAT 和 GND（此操作可能损坏电池）。

## 引脚与功能限制

| 功能区域 | 说明 | 备注 |
| :--- | :--- | :--- |
| **WAKE 互斥** | GPIO0/2 互斥；GPIO3/4 互斥 | 仅 WAKE 功能生效 |
| **唤醒 (WAKE)** | GPIO0/2/3/4 支持唤醒 | GPIO1 不支持 WAKE |
| **ADC 通道** | GPIO1=ADC1, GPIO2=ADC2 | 温度为内部通道 |
| **PWM 通道** | GPIO3=PWM0, GPIO4=PWM1 | 频率全通道共享 |
| **NeoPixel** | 仅 GPIO0 支持；LED 数量 1-31；数据区 0x60 - 0x9F | 刷新时 I2C 短暂不可中断 |
| **I2C 休眠** | 空闲超时可进入低功耗 | PWM 使能或下载模式下休眠失效 |
| **寄存器区块** | 0x00 - 0x0C, 0x10 - 0x19, 0x20 - 0x2A, 0x30 - 0x35, 0x38 - 0x3D, 0x40 - 0x45, 0x48 - 0x4A, 0x50, 0x53, 0x60 - 0x9F, 0xA0 - 0xBF | 支持连续读写 |

## 版本与依赖

- **版本**: 1.0.1
- **平台**: Arduino / ESP-IDF (>=4.4)
- **依赖**: espressif/i2c_bus

