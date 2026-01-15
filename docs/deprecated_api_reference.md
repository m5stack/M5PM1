# m5_stamp_pm1 到 M5PM1 API 迁移参考手册

本文档帮助用户从旧版 `m5_stamp_pm1` 库迁移到新版 `M5PM1` 库。

## 目录

- [概述](#概述)
- [主要变化](#主要变化)
- [初始化](#初始化)
- [错误处理](#错误处理)
- [GPIO 功能](#gpio-功能)
- [ADC 功能](#adc-功能)
- [PWM 功能](#pwm-功能)
- [电压读取](#电压读取)
- [电源管理](#电源管理)
- [看门狗](#看门狗)
- [定时器](#定时器)
- [按钮功能](#按钮功能)
- [中断功能](#中断功能)
- [NeoPixel LED](#neopixel-led)
- [RTC RAM](#rtc-ram)
- [系统命令](#系统命令)
- [I2C 配置](#i2c-配置)
- [枚举类型对照表](#枚举类型对照表)
- [完整迁移示例](#完整迁移示例)

---

## 概述

### 库名变化

| 旧库 | 新库 |
|------|------|
| `m5_stamp_pm1` | `M5PM1` |
| `#include "m5_stamp_pm1.h"` | `#include "M5PM1.h"` |

### 编程风格变化

- **旧库**: C 风格函数 `pm1_*()` 或通过单例类调用
- **新库**: 面向对象的 C++ 类实例方法

```cpp
// 旧库 (C 风格)
pm1_vbat_read(&voltage);

// 旧库 (单例类)
m5_stamp_pm1::getInstance().pm1_vbat_read(&voltage);

// 新库
M5PM1 pm1;
pm1.readVbat(&voltage);
```

---

## 主要变化

| 特性 | 旧库 (`m5_stamp_pm1`) | 新库 (`M5PM1`) |
|------|------------------------|----------------|
| 头文件 | `m5_stamp_pm1.h` | `M5PM1.h` |
| 错误类型 | `esp_err_t` | `m5pm1_err_t` |
| 初始化 | `pm1_init()` | `pm1.begin()` |
| GPIO 风格 | 原生函数 | Arduino 兼容 |
| 状态缓存 | 无 | 内置快照机制 |
| 日志系统 | ESP-IDF 原生 | 可配置级别 |

---

## 初始化

### 旧库

```cpp
#include "m5_stamp_pm1.h"

// Arduino
Wire.begin(SDA_PIN, SCL_PIN, 400000);
pm1_init(&Wire, SDA_PIN, SCL_PIN, 400000);

// 或使用单例
m5_stamp_pm1::getInstance().pm1_init(&Wire, SDA_PIN, SCL_PIN, 400000);
```

### 新库

```cpp
#include "M5PM1.h"

M5PM1 pm1;

// Arduino - 方式1: 使用已初始化的 Wire
Wire.begin(SDA_PIN, SCL_PIN, 400000);
pm1.begin(&Wire);

// Arduino - 方式2: 让库初始化 Wire
pm1.begin(&Wire, 0x6E, SDA_PIN, SCL_PIN, 400000);

// ESP-IDF - 自动创建 I2C 总线
pm1.begin(I2C_NUM_0, 0x6E, SDA_PIN, SCL_PIN, 400000);

// ESP-IDF - 使用已存在的 I2C 总线
pm1.begin(existing_bus_handle, 0x6E, 400000);
```

### API 对照

| 旧 API | 新 API |
|--------|--------|
| `pm1_init(wire, sda, scl, freq)` | `pm1.begin(wire, addr, sda, scl, speed)` |
| `pm1_deinit()` | *(析构函数自动处理)* |

---

## 错误处理

### 错误类型变化

| 旧类型 | 新类型 |
|--------|--------|
| `esp_err_t` | `m5pm1_err_t` |
| `ESP_OK` | `M5PM1_OK` |
| `ESP_FAIL` | `M5PM1_FAIL` |
| `ESP_ERR_INVALID_ARG` | `M5PM1_ERR_INVALID_ARG` |

### 新增错误码

```cpp
typedef enum {
    M5PM1_OK = 0,                  // 成功
    M5PM1_FAIL = -1,               // 一般失败
    M5PM1_ERR_I2C_CONFIG = -2,     // I2C 配置错误
    M5PM1_ERR_RULE_VIOLATION = -3, // 条件规则错误 (引脚冲突)
    M5PM1_ERR_INVALID_ARG = -4,    // 无效参数
    M5PM1_ERR_TIMEOUT = -5,        // 超时
    M5PM1_ERR_NOT_SUPPORTED = -6,  // 不支持的功能
    M5PM1_ERR_I2C_COMM = -7,       // I2C 通信错误
    M5PM1_ERR_NOT_INIT = -8,       // 设备未初始化
    M5PM1_ERR_INTERNAL = -9,       // 内部错误
} m5pm1_err_t;
```

---

## GPIO 功能

### 基础 GPIO 操作

新库提供 Arduino 兼容的 `pinMode()`, `digitalWrite()`, `digitalRead()` 函数。

| 旧 API | 新 API |
|--------|--------|
| `pm1_gpio_set_mode(pin, PM1_GPIO_MODE_OUTPUT)` | `pm1.pinMode(pin, OUTPUT)` |
| `pm1_gpio_set_mode(pin, PM1_GPIO_MODE_INPUT)` | `pm1.pinMode(pin, INPUT)` |
| `pm1_gpio_set_state(pin, PM1_GPIO_OUTPUT_HIGH)` | `pm1.digitalWrite(pin, HIGH)` |
| `pm1_gpio_set_state(pin, PM1_GPIO_OUTPUT_LOW)` | `pm1.digitalWrite(pin, LOW)` |
| `pm1_gpio_get_in_state(pin, &state)` | `value = pm1.digitalRead(pin)` |

### 高级 GPIO 配置

| 旧 API | 新 API |
|--------|--------|
| `pm1_gpio_set_func(pin, func)` | `pm1.gpioSetFunc(pin, func)` |
| `pm1_gpio_set(pin, mode, state, pupd, drv)` | `pm1.gpioSet(pin, mode, value, pull, drive)` |
| `pm1_gpio_set_pupd(pin, pupd)` | `pm1.gpioSetPull(pin, pull)` |
| `pm1_gpio_set_drv(pin, drv)` | `pm1.gpioSetDrive(pin, drive)` |
| `pm1_led_en_set_drv(drv)` | `pm1.ledEnSetDrive(drive)` |
| `pm1_gpio_set_wake_en(pin, enable)` | `pm1.gpioSetWakeEnable(pin, enable)` |
| `pm1_gpio_set_wake_cfg(pin, edge)` | `pm1.gpioSetWakeEdge(pin, edge)` |

### GPIO 电源保持

| 旧 API | 新 API |
|--------|--------|
| `pm1_gpio_set_power_hold(pin, enable)` | `pm1.gpioSetPowerHold(pin, enable)` |
| `pm1_gpio_get_power_hold(pin, &enable)` | `pm1.gpioGetPowerHold(pin, &enable)` |
| `pm1_ldo_set_power_hold(enable)` | `pm1.ldoSetPowerHold(enable)` |
| `pm1_ldo_get_power_hold(&enable)` | `pm1.ldoGetPowerHold(&enable)` |
| `pm1_dcdc_set_power_hold(enable)` | `pm1.dcdcSetPowerHold(enable)` |
| `pm1_dcdc_get_power_hold(&enable)` | `pm1.dcdcGetPowerHold(&enable)` |

### 示例对照

```cpp
// ===== 旧库 =====
pm1_gpio_set_func(PM1_GPIO_NUM_0, PM1_GPIO_FUNC_GPIO);
pm1_gpio_set_mode(PM1_GPIO_NUM_0, PM1_GPIO_MODE_OUTPUT);
pm1_gpio_set_state(PM1_GPIO_NUM_0, PM1_GPIO_OUTPUT_HIGH);
pm1_gpio_set_pupd(PM1_GPIO_NUM_0, PM1_GPIO_PUPD_NC);

// ===== 新库 =====
pm1.pinMode(M5PM1_GPIO_NUM_0, OUTPUT);
pm1.digitalWrite(M5PM1_GPIO_NUM_0, HIGH);

// 或使用高级配置
pm1.gpioSet(M5PM1_GPIO_NUM_0, M5PM1_GPIO_MODE_OUTPUT,
            HIGH, M5PM1_GPIO_PULL_NONE, M5PM1_GPIO_DRIVE_PUSHPULL);
```

---

## ADC 功能

| 旧 API | 新 API |
|--------|--------|
| `pm1_adc_read(channel, &value)` | `pm1.analogRead(channel, &value)` |
| *(无)* | `pm1.isAdcBusy(&busy)` |
| *(无)* | `pm1.disableAdc()` |
| *(无)* | `pm1.readTemperature(&temp)` |

### ADC 通道枚举

| 旧枚举 | 新枚举 |
|--------|--------|
| `PM1_ADC_CHANNEL_1` | `M5PM1_ADC_CH_1` |
| `PM1_ADC_CHANNEL_2` | `M5PM1_ADC_CH_2` |
| `PM1_ADC_CHANNEL_TEMP` | `M5PM1_ADC_CH_TEMP` |

### 示例对照

```cpp
// ===== 旧库 =====
uint16_t adc_value;
pm1_adc_read(PM1_ADC_CHANNEL_1, &adc_value);

// ===== 新库 =====
uint16_t adc_value;
pm1.analogRead(M5PM1_ADC_CH_1, &adc_value);

// 新功能: 读取温度
uint16_t temperature;
pm1.readTemperature(&temperature);
```

---

## PWM 功能

| 旧 API | 新 API |
|--------|--------|
| `pm1_pwm_set(ch, ctrl, pol, freq, duty)` | `pm1.setPwmConfig(ch, enable, pol, freq, duty12)` |
| *(无)* | `pm1.setPwmFrequency(freq)` |
| *(无)* | `pm1.getPwmFrequency(&freq)` |
| *(无)* | `pm1.setPwmDuty(ch, duty8, pol, enable)` |
| *(无)* | `pm1.setPwmDuty12bit(ch, duty12, pol, enable)` |
| *(无)* | `pm1.analogWrite(ch, value)` |

### PWM 通道枚举

| 旧枚举 | 新枚举 |
|--------|--------|
| `PM1_PWM_CHANNEL_0` | `M5PM1_PWM_CH_0` |
| `PM1_PWM_CHANNEL_1` | `M5PM1_PWM_CH_1` |

### 示例对照

```cpp
// ===== 旧库 =====
pm1_pwm_set(PM1_PWM_CHANNEL_0, PM1_PWM_CTRL_ENABLE,
            PM1_PWM_POLARITY_NORMAL, 1000, 2048);

// ===== 新库 =====
// 方式1: 一次性配置
pm1.setPwmConfig(M5PM1_PWM_CH_0, true, false, 1000, 2048);

// 方式2: 分步配置
pm1.setPwmFrequency(1000);
pm1.setPwmDuty12bit(M5PM1_PWM_CH_0, 2048, false, true);

// 方式3: Arduino 风格 (8位精度)
pm1.analogWrite(M5PM1_PWM_CH_0, 128);  // 50% 占空比
```

---

## 电压读取

| 旧 API | 新 API |
|--------|--------|
| `pm1_vref_read(&mv)` | `pm1.readVref(&mv)` 或 `pm1.getRefVoltage(&mv)` |
| `pm1_vbat_read(&mv)` | `pm1.readVbat(&mv)` |
| `pm1_vin_read(&mv)` | `pm1.readVin(&mv)` |
| `pm1_5vinout_read(&mv)` | `pm1.read5VInOut(&mv)` |

### 示例对照

```cpp
// ===== 旧库 =====
uint16_t vbat, vin, vinout;
pm1_vbat_read(&vbat);
pm1_vin_read(&vin);
pm1_5vinout_read(&vinout);

// ===== 新库 =====
uint16_t vbat, vin, vinout;
pm1.readVbat(&vbat);
pm1.readVin(&vin);
pm1.read5VInOut(&vinout);
```

---

## 电源管理

### 电源源读取

| 旧 API | 新 API |
|--------|--------|
| `pm1_pwr_src_read(&src)` | `pm1.getPowerSource(&src)` |

### 电源源枚举

| 旧枚举 | 新枚举 |
|--------|--------|
| `PM1_PWR_SRC_5VIN` | `M5PM1_PWR_SRC_5VIN` |
| `PM1_PWR_SRC_5VINOUT` | `M5PM1_PWR_SRC_5VINOUT` |
| `PM1_PWR_SRC_BAT` | `M5PM1_PWR_SRC_BAT` |
| `PM1_PWR_SRC_UNKNOWN` | `M5PM1_PWR_SRC_UNKNOWN` |

### 唤醒源

| 旧 API | 新 API |
|--------|--------|
| `pm1_wake_src_read(&src, clean_type)` | `pm1.getWakeSource(&src, clearAfterRead)` |
| *(无)* | `pm1.clearWakeSource(mask)` |

### 电源配置

| 旧 API | 新 API |
|--------|--------|
| `pm1_pwr_set_cfg(mask, value, &final)` | `pm1.setPowerConfig(mask, value)` |
| `pm1_pwr_clear_cfg(cfg, &final)` | `pm1.clearPowerConfig(mask)` |
| `pm1_pwr_get_cfg(&cfg)` | `pm1.getPowerConfig(&cfg)` |
| `pm1_pwr_set_chg_en(enable)` | `pm1.setChargeEnable(enable)` |
| `pm1_pwr_set_dcdc_en(enable)` | `pm1.setDcdcEnable(enable)` |
| `pm1_pwr_set_ldo_en(enable)` | `pm1.setLdoEnable(enable)` |
| `pm1_pwr_set_5v_inout(enable)` | `pm1.set5VInOutEnable(enable)` |
| `pm1_pwr_set_led_control(enable)` | `pm1.setLedControlEnable(enable)` |

### 电池低压保护

| 旧 API | 新 API |
|--------|--------|
| `pm1_batt_set_lvp(mv)` | `pm1.setBatteryLvp(mv)` |

### 示例对照

```cpp
// ===== 旧库 =====
pm1_pwr_src_t pwr_src;
pm1_pwr_src_read(&pwr_src);
pm1_pwr_set_chg_en(1);
pm1_pwr_set_dcdc_en(1);

// ===== 新库 =====
m5pm1_pwr_src_t pwr_src;
pm1.getPowerSource(&pwr_src);
pm1.setChargeEnable(true);
pm1.setDcdcEnable(true);
```

---

## 看门狗

| 旧 API | 新 API |
|--------|--------|
| `pm1_wdt_set(ctrl, timeout)` | `pm1.wdtSet(timeout)` |
| `pm1_wdt_feed(key)` | `pm1.wdtFeed()` |
| `pm1_wdt_get_wdt_cnt(&cnt)` | `pm1.wdtGetCount(&cnt)` |

### 示例对照

```cpp
// ===== 旧库 =====
pm1_wdt_set(PM1_WDT_CTRL_ENABLE, 30);  // 30秒超时
pm1_wdt_feed(0xA5);                     // 喂狗

// ===== 新库 =====
pm1.wdtSet(30);   // 30秒超时, 0 禁用
pm1.wdtFeed();    // 喂狗 (密钥自动处理)
```

---

## 定时器

| 旧 API | 新 API |
|--------|--------|
| `pm1_tim_set(ctrl, action, count)` | `pm1.timerSet(seconds, action, autoRearm)` |
| `pm1_tim_clear(key)` | `pm1.timerClear()` |

### 定时器动作枚举

| 旧枚举 | 新枚举 |
|--------|--------|
| `PM1_TIM_ACTION_000` | `M5PM1_TIM_ACTION_STOP` |
| `PM1_TIM_ACTION_001` | `M5PM1_TIM_ACTION_FLAG` |
| `PM1_TIM_ACTION_010` | `M5PM1_TIM_ACTION_REBOOT` |
| `PM1_TIM_ACTION_011` | `M5PM1_TIM_ACTION_POWERON` |
| `PM1_TIM_ACTION_100` | `M5PM1_TIM_ACTION_POWEROFF` |

### 示例对照

```cpp
// ===== 旧库 =====
pm1_tim_set(PM1_ADDR_TIM_ENABLE, PM1_TIM_ACTION_010, 3600);  // 1小时后重启
pm1_tim_clear(0xA5);

// ===== 新库 =====
pm1.timerSet(3600, M5PM1_TIM_ACTION_REBOOT, false);  // 1小时后重启
pm1.timerClear();  // 密钥自动处理
```

---

## 按钮功能

| 旧 API | 新 API |
|--------|--------|
| `pm1_btn_set_cfg(type, delay)` | `pm1.btnSetConfig(type, delay)` |
| `pm1_btn_get_status(&status, PM1_BTN_READ_STATUS)` | `pm1.btnGetState(&pressed)` |
| `pm1_btn_get_status(&status, PM1_BTN_READ_FLAG)` | `pm1.btnGetFlag(&wasPressed)` |
| `pm1_single_reset_dis_set(enable)` | `pm1.setSingleResetDisable(disable)` |
| `pm1_single_reset_dis_get(&enable)` | `pm1.getSingleResetDisable(&disabled)` |
| `pm1_double_poweroff_dis_set(enable)` | `pm1.setDoubleOffDisable(disable)` |
| `pm1_double_poweroff_dis_get(&enable)` | `pm1.getDoubleOffDisable(&disabled)` |

### 按钮类型枚举

| 旧枚举 | 新枚举 |
|--------|--------|
| `PM1_ADDR_BTN_TYPE_CLICK` | `M5PM1_BTN_TYPE_CLICK` |
| `PM1_ADDR_BTN_TYPE_DOUBLE_CLICK` | `M5PM1_BTN_TYPE_DOUBLE` |
| `PM1_ADDR_BTN_TYPE_LONG_PRESS` | `M5PM1_BTN_TYPE_LONG` |

### 按钮延时枚举

| 旧枚举 | 新枚举 |
|--------|--------|
| `PM1_ADDR_BTN_CLICK_DELAY_125MS` | `M5PM1_BTN_DELAY_125MS` |
| `PM1_ADDR_BTN_CLICK_DELAY_250MS` | `M5PM1_BTN_DELAY_250MS` |
| `PM1_ADDR_BTN_CLICK_DELAY_500MS` | `M5PM1_BTN_DELAY_500MS` |
| `PM1_ADDR_BTN_CLICK_DELAY_1000MS` | `M5PM1_BTN_DELAY_1000MS` |

### 示例对照

```cpp
// ===== 旧库 =====
pm1_btn_set_cfg(PM1_ADDR_BTN_TYPE_CLICK, PM1_ADDR_BTN_CLICK_DELAY_250MS);
pm1_btn_status_t status;
pm1_btn_get_status(&status, PM1_BTN_READ_STATUS);

// ===== 新库 =====
pm1.btnSetConfig(M5PM1_BTN_TYPE_CLICK, M5PM1_BTN_DELAY_250MS);
bool pressed;
pm1.btnGetState(&pressed);
```

---

## 中断功能

### GPIO 中断

| 旧 API | 新 API |
|--------|--------|
| `pm1_irq_get_status(&gpio, clean_type)` | `pm1.irqGetGpioStatus(&status, clearAfterRead)` |
| `pm1_irq_clear_gpio_flag(gpio)` | `pm1.irqClearGpio(mask)` |
| `pm1_irq_set_gpio_mask(pin, ctrl)` | `pm1.irqSetGpioMask(pin, mask)` |
| `pm1_irq_get_gpio_mask(pin, &ctrl)` | `pm1.irqGetGpioMask(pin, &mask)` |
| `pm1_irq_set_gpio_mask_all(value)` | `pm1.irqSetGpioMaskAll(mask)` |
| `pm1_irq_get_gpio_mask_all(&value)` | `pm1.irqGetGpioMaskAll(&mask)` |

### 系统中断

| 旧 API | 新 API |
|--------|--------|
| `pm1_irq_get_sys_status(&sys, clean_type)` | `pm1.irqGetSysStatus(&status, clearAfterRead)` |
| `pm1_irq_clear_sys_status(sys)` | `pm1.irqClearSys(mask)` |
| `pm1_irq_set_sys_mask(sys, ctrl)` | `pm1.irqSetSysMask(event, mask)` |
| `pm1_irq_get_sys_mask(sys, &ctrl)` | `pm1.irqGetSysMask(event, &mask)` |
| `pm1_irq_set_sys_mask_all(value)` | `pm1.irqSetSysMaskAll(mask)` |
| `pm1_irq_get_sys_mask_all(&value)` | `pm1.irqGetSysMaskAll(&mask)` |

### 按钮中断

| 旧 API | 新 API |
|--------|--------|
| `pm1_irq_get_btn_status(&btn, clean_type)` | `pm1.irqGetBtnStatus(&status, clearAfterRead)` |
| `pm1_irq_clear_btn_status(btn)` | `pm1.irqClearBtn(mask)` |
| `pm1_irq_set_btn_mask(btn, ctrl)` | `pm1.irqSetBtnMask(type, mask)` |
| `pm1_irq_get_btn_mask(btn, &ctrl)` | `pm1.irqGetBtnMask(type, &mask)` |
| `pm1_irq_set_btn_mask_all(value)` | `pm1.irqSetBtnMaskAll(mask)` |
| `pm1_irq_get_btn_mask_all(&value)` | `pm1.irqGetBtnMaskAll(&mask)` |

### clearAfterRead 参数说明

| 值 | 说明 |
|----|------|
| `0` | 不清除状态 |
| `1` | 仅清除已触发的位 |
| `2` | 清除所有状态位 |

---

## NeoPixel LED

| 旧 API | 新 API |
|--------|--------|
| `pm1_neo_set_cfg(num, data, refresh)` | `pm1.setLeds(colors, arraySize, count, autoRefresh)` |
| `pm1_neo_refresh()` | `pm1.refreshLeds()` |
| *(无)* | `pm1.setLedCount(count)` |
| *(无)* | `pm1.setLedColor(index, r, g, b)` |
| *(无)* | `pm1.disableLeds()` |

### 示例对照

```cpp
// ===== 旧库 =====
uint16_t neo_data[3] = {0xF800, 0x07E0, 0x001F};  // RGB565 格式
pm1_neo_set_cfg(3, neo_data, 1);

// ===== 新库 =====
// 方式1: 使用 RGB 结构
m5pm1_rgb_t colors[3] = {{255,0,0}, {0,255,0}, {0,0,255}};
pm1.setLeds(colors, 3, 3, true);

// 方式2: 逐个设置
pm1.setLedCount(3);
pm1.setLedColor(0, 255, 0, 0);    // 红色
pm1.setLedColor(1, 0, 255, 0);    // 绿色
pm1.setLedColor(2, 0, 0, 255);    // 蓝色
pm1.refreshLeds();
```

---

## RTC RAM

| 旧 API | 新 API |
|--------|--------|
| `pm1_rtc_ram_write(addr, len, data)` | `pm1.writeRtcRAM(offset, data, len)` |
| `pm1_rtc_ram_read(addr, len, data)` | `pm1.readRtcRAM(offset, data, len)` |

### 示例对照

```cpp
// ===== 旧库 =====
uint8_t write_data[4] = {0x12, 0x34, 0x56, 0x78};
pm1_rtc_ram_write(0, 4, write_data);

uint8_t read_data[4];
pm1_rtc_ram_read(0, 4, read_data);

// ===== 新库 =====
uint8_t write_data[4] = {0x12, 0x34, 0x56, 0x78};
pm1.writeRtcRAM(0, write_data, 4);

uint8_t read_data[4];
pm1.readRtcRAM(0, read_data, 4);
```

---

## 系统命令

| 旧 API | 新 API |
|--------|--------|
| `pm1_sys_cmd(PM1_SYS_CMD_SHUTDOWN)` | `pm1.shutdown()` 或 `pm1.sysCmd(M5PM1_SYS_CMD_OFF)` |
| `pm1_sys_cmd(PM1_SYS_CMD_REBOOT)` | `pm1.reboot()` 或 `pm1.sysCmd(M5PM1_SYS_CMD_RESET)` |
| `pm1_sys_cmd(PM1_SYS_CMD_JTAG)` | `pm1.enterDownloadMode()` 或 `pm1.sysCmd(M5PM1_SYS_CMD_DL)` |
| `pm1_download_enable_set(enable)` | `pm1.setDownloadLock(!enable)` |
| `pm1_download_enable_get(&enable)` | `pm1.getDownloadLock(&lock)` |

### 系统命令枚举

| 旧枚举 | 新枚举 |
|--------|--------|
| `PM1_SYS_CMD_NULL` | `M5PM1_SYS_CMD_NONE` |
| `PM1_SYS_CMD_SHUTDOWN` | `M5PM1_SYS_CMD_OFF` |
| `PM1_SYS_CMD_REBOOT` | `M5PM1_SYS_CMD_RESET` |
| `PM1_SYS_CMD_JTAG` | `M5PM1_SYS_CMD_DL` |

---

## I2C 配置

| 旧 API | 新 API |
|--------|--------|
| `pm1_set_clk_speed(speed)` | `pm1.switchI2cSpeed(speed)` |
| `pm1_set_i2c_sleep_time(time)` | `pm1.setI2cSleepTime(seconds)` |
| `pm1_i2c_try_wake(type, param)` | `pm1.sendWakeSignal()` |
| `pm1_i2c_set_auto_wake_enable(enable)` | `pm1.setAutoWakeEnable(enable)` |
| *(无)* | `pm1.setI2cConfig(sleepTime, speed)` |
| *(无)* | `pm1.getI2cSpeed(&speed)` |
| *(无)* | `pm1.getI2cSleepTime(&seconds)` |

### I2C 速度枚举

| 旧枚举 | 新枚举 |
|--------|--------|
| `PM1_CLK_SPEED_100KHZ` | `M5PM1_I2C_SPEED_100K` |
| `PM1_CLK_SPEED_400KHZ` | `M5PM1_I2C_SPEED_400K` |

---

## 枚举类型对照表

### GPIO 相关

| 旧枚举类型 | 新枚举类型 |
|------------|------------|
| `pm1_gpio_num_t` | `m5pm1_gpio_num_t` |
| `pm1_gpio_mode_t` | `m5pm1_gpio_mode_t` |
| `pm1_gpio_func_t` | `m5pm1_gpio_func_t` |
| `pm1_gpio_pupd_t` / `pm1_gpio_pull_t` | `m5pm1_gpio_pull_t` |
| `pm1_gpio_drv_t` | `m5pm1_gpio_drive_t` |
| `pm1_gpio_wake_edge_t` | `m5pm1_gpio_wake_edge_t` |

### ADC/PWM 相关

| 旧枚举类型 | 新枚举类型 |
|------------|------------|
| `pm1_adc_channel_t` | `m5pm1_adc_channel_t` |
| `pm1_pwm_channel_t` | `m5pm1_pwm_channel_t` |

### 电源相关

| 旧枚举类型 | 新枚举类型 |
|------------|------------|
| `pm1_pwr_src_t` | `m5pm1_pwr_src_t` |
| `pm1_wake_src_t` | `m5pm1_wake_src_t` |
| `pm1_sys_cmd_t` | `m5pm1_sys_cmd_t` |
| `pm1_tim_action_t` | `m5pm1_tim_action_t` |

### 按钮相关

| 旧枚举类型 | 新枚举类型 |
|------------|------------|
| `pm1_btn_type_t` | `m5pm1_btn_type_t` |
| `pm1_btn_delay_t` | `m5pm1_btn_delay_t` |
| `pm1_irq_btn_t` | `m5pm1_btn_irq_t` |

### I2C 相关

| 旧枚举类型 | 新枚举类型 |
|------------|------------|
| `pm1_clk_speed_t` | `m5pm1_i2c_speed_t` |

---

## 新库独有功能

新库 `M5PM1` 提供了一些旧库没有的功能：

### 1. 状态快照

```cpp
pm1.setAutoSnapshot(true);    // 启用自动快照
pm1.updateSnapshot();          // 手动更新快照
m5pm1_snapshot_verify_t result = pm1.verifySnapshot();  // 验证快照一致性
```

### 2. 配置验证

```cpp
m5pm1_validation_t v = pm1.validateConfig(pin, M5PM1_CONFIG_PWM, true);
if (!v.valid) {
    Serial.printf("Config error: %s\r\n", v.error_msg);
}
```

### 3. 日志级别控制

```cpp
M5PM1::setLogLevel(M5PM1_LOG_LEVEL_DEBUG);
m5pm1_log_level_t level = M5PM1::getLogLevel();
```

### 4. 缓存状态查询

```cpp
uint16_t freq;
pm1.getCachedPwmFrequency(&freq);

uint16_t duty;
bool enable, polarity;
pm1.getCachedPwmState(M5PM1_PWM_CH_0, &duty, &enable, &polarity);
```

### 5. 引脚状态获取

```cpp
m5pm1_pin_status_t status;
pm1.getPinStatus(M5PM1_GPIO_NUM_0, &status);
pm1.dumpPinStatus();  // 调试用，打印所有引脚状态
```

---

## 完整迁移示例

### 旧库代码

```cpp
#include "m5_stamp_pm1.h"

void setup() {
    Serial.begin(115200);
    Wire.begin(47, 48, 400000);

    // 初始化
    if (pm1_init(&Wire, 47, 48, 400000) != ESP_OK) {
        Serial.println("PM1 init failed");
        return;
    }

    // 读取设备信息
    uint8_t device_id, sw_ver;
    pm1_get_device_id(&device_id);
    pm1_get_sw_version(&sw_ver);
    Serial.printf("Device ID: 0x%02X, FW: %d\n", device_id, sw_ver);

    // 配置 GPIO
    pm1_gpio_set_func(PM1_GPIO_NUM_0, PM1_GPIO_FUNC_GPIO);
    pm1_gpio_set_mode(PM1_GPIO_NUM_0, PM1_GPIO_MODE_OUTPUT);
    pm1_gpio_set_state(PM1_GPIO_NUM_0, PM1_GPIO_OUTPUT_HIGH);

    // 配置 PWM
    pm1_gpio_set_func(PM1_GPIO_NUM_3, PM1_GPIO_FUNC_OTHER);
    pm1_pwm_set(PM1_PWM_CHANNEL_0, PM1_PWM_CTRL_ENABLE,
                PM1_PWM_POLARITY_NORMAL, 1000, 2048);

    // 读取电压
    uint16_t vbat, vin;
    pm1_vbat_read(&vbat);
    pm1_vin_read(&vin);
    Serial.printf("VBAT: %dmV, VIN: %dmV\n", vbat, vin);

    // 配置看门狗
    pm1_wdt_set(PM1_WDT_CTRL_ENABLE, 30);
}

void loop() {
    pm1_wdt_feed(0xA5);
    delay(10000);
}
```

### 新库代码

```cpp
#include "M5PM1.h"

M5PM1 pm1;

void setup() {
    Serial.begin(115200);

    // 初始化 (方式1: 让库处理 Wire)
    if (pm1.begin(&Wire, 0x6E, 47, 48, 400000) != M5PM1_OK) {
        Serial.println("PM1 init failed");
        return;
    }

    // 读取设备信息
    uint8_t device_id, sw_ver;
    pm1.getDeviceId(&device_id);
    pm1.getSwVersion(&sw_ver);
    Serial.printf("Device ID: 0x%02X, FW: %d\r\n", device_id, sw_ver);

    // 配置 GPIO (Arduino 风格)
    pm1.pinMode(M5PM1_GPIO_NUM_0, OUTPUT);
    pm1.digitalWrite(M5PM1_GPIO_NUM_0, HIGH);

    // 配置 PWM
    pm1.gpioSetFunc(M5PM1_GPIO_NUM_3, M5PM1_GPIO_FUNC_OTHER);
    pm1.setPwmConfig(M5PM1_PWM_CH_0, true, false, 1000, 2048);

    // 读取电压
    uint16_t vbat, vin;
    pm1.readVbat(&vbat);
    pm1.readVin(&vin);
    Serial.printf("VBAT: %dmV, VIN: %dmV\r\n", vbat, vin);

    // 配置看门狗
    pm1.wdtSet(30);
}

void loop() {
    pm1.wdtFeed();
    delay(10000);
}
```

---

## 常见问题

### Q1: 新库是否兼容旧库的 I2C 地址？

是的，默认 I2C 地址都是 `0x6E`。

### Q2: 如何处理 `esp_err_t` 和 `m5pm1_err_t` 的差异？

两者都是整数类型，`M5PM1_OK (0)` 等同于 `ESP_OK (0)`。可以直接用布尔判断：

```cpp
if (pm1.readVbat(&vbat) != M5PM1_OK) {
    // 处理错误
}
```

### Q3: 新库的 Arduino 风格 GPIO 函数没有返回值怎么办？

使用带 `WithRes` 后缀的版本：

```cpp
m5pm1_err_t err;
pm1.digitalWriteWithRes(0, HIGH, &err);
if (err != M5PM1_OK) {
    // 处理错误
}
```

### Q4: 旧库的 AW8737A 功能在新库中如何使用？

```cpp
// 旧库
pm1_aw8737a_pulse_set(PM1_GPIO_NUM_0, PM1_AW8737A_PULSE_NUM_2, PM1_AW8737A_REFRESH_NOW);

// 新库
pm1.setAw8737aPulse(M5PM1_GPIO_NUM_0, M5PM1_AW8737A_PULSE_2, M5PM1_AW8737A_REFRESH_NOW);
```

---

## 版本信息

- 旧库版本: `PM1_DEV_VERSION "0.0.3"`
- 新库版本: 请参考 `library.json` 或 `library.properties`
- 本文档最后更新: 2025-01-15
