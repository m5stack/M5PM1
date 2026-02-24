/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __M5PM1_I2C_COMPAT_H__
#define __M5PM1_I2C_COMPAT_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef ARDUINO

#include "Wire.h"

// ============================
// Arduino I2C 功能
// Arduino I2C Functions
// ============================

#ifndef M5PM1_I2C_READ_BYTE
static inline bool M5PM1_I2C_READ_BYTE(TwoWire *wire, uint8_t addr, uint8_t reg, uint8_t *data)
{
    wire->beginTransmission(addr);
    wire->write(reg);
    if (wire->endTransmission(false) != 0) {
        return false;
    }
    if (wire->requestFrom(addr, (uint8_t)1) != 1) {
        return false;
    }
    *data = wire->read();
    return true;
}
#endif

#ifndef M5PM1_I2C_READ_BYTES
static inline bool M5PM1_I2C_READ_BYTES(TwoWire *wire, uint8_t addr, uint8_t start_reg, size_t len, uint8_t *data)
{
    wire->beginTransmission(addr);
    wire->write(start_reg);
    if (wire->endTransmission(false) != 0) {
        return false;
    }
    if (wire->requestFrom(addr, (uint8_t)len) != len) {
        return false;
    }
    for (size_t i = 0; i < len; i++) {
        data[i] = wire->read();
    }
    return true;
}
#endif

#ifndef M5PM1_I2C_READ_REG16
static inline bool M5PM1_I2C_READ_REG16(TwoWire *wire, uint8_t addr, uint8_t reg, uint16_t *data)
{
    uint8_t buf[2];
    if (!M5PM1_I2C_READ_BYTES(wire, addr, reg, 2, buf)) {
        return false;
    }
    // 小端模式：低字节在前
    // Little-endian: low byte first
    *data = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    return true;
}
#endif

#ifndef M5PM1_I2C_WRITE_BYTE
static inline bool M5PM1_I2C_WRITE_BYTE(TwoWire *wire, uint8_t addr, uint8_t reg, uint8_t data)
{
    wire->beginTransmission(addr);
    wire->write(reg);
    wire->write(data);
    if (wire->endTransmission() != 0) {
        return false;
    }
    return true;
}
#endif

#ifndef M5PM1_I2C_WRITE_BYTES
static inline bool M5PM1_I2C_WRITE_BYTES(TwoWire *wire, uint8_t addr, uint8_t start_reg, size_t len,
                                         const uint8_t *data)
{
    wire->beginTransmission(addr);
    wire->write(start_reg);
    for (size_t i = 0; i < len; i++) {
        wire->write(data[i]);
    }
    if (wire->endTransmission() != 0) {
        return false;
    }
    return true;
}
#endif

#ifndef M5PM1_I2C_WRITE_REG16
static inline bool M5PM1_I2C_WRITE_REG16(TwoWire *wire, uint8_t addr, uint8_t reg, uint16_t data)
{
    uint8_t buf[2];
    // 小端模式：低字节在前
    // Little-endian: low byte first
    buf[0] = (uint8_t)(data & 0xFF);
    buf[1] = (uint8_t)((data >> 8) & 0xFF);
    return M5PM1_I2C_WRITE_BYTES(wire, addr, reg, 2, buf);
}
#endif

// PM1睡眠模式唤醒信号
// Wake signal for PM1 sleep mode
#ifndef M5PM1_I2C_SEND_WAKE
static inline void M5PM1_I2C_SEND_WAKE(TwoWire *wire, uint8_t addr)
{
    // Send START signal to generate SDA falling edge for PM1 wake
    // PM1 uses SDA pin (PB4) falling edge to trigger EXTI4 interrupt for wakeup
    wire->beginTransmission(addr);
    wire->endTransmission();  // Send START with STOP
}
#endif

#else  // ESP-IDF

#include <esp_err.h>
#include <esp_idf_version.h>

// ============================
// I2C 驱动检测
// I2C Driver Detection
// ============================

// 检测 i2c_bus 是否可用
// Detect if i2c_bus is available
//
// ESP-IDF < 5.3.0
//   未启用 BACKWARD_CONFIG
//     → 不支持 i2c_bus，使用传统 driver/i2c.h Legacy API
//   启用  BACKWARD_CONFIG
//     → i2c_bus.h 内部回退到 driver/i2c.h，可安全使用
//
// ESP-IDF >= 5.3.0
//   启用  BACKWARD_CONFIG
//     → i2c_bus.h 内部使用 driver/i2c.h，无冲突风险
//   未启用 BACKWARD_CONFIG
//     driver/i2c.h 已被其他组件提前包含（_DRIVER_I2C_H_ 已定义）
//       → i2c_bus.h 会自定义 i2c_config_t，与已定义的版本冲突 → 禁用
//     driver/i2c.h 尚未被包含（_DRIVER_I2C_H_ 未定义）
//       → 无冲突风险，按默认配置启用 i2c_bus
//
// Detection logic:
//   ESP-IDF < 5.3.0:
//     Without BACKWARD_CONFIG: i2c_bus not supported; use legacy driver/i2c.h API.
//     With    BACKWARD_CONFIG: i2c_bus.h falls back to driver/i2c.h internally, safe to use.
//   ESP-IDF >= 5.3.0:
//     With    BACKWARD_CONFIG: i2c_bus.h uses driver/i2c.h internally, always conflict-free.
//     Without BACKWARD_CONFIG:
//       _DRIVER_I2C_H_ defined   (driver/i2c.h already included by another component)
//         → i2c_bus.h would define its own i2c_config_t, conflicting with the existing one → disabled.
//       _DRIVER_I2C_H_ not defined (driver/i2c.h not yet included)
//         → no conflict risk, enable i2c_bus with default config.
//
// Note: _DRIVER_I2C_H_ is the include guard of driver/i2c.h (ESP-IDF legacy I2C header).
//       Checking it at preprocessor time reflects whether driver/i2c.h was included
//       BEFORE this header. Inclusion after this header cannot be detected here;
//       in that case the user is responsible for ensuring no conflict (or enabling BACKWARD_CONFIG).
#if __has_include(<i2c_bus.h>)
  #if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 3, 0)
    #if defined(CONFIG_I2C_BUS_BACKWARD_CONFIG)
      #define M5PM1_HAS_I2C_BUS 1  // IDF < 5.3.0 + BACKWARD_CONFIG：可用 / available
    #else
      #define M5PM1_HAS_I2C_BUS 0  // IDF < 5.3.0：默认 Legacy API / legacy API by default
    #endif
  #else
    // IDF >= 5.3.0
    #if defined(CONFIG_I2C_BUS_BACKWARD_CONFIG)
      #define M5PM1_HAS_I2C_BUS 1  // BACKWARD_CONFIG：i2c_bus.h 使用 driver/i2c.h，无冲突 / no conflict
    #elif defined(_DRIVER_I2C_H_)
      #define M5PM1_HAS_I2C_BUS 0  // driver/i2c.h 已提前包含 + 无 BACKWARD_CONFIG → 冲突风险，禁用
                                    // driver/i2c.h already included + no BACKWARD_CONFIG → conflict risk, disabled
    #else
      #define M5PM1_HAS_I2C_BUS 1  // driver/i2c.h 尚未包含，无冲突风险 / driver/i2c.h not yet included, no conflict
    #endif
  #endif
#else
  #define M5PM1_HAS_I2C_BUS 0
#endif

// 选择 I2C 驱动头文件
// Select I2C driver header
//
// ESP-IDF < 5.3.0：使用传统 Legacy I2C API（driver/i2c.h），与 i2c_bus 无关
// ESP-IDF < 5.3.0: use legacy I2C API (driver/i2c.h), independent of i2c_bus.
//
// ESP-IDF >= 5.3.0：优先使用新版 i2c_master 驱动
// ESP-IDF >= 5.3.0: prefer the new i2c_master driver.
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 3, 0)
  #include <driver/i2c.h>
#elif __has_include(<driver/i2c_master.h>)
  #include <driver/i2c_master.h>
#else
  #include <driver/i2c.h>
#endif

#if M5PM1_HAS_I2C_BUS
  #include <i2c_bus.h>
#else
  // i2c_bus 桩类型
  // i2c_bus stub types
  typedef void *i2c_bus_handle_t;
  typedef void *i2c_bus_device_handle_t;
#endif

#ifdef __cplusplus
extern "C" {
#endif

// ============================
// I2C 驱动类型选择
// I2C Driver Type Selection
// ============================
typedef enum {
    M5PM1_I2C_DRIVER_NONE = 0,      // 未初始化 / Not initialized
    M5PM1_I2C_DRIVER_SELF_CREATED,  // 使用 i2c_port_t 自创建 / Self-created using i2c_port_t
    M5PM1_I2C_DRIVER_MASTER,        // ESP-IDF 原生 i2c_master 驱动 / ESP-IDF native i2c_master driver
#if M5PM1_HAS_I2C_BUS
    M5PM1_I2C_DRIVER_BUS            // esp-idf-lib i2c_bus 组件 / esp-idf-lib i2c_bus component
#endif
} m5pm1_i2c_driver_t;

// ============================
// ESP-IDF I2C 函数 (i2c_bus)
// ESP-IDF I2C Functions (i2c_bus)
// ============================

#if M5PM1_HAS_I2C_BUS

#ifndef M5PM1_I2C_READ_BYTE
static inline esp_err_t M5PM1_I2C_READ_BYTE(i2c_bus_device_handle_t dev, uint8_t reg, uint8_t *data)
{
    return i2c_bus_read_byte(dev, reg, data);
}
#endif

#ifndef M5PM1_I2C_READ_BYTES
static inline esp_err_t M5PM1_I2C_READ_BYTES(i2c_bus_device_handle_t dev, uint8_t start_reg, size_t len, uint8_t *data)
{
    return i2c_bus_read_bytes(dev, start_reg, len, data);
}
#endif

#ifndef M5PM1_I2C_READ_REG16
static inline esp_err_t M5PM1_I2C_READ_REG16(i2c_bus_device_handle_t dev, uint8_t reg, uint16_t *data)
{
    uint8_t buf[2];
    esp_err_t ret = i2c_bus_read_bytes(dev, reg, 2, buf);
    if (ret == ESP_OK) {
        // 小端模式：低字节在前
        // Little-endian: low byte first
        *data = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    }
    return ret;
}
#endif

#ifndef M5PM1_I2C_WRITE_BYTE
static inline esp_err_t M5PM1_I2C_WRITE_BYTE(i2c_bus_device_handle_t dev, uint8_t reg, uint8_t data)
{
    return i2c_bus_write_byte(dev, reg, data);
}
#endif

#ifndef M5PM1_I2C_WRITE_BYTES
static inline esp_err_t M5PM1_I2C_WRITE_BYTES(i2c_bus_device_handle_t dev, uint8_t start_reg, size_t len,
                                              const uint8_t *data)
{
    return i2c_bus_write_bytes(dev, start_reg, len, (uint8_t *)data);
}
#endif

#ifndef M5PM1_I2C_WRITE_REG16
static inline esp_err_t M5PM1_I2C_WRITE_REG16(i2c_bus_device_handle_t dev, uint8_t reg, uint16_t data)
{
    uint8_t buf[2];
    // 小端模式：低字节在前
    // Little-endian: low byte first
    buf[0] = (uint8_t)(data & 0xFF);
    buf[1] = (uint8_t)((data >> 8) & 0xFF);
    return i2c_bus_write_bytes(dev, reg, 2, buf);
}
#endif

// PM1睡眠模式唤醒信号 (i2c_bus)
// Wake signal for PM1 sleep mode (i2c_bus)
#ifndef M5PM1_I2C_SEND_WAKE
static inline esp_err_t M5PM1_I2C_SEND_WAKE(i2c_bus_device_handle_t dev, uint8_t reg)
{
    uint8_t dummy;
    return i2c_bus_read_byte(dev, reg, &dummy);
}
#endif

#else  // !M5PM1_HAS_I2C_BUS

// i2c_bus API 不可用时的编译期报错，可能原因：
//   A) ESP-IDF < 5.3.0 且未启用 CONFIG_I2C_BUS_BACKWARD_CONFIG
//   B) ESP-IDF >= 5.3.0 且未启用 CONFIG_I2C_BUS_BACKWARD_CONFIG，
//      且 driver/i2c.h 已被其他组件在本头文件之前包含（检测到 _DRIVER_I2C_H_ 已定义），
//      i2c_bus.h 自定义的 i2c_config_t 与其冲突。
//
// Compile-time error when i2c_bus API is unavailable. Possible reasons:
//   A) ESP-IDF < 5.3.0 and CONFIG_I2C_BUS_BACKWARD_CONFIG is not enabled.
//   B) ESP-IDF >= 5.3.0, CONFIG_I2C_BUS_BACKWARD_CONFIG is not enabled, and
//      driver/i2c.h was already included by another component before this header
//      (_DRIVER_I2C_H_ was defined), causing i2c_config_t conflict with i2c_bus.h.
#if defined(__GNUC__) || defined(__clang__)
void __attribute__((error(
    "M5PM1: i2c_bus API is unavailable. "
    "Reason A (ESP-IDF < 5.3.0): i2c_bus requires CONFIG_I2C_BUS_BACKWARD_CONFIG on this IDF version. "
    "Reason B (ESP-IDF >= 5.3.0): driver/i2c.h was included before this header and "
    "CONFIG_I2C_BUS_BACKWARD_CONFIG is not enabled, causing i2c_config_t conflict. "
    "Fix 1: enable CONFIG_I2C_BUS_BACKWARD_CONFIG in menuconfig "
    "(Component config -> i2c_bus -> Enable backward compatible config). "
    "Fix 2: ensure no other component includes driver/i2c.h before M5PM1 headers (ESP-IDF >= 5.3.0 only). "
    "Fix 3: use M5PM1_I2C_DRIVER_MASTER or M5PM1_I2C_DRIVER_SELF_CREATED instead."
))) _m5pm1_i2c_bus_api_unavailable(void);
#endif

#endif  // M5PM1_HAS_I2C_BUS

// ============================
// ESP-IDF I2C 函数 (i2c_master - 原生驱动)
// ESP-IDF I2C Functions (i2c_master - native driver)
// 仅 ESP-IDF >= 5.3.0 支持（driver/i2c_master.h 引入于 5.1，API 稳定于 5.3）
// Available on ESP-IDF >= 5.3.0 only (driver/i2c_master.h introduced in 5.1, stable in 5.3)
// ============================

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)

#ifndef M5PM1_I2C_MASTER_READ_BYTE
static inline esp_err_t M5PM1_I2C_MASTER_READ_BYTE(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data)
{
    return i2c_master_transmit_receive(dev, &reg, 1, data, 1, -1);
}
#endif

#ifndef M5PM1_I2C_MASTER_READ_BYTES
static inline esp_err_t M5PM1_I2C_MASTER_READ_BYTES(i2c_master_dev_handle_t dev, uint8_t start_reg, size_t len,
                                                    uint8_t *data)
{
    return i2c_master_transmit_receive(dev, &start_reg, 1, data, len, -1);
}
#endif

#ifndef M5PM1_I2C_MASTER_READ_REG16
static inline esp_err_t M5PM1_I2C_MASTER_READ_REG16(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t *data)
{
    uint8_t buf[2];
    esp_err_t ret = i2c_master_transmit_receive(dev, &reg, 1, buf, 2, -1);
    if (ret == ESP_OK) {
        // 小端模式：低字节在前
        // Little-endian: low byte first
        *data = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    }
    return ret;
}
#endif

#ifndef M5PM1_I2C_MASTER_WRITE_BYTE
static inline esp_err_t M5PM1_I2C_MASTER_WRITE_BYTE(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(dev, buf, 2, -1);
}
#endif

#ifndef M5PM1_I2C_MASTER_WRITE_BYTES
static inline esp_err_t M5PM1_I2C_MASTER_WRITE_BYTES(i2c_master_dev_handle_t dev, uint8_t start_reg, size_t len,
                                                     const uint8_t *data)
{
    // 需要在数据前添加寄存器地址
    // Need to prepend register address
    uint8_t *buf = (uint8_t *)malloc(len + 1);
    if (buf == NULL) return ESP_ERR_NO_MEM;
    buf[0] = start_reg;
    memcpy(buf + 1, data, len);
    esp_err_t ret = i2c_master_transmit(dev, buf, len + 1, -1);
    free(buf);
    return ret;
}
#endif

#ifndef M5PM1_I2C_MASTER_WRITE_REG16
static inline esp_err_t M5PM1_I2C_MASTER_WRITE_REG16(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t data)
{
    uint8_t buf[3];
    buf[0] = reg;
    // 小端模式：低字节在前
    // Little-endian: low byte first
    buf[1] = (uint8_t)(data & 0xFF);
    buf[2] = (uint8_t)((data >> 8) & 0xFF);
    return i2c_master_transmit(dev, buf, 3, -1);
}
#endif

// PM1睡眠模式唤醒信号 (i2c_master)
// Wake signal for PM1 sleep mode (i2c_master)
#ifndef M5PM1_I2C_MASTER_SEND_WAKE
static inline esp_err_t M5PM1_I2C_MASTER_SEND_WAKE(i2c_master_bus_handle_t bus, uint8_t addr)
{
    return i2c_master_probe(bus, addr, 10);
}
#endif

#endif  // ESP_IDF_VERSION >= 5.3.0

#ifdef __cplusplus
}
#endif

#endif  // ARDUINO

#endif  // __M5PM1_I2C_COMPAT_H__
