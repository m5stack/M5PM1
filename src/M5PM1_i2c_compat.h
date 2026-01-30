/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 *
 * Modified by LOVECHEN: Removed i2c_bus dependency, only use ESP-IDF native i2c_master driver
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
#include <driver/i2c_master.h>  // ESP-IDF native i2c_master driver ONLY

#ifdef __cplusplus
extern "C" {
#endif

// ============================
// I2C 驱动类型选择
// I2C Driver Type Selection
// NOTE: i2c_bus support removed to avoid driver conflict with M5GFX
// ============================
typedef enum {
    M5PM1_I2C_DRIVER_NONE = 0,      // 未初始化 / Not initialized
    M5PM1_I2C_DRIVER_SELF_CREATED,  // 使用 i2c_port_t 自创建 / Self-created using i2c_port_t
    M5PM1_I2C_DRIVER_MASTER         // ESP-IDF 原生 i2c_master 驱动 / ESP-IDF native i2c_master driver
    // M5PM1_I2C_DRIVER_BUS removed - causes conflict with M5GFX's i2c_master driver
} m5pm1_i2c_driver_t;

// ============================
// ESP-IDF I2C 函数 (i2c_master - 原生驱动)
// ESP-IDF I2C Functions (i2c_master - native driver)
// ============================

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

// 使用i2c_master的PM1睡眠模式唤醒信号
// Wake signal for PM1 sleep mode using i2c_master
#ifndef M5PM1_I2C_MASTER_SEND_WAKE
static inline esp_err_t M5PM1_I2C_MASTER_SEND_WAKE(i2c_master_bus_handle_t bus, uint8_t addr)
{
    // Use i2c_master_probe to generate START signal for wake
    return i2c_master_probe(bus, addr, 10);
}
#endif

#ifdef __cplusplus
}
#endif

#endif  // ARDUINO

#endif  // __M5PM1_I2C_COMPAT_H__
