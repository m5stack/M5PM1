/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file M5PM1.h
 * @brief M5Stack PM1 Power Management IC Driver Library (Dual-Platform: ESP-IDF & Arduino)
 *        M5Stack PM1 电源管理IC驱动库（双平台：ESP-IDF 和 Arduino）
 *
 * @note PM1 is a multi-function power management IC supporting:
 *       PM1 是一款多功能电源管理IC，支持：
 *       - Battery charging and monitoring / 电池充电和监控
 *       - Multiple power rails (DCDC 5V, LDO 3.3V) / 多路电源轨（DCDC 5V，LDO 3.3V）
 *       - 5 GPIO pins with various functions / 5个GPIO引脚支持多种功能
 *       - PWM output, ADC input / PWM输出、ADC输入
 *       - NeoPixel LED control / NeoPixel LED控制
 *       - Watchdog timer / 看门狗定时器
 *       - RTC RAM (32 bytes, retained in sleep) / RTC RAM（32字节，睡眠保持）
 *       - I2C auto-sleep/wake feature / I2C自动睡眠/唤醒功能
 */

#ifndef _M5PM1_H_
#define _M5PM1_H_

#include "M5PM1_i2c_compat.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef ARDUINO
// Arduino: FreeRTOS headers included via Arduino framework
#else
// ESP-IDF specific includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#endif

// ============================
// Error Codes
// 错误码
// ============================
typedef enum {
    M5PM1_OK = 0,                  // 成功
                                   // Success
    M5PM1_FAIL = -1,               // 一般失败
                                   // General failure
    M5PM1_ERR_I2C_CONFIG = -2,     // I2C 配置错误 (如频率不支持)
                                   // I2C configuration error
    M5PM1_ERR_RULE_VIOLATION = -3, // 条件规则错误 (如引脚冲突，互斥功能)
                                   // Condition rule violation
    M5PM1_ERR_INVALID_ARG = -4,    // 无效参数
                                   // Invalid argument
    M5PM1_ERR_TIMEOUT = -5,        // 超时
                                   // Timeout
    M5PM1_ERR_NOT_SUPPORTED = -6,  // 不支持的功能
                                   // Function not supported
    M5PM1_ERR_I2C_COMM = -7,       // I2C 通信错误
                                   // I2C communication error
    M5PM1_ERR_NOT_INIT = -8,       // 设备未初始化
                                   // Device not initialized
    M5PM1_ERR_INTERNAL = -9,       // 内部错误
                                   // Internal error
    M5PM1_ERR_VERIFY_FAILED = -10, // 配置校验失败（读回值与缓存不匹配）
                                   // Configuration verification failed
} m5pm1_err_t;

// ============================
// Device Constants
// 设备常量
// ============================
#define M5PM1_DEFAULT_ADDR          0x6E    // Default I2C address / 默认I2C地址
#define M5PM1_MAX_GPIO_PINS         5       // GPIO0-GPIO4, total 5 pins / GPIO0-GPIO4，共5个引脚
#define M5PM1_MAX_PWM_CHANNELS      2       // PWM0 (GPIO3), PWM1 (GPIO4) / PWM0（GPIO3），PWM1（GPIO4）
#define M5PM1_MAX_ADC_CHANNELS      3       // ADC1 (GPIO1), ADC2 (GPIO2), TEMP / ADC1（GPIO1），ADC2（GPIO2），温度
#define M5PM1_MAX_LED_COUNT         32      // Maximum NeoPixel LED count / 最大NeoPixel LED数量
#define M5PM1_RTC_RAM_SIZE          32      // RTC RAM size in bytes (retained in sleep) / RTC RAM大小（睡眠保持）

// ============================
// I2C Frequency Constants
// I2C 频率常量
// ============================
#define M5PM1_I2C_FREQ_100K         100000  // Standard mode / 标准模式
#define M5PM1_I2C_FREQ_400K         400000  // Fast mode / 快速模式
#define M5PM1_I2C_FREQ_DEFAULT      M5PM1_I2C_FREQ_100K
// I2C 重试参数
// I2C retry settings
#define M5PM1_I2C_RETRY_COUNT       2
#define M5PM1_I2C_RETRY_DELAY_MS    2

// ============================
// Register Addresses
// 寄存器地址
// ============================

// ---- System Registers ----
// ---- 系统寄存器 ----
#define M5PM1_REG_DEVICE_ID         0x00    // R     [7:0] Device ID / 设备ID
#define M5PM1_REG_DEVICE_MODEL      0x01    // R     [7:0] Device Model / 设备型号
#define M5PM1_REG_HW_REV            0x02    // R     [7:0] Hardware version / 硬件版本
#define M5PM1_REG_SW_REV            0x03    // R     [7:0] Software/Firmware version / 固件版本

// ---- Power Status Registers ----
// ---- 电源状态寄存器 ----
#define M5PM1_REG_PWR_SRC           0x04    // R     [2:0] Power source / 当前供电源
                                            //       0: 5VIN (USB/DC input)
                                            //       5VIN（USB/DC输入）
                                            //       1: 5VINOUT (bidirectional port)
                                            //       5VINOUT（双向端口）
                                            //       2: BAT (battery)
                                            //       电池
#define M5PM1_REG_WAKE_SRC          0x05    // R/W   [6:0] Wake source flags (write 1 to clear)
                                            //       唤醒源标志（写1清除）
                                            //       [6] 5VINOUT插入唤醒
                                            //       5VINOUT insertion wake
                                            //       [5] 外部GPIO唤醒
                                            //       External GPIO wake
                                            //       [4] 命令复位唤醒
                                            //       Command reset wake
                                            //       [3] 复位按钮唤醒
                                            //       Reset button wake
                                            //       [2] 电源按钮唤醒
                                            //       Power button wake
                                            //       [1] VIN插入唤醒
                                            //       VIN insertion wake
                                            //       [0] 定时器唤醒
                                            //       Timer wake

// ---- Power Configuration ----
// ---- 电源配置 ----
#define M5PM1_REG_PWR_CFG           0x06    // R/W   [7-5] Reserved 保留
                                            //       [4] LED_EN 默认电平状态: 0=低电平 1=高电平
                                            //       [3] 5V_INOUT - 5V双向端口: 0=输入模式 1=输出模式(由DCDC供电)
                                            //       [2] LDO_EN - 3.3V LDO使能: 0=关闭 1=开启
                                            //       [1] DCDC_EN - 5V DCDC使能: 0=关闭 1=开启
                                            //       [0] CHG_EN - 充电使能: 0=关闭 1=开启
                                            //       注意：下载模式或复位事件时自动清零
                                            //       Note: Auto-clears on download mode or reset events

#define M5PM1_REG_HOLD_CFG          0x07    // R/W   Power hold configuration / 电源保持配置
                                            //       [7] Reserved 保留
                                            //       [6] DCDC(5V) - DCDC电源保持
                                            //       [5] LDO(3.3V) - LDO电源保持
                                            //       [4] GPIO4 - GPIO4输出状态保持
                                            //       [3] GPIO3 - GPIO3输出状态保持
                                            //       [2] GPIO2 - GPIO2输出状态保持
                                            //       [1] GPIO1 - GPIO1输出状态保持
                                            //       [0] GPIO0 - GPIO0输出状态保持
                                            //       注意：复位/下载模式/关机时自动清零为0x00
                                            //       Note: Auto-clears to 0x00 on reset/download/shutdown

#define M5PM1_REG_BATT_LVP          0x08    // R/W   Battery low voltage protection threshold
                                            //       电池低压保护阈值
                                            //       Value = (voltage_mV - 2000) / 7.81
                                            //       电压(mV) = 2000 + n * 7.81
                                            //       Range: 2000mV ~ 4000mV
                                            //       范围：2000mV ~ 4000mV

#define M5PM1_REG_I2C_CFG           0x09    // R/W   I2C configuration / I2C配置
                                            //       [7-5] Reserved 保留
                                            //       [4] SPD - I2C速度: 0=100KHz 1=400KHz
                                            //       [3:0] SLP_TO - I2C睡眠超时(秒): 0=禁用 1-15=1-15秒
                                            //       Note: PM1 enters sleep after I2C idle timeout
                                            //       注意：I2C空闲超时后PM1进入睡眠模式

// ---- Watchdog Registers ----
// ---- 看门狗寄存器 ----
#define M5PM1_REG_WDT_CNT           0x0A    // R/W   Watchdog countdown (seconds) / 看门狗倒计时（秒）
                                            //       0 = disabled
                                            //       0=禁用
                                            //       1-255 = timeout in seconds
                                            //       1-255=超时秒数
                                            //       系统复位当倒计时到0
                                            //       System resets when countdown reaches 0
#define M5PM1_REG_WDT_KEY           0x0B    // W     Write 0xA5 to feed watchdog / 写入0xA5喂狗
#define M5PM1_REG_SYS_CMD           0x0C    // W     System command / 系统命令
                                            //       [7:4] KEY must be 0xA
                                            //       密钥必须为0xA
                                            //       [3:2] Reserved 保留
                                            //       [1:0] CMD - 00:无操作 01:关机 10:复位 11:下载模式

// ---- GPIO Registers ----
// ---- GPIO寄存器 ----
#define M5PM1_REG_GPIO_MODE         0x10    // R/W   [4:0] GPIO direction / GPIO方向
                                            //       每bit对应一个GPIO: 1=输出 0=输入
                                            //       Each bit: 1=output, 0=input
#define M5PM1_REG_GPIO_OUT          0x11    // R/W   [4:0] GPIO output level / GPIO输出电平
                                            //       每bit对应一个GPIO: 1=高 0=低
                                            //       Each bit: 1=high, 0=low
#define M5PM1_REG_GPIO_IN           0x12    // R     [4:0] GPIO input state / GPIO输入状态
                                            //       每bit对应一个GPIO的当前输入电平
                                            //       Each bit: current input level of GPIO
#define M5PM1_REG_GPIO_DRV          0x13    // R/W   GPIO drive mode / GPIO驱动模式
                                            //       [7-6] Reserved 保留
                                            //       [5] LED_EN_DRV - LED使能引脚驱动: 0=推挽 1=开漏
                                            //       [4:0] GPIO drive
                                            //       GPIO驱动: 1=开漏 0=推挽
#define M5PM1_REG_GPIO_PUPD0        0x14    // R/W   Pull-up/down for GPIO0-3 / GPIO0-3上下拉配置
                                            //       每GPIO占2bit: 00=无 01=上拉 10=下拉 11=保留
                                            //       [7:6] GPIO3, [5:4] GPIO2, [3:2] GPIO1, [1:0] GPIO0
#define M5PM1_REG_GPIO_PUPD1        0x15    // R/W   Pull-up/down for GPIO4 / GPIO4上下拉配置
                                            //       [1:0] GPIO4: 00=无 01=上拉 10=下拉 11=保留
#define M5PM1_REG_GPIO_FUNC0        0x16    // R/W   GPIO0-3 function / GPIO0-3功能选择
                                            //       每GPIO占2bit: 00=GPIO 01=IRQ 10=WAKE 11=特殊功能
                                            //       GPIO0: 11=LED_EN (NeoPixel使能)
                                            //       GPIO1: 11=ADC1
                                            //       GPIO2: 11=ADC2
                                            //       GPIO3: 11=PWM0
#define M5PM1_REG_GPIO_FUNC1        0x17    // R/W   GPIO4 function / GPIO4功能选择
                                            //       [1:0] GPIO4: 00=GPIO 01=IRQ 10=WAKE 11=PWM1
#define M5PM1_REG_GPIO_WAKE_EN      0x18    // R/W   [4:0] GPIO wake enable / GPIO唤醒使能
                                            //       每bit使能对应GPIO的唤醒功能
                                            //       Each bit enables wake function for corresponding GPIO
#define M5PM1_REG_GPIO_WAKE_CFG     0x19    // R/W   [4:0] GPIO wake edge config / GPIO唤醒边沿配置
                                            //       每bit: 0=下降沿唤醒 1=上升沿唤醒
                                            //       Each bit: 0=falling edge, 1=rising edge

// ---- Voltage Reading Registers ----
// ---- 电压读取寄存器 ----
#define M5PM1_REG_VREF_L            0x20    // R     VREF low byte (mV) / 参考电压低字节
#define M5PM1_REG_VREF_H            0x21    // R     VREF high byte / 参考电压高字节
#define M5PM1_REG_VBAT_L            0x22    // R     Battery voltage low byte (mV) / 电池电压低字节
#define M5PM1_REG_VBAT_H            0x23    // R     Battery voltage high 4 bits / 电池电压高4位
#define M5PM1_REG_VIN_L             0x24    // R     VIN voltage low byte (mV) / VIN电压低字节
#define M5PM1_REG_VIN_H             0x25    // R     VIN voltage high 4 bits / VIN电压高4位
#define M5PM1_REG_5VINOUT_L         0x26    // R     5VINOUT voltage low byte (mV) / 5VINOUT电压低字节
#define M5PM1_REG_5VINOUT_H         0x27    // R     5VINOUT voltage high 4 bits / 5VINOUT电压高4位
#define M5PM1_REG_ADC_RES_L         0x28    // R     ADC result low byte (mV) / ADC结果低字节
#define M5PM1_REG_ADC_RES_H         0x29    // R     ADC result high 4 bits / ADC结果高4位
#define M5PM1_REG_ADC_CTRL          0x2A    // R/W   ADC control / ADC控制
                                            //       [7-4] Reserved 保留
                                            //       [3:1] Channel
                                            //       通道: 1=ADC1(GPIO1) 2=ADC2(GPIO2) 6=芯片内部温度
                                            //       [0] START - 写1启动转换
                                            //       Write 1 to start conversion

// ---- PWM Registers ----
// ---- PWM寄存器 ----
#define M5PM1_REG_PWM0_L            0x30    // R/W   PWM0 duty low byte / PWM0占空比低字节
#define M5PM1_REG_PWM0_HC           0x31    // R/W   PWM0 high byte + control / PWM0高字节+控制
                                            //       [7-6] Reserved 保留
                                            //       [5] POL - 极性: 0=正常 1=反转
                                            //       [4] EN - 使能: 0=关闭 1=开启
                                            //       [3:0] Duty high 4 bits
                                            //       占空比高4位
                                            //       12-bit duty: 0-4095 (0-100%)
                                            //       12位占空比
#define M5PM1_REG_PWM1_L            0x32    // R/W   PWM1 duty low byte / PWM1占空比低字节
#define M5PM1_REG_PWM1_HC           0x33    // R/W   PWM1 high byte + control / PWM1高字节+控制
                                            //       Same format as PWM0_HC
                                            //       格式同PWM0_HC
#define M5PM1_REG_PWM_FREQ_L        0x34    // R/W   PWM frequency low byte (Hz) / PWM频率低字节
#define M5PM1_REG_PWM_FREQ_H        0x35    // R/W   PWM frequency high byte / PWM频率高字节
                                            //       Range: 1-65535 Hz
                                            //       范围：1-65535 Hz

// ---- Timer Registers ----
// ---- 定时器寄存器 ----
#define M5PM1_REG_TIM_CNT_0         0x38    // R/W   Timer counter byte 0 (LSB) / 定时器计数字节0
#define M5PM1_REG_TIM_CNT_1         0x39    // R/W   Timer counter byte 1 / 定时器计数字节1
#define M5PM1_REG_TIM_CNT_2         0x3A    // R/W   Timer counter byte 2 / 定时器计数字节2
#define M5PM1_REG_TIM_CNT_3         0x3B    // R/W   Timer counter byte 3 (bit 6:0, max 31 bits)
                                            //       31位定时器，单位秒，最大约68年
                                            //       31-bit timer in seconds, max ~68 years
#define M5PM1_REG_TIM_CFG           0x3C    // R/W   Timer configuration / 定时器配置
                                            //       [7-4] Reserved 保留
                                            //       [3] ARM - 自动重装: 0=单次 1=自动重装
                                            //       [2:0] ACTION - 超时动作:
                                            //         000=停止 001=标志 010=复位 011=开机 100=关机
#define M5PM1_REG_TIM_KEY           0x3D    // W     Write 0xA5 to reload timer / 写入0xA5重载定时器

// ---- IRQ Registers ----
// ---- 中断寄存器 ----
// 注意：状态寄存器写1清除对应位
// Note: Write 1 to clear status bits
#define M5PM1_REG_IRQ_STATUS1       0x40    // R/W   [4:0] GPIO interrupt status / GPIO中断状态
                                            //       触发条件：GPIO配置为IRQ功能时的边沿触发
                                            //       Trigger: Edge trigger when GPIO configured as IRQ
#define M5PM1_REG_IRQ_STATUS2       0x41    // R/W   System interrupt status / 系统中断状态
                                            //       [7-6] Reserved 保留
                                            //       [5] 电池移除
                                            //       Battery removed
                                            //       [4] 电池插入
                                            //       Battery inserted
                                            //       [3] 5VINOUT移除
                                            //       5VINOUT removed
                                            //       [2] 5VINOUT插入
                                            //       5VINOUT inserted
                                            //       [1] 5VIN移除
                                            //       5VIN removed
                                            //       [0] 5VIN插入
                                            //       5VIN inserted
                                            //       备注：电池事件仅充电使能时有效，5VINOUT事件仅输入模式时有效
                                            //       Note: Battery events only when charging enabled
                                            //             5VINOUT events only when in input mode
#define M5PM1_REG_IRQ_STATUS3       0x42    // R/W   Button interrupt status / 按钮中断状态
                                            //       [7-3] Reserved 保留
                                            //       [2] 双击事件
                                            //       Double click event
                                            //       [1] 唤醒事件
                                            //       Wakeup event
                                            //       [0] 单击事件
                                            //       Single click event
#define M5PM1_REG_IRQ_MASK1         0x43    // R/W   GPIO interrupt mask / GPIO中断屏蔽
                                            //       [4:0] 每bit: 0=使能中断 1=屏蔽中断
#define M5PM1_REG_IRQ_MASK2         0x44    // R/W   System interrupt mask / 系统中断屏蔽
#define M5PM1_REG_IRQ_MASK3         0x45    // R/W   Button interrupt mask / 按钮中断屏蔽

// ---- Button Registers ----
// ---- 按钮寄存器 ----
#define M5PM1_REG_BTN_STATUS        0x48    // R     Button status / 按钮状态
                                            //       [7] BTN_FLAG - 按钮曾被按下标志（读取后自动清除）
                                            //       [6-1] Reserved 保留
                                            //       [0] BTN_STATE - 当前按钮状态: 0=释放 1=按下
#define M5PM1_REG_BTN_CFG_1           0x49    // R/W   Button configuration / 按钮配置
                                            //       [7] DL_LOCK - 下载模式锁定: 0=正常 1=锁定（禁止进入下载模式）
                                            //       [6:5] LONG_DLY - 长按延时: 00=125ms 01=250ms 10=500ms 11=1s
                                            //       [4:3] DBL_DLY - 双击间隔: 00=125ms 01=250ms 10=500ms 11=1s
                                            //       [2:1] CLK_DLY - 单击延时: 00=125ms 01=250ms 10=500ms 11=1s
                                            //       [0] SINGLE_RST_DIS - 单击复位禁用: 0=使能 1=禁用
#define M5PM1_REG_BTN_CFG_2         0x4A    // R/W   Button configuration 2 / 按钮配置2
                                            //       [7-1] Reserved 保留
                                            //       [0] DOUBLE_OFF_DIS - 双击关机禁用: 0=使能 1=禁用

// ---- NeoPixel Registers ----
// ---- NeoPixel寄存器 ----
#define M5PM1_REG_NEO_CFG           0x50    // R/W   NeoPixel configuration / NeoPixel配置
                                            //       [7-6] Reserved 保留
                                            //       [5] REFRESH - 写1刷新LED
                                            //       Write 1 to refresh LEDs
                                            //       [4:0] LED_CNT - LED数量 (0-32)
#define M5PM1_REG_AW8737A_PULSE     0x53    // R/W   AW8737A pulse control / AW8737A脉冲控制
                                            //       用于控制AW8737A音频放大器增益
                                            //       For controlling AW8737A audio amplifier gain
                                            //       [7] REFRESH - 写1执行脉冲
                                            //       Write 1 to execute pulse
                                            //       [6:5] NUM - 脉冲数量: 00=0 01=1 10=2 11=3
                                            //       [4:0] GPIO - 输出GPIO引脚号

// ---- NeoPixel Data Area ----
// ---- NeoPixel数据区 ----
#define M5PM1_REG_NEO_DATA_START    0x60    // R/W   NeoPixel RGB565 data start / 数据起始地址
#define M5PM1_REG_NEO_DATA_END      0x9F    // R/W   NeoPixel RGB565 data end / 数据结束地址
                                            //       每LED占2字节(RGB565格式), 共32个LED
                                            //       Each LED: 2 bytes (RGB565), total 32 LEDs

// ---- RTC RAM Area ----
// ---- RTC RAM区域 ----
#define M5PM1_REG_RTC_RAM_START     0xA0    // R/W   RTC RAM start (32 bytes) / RTC RAM起始
#define M5PM1_REG_RTC_RAM_END       0xBF    // R/W   RTC RAM end / RTC RAM结束
                                            //       睡眠期间数据保持，可用于存储小量重要数据
                                            //       Data retained during sleep, for storing small important data

// ============================
// Bit Definitions
// 位定义
// ============================

// ---- I2C_CFG Register Bits ----
// ---- I2C配置寄存器位 ----
#define M5PM1_I2C_CFG_SLEEP_MASK    0x0F        // I2C睡眠超时掩码 / I2C sleep timeout mask
                                                // 0=禁用 1-15=1-15秒
#define M5PM1_I2C_CFG_SPEED_400K    (1 << 4)    // I2C速度选择 / I2C speed select
                                                // 0=100KHz 1=400KHz

// ---- SYS_CMD Register Values ----
// ---- 系统命令寄存器值 ----
#define M5PM1_SYS_CMD_KEY           0xA0        // 命令密钥 / Command key (must be 0xA in high nibble)
#define M5PM1_SYS_CMD_SHUTDOWN      0x01        // 关机命令 / Shutdown command
#define M5PM1_SYS_CMD_REBOOT        0x02        // 复位命令 / Reboot command
#define M5PM1_SYS_CMD_JTAG          0x03        // 下载模式命令 / Download mode command

// ---- Key Values ----
// ---- 密钥值 ----
#define M5PM1_WDT_FEED_KEY          0xA5        // 喂狗密钥 / Watchdog feed key
#define M5PM1_TIM_RELOAD_KEY        0xA5        // 定时器重载密钥 / Timer reload key

// ---- NEO_CFG Register Bits ----
// ---- NeoPixel配置寄存器位 ----
#define M5PM1_NEO_CFG_REFRESH       (1 << 5)    // 刷新标志 / Refresh flag (write 1 to update LEDs)
#define M5PM1_NEO_CFG_COUNT_MASK    0x1F        // LED数量掩码 / LED count mask (0-31)

// ============================
// Enumerations
// 枚举类型
// ============================

/**
 * @brief GPIO pin number / GPIO引脚编号
 * @note GPIO0: LED_EN功能 / LED_EN function
 *       GPIO1: ADC1功能 / ADC1 function
 *       GPIO2: ADC2功能 / ADC2 function
 *       GPIO3: PWM0功能 / PWM0 function
 *       GPIO4: PWM1功能 / PWM1 function
 */
typedef enum {
    M5PM1_GPIO_NUM_0 = 0,       // GPIO0 (可用于LED_EN)
                                // (can be LED_EN)
    M5PM1_GPIO_NUM_1 = 1,       // GPIO1 (可用于ADC1)
                                // (can be ADC1)
    M5PM1_GPIO_NUM_2 = 2,       // GPIO2 (可用于ADC2)
                                // (can be ADC2)
    M5PM1_GPIO_NUM_3 = 3,       // GPIO3 (可用于PWM0)
                                // (can be PWM0)
    M5PM1_GPIO_NUM_4 = 4,       // GPIO4 (可用于PWM1)
                                // (can be PWM1)
    M5PM1_GPIO_NUM_NC = 255     // 未连接
                                // Not connected
} m5pm1_gpio_num_t;

/**
 * @brief GPIO direction mode / GPIO方向模式
 */
typedef enum {
    M5PM1_GPIO_MODE_INPUT = 0,  // 输入模式
                                // Input mode
    M5PM1_GPIO_MODE_OUTPUT = 1  // 输出模式
                                // Output mode
} m5pm1_gpio_mode_t;

/**
 * @brief GPIO function selection / GPIO功能选择
 * @note 特殊功能(0b11)因引脚而异:
 *       Special function (0b11) varies by pin:
 *       - GPIO0: LED_EN (NeoPixel使能)
 *       - GPIO1: ADC1 (模拟输入)
 *       - GPIO2: ADC2 (模拟输入)
 *       - GPIO3: PWM0 (脉宽调制输出)
 *       - GPIO4: PWM1 (脉宽调制输出)
 */
typedef enum {
    M5PM1_GPIO_FUNC_GPIO = 0b00,    // 普通GPIO功能
                                    // Normal GPIO function
    M5PM1_GPIO_FUNC_IRQ = 0b01,     // 中断触发功能
                                    // Interrupt trigger function
    M5PM1_GPIO_FUNC_WAKE = 0b10,    // 唤醒功能
                                    // Wake function
    M5PM1_GPIO_FUNC_OTHER = 0b11   // 特殊功能(LED/PWM/ADC)
                                   // Special function
} m5pm1_gpio_func_t;

/**
 * @brief GPIO pull-up/pull-down configuration / GPIO上下拉配置
 */
typedef enum {
    M5PM1_GPIO_PULL_NONE = 0b00,    // 无上下拉
                                    // No pull
    M5PM1_GPIO_PULL_UP = 0b01,      // 上拉使能
                                    // Pull-up enabled
    M5PM1_GPIO_PULL_DOWN = 0b10     // 下拉使能
                                    // Pull-down enabled
} m5pm1_gpio_pull_t;

/**
 * @brief GPIO output drive mode / GPIO输出驱动模式
 */
typedef enum {
    M5PM1_GPIO_DRIVE_PUSHPULL = 0,  // 推挽输出
                                    // Push-pull output
    M5PM1_GPIO_DRIVE_OPENDRAIN = 1  // 开漏输出
                                    // Open-drain output
} m5pm1_gpio_drive_t;

/**
 * @brief GPIO wake edge configuration / GPIO唤醒边沿配置
 */
typedef enum {
    M5PM1_GPIO_WAKE_FALLING = 0,    // 下降沿唤醒
                                    // Wake on falling edge
    M5PM1_GPIO_WAKE_RISING = 1      // 上升沿唤醒
                                    // Wake on rising edge
} m5pm1_gpio_wake_edge_t;

/**
 * @brief ADC channel selection / ADC通道选择
 * @note GPIO必须配置为FUNC_OTHER(0b11)才能使用ADC功能
 *       GPIO must be configured as FUNC_OTHER (0b11) to use ADC function
 */
typedef enum {
    M5PM1_ADC_CH_1 = 1,             // ADC通道1 (GPIO1)
                                    // ADC channel 1 (GPIO1)
    M5PM1_ADC_CH_2 = 2,             // ADC通道2 (GPIO2)
                                    // ADC channel 2 (GPIO2)
    M5PM1_ADC_CH_TEMP = 6           // 内部温度传感器
                                    // Internal temperature sensor
} m5pm1_adc_channel_t;

/**
 * @brief PWM channel selection / PWM通道选择
 * @note GPIO必须配置为FUNC_OTHER(0b11)才能使用PWM功能
 *       GPIO must be configured as FUNC_OTHER (0b11) to use PWM function
 */
typedef enum {
    M5PM1_PWM_CH_0 = 0,             // PWM通道0 (GPIO3)
                                    // PWM channel 0 (GPIO3)
    M5PM1_PWM_CH_1 = 1              // PWM通道1 (GPIO4)
                                    // PWM channel 1 (GPIO4)
} m5pm1_pwm_channel_t;

/**
 * @brief Current power source / 当前供电源
 */
typedef enum {
    M5PM1_PWR_SRC_5VIN = 0,         // USB/DC 5V输入
                                    // USB/DC 5V input
    M5PM1_PWR_SRC_5VINOUT = 1,      // 5V双向端口输入
                                    // 5V bidirectional port input
    M5PM1_PWR_SRC_BAT = 2,          // 电池供电
                                    // Battery power
    M5PM1_PWR_SRC_UNKNOWN = 3       // 未知/无供电
                                    // Unknown/no power
} m5pm1_pwr_src_t;

/**
 * @brief Wake source flags / 唤醒源标志
 * @note 可组合使用(位掩码) / Can be combined (bitmask)
 *       写1清除对应唤醒标志 / Write 1 to clear corresponding flag
 */
typedef enum {
    M5PM1_WAKE_SRC_TIM = 0x01,      // 定时器唤醒
                                    // Timer wake
    M5PM1_WAKE_SRC_VIN = 0x02,      // VIN插入唤醒
                                    // VIN insertion wake
    M5PM1_WAKE_SRC_PWRBTN = 0x04,   // 电源按钮唤醒
                                    // Power button wake
    M5PM1_WAKE_SRC_RSTBTN = 0x08,   // 复位按钮唤醒
                                    // Reset button wake
    M5PM1_WAKE_SRC_CMD_RST = 0x10,  // 命令复位唤醒
                                    // Command reset wake
    M5PM1_WAKE_SRC_EXT_WAKE = 0x20, // 外部GPIO唤醒
                                    // External GPIO wake
    M5PM1_WAKE_SRC_5VINOUT = 0x40   // 5VINOUT插入唤醒
                                    // 5VINOUT insertion wake
} m5pm1_wake_src_t;

/**
 * @brief System command / 系统命令
 * @note 命令需要与密钥(0xA0)组合使用
 *       Command needs to be combined with key (0xA0)
 */
typedef enum {
    M5PM1_SYS_CMD_NONE = 0x00,      // 无操作
                                    // No operation
    M5PM1_SYS_CMD_OFF = 0x01,       // 关机
                                    // Shutdown
    M5PM1_SYS_CMD_RESET = 0x02,     // 复位
                                    // Reset
    M5PM1_SYS_CMD_DL = 0x03         // 进入下载模式
                                    // Enter download mode
} m5pm1_sys_cmd_t;

/**
 * @brief Timer timeout action / 定时器超时动作
 */
typedef enum {
    M5PM1_TIM_ACTION_STOP = 0b000,      // 停止，无动作
                                        // Stop, no action
    M5PM1_TIM_ACTION_FLAG = 0b001,      // 仅设置标志
                                        // Set flag only
    M5PM1_TIM_ACTION_REBOOT = 0b010,    // 系统复位
                                        // System reboot
    M5PM1_TIM_ACTION_POWERON = 0b011,   // 开机
                                        // Power on
    M5PM1_TIM_ACTION_POWEROFF = 0b100   // 关机
                                        // Power off
} m5pm1_tim_action_t;

/**
 * @brief Button event type / 按钮事件类型
 */
typedef enum {
    M5PM1_BTN_TYPE_CLICK = 0,       // 单击
                                    // Single click
    M5PM1_BTN_TYPE_DOUBLE = 1,      // 双击
                                    // Double click
    M5PM1_BTN_TYPE_LONG = 2         // 长按
                                    // Long press
} m5pm1_btn_type_t;

/**
 * @brief Button delay/timeout configuration / 按钮延时配置
 */
typedef enum {
    M5PM1_BTN_DELAY_125MS = 0,      // 125毫秒
                                    // 125 milliseconds
    M5PM1_BTN_DELAY_250MS = 1,      // 250毫秒
                                    // 250 milliseconds
    M5PM1_BTN_DELAY_500MS = 2,      // 500毫秒
                                    // 500 milliseconds
    M5PM1_BTN_DELAY_1000MS = 3      // 1000毫秒
                                    // 1000 milliseconds
} m5pm1_btn_delay_t;

/**
 * @brief Button IRQ interrupt type / 按钮中断类型
 * @note 用于 IRQ_STATUS3 寄存器的位定义 / For IRQ_STATUS3 register bit definitions
 */
typedef enum {
    M5PM1_BTN_IRQ_CLICK = 0x00,     // 单击中断
                                    // Single click interrupt
    M5PM1_BTN_IRQ_WAKEUP = 0x01,    // 唤醒中断
                                    // Wakeup interrupt
    M5PM1_BTN_IRQ_DOUBLE = 0x02,    // 双击中断
                                    // Double click interrupt
    M5PM1_BTN_IRQ_ALL = 0x07,       // 所有按钮中断
                                    // All button interrupts
    M5PM1_BTN_IRQ_NONE = 0xFF       // 无中断
                                    // No interrupt
} m5pm1_btn_irq_t;

/**
 * @brief I2C speed selection / I2C速度选择
 * @note PM1上电默认100KHz，切换400KHz需要先写入配置再重新初始化I2C
 *       PM1 defaults to 100KHz on power-up, switching to 400KHz requires
 *       writing config first then re-initializing I2C
 */
typedef enum {
    M5PM1_I2C_SPEED_100K = 0,       // 100KHz 标准模式
                                    // Standard mode
    M5PM1_I2C_SPEED_400K = 1        // 400KHz 快速模式
                                    // Fast mode
} m5pm1_i2c_speed_t;

// 日志级别定义
// Log level definitions
typedef enum {
    M5PM1_LOG_LEVEL_NONE = 0,  // 无日志输出
                               // No log output
    M5PM1_LOG_LEVEL_ERROR,     // 仅错误消息
                               // Error messages only
    M5PM1_LOG_LEVEL_WARN,      // 警告和错误消息
                               // Warning and error messages
    M5PM1_LOG_LEVEL_INFO,      // 信息、警告和错误消息（默认）
                               // Info, warning and error messages (default)
    M5PM1_LOG_LEVEL_DEBUG,     // 调试、信息、警告和错误消息
                               // Debug, info, warning and error messages
    M5PM1_LOG_LEVEL_VERBOSE    // 所有消息包括详细输出
                               // All messages including verbose
} m5pm1_log_level_t;

/**
 * @brief AW8737A pulse count for gain control / AW8737A增益控制脉冲数
 * @note AW8737A是一款音频功率放大器，通过脉冲数设置增益
 *       AW8737A is an audio power amplifier, gain is set by pulse count
 */
typedef enum {
    M5PM1_AW8737A_PULSE_0 = 0,      // 0个脉冲 (静音)
                                    // 0 pulses (mute)
    M5PM1_AW8737A_PULSE_1 = 1,      // 1个脉冲
                                    // 1 pulse
    M5PM1_AW8737A_PULSE_2 = 2,      // 2个脉冲
                                    // 2 pulses
    M5PM1_AW8737A_PULSE_3 = 3       // 3个脉冲
                                    // 3 pulses
} m5pm1_aw8737a_pulse_t;

/**
 * @brief AW8737A pulse refresh mode / AW8737A脉冲刷新模式
 */
typedef enum {
    M5PM1_AW8737A_REFRESH_WAIT = 0, // 等待手动刷新
                                    // Wait for manual refresh
    M5PM1_AW8737A_REFRESH_NOW = 1   // 立即执行
                                    // Execute immediately
} m5pm1_aw8737a_refresh_t;

// ============================
// 清除类型枚举（用于 IRQ/WAKE 状态读取）
// Clean Type Enumeration (for IRQ/WAKE status read)
// ============================
/**
 * @brief 状态读取后的清除行为
 *        Clean behavior after status read
 * @note 用于 getWakeSource(), irqGetGpioStatus(), irqGetSysStatus(), irqGetBtnStatus()
 *       Used for getWakeSource(), irqGetGpioStatus(), irqGetSysStatus(), irqGetBtnStatus()
 */
typedef enum {
    M5PM1_CLEAN_NONE = 0x00,        // 不清除
                                    // No clean
    M5PM1_CLEAN_TRIGGERED = 0x01,   // 清除已触发的位
                                    // Clean triggered bits only
    M5PM1_CLEAN_ALL = 0x02          // 清除所有位
                                    // Clean all bits
} m5pm1_clean_type_t;

// ============================
// GPIO 中断标志枚举
// GPIO IRQ Flag Enumeration
// ============================
/**
 * @brief GPIO 中断标志位
 *        GPIO interrupt flag bits
 * @note 用于 irqGetGpioStatus(), irqClearGpio()
 *       Used for irqGetGpioStatus(), irqClearGpio()
 */
typedef enum {
    M5PM1_IRQ_GPIO0 = (1 << 0),     // GPIO0 中断标志
                                    // GPIO0 interrupt flag
    M5PM1_IRQ_GPIO1 = (1 << 1),     // GPIO1 中断标志
                                    // GPIO1 interrupt flag
    M5PM1_IRQ_GPIO2 = (1 << 2),     // GPIO2 中断标志
                                    // GPIO2 interrupt flag
    M5PM1_IRQ_GPIO3 = (1 << 3),     // GPIO3 中断标志
                                    // GPIO3 interrupt flag
    M5PM1_IRQ_GPIO4 = (1 << 4),     // GPIO4 中断标志
                                    // GPIO4 interrupt flag
    M5PM1_IRQ_GPIO_ALL = 0x1F,      // 所有 GPIO 中断标志
                                    // All GPIO interrupt flags
    M5PM1_IRQ_GPIO_NONE = 0x00      // 无中断
                                    // No interrupt
} m5pm1_irq_gpio_t;

// ============================
// 系统中断标志枚举
// System IRQ Flag Enumeration
// ============================
/**
 * @brief 系统中断标志位
 *        System interrupt flag bits
 * @note 用于 irqGetSysStatus(), irqClearSys()
 *       Used for irqGetSysStatus(), irqClearSys()
 */
typedef enum {
    M5PM1_IRQ_SYS_5VIN_INSERT = (1 << 0),       // 5VIN 插入
                                                // 5VIN insertion
    M5PM1_IRQ_SYS_5VIN_REMOVE = (1 << 1),       // 5VIN 移除
                                                // 5VIN removal
    M5PM1_IRQ_SYS_5VINOUT_INSERT = (1 << 2),    // 5VINOUT 插入
                                                // 5VINOUT insertion
    M5PM1_IRQ_SYS_5VINOUT_REMOVE = (1 << 3),    // 5VINOUT 移除
                                                // 5VINOUT removal
    M5PM1_IRQ_SYS_BAT_INSERT = (1 << 4),        // 电池插入
                                                // Battery insertion
    M5PM1_IRQ_SYS_BAT_REMOVE = (1 << 5),        // 电池移除
                                                // Battery removal
    M5PM1_IRQ_SYS_ALL = 0x3F,                   // 所有系统中断标志
                                                // All system interrupt flags
    M5PM1_IRQ_SYS_NONE = 0x00                   // 无中断
                                                // No interrupt
} m5pm1_irq_sys_t;

// ============================
// 电源配置位掩码枚举
// Power Configuration Bitmask Enumeration
// ============================
/**
 * @brief 电源配置位掩码
 *        Power configuration bitmask
 * @note 用于 setPowerConfig(), getPowerConfig()
 *       Used for setPowerConfig(), getPowerConfig()
 */
typedef enum {
    M5PM1_PWR_CFG_CHG_EN = (1 << 0),    // 充电使能
                                        // Charge enable
    M5PM1_PWR_CFG_DCDC_EN = (1 << 1),   // 5V DCDC 使能
                                        // 5V DCDC enable
    M5PM1_PWR_CFG_LDO_EN = (1 << 2),    // 3.3V LDO 使能
                                        // 3.3V LDO enable
    M5PM1_PWR_CFG_5V_INOUT = (1 << 3),  // 5V 双向端口模式
                                        // 5V bidirectional port mode
    M5PM1_PWR_CFG_LED_CTRL = (1 << 4)   // LED_EN 默认电平状态 0:低电平 1:高电平
                                        // LED_EN default level state 0: low 1: high
} m5pm1_pwr_cfg_t;

// ============================
// GPIO 输出状态枚举
// GPIO Output State Enumeration
// ============================
/**
 * @brief GPIO 输出状态
 *        GPIO output state
 * @note 用于 gpioSetOutput()
 *       Used for gpioSetOutput()
 */
typedef enum {
    M5PM1_GPIO_STATE_LOW = 0,       // 低电平
                                    // Low level
    M5PM1_GPIO_STATE_HIGH = 1       // 高电平
                                    // High level
} m5pm1_gpio_state_t;

// ============================
// GPIO 电平定义
// GPIO Level Definitions
// ============================
#ifndef LOW
#define LOW             0
#endif
#ifndef HIGH
#define HIGH            1
#endif

// ============================
// GPIO 模式定义（Arduino 兼容）
// GPIO Mode Definitions (Arduino-compatible)
// ============================
#ifndef INPUT
#define INPUT           0x01
#endif
#ifndef OUTPUT
#define OUTPUT          0x03
#endif
#ifndef PULLUP
#define PULLUP          0x04
#endif
#ifndef INPUT_PULLUP
#define INPUT_PULLUP    0x05
#endif
#ifndef PULLDOWN
#define PULLDOWN        0x08
#endif
#ifndef INPUT_PULLDOWN
#define INPUT_PULLDOWN  0x09
#endif
#ifndef OPEN_DRAIN
#define OPEN_DRAIN      0x10
#endif
#ifndef OUTPUT_OPEN_DRAIN
#define OUTPUT_OPEN_DRAIN 0x13
#endif
#ifndef ANALOG
#define ANALOG          0xC0
#endif

// ============================
// RGB Color Structure
// RGB 颜色结构
// ============================
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} m5pm1_rgb_t;

// ============================
// Pin Status Structure
// Pin 状态结构
// ============================
/**
 * @brief Cached GPIO pin status / 缓存的 GPIO 引脚状态
 * @note This structure maintains software-level pin state to reduce I2C transactions
 *       此结构维护软件层引脚状态以减少 I2C 事务
 */
typedef struct {
    m5pm1_gpio_func_t func;       ///< GPIO function / GPIO 功能 (GPIO/IRQ/WAKE/OTHER)
    m5pm1_gpio_mode_t mode;       ///< Direction / 方向 (INPUT/OUTPUT)
    uint8_t output;               ///< Output value / 输出值 (0/1)
    m5pm1_gpio_pull_t pull;       ///< Pull-up/pull-down / 上下拉 (NONE/UP/DOWN)
    bool wake_en;                 ///< Wake enabled / 唤醒使能
    m5pm1_gpio_wake_edge_t wake_edge; ///< Wake edge / 唤醒边沿 (RISING/FALLING)
    m5pm1_gpio_drive_t drive;     ///< Drive mode / 驱动模式 (PUSHPULL/OPENDRAIN)
    bool power_hold;              ///< Power hold state / 电源保持状态
} m5pm1_pin_status_t;

// ============================
// 用于验证的配置类型
// Configuration Type for Validation
// ============================
typedef enum {
    M5PM1_CONFIG_GPIO_INPUT = 0,
    M5PM1_CONFIG_GPIO_OUTPUT,
    M5PM1_CONFIG_GPIO_INTERRUPT,
    M5PM1_CONFIG_GPIO_WAKE,
    M5PM1_CONFIG_ADC,
    M5PM1_CONFIG_PWM,
    M5PM1_CONFIG_NEOPIXEL
} m5pm1_config_type_t;

// ============================
// 快照更新域
// Snapshot Update Domains
// ============================
typedef enum {
    M5PM1_SNAPSHOT_DOMAIN_GPIO       = 1 << 0,
    M5PM1_SNAPSHOT_DOMAIN_PWM        = 1 << 1,
    M5PM1_SNAPSHOT_DOMAIN_ADC        = 1 << 2,
    M5PM1_SNAPSHOT_DOMAIN_POWER      = 1 << 3,
    M5PM1_SNAPSHOT_DOMAIN_BUTTON     = 1 << 4,
    M5PM1_SNAPSHOT_DOMAIN_IRQ_MASK   = 1 << 5,
    M5PM1_SNAPSHOT_DOMAIN_IRQ_STATUS = 1 << 6,
    M5PM1_SNAPSHOT_DOMAIN_I2C        = 1 << 7,
    M5PM1_SNAPSHOT_DOMAIN_NEO        = 1 << 8,
    M5PM1_SNAPSHOT_DOMAIN_ALL        = 0x1FF
} m5pm1_snapshot_domain_t;

// ============================
// 配置验证结果
// Configuration Validation Result
// ============================
typedef struct {
    bool valid;
    char error_msg[64];
    uint8_t conflicting_pin;
} m5pm1_validation_t;

// ============================
// 快照验证结果
// Snapshot Verification Result
// ============================
typedef struct {
    bool consistent;            // 缓存与寄存器一致时为 true
                                // True if cached values match registers
    bool gpio_mismatch;         // GPIO 快照不一致
                                // GPIO snapshot mismatch
    bool pwm_mismatch;          // PWM 快照不一致
                                // PWM snapshot mismatch
    bool adc_mismatch;          // ADC 快照不一致
                                // ADC snapshot mismatch
    bool power_mismatch;        // 电源配置快照不一致
                                // Power config snapshot mismatch
    bool button_mismatch;       // 按钮配置快照不一致
                                // Button config snapshot mismatch
    bool irq_mask_mismatch;     // IRQ 掩码快照不一致
                                // IRQ mask snapshot mismatch
    bool i2c_mismatch;          // I2C 配置快照不一致
                                // I2C config snapshot mismatch
    bool neo_mismatch;          // Neo 配置快照不一致
                                // Neo config snapshot mismatch
    uint8_t expected_gpio_mode; // 缓存的 GPIO 模式值
                                // Cached GPIO mode value
    uint8_t actual_gpio_mode;   // 实际的 GPIO 模式值
                                // Actual GPIO mode value
    uint8_t expected_gpio_out;  // 缓存的 GPIO 输出值
                                // Cached GPIO output value
    uint8_t actual_gpio_out;    // 实际的 GPIO 输出值
                                // Actual GPIO output value
    uint8_t expected_pwr_cfg;   // 缓存的 PWR_CFG 值
                                // Cached PWR_CFG value
    uint8_t actual_pwr_cfg;     // 实际的 PWR_CFG 值
                                // Actual PWR_CFG value
    uint8_t expected_hold_cfg;  // 缓存的 HOLD_CFG 值
                                // Cached HOLD_CFG value
    uint8_t actual_hold_cfg;    // 实际的 HOLD_CFG 值
                                // Actual HOLD_CFG value
    uint8_t expected_i2c_cfg;   // 缓存的 I2C_CFG 值
                                // Cached I2C_CFG value
    uint8_t actual_i2c_cfg;     // 实际的 I2C_CFG 值
                                // Actual I2C_CFG value
    uint8_t expected_neo_cfg;   // 缓存的 NEO_CFG 值
                                // Cached NEO_CFG value
    uint8_t actual_neo_cfg;     // 实际的 NEO_CFG 值
                                // Actual NEO_CFG value
} m5pm1_snapshot_verify_t;

// ============================
// M5PM1 Class
// M5PM1 类
// ============================
class M5PM1 {
public:
    M5PM1();
    ~M5PM1();

    // ========================
    // Initialization
    // 初始化
    // ========================
#ifdef ARDUINO
    /**
     * @brief Initialize the device (Arduino)
     * @param wire Pointer to TwoWire instance
     * @param addr I2C address (default 0x6E)
     * @param sda SDA pin (default -1, uses default I2C pins)
     * @param scl SCL pin (default -1, uses default I2C pins)
     * @param speed I2C speed in Hz (default 100000)
     * @return 成功返回 M5PM1_OK，否则返回错误码
     *         Return M5PM1_OK on success, error code otherwise
     */
    m5pm1_err_t begin(TwoWire *wire = &Wire, uint8_t addr = M5PM1_DEFAULT_ADDR,
                      int8_t sda = -1, int8_t scl = -1, uint32_t speed = M5PM1_I2C_FREQ_100K);
#else // ESP-IDF
    /**
     * @brief Initialize with self-created I2C bus (ESP-IDF)
     * @param port I2C port number
     * @param addr I2C address (default 0x6E)
     * @param sda SDA pin
     * @param scl SCL pin
     * @param speed I2C speed in Hz
     * @return 成功返回 M5PM1_OK，否则返回错误码
     *         Return M5PM1_OK on success, error code otherwise
     */
    m5pm1_err_t begin(i2c_port_t port = I2C_NUM_0, uint8_t addr = M5PM1_DEFAULT_ADDR,
                      int sda = 21, int scl = 22, uint32_t speed = M5PM1_I2C_FREQ_100K);

    /**
     * @brief Initialize with existing i2c_master_bus handle (ESP-IDF native)
     * @param bus Existing i2c_master_bus_handle_t
     * @param addr I2C address (default 0x6E)
     * @param speed I2C speed in Hz
     * @return 成功返回 M5PM1_OK，否则返回错误码
     *         Return M5PM1_OK on success, error code otherwise
     */
    m5pm1_err_t begin(i2c_master_bus_handle_t bus, uint8_t addr = M5PM1_DEFAULT_ADDR,
                      uint32_t speed = M5PM1_I2C_FREQ_100K);

    /**
     * @brief Initialize with existing i2c_bus handle (esp-idf-lib)
     * @param bus Existing i2c_bus_handle_t
     * @param addr I2C address (default 0x6E)
     * @param speed I2C speed in Hz
     * @return 成功返回 M5PM1_OK，否则返回错误码
     *         Return M5PM1_OK on success, error code otherwise
     */
    m5pm1_err_t begin(i2c_bus_handle_t bus, uint8_t addr = M5PM1_DEFAULT_ADDR,
                      uint32_t speed = M5PM1_I2C_FREQ_100K);
#endif

    /**
     * @brief 设置全局日志级别
     *        Set global log level
     * @param level 日志级别
     *        Log level
     */
    static void setLogLevel(m5pm1_log_level_t level);

    /**
     * @brief 获取当前日志级别
     *        Get current log level
     * @return 日志级别
     *         Log level
     */
    static m5pm1_log_level_t getLogLevel();

    // ========================
    // 设备信息
    // Device Information
    // ========================
    m5pm1_err_t getDeviceId(uint8_t* id);
    m5pm1_err_t getDeviceModel(uint8_t* model);
    m5pm1_err_t getHwVersion(uint8_t* version);
    m5pm1_err_t getSwVersion(uint8_t* version);

    // ========================
    // GPIO 功能 (Arduino风格 - 带返回值)
    // GPIO Functions (Arduino-style - WithRes)
    // ========================
    void pinModeWithRes(uint8_t pin, uint8_t mode, m5pm1_err_t* err);
    void digitalWriteWithRes(uint8_t pin, uint8_t value, m5pm1_err_t* err);
    int digitalReadWithRes(uint8_t pin, m5pm1_err_t* err);

    // ========================
    // GPIO 功能 (Arduino风格)
    // GPIO Functions (Arduino-style)
    // ========================
    void pinMode(uint8_t pin, uint8_t mode);
    void digitalWrite(uint8_t pin, uint8_t value);
    int digitalRead(uint8_t pin);

    // ========================
    // 高级 GPIO 功能
    // Advanced GPIO Functions
    // ========================
    /**
     * @brief Configure GPIO in one call / 一键配置 GPIO
     * @param pin GPIO pin number (0-4) / GPIO 引脚号
     * @param mode Mode: M5PM1_GPIO_MODE_INPUT / OUTPUT
     * @param value Output level (only for output mode): 0/1
     * @param pull Pull mode: M5PM1_GPIO_PULL_NONE / UP / DOWN
     * @param drive Drive mode: M5PM1_GPIO_DRIVE_PUSHPULL / OPENDRAIN
     * @return M5PM1_OK if successful, error code otherwise
     */
    m5pm1_err_t gpioSet(m5pm1_gpio_num_t pin, m5pm1_gpio_mode_t mode,
                        uint8_t value, m5pm1_gpio_pull_t pull, m5pm1_gpio_drive_t drive);

    m5pm1_err_t gpioSetFunc(m5pm1_gpio_num_t pin, m5pm1_gpio_func_t func);
    m5pm1_err_t gpioSetMode(m5pm1_gpio_num_t pin, m5pm1_gpio_mode_t mode);
    m5pm1_err_t gpioSetOutput(m5pm1_gpio_num_t pin, uint8_t value);
    m5pm1_err_t gpioGetInput(m5pm1_gpio_num_t pin, uint8_t* value);
    m5pm1_err_t gpioSetPull(m5pm1_gpio_num_t pin, m5pm1_gpio_pull_t pull);
    m5pm1_err_t gpioSetDrive(m5pm1_gpio_num_t pin, m5pm1_gpio_drive_t drive);
    m5pm1_err_t gpioSetWakeEnable(m5pm1_gpio_num_t pin, bool enable);
    m5pm1_err_t gpioSetWakeEdge(m5pm1_gpio_num_t pin, m5pm1_gpio_wake_edge_t edge);

    /**
     * @brief Set LED_EN pin drive mode / 设置 LED_EN 引脚驱动模式
     * @param drive Drive mode: M5PM1_GPIO_DRIVE_PUSHPULL / OPENDRAIN
     * @return M5PM1_OK if successful, error code otherwise
     * @note LED_EN is a special pin (bit 5 in GPIO_DRV register)
     */
    m5pm1_err_t ledEnSetDrive(m5pm1_gpio_drive_t drive);

    /**
     * @brief Dump all pin status for debugging / 打印所有引脚状态（调试用）
     * @return M5PM1_OK if successful, error code otherwise
     */
    m5pm1_err_t dumpPinStatus();

    /**
     * @brief Read back and verify GPIO configuration / 读回并校验GPIO配置
     * @param enableLog Enable warning logs for mismatches / 启用不匹配的警告日志
     * @return M5PM1_OK if successful, error code otherwise
     * @note This function reads all GPIO registers and compares with cached values.
     *       If mismatches are found and enableLog is true, warnings will be logged.
     *       此函数读取所有GPIO寄存器并与缓存值比对。
     *       如果发现不匹配且enableLog为true，将记录警告日志。
     */
    m5pm1_err_t verifyPinConfig(bool enableLog = true);

    /**
     * @brief Get cached pin status / 获取缓存的引脚状态
     * @param pin GPIO pin number (0-4) / GPIO 引脚号
     * @param status Output: cached pin status / 输出: 缓存的引脚状态
     * @return M5PM1_OK if successful, error code otherwise
     * @note This reads from software cache, no I2C transaction
     *       此函数从软件缓存读取，无 I2C 事务
     */
    m5pm1_err_t getPinStatus(m5pm1_gpio_num_t pin, m5pm1_pin_status_t* status);

    /**
     * @brief Get cached pin status array pointer / 获取缓存引脚状态数组指针
     * @return Pointer to internal pin status array (size: 5)
     * @note Returns pointer to internal cache, do not modify externally
     *       返回内部缓存指针，请勿在外部修改
     */
    const m5pm1_pin_status_t* getPinStatusArray() const;

    // ========================
    // 电源保持功能
    // Power Hold Functions
    // ========================
    m5pm1_err_t gpioSetPowerHold(m5pm1_gpio_num_t pin, bool enable);
    m5pm1_err_t gpioGetPowerHold(m5pm1_gpio_num_t pin, bool* enable);
    m5pm1_err_t ldoSetPowerHold(bool enable);
    m5pm1_err_t ldoGetPowerHold(bool* enable);
    m5pm1_err_t dcdcSetPowerHold(bool enable);
    m5pm1_err_t dcdcGetPowerHold(bool* enable);

    // ========================
    // ADC 功能
    // ADC Functions
    // ========================
    m5pm1_err_t analogRead(m5pm1_adc_channel_t channel, uint16_t* value);
    m5pm1_err_t isAdcBusy(bool* busy);
    m5pm1_err_t disableAdc();

    // ========================
    // 温度传感器
    // Temperature Sensor
    // ========================
    m5pm1_err_t readTemperature(uint16_t* temperature);

    // ========================
    // PWM 功能
    // PWM Functions
    // ========================
    m5pm1_err_t setPwmFrequency(uint16_t frequency);
    m5pm1_err_t getPwmFrequency(uint16_t* frequency);
    m5pm1_err_t setPwmDuty(m5pm1_pwm_channel_t channel, uint8_t duty,
                           bool polarity = false, bool enable = true);
    m5pm1_err_t getPwmDuty(m5pm1_pwm_channel_t channel, uint8_t* duty,
                           bool* polarity, bool* enable);
    m5pm1_err_t setPwmDuty12bit(m5pm1_pwm_channel_t channel, uint16_t duty12,
                               bool polarity = false, bool enable = true);
    m5pm1_err_t getPwmDuty12bit(m5pm1_pwm_channel_t channel, uint16_t* duty12,
                               bool* polarity, bool* enable);
    /**
     * @brief Configure PWM in one call
     *        一次性配置 PWM 参数
     * @param channel PWM channel (0-1)
     *               PWM 通道（0-1）
     * @param enable Enable output (true=enable, false=disable)
     *              输出使能（true=启用，false=禁用）
     * @param polarity Polarity (false=normal, true=inverted)
     *                极性（false=正常，true=反相）
     * @param frequency PWM frequency in Hz (0-65535, shared by all channels)
     *                  PWM 频率（0-65535，全通道共享）
     * @param duty12 12-bit duty (0-4095)
     *               12 位占空比（0-4095）
     * @note This API warns on conflicts but still applies settings
     *       此 API 对冲突仅告警，仍会继续配置
     * @note Changing frequency affects all channels
     *       变更频率会影响所有通道
     */
    m5pm1_err_t setPwmConfig(m5pm1_pwm_channel_t channel, bool enable, bool polarity,
                            uint16_t frequency, uint16_t duty12);
    m5pm1_err_t analogWrite(m5pm1_pwm_channel_t channel, uint8_t value);

    // ========================
    // 电压读取功能
    // Voltage Reading Functions
    // ========================
    m5pm1_err_t readVref(uint16_t* mv);
    m5pm1_err_t getRefVoltage(uint16_t* mv);
    m5pm1_err_t readVbat(uint16_t* mv);
    m5pm1_err_t readVin(uint16_t* mv);
    m5pm1_err_t read5VInOut(uint16_t* mv);

    // ========================
    // 电源管理
    // Power Management
    // ========================
    m5pm1_err_t getPowerSource(m5pm1_pwr_src_t* src);

    /**
     * @brief Read wake source / 读取唤醒源
     * @param src Output: wake source bitmask / 输出: 唤醒源位掩码
     * @param cleanType Clean behavior after read / 读取后的清除行为
     *                  M5PM1_CLEAN_NONE=不清除, M5PM1_CLEAN_TRIGGERED=清除触发位, M5PM1_CLEAN_ALL=清除全部
     * @return M5PM1_OK if successful, error code otherwise
     *         成功返回 M5PM1_OK，否则返回错误码
     */
    m5pm1_err_t getWakeSource(uint8_t* src, m5pm1_clean_type_t cleanType = M5PM1_CLEAN_NONE);
    m5pm1_err_t clearWakeSource(uint8_t mask);

    m5pm1_err_t setPowerConfig(uint8_t mask, uint8_t value);
    m5pm1_err_t getPowerConfig(uint8_t* config);

    /**
     * @brief Clear power config bits / 清除电源配置位
     * @param mask Bits to clear / 要清除的位
     * @return M5PM1_OK if successful, error code otherwise
     */
    m5pm1_err_t clearPowerConfig(uint8_t mask);

    m5pm1_err_t setChargeEnable(bool enable);
    m5pm1_err_t setDcdcEnable(bool enable);
    m5pm1_err_t setLdoEnable(bool enable);
    m5pm1_err_t set5VInOutEnable(bool enable);
    m5pm1_err_t setLedControlEnable(bool enable);

    // ========================
    // 电池功能
    // Battery Functions
    // ========================
    m5pm1_err_t setBatteryLvp(uint16_t mv);

    // ========================
    // 看门狗功能
    // Watchdog Functions
    // ========================
    m5pm1_err_t wdtSet(uint8_t timeout_sec);
    m5pm1_err_t wdtFeed();
    m5pm1_err_t wdtGetCount(uint8_t* count);

    // ========================
    // 定时器功能
    // Timer Functions
    // ========================
    m5pm1_err_t timerSet(uint32_t seconds, m5pm1_tim_action_t action);
    m5pm1_err_t timerClear();

    // ========================
    // 按钮功能
    // Button Functions
    // ========================
    m5pm1_err_t btnSetConfig(m5pm1_btn_type_t type, m5pm1_btn_delay_t delay);
    m5pm1_err_t btnGetState(bool* pressed);
    m5pm1_err_t btnGetFlag(bool* wasPressed);
    m5pm1_err_t setSingleResetDisable(bool disable);
    m5pm1_err_t getSingleResetDisable(bool* disabled);
    m5pm1_err_t setDoubleOffDisable(bool disable);
    m5pm1_err_t getDoubleOffDisable(bool* disabled);

    // ========================
    // 中断功能
    // IRQ Functions
    // ========================
    /**
     * @brief Read GPIO interrupt status / 读取 GPIO 中断状态
     * @param status Output: status bitmask / 输出: 状态位掩码
     * @param cleanType Clean behavior after read / 读取后的清除行为
     *                  M5PM1_CLEAN_NONE=不清除, M5PM1_CLEAN_TRIGGERED=清除触发位, M5PM1_CLEAN_ALL=清除全部
     * @return 成功返回 M5PM1_OK，否则返回错误码
     *         Return M5PM1_OK on success, error code otherwise
     */
    m5pm1_err_t irqGetGpioStatus(uint8_t* status, m5pm1_clean_type_t cleanType = M5PM1_CLEAN_NONE);
    m5pm1_err_t irqClearGpio(uint8_t mask);

    /**
     * @brief Read system interrupt status / 读取系统中断状态
     * @param status Output: status bitmask / 输出: 状态位掩码
     * @param cleanType Clean behavior after read / 读取后的清除行为
     *                  M5PM1_CLEAN_NONE=不清除, M5PM1_CLEAN_TRIGGERED=清除触发位, M5PM1_CLEAN_ALL=清除全部
     * @return 成功返回 M5PM1_OK，否则返回错误码
     *         Return M5PM1_OK on success, error code otherwise
     */
    m5pm1_err_t irqGetSysStatus(uint8_t* status, m5pm1_clean_type_t cleanType = M5PM1_CLEAN_NONE);
    m5pm1_err_t irqClearSys(uint8_t mask);

    /**
     * @brief Read button interrupt status / 读取按钮中断状态
     * @param status Output: status bitmask / 输出: 状态位掩码
     * @param cleanType Clean behavior after read / 读取后的清除行为
     *                  M5PM1_CLEAN_NONE=不清除, M5PM1_CLEAN_TRIGGERED=清除触发位, M5PM1_CLEAN_ALL=清除全部
     * @return 成功返回 M5PM1_OK，否则返回错误码
     *         Return M5PM1_OK on success, error code otherwise
     */
    m5pm1_err_t irqGetBtnStatus(uint8_t* status, m5pm1_clean_type_t cleanType = M5PM1_CLEAN_NONE);
    m5pm1_err_t irqClearBtn(uint8_t mask);

    /**
     * @brief Set single GPIO pin interrupt mask / 设置单个 GPIO 引脚中断屏蔽
     * @param pin GPIO pin number (0-4) / GPIO 引脚号
     * @param mask true=mask(disable), false=unmask(enable) / true=屏蔽, false=不屏蔽
     * @return 成功返回 M5PM1_OK，否则返回错误码
     *         Return M5PM1_OK on success, error code otherwise
     */
    m5pm1_err_t irqSetGpioMask(m5pm1_gpio_num_t pin, bool mask);
    m5pm1_err_t irqGetGpioMask(m5pm1_gpio_num_t pin, bool* mask);

    /**
     * @brief Set all GPIO interrupt mask at once / 一次性设置所有 GPIO 中断屏蔽
     * @param mask Bitmask for all GPIO pins / 所有 GPIO 引脚的位掩码
     * @return 成功返回 M5PM1_OK，否则返回错误码
     *         Return M5PM1_OK on success, error code otherwise
     */
    m5pm1_err_t irqSetGpioMaskAll(uint8_t mask);
    m5pm1_err_t irqGetGpioMaskAll(uint8_t* mask);

    /**
     * @brief Set single system event interrupt mask / 设置单个系统事件中断屏蔽
     * @param event Event bit (0-5): 0=5VIN_IN, 1=5VIN_OUT, 2=5VINOUT_IN, 3=5VINOUT_OUT, 4=BAT_IN, 5=BAT_OUT
     * @param mask true=mask(disable), false=unmask(enable)
     * @return 成功返回 M5PM1_OK，否则返回错误码
     *         Return M5PM1_OK on success, error code otherwise
     */
    m5pm1_err_t irqSetSysMask(uint8_t event, bool mask);
    m5pm1_err_t irqGetSysMask(uint8_t event, bool* mask);

    /**
     * @brief Set all system interrupt mask at once / 一次性设置所有系统中断屏蔽
     * @param mask Bitmask for all system events / 所有系统事件的位掩码
     * @return 成功返回 M5PM1_OK，否则返回错误码
     *         Return M5PM1_OK on success, error code otherwise
     */
    m5pm1_err_t irqSetSysMaskAll(uint8_t mask);
    m5pm1_err_t irqGetSysMaskAll(uint8_t* mask);

    /**
     * @brief Set single button event interrupt mask / 设置单个按钮事件中断屏蔽
     * @param type Button event type: M5PM1_BTN_IRQ_CLICK / WAKEUP / DOUBLE
     * @param mask true=mask(disable), false=unmask(enable)
     * @return 成功返回 M5PM1_OK，否则返回错误码
     *         Return M5PM1_OK on success, error code otherwise
     */
    m5pm1_err_t irqSetBtnMask(m5pm1_btn_irq_t type, bool mask);
    m5pm1_err_t irqGetBtnMask(m5pm1_btn_irq_t type, bool* mask);

    /**
     * @brief Set all button interrupt mask at once / 一次性设置所有按钮中断屏蔽
     * @param mask Bitmask for all button events / 所有按钮事件的位掩码
     * @return 成功返回 M5PM1_OK，否则返回错误码
     *         Return M5PM1_OK on success, error code otherwise
     */
    m5pm1_err_t irqSetBtnMaskAll(uint8_t mask);
    m5pm1_err_t irqGetBtnMaskAll(uint8_t* mask);

    // ========================
    // 系统命令
    // System Commands
    // ========================
    m5pm1_err_t sysCmd(m5pm1_sys_cmd_t cmd);
    m5pm1_err_t shutdown();
    m5pm1_err_t reboot();
    m5pm1_err_t enterDownloadMode();

    /**
     * @brief Set download mode lock / 设置下载模式锁
     * @param lock true=lock(disable download), false=unlock(enable download)
     *             true=锁定(禁止下载), false=解锁(允许下载)
     * @return 成功返回 M5PM1_OK，否则返回错误码
     *         Return M5PM1_OK on success, error code otherwise
     * @note This controls bit 7 (DL_LOCK) in BTN_CFG register
     */
    m5pm1_err_t setDownloadLock(bool lock);
    m5pm1_err_t getDownloadLock(bool* lock);

    // ========================
    // NeoPixel 功能
    // NeoPixel Functions
    // ========================
    m5pm1_err_t setLedCount(uint8_t count);
    m5pm1_err_t setLedColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
    m5pm1_err_t setLedColor(uint8_t index, m5pm1_rgb_t color);
    m5pm1_err_t refreshLeds();
    m5pm1_err_t disableLeds();

    /**
     * @brief Configure NeoPixel in one call / 一键配置 NeoPixel
     * @param count LED count (1-32) / LED 数量
     * @param rgb565Data RGB565 format color data array / RGB565 格式颜色数据数组
     * @param refresh Whether to refresh immediately / 是否立即刷新
     * @return true if successful
     */
    m5pm1_err_t setLeds(const m5pm1_rgb_t* colors, uint8_t arraySize, uint8_t count,
                        bool autoRefresh = true);

    // ========================
    // AW8737A 脉冲功能
    // AW8737A Pulse Functions
    // ========================
    m5pm1_err_t setAw8737aPulse(m5pm1_gpio_num_t pin, m5pm1_aw8737a_pulse_t num,
                               m5pm1_aw8737a_refresh_t refresh = M5PM1_AW8737A_REFRESH_NOW);
    m5pm1_err_t refreshAw8737aPulse();

    // ========================
    // RTC RAM 功能
    // RTC RAM Functions
    // ========================
    m5pm1_err_t writeRtcRAM(uint8_t offset, const uint8_t* data, uint8_t len);
    m5pm1_err_t readRtcRAM(uint8_t offset, uint8_t* data, uint8_t len);

    // ========================
    // I2C 配置
    // I2C Configuration
    // ========================
    m5pm1_err_t setI2cConfig(uint8_t sleepTime,
                             m5pm1_i2c_speed_t speed = M5PM1_I2C_SPEED_100K);
    m5pm1_err_t switchI2cSpeed(m5pm1_i2c_speed_t speed);
    m5pm1_err_t getI2cSpeed(m5pm1_i2c_speed_t* speed);
    m5pm1_err_t setI2cSleepTime(uint8_t seconds);
    m5pm1_err_t getI2cSleepTime(uint8_t* seconds);

    // ========================
    // 自动唤醒功能
    // Auto Wake Feature
    // ========================
    /**
     * @brief Enable/disable automatic wake signal before I2C operations
     *        启用/禁用 I2C 操作前的自动唤醒信号
     * @param enable true to enable auto-wake, false to disable
     * @note When PM1 enters sleep mode (I2C sleep timeout), it needs a
     *       START signal on SDA to wake up. This feature automatically
     *       sends the wake signal when needed.
     *       当 PM1 进入睡眠模式（I2C 睡眠超时）后，需要在 SDA 上发送
     *       START 信号来唤醒。此功能会在需要时自动发送唤醒信号。
     * @note Even without enabling this option, communication will likely
     *       succeed in most cases, as the first I2C transaction itself
     *       can wake the device. Enable this for guaranteed reliability.
     *       即使不启用此选项，通讯在大多数情况下也能成功，因为第一次
     *       I2C 传输本身就能唤醒设备。启用此选项可确保可靠性。
     */
    void setAutoWakeEnable(bool enable);

    /**
     * @brief Check if auto wake is enabled / 检查自动唤醒是否启用
     * @return true if enabled
     */
    bool isAutoWakeEnabled() const;

    /**
     * @brief Manually send wake signal to PM1 / 手动发送唤醒信号到 PM1
     * @return M5PM1_OK if successful, error code otherwise
     */
    m5pm1_err_t sendWakeSignal();

    // ========================
    // 状态快照功能
    // State Snapshot Functions
    // ========================
    void setAutoSnapshot(bool enable);
    bool isAutoSnapshotEnabled() const;
    m5pm1_err_t updateSnapshot();

    // ========================
    // 快照验证
    // Snapshot Verification
    // ========================
    m5pm1_snapshot_verify_t verifySnapshot();

    // ========================
    // 配置验证
    // Configuration Validation
    // ========================
    m5pm1_validation_t validateConfig(uint8_t pin, m5pm1_config_type_t configType, bool enable = true);

    // ========================
    // 缓存状态查询函数
    // Cached State Query Functions
    // ========================
    m5pm1_err_t getCachedPwmFrequency(uint16_t* frequency);
    m5pm1_err_t getCachedPwmState(m5pm1_pwm_channel_t channel, uint16_t* duty12, bool* enable, bool* polarity);
    m5pm1_err_t getCachedAdcState(m5pm1_adc_channel_t* channel, bool* busy, uint16_t* lastValue);
    m5pm1_err_t getCachedPowerConfig(uint8_t* pwrCfg, uint8_t* holdCfg);
    m5pm1_err_t getCachedButtonConfig(uint8_t* cfg1, uint8_t* cfg2);
    m5pm1_err_t getCachedIrqMasks(uint8_t* mask1, uint8_t* mask2, uint8_t* mask3);
    m5pm1_err_t getCachedIrqStatus(uint8_t* status1, uint8_t* status2, uint8_t* status3);

private:
    // Device state
    // 设备状态
    uint8_t _addr;
    bool _initialized;
    bool _autoWakeEnabled;
    bool _autoSnapshot;
    uint8_t _i2cSleepTime;
    uint32_t _requestedSpeed;
    struct {
        uint8_t sleepTime;
        bool speed400k;
    } _i2cConfig;
    bool _i2cConfigValid;
    uint32_t _lastCommTime;

    // PWM state cache
    // PWM 状态缓存
    struct {
        uint16_t duty12;
        bool enabled;
        bool polarity;
    } _pwmStates[M5PM1_MAX_PWM_CHANNELS];
    uint16_t _pwmFrequency;
    bool _pwmStatesValid;

    // ADC state cache
    // ADC 状态缓存
    struct {
        uint8_t channel;
        bool busy;
        uint16_t lastValue;
    } _adcState;
    bool _adcStateValid;

    // Power config cache
    // 电源配置缓存
    uint8_t _powerCfg;
    uint8_t _holdCfg;
    bool _powerConfigValid;

    // Button config cache
    // 按钮配置缓存
    uint8_t _btnCfg1;
    uint8_t _btnCfg2;
    bool _btnConfigValid;

    // Button flag cache (for BTN_FLAG read-clear handling)
    // 按钮标志缓存（用于处理 BTN_FLAG 读后清除问题）
    bool _btnFlagCache;

    // IRQ cache
    // IRQ 缓存
    uint8_t _irqMask1;
    uint8_t _irqMask2;
    uint8_t _irqMask3;
    bool _irqMaskValid;
    uint8_t _irqStatus1;
    uint8_t _irqStatus2;
    uint8_t _irqStatus3;
    bool _irqStatusValid;

    // Neo 配置缓存
    // Neo config cache
    uint8_t _neoCfg;
    bool _neoConfigValid;

    // Pin status cache
    // Pin 状态缓存
    m5pm1_pin_status_t _pinStatus[M5PM1_MAX_GPIO_PINS];  // Cached status for GPIO0-4
    bool _cacheValid;  // Cache validity flag / 缓存有效性标志

#ifdef ARDUINO
    TwoWire *_wire;
    int8_t _sda;
    int8_t _scl;
#else
    // I2C driver type selection
    // I2C 驱动类型选择
    m5pm1_i2c_driver_t _i2cDriverType;

    // I2C handles
    // I2C 句柄
    i2c_master_bus_handle_t _i2c_master_bus;
    i2c_master_dev_handle_t _i2c_master_dev;
    i2c_bus_handle_t _i2c_bus;
    i2c_bus_device_handle_t _i2c_device;

    // I2C management flags
    // I2C 管理标志
    bool _busExternal;

    // Self-created bus pins
    // 自创建总线引脚
    int _sda;
    int _scl;
    i2c_port_t _port;
#endif

    // Internal helper functions
    // 内部辅助函数
    bool _writeReg(uint8_t reg, uint8_t value);
    bool _writeReg16(uint8_t reg, uint16_t value);
    bool _readReg(uint8_t reg, uint8_t* value);
    bool _readReg16(uint8_t reg, uint16_t* value);
    bool _writeBytes(uint8_t reg, const uint8_t* data, uint8_t len);
    bool _readBytes(uint8_t reg, uint8_t* data, uint8_t len);

    bool _isValidPin(uint8_t pin);
    bool _isAdcPin(uint8_t pin);
    bool _isPwmPin(uint8_t pin);
    bool _isNeoPin(uint8_t pin);
    bool _hasActiveAdc(uint8_t pin);
    bool _hasActivePwm(uint8_t pin);
    bool _hasActiveIrq(uint8_t pin);
    bool _hasActiveWake(uint8_t pin);
    bool _hasActiveNeo(uint8_t pin);
    bool _isValidI2cFrequency(uint32_t speed);
    void _checkAutoWake();
    bool _initDevice();
    void _clearPinStates();
    void _clearPwmStates();
    void _clearAdcState();
    void _clearPowerConfig();
    void _clearButtonConfig();
    void _clearIrqMasks();
    void _clearIrqStatus();
    void _clearAll();
    bool _snapshotPinStates();
    bool _snapshotPwmStates();
    bool _snapshotAdcState();
    bool _snapshotPowerConfig();
    bool _snapshotButtonConfig();
    bool _snapshotIrqMasks();
    bool _snapshotIrqStatus();
    bool _snapshotAll();
    void _autoSnapshotUpdate(uint16_t domains);
    bool _snapshotI2cConfig();
    void _clearI2cConfig();
    bool _snapshotNeoConfig();
    void _clearNeoConfig();

    // Cache management functions
    // 缓存管理函数
    void _initPinCache();
    void _updatePinCache(m5pm1_gpio_num_t pin);

    // Button status internal function
    // 按钮状态内部函数
    bool _readBtnStatus(uint8_t* rawValue);
};

#endif // _M5PM1_H_
