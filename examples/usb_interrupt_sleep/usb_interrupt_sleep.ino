/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

#include <Arduino.h>
#include <M5PM1.h>

/*
 * USB拔插中断/关机与唤醒示例。
 * USB Plug/Unplug Interrupt/Shutdown & Wake Demo.
 *
 * 功能：
 * 1. 启动时打印唤醒源。
 * 2. 运行时监听 USB (5VIN) 的拔插事件。
 * 3. 当检测到 USB 拔出 (5VIN Remove) 时，倒计时5秒后关机。
 * 4. 关机前设置10秒定时器唤醒，10秒后自动开机。
 *
 * Features:
 * 1. Print wake source on startup.
 * 2. Listen for USB (5VIN) plug/unplug events.
 * 3. When USB unplug (5VIN Remove) detected, shutdown after 5s countdown.
 * 4. Set 10s timer wake up before shutdown, auto wake up after 10s.
 */

M5PM1 pm1;

#define LOGI(fmt, ...) Serial.printf("[PM1][I] " fmt "\r\n", ##__VA_ARGS__)
#define LOGW(fmt, ...) Serial.printf("[PM1][W] " fmt "\r\n", ##__VA_ARGS__)
#define LOGE(fmt, ...) Serial.printf("[PM1][E] " fmt "\r\n", ##__VA_ARGS__)

#ifndef PM1_I2C_SDA
#define PM1_I2C_SDA 47
#endif
#ifndef PM1_I2C_SCL
#define PM1_I2C_SCL 48
#endif
#ifndef PM1_I2C_FREQ
#define PM1_I2C_FREQ M5PM1_I2C_FREQ_100K
#endif
#ifndef PM1_ESP_IRQ_GPIO
#define PM1_ESP_IRQ_GPIO 13
#endif

volatile bool irqFlag = false;
void IRAM_ATTR pm1_irq_handler()
{
    irqFlag = true;
}

static const uint32_t WAKE_TIMER_SEC = 10;

static void printDivider()
{
    Serial.println("--------------------------------------------------");
}

static void printWakeSource(uint8_t src)
{
    LOGI("Wake source mask: 0x%02X", src);
    if (src & M5PM1_WAKE_SRC_TIM) LOGI("- TIMER");
    if (src & M5PM1_WAKE_SRC_VIN) LOGI("- VIN");
    if (src & M5PM1_WAKE_SRC_PWRBTN) LOGI("- PWR_BTN");
    if (src & M5PM1_WAKE_SRC_RSTBTN) LOGI("- RST_BTN");
    if (src & M5PM1_WAKE_SRC_CMD_RST) LOGI("- CMD_RESET");
    if (src & M5PM1_WAKE_SRC_EXT_WAKE) LOGI("- EXT_WAKE");
    if (src & M5PM1_WAKE_SRC_5VINOUT) LOGI("- 5VINOUT");
}

static void enterSleep()
{
    LOGW("Prepare for shutdown...");

    // 先配置10s的定时开机
    // Configure 10s timer wake up first
    pm1.timerSet(WAKE_TIMER_SEC, M5PM1_TIM_ACTION_POWERON);

    // 等待5s关机
    // Wait 5s before shutdown
    for (int i = 5; i > 0; i--) {
        LOGW("Shutdown in %d s...", i);
        delay(1000);
    }

    // 关闭LED_EN灯显（将默认电平配置为低电平）
    // Turn off LED_EN indicator (by setting default level to LOW)
    pm1.setLedEnLevel(false);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    LOGW("Shutdown now. Wake by %us timer", WAKE_TIMER_SEC);
    pm1.shutdown();
}

void setup()
{
    Serial.begin(115200);
    delay(200);
    printDivider();
    LOGI("USB Interrupt + Sleep demo start");

    m5pm1_err_t err = pm1.begin(&Wire, M5PM1_DEFAULT_ADDR, PM1_I2C_SDA, PM1_I2C_SCL, PM1_I2C_FREQ);
    if (err != M5PM1_OK) {
        LOGE("PM1 begin failed: %d", err);
        while (true) {
            delay(1000);
        }
    }

    // 读取并清除一次唤醒来源，便于调试本次启动原因。
    // Read and clear wake source once for debugging this boot reason.
    uint8_t wakeSrc = 0;
    if (pm1.getWakeSource(&wakeSrc, M5PM1_CLEAN_ONCE) == M5PM1_OK) {
        printWakeSource(wakeSrc);
    }

    // 清空timer设置，避免误触发。
    // Clear timer settings to avoid mis-trigger.
    pm1.timerClear();

    // 设置ESP32的中断引脚，用于接收PM1的IRQ信号。
    // Setup ESP32 interrupt pin to receive IRQ signal from PM1.
    pinMode(PM1_ESP_IRQ_GPIO, INPUT_PULLUP);
    attachInterrupt(PM1_ESP_IRQ_GPIO, pm1_irq_handler, FALLING);

    // 屏蔽所有GPIO和按钮中断
    // Mask all GPIO and Button IRQs (disable them)
    pm1.irqSetGpioMaskAll(M5PM1_IRQ_MASK_ENABLE);
    pm1.irqSetBtnMaskAll(M5PM1_IRQ_MASK_ENABLE);

    // 配置系统中断：只启用 VIN 移除中断（也可以全部开启）
    // Configure System IRQs: Enable VIN remove interrupt
    pm1.irqSetSysMaskAll(M5PM1_IRQ_MASK_ENABLE);                           // 先全部屏蔽
    pm1.irqSetSysMask(M5PM1_IRQ_SYS_5VIN_REMOVE, M5PM1_IRQ_MASK_DISABLE);  // 开启5VIN移除
    // pm1.irqSetSysMask(M5PM1_IRQ_SYS_5VIN_INSERT, M5PM1_IRQ_MASK_DISABLE); // 可选开启插入

    // 注意：有些板子 TypeC 可能连接到 5VINOUT
    pm1.irqSetSysMask(M5PM1_IRQ_SYS_5VINOUT_REMOVE, M5PM1_IRQ_MASK_DISABLE);

    printDivider();
    LOGI("Waiting for USB unplug event...");
}

void loop()
{
    if (irqFlag) {
        irqFlag = false;

        uint8_t sysIrq = 0;
        if (pm1.irqGetSysStatus(&sysIrq, M5PM1_CLEAN_ONCE) == M5PM1_OK) {
            if (sysIrq != M5PM1_IRQ_SYS_NONE) {
                LOGI("System IRQ status: 0x%02X", sysIrq);

                // 检测 5VIN 移除
                if (sysIrq & M5PM1_IRQ_SYS_5VIN_REMOVE) {
                    LOGI("Event: 5VIN Removed");
                    enterSleep();
                }

                // 检测 5VINOUT 移除 (以防万一)
                if (sysIrq & M5PM1_IRQ_SYS_5VINOUT_REMOVE) {
                    LOGI("Event: 5VINOUT Removed");
                    enterSleep();
                }

                if (sysIrq & M5PM1_IRQ_SYS_5VIN_INSERT) {
                    LOGI("Event: 5VIN Inserted");
                }
            }
        }

        // 同时也清除任何可能的误触发（虽然我们mask了btn和gpio）
        uint8_t dummy;
        pm1.irqGetGpioStatus(&dummy, M5PM1_CLEAN_ALL);
        pm1.irqGetBtnStatus(&dummy, M5PM1_CLEAN_ALL);

    } else if (digitalRead(PM1_ESP_IRQ_GPIO) == LOW) {
        // 存在未处理的中断信号，读取以清除（但忽略内容）
        // Unhandled IRQ signal exists, read to clear (but ignore content)
        uint8_t dummy;
        pm1.irqGetGpioStatus(&dummy, M5PM1_CLEAN_ALL);
        pm1.irqGetBtnStatus(&dummy, M5PM1_CLEAN_ALL);
        pm1.irqGetSysStatus(&dummy, M5PM1_CLEAN_ALL);
    }

    delay(10);
}
