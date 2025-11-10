/**
 ******************************************************************************
 * @file        : pca9685.cpp
 * @brief       : Driver for the PCA9685 PWM controller
 * @author      : Jacques Supcik <jacques@supcik.net>
 * @date        : 3 April 2024
 ******************************************************************************
 * @copyright   : Copyright (c) 2024 Jacques Supcik
 * @attention   : SPDX-License-Identifier: MIT
 ******************************************************************************
 * @details
 *
 ******************************************************************************
 */

#include "pca9685.hpp"

#include "esp_err.h"
#include "esp_log.h"
#include "math.h"

static const char* kTag = "PCA9685";
static const int kNumberOfLeds = 16;
static const int kI2CtimeoutMs = 100;

PCA9685::PCA9685(i2c_master_dev_handle_t handle,
                 i2c_master_dev_handle_t reset_handle,
                 uint32_t frequency,
                 bool invert,
                 uint8_t out_drv_mode) {
    this->handle_ = handle;
    this->reset_handle_ = reset_handle;
    SwitchAllOff();

    SetPwmFrequency(frequency);

    ESP_LOGD(kTag, "Configuring PCA9685");
    uint8_t buffer[3];
    buffer[0] = 0x00;
    buffer[1] = (1 << 5);  // MDODE 1 : Set Auto Increment
    // MODE2
    buffer[2] = ((invert ? 1 : 0) << 4) | ((out_drv_mode & 0x1) << 2);
    ESP_ERROR_CHECK(i2c_master_transmit(this->handle_, buffer, 3, kI2CtimeoutMs));
}

esp_err_t PCA9685::NewI2cHandle(i2c_port_num_t i2c_port,
                                gpio_num_t sda,
                                gpio_num_t scl,
                                uint16_t dev_addr,
                                i2c_master_dev_handle_t* dev_handle) {
    i2c_master_bus_config_t i2c_mst_config = {};
    i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_mst_config.i2c_port = i2c_port;
    i2c_mst_config.scl_io_num = scl;
    i2c_mst_config.sda_io_num = sda;
    i2c_mst_config.glitch_ignore_cnt = 7;
    i2c_mst_config.flags.enable_internal_pullup = false;

    i2c_master_bus_handle_t bus_handle;
    esp_err_t err = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    if (err != ESP_OK) {
        return err;
    }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = dev_addr;
    dev_cfg.scl_speed_hz = 100000;

    return i2c_master_bus_add_device(bus_handle, &dev_cfg, dev_handle);
}

void PCA9685::Reset() {
    if (reset_handle_ == nullptr) {
        ESP_LOGW(kTag, "Reset handle not set, cannot reset PCA9685");
        return;
    }
    ESP_LOGD(kTag, "Resetting PCA9685");
    uint8_t buffer[1];
    buffer[0] = 0x06;
    ESP_ERROR_CHECK(i2c_master_transmit(this->handle_, buffer, 1, kI2CtimeoutMs));
}

void PCA9685::SetPwmFrequency(uint32_t frequency) {
    uint8_t buffer[2];
    uint32_t prescale = roundf(25000000.0f / (4096.0f * frequency)) - 1;
    ESP_LOGD(kTag, "Setting prescale to %lu", prescale);
    buffer[0] = 0xFE;
    buffer[1] = prescale;
    ESP_ERROR_CHECK(i2c_master_transmit(this->handle_, buffer, 2, kI2CtimeoutMs));
}

void PCA9685::SetPwm(uint8_t led, uint16_t on, uint16_t off) {
    if (on > 4096) on = 4096;
    if (off > 4096) off = 4096;
    ESP_LOGD(kTag, "Setting PWM for LED %d to (%d, %d)", led, on, off);
    buffer_[led * 4 + 1] = on & 0xFF;
    buffer_[led * 4 + 2] = on >> 8;
    buffer_[led * 4 + 3] = off & 0xFF;
    buffer_[led * 4 + 4] = off >> 8;
}

void PCA9685::SetPwm(uint8_t led, float duty_cycle) {
    if (duty_cycle <= 0.0f) {
        SwitchOff(led);
    } else if (duty_cycle >= 1.0f) {
        SwitchOn(led);
    } else {
        SetPwm(led, 0, roundf(4096.0f * duty_cycle));
    }
}

void PCA9685::SwitchOn(uint8_t led) { SetPwm(led, 4096, 0); }
void PCA9685::SwitchOff(uint8_t led) { SetPwm(led, 0, 4096); }

void PCA9685::SetAllPwm(uint16_t on, uint16_t off) {
    for (int i = 0; i < kNumberOfLeds; i++) {
        SetPwm(i, on, off);
    }
}

void PCA9685::SetAllPwm(float duty_cycle) {
    for (int i = 0; i < kNumberOfLeds; i++) {
        SetPwm(i, duty_cycle);
    }
}

void PCA9685::SwitchAllOn() { SetAllPwm(4096, 0); }
void PCA9685::SwitchAllOff() { SetAllPwm(0, 4096); }

void PCA9685::Refresh() {
    ESP_LOGD(kTag, "Refreshing PCA9685");
    buffer_[0] = 0x06;
    ESP_ERROR_CHECK(i2c_master_transmit(this->handle_, buffer_, sizeof(buffer_), kI2CtimeoutMs));
}
