/**
 ******************************************************************************
 * @file        : pca9685.hpp
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

#pragma once

#include "driver/i2c_master.h"

class PCA9685 {
   public:
    /**
     * @brief Construct a new PCA9685 object
     * @param handle : I2C master device handle
     * @param frequency : PWM frequency
     * @param invert : Invert output
     * @param out_drv_mode : Output driver mode (see datasheet for details)
     */
    PCA9685(i2c_master_dev_handle_t handle,
            i2c_master_dev_handle_t reset_handle = nullptr,
            uint32_t frequency = 200,
            bool invert = true,
            uint8_t out_drv_mode = 0);

    /**
     * @brief Create a new I2C handle for the PCA9685 with default settings
     * @param i2c_port : I2C port number
     * @param sda : SDA pin
     * @param scl : SCL pin
     * @param dev_addr : Device address
     * @param dev_handle : Pointer to the new device handle
     * @return esp_err_t : ESP_OK on success
     */
    static esp_err_t NewI2cHandle(i2c_port_num_t i2c_port,
                                  gpio_num_t sda,
                                  gpio_num_t scl,
                                  uint16_t dev_addr,
                                  i2c_master_dev_handle_t* dev_handle);

    /**
     * @brief Reset the PCA9685
     */
    void Reset();

    /**
     * @brief Set the PWM frequency in sleep mode
     * @param frequency : PWM frequency
     */
    void SetPwmFrequency(uint32_t frequency);

    /**
     * @brief Set the PWM for a specific LED
     * @param led : LED number
     * @param on : On time (between 0 and 4096)
     * @param off : Off time (between 0 and 4096)
     * @note The LED will not be updated until Refresh() is called
     */
    void SetPwm(uint8_t led, uint16_t on, uint16_t off);

    /**
     * @brief Set the PWM for a specific LED using a duty cycle
     * @param led : LED number
     * @param duty_cycle : Duty cycle (between 0.0 and 1.0)
     * @note The LEDs will not be updated until Refresh() is called
     */
    void SetPwm(uint8_t led, float duty_cycle);

    /**
     * @brief Switch on a specific LED
     * @param led : LED number
     * @note The LEDs will not be updated until Refresh() is called
     */
    void SwitchOn(uint8_t led);

    /**
     * @brief Switch off a specific LED
     * @param led : LED number
     * @note The LEDs will not be updated until Refresh() is called
     */
    void SwitchOff(uint8_t led);

    /**
     * @brief Set the PWM for all LEDs
     * @param on : On time (between 0 and 4096)
     * @param off : Off time (between 0 and 4096)
     * @note The LEDs will not be updated until Refresh() is called
     */
    void SetAllPwm(uint16_t on, uint16_t off);

    /**
     * @brief Set the PWM for all LEDs using a duty cycle
     * @param duty_cycle : Duty cycle (between 0.0 and 1.0)
     * @note The LEDs will not be updated until Refresh() is called
     */
    void SetAllPwm(float duty_cycle);

    /**
     * @brief Switch all LEDs on
     * @note The LEDs will not be updated until Refresh() is called
     */
    void SwitchAllOn();

    /**
     * @brief Switch all LEDs off
     * @note The LEDs will not be updated until Refresh() is called
     */
    void SwitchAllOff();

    /**
     * @brief Refresh the LEDs
     * @note This will update the LEDs with the values set by the previous calls to SetPwm,
     * SetAllPwm, SwitchOn, SwitchOff, SwitchAllOn, SwitchAllOff
     */
    void Refresh();

   private:
    i2c_master_dev_handle_t handle_;
    i2c_master_dev_handle_t reset_handle_;
    uint8_t address_;
    uint8_t buffer_[16 * 4 + 1];
};
