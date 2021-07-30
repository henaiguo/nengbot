///////////////////////////////////////////////////////////
/// @file   pca_9685.h
/// @brief  Servo control based on pca9685 through i2c
/// @author henaiguo
/// Copyright (C) 2021- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////
#ifndef PCA9685_H
#define PCA9685_H

#include <common_library/types/result.h>
#include <common_library/io/i2c.h>

namespace servo_controller {
///////////////////////////////////////////////////////////
/// @class PCA9685
/// @brief Servo control based on pca9685 through i2c
/// @note
///////////////////////////////////////////////////////////
class PCA9685
{
public:
    ///////////////////////////////////////////////////////////
    /// @brief  Default constructor
    /// @return None
    /// @note
    ///////////////////////////////////////////////////////////
    PCA9685();

    ///////////////////////////////////////////////////////////
    /// @brief  Destructor
    /// @return None
    /// @note
    ///////////////////////////////////////////////////////////
    virtual ~PCA9685();

    ///////////////////////////////////////////////////////////
    /// @brief  Initialize
    /// @return common_library::types::Result
    /// @note
    ///////////////////////////////////////////////////////////
    common_library::types::Result Initialize();

    ///////////////////////////////////////////////////////////
    /// @brief  Finalize
    /// @return None
    /// @note
    ///////////////////////////////////////////////////////////
    void Finalize();

    ///////////////////////////////////////////////////////////
    /// @brief  Send PWM frequency through i2c to pca9685
    /// @param[in]  _frequency 40Hz to 1000Hz using internal 25MHz oscillator
    /// @return common_library::types::Result
    /// @note
    ///////////////////////////////////////////////////////////
    common_library::types::Result SetPWMFrequency(uint8_t _frequency);

    ///////////////////////////////////////////////////////////
    /// @brief  Send PWM through i2c to pca9685
    /// @param[in]  _channel Channel 1-16
    /// @param[in]  __value 0-4095 value for PWM
    /// @return common_library::types::Result
    /// @note
    ///////////////////////////////////////////////////////////
    common_library::types::Result SetPWM(uint8_t _channel, int _value);

    ///////////////////////////////////////////////////////////
    /// @brief  Send PWM through i2c to pca9685
    /// @param[in]  _channel Channel 1-16
    /// @param[in]  _on 0-4095 value to turn on the pulse
    /// @param[in]  _off 0-4095 value to turn off the pulse
    /// @return common_library::types::Result
    /// @note
    ///////////////////////////////////////////////////////////
    common_library::types::Result SetPWM(uint8_t _channel, int _on, int _off);

    ///////////////////////////////////////////////////////////
    /// @brief  Get current PWM value
    /// @param[in]  _channel Channel 1-16
    /// @param[out]  _value PWM value
    /// @return int common_library::types::Result
    /// @note
    ///////////////////////////////////////////////////////////
    common_library::types::Result GetPWM(uint8_t _channel, int& _value);

private:
    /// i2c driver
    common_library::io::I2C m_i2c;

    ///////////////////////////////////////////////////////////
    /// @brief  reset lcd1602
    /// @return common_library::types::Result
    /// @note
    ///////////////////////////////////////////////////////////
    common_library::types::Result reset();
};
} // namespace servo_controller

#endif // PCA9685_H
