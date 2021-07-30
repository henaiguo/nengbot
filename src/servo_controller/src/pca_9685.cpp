///////////////////////////////////////////////////////////
/// @file   pca_9685.cpp
/// @brief  Servo control based on pca9685 through i2c
/// @author henaiguo
/// Copyright (C) 2021- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////
#include <servo_controller/pca_9685.h>

using common_library::types::Result;
using common_library::io::I2C;

namespace servo_controller {

#define MODE1 0x00
#define MODE2 0x01
#define SUBADR1 0x02      // enable sub address 1 support
#define SUBADR2 0x03      // enable sub address 2 support
#define SUBADR3 0x04      // enable sub address 2 support
#define PRESCAL 0xFE
#define CHANNEL_ON_L  0x06
#define CHANNEL_ON_H 0x07
#define CHANNEL_OFF_L 0x08
#define CHANNEL_OFF_H 0x09
#define ALL_CHANNELS_ON_L 0xFA
#define ALL_CHANNELS_ON_H 0xFB
#define ALL_CHANNELS_OFF_L 0xFC
#define ALL_CHANNELS_OFF_H 0xFD
#define RESTART 0x80
#define SLEEP 0x10      // enable low power mode
#define ALLCALL 0x01
#define INVRT 0x10      // invert the output control logic
#define OUTDRV 0x04

///////////////////////////////////////////////////////////
/// @brief  Default constructor
/// @return None
/// @note
///////////////////////////////////////////////////////////
PCA9685::PCA9685()
{
    // None
}

///////////////////////////////////////////////////////////
/// @brief  Destructor
/// @return None
/// @note
///////////////////////////////////////////////////////////
PCA9685::~PCA9685()
{
    Finalize();
}

///////////////////////////////////////////////////////////
/// @brief  Initialize
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result PCA9685::Initialize()
{

}

///////////////////////////////////////////////////////////
/// @brief  Finalize
/// @return None
/// @note
///////////////////////////////////////////////////////////
void PCA9685::Finalize()
{

}

///////////////////////////////////////////////////////////
/// @brief  Send PWM frequency through i2c to pca9685
/// @param[in]  _frequency 40Hz to 1000Hz using internal 25MHz oscillator
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result PCA9685::SetPWMFrequency(uint8_t _frequency)
{

}

///////////////////////////////////////////////////////////
/// @brief  Send PWM through i2c to pca9685
/// @param[in]  _channel Channel 1-16
/// @param[in]  __value 0-4095 value for PWM
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result PCA9685::SetPWM(uint8_t _channel, int _value)
{

}

///////////////////////////////////////////////////////////
/// @brief  Send PWM through i2c to pca9685
/// @param[in]  _channel Channel 1-16
/// @param[in]  _on 0-4095 value to turn on the pulse
/// @param[in]  _off 0-4095 value to turn off the pulse
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result PCA9685::SetPWM(uint8_t _channel, int _on, int _off)
{

}

///////////////////////////////////////////////////////////
/// @brief  Get current PWM value
/// @param[in]  _channel Channel 1-16
/// @param[out]  _value PWM value
/// @return int common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result PCA9685::GetPWM(uint8_t _channel, int& _value)
{

}

///////////////////////////////////////////////////////////
/// @brief  reset lcd1602
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result PCA9685::reset()
{

}

} // namespace servo_controller