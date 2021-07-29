///////////////////////////////////////////////////////////
/// @file   pca_9685.cpp
/// @brief  Servo control based on pca9685 through m_i2c
/// @author henaiguo
/// Copyright (C) 2021- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////

#include <motor_controller/pca_9685.h>

using common_library::types::Result;
using common_library::io::I2C;

namespace motor_controller {
/// Device name
#define PCA9685_I2C_DEVICE_NAME "/dev/m_i2c-2"

/// Device address
#define PCA9685_I2C_DEVICE_ADDRESS 0x70

/// Mode  register
#define MODE1 0x00
#define MODE2 0x01

/// I2C-bus subaddress
#define SUBADR1 0x02
#define SUBADR2 0x03
#define SUBADR3 0x04

/// LED All Call I2C-bus address
#define ALLCALLADR 0x05

/// LED0 start register
#define LED0 0x6

/// LED0 output and brightness control
#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

/// For the other 15 channels
#define LED_MULTIPLYER 4

/// Load all the LEDn_ON registers
#define ALLLED_ON_L 0xFA    // byte 0 (turn 0-7 channels on)
#define ALLLED_ON_H 0xFB	// byte 1 (turn 8-15 channels on)
#define ALLLED_OFF_L 0xFC	// byte 0 (turn 0-7 channels off)
#define ALLLED_OFF_H 0xFD	// byte 1 (turn 8-15 channels off)

/// Prescaler for output frequency
#define PRE_SCALE 0xFE

/// 25MHz default osc clock
#define CLOCK_FREQ 25000000.0

///////////////////////////////////////////////////////////
/// @brief  Default constructor
/// @return None
/// @note
///////////////////////////////////////////////////////////
PCA9685::PCA9685()
{

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
    Result result = m_i2c.Open(PCA9685_I2C_DEVICE_NAME, PCA9685_I2C_DEVICE_ADDRESS);
    if (!result) return result;

    result = reset();
    if (!result) return result;

    return setPWMFrequency(50);
}

///////////////////////////////////////////////////////////
/// @brief  Finalize
/// @return None
/// @note
///////////////////////////////////////////////////////////
void PCA9685::Finalize()
{    
    m_i2c.Close();
}

///////////////////////////////////////////////////////////
/// @brief  Set actuator position
/// @param[in]  TODO
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result PCA9685::SetActuatorPosition(/*TODO*/)
{

}

///////////////////////////////////////////////////////////
/// @brief  Set actuator positions
/// @param[in]  TODO
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result PCA9685::SetActuatorPositions(/*TODO*/)
{

}

///////////////////////////////////////////////////////////
/// @brief  reset lcd1602
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result PCA9685::reset()
{
    // Normal mode
    Result result = m_i2c.WriteByte(MODE1, 0X00);
    if (!result) return result;
    // Totem mode
    return m_i2c.WriteByte(MODE2, 0X04);
}

///////////////////////////////////////////////////////////
/// @brief  Send PWM frequency through m_i2c to pca9685
/// @param[in]  _frequency 40Hz to 1000Hz using internal 25MHz oscillator
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result PCA9685::setPWMFrequency(uint8_t _frequency)
{
		uint8_t prescale_val = (CLOCK_FREQ / 4096 / _frequency)  - 1;
        // Sleep
		Result result = m_i2c.WriteByte(MODE1, 0x10);
        if (!result) return result;

		result = m_i2c.WriteByte(PRE_SCALE, prescale_val);
        if (!result) return result;
        // Restart
		result = m_i2c.WriteByte(MODE1, 0x80);
        if (!result) return result;
        // Totem pole (default)
		return m_i2c.WriteByte(MODE2, 0x04);
}

///////////////////////////////////////////////////////////
/// @brief  Send PWM through m_i2c to pca9685
/// @param[in]  _channel Channel 1-16
/// @param[in]  __value 0-4095 value for PWM
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result PCA9685::setPWM(uint8_t _channel, int _value)
{
    return setPWM(_channel, 0, _value);
}

///////////////////////////////////////////////////////////
/// @brief  Send PWM through m_i2c to pca9685
/// @param[in]  _channel Channel 1-16
/// @param[in]  _on 0-4095 value to turn on the pulse
/// @param[in]  _off 0-4095 value to turn off the pulse
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result PCA9685::setPWM(uint8_t _channel, int _on, int _off)
{
		Result result = m_i2c.WriteByte(LED0_ON_L + LED_MULTIPLYER * (led - 1), on_value & 0xFF);
        if (!result) return result;

		result = m_i2c.WriteByte(LED0_ON_H + LED_MULTIPLYER * (led - 1), on_value >> 8);
        if (!result) return result;

		result = m_i2c.WriteByte(LED0_OFF_L + LED_MULTIPLYER * (led - 1), off_value & 0xFF);
        if (!result) return result;

		return m_i2c.WriteByte(LED0_OFF_H + LED_MULTIPLYER * (led - 1), off_value >> 8);
}

///////////////////////////////////////////////////////////
/// @brief  Get current PWM value
/// @param[in]  _channel Channel 1-16
/// @param[out]  _value PWM value
/// @return int common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result PCA9685::getPWM(uint8_t _channel, int& _value)
{
	uint8_t pwmValueH, pwmValueL = 0;

	Result result = m_i2c.ReadByte(LED0_OFF_H + LED_MULTIPLYER * (led-1), pwmValueH);
    if (!result) return result;
	pwmValueH = pwmValueH & 0xf;
	pwmValueH <<= 8;

	result = m_i2c.ReadByte(LED0_OFF_L + LED_MULTIPLYER * (led-1), pwmValueL);
    if (!result) return result;

    _value = static_cast<int>(pwmValueH + pwmValueL);
	return Result::CreateSuccess();
}

} // namespace motor_controller