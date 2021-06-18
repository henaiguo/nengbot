///////////////////////////////////////////////////////////
/// @file   lcd1602.h
/// @brief  lcd1602 display based on pcf8574 through i2c
/// @author henaiguo
/// Copyright (C) 2021- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////
#ifndef LCD1602_H
#define LCD1602_H

#include <lcd_monitor/i_lcd_control.h>
#include <common_library/io/i2c.h>

namespace lcd_monitor {
///////////////////////////////////////////////////////////
/// @class LCD1602
/// @brief lcd1602 display based on i2c
/// @note
///////////////////////////////////////////////////////////
class LCD1602 : public ILCDControl
{
public:
    ///////////////////////////////////////////////////////////
    /// @brief  Default constructor
    /// @return None
    /// @note
    ///////////////////////////////////////////////////////////
    LCD1602();

    ///////////////////////////////////////////////////////////
    /// @brief  Destructor
    /// @return None
    /// @note
    ///////////////////////////////////////////////////////////
    virtual ~LCD1602();

    ///////////////////////////////////////////////////////////
    /// @brief  Initialize
    /// @return common_library::Error
    /// @note
    ///////////////////////////////////////////////////////////
    virtual common_library::Error Initialize();

    ///////////////////////////////////////////////////////////
    /// @brief  Finalize
    /// @return None
    /// @note
    ///////////////////////////////////////////////////////////
    virtual void Finalize();

    ///////////////////////////////////////////////////////////
    /// @brief  Display message on screen
    /// @param[in]  _msg Message to display
    /// @param[in]  _line line number to display
    /// @return common_library::Error
    /// @note
    ///////////////////////////////////////////////////////////
    virtual common_library::Error Display(std::string _msg, int _line);

    ///////////////////////////////////////////////////////////
    /// @brief  Clear screen
    /// @return None
    /// @note
    ///////////////////////////////////////////////////////////
    virtual void Clear();

    ///////////////////////////////////////////////////////////
    /// @brief  Backlight on or off
    /// @param[in]  _onoff switch of backlight
    /// @return None
    /// @note
    ///////////////////////////////////////////////////////////
    virtual void Backlight(bool _onoff);

private:
    /// i2c driver
    common_library::io::I2C m_i2c;

    /// Backlight
    uint8_t m_backlight;

    ///////////////////////////////////////////////////////////
    /// @brief  Send command through i2c to lcd1602
    /// @param[in]  _command Command to send
    /// @return common_library::Error
    /// @note
    ///////////////////////////////////////////////////////////
    common_library::Error i2cWriteCommand(uint8_t _command);

    ///////////////////////////////////////////////////////////
    /// @brief  Send char value through i2c to lcd1602
    /// @param[in]  _command Command to send
    /// @return common_library::Error
    /// @note
    ///////////////////////////////////////////////////////////
    common_library::Error i2cWriteChar(char _data);

    ///////////////////////////////////////////////////////////
    /// @brief  Send 4 bits through i2c to lcd1602
    /// @param[in]  _command Command to send
    /// @return common_library::Error
    /// @note
    ///////////////////////////////////////////////////////////
    common_library::Error i2cSend4Bits(uint8_t _data);

    ///////////////////////////////////////////////////////////
    /// @brief  i2c plus enable
    /// @param[in]  _command Command to send
    /// @return common_library::Error
    /// @note   Clocks EN to latch command
    ///////////////////////////////////////////////////////////
    common_library::Error i2cPlusEnable(uint8_t _data);

    ///////////////////////////////////////////////////////////
    /// @brief  Send byte through i2c to lcd1602
    /// @param[in]  _byte Byte to send
    /// @return common_library::Error
    /// @note
    ///////////////////////////////////////////////////////////
    common_library::Error i2cSendByte(uint8_t _byte);
};
} // namespace lcd_monitor

#endif // LCD1602_H
