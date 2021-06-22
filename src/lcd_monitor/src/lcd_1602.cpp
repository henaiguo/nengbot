///////////////////////////////////////////////////////////
/// @file   lcd1602.cpp
/// @brief  lcd1602 display based on pcf8574 through i2c
/// @author henaiguo
/// Copyright (C) 2021- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////

#include <lcd_monitor/lcd_1602.h>

#include <unistd.h>
#include <iostream>

using namespace common_library::types;

namespace lcd_monitor {
/// Device name
#define LCD_I2C_DEVICE_NAME "/dev/i2c-2"

/// Device address
#define LCD_I2C_DEVICE_ADDRESS 0x27

/// Commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

/// Flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

/// Flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

/// Flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

/// Flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

/// Flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

/// Enable bit
#define En 0B00000100

/// Read/Write bit
#define Rw 0B00000010

/// Register select bit
#define Rs 0B00000001
} // namespace lcd_monitor

namespace lcd_monitor {
///////////////////////////////////////////////////////////
/// @brief  Default constructor
/// @return None
/// @note
///////////////////////////////////////////////////////////
LCD1602::LCD1602()
    : m_backlight(LCD_BACKLIGHT)
{
    // None
}

///////////////////////////////////////////////////////////
/// @brief  Destructor
/// @return None
/// @note
///////////////////////////////////////////////////////////
LCD1602::~LCD1602()
{
    Finalize();
}

///////////////////////////////////////////////////////////
/// @brief  Initialize
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result LCD1602::Initialize()
{
    Result result;

    result = m_i2c.Open(LCD_I2C_DEVICE_NAME, LCD_I2C_DEVICE_ADDRESS);
    if (!result) return result;

    // Set lcd1602 to 4 bits mode
    result = i2cWriteCommand(0x03);
    if (!result) return result;
    result = i2cWriteCommand(0x03);
    if (!result) return result;
    result = i2cWriteCommand(0x03);
    if (!result) return result;
    result = i2cWriteCommand(0x02);
    if (!result) return result;

    // Set other mode
    result = i2cWriteCommand(LCD_FUNCTIONSET | LCD_2LINE | LCD_5x8DOTS | LCD_4BITMODE);
    if (!result) return result;
    result = i2cWriteCommand(LCD_DISPLAYCONTROL | LCD_DISPLAYON);
    if (!result) return result;
    result = i2cWriteCommand(LCD_CLEARDISPLAY);
    if (!result) return result;
    result = i2cWriteCommand(LCD_ENTRYMODESET | LCD_ENTRYLEFT);
    if (!result) return result;

    ::usleep(200);

    return result;
}

///////////////////////////////////////////////////////////
/// @brief  Finalize
/// @return None
/// @note
///////////////////////////////////////////////////////////
void LCD1602::Finalize()
{
    Clear();
    Backlight(false);
    m_i2c.Close();
}

///////////////////////////////////////////////////////////
/// @brief  Display message on screen
/// @param[in]  _msg Message to display
/// @param[in]  _line line number to display
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result LCD1602::Display(std::string _msg, int _line)
{
    uint8_t position;
    if (_line == 1) {
        position = 0;
    } 
    else if (_line == 2) {
        position = 0x40;
    }
    else {
        return Result::CreateError("LCD1602 display failed: invalid _line(%d)", _line);
    }

    Result result = i2cWriteCommand(0x80 + position);
    if (!result) return result;

    for (int i = 0; i < _msg.size(); ++i) {
        result = i2cWriteChar(_msg[i]);
        if (!result) return result;
    }

    return result;
}

///////////////////////////////////////////////////////////
/// @brief  Clear screen
/// @return None
/// @note
///////////////////////////////////////////////////////////
void LCD1602::Clear()
{
    (void)i2cWriteCommand(LCD_CLEARDISPLAY);
    (void)i2cWriteCommand(LCD_RETURNHOME);
}

///////////////////////////////////////////////////////////
/// @brief  Backlight on or off
/// @param[in]  _onoff switch of backlight
/// @return None
/// @note
///////////////////////////////////////////////////////////
void LCD1602::Backlight(bool _onoff)
{
    if (_onoff) {
        m_backlight = LCD_BACKLIGHT;
        (void)i2cWriteCommand(0);
    }
    else {
        m_backlight = LCD_NOBACKLIGHT;
        (void)i2cWriteCommand(0);
    }
}

///////////////////////////////////////////////////////////
/// @brief  Send command through i2c to lcd1602
/// @param[in]  _command Command to send
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result LCD1602::i2cWriteCommand(uint8_t _command)
{
    Result result;

    result = i2cSend4Bits(_command & 0xF0);
    if (!result) return result;

    result = i2cSend4Bits((_command << 4) & 0xF0);
    if (!result) return result;

    return result;
}

///////////////////////////////////////////////////////////
/// @brief  Send char value through i2c to lcd1602
/// @param[in]  _command Command to send
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result LCD1602::i2cWriteChar(char _data)
{
    Result result;

    result = i2cSend4Bits(Rs | ((uint8_t)_data & 0xF0));
    if (!result) return result;

    result = i2cSend4Bits(Rs | (((uint8_t)_data << 4) & 0xF0));
    if (!result) return result;

    return result;
}

///////////////////////////////////////////////////////////
/// @brief  Send 4 bits through i2c to lcd1602
/// @param[in]  _data Command to send
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result LCD1602::i2cSend4Bits(uint8_t _data)
{
    Result result;

    result = i2cSendByte(_data);
    if (!result) return result;

    result = i2cPlusEnable(_data);
    if (!result) return result;

    return result;
}

///////////////////////////////////////////////////////////
/// @brief  i2c plus enable
/// @param[in]  _data Command to send
/// @return common_library::types::Result
/// @note   Clocks EN to latch command
///////////////////////////////////////////////////////////
common_library::types::Result LCD1602::i2cPlusEnable(uint8_t _data)
{
    Result result;

    result = i2cSendByte(_data | En);
    if (!result) return result;
    ::usleep(1);

    result = i2cSendByte(_data & ~En);
    if (!result) return result;
    ::usleep(50);

    return result;
}

///////////////////////////////////////////////////////////
/// @brief  Send byte through i2c to lcd1602
/// @param[in]  _byte Byte to send
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result LCD1602::i2cSendByte(uint8_t _byte)
{
    Result result = m_i2c.SendByte(_byte | m_backlight);
    if (!result) return result;
    
    ::usleep(1000); // 1ms
    return result;
}
} // namespace lcd_monitor
