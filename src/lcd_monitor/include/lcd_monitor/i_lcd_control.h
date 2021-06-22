///////////////////////////////////////////////////////////
/// @file   i_lcd_control.h
/// @brief  Interface of lcd control
/// @author henaiguo
/// Copyright (C) 2021- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////
#ifndef I_LCD_CONTROL_H
#define I_LCD_CONTROL_H

#include <common_library/types/result.h>
#include <string>

namespace lcd_monitor {
///////////////////////////////////////////////////////////
/// @class ILCDControl
/// @brief Interface of lcd control
/// @note
///////////////////////////////////////////////////////////
class ILCDControl {
public:
    ///////////////////////////////////////////////////////////
    /// @brief  Initialize
    /// @return common_library::types::Result
    /// @note
    ///////////////////////////////////////////////////////////
    virtual common_library::types::Result Initialize() = 0;

    ///////////////////////////////////////////////////////////
    /// @brief  Finalize
    /// @return None
    /// @note
    ///////////////////////////////////////////////////////////
    virtual void Finalize() = 0;

    ///////////////////////////////////////////////////////////
    /// @brief  Display message on screen
    /// @param[in]  _msg Message to display
    /// @param[in]  _line line number to display
    /// @return common_library::types::Result
    /// @note
    ///////////////////////////////////////////////////////////
    virtual common_library::types::Result Display(std::string _msg, int _line) = 0;

    ///////////////////////////////////////////////////////////
    /// @brief  Clear screen
    /// @return None
    /// @note
    ///////////////////////////////////////////////////////////
    virtual void Clear() = 0;

    ///////////////////////////////////////////////////////////
    /// @brief  Backlight on or off
    /// @param[in]  _onoff switch of backlight
    /// @return None
    /// @note
    ///////////////////////////////////////////////////////////
    virtual void Backlight(bool _onoff) = 0;
};
} // namespace lcd_monitor

#endif // I_LCD_CONTROL_H
