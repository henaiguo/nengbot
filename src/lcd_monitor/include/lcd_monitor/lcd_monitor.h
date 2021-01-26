///////////////////////////////////////////////////////////
/// @file   lcd_monitor.h
/// @brief  lcd monitor
/// @author henaiguo
/// Copyright (C) 2021- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////
#ifndef LCD_MONITOR_H
#define LCD_MONITOR_H

#include <lcd_monitor/ilcd_control.h>
#include <common_library/types/ebasic_state.h>

#include <ros/ros.h>
#include <neng_msgs/BasicState.h>

namespace lcd_monitor {
///////////////////////////////////////////////////////////
/// @class LCDMonitor
/// @brief lcd monitor
/// @note
///////////////////////////////////////////////////////////
class LCDMonitor {
public:
    ///////////////////////////////////////////////////////////
    /// @brief  Default constructor
    /// @return None
    /// @note
    ///////////////////////////////////////////////////////////
    LCDMonitor();

    ///////////////////////////////////////////////////////////
    /// @brief  Destructor
    /// @return None
    /// @note
    ///////////////////////////////////////////////////////////
    virtual ~LCDMonitor();

    ///////////////////////////////////////////////////////////
    /// @brief  Initialize
    /// @retval true
    /// @retval false
    /// @note
    ///////////////////////////////////////////////////////////
    bool Initialize();

    ///////////////////////////////////////////////////////////
    /// @brief  Finalize
    /// @return None
    /// @note
    ///////////////////////////////////////////////////////////
    void Finalize();

private:
    /// ilcd_control
    ILCDControl* m_lcdControl;

    /// BasicState subscriber
    ros::Subscriber m_basicStateSub;

    ///////////////////////////////////////////////////////////
    /// @brief  Baisc state callback
    /// @param[in]  _message Message of basic state
    /// @return None
    /// @note
    ///////////////////////////////////////////////////////////
    void basicStateCallback(const neng_msgs::BasicState::ConstPtr& _message);

    ///////////////////////////////////////////////////////////
    /// @brief  Display message on screen
    /// @param[in]  _state Current state to display
    /// @retval true
    /// @retval false
    /// @note
    ///////////////////////////////////////////////////////////
    bool displayCurrentState(common_library::types::eBasicState _state);
};
} // namespace lcd_monitor

#endif // LCD_MONITOR_H
