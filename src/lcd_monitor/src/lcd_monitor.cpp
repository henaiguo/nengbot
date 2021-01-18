
///////////////////////////////////////////////////////////
/// @file	lcd_monitor.cpp
/// @brief	lcd monitor
/// @author	henaiguo
/// Copyright (C) 2021- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////

#include <lcd_monitor/lcd_monitor.h>
#include <lcd_monitor/lcd_1602.h>

#include <sstream>

using namespace CommonLibrary::Types;

namespace LCDMonitor {
///////////////////////////////////////////////////////////
/// @brief		Default constructor
/// @return		None
/// @note
///////////////////////////////////////////////////////////
LCDMonitor::LCDMonitor()
{
    m_lcdControl = new LCD1602();
}

///////////////////////////////////////////////////////////
/// @brief		Destructor
/// @return		None
/// @note
///////////////////////////////////////////////////////////
LCDMonitor::~LCDMonitor()
{
    Finalize();
}

///////////////////////////////////////////////////////////
/// @brief		Initialize
/// @retval		true
/// @retval		false
/// @note
///////////////////////////////////////////////////////////
bool LCDMonitor::Initialize()
{
    if (!m_lcdControl->Initialize()) {

        return false;
    }

    ros::NodeHandle n;
    m_basicStateSub = n.subscribe("/basic_state", 10, basicStateCallback);

    return true;
}

///////////////////////////////////////////////////////////
/// @brief		Finalize
/// @return		None
/// @note
///////////////////////////////////////////////////////////
void LCDMonitor::Finalize()
{
    m_lcdControl->Finalize();
    delete m_lcdControl;
    m_lcdControl = NULL;
}

///////////////////////////////////////////////////////////
/// @brief		Baisc state callback
/// @param[in]	_message Message of basic state
/// @return		None
/// @note
///////////////////////////////////////////////////////////
void LCDMonitor::basicStateCallback(const neng_msgs::BasicState::ConstPtr& _message)
{
    displayCurrentState(_message->current_state);
}

///////////////////////////////////////////////////////////
/// @brief		Display message on screen
/// @param[in]	_state Current state to display
/// @retval		true
/// @retval		false
/// @note
///////////////////////////////////////////////////////////
bool LCDMonitor::displayCurrentState(CommonLibrary::Types::eBaiscState _state)
{
    std::string state = ToString(_state);
    if (state == "INVALID_ENUM_VALUE") return false;
    std::stringstream message;
    message << "state: " << state;

    m_lcdControl->Clear();
    return m_lcdControl->Display(message.str(), 1);
}
} // namespace LCDMonitor
