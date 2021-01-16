
///////////////////////////////////////////////////////////
/// @file	lcd_monitor.cpp
/// @brief	lcd monitor
/// @author	henaiguo
/// Copyright (C) 2021- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////

#include <lcd_monitor/lcd_monitor.h>
#include <lcd_monitor/lcd1602.h>

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
        return m_lcdControl->Initialize();
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
    }

    ///////////////////////////////////////////////////////////
	/// @brief		Display message on screen
    /// @param[in]	_msg Message to display
    /// @param[in]	_line line number to display
    /// @retval		true
    /// @retval		false
	/// @note
	///////////////////////////////////////////////////////////
    bool LCDMonitor::Display(std::string _msg, int _line)
    {
        return m_lcdControl->Display(_msg, _line);
    }

    ///////////////////////////////////////////////////////////
	/// @brief		Clear screen
	/// @return		None
	/// @note
	///////////////////////////////////////////////////////////
    void LCDMonitor::Clear()
    {
        return m_lcdControl->Clear();
    }
} // namespace LCDMonitor
