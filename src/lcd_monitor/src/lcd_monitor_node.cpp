///////////////////////////////////////////////////////////
/// @file	lcd_monitor_node.cpp
/// @brief	lcd monitor node
/// @author	henaiguo
/// Copyright (C) 2021- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////

#include <lcd_monitor/lcd_monitor.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lcd_monitor_node");

    LCDMonitor::LCDMonitor lcdMonitor;
    lcdMonitor.Initialize();
    lcdMonitor.Display("Hello world!", 1);
    lcdMonitor.Display("I'm nengbot~", 2);

    ros::Duration(10).sleep();
}
