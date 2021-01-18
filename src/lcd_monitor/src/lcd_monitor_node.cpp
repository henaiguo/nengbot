///////////////////////////////////////////////////////////
/// @file	lcd_monitor_node.cpp
/// @brief	lcd monitor node
/// @author	henaiguo
/// Copyright (C) 2021- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////

#include <lcd_monitor/lcd_monitor.h>
#include <ros/ros.h>

using namespace LCDMonitor;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lcd_monitor_node");
    
    LCDMonitor lcdMonitor;
    if (!lcdMonitor.Initialize()) {
        ROS_ERROR("");
        lcdMonitor.Finalize();
        return 1;
    }

    ROS_INFO("");
    ros::spin();
    return 0;
}
