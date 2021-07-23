///////////////////////////////////////////////////////////
/// @file   lcd_monitor_node.cpp
/// @brief  lcd monitor node
/// @author henaiguo
/// Copyright (C) 2021- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////

#include <lcd_monitor/lcd_monitor.h>
#include <ros/ros.h>

using lcd_monitor::LCDMonitor;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lcd_monitor_node");
    
    LCDMonitor lcdMonitor;
    if (!lcdMonitor.Initialize()) {
        lcdMonitor.Finalize();
        return 1;
    }

    ROS_INFO("lcd_monitor_node init success");
    ros::spin();
    return 0;
}
