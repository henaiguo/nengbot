///////////////////////////////////////////////////////////
/// @file	lcd_monitor_node.cpp
/// @brief	lcd monitor node
/// @author	henaiguo
/// Copyright (C) 2021- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////

#include <lcd_monitor/lcd_monitor.h>
#include <ros/ros.h>
#include <neng_msgs/BasicState.h>

LCDMonitor::LCDMonitor lcdMonitor;

void basicStateCallback(const neng_msgs::BasicState::ConstPtr& msg)
{
    lcdMonitor.Clear();
    switch (msg->current_state) {
    case 0:
        lcdMonitor.Display("Hi, girl!", 1);
        lcdMonitor.Display("I'm nengbot.", 2);
	break;
    case 1:
        lcdMonitor.Display("I wanna say..", 1);
        lcdMonitor.Display("      ..", 2);
	break;
    case 2:
        lcdMonitor.Display("Sorry :-<", 1);
        lcdMonitor.Display("I'm Sorry :-<", 2);
	break;
    default:
	break;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lcd_monitor_node");
    ros::NodeHandle n;

    lcdMonitor.Initialize();

    ros::Subscriber sub = n.subscribe("/basic_state", 1000, basicStateCallback);

    ros::spin();
    return 0;
}
