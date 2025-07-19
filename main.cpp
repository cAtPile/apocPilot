#include <ros/ros.h>
#include "OffboardControl.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");

    OffboardControl controller;

    // 连接到飞控
    controller.connect();

    // 设置飞行模式为 OFFBOARD
    controller.setMode("OFFBOARD");

    // 解锁飞行器
    controller.arm(true);

    // 记录home位置
    controller.recordingHomePoints();

    if(controller.connect() && controller.arm(true) && controller.setMode("OFFBOARD")) {
        ROS_INFO("Connected, armed and mode switched successfully, ready to execute mission.");

        //1. 飞行到第一个目标位置
        controller.flyToPosition(0, 0, 1.0);

        //2. 悬停1秒
        controller.hover(1.0);

        //3. 降落
        controller.landing();

        ROS_INFO("Mission completed successfully!");

        //解除武装
        controller.arm(false);
    } else {
        ROS_ERROR("Failed to connect or arm.");
        return -1;
    }

    return 0;
}
