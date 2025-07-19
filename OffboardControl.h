#ifndef OFFBOARD_CONTROL_H
#define OFFBOARD_CONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <vector>
#include <cmath>

class OffboardControl {
private:
    // ROS节点句柄
    ros::NodeHandle nh;

    // 订阅者、发布者和服务客户端
    ros::Subscriber state_sub;
    ros::Publisher local_pos_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::Subscriber local_pos_sub;
    ros::Time waypoint_reached_time;
    ros::Time land_start_time;
    bool waiting_at_waypoint = false;
    bool returning_home = false;
    bool landing_initiated = false;

    // 当前状态
    mavros_msgs::State current_state;

    // 目标位置和实际位置
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped current_position;
    geometry_msgs::PoseStamped home_position;

    // 发布频率
    ros::Rate rate;

    // 上次请求时间
    ros::Time last_request;

    // 状态回调函数
    void state_cb(const mavros_msgs::State::ConstPtr& msg);

    // 实际位置回调函数
    void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

public:
    // 构造函数
    OffboardControl();

    // 连接到飞控
    bool connect();

    // 解锁无人机
    bool arm(bool arm_state = true);

    // 设置飞行模式
    bool setMode(const std::string& mode, double timeout = 5.0);

    // 发送初始位置指令
    bool recordingHomePoints();

    // 设置目标位置
    void setPosition(float target_x, float target_y, float target_z);

    // 计算两点间距离
    float calculateDistance(float x1, float y1, float z1, float x2, float y2, float z2);

    //飞行到指定位置
    bool flyToPosition(float x, float y, float z, double timeout = 30.0);

    // 检查是否到达目标位置
    bool hasReachedTarget(float target_x, float target_y, float target_z, float tolerance = 0.1);

    // 移动到相对位置
    void move(float move_x, float move_y, float move_z, float tolerance);

    // 开始自动降落
    void startAutoLanding();

    //等待降落完成
    void waitForLanding();

    // 降落操作
    bool landing();
};

#endif
