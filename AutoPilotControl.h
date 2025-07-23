#ifndef AUTO_PILOT_CONTROL_H
#define AUTO_PILOT_CONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <vector>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class AutoPilotControl {
private:
    // ROS节点句柄
    ros::NodeHandle nh;

    // 订阅者、发布者和服务客户端
    ros::Subscriber state_sub;
    ros::Publisher local_pos_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::Subscriber local_pos_sub;
    ros::Time land_start_time;

    //返航（待定）
    bool returning_home = false;

    //降落初始化（待定）
    bool landing_initiated = false;

    // 当前状态
    mavros_msgs::State current_state;

    // 目标位置和实际位置
    geometry_msgs::PoseStamped pose;

    // 发布频率
    ros::Rate rate;

    // 上次请求时间
    ros::Time last_request;

    // 状态回调函数
    void state_cb(const mavros_msgs::State::ConstPtr& msg);

    // 实际位置回调函数
    void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

public:

    geometry_msgs::PoseStamped current_position;
    geometry_msgs::PoseStamped home_position;

    // 构造函数
    OffboardControl();

    // 连接到飞控
    bool connect();

    // 解锁无人机
    bool arm(bool arm_state = true);

    // 设置飞行模式
    bool setMode(const std::string& mode);

    // 记录Home位置
    bool recordingHomePoint();

    // 设置目标位置
    void setPosition(float set_point_x, float set_point_y, float set_point_z);

    //飞行到指定位置
    bool flyToPosition(float x, float y, float z);

    // 检查是否到达目标位置
    bool hasReachedTarget(float check_target_x, float check_target_y, float check_target_z);

    // 移动到相对位置
    void move(float move_x, float move_y, float move_z);

    // 开始自动降落
    void startAutoLanding();

    //等待降落完成
    void waitForLanding();

    // 降落操作
    bool landing(float Landing_x, float Landing_y);

    // 悬停指定时间
    void hover(float hover_time); 

    //自转
    void rotation(double angle, double angular_velocity = 0.1);

    //相对起飞点飞行
    void targetPositionRelativeHome(float relative_home_x, float relative_home_y, float relative_home_z);

    //

};

#endif
