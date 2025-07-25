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

class autoPilotControl {
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

    // PID 控制器
    float pidControl(float error, float &previous_error, float &integral, 
                        float Kp, float Ki, float Kd);

public:

    geometry_msgs::PoseStamped current_position;
    geometry_msgs::PoseStamped home_position;

    // 构造函数
    autoPilotControl();

    // 连接到飞控
    bool connect();

    // 解锁无人机
    bool arm(bool arm_state = true);

    // 设置飞行模式
    bool setMode(const std::string& mode);

    // 记录Home位置
    void recordingHomePoint();

    // 起飞(速度控制)
    void takeoff(float altitude);

    // 设置目标位置（待定）
    void setPosition(float set_point_x, float set_point_y, float set_point_z);

    //飞行到指定位置（待定）
    bool flyToPosition(float target_absolute_x, float target_absolute_y, float target_absolute_z);

    // 检查是否到达目标位置（待修）
    bool hasReachedTarget(float check_target_x, float check_target_y, float check_target_z);

    // 移动到相对位置(待定)
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

    //闭环相对运动
    void moveRelativePID(float in_loop_x,float in_loop_y,float in_loop_h);

    //闭环相对Home运动
    void approchRelativeHomePID(float approch_inLoop_x,float approch_inLoop_y,float approch_inLoop_h);

    //

};

#endif
