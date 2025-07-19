#include "OffboardControl.h"

OffboardControl::OffboardControl() : rate(20.0) {

    // 初始化订阅者、发布者和服务客户端
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &OffboardControl::state_cb, this);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &OffboardControl::local_pos_cb, this);

    // 初始化状态
    current_state.connected = false;
    current_state.armed = false;
    current_state.mode = "STABILIZED";  // 默认模式
    pose.header.frame_id = "map";

    // 初始化目标位置
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    // 初始化上次请求时间
    last_request = ros::Time::now();

}

void OffboardControl::state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void OffboardControl::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_position = *msg;
}

bool OffboardControl::connect() {

    // 等待 FCU 连接
    ROS_INFO("Waiting for FCU connection...");
    ros::Time start_time = ros::Time::now();
    ros::Duration timeout_duration(10.0);  // 超时10秒

    // 循环等待直到 FCU 连接或者超时
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();

        // 检查是否超时
        if ((ros::Time::now() - start_time) > timeout_duration) {
            ROS_ERROR("Failed to connect to FCU within timeout period!");
            return false;
        }
    }

    // 如果连接成功
    if (current_state.connected) {
        ROS_INFO("FCU connected!");
        return true;
    }

    // 额外的安全检查：如果出现异常没有连接成功
    ROS_ERROR("FCU connection failed unexpectedly!");
    return false;
}

// 解锁无人机
bool OffboardControl::arm(bool arm_state) {
    
    // 容忍时间
    ros::Duration timeout_duration(5.0); 
    ros::Time start_time = ros::Time::now();

    // 检查是否连接到飞控
    if (!current_state.connected) {
        ROS_WARN("Not connected to FCU, cannot arm/disarm");
        return false;
    }

    // 检查是切换到offboard模式
    if (current_state.mode != "OFFBOARD") {
        ROS_WARN("Cannot arm/disarm, not in OFFBOARD mode");
        return false;
    }

    // 打印尝试解锁或锁定的状态
    std::string action = arm_state ? "arming" : "disarming";
    ROS_INFO_STREAM("Attempting to " << action);

    // 循环直到状态改变或超时
    while (ros::ok() && current_state.armed != arm_state) {
        
        // 创建解锁或锁定命令
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = arm_state;

        // 调用解锁或锁定服务
        if (arming_client.call(arm_cmd)) {
            if (arm_cmd.response.success) {
                ROS_INFO_STREAM("Vehicle " << action);
                last_request = ros::Time::now();
                return true;
            }
        }

        // 检查超时
        if ((ros::Time::now() - start_time)> timeout_duration) {
            ROS_WARN_STREAM("Timeout during " << action);
            return false;
        }

        // 发布当前位置
        local_pos_pub.publish(pose);

        // 处理回调函数
        ros::spinOnce();
        rate.sleep();
    }

    return true;
}

//记录home位置姿态
bool recordingHomePoints() {

    ROS_INFO("Sending Home Position...");
         
    // 记录当前位置的位姿
    home_position = current_position; 

    // 记录home位置
    ROS_INFO_STREAM("home Pose recorded: x=" << home_position.pose.position.x 
                                    << " y=" << home_position.pose.position.y 
                                    << " z=" << home_position.pose.position.z);

    ROS_INFO_STREAM("home Orientation recorded: x=" << home_position.pose.orientation.x 
                                            << " y=" << home_position.pose.orientation.y 
                                            << " z=" << home_position.pose.orientation.z 
                                            << " w=" << home_position.pose.orientation.w);

    return true;
}

// 设置目标位置
void setPosition(float target_x, float target_y, float target_z) {
    
    pose.pose.position.x = target_x;
    pose.pose.position.y = target_y;
    pose.pose.position.z = target_z;

    // 保持当前朝向
    pose.pose.orientation.x = current_position.pose.orientation.x;
    pose.pose.orientation.y = current_position.pose.orientation.y;
    pose.pose.orientation.z = current_position.pose.orientation.z;
    pose.pose.orientation.w = current_position.pose.orientation.w; 

}

// 计算两点间距离
float calculateDistance(float x1, float y1, float z1, float x2, float y2, float z2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));
}

// 飞行到指定位置
bool flyToPosition(float target_x, float target_y, float target_z) {
        
    // 设置目标位置
    setPosition(target_x, target_y, target_z);

    //超时处理
    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(30.0);  // 超时时间30秒

    //检查是否到达目标位置
    while (ros::ok() && !hasReachedTarget(x, y, z)) {
            
        //超时退出
        if ((ros::Time::now() - start_time) > timeout) {
                
            ROS_WARN("Timeout flying to position");
            return false;
            
        }
        
        // 发布当前位置
        local_pos_pub.publish(pose);
            
        ros::spinOnce();
            
        rate.sleep();
        
    }
        
    return true;
    
}

// 检查是否到达目标位置
bool hasReachedTarget(float target_x, float target_y, float target_z) {
    
    // 默认容忍距离为0.1米
    float tolerance = 0.1

    // 计算当前位置与目标位置的距离
    return calculateDistance(
        current_position.pose.position.x, 
        current_position.pose.position.y, 
        current_position.pose.position.z, 
        target_x, target_y, target_z) < tolerance;

}

// 移动到相对位置
void move( float move_x , float move_y , float move_z ) {
       
    //设置容忍值
    float tolerance = 0.1;

    //超时时间
    ros::Duration timeout(30.0);  // 超时时间10秒
    
    // 计算目标位置 
    float target_x = current_position.pose.position.x + move_x;
    float target_y = current_position.pose.position.y + move_y;
    float target_z = current_position.pose.position.z + move_z;

    ROS_INFO("Moving to position (%.2f, %.2f, %.2f) with tolerance %.2f", target_x, target_y, target_z, tolerance);

    // 设置目标位置
    setPosition(target_x, target_y, target_z);
        
    // 发布位置
    local_pos_pub.publish(pose);

    // 等待直到到达目标位置
    ros::Time start_time = ros::Time::now();
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();

        // 检查是否到达目标位置
        float dx = std::abs(controller.current_position.pose.position.x - target_x);
        float dy = std::abs(controller.current_position.pose.position.y - target_y);
        float dz = std::abs(controller.current_position.pose.position.z - target_z);

        if (dx < tolerance && dy < tolerance && dz < tolerance) {
            ROS_INFO("Reached target position.");
            break;
        }

        // 超时检测
        if ((ros::Time::now() - start_time) > timeout) {
            ROS_WARN("Timeout while moving to target position.");
            break;
        }
    }
}

void waitForLanding() {
    ROS_INFO("Waiting for landing...");
    ros::Time land_start = ros::Time::now();
        
    while (ros::ok()) {
        // 检查高度和武装状态
        if (current_position.pose.position.z < 0.1 || !current_state.armed) {
            ROS_INFO("Landed safely.");
            break;
        }
            
        // 超时处理
        if ((ros::Time::now() - land_start).toSec() > 30.0) {
            ROS_ERROR("Landing timeout! Forcing disarm.");
            arm(false);  // 强制解除武装
            break;
        }
            
        ros::spinOnce();
        rate.sleep();
    }
}

bool landing(float Landing_x, float Landing_y, float Landing_hight = 0.0) {
        
    ROS_INFO("Initiating landing...");

    flyToPosition(  Landing_x, Landing_y, Landing_hight);  // 飞行到降落位置
    ROS_INFO("Has reached landing position, starting auto landing...");
    ROS_INFO(landing position: x=%.2f, y=%.2f, z=%.2f",
            home_position.pose.position.x,
            home_position.pose.position.y,
            home_position.pose.position.z");

    // 开始自动降落
    setMode("AUTO.LAND");
    ROS_INFO("Auto landing initiated, waiting for landing...");

    waitForLanding();
    ROS_INFO("Landing completed successfully.");
        
    return true;

}