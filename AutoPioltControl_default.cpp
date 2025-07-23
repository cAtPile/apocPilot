#include "offboard_ctrl/OffboardControl.h"

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
        local_pos_pub.publish(pose);
        return true;
    }

    // 额外的安全检查：如果出现异常没有连接成功
    ROS_ERROR("FCU connection failed unexpectedly!");
    return false;
}

//设置飞行模式
bool OffboardControl::setMode(const std::string& mode){
            
    if (!current_state.connected) {
        ROS_WARN("Not connected to FCU, cannot set mode");
        return false;
    }
        
    ROS_INFO_STREAM("Attempting to set mode: " << mode);
    ros::Time start = ros::Time::now();

    while (ros::ok() && current_state.mode != mode) {
        mavros_msgs::SetMode mode_cmd;
        mode_cmd.request.custom_mode = mode;

        if (set_mode_client.call(mode_cmd) && mode_cmd.response.mode_sent) {
            ROS_INFO_STREAM(mode << " mode enabled");
            return true;
        }

        if (ros::Time::now() - start> ros::Duration(5.0)) {
            ROS_WARN_STREAM("Timeout setting " << mode << " mode");
            return false;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return true;
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
    if (!OffboardControl::setMode("OFFBOARD")) {
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
bool OffboardControl::recordingHomePoints() {

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
void OffboardControl::setPosition(float target_x, float target_y, float target_z) {
    
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
float OffboardControl::calculateDistance(float x1, float y1, float z1, float x2, float y2, float z2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));
}

// 飞行到指定位置
bool OffboardControl::flyToPosition(float target_x, float target_y, float target_z) {
        
    // 设置目标位置
    setPosition(target_x, target_y, target_z);

    //超时处理
    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(30.0);  // 超时时间30秒

    //检查是否到达目标位置
    while (ros::ok() && !hasReachedTarget(target_x, target_y, target_z)) {
            
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

// 检查是否到达目标位置（分别判断x, y, z轴）
bool OffboardControl::hasReachedTarget(float target_x, float target_y, float target_z) {
    
    // 默认容忍距离为0.05米
    float tolerance = 0.05;

    // 分别判断x, y, z方向上的偏差
    bool reached_x = abs(current_position.pose.position.x - target_x) < tolerance;
    bool reached_y = abs(current_position.pose.position.y - target_y) < tolerance;
    bool reached_z = abs(current_position.pose.position.z - target_z) < tolerance;

    // 只有当所有轴都到达目标位置时，才返回true
    return reached_x && reached_y && reached_z;
}


void OffboardControl::move(float move_x, float move_y, float move_z) {
    ros::Rate rate(20);  // 20Hz
    float tolerance = 0.05;
    ros::Duration timeout(30.0);

    float target_x = current_position.pose.position.x + move_x;
    float target_y = current_position.pose.position.y + move_y;
    float target_z = current_position.pose.position.z + move_z;

    ROS_INFO("Moving to position (%.2f, %.2f, %.2f)", target_x, target_y, target_z);

    ros::Time start_time = ros::Time::now();
    while (ros::ok()) {
        // 关键修改：每次循环都发布指令
        setPosition(target_x, target_y, target_z);
        local_pos_pub.publish(pose);

        // 检查位置容差
        float dx = abs(current_position.pose.position.x - target_x);
        float dy = abs(current_position.pose.position.y - target_y);
        float dz = abs(current_position.pose.position.z - target_z);

        if (dx < tolerance && dy < tolerance && dz < tolerance) {
            ROS_INFO("Reached target position");
            break;
        }

        // 超时检查
        if ((ros::Time::now() - start_time) > timeout) {
            ROS_WARN("Movement timeout");
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }
}

void OffboardControl::waitForLanding() {
    ROS_INFO("Waiting for landing...");
    ros::Time land_start = ros::Time::now();
        
    while (ros::ok()) {
        // 检查高度和武装状态
        if (current_position.pose.position.z < 0.05 || !current_state.armed) {
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

bool OffboardControl::landing(float Landing_x, float Landing_y) {
     /*   
    ROS_INFO("Initiating landing...");

    flyToPosition(  Landing_x, Landing_y, Landing_hight);  // 飞行到降落位置
    ROS_INFO("Has reached landing position, starting auto landing...");
    ROS_INFO("landing position: x=%.2f, y=%.2f, z=%.2f",
            Landing_x , Landing_y , Landing_hight );

    // 开始自动降落
    setMode("AUTO.LAND");
    ROS_INFO("Auto landing initiated, waiting for landing...");

    waitForLanding();
    ROS_INFO("Landing completed successfully.");
        
    return true;

*/
    float Landing_hight=0;

    float descent_rate = 0.05; // 每次下降0.05米，可根据需求调节
    float sleep_time = 0.2;   // 下降控制循环间隔

    ros::Rate rate(1.0 / sleep_time);

    float current_z = current_position.pose.position.z;

    while (ros::ok() && current_z > Landing_hight + 0.05) { // 留0.05m做安全冗余
        current_z -= descent_rate;
        if(current_z < Landing_hight)
            current_z = Landing_hight;

        setPosition(Landing_x, Landing_y, current_z);
        local_pos_pub.publish(pose);

        ROS_INFO("Descending... current target z: %.2f", current_z);

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Final stage landing...");

    // 最后切换到AUTO.LAND确保完全放下并解锁
    setMode("AUTO.LAND");
    waitForLanding();

    ROS_INFO("Smooth landing completed.");

    return true;

}

void OffboardControl::hover(float hover_time) {
    
    ROS_INFO("Hovering for %.2f seconds...", hover_time);
    ros::Time start_time = ros::Time::now();
    
    // 在滞空时间内持续发布当前位置
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < hover_time) {
        // 发布当前位置，保持飞行器不动
        local_pos_pub.publish(current_position);
        ros::spinOnce();
        rate.sleep();   
    }

    ROS_INFO("Hovering complete.");
}

// 旋转函数：绕Z轴相对自转
void OffboardControl::rotation(double angle, double angular_velocity) {

    ros::Rate rate(20); // 发布频率

    // 将目标角度转换为弧度
    double delta_yaw = angle * M_PI / 180.0;

    // 1. 获取当前偏航角
    tf2::Quaternion q(
        current_position.pose.orientation.x,
        current_position.pose.orientation.y,
        current_position.pose.orientation.z,
        current_position.pose.orientation.w
    );

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    ROS_INFO("Current yaw: %.2f rad (%.2f deg)", yaw, yaw * 180.0 / M_PI);

    // 2. 计算目标偏航角
    double target_yaw = yaw + delta_yaw;

    ROS_INFO("Rotating by %.2f deg to target yaw: %.2f deg", angle, target_yaw * 180.0 / M_PI);

    // 3. 开始旋转
    ros::Time start_time = ros::Time::now();

    while (ros::ok()) {

        ros::spinOnce();

        // 根据旋转速度计算当前目标角度
        double elapsed = (ros::Time::now() - start_time).toSec();
        double current_target_yaw = yaw + std::min(delta_yaw, angular_velocity * elapsed);

        // 将角度转换为四元数
        tf2::Quaternion quat;
        quat.setRPY(0, 0, current_target_yaw);
        quat.normalize();

        // 更新目标姿态
        pose.pose.orientation.x = quat.x();
        pose.pose.orientation.y = quat.y();
        pose.pose.orientation.z = quat.z();
        pose.pose.orientation.w = quat.w();

        // 发布更新后的目标位置和姿态
        local_pos_pub.publish(pose);

        // 判断是否达到目标角度
        if (std::abs(current_target_yaw - target_yaw) < 0.01) {
            ROS_INFO("Rotation completed. Final yaw: %.2f deg", current_target_yaw * 180.0 / M_PI);
            break;
        }

        rate.sleep();
    }
}

void OffboardControl::targetPositionRelativeHome(float relative_home_x, float relative_home_y, float relative_home_z) {

    // 确保home位置已经被记录
    if (home_position.header.stamp == ros::Time(0)) {
        ROS_ERROR("Home position not recorded. Please record the home position first.");
        return;
    }

    // 获取当前的home位置
    float home_x = home_position.pose.position.x;
    float home_y = home_position.pose.position.y;
    float home_z = home_position.pose.position.z;

    // 计算目标位置，基于home位置加上相对偏移量
    float target_x = home_x + relative_home_x;
    float target_y = home_y + relative_home_y;
    float target_z = home_z + relative_home_z;

    ROS_INFO("Moving to relative position (%.2f, %.2f, %.2f) relative to home position (%.2f, %.2f, %.2f)",
             relative_home_x, relative_home_y, relative_home_z,
             home_x, home_y, home_z);

    // 使用flyToPosition函数飞到目标位置
    flyToPosition(target_x, target_y, target_z);
}

