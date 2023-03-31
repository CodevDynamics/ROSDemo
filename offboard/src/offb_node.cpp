// ROS 头文件
#include <ros/ros.h>
// 用于发送 Offboard 坐标点的消息包头文件
#include <geometry_msgs/PoseStamped.h>
// 用于发送通用命令的消息包的头文件，如：解锁飞机
#include <mavros_msgs/CommandBool.h>
// 用于发送切换飞行模式的消息包头文件，如：切换飞机到 Offboard 模式
#include <mavros_msgs/SetMode.h>
// 用于获取飞机状态的消息包头文件，在该程序中用于闭环判断飞机控制状态，如：是否解锁，是否处于 Offboard 飞行模式
#include <mavros_msgs/State.h>

// 定义全局飞机状态变量，用于闭环判断
mavros_msgs::State current_state;
// ROS 订阅飞机状态的回调函数
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // ROS 订阅飞机状态消息
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // ROS Topic 发布 Offboard 控制坐标点
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    // ROS Service 请求控制飞机解锁
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    // ROS Service 请求控制飞机切换飞行模式
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    // 设定值发布速率 20 hz
    ros::Rate rate(20.0);

    // 等待飞机连上
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // 定义 Offboard 坐标点
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    // 启动前发送一会设定值，因为飞机未解锁和切换到 Offboard，并不会动作，这里只是为了确保安全
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // 定义 Offboard 飞行模式消息包
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // 定义飞机解锁消息包
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // 定义时间戳，用于保证控制切换飞行模式和解锁消息发送的不能过快
    ros::Time last_request = ros::Time::now();

    // 主循环
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){ // 首先确保飞机已经切换解锁状态
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){ // 再解锁飞机
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // 发送 Offboard 控制值，并保活
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}