// ROS 头文件
#include <ros/ros.h>
// 用于发送云台控制包的头文件
#include <mavros_msgs/MountControl.h>
// 用于接受云台的姿态包的头文件
#include <geometry_msgs/Quaternion.h>
// 四元数和RPY欧拉角转换函数头文件
#include <tf/tf.h>

// 定义一些必要的全局变量
double pitch, roll, yaw;
bool gimbal_init = false;
// ROS 订阅来自地面站的目标选择消息的回调函数
void mount_orientation_cb(const geometry_msgs::Quaternion::ConstPtr& msg){
    gimbal_init = true;
    // 四元数到RPY欧拉角转换,注意极性
    double pitch_r, roll_r, yaw_r;
    tf::Quaternion quater;
    tf::quaternionMsgToTF(*msg, quater);
    tf::Matrix3x3(quater).getRPY(roll_r, pitch_r, yaw_r);
    pitch = pitch_r * 180 / M_PI;
    roll = roll_r * 180 / M_PI;
    yaw = -yaw_r * 180 / M_PI ;
    ROS_INFO("Received Mount Orientaion Message pitch, roll, yaw: (%f, %f, %f)", pitch, roll, yaw);
}

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "gimbal_node");
    ros::NodeHandle nh;

    // ROS 订阅获取云台的姿态信息
    ros::Subscriber mount_orientation_sub = nh.subscribe<geometry_msgs::Quaternion>
            ("mavros/mount_control/orientation", 1, mount_orientation_cb);
    // ROS Topic 发布控制云台姿态
    ros::Publisher mount_control_pub = nh.advertise<mavros_msgs::MountControl>
            ("mavros/mount_control/command", 1);

    // 设定值发布速率 20 hz
    ros::Rate rate(5.0);

    // 等待 Gimbal 姿态包
    while(ros::ok() && !gimbal_init){
        ros::spinOnce();
        rate.sleep();
    }

    // 定义飞机解锁消息包
    mavros_msgs::MountControl mount_control;
    // 保持 Roll 为 0
    mount_control.roll = 0;
    // 角度模式
    mount_control.mode = 2;
    // 控制角度变化
    mount_control.yaw = fabs(yaw) < 10 ? 100 : 0;
    mount_control.pitch = 0;
    // 主循环
    while(ros::ok()) {
        mount_control_pub.publish(mount_control);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}