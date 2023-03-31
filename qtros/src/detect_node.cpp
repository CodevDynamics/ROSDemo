// ROS 头文件
#include <ros/ros.h>
// 用于订阅来自地面站的目标选择消息
#include <codevqtros_msgs/SetObject.h>
// 用于发布识别算法推算结果给地面站
#include <codevqtros_msgs/DetectObject.h>
// 生成随机数的头文件
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// ROS 订阅来自地面站的目标选择消息的回调函数
void set_object_cb(const codevqtros_msgs::SetObject::ConstPtr& msg){
    ROS_INFO("Received Set Object Message from Fly: (%f, %f, %f, %f)", msg->x, msg->y, msg->width, msg->height);
}

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "detect_node");
    ros::NodeHandle nh;

    // ROS 订阅来自地面站的目标选择消息
    ros::Subscriber set_object_sub = nh.subscribe<codevqtros_msgs::SetObject>
            ("codevqtros/detect/SetObject", 1, set_object_cb);
    // ROS Topic 发布 Offboard 控制坐标点
    ros::Publisher detect_object_pub = nh.advertise<codevqtros_msgs::DetectObject>
            ("codevqtros/detect/Objects", 20);

    // 设定值发布速率 20 hz
    ros::Rate rate(20.0);

    // 计数，到底一定数量后清除地面站上的结果
    int count = 0;

    // 主循环
    while(ros::ok()){
        // 获得开启还是关闭参数
        bool enabled_;
		nh.getParam("codevqtros/detect/enabled", enabled_);
        // 如果开启则运行
        if(enabled_) {
            // 生成x,y两个0～100以内的随机数
            int x,y;
            srand(count);
            x = rand() % 100;
            srand(x);
            y = rand() % 100;

            // 模拟发出识别推算的目标
            codevqtros_msgs::DetectObject msg;
            // count 大于20，模拟一帧推算结束，清除过期数据
            // 发送 x=y=width=height=0 即可清除
            if(count++ >= 20) {
                count = 0;
                msg.x = 0;
                msg.y = 0;
                msg.width = 0;
                msg.height = 0;
            } else {
                msg.x = x / 100.0f;
                msg.y = y / 100.0f;
                msg.width = 0.1f;
                msg.height = 0.3f;
            }
            msg.objectId = "Apple";
            // 颜色顺序 #[Alpha][Red][Green][Blue]
            msg.rgba = "#1100ff00";
            detect_object_pub.publish(msg);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}