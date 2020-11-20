
#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "SCServo.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"

std::string servo_name;
const float pi = 3.14159;

int main(int argc, char **argv){
    //初始化ros节点
    ros::init(argc, argv, "servo_publisher");
    //创建节点句柄
    ros::NodeHandle node;

	servo_name = "servo";

    ros::Publisher servo_pub = node.advertise<geometry_msgs::Vector3> ("servo/pose",1000);

    //设置一个循环频率每秒5次
    ros::Rate loop_rate(5);

    //计数发布次数
    float i  = 0;
    while (ros::ok()){
        //初始化消息类型double
        //std_msgs::Float64 theta1;
        geometry_msgs::Vector3 theta;
        
        //std::cout << "Input bottom servo rotation degree:" << std::endl;
        //std::cin >> theta1.data;
        if(i==90)
        {
            i=0;
        }
        i++;
        theta.z = i*pi/180;

        theta.y = i*pi/90;

        //发布消息
        servo_pub.publish(theta);

        //发布成功在该节点终端显示
        //已成功发布第%d条消息,发布的内容为%s打印std_msgs::String类型
        ROS_INFO("Successfully published:botteom servo[%f] degree,top servo[%f] degree",theta.z,theta.y);

        loop_rate.sleep();

    }
    return 0;
}
