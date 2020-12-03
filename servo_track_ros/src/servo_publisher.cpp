#include <iostream>
#include <ros/ros.h>
#include <string.h>
#include <tf/transform_broadcaster.h>
#include "SCServo.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"
//带时间戳的Vector3消息
#include "geometry_msgs/Vector3Stamped.h"

using namespace std;

SMSBL servo_bottom;
SMSBL servo_top;

std::string servo_name;
const float pi = 3.14159;

float BtmServoCurrentDegree = -1;
float TopServoCurrentDegree = -1;

float bottom;
float top;

geometry_msgs::Vector3Stamped current;
ros::Publisher currentServoDegree;

void poseCallback(const geometry_msgs::Vector3 theta)
{   
    //底部舵机角度换算
    bottom = theta.x * (4095.0/360.0);
    cout << "bottom:" << bottom << endl;

    //顶部舵机角度换算
    top = theta.y * (4095.0/360.0);
    cout << "top:" << top << endl;

    //底部舵机控制
    servo_bottom.WritePosEx(1, int(bottom), 80, 100);
    std::cout<< "Bottom servo pos ="<< bottom <<std::endl;
	
	//顶部舵机控制
    servo_top.WritePosEx(2, int(top), 80, 100);
	std::cout<< "Top servo pos ="<< top <<std::endl;

    if(servo_top.FeedBack(2) != -1)
    {
        //读取舵机当前角度
        TopServoCurrentDegree = servo_top.ReadPos(2);
        BtmServoCurrentDegree = servo_bottom.ReadPos(1);

        //将当前角度进行换算
        TopServoCurrentDegree = TopServoCurrentDegree*(360.0/4095.0);
        BtmServoCurrentDegree = BtmServoCurrentDegree*(360.0/4095.0);

        //将当前角度写入消息
        current.vector.x = BtmServoCurrentDegree;
        current.vector.y = TopServoCurrentDegree;
        current.header.stamp = ros::Time::now();

        //发布当前角度的消息
        currentServoDegree.publish(current);

        usleep(10*1000);
    }

	ROS_INFO("Successfully received:btm_servo [%f] degree,top_servo[%f] degree",theta.x, theta.y);

    cout << "Bottom Current degree:" <<  BtmServoCurrentDegree;
    cout << "     Top Current degree:" << TopServoCurrentDegree << endl;
    cout << endl;
    cout << "current time:" << current.header.stamp;

}

int main(int argc, char **argv){

    servo_name = "servo";

    if(argc<2){
        std::cout<< "argc error!"<<std::endl;
        return 0;
	}

    if(!servo_bottom.begin(115200, argv[1])){
        std::cout<< "Failed to init smsbl motor!"<<std::endl;
        return 0;
    }

    if(!servo_top.begin(115200, argv[1])){
        std::cout<< "Failed to init smsbl motor!"<<std::endl;
        return 0;
    }

    //初始化ros节点
    ros::init(argc, argv, "servo_publisher");
    //创建节点句柄
    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe<geometry_msgs::Vector3>("/current_servo/pose_setpoint", 10, &poseCallback);

    //创建输出当前舵机角度的发布器
    currentServoDegree = node.advertise<geometry_msgs::Vector3Stamped> ("current_servo/pose",1000);

    // 循环等待回调函数
	ros::spin();

    return 0;

}