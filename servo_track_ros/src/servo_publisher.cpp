#include <iostream>
#include <ros/ros.h>
#include <string.h>
#include <tf/transform_broadcaster.h>
#include "SCServo.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"

SMSBL servo_bottom;
SMSBL servo_top;

std::string servo_name;
const float pi = 3.14159;



void poseCallback(const geometry_msgs::Vector3 theta)
{
	// 创建tf的广播器
	//static tf::TransformBroadcaster br;

	// 初始化tf数据
	//tf::Transform transform;


	//BOTTOM SERVO 
    servo_bottom.WritePosEx(1, theta.x*(4095/360), 80, 100);
    std::cout<< "Bottom servo pos ="<<theta.x*(180/pi)<<std::endl;
	// transform.setOrigin( tf::Vector3(0, 0, L1) );
	// tf::Quaternion q;
	// q.setRPY(0, 0, theta.z);
	// transform.setRotation(q);

	// br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "servo1"));
	
	
	//TOP SERVO 
    servo_top.WritePosEx(2, theta.y*(4095/360)/3, 80, 100);
	std::cout<< "Top servo pos ="<<theta.y*(180/pi)<<std::endl;
	// tf:: Transform transform2;
	// //transform2.setOrigin( tf::Vector3(L2*sin(theta.y),L2*cos(theta.y)*cos(theta.z), L2*cos(theta.y)*sin(theta.z)));
	// transform2.setOrigin( tf::Vector3(L2*sin(theta.y),0,L2*cos(theta.y)));
	// tf::Quaternion q2;
	// q2.setRPY(0,theta.y,0);
	// transform2.setRotation(q2);

	// // 广播world与海龟坐标系之间的tf数据e

	// br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "servo1", "servo2"));

	ROS_INFO("Successfully received:servo1 [%f] degree,top servo[%f] degree",theta.x, theta.y);
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

    ros::Subscriber sub = node.subscribe<geometry_msgs::Vector3>("/servo/pose", 10, &poseCallback);

    // 循环等待回调函数
	ros::spin();

    return 0;

}


    //创建一个发布者Publisher，消息管道/turtle1/cmd_vel，消息类型std_msgs::String
    //ros::Publisher servo_pub = node.advertise<std_msgs::Float64> ("servo1/pose",1000);
//     ros::Publisher servo_pub = node.advertise<geometry_msgs::Vector3> ("servo/pose",1000);

//     //设置一个循环频率每秒5次
//     ros::Rate loop_rate(5);

//     //计数发布次数
//     int i  = 0;
//     while (ros::ok()){

//         geometry_msgs::Vector3 theta;

//         for(i=90;i<=180;i+=1)
//         {
//             theta.z = i*pi/180;

//             theta.y = i*pi/180;


//             servo_bottom.WritePosEx(1, i*(4095/360), 80, 100);
//             //舵机(ID1)以最高速度V=80(50*80步/秒)，加速度A=100(100*100步/秒^2)，运行至P1=4095位置
// 		    std::cout<< "Bottom servo pos ="<<theta.z*(180/pi)<<std::endl;
// 		    //usleep(400*1000);//[(P1-P0)/(50*V)]*1000+[(50*V)/(A*100)]*1000

//             servo_top.WritePosEx(2, i*(4095/360)/3, 80, 100);
//             //舵机(ID1)以最高速度V=80(50*80步/秒)，加速度A=100(100*100步/秒^2)，运行至P1=4095位置
// 		    std::cout<< "Top servo pos ="<<theta.y*(180/pi)<<std::endl;
// 		    //usleep(400*1000);//[(P1-P0)/(50*V)]*1000+[(50*V)/(A*100)]*1000

//             servo_pub.publish(theta);

//             //ROS_INFO("Successfully published:botteom servo[%f] degree,top servo[%f] degree",theta.z,theta.y);

//         }

//         for(i=180;i>=90;i-=1)
//         {
//             theta.z = i*pi/180;

//             theta.y = i*pi/180;

//             servo_bottom.WritePosEx(1, i*(4095/360), 80, 100);//舵机(ID1)以最高速度V=80(50*80步/秒)，加速度A=100(100*100步/秒^2)，运行至P1=4095位置
// 		    std::cout<< "Bottom servo pos ="<<theta.z*(180/pi) <<std::endl;
// 		    //usleep(400*1000);//[(P1-P0)/(50*V)]*1000+[(50*V)/(A*100)]*1000

//             servo_top.WritePosEx(2, i*(4095/360)/3, 80, 100);//舵机(ID1)以最高速度V=80(50*80步/秒)，加速度A=100(100*100步/秒^2)，运行至P1=4095位置
// 		    std::cout<< "Top servo pos ="<<theta.y*(180/pi) <<std::endl;
// 		    //usleep(400*1000);//[(P1-P0)/(50*V)]*1000+[(50*V)/(A*100)]*1000

//             servo_pub.publish(theta);

//             //ROS_INFO("Successfully published:botteom servo[%f] degree,top servo[%f] degree",theta.z,theta.y);

//         }

//         // servo_bottom.WritePosEx(1, 4095, 80, 100);
//         // //舵机(ID1)以最高速度V=80(50*80步/秒)，加速度A=100(100*100步/秒^2)，运行至P1=4095位置
// 		// std::cout<< "Bottom servo pos ="<<4095<<std::endl;
// 		// usleep(1495*1000);//[(P1-P0)/(50*V)]*1000+[(50*V)/(A*100)]*1000

//         // servo_bottom.WritePosEx(1, 0, 80, 100);
//         // //舵机(ID1)以最高速度V=80(50*80步/秒)，加速度A=100(100*100步/秒^2)，运行至P1=4095位置
// 		// std::cout<< "Bottom servo pos ="<<0<<std::endl;
// 		// usleep(1495*1000);//[(P1-P0)/(50*V)]*1000+[(50*V)/(A*100)]*1000


//         //发布消息
//         //servo_pub.publish(theta);

//         //发布成功在该节点终端显示
//         //已成功发布第%d条消息,发布的内容为%s打印std_msgs::String类型
//         //ROS_INFO("Successfully published:botteom servo[%f] degree,top servo[%f] degree",theta.z,theta.y);

//         loop_rate.sleep();

//     }
//     return 0;
// }
