/*
 * Author:Jack Ju
 * Function:实现对imu和GPS的数据处理，然后从特定起点规划到特定终点
 * HIT
 * 20200712
 */
 
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Twist.h"
#include <math.h>

using namespace std;
/*****初始位置姿态设置和目标点设置*****/
double x=1;
double y=1;//起点坐标
double theta=3.1415926/2;//初始航向角
double x_goal=10;
double y_goal=10;
double T=1/50;//采样频率，imu发布话题的频率为50HZ
double L=4.0;//两个推进器间的距离
//pid控制器参数设置
    //航向控制参数设置
double k=10;
double ki=0.1;
double kd=0.1;
double error_sum=0;
double pre_error=0;
//距离控制参数设置
 double k2=10;
//是否初始化
bool isinitized=false;
double w;
double v_x;
double v_y;
// 接收到订阅的消息后，会进入消息回调函数
void imuInfoCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // 将接收到的消息打印出来
    //ROS_INFO("Subcribe Imu Info: x:%d  y:%d z:%d", 
	//msg->linear_acceleration.x, msg->linear_acceleration.y,x);
    cout<<"imu:"<<msg->linear_acceleration.x<<endl;
   // cout<<"test:"<<x<<endl;
if(!isinitized)
{

//航向角度闭环pid控制
    double theta_error;
    theta_error=theta-atan((y_goal-y)/(x_goal-x));
    error_sum=error_sum+theta_error;
    w=-k*theta_error+ki*error_sum+kd*(theta_error-pre_error);
    pre_error=theta_error;

   //速度pid控制
   
double dist=sqrt((x-x_goal)*(x-x_goal)+(y-y_goal)*(y-y_goal));
double v=k2*dist;
v_x=v*cos(theta);
v_y=v*sin(theta);
isinitized=true;
}
else
{
    double angular_z=(msg->angular_velocity.z);
    theta=theta+angular_z*T;//航向角推算
    /*****位置推算*****/
    double  x_acc=(msg->linear_acceleration.x);
    double  y_acc=(msg->linear_acceleration.y);
    v_x=v_x+x_acc*T;
    v_y=v_y+y_acc*T;
    x=x+v_x*T;//推算出的x坐标
    y=y+v_y*T;//推算出的y坐标

//航向角度闭环pid控制
    double theta_error;
    theta_error=theta-atan((y_goal-y)/(x_goal-x));
    error_sum=error_sum+theta_error;
    w=-k*theta_error+ki*error_sum+kd*(theta_error-pre_error);
    pre_error=theta_error;

   //速度pid控制
   
double dist=sqrt((x-x_goal)*(x-x_goal)+(y-y_goal)*(y-y_goal));
double v=k2*dist;
v_x=v*cos(theta);
v_y=v*sin(theta);
isinitized=true;
}
   
}


// 接收到订阅的消息后，会进入消息回调函数
void gpsInfoCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    // 将接收到的消息打印出来
  //  ROS_INFO("Subcribe gps Info: x:%d  y:%d test:%d", 
	//		 msg->latitude, msg->longitude,x);
    cout<<"gps:"<<msg->latitude<<endl;
    cout<<"test:"<<x<<endl;
}
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "imu_sudcriber");
    ros::init(argc, argv, "gps_sudcriber");
    ros::init(argc, argv, "Twist_publisher");
    // 创建节点句柄
    ros::NodeHandle n;
    
   // 创建一个Publisher，发布名为/person_info的topic，消息类型为learning_topic::Person，队列长度10
    ros::Publisher Twist_info_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    ros::Subscriber gps_info_sub = n.subscribe("/fix", 10, gpsInfoCallback);

    // 创建一个Subscriber，订阅名为/imu的topic，注册回调函数personInfoCallback
    ros::Subscriber imu_info_sub = n.subscribe("/imu", 10, imuInfoCallback);
    
    /******讲控制指令转换成速度信息发布出去**********/
   
     geometry_msgs::Twist vel_pub;
     vel_pub.angular.z=w;
     vel_pub.linear.x=v_x;
     vel_pub.linear.y=v_y;
     Twist_info_pub.publish(vel_pub);

    // 循环等待回调函数
    ros::spin();

    return 0;
}
