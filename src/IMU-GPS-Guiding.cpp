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
#include "sensor_msgs/LaserScan.h"
#include <boost/thread.hpp>
//#include<subscriber.h>
#include "sensor_msgs/Image.h"
using namespace std;

class IMU_GPS_Guiding
{
private:
    /* data */
    ros::NodeHandle n;
    ros::Publisher Twist_info_pub;
    ros::Subscriber gps_info_sub;
    ros::Subscriber imu_info_sub;
    ros::Subscriber lidar_info_sub;
    ros::Subscriber camera_info_sub;
    double gps_x;
    double gps_y;
    /*****初始位置姿态设置和目标点设置*****/
    double x=0;
    double y=0;//起点坐标
    double theta=3.1415926/2;//初始航向角
    double x_goal=10;
    double y_goal=10;
    double t=0.02;//采样频率，imu发布话题的频率为50HZ
    double L=4;//两个推进器间的距离
    //pid控制器参数设置
        //航向控制参数设置
    double k=10;
    double ki=0.1;
    double kd=0.1;
    double error_sum=0;
    double pre_error=0;
    //距离控制参数设置
    double k2=0.1;
    //是否初始化
    bool isinitized=false;
    double w;
    double v_x;
    double v_y;

public:
    IMU_GPS_Guiding(/* args */);
    void imuInfoCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void gpsInfoCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void scanInfoCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void cameraInfoCallback(const sensor_msgs::Image::ConstPtr& msg);

    ~IMU_GPS_Guiding();
    void initized_and_parameter_setting();
};
IMU_GPS_Guiding::IMU_GPS_Guiding(/* args */)
{
    //Twist_info_pub=n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
   // gps_info_sub=n.subscribe("/fix",10,&IMU_GPS_Guiding::gpsInfoCallback,this);
    //imu_info_sub=n.subscribe("/imu",10,&IMU_GPS_Guiding::imuInfoCallback,this);
    //lidar_info_sub=n.subscribe("/scan",10,&IMU_GPS_Guiding::scanInfoCallback,this);
    camera_info_sub=n.subscribe("/kinect2/hd/image_color",10,&IMU_GPS_Guiding::cameraInfoCallback,this);
}

IMU_GPS_Guiding::~IMU_GPS_Guiding()
{
}

void IMU_GPS_Guiding::initized_and_parameter_setting()
{

 //isinitized=false;
}





// 接收到订阅的消息后，会进入消息回调函数
void IMU_GPS_Guiding::imuInfoCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // 将接收到的消息打印出来
    //ROS_INFO("Subcribe Imu Info: x:%d  y:%d z:%d", 
	//msg->linear_acceleration.x, msg->linear_acceleration.y,x);
   // cout<<"imu:"<<msg->linear_acceleration.x<<endl;
   // cout<<"test:"<<x<<endl;
if(!this->isinitized)
{
  cout<<"初始化中"<<endl;
//航向角度闭环pid控制
    double theta_error;
    theta_error=this->theta-atan((this->y_goal-this->y)/(this->x_goal-this->x));//防止分母为零。
    this->error_sum=this->error_sum+theta_error;
    this->w=-this->k*theta_error+this->ki*this->error_sum+this->kd*(theta_error-this->pre_error);
    this->pre_error=theta_error;

//速度pid控制
double dist=sqrt((this->x-this->x_goal)*(this->x-this->x_goal)+(this->y-this->y_goal)*(this->y-this->y_goal));
double v=k2*dist;
this->v_x=v*cos(this->theta);
this->v_y=v*sin(this->theta);
this->isinitized=true;
}
else
{
    double angular_z=(msg->angular_velocity.z)-0.01;//0.01是对imu偏差的补偿
    this->theta=this->theta+angular_z*this->t;//航向角推算
    /*****位置推算*****/
    double  x_acc_rece=(msg->linear_acceleration.x)-0.0337;//0.3是我补偿
    double  y_acc_rece=(msg->linear_acceleration.y)-0.7182;//0.46是我补偿
    //坐标变换：将加速度计的信息从小车坐标系转换到大地坐标系
    double x_acc=x_acc_rece*cos(this->theta)-y_acc_rece*sin(this->theta);
    double y_acc=x_acc_rece*sin(this->theta)+y_acc_rece*cos(this->theta);
    
    //开始推算坐标
    this->v_x=this->v_x+x_acc*this->t;       
    this->v_y=this->v_y+y_acc*this->t;
    this->x=this->x+this->v_x*this->t;//推算出的x坐标
    this->y=this->y+this->v_y*this->t;//推算出的y坐标
    cout<<"ax:"<<x_acc<<endl;
    cout<<"T:"<<this->t<<endl;
    cout<<endl;
    cout<<"初始化已经结束"<<endl;
    cout<<"gpx_x:"<<this->gps_x<<"gps_y"<<this->gps_y<<endl;
//航向角度闭环pid控制
    double theta_error;
    theta_error=this->theta-atan((this->y_goal-this->y)/(this->x_goal-this->x));
    this->error_sum=this->error_sum+theta_error;
    this->w=-this->k*theta_error+this->ki*this->error_sum+this->kd*(theta_error-this->pre_error);
    this->pre_error=theta_error;

   //速度pid控制
   
double dist=sqrt((this->x-this->x_goal)*(this->x-this->x_goal)+(this->y-this->y_goal)*(this->y-this->y_goal));
double v=k2*dist;
this->v_x=v*cos(this->theta);
this->v_y=v*sin(this->theta);
}



/***设定运动停止*****/
double dist_compare=sqrt((this->x-this->x_goal)*(this->x-this->x_goal)+(this->y-this->y_goal)*(this->y-this->y_goal));
if(dist_compare<1)
{
 this->v_x=0;
 this->v_y=0;
 this->w=0;
 imu_info_sub.shutdown();//当到达目标，不会去订阅imu话题。
}


  // ros::init(argc, argv, "Twist_publisher");
  /*
 ros::NodeHandle n;
 ros::Publisher Twist_info_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
*/
 geometry_msgs::Twist vel_pub;

     vel_pub.angular.z=this->w;
     vel_pub.linear.x=this->v_x;
     vel_pub.linear.y=this->v_y;
     Twist_info_pub.publish(vel_pub);
    // Twist_info_pub.publish(vel_pub);
    cout<<"x:"<<this->x<<endl;
    cout<<"y:"<<this->y<<endl;
    cout<<"v_x"<<this->v_x<<endl;
    cout<<"v_y"<<this->v_y<<endl;
    cout<<"theta:"<<this->theta<<endl;
   
}


// 接收到订阅的消息后，会进入消息回调函数
 void IMU_GPS_Guiding::gpsInfoCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    // 将接收到的消息打印出来
  //  ROS_INFO("Subcribe gps Info: x:%d  y:%d test:%d", 
	//		 msg->latitude, msg->longitude,x);
    //cout<<"gps:"<<msg->latitude<<endl;
    //cout<<"test:"<<x<<endl;
    gps_x=msg->latitude;
    gps_y=msg->longitude;
}


// 接收到订阅的消息后，会进入消息回调函数
void IMU_GPS_Guiding::scanInfoCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // 将接收到的消息打印出来
  //  ROS_INFO("Subcribe gps Info: x:%d  y:%d test:%d", 
	//		 msg->latitude, msg->longitude,x);
//for(int i=0;i<msg->ranges.size();i++)
//通过测试s1雷达ranges[]长度为720
  cout<<"lidar_length:"<<msg->ranges.size()<<endl;
  cout<<"我认为的正前方=ranges[0]:"<<msg->ranges[0]<<endl;
  cout<<"我认为的正前方=ranges[720]:"<<msg->ranges[720]<<endl;
  cout<<"我认为的左侧方=ranges[180]："<<msg->ranges[180]<<endl;
  cout<<"我认为的左侧方=ranges[270]："<<msg->ranges[180]<<endl;
  cout<<"我认为的右侧方=ranges[540]:"<<msg->ranges[540]<<endl;
  cout<<"我认为的后方方=ranges[360]:"<<msg->ranges[360]<<endl;

   // cout<<"test:"<<x<<endl;
}

// 接收到订阅的消息后，会进入消息回调函数
void IMU_GPS_Guiding::cameraInfoCallback(const sensor_msgs::Image::ConstPtr &msg)
{
cout<<"图像高度"<<msg->height<<endl;
cout<<"图像宽度："<<msg->width<<endl;

cout<<"data矩阵大小"<<msg->data.size()<<endl;
cout<<"内容"<<msg->data[1]<<endl;


}







int main(int argc, char **argv)
{
    // 初始化ROS节点
    
    ros::init(argc, argv, "Guide_sudcriber");
   
     
    // 创建节点句柄
    ros::NodeHandle n;
    
   // 创建一个Publisher，发布名为/Twist_info_pub的topic，消息类型为geometry_msgs::Twist，队列长度10
   
    
   // ros::Subscriber gps_info_sub = n.subscribe("/fix", 10, gpsInfoCallback);

    // 创建一个Subscriber，订阅名为/imu的topic，注册回调函数imuInfoCallback
   // ros::init(argc, argv, "imu_sudcriber");
   // ros::Subscriber imu_info_sub = n.subscribe("/imu", 10, imuInfoCallback);
    
     // 创建一个Subscriber，订阅名为/imu的topic，注册回调函数personInfoCallback
     //ros::init(argc, argv, "Lidar_sudcriber");
   // ros::Subscriber lidar_info_sub = n.subscribe("/scan", 10, scanInfoCallback);
   // ros::Publisher Twist_info_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
       // 设置循环的频率
    //ros::Rate loop_rate(1);
     //while (ros::ok())
     //{

      /******讲控制指令转换成速度信息发布出去**********/
   /*
     geometry_msgs::Twist vel_pub;
     vel_pub.angular.z=w;
     vel_pub.linear.x=v_x;
     vel_pub.linear.y=v_y;
     Twist_info_pub.publish(vel_pub);
    cout<<"x:"<<x<<endl;
    cout<<"y:"<<y<<endl;
    cout<<"theta:"<<theta<<endl;
    */
    // 循环等待回调函数
   // ros::spin();

     ///}
   IMU_GPS_Guiding test;
    //ros::spin();
    ros::MultiThreadedSpinner spinner(4);//三个线程订阅
    spinner.spin();


    return 0;
}