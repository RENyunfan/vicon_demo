[toc]

> 第一章 动捕系统与机器人的通信
>
> 1.1 坐标系设置
>
> 1.2 机器人初始位置设定
>
> 1.3 网络设置（动捕网络、机器人网络）
>
> 1.4 使用SDK读取动捕数据
>
> 1.5 机器人运动姿态数据的导出（从动捕中导出机器人的运动轨迹数据）
>
> 第二章 使用动捕作为反馈实现机器人定点控制
>
> 整体实现方案的详细描述（作为一章的引言总体写）
>
> 2.1 vicon_bridge
>
> 1、功能说明
>
> 2、代码结构及分析
>
> 2.2 vicon_repub
>
> 1、功能说明
>
> 2、代码结构及分析
>
> 2.3 polar_controller
>
> 1、功能说明
>
> 2、代码结构及分析
>
> 2.4 joy2twist-master
>
> 1、功能说明
>
> 2、代码结构及分析
>
> 2.5 CMakeLists

# 第一章 动捕系统与机器人的通信

## 1.1 坐标系设置

首先启动`vicon`软件，等待Vicon设备初始化完成。



随后点击校准



将动捕标定杆放到希望设置的世界坐标系原点位置，标定杆的方向朝向期望的世界坐标系朝向。

点击设定坐标系原点



再次点击，设置坐标轴朝向。



图片中动捕摄像头的位置与坐标系相对位置一致，则设定完成。

## 1.2 机器人初始位置设定

在机器人上贴至少三个动捕标签，尽量使图形不对称，将其中一个动捕小球贴到机器人坐标原点



将机器人放到动捕范围内，将机身坐标系和世界坐标系对齐。



在vicon软件中，按住shif拖动鼠标，选中机器人上所有动捕标签



点击创建刚体。



创建完成后选择该刚体，点击暂停，选择第一步作为原点的动捕小球，将xyz的偏置设为0，选择姿态全部设为0.



观察机器人坐标系和期望一致，则设置完成。

## 1.3 网络设置（动捕网络、机器人网络）

在1.1和1.2完成后，机器人的位姿已经通过vicon的特殊协议，通过TCP-IP发送到vicon电脑所在的局域网中。接下来只需要将局域网中的数据读出。



打开机器人车载电脑，连接到同一局域网(wifi名：zdh)，连接后如2.2中操作，即可实现对动捕数据的读取。



## 1.4 机器人运动姿态数据的导出（从动捕中导出机器人的运动轨迹数据）



# 第二章 使用动捕作为反馈实现机器人定点控制

在控制与机器人领域，动作捕捉系统通常作为一种高速，高精度已经可靠的位姿传感。本章将详细介绍利用动作捕捉系统作为位置反馈，实现差速移动机器人定点控制的细节。

本功能包中包含四个`rospackage`，分别如下

```bash
├── joy2twist
├── polar_controller
├── vicon_bridge
└── vicon_repub
```

## 2.0 安装依赖

`joy2Twist`使用前需要安装依赖

```bash
sudo apt-get install ros-melodic-joystick-*
```

注意，该功能包需要与支持`joystick`的遥控器硬件配合使用。

随后安装Eigen库

```bash
sudo apt-get install libegin3
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
```



## 2.1 joy2twist

#### 功能说明

`joy2twist`是一个关于遥控器公能的话题转换包。功能为将遥控器发出的`sensor_msgs::Joy`类型数据，转换成无人小车需要的控制数据`geometry_msgs/Twist`。

#### 使用方法

打开`./launch/xbox.launch`文件，修改参数如下

```xml
<launch>
    <!-- 根据是否使用时间戳，填写下行value = true或false-->
	<arg name="use_stamped" value="false"> </arg>
    
	<node pkg="joy" name="joy_node" type="joy_node" output="screen" />
    
	<node pkg="joy2twist" name="xbox" type="xbox" output="screen" >
	    <param name="/use_stamped"   value="$(arg use_stamped)"/>
		<!-- 根据需要发布的话题名，填写下行中to后面的内容，例如dashgo小车需要使用的名字为："/smoother_cmd_vel" -->
        <remap from="cmd_vel" to="/smoother_cmd_vel" />
	</node>

</launch>
```

在更改完成后，运行下方命令即可使用

```
roslaunch joy2twist xbox.launch
```

#### 代码说明

核心代码保存于`src/xbox.cpp`，具体功能见注释：

```cpp
void JoyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  /* 	读取sensor_msgs中的Joy类型数据，将遥控器遥控数据映射到Twist类型，用于控制无人机或者无人小车
   *		isStamped 参数表示发布Twist的类型
   * 			当isStamped == treu，表示发布 geometry_msgs/TwistStamped类型
   * 			当isStamped == false，表示发布 geometry_msgs/Twist类型
   */
	if(isStamped){
        geometry_msgs::TwistStamped twist;
        twist.header.frame_id = "world";
        twist.header.stamp = ros::Time::now();
        twist.twist.angular.x=0;
        twist.twist.angular.y=0;
        twist.twist.angular.z=msg->axes[0];
        twist.twist.linear.x=msg->axes[4];
        twist.twist.linear.y=msg->axes[3];
        twist.twist.linear.z=msg->axes[1];
        pub.publish(twist);
	}else{
        geometry_msgs::Twist twist;
        twist.angular.x=0;
        twist.angular.y=0;
        twist.angular.z=msg->axes[0];
        twist.linear.x=msg->axes[4];
        twist.linear.y=msg->axes[3];
        twist.linear.z=0;
        pub.publish(twist);
	}
}
```



## 2.2 vicon_bridge

#### 功能说明

[`vicon_bridge`](https://github.com/ethz-asl/vicon_bridge)是由ETHZ ASL实验室开发的工具包，其功能为，将局域网中共享的`vicon`数据转换为ROS中的话题，发布的消息类型为`geometry_msgs/TransformStamped`。

#### 使用方法

打开`./launch/vicon.launch`文件，修改参数如下

```xml
<launch>
	<node pkg="vicon_bridge" type="vicon_bridge" name="vicon" output="screen">
		<param name="stream_mode" value="ClientPull" type="str" />
		<!--修改下方value后的数值为vicon系统所在的局域网IP地址和端口号（可以从vicon软件所在的电脑上读取），K321实验室的wifi名为zdh，对应ip和端口号如下-->
		<param name="datastream_hostport" value="192.168.0.13:801" type="str" />
		<param name="tf_ref_frame_id" value="/world" type="str" />
		</node>
</launch>
```

在更改完成后，运行下方命令即可在ROS话题广场中获得vicon数据。



## 2.3 vicon_repub

#### 功能说明

在1.2中，我们介绍了如何使用`vicon_bridge`将动捕数据发布到ROS话题广场。然而，发布话题类型`geometry_msgs/TransformStamped`无法在`Rviz`中进行可视化，同时和我们常用的`nav_msgs/Odometry`信息结构上也略有差异，因此我们通过`vicon_repub`功能包，实现换题类型的转换。

除此之外，`vicon_repub`还提供了一个地图边界绘制功能，通过设置`config/map.yaml`中的地图大小，可以在Rviz中可视化一个方框。如无需方框可以将地图大小设置为0。

#### 使用方法

打开`./launch/vicon.launch`文件，修改参数如下

```xml
<launch>

	<rosparam   command="load" file="$(find vicon_repub)/config/map.yaml"/>
    <rosparam   command="load" file="$(find polar_controller)/config/default.yaml"/>
	
    <node pkg="vicon_repub" name="vicon_repub" type="vicon_repub" output="screen" >
        <!-- 修改下方 to 后面的节点名字为vicon节点的名字 -->
        <remap from="/vicon_input" to="/vicon/vehicle/vehicle" />
         <!-- 修改下方 to 后面的节点名字为你想要发布的节点名字 -->
        <remap from="/vicon_output" to="/vicon_odom" />
    </node>

	<node pkg="rviz" name="rviz" type="rviz" args="-d $(find vicon_repub)/config/default.rviz" />

    <include file="$(find joy2twist)/launch/xbox.launch"/>
    <include file="$(find vicon_bridge)/launch/vicon.launch"/>
    <include file="$(find dashgo_driver)/launch/driver.launch"/>
</launch>

```

随后直接使用启动

```bash
roslaunch vicon_repub vicon.launch
```



#### 代码说明

核心代码保存在`src/vicon_repub.cpp`中，具体功能和注释如下

```cpp
void vicon_callback(const geometry_msgs::TransformStamped::ConstPtr & msg){
   /* 	读取geometry_msgs中的TransformStamped类型数据，将vicon发布的位姿数据转化更常用的nav_msgs/Odometry类型
   */
	nav_msgs::Odometry odom_;
	odom_.pose.pose.position.x = msg-> transform. translation.x;
	odom_.pose.pose.position.y = msg-> transform. translation.y;
	odom_.pose.pose.position.z = msg-> transform. translation.z;
	odom_.pose.pose.orientation = msg-> transform.rotation;
	odom_.header.stamp = ros::Time::now();
  /*	设置可视化轨迹的坐标系为世界坐标系 world*/ 
	odom_.header.frame_id = "world";
	odom_pub.publish(odom_);
}
```

即在动捕话题的回调函数中，重新整理消息类型，并发布到ROS话题广场中。

## 2.4 polar_controller

#### 功能说明

`polar_controller`是一个基于极坐标控制器的定点控制功能包。其中提供了一个`PolarController`类，可以提供极坐标定点控制器的计算。

#### 使用方法

打开`./launch/vicon.launch`文件，修改参数如下

```xml
<launch>
	<rosparam   command="load" file="$(find vicon_repub)/config/map.yaml"/>
	<node pkg="polar_controler" type="controller" name="controller">
		<!-- 修改下方 to 为里程计输入的节点名字 -->
        <remap from="/odom" to="/vicon_odom" />
          <!-- 修改下方 to 为Rviz 2D Nav Goal 输入的名字 -->
		<remap from="/goal" to="/move_base_simple/goal" />
	</node>
    
	<include file="$(find vicon_repub)/launch/vicon.launch"/>
	<include file="$(find joy2twist)/launch/xbox.launch"/>
	<include file="$(find vicon_bridge)/launch/vicon.launch"/>
	<include file="$(find dashgo_driver)/launch/driver.launch"/>
</launch>
```

随后直接使用启动

```bash
roslaunch polar_controller polar_controller.launch
```

#### 代码说明

`polar_controller`中提供了一个纯头文件构成的极坐标控制器类，支持用户将该控制器部署到各种设备中

```cpp
//
// Created by yunfan on 2021/3/24.
//

#ifndef VICON_REPUB_POLAR_CONTROLLER_HPP
#define VICON_REPUB_POLAR_CONTROLLER_HPP

#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "Eigen/Core"
#include "tf/tf.h"
#include "memory"
using namespace Eigen;
using namespace std;
class PolarController {
private:
    ros::NodeHandle nh_;
    struct ControllerParam{
        double k_alpha,k_beta,k_dis;
        double limit_vel,limit_ang;

    }cp_;
  	/*	对输出速度大小进行限幅 */
    inline void cap_range( double &val, double val_min, double val_max ){
        if( val > val_max ) val = val_max;
        if( val < val_min ) val = val_min;
    }
  	/*	对输出角速度大小进行限幅 */
    inline void cap_angle(double &val){
        while ( fabs(val) > M_PI && val < 0 ){
            val += 2*M_PI;
        }
        while ( fabs(val) > M_PI && val > 0 ){
            val -= 2*M_PI;
        }
    }

public:
    PolarController(){}
    ~PolarController(){}
    typedef shared_ptr<PolarController> Ptr;
    inline void init(ros::NodeHandle nh){
        nh_ = nh;
     /*	读取PID参数	*/
        nh_.param<double>("/controller/pid/alpha",cp_.k_alpha,-0.1);
        nh_.param<double>("/controller/pid/beta",cp_.k_beta,-0.1);
        nh_.param<double>("/controller/pid/dist",cp_.k_dis,-0.1);
        nh_.param<double>("/controller/limit/ang",cp_.limit_ang,-0.1);
        nh_.param<double>("/controller/limit/vel",cp_.limit_vel,-0.1);
        ROS_INFO("CONTROLLER INIT SUCCESS, with k_alpha = %lf, k_beta = %lf, k_dis = %lf, limit_vel = %lf, limit_ang = %lf.",cp_.k_alpha,cp_.k_beta,cp_.k_dis,cp_.limit_vel,cp_.limit_ang);
    }
		/*	定点控制器算法 */
    inline geometry_msgs::Twist positionControl(nav_msgs::Odometry odom,Vector3d target){
        tf::Quaternion quat;
        tf::quaternionMsgToTF( odom.pose.pose.orientation, quat );
        double yaw,pitch,roll;
        tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);

        Vector3d curPos;
        Vector2d curVel;

        curPos << odom.pose.pose.position.x, odom.pose.pose.position.y, yaw;
        curVel << odom.twist.twist.linear.x, odom.twist.twist.angular.z;

        double err_ang_alpha, err_ang_beta, err_distance;
        err_distance = sqrt( pow(target.y() - curPos.y(), 2) + pow(target.x() - curPos.x(), 2) );
        err_ang_alpha = atan2( target.y() - curPos.y(), target.x() - curPos.x() ) - curPos.z();
        err_ang_beta = target.z() - curPos.z();

        cap_angle(err_ang_beta);
        cap_angle(err_ang_alpha);
        geometry_msgs::Twist vel_msg;
				double cur_alpha = cp_.k_alpha;
        double output_ang = (err_ang_alpha * cp_.k_alpha+ err_ang_beta * cp_.k_beta);
        double output_vel = err_distance * cp_.k_dis;
        cap_range(output_ang, -3,3);
        vel_msg.angular.z =  output_ang>cp_.limit_ang?cp_.limit_ang:output_ang;
        vel_msg.linear.x = output_vel>cp_.limit_vel?cp_.limit_vel:output_vel;
        return vel_msg;
    }
};

#endif //VICON_REPUB_POLAR_CONTROLLER_HPP
```

而在`src/controller.cpp`中，提供了一个使用范例

```cpp
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include "Eigen/Dense"
/*	引用极坐标控制器头文件 */
#include "polar_controller/polar_controller.hpp"

typedef Eigen::Matrix<double,3,1> Vec3;
struct state{
    Vec3 position;
    double yaw;
};
/*	创建全局控制器智能指针 */
PolarController::Ptr controller_;
/*	创建全局里程计反馈数据 */
nav_msgs::Odometry cur_odom;
/*	创建可视化和控制命令发布者 */
ros::Publisher end_point_state_pub, vel_pub;
/*	创建目标位置点 */
Vec3 target;

/*	小车当前里程计反馈，例如使用vicon动捕系统或者激光SLAM系统 */
void odom_callback(const nav_msgs::OdometryConstPtr & msg){
    cur_odom = *msg;
}

/*	定点控制目标回调函数 */
void target_callback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  	/* 使用Eigen中的向量类型保存 */
    Vec3 cur_pt = Vec3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    if(cur_pt.z( )<1e-3)
        cur_pt.z() = 2.5;
    ROS_INFO("Get target at %lf,%lf,%lf",msg->pose.position.x, msg->pose.position.y, cur_pt.z() );
		
    /* 记录并可视化当前目标点 */
    tf::Quaternion quat;
    tf::quaternionMsgToTF( msg->pose.orientation,quat);
    double y,p,r;
    tf::Matrix3x3(quat).getRPY(r,p,y);
    geometry_msgs::PoseStamped point;
    point.pose = msg->pose;
    point.pose.position.z = cur_pt.z();
    point.header.frame_id="world";
    point.header.stamp = ros::Time::now();
    end_point_state_pub.publish(point);
		
  	/*	记录当前目标点位置 */
    target = cur_pt;
    target.z() = y;
}

/*	控制器定时中断，中断频率为100Hz*/ 
void controller_callback(const ros::TimerEvent & e){
    geometry_msgs::Twist cur_twist = controller_->positionControl(cur_odom,target);
    vel_pub.publish(cur_twist);
}

/*	主函数	*/
int main(int argc, char **argv)
{
    /*	初始化ROS服务和节点句柄*/
    ros::init(argc, argv, "~");
    ros::NodeHandle nh;
    /*	初始化控制器*/
    controller_.reset(new PolarController);
    controller_->init(nh);
    /*	创建里程计接收者和目标位置接收者*/
    ros::Subscriber waypoints_sub = nh.subscribe("/goal",1,target_callback);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom",1,odom_callback);
		/*	创建控制器定时器中断 */
    ros::Timer controller_timer_ = nh.createTimer(ros::Duration(0.01), controller_callback);
  	/*	创建当前目标可视化发布者 */
    end_point_state_pub = nh.advertise<geometry_msgs::PoseStamped>("/end_point_vis",10);
    /*	创建当前速度目标发布者 */
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Duration(0.1).sleep();
  	/*	开启多线程回调 */
    ros::MultiThreadedSpinner s(3);
    ros::spin(s);
}

```



