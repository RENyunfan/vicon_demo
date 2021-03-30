#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include "Eigen/Dense"
#include "polar_controller/polar_controller.hpp"

typedef Eigen::Matrix<double,3,1> Vec3;
struct state{
    Vec3 position;
    double yaw;
};
PolarController::Ptr controller_;
nav_msgs::Odometry cur_odom;
ros::Publisher end_point_state_pub, vel_pub;
Vec3 target;
void odom_callback(const nav_msgs::OdometryConstPtr & msg){
    cur_odom = *msg;
}
void target_callback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    Vec3 cur_pt = Vec3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    if(cur_pt.z( )<1e-3)
        cur_pt.z() = 2.5;
    ROS_INFO("Get target at %lf,%lf,%lf",msg->pose.position.x, msg->pose.position.y, cur_pt.z() );

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

    target = cur_pt;
    target.z() = y;
}

void controller_callback(const ros::TimerEvent & e){
    geometry_msgs::Twist cur_twist = controller_->positionControl(cur_odom,target);
    vel_pub.publish(cur_twist);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "~");
	ros::NodeHandle nh;
    controller_.reset(new PolarController);
	controller_->init(nh);
    ros::Subscriber waypoints_sub = nh.subscribe("/goal",1,target_callback);
	ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom",1,odom_callback);
    ros::Timer controller_timer_ = nh.createTimer(ros::Duration(0.01), controller_callback);
    end_point_state_pub = nh.advertise<geometry_msgs::PoseStamped>("/end_point_vis",10);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Duration(0.1).sleep();
    ros::MultiThreadedSpinner s(3);
    ros::spin(s);
}
