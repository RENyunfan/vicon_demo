#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include "visualization_msgs/Marker.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher odom_pub, map_pub;
ros::Timer visual_map;
double size_x,size_y,init_x,init_y,map_width;

void vicon_callback(const geometry_msgs::TransformStamped::ConstPtr & msg){
	nav_msgs::Odometry odom_;
	odom_.pose.pose.position.x = msg-> transform. translation.x;
	odom_.pose.pose.position.y = msg-> transform. translation.y;
	odom_.pose.pose.position.z = msg-> transform. translation.z;
	
	odom_.pose.pose.orientation = msg-> transform.rotation;
	odom_.header.stamp = ros::Time::now();
	odom_.header.frame_id = "world";
	odom_pub.publish(odom_);
	
}

void map_callback(const ros::TimerEvent & e){
    pcl::PointXYZ pt_random;
    pcl::PointCloud<pcl::PointXYZ> pc;
    for(double i = -init_x ; i <= size_x - init_x; i+=map_width){
        // Create the vertices for the points and lines
        geometry_msgs::Point p;
        pt_random.x = i;
        pt_random.y = -init_y;
        pt_random.z = 0;
        pc.points.push_back(pt_random);


        pt_random.x = i;
        pt_random.y = size_y-init_y;
        pt_random.z = 0;
        pc.points.push_back(pt_random);
    }
    // Draw line 1
    for(double i = -init_y ; i <= size_y - init_y; i+=map_width){
        geometry_msgs::Point p;
        pt_random.x = -init_x;
        pt_random.y = i;
        pt_random.z = 0;
        pc.points.push_back(pt_random);

        pt_random.x = size_x-init_x;
        pt_random.y = i;
        pt_random.z = 0;
        pc.points.push_back(pt_random);
    }
    pc.width = pc.points.size();
    pc.height = 1;
    pc.is_dense = true;
    sensor_msgs::PointCloud2 globalMap_pcd;
    pcl::toROSMsg(pc, globalMap_pcd);
    globalMap_pcd.header.frame_id = "world";
    map_pub.publish(globalMap_pcd);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vicon_repub");
	ros::NodeHandle nh("~");
	ros::Subscriber vicon_sub = nh.subscribe<geometry_msgs::TransformStamped>("/vicon_input", 1, vicon_callback);
	nh.param<double>("/map/size_x",size_x,5.0);
    nh.param<double>("/map/size_y",size_y,5.0);
    nh.param<double>("/map/init_x",init_x,5.0);
    nh.param<double>("/map/init_y",init_y,5.0);
    nh.param<double>("/map/map_width",map_width,0.2);
	odom_pub = nh.advertise<nav_msgs::Odometry>("/vicon_output",1);
    map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_boundary",1);
	visual_map = nh.createTimer(ros::Duration(0.1), map_callback);
	ros::spin();
}
