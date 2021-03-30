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
    inline void cap_range( double &val, double val_min, double val_max ){
        if( val > val_max ) val = val_max;
        if( val < val_min ) val = val_min;
    }
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
        nh_.param<double>("/controller/pid/alpha",cp_.k_alpha,-0.1);
        nh_.param<double>("/controller/pid/beta",cp_.k_beta,-0.1);
        nh_.param<double>("/controller/pid/dist",cp_.k_dis,-0.1);
        nh_.param<double>("/controller/limit/ang",cp_.limit_ang,-0.1);
        nh_.param<double>("/controller/limit/vel",cp_.limit_vel,-0.1);
        ROS_INFO("CONTROLLER INIT SUCCESS, with k_alpha = %lf, k_beta = %lf, k_dis = %lf, limit_vel = %lf, limit_ang = %lf.",
                 cp_.k_alpha,cp_.k_beta,cp_.k_dis,cp_.limit_vel,cp_.limit_ang);
    }

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
	//if(err_distance<0.3) cur_alpha=cp_.k_alpha*err_distance/0.3;
        double output_ang = (err_ang_alpha * cp_.k_alpha+ err_ang_beta * cp_.k_beta);
        double output_vel = err_distance * cp_.k_dis;
        cap_range(output_ang, -3,3);
        vel_msg.angular.z =  output_ang>cp_.limit_ang?cp_.limit_ang:output_ang;
        vel_msg.linear.x = output_vel>cp_.limit_vel?cp_.limit_vel:output_vel;
        return vel_msg;
    }
};

#endif //VICON_REPUB_POLAR_CONTROLLER_HPP
