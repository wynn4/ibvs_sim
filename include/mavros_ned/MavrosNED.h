#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/tf.h>

namespace mavros_ned
{
	class MavrosNED
	{
	public:
		MavrosNED();

	private:
		// ROS node handles
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        // ROS publisher
        ros::Publisher estimate_pub_;
        ros::Publisher euler_pub_;

        // ROS subscribers
        ros::Subscriber odom_sub_;

        // Static Rotations
        Eigen::Matrix3d R_flu_frd_;
        Eigen::Matrix3d R_enu_ned_;

        // Estimate Message
        nav_msgs::Odometry estimate_msg_;

        // Euler Message
        geometry_msgs::Vector3Stamped euler_msg_;

        // Eigen Matrices to hold incoming mavros data
        Eigen::Matrix<double, 3, 1> eulerFlu_;
        Eigen::Matrix<double, 3, 1> positionEnu_;
        Eigen::Matrix<double, 3, 1> velLinRfu_;
        Eigen::Matrix<double, 3, 1> velAngFlu_;

        // Eigen Matrices to hold NED/FRD data to be published
        Eigen::Matrix<double, 3, 1> eulerFrd_;
        Eigen::Matrix<double, 3, 1> positionNED_;
        Eigen::Matrix<double, 3, 1> velLinFrd_;
        Eigen::Matrix<double, 3, 1> velAngFrd_;

        // tf::Quaternion quatFrd_;
        geometry_msgs::Quaternion quatFrd_;

        //
        // Methods
        //

        // mavros odom subscriber
        void odomCallback(const nav_msgs::OdometryConstPtr& msg);

        void publishEstimate();

        // rotation functions
        Eigen::Matrix<double, 3, 1> enuToNed(const Eigen::Matrix<double, 3, 1>& enuVec);
        Eigen::Matrix<double, 3, 1> fluToFrd(const Eigen::Matrix<double, 3, 1>& fluVec);
	};
}
