#include "mavros_ned/MavrosNED.h"

namespace mavros_ned
{
	MavrosNED::MavrosNED() :
	    nh_(ros::NodeHandle()),
	    nh_private_("~")
	 {
	 	// Configure Rotations
	 	R_flu_frd_.setZero();
	 	R_flu_frd_(0,0) = 1.0;
	 	R_flu_frd_(1,1) = -1.0;
	 	R_flu_frd_(2,2) = -1.0;

	 	R_enu_ned_.setZero();
	 	R_enu_ned_(0,1) = 1.0;
	 	R_enu_ned_(1,0) = 1.0;
	 	R_enu_ned_(2,2) = -1.0;

	 	// Zero out other Eigen Matrices
	 	eulerFlu_.setZero();
        positionEnu_.setZero();
        velLinRfu_.setZero();
        velAngFlu_.setZero();

        eulerFrd_.setZero();
        positionNED_.setZero();
        velLinFrd_.setZero();
        velAngFrd_.setZero();

	 	// Register ROS Publisher
	 	estimate_pub_ = nh_private_.advertise<nav_msgs::Odometry>("estimate", 1);
	 	euler_pub_ = nh_private_.advertise<geometry_msgs::Vector3Stamped>("euler", 1);

	 	// Register ROS Subscribers
	 	odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 1, &MavrosNED::odomCallback, this);
	 }

	 //
	 // Private Methods
	 //

	 void MavrosNED::odomCallback(const nav_msgs::OdometryConstPtr& msg)
	 {
	 	// Pull off the ENU position data
	 	positionEnu_(0,0) = msg->pose.pose.position.x;
	 	positionEnu_(1,0) = msg->pose.pose.position.y;
	 	positionEnu_(2,0) = msg->pose.pose.position.z;

        // Rotate into NED frame
        positionNED_ = enuToNed(positionEnu_);

        // Pull off the ENU orientation data
        tf::quaternionMsgToTF(msg->pose.pose.orientation, tf_quat_);
        tf::Matrix3x3(tf_quat_).getRPY(phi_, theta_, psi_);
        eulerFlu_(0,0) = phi_;
        eulerFlu_(1,0) = theta_;
        eulerFlu_(2,0) = psi_;

        // Rotate into NED frame
        eulerFrd_ = fluToFrd(eulerFlu_);

        // Convert back to quaternion
        // quatFrd_ = tf::createQuaternionFromRPY(eulerFrd_(0,0), eulerFrd_(1,0), eulerFrd_(2,0));
        quatFrd_ = tf::createQuaternionMsgFromRollPitchYaw(eulerFrd_(0,0), eulerFrd_(1,0), eulerFrd_(2,0) + 1.5707963);

        // Pull off the RFU linear velocity data
	 	velLinRfu_(0,0) = msg->twist.twist.linear.x;
	 	velLinRfu_(1,0) = msg->twist.twist.linear.y;
	 	velLinRfu_(2,0) = msg->twist.twist.linear.z;

	 	// Rotate it into the FRD frame
	 	velLinFrd_ = fluToFrd(velLinRfu_);

	 	// Pull off the RFU angular velocity data
	 	velAngFlu_(0,0) = msg->twist.twist.angular.x;
	 	velAngFlu_(1,0) = msg->twist.twist.angular.y;
	 	velAngFlu_(2,0) = msg->twist.twist.angular.z;

	 	// Rotate it into the FRD frame
	 	velAngFrd_ = fluToFrd(velAngFlu_);

        // Publish estimate data
        publishEstimate();
	 }


	 void MavrosNED::publishEstimate()
	 {
	 	// Fill out the estimate message
	 	// Header
	 	estimate_msg_.header.stamp = ros::Time::now();

	 	// Position Data
	 	estimate_msg_.pose.pose.position.x = positionNED_(0,0);
	 	estimate_msg_.pose.pose.position.y = positionNED_(1,0);
	 	estimate_msg_.pose.pose.position.z = positionNED_(2,0);

	 	// Orientation Data
	 	estimate_msg_.pose.pose.orientation = quatFrd_;

	 	// Linear Velocity Data
	 	estimate_msg_.twist.twist.linear.x = velLinFrd_(0,0);
	 	estimate_msg_.twist.twist.linear.y = velLinFrd_(1,0);
	 	estimate_msg_.twist.twist.linear.z = velLinFrd_(2,0);

	 	// Angular Velocity Data
	 	estimate_msg_.twist.twist.angular.x = velAngFrd_(0,0);
	 	estimate_msg_.twist.twist.angular.y = velAngFrd_(1,0);
	 	estimate_msg_.twist.twist.angular.z = velAngFrd_(2,0);

	 	// Fill out the euler message
	 	// Header
	 	euler_msg_.header.stamp = estimate_msg_.header.stamp;

	 	// Euler Angles (RPY)
	 	euler_msg_.vector.x = eulerFrd_(0,0);
	 	euler_msg_.vector.y = eulerFrd_(1,0);
	 	euler_msg_.vector.z = eulerFrd_(2,0);

	 	// Publish
	 	estimate_pub_.publish(estimate_msg_);
	 	euler_pub_.publish(euler_msg_);
	 }


	 Eigen::Matrix<double, 3, 1> MavrosNED::enuToNed(const Eigen::Matrix<double, 3, 1>& enuVec)
	 {
	 	return R_enu_ned_ * enuVec;
	 }


	 Eigen::Matrix<double, 3, 1> MavrosNED::fluToFrd(const Eigen::Matrix<double, 3, 1>& fluVec)
	 {
	 	return R_flu_frd_ * fluVec;
	 }
}
