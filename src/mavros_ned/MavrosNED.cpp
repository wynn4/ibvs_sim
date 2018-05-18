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

	 	// Register ROS Subscribers
	 	pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 1, &MavrosNED::poseCallback, this);
	 	velocity_sub_ = nh_.subscribe("/mavros/local_position/velocity", 1, &MavrosNED::velocityCallback, this);
	 }

	 //
	 // Private Methods
	 //

	 void MavrosNED::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
	 {
	 	// Pull off the ENU position data
	 	positionEnu_(0,0) = msg->pose.position.x;
	 	positionEnu_(1,0) = msg->pose.position.y;
	 	positionEnu_(2,0) = msg->pose.position.z;

        // Rotate into NED frame
        positionNED_ = enuToNed(positionEnu_);

        // Pull off the ENU orientation data
        tf::Quaternion tf_quat;
        double phi, theta, psi;
        tf::quaternionMsgToTF(msg->pose.orientation, tf_quat);
        tf::Matrix3x3(tf_quat).getRPY(phi, theta, psi);
        eulerFlu_(0,0) = phi;
        eulerFlu_(1,0) = theta;
        eulerFlu_(2,0) = psi;

        // Rotate into NED frame
        eulerFrd_ = fluToFrd(eulerFlu_);

        // Convert back to quaternion
        // quatFrd_ = tf::createQuaternionFromRPY(eulerFrd_(0,0), eulerFrd_(1,0), eulerFrd_(2,0));
        quatFrd_ = tf::createQuaternionMsgFromRollPitchYaw(eulerFrd_(0,0), eulerFrd_(1,0), eulerFrd_(2,0));

        // Publish estimate data
        publishEstimate();
	 }


	 void MavrosNED::velocityCallback(const geometry_msgs::TwistStampedConstPtr& msg)
	 {
	 	// Pull off the RFU linear velocity data
	 	velLinRfu_(0,0) = msg->twist.linear.x;
	 	velLinRfu_(1,0) = msg->twist.linear.y;
	 	velLinRfu_(2,0) = msg->twist.linear.z;

	 	// Rotate it into the FRD frame
	 	velLinFrd_ = enuToNed(velLinRfu_);

	 	// Pull off the RFU angular velocity data
	 	velAngFlu_(0,0) = msg->twist.angular.x;
	 	velAngFlu_(1,0) = msg->twist.angular.y;
	 	velAngFlu_(2,0) = msg->twist.angular.z;

	 	// Rotate it into the FRD frame
	 	velAngFrd_ = fluToFrd(velAngFlu_);

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

	 	// Publish
	 	estimate_pub_.publish(estimate_msg_);
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
