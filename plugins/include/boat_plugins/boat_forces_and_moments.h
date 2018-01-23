/*
 * boat_forces_and_moments.h
 *
 *  Created on: May 17, 2017
 *      Author: andrew
 */

#ifndef FCU_SIM_FCU_SIM_PLUGINS_INCLUDE_FCU_SIM_PLUGINS_BOAT_FORCES_AND_MOMENTS_H_
#define FCU_SIM_FCU_SIM_PLUGINS_INCLUDE_FCU_SIM_PLUGINS_BOAT_FORCES_AND_MOMENTS_H_

#include <stdio.h>
#include <iostream> // ------------------------------------------

#include <vector>
#include <boost/bind.hpp>
#include <Eigen/Eigen>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include "geometry_msgs/Pose.h"
#include <rosflight_msgs/Attitude.h>
#include <rosflight_utils/simple_pid.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>

#include "common.h"

namespace gazebo {

class BoatForcesAndMoments : public ModelPlugin {
public:
  BoatForcesAndMoments();

  ~BoatForcesAndMoments();

  void InitializeParams();
  void SendForces();

protected:
  void UpdateForcesAndMoments();
  void Reset();
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo & /*_info*/);

private:
  std::string command_topic_;
  std::string attitude_topic_;
  std::string joint_name_;
  std::string link_name_;
  std::string parent_frame_id_;
  std::string namespace_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  physics::JointPtr joint_;
  physics::EntityPtr parent_link_;
  event::ConnectionPtr updateConnection_; // Pointer to the update event connection.

  double mass_; // of the boat
  double gravity_;
  double linearPgain_;
  double linearIgain_;
  double linearDgain_;
  double angularPgain_;
  double angularIgain_;
  double angularDgain_;
  double angularSat_;

  // container for forces
  struct ForcesAndTorques{
    double Fx;
    double Fy;
    double Fz;
    double l;
    double m;
    double n;
  } actual_forces_;

  // a PID controller for each degree of freedom
  rosflight_utils::SimplePID roll_controller_;
  rosflight_utils::SimplePID pitch_controller_;
  rosflight_utils::SimplePID yaw_controller_;
  rosflight_utils::SimplePID x_controller_;
  rosflight_utils::SimplePID y_controller_;
  rosflight_utils::SimplePID z_controller_;

  geometry_msgs::Pose command_;

  // Time Counters
  double sampling_time_;
  double prev_sim_time_;

  ros::NodeHandle* nh_;
  ros::Subscriber command_sub_;
  ros::Publisher attitude_pub_;

  boost::thread callback_queue_thread_; // ???????
  void QueueThread(); // ??????????????????????
  void CommandCallback(const geometry_msgs::Pose msg);
  void ComputeControl(void);
  double sat(double x, double max, double min);
  double max(double x, double y);
};

} // namespace gazebo

#endif /* FCU_SIM_FCU_SIM_PLUGINS_INCLUDE_FCU_SIM_PLUGINS_BOAT_FORCES_AND_MOMENTS_H_ */
