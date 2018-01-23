/*
 * boat_forces_and_moments.cpp
 *
 *  Created on: May 17, 2017
 *      Author: andrew
 */

#include "boat_plugins/boat_forces_and_moments.h"

namespace gazebo {

BoatForcesAndMoments::BoatForcesAndMoments() {
  nh_ = new ros::NodeHandle();

  prev_sim_time_ = 0.0;
  sampling_time_ = 0.0;
  mass_ = 0.0;
  gravity_ = 0.0;
  linearPgain_ = 0.0;
  linearIgain_ = 0.0;
  linearDgain_ = 0.0;
  angularPgain_ = 0.0;
  angularIgain_ = 0.0;
  angularDgain_ = 0.0;
  angularSat_ = 0.0;
}

BoatForcesAndMoments::~BoatForcesAndMoments() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (nh_) {
    nh_->shutdown();
    delete nh_;
  }
}

void BoatForcesAndMoments::SendForces() // apply forces and torques to joint
{
  link_->AddForce(
      math::Vector3(actual_forces_.Fx, actual_forces_.Fy, actual_forces_.Fz));
  link_->AddRelativeTorque(
      math::Vector3(actual_forces_.l, actual_forces_.m, actual_forces_.n));
}

void BoatForcesAndMoments::Load(physics::ModelPtr _model,
                                sdf::ElementPtr _sdf) {
  model_ = _model;
  world_ = model_->GetWorld();

  namespace_.clear();

  if (_sdf->HasElement("namespace"))
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
  else
    gzerr << "[boat_forces_and_moments] Please specify a namespace.\n";

  nh_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[boat_forces_and_moments] Please specify a linkName of the "
             "forces and moments plugin.\n";

  link_ = model_->GetLink(link_name_);

  if (link_ == NULL)
    gzthrow("[boat_forces_and_moments] Couldn't find specified link \""
            << link_name_ << "\".");

  /* Load Params from Gazebo Server */

  getSdfParam<std::string>(_sdf, "commandTopic", command_topic_, "command");
  getSdfParam<std::string>(_sdf, "attitudeTopic", attitude_topic_, "attitude");

  /* Load Params from ROS Server */
  mass_ = nh_->param<double>("boat_mass", 2.0);
  gravity_ = nh_->param<double>("GRAVITY_CONST", 9.80665);
  linearPgain_ = nh_->param<double>("LINEAR_P_GAIN", 0.8);
  linearIgain_ = nh_->param<double>("LINEAR_I_GAIN", 0.0);
  linearDgain_ = nh_->param<double>("LINEAR_D_GAIN", 1.4);
  angularPgain_ = nh_->param<double>("ANGULAR_P_GAIN", 0.01);
  angularIgain_ = nh_->param<double>("ANGULAR_I_GAIN", 0.0);
  angularDgain_ = nh_->param<double>("ANGULAR_D_GAIN", 0.05);
  angularSat_ = nh_->param<double>("ANGULAR_SATURATION", 0.1);

  // Get PID Gains
  double xP, xI, xD;
  double yP, yI, yD;
  double zP, zI, zD;
  double rollP, rollI, rollD;
  double pitchP, pitchI, pitchD;
  double yawP, yawI, yawD;

  rollP = nh_->param<double>("roll_P", angularPgain_);
  rollI = nh_->param<double>("roll_I", angularIgain_);
  rollD = nh_->param<double>("roll_D", angularDgain_);
  pitchP = nh_->param<double>("pitch_P", angularPgain_);
  pitchI = nh_->param<double>("pitch_I", angularIgain_);
  pitchD = nh_->param<double>("pitch_D", angularDgain_);
  yawP = nh_->param<double>("yaw_P", angularPgain_);
  yawI = nh_->param<double>("yaw_I", angularIgain_);
  yawD = nh_->param<double>("yaw_D", angularDgain_);
  xP = nh_->param<double>("x_P", linearPgain_);
  xI = nh_->param<double>("x_I", linearIgain_);
  xD = nh_->param<double>("x_D", linearDgain_);
  yP = nh_->param<double>("y_P", linearPgain_);
  yI = nh_->param<double>("y_I", linearIgain_);
  yD = nh_->param<double>("y_D", linearDgain_);
  zP = nh_->param<double>("z_P", linearPgain_);
  zI = nh_->param<double>("z_I", linearIgain_);
  zD = nh_->param<double>("z_D", linearDgain_);

  roll_controller_.setGains(rollP, rollI, rollD);
  pitch_controller_.setGains(pitchP, pitchI, pitchD);
  yaw_controller_.setGains(yawP, yawI, yawD);
  x_controller_.setGains(xP, xI, xD);
  y_controller_.setGains(yP, yI, yD);
  z_controller_.setGains(zP, zI, zD);

  // Connect the update function to the simulation
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&BoatForcesAndMoments::OnUpdate, this, _1));

  // Connect Subscriber
  command_sub_ = nh_->subscribe(command_topic_, 1,
                                &BoatForcesAndMoments::CommandCallback, this);

  // Connect Publishers (for whatever it may be needed for...)
  attitude_pub_ = nh_->advertise<rosflight_msgs::Attitude>(attitude_topic_, 1);

  // Initialize State
  this->Reset();
}

void BoatForcesAndMoments::OnUpdate(const common::UpdateInfo &_info) {
  sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = _info.simTime.Double();
  UpdateForcesAndMoments();
  SendForces();
}

void BoatForcesAndMoments::CommandCallback(const geometry_msgs::Pose msg) {
  command_ = msg;
}

void BoatForcesAndMoments::Reset() {
  // Re-Initialize Memory Variables

  actual_forces_.Fx = 0;
  actual_forces_.Fy = 0;
  actual_forces_.Fz = mass_ * gravity_;
  actual_forces_.l = 0;
  actual_forces_.m = 0;
  actual_forces_.n = 0;

  prev_sim_time_ = -1.0;
  sampling_time_ = -1.0;
  command_.position.x = -9999.0; // flag for not having received a command yet
}

void BoatForcesAndMoments::UpdateForcesAndMoments() {
  /* Get state information from Gazebo                          *
   * C denotes child frame, P parent frame, and W world frame.  *
   * Further C_pose_W_P denotes pose of P wrt. W expressed in C.*/

  math::Pose W_pose_W_C = link_->GetWorldCoGPose();
  double pn = W_pose_W_C.pos.x;
  double pe = W_pose_W_C.pos.y;
  double pd = W_pose_W_C.pos.z;
  math::Vector3 euler_angles = W_pose_W_C.rot.GetAsEuler();
  double phi = euler_angles.x;   // roll
  double theta = euler_angles.y; // pitch
  double psi = euler_angles.z;   // yaw, wrapped between -pi and pi

  math::Vector3 C_linear_velocity_W_C = link_->GetRelativeLinearVel();
  double u = C_linear_velocity_W_C.x;
  double v = C_linear_velocity_W_C.y;
  double w = C_linear_velocity_W_C.z;
  math::Vector3 C_angular_velocity_W_C = link_->GetRelativeAngularVel();
  double p = C_angular_velocity_W_C.x;
  double q = C_angular_velocity_W_C.y;
  double r = C_angular_velocity_W_C.z;

  // Calculate the appropriate control
  if (command_.position.x == -9999.0) {
    // Haven't received a command yet, do nothing.
  } else {
    // ** Currently only saturating angular forces; update linearSat_ in yaml
    // ** and in code if linear saturation needed
    actual_forces_.Fx =
        x_controller_.computePID(command_.position.x, pn, sampling_time_);
    actual_forces_.Fy =
        y_controller_.computePID(command_.position.y, pe, sampling_time_);
    actual_forces_.Fz =
        z_controller_.computePID(command_.position.z, pd, sampling_time_) +
        mass_ * gravity_;

    while (command_.orientation.x - phi < -M_PI)
      phi -= 2 * M_PI;
    while (command_.orientation.x - phi > M_PI)
      phi += 2 * M_PI;

    actual_forces_.l = sat(roll_controller_.computePID(command_.orientation.x,
                                                       phi, sampling_time_),
                           angularSat_, -angularSat_);

    while (command_.orientation.y - theta < -M_PI)
      theta -= 2 * M_PI;
    while (command_.orientation.y - theta > M_PI)
      theta += 2 * M_PI;

    actual_forces_.m = sat(pitch_controller_.computePID(command_.orientation.y,
                                                        theta, sampling_time_),
                           angularSat_, -angularSat_);

    while (command_.orientation.z - psi < -M_PI) // psi too large
      psi -= 2 * M_PI;
    while (command_.orientation.z - psi > M_PI) // psi too small
      psi += 2 * M_PI;

    actual_forces_.n = sat(
        yaw_controller_.computePID(command_.orientation.z, psi, sampling_time_),
        angularSat_, -angularSat_);
  }
}

double BoatForcesAndMoments::sat(double x, double max, double min) {
  if (x > max)
    return max;
  else if (x < min)

    return min;
  else
    return x;
}

double BoatForcesAndMoments::max(double x, double y) { return (x > y) ? x : y; }

GZ_REGISTER_MODEL_PLUGIN(BoatForcesAndMoments);

} // namespace gazebo
