/****************************************************************************
 *   Copyright (c) 2017 Michael Shomin. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ATLFlight nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * In addition Supplemental Terms apply.  See the SUPPLEMENTAL file.
 ****************************************************************************/
#include "snav_interface/snav_interface.hpp"

using FCI = snav_fci::FlightControlInterface;

SnavInterface::SnavInterface(ros::NodeHandle nh, ros::NodeHandle pnh,
    FCI::Permissions requested_perm) : nh_(nh), pnh_(pnh),
  tf_listener_(tf_buffer_, nh_), fci_(requested_perm),
  takeoff_server_(nh_, "takeoff", boost::bind(&SnavInterface::TakeoffCB, this, _1), false),
  land_server_(nh_, "land", boost::bind(&SnavInterface::LandCB, this, _1), false),
  mission_server_(nh_, "execute_mission", boost::bind(&SnavInterface::ExecuteMissionCB, this, _1), false),
  go_to_waypoint_server_(nh_, "go_to_waypoint", boost::bind(&SnavInterface::GoToWaypointCB, this, _1), false),
  compute_traj_server_(nh_, "compute_traj", boost::bind(&SnavInterface::ComputeTrajCB, this, _1), false)
{
  last_vel_command_time_ = ros::Time(0);

  // Setup the publishers
  pose_est_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 10);
  pose_des_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("pose_des", 10);
  pose_sim_gt_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("pose_sim_gt", 10);
  battery_voltage_publisher_ = nh_.advertise<std_msgs::Float32>("battery_voltage", 10);
  on_ground_publisher_ = nh_.advertise<std_msgs::Bool>("on_ground", 10);
  waypoint_publisher_ = nh_.advertise<snav_msgs::WaypointArray>("planner/waypoints/input", 10);
  optimized_waypoint_publisher_ = nh_.advertise<snav_msgs::WaypointArray>("planner/waypoints/optimized", 10);
  traj_publisher_ = nh_.advertise<nav_msgs::Path>("trajectory", 10);
  props_state_publisher_ = nh_.advertise<std_msgs::String>("props_state", 10);

  cmd_vel_subscriber_ = nh_.subscribe("cmd_vel", 10, &SnavInterface::CmdVelCallback, this);
  start_props_subscriber_ = nh_.subscribe("start_props", 10, &SnavInterface::StartPropsCallback, this);
  stop_props_subscriber_ = nh_.subscribe("stop_props", 10, &SnavInterface::StopPropsCallback, this);
  waypoint_subscriber_ = nh_.subscribe("input_waypoints", 10, &SnavInterface::InputWaypointCallback, this);

  pnh_.param("gps_enu_frame", gps_enu_frame_, std::string("/gps/enu"));
  pnh_.param("estimation_frame", estimation_frame_, std::string("/odom"));
  pnh_.param("base_link_frame", base_link_frame_, std::string("/base_link"));
  pnh_.param("base_link_stab_frame", base_link_stab_frame_, std::string("/base_link_stab"));
  pnh_.param("base_link_no_rot_frame", base_link_no_rot_frame_, std::string("/base_link_no_rot"));
  pnh_.param("desired_frame",desired_frame_,std::string("/desired"));
  pnh_.param("sim_gt_frame", sim_gt_frame_, std::string("/sim/ground_truth"));
  pnh_.param("launch_frame", launch_frame_, std::string("/launch"));
  pnh_.param("waypoint_frame", waypoint_frame_, std::string("/waypoint"));

  pnh_.param("simulation", simulation_, false);

  rc_cmd_type_ = SN_RC_POS_HOLD_CMD;
  rc_cmd_mapping_ = RC_OPT_LINEAR_MAPPING;

  std::string rc_cmd_type_string;
  std::string rc_cmd_mapping_string;

  pnh_.param("sn_rc_cmd_type", rc_cmd_type_string, std::string("SN_RC_POS_HOLD_CMD"));
  pnh_.param("sn_rc_mapping_type", rc_cmd_mapping_string, std::string("RC_OPT_LINEAR_MAPPING"));

  pnh_.param("high_level_actions_only", high_level_actions_only_, true);

  if (!high_level_actions_only_) {
    SetRcCommandType(rc_cmd_type_string);
    SetRcMappingType(rc_cmd_mapping_string);
  }

  std::string tx_config_desired_mode_string;
  std::string tx_config_waypoint_frame_parent_string;
  std::string planner_config_traj_type_string;
  pnh_.param("tx_config_rate", tx_config_.tx_rate, tx_config_.tx_rate);
  pnh_.param("tx_config_desired_mode", tx_config_desired_mode_string, std::string("SN_POS_HOLD_MODE"));
  pnh_.param("tx_config_use_traj_tracking", tx_config_.use_traj_tracking, tx_config_.use_traj_tracking);
  pnh_.param("tx_config_waypoint_frame_parent", tx_config_waypoint_frame_parent_string, std::string("ESTIMATION"));
  pnh_.param("planner_config_traj_type", planner_config_traj_type_string, std::string("MIN_SNAP"));
  pnh_.param("planner_config_max_allowed_vel_xy", planner_config_.max_allowed_vel_xy, planner_config_.max_allowed_vel_xy);
  pnh_.param("planner_config_max_allowed_vel_z", planner_config_.max_allowed_vel_z, planner_config_.max_allowed_vel_z);
  pnh_.param("planner_config_max_allowed_acc_xy", planner_config_.max_allowed_acc_xy, planner_config_.max_allowed_acc_xy);
  pnh_.param("planner_config_max_allowed_acc_z", planner_config_.max_allowed_acc_z, planner_config_.max_allowed_acc_z);
  pnh_.param("planner_config_max_allowed_jerk_xy", planner_config_.max_allowed_jerk_xy, planner_config_.max_allowed_jerk_xy);
  pnh_.param("planner_config_max_allowed_jerk_z", planner_config_.max_allowed_jerk_z, planner_config_.max_allowed_jerk_z);

  SetTxConfigDesiredMode(tx_config_desired_mode_string);
  SetTxConfigWaypointFrameParent(tx_config_waypoint_frame_parent_string);
  SetPlannerConfigTrajType(planner_config_traj_type_string);

  pnh_.param("rx_config_rate", rx_config_.rx_rate, rx_config_.rx_rate);

  cmd_type_subscriber_ = nh_.subscribe("cmd_type", 10, &SnavInterface::CmdTypeCallback, this);
  cmd_mapping_subscriber_ = nh_.subscribe("cmd_mapping", 10, &SnavInterface::CmdMappingCallback, this);

  if(!simulation_)
    GetDSPTimeOffset();
  else
  {
    dsp_offset_in_ns_  = 0;
    clock_publisher_ = nh_.advertise<rosgraph_msgs::Clock>("clock", 1);
  }

  valid_rotation_est_ = false;
  valid_rotation_sim_gt_ = false;
  valid_waypoint_frame_ = false;

  traj_viz_sample_rate_ = 10;
}

int SnavInterface::Initialize()
{
  recon_server_.setCallback(boost::bind(&SnavInterface::ReconCallback, this, _1, _2));

  if (fci_.get_permissions() == FCI::Permissions::READ_WRITE)
  {
    fci_.configure_tx(tx_config_);
    fci_.configure_planner(planner_config_);
  }

  fci_.configure_rx(rx_config_);

  if (fci_.connect() != FCI::Return::SUCCESS) return -2;

  takeoff_server_.start();
  land_server_.start();
  mission_server_.start();
  go_to_waypoint_server_.start();
  compute_traj_server_.start();

  return 0;
}

void SnavInterface::CmdTypeCallback(const std_msgs::String::ConstPtr& msg)
{
  SetRcCommandType(msg->data);
}

void SnavInterface::CmdMappingCallback(const std_msgs::String::ConstPtr& msg)
{
  SetRcMappingType(msg->data);
}

void SnavInterface::TakeoffCB(const snav_msgs::TakeoffGoalConstPtr& goal)
{
  fci_.takeoff_nb(takeoff_config_);

  snav_fci::StateVector state_est, state_des;
  ros::Rate loop_rate(10);
  while (fci_.get_current_action() == FCI::Action::TAKEOFF && ros::ok())
  {
    SnavCachedData snav_data = fci_.get_snav_cached_data();
    state_est = fci_.get_estimated_state(snav_data);
    state_des = fci_.get_desired_state(snav_data);
    snav_msgs::TakeoffFeedback feedback;
    feedback.height_desired = state_des.position[2];
    feedback.height_estimated = state_est.position[2];
    takeoff_server_.publishFeedback(feedback);

    if (takeoff_server_.isPreemptRequested())
    {
      fci_.preempt_current_action();
    }

    loop_rate.sleep();
  }

  snav_msgs::TakeoffResult result;
  result.height_desired = state_des.position[2];
  result.height_estimated = state_est.position[2];
  result.return_code.data = fci_.get_return_as_string(fci_.get_last_action_result());

  if (fci_.get_last_action_result() == FCI::Return::SUCCESS)
  {
    takeoff_server_.setSucceeded(result);
  }
  else if (fci_.get_last_action_result() == FCI::Return::ACTION_PREEMPTED)
  {
    takeoff_server_.setPreempted(result, fci_.get_return_as_string(fci_.get_last_action_result()));
  }
  else
  {
    takeoff_server_.setAborted(result, fci_.get_return_as_string(fci_.get_last_action_result()));
  }
}

void SnavInterface::LandCB(const snav_msgs::LandGoalConstPtr& goal)
{
  fci_.land_nb(landing_config_);

  snav_fci::StateVector state_est, state_des;
  ros::Rate loop_rate(10);
  while (fci_.get_current_action() == FCI::Action::LAND && ros::ok())
  {
    SnavCachedData snav_data = fci_.get_snav_cached_data();
    state_est = fci_.get_estimated_state(snav_data);
    state_des = fci_.get_desired_state(snav_data);
    snav_msgs::LandFeedback feedback;
    feedback.height_desired = state_des.position[2];
    feedback.height_estimated = state_est.position[2];
    land_server_.publishFeedback(feedback);

    if (land_server_.isPreemptRequested())
    {
      fci_.preempt_current_action();
    }

    loop_rate.sleep();
  }

  snav_msgs::LandResult result;
  result.return_code.data = fci_.get_return_as_string(fci_.get_last_action_result());
  result.height_desired = state_des.position[2];
  result.height_estimated = state_est.position[2];

  if (fci_.get_last_action_result() == FCI::Return::SUCCESS)
  {
    land_server_.setSucceeded(result);
  }
  else if (fci_.get_last_action_result() == FCI::Return::ACTION_PREEMPTED)
  {
    land_server_.setPreempted(result, fci_.get_return_as_string(fci_.get_last_action_result()));
  }
  else
  {
    land_server_.setAborted(result, fci_.get_return_as_string(fci_.get_last_action_result()));
  }
}

void SnavInterface::ExecuteMissionCB(const snav_msgs::ExecuteMissionGoalConstPtr& goal)
{
  ros::Time start_time(goal->start);
  double start_time_secs = start_time.toSec();
  fci_.execute_mission_nb(start_time_secs);
  ros::Rate loop_rate(10);
  while (fci_.get_current_action() == FCI::Action::EXECUTE_MISSION && ros::ok())
  {
    snav_msgs::ExecuteMissionFeedback feedback;
    feedback.traj_time = fci_.get_trajectory_time();
    std::vector<snav_fci::Waypoint> waypoints;
    fci_.get_waypoints(waypoints);
    for (auto&& it = waypoints.begin(); it != waypoints.end(); ++it) {
      if (it->status == snav_fci::Waypoint::Status::ENROUTE) {
        feedback.active_waypoint.header.stamp = ros::Time(it->time);
        feedback.active_waypoint.header.seq = (it - waypoints.begin());
        feedback.active_waypoint.vector.x = it->position[0];
        feedback.active_waypoint.vector.y = it->position[1];
        feedback.active_waypoint.vector.z = it->position[2];
      }
    }
    mission_server_.publishFeedback(feedback);

    if (mission_server_.isPreemptRequested())
    {
      fci_.preempt_current_action();
    }

    loop_rate.sleep();
  }

  snav_msgs::ExecuteMissionResult result;
  result.return_code.data = fci_.get_return_as_string(fci_.get_last_action_result());

  if (fci_.get_last_action_result() == FCI::Return::SUCCESS)
  {
    mission_server_.setSucceeded(result);
  }
  else if (fci_.get_last_action_result() == FCI::Return::ACTION_PREEMPTED)
  {
    mission_server_.setPreempted(result);
  }
  else
  {
    mission_server_.setAborted(result);
  }
}

void SnavInterface::GoToWaypointCB(const snav_msgs::GoToWaypointGoalConstPtr& goal)
{
  snav_fci::Waypoint waypoint(snav_fci::StateVector(Eigen::Vector3f(goal->position.x,
        goal->position.y, goal->position.z), goal->yaw));

  snav_fci::WaypointConfig config;
  config.yaw_type = snav_fci::WaypointConfig::YawType::WAYPOINT;
  waypoint.set_config(config);
  fci_.go_to_waypoint_nb(waypoint);

  snav_fci::StateVector state_est, state_des;
  ros::Rate loop_rate(10);
  while (fci_.get_current_action() == FCI::Action::GO_TO_WAYPOINT && ros::ok())
  {
    SnavCachedData snav_data = fci_.get_snav_cached_data();
    state_est = fci_.get_estimated_state(snav_data);
    state_des = fci_.get_desired_state(snav_data);

    snav_msgs::GoToWaypointFeedback feedback;
    feedback.position_desired.x = state_des.position[0];
    feedback.position_desired.y = state_des.position[1];
    feedback.position_desired.z = state_des.position[2];
    feedback.position_estimated.x = state_est.position[0];
    feedback.position_estimated.y = state_est.position[1];
    feedback.position_estimated.z = state_est.position[2];
    feedback.yaw_desired = state_des.yaw;
    feedback.yaw_estimated = state_est.yaw;
    go_to_waypoint_server_.publishFeedback(feedback);

    if (go_to_waypoint_server_.isPreemptRequested())
    {
      fci_.preempt_current_action();
    }

    loop_rate.sleep();
  }

  snav_msgs::GoToWaypointResult result;
  result.return_code.data = fci_.get_return_as_string(fci_.get_last_action_result());
  result.position_desired.x = state_des.position[0];
  result.position_desired.y = state_des.position[1];
  result.position_desired.z = state_des.position[2];
  result.position_estimated.x = state_est.position[0];
  result.position_estimated.y = state_est.position[1];
  result.position_estimated.z = state_est.position[2];
  result.yaw_desired = state_des.yaw;
  result.yaw_estimated = state_est.yaw;

  if (fci_.get_last_action_result() == FCI::Return::SUCCESS)
  {
    go_to_waypoint_server_.setSucceeded(result);
  }
  else if (fci_.get_last_action_result() == FCI::Return::ACTION_PREEMPTED)
  {
    go_to_waypoint_server_.setPreempted(result, fci_.get_return_as_string(fci_.get_last_action_result()));
  }
  else
  {
    go_to_waypoint_server_.setAborted(result, fci_.get_return_as_string(fci_.get_last_action_result()));
  }
}

void SnavInterface::ComputeTrajCB(const snav_msgs::ComputeTrajGoalConstPtr& goal)
{
  std::vector<snav_msgs::ComputeTrajResult> result_vec;
  FCI::Return compute_result = RecomputeTrajectory();

  std::vector<snav_traj_gen::SnavTrajectory> snav_traj_vector;
  if (fci_.get_last_optimized_trajectory(snav_traj_vector) == FCI::Return::SUCCESS) {
    for (auto&& it = snav_traj_vector.begin(); it != snav_traj_vector.end(); ++it) {
      snav_msgs::ComputeTrajResult result;
      if (it->optimized_wps.size() > 0) {
        result.traj_duration = (it->optimized_wps.end()-1)->time - it->optimized_wps.begin()->time;
      }
      else {
        result.traj_duration = 0;
      }

      result.max_velocity_x = it->max_velocity_x;
      result.max_velocity_y = it->max_velocity_y;
      result.max_velocity_z = it->max_velocity_z;
      result.max_velocity_xy = it->max_velocity_xy;
      result.max_velocity_xyz = it->max_velocity_xyz;
      result.max_acceleration_x = it->max_acceleration_x;
      result.max_acceleration_y = it->max_acceleration_y;
      result.max_acceleration_z = it->max_acceleration_z;
      result.max_acceleration_xy = it->max_acceleration_xy;
      result.max_acceleration_xyz = it->max_acceleration_xyz;
      result.max_jerk_x = it->max_jerk_x;
      result.max_jerk_y = it->max_jerk_y;
      result.max_jerk_z = it->max_jerk_z;
      result.max_jerk_xy = it->max_jerk_xy;
      result.max_jerk_xyz = it->max_jerk_xyz;
      result_vec.push_back(result);
    }
  }

  snav_msgs::ComputeTrajResult result;
  result.return_code.data = fci_.get_return_as_string(compute_result);
  for (auto&& it = result_vec.begin(); it != result_vec.end(); ++it) {
    result.traj_duration += it->traj_duration;
    result.max_velocity_x = std::max(result.max_velocity_x, it->max_velocity_x);
    result.max_velocity_y = std::max(result.max_velocity_y, it->max_velocity_y);
    result.max_velocity_z = std::max(result.max_velocity_z, it->max_velocity_z);
    result.max_velocity_xy = std::max(result.max_velocity_xy, it->max_velocity_xy);
    result.max_velocity_xyz = std::max(result.max_velocity_xyz, it->max_velocity_xyz);
    result.max_acceleration_x = std::max(result.max_acceleration_x, it->max_acceleration_x);
    result.max_acceleration_y = std::max(result.max_acceleration_y, it->max_acceleration_y);
    result.max_acceleration_z = std::max(result.max_acceleration_z, it->max_acceleration_z);
    result.max_acceleration_xy = std::max(result.max_acceleration_xy, it->max_acceleration_xy);
    result.max_acceleration_xyz = std::max(result.max_acceleration_xyz, it->max_acceleration_xyz);
    result.max_jerk_x = std::max(result.max_jerk_x, it->max_jerk_x);
    result.max_jerk_y = std::max(result.max_jerk_y, it->max_jerk_y);
    result.max_jerk_z = std::max(result.max_jerk_z, it->max_jerk_z);
    result.max_jerk_xy = std::max(result.max_jerk_xy, it->max_jerk_xy);
    result.max_jerk_xyz = std::max(result.max_jerk_xyz, it->max_jerk_xyz);
  }

  if (compute_result == FCI::Return::SUCCESS) {
    compute_traj_server_.setSucceeded(result);
  }
  else {
    compute_traj_server_.setAborted(result, fci_.get_return_as_string(fci_.get_last_action_result()));
  }
}

void SnavInterface::SetRcMappingType(std::string rc_cmd_mapping_string)
{
  if (!high_level_actions_only_) {
    if(rc_cmd_mapping_string == "RC_OPT_LINEAR_MAPPING")
    {
      rc_cmd_mapping_ = RC_OPT_LINEAR_MAPPING;
      ROS_INFO("cmd_vel SNAV mapping : RC_OPT_LINEAR_MAPPING");
    }
    else if(rc_cmd_mapping_string == "RC_OPT_ENABLE_DEADBAND")
    {
      rc_cmd_mapping_ = RC_OPT_ENABLE_DEADBAND;
      ROS_INFO("cmd_vel SNAV mapping : RC_OPT_ENABLE_DEADBAND");
    }
    else  if(rc_cmd_mapping_string == "RC_OPT_COMPLIANT_TRACKING")
    {
      rc_cmd_mapping_ = RC_OPT_COMPLIANT_TRACKING;
      ROS_INFO("cmd_vel SNAV mapping : RC_OPT_COMPLIANT_TRACKING");
    }
    else  if(rc_cmd_mapping_string == "RC_OPT_DEFAULT_RC")
    {
      rc_cmd_mapping_ = RC_OPT_DEFAULT_RC;
      ROS_INFO("cmd_vel SNAV mapping : RC_OPT_DEFAULT_RC");
    }
    else  if(rc_cmd_mapping_string == "RC_OPT_TRIGGER_LANDING")
    {
      rc_cmd_mapping_ = RC_OPT_TRIGGER_LANDING;
      ROS_INFO("cmd_vel SNAV mapping : RC_OPT_TRIGGER_LANDING");
    }
    else
    {
      ROS_ERROR_STREAM("unrecognized sn_rc_mapping_type, keeping mapping type: " << (int)rc_cmd_mapping_ );
    }
  }
  else {
    ROS_WARN("rc cmd vel interface is disabled in favor of high level actions");
  }
}


void SnavInterface::SetRcCommandType(std::string rc_cmd_type_string)
{
  if (!high_level_actions_only_) {
    if(rc_cmd_type_string == "SN_RC_RATES_CMD")
    {
      rc_cmd_type_ = SN_RC_RATES_CMD;
      ROS_INFO("cmd_vel SNAV cmd type: SN_RC_RATES_CMD");
    }
    else if(rc_cmd_type_string == "SN_RC_THRUST_ANGLE_CMD")
    {
      rc_cmd_type_ = SN_RC_THRUST_ANGLE_CMD;
      ROS_INFO("cmd_vel SNAV cmd type: SN_RC_THRUST_ANGLE_CMD");
    }
    else if(rc_cmd_type_string == "SN_RC_ALT_HOLD_CMD")
    {
      rc_cmd_type_ = SN_RC_ALT_HOLD_CMD;
      ROS_INFO("cmd_vel SNAV cmd type: SN_RC_ALT_HOLD_CMD");
    }
    else if(rc_cmd_type_string == "SN_RC_THRUST_ANGLE_GPS_HOVER_CMD")
    {
      rc_cmd_type_ = SN_RC_THRUST_ANGLE_GPS_HOVER_CMD;
      ROS_INFO("cmd_vel SNAV cmd type: SN_RC_THRUST_ANGLE_GPS_HOVER_CMD");
    }
    else if(rc_cmd_type_string == "SN_RC_GPS_POS_HOLD_CMD")
    {
      rc_cmd_type_ = SN_RC_GPS_POS_HOLD_CMD;
      ROS_INFO("cmd_vel SNAV cmd type: SN_RC_GPS_POS_HOLD_CMD");
    }
    else if(rc_cmd_type_string == "SN_RC_OPTIC_FLOW_POS_HOLD_CMD")
    {
      rc_cmd_type_ = SN_RC_OPTIC_FLOW_POS_HOLD_CMD;
      ROS_INFO("cmd_vel SNAV cmd type: SN_RC_OPTIC_FLOW_POS_HOLD_CMD");
    }
    else if(rc_cmd_type_string == "SN_RC_VIO_POS_HOLD_CMD")
    {
      rc_cmd_type_ = SN_RC_VIO_POS_HOLD_CMD;
      ROS_INFO("cmd_vel SNAV cmd type: SN_RC_VIO_POS_HOLD_CMD");
    }
    else if(rc_cmd_type_string == "SN_RC_ALT_HOLD_LOW_ANGLE_CMD")
    {
      rc_cmd_type_ = SN_RC_ALT_HOLD_LOW_ANGLE_CMD;
      ROS_INFO("cmd_vel SNAV cmd type: SN_RC_ALT_HOLD_LOW_ANGLE_CMD");
    }
    else if(rc_cmd_type_string == "SN_RC_POS_HOLD_CMD")
    {
      rc_cmd_type_ = SN_RC_POS_HOLD_CMD;
      ROS_INFO("cmd_vel SNAV cmd type: SN_RC_POS_HOLD_CMD");
    }
    else
    {
      ROS_ERROR_STREAM("Unrecognized sn_rc_cmd_type, keeping mapping type: " << (int)rc_cmd_type_ );
    }
  }
  else {
    ROS_WARN("rc cmd vel interface is disabled in favor of high level actions");
  }
}

void SnavInterface::SetTxConfigDesiredMode(std::string desired_mode_string)
{
  if (desired_mode_string == "SN_ESC_RPM_MODE")
  {
    tx_config_.desired_mode = SN_ESC_RPM_MODE;
    ROS_INFO("snav desired mode = SN_ESC_RPM_MODE");
  }
  else if (desired_mode_string == "SN_ESC_PWM_MODE")
  {
    tx_config_.desired_mode = SN_ESC_PWM_MODE;
    ROS_INFO("snav desired mode = SN_ESC_PWM_MODE");
  }
  else if (desired_mode_string == "SN_RATE_MODE")
  {
    tx_config_.desired_mode = SN_RATE_MODE;
    ROS_INFO("snav desired mode = SN_RATE_MODE");
  }
  else if (desired_mode_string == "SN_THRUST_ANGLE_MODE")
  {
    tx_config_.desired_mode = SN_THRUST_ANGLE_MODE;
    ROS_INFO("snav desired mode = SN_THRUST_ANGLE_MODE");
  }
  else if (desired_mode_string == "SN_ALT_HOLD_MODE")
  {
    tx_config_.desired_mode = SN_ALT_HOLD_MODE;
    ROS_INFO("snav desired mode = SN_ALT_HOLD_MODE");
  }
  else if (desired_mode_string == "SN_GPS_POS_HOLD_MODE")
  {
    tx_config_.desired_mode = SN_GPS_POS_HOLD_MODE;
    ROS_INFO("snav desired mode = SN_GPS_POS_HOLD_MODE");
  }
  else if (desired_mode_string == "SN_OPTIC_FLOW_POS_HOLD_MODE")
  {
    tx_config_.desired_mode = SN_OPTIC_FLOW_POS_HOLD_MODE;
    ROS_INFO("snav desired mode = SN_OPTIC_FLOW_POS_HOLD_MODE");
  }
  else if (desired_mode_string == "SN_VIO_POS_HOLD_MODE")
  {
    tx_config_.desired_mode = SN_VIO_POS_HOLD_MODE;
    ROS_INFO("snav desired mode = SN_VIO_POS_HOLD_MODE");
  }
  else if (desired_mode_string == "SN_THRUST_ATT_ANG_VEL_MODE")
  {
    tx_config_.desired_mode = SN_THRUST_ATT_ANG_VEL_MODE;
    ROS_INFO("snav desired mode = SN_THRUST_ATT_ANG_VEL_MODE");
  }
  else if (desired_mode_string == "SN_ALT_HOLD_LOW_ANGLE_MODE")
  {
    tx_config_.desired_mode = SN_ALT_HOLD_LOW_ANGLE_MODE;
    ROS_INFO("snav desired mode = SN_ALT_HOLD_LOW_ANGLE_MODE");
  }
  else if (desired_mode_string == "SN_POS_HOLD_MODE")
  {
    tx_config_.desired_mode = SN_POS_HOLD_MODE;
    ROS_INFO("snav desired mode = SN_POS_HOLD_MODE");
  }
  else
  {
    ROS_ERROR_STREAM("Unrecognized tx_config_desired_mode, using default");
  }
}

void SnavInterface::SetTxConfigWaypointFrameParent(std::string wp_frame_parent)
{
  if (wp_frame_parent == "ESTIMATION" || wp_frame_parent == "odom")
  {
    tx_config_.waypoint_frame_parent = snav_fci::ReferenceFrame::ESTIMATION;
    ROS_INFO("tx_config.waypoint_frame_parent = snav_fci::ReferenceFrame::ESTIMATION (/odom)");
  }
  else if (wp_frame_parent == "LAUNCH" || wp_frame_parent == "launch")
  {
    tx_config_.waypoint_frame_parent = snav_fci::ReferenceFrame::LAUNCH;
    ROS_INFO("tx_config.waypoint_frame_parent = snav_fci::ReferenceFrame::LAUNCH (/launch)");
  }
  else
  {
    ROS_ERROR("Unrecognized tx_config_waypoint_frame_parent, using default");
  }
}

void SnavInterface::SetPlannerConfigTrajType(std::string traj_type)
{
  if (traj_type == "MIN_SNAP")
  {
    planner_config_.traj_type = snav_fci::PlannerConfig::TrajType::MIN_SNAP;
    ROS_INFO("planner_config.traj_type = snav_fci::PlannerConfig::TrajType::MIN_SNAP");
  }
  else if (traj_type == "MIN_JERK")
  {
    planner_config_.traj_type = snav_fci::PlannerConfig::TrajType::MIN_JERK;
    ROS_INFO("planner_config.traj_type = snav_fci::PlannerConfig::TrajType::MIN_JERK");
  }
  else if (traj_type == "MIN_ACC")
  {
    planner_config_.traj_type = snav_fci::PlannerConfig::TrajType::MIN_ACC;
    ROS_INFO("planner_config.traj_type = snav_fci::PlannerConfig::TrajType::MIN_ACC");
  }
  else if (traj_type == "SHORTEST_PATH")
  {
    planner_config_.traj_type = snav_fci::PlannerConfig::TrajType::SHORTEST_PATH;
    ROS_INFO("planner_config.traj_type = snav_fci::PlannerConfig::TrajType::SHORTEST_PATH");
  }
  else
  {
    ROS_ERROR("Unrecognized planner_config_traj_type, using default");
  }
}

void SnavInterface::ReconCallback(snav_ros::SnavRosConfig &config, uint32_t level)
{
  traj_viz_sample_rate_ = config.traj_viz_sample_rate;
  takeoff_config_.height = config.takeoff_height;
  planner_config_.average_speed_xy = config.planner_average_speed_xy;
  planner_config_.average_speed_z = config.planner_average_speed_z;
  planner_config_.loop = config.planner_loop;
  planner_config_.traj_type =
    static_cast<snav_fci::PlannerConfig::TrajType>(config.planner_traj_type);
  planner_config_.timing =
    static_cast<snav_fci::PlannerConfig::TimestampStrategy>(config.planner_timestamp_strategy);

  if (FCI::ok()) {
    fci_.configure_planner(planner_config_);
    std::vector<snav_fci::Waypoint> waypoints;
    if (fci_.get_waypoints(waypoints) == FCI::Return::SUCCESS) {
      if (waypoints.size() > 0) RecomputeTrajectory();
    }
  }
}

snav_fci::FlightControlInterface::Return SnavInterface::RecomputeTrajectory()
{
  SnavCachedData snav_data = fci_.get_snav_cached_data();
  snav_fci::StateVector starting_state_wrt_w = fci_.get_desired_state(snav_data);
  if ((SnPropsState)snav_data.general_status.props_state == SN_PROPS_STATE_NOT_SPINNING)
  {
    // assumes that vehicle will take off to the default height
    starting_state_wrt_w.position[2] += takeoff_config_.height;
  }

  starting_state_wrt_w.velocity.setZero();
  starting_state_wrt_w.acceleration.setZero();

  FCI::Return compute_result = FCI::Return::FAILURE;
  try
  {
    compute_result = fci_.compute_trajectory(starting_state_wrt_w);
    if (compute_result != FCI::Return::SUCCESS)
    {
      ROS_WARN("Unable to compute trajectory");
    }
  }
  catch (const std::exception& ex)
  {
    std::cout << "caught exception: " << ex.what() << std::endl;
  }

  return compute_result;
}

void SnavInterface::GetDSPTimeOffset()
{
  // get the adsp offset.
  int64_t dsptime;
#ifdef QC_SOC_TARGET_APQ8096
  static const char qdspTimerTickPath[] = "/sys/kernel/boot_slpi/qdsp_qtimer";
#endif
#ifdef QC_SOC_TARGET_APQ8074
  static const char qdspTimerTickPath[] = "/sys/kernel/boot_adsp/qdsp_qtimer";
#endif
  char qdspTicksStr[20] = "";

  static const double clockFreq = 1 / 19.2;
  FILE * qdspClockfp = fopen( qdspTimerTickPath, "r" );
  fread( qdspTicksStr, 16, 1, qdspClockfp );
  uint64_t qdspTicks = strtoull( qdspTicksStr, 0, 16 );
  fclose( qdspClockfp );

  dsptime = (int64_t)( qdspTicks*clockFreq*1e3 );

  //get the apps proc timestamp;
  int64_t appstimeInNs;
  struct timespec t;
  clock_gettime( CLOCK_REALTIME, &t );

  uint64_t timeNanoSecMonotonic = (uint64_t)(t.tv_sec) * 1000000000ULL + t.tv_nsec;
  appstimeInNs = (int64_t)timeNanoSecMonotonic;

  // now compute the offset.
  dsp_offset_in_ns_  = appstimeInNs - dsptime;

  ROS_INFO_STREAM("DSP offset: " <<   dsp_offset_in_ns_ << " ns");
}

snav_msgs::WaypointArray SnavInterface::WaypointVectorToRosMsg(
    const std::vector<snav_fci::Waypoint>& wps)
{
  snav_msgs::WaypointArray wp_msg;
  wp_msg.header.stamp = timestamp_;
  wp_msg.header.frame_id = waypoint_frame_;
  ++wp_msg.header.seq;

  for (auto&& itr = wps.begin(); itr != wps.end(); ++itr)
  {
    snav_msgs::Waypoint wp;
    if (!isnan(itr->time)) wp.header.stamp = ros::Time(itr->time);
    wp.header.frame_id = waypoint_frame_;
    wp.header.seq = (itr - wps.begin());
    wp.position.x = itr->position(0);
    wp.position.y = itr->position(1);
    wp.position.z = itr->position(2);
    wp.velocity.x = itr->velocity(0);
    wp.velocity.y = itr->velocity(1);
    wp.velocity.z = itr->velocity(2);
    wp.acceleration.x = itr->acceleration(0);
    wp.acceleration.y = itr->acceleration(1);
    wp.acceleration.z = itr->acceleration(2);
    wp.jerk.x = itr->jerk(0);
    wp.jerk.y = itr->jerk(1);
    wp.jerk.z = itr->jerk(2);
    wp.yaw = itr->yaw;
    wp.yaw_rate = itr->yaw_rate;
    wp.yaw_acceleration = itr->yaw_acceleration;
    wp.constrained[0] = itr->constrained(0);
    wp.constrained[1] = itr->constrained(1);
    wp.constrained[2] = itr->constrained(2);
    wp.constrained[3] = itr->constrained(3);
    wp_msg.waypoints.push_back(wp);
  }

  return wp_msg;
}

void SnavInterface::PublishWaypoints()
{
  std::vector<snav_fci::Waypoint> waypoints;
  if (fci_.get_waypoints(waypoints) != FCI::Return::SUCCESS) return;
  if (waypoints.size() == 0) return;

  snav_msgs::WaypointArray wp_msg = WaypointVectorToRosMsg(waypoints);
  waypoint_publisher_.publish(wp_msg);

  std::vector<snav_fci::Waypoint> optimized_waypoints;
  if (fci_.get_optimized_waypoints(optimized_waypoints) != FCI::Return::SUCCESS) return;
  if (optimized_waypoints.size() == 0) return;

  snav_msgs::WaypointArray wp_opt_msg = WaypointVectorToRosMsg(optimized_waypoints);
  optimized_waypoint_publisher_.publish(wp_opt_msg);
}

void SnavInterface::PublishTrajectory()
{
  nav_msgs::Path path;
  path.header.stamp = timestamp_;
  path.header.frame_id = waypoint_frame_;

  std::vector<snav_traj_gen::SnavTrajectory> snav_traj_vector;
  if (fci_.get_current_trajectory(snav_traj_vector) == FCI::Return::SUCCESS) {
    for (auto&& it = snav_traj_vector.begin(); it != snav_traj_vector.end(); ++it)
    {
      if (it->optimized_wps.size() > 1) {
        float t_start = it->optimized_wps.begin()->time;
        float t_end = (it->optimized_wps.end()-1)->time;
        if (t_start >= 0 && t_end > 0 && t_end > t_start) {
          float dt = 1.0 / traj_viz_sample_rate_;
          for (float t = t_start; t <= t_end; t += dt)
          {
            snav_traj_gen::StateVector state;
            it->sample(state, t);
            geometry_msgs::PoseStamped pose;
            pose.pose.orientation.w = 1;
            pose.pose.position.x = state.position[0];
            pose.pose.position.y = state.position[1];
            pose.pose.position.z = state.position[2];
            pose.header.frame_id = waypoint_frame_;
            pose.header.stamp = ros::Time(t);
            path.poses.push_back(pose);
          }
        }
      }
    }
  }

  traj_publisher_.publish(path);
}

void SnavInterface::PublishLowFrequencyData(const ros::TimerEvent& event)
{
  PublishBatteryVoltage();
  PublishOnGroundFlag();
  PublishPropsState();
  if (valid_waypoint_frame_) {
    PublishWaypoints();
    PublishTrajectory();
  }
}

void SnavInterface::CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  commanded_vel_ = *msg;
  last_vel_command_time_ = ros::Time::now();
  if (!high_level_actions_only_) SendVelocityCommand();
  else ROS_WARN("cmd vel callback is disabled");
}

void SnavInterface::StartPropsCallback(const std_msgs::Empty::ConstPtr& msg)
{
  if (!high_level_actions_only_) fci_.start_props();
  else {
    ROS_WARN("start props cb disabled: only high level actions are enabled");
  }
}

void SnavInterface::StopPropsCallback(const std_msgs::Empty::ConstPtr& msg)
{
  if (!high_level_actions_only_) fci_.stop_props();
  else
  {
    ROS_WARN("stop props cb disabled: only high level actions are enabled");
  }
}

void SnavInterface::InputWaypointCallback(const snav_msgs::WaypointWithConfigArray::ConstPtr& msg)
{
  snav_msgs::WaypointWithConfigArray waypoint_array = *msg;
  std::vector<snav_fci::Waypoint> waypoints;
  for (auto&& it = waypoint_array.waypoints.begin(); it != waypoint_array.waypoints.end(); ++it) {
    snav_fci::Waypoint wp;
    wp.time = it->waypoint.header.stamp.toSec();
    wp.position[0] = it->waypoint.position.x;
    wp.position[1] = it->waypoint.position.y;
    wp.position[2] = it->waypoint.position.z;
    wp.velocity[0] = it->waypoint.velocity.x;
    wp.velocity[1] = it->waypoint.velocity.y;
    wp.velocity[2] = it->waypoint.velocity.z;
    wp.acceleration[0] = it->waypoint.acceleration.x;
    wp.acceleration[1] = it->waypoint.acceleration.y;
    wp.acceleration[2] = it->waypoint.acceleration.z;
    wp.jerk[0] = it->waypoint.jerk.x;
    wp.jerk[1] = it->waypoint.jerk.y;
    wp.jerk[2] = it->waypoint.jerk.z;
    wp.yaw = it->waypoint.yaw;
    wp.yaw_rate = it->waypoint.yaw_rate;
    wp.yaw_acceleration = it->waypoint.yaw_acceleration;
    wp.constrained[0] = it->waypoint.constrained[0];
    wp.constrained[1] = it->waypoint.constrained[1];
    wp.constrained[2] = it->waypoint.constrained[2];
    wp.constrained[3] = it->waypoint.constrained[3];

    snav_fci::WaypointConfig config;
    if (it->waypoint.yaw_type == "waypoint")
      config.yaw_type = snav_fci::WaypointConfig::YawType::WAYPOINT;
    else
      config.yaw_type = snav_fci::WaypointConfig::YawType::FORWARD;

    if (it->use_config) {
      config.max_linear_velocity_norm = it->config.max_linear_velocity_norm;
      config.max_linear_acceleration_norm = it->config.max_linear_acceleration_norm;
      config.max_yaw_velocity_norm = it->config.max_yaw_velocity_norm;
    }

    wp.set_config(config);

    waypoints.push_back(wp);
  }

  ROS_INFO_STREAM("Attempting to add " << waypoints.size() << " waypoints");

  if (fci_.preload_waypoints(waypoints) != FCI::Return::SUCCESS)
    ROS_WARN("Unable to load waypoints");

  if (waypoints.size() > 0) RecomputeTrajectory();
}

void SnavInterface::SendVelocityCommand()
{
  std::array<float, 3> velocity = { static_cast<float>(commanded_vel_.linear.x),
      static_cast<float>(commanded_vel_.linear.y),
      static_cast<float>(commanded_vel_.linear.z)};
  snav_fci::RcCommand rc_command;
  fci_.convert_velocity_to_rc_command(velocity, commanded_vel_.angular.z,
      rc_command);
  fci_.set_tx_command(rc_command);
}

void SnavInterface::UpdatePoseMessages()
{
  tf2::Quaternion q;
  GetRotationQuaternion(q);
  UpdatePosVelMessages(q);
  UpdateSimMessages();
}

void SnavInterface::GetRotationQuaternion(tf2::Quaternion &q)
{
  // Get Rotation Matrix from sn_cached_data_, convert to tf2 Matrix
  SnavCachedData snav_data = fci_.get_snav_cached_data();

  tf2::Matrix3x3 RR( snav_data.attitude_estimate.rotation_matrix[0],
      snav_data.attitude_estimate.rotation_matrix[1],
      snav_data.attitude_estimate.rotation_matrix[2],
      snav_data.attitude_estimate.rotation_matrix[3],
      snav_data.attitude_estimate.rotation_matrix[4],
      snav_data.attitude_estimate.rotation_matrix[5],
      snav_data.attitude_estimate.rotation_matrix[6],
      snav_data.attitude_estimate.rotation_matrix[7],
      snav_data.attitude_estimate.rotation_matrix[8]);

  // Convert Rotation Matrix to quaternion
  RR.getRotation(q);

  // Check for NAN in quaternion
  if(q.getX()!=q.getX() || q.getY()!=q.getY() ||
      q.getZ()!=q.getZ() || q.getW()!=q.getW())
  {
    ROS_WARN("Rotation Quaternion is NAN");
    valid_rotation_est_ = false;
  }
  else
  {
    valid_rotation_est_ = true;
  }
}

void SnavInterface::UpdatePosVelMessages(tf2::Quaternion q)
{
  SnavCachedData snav_data = fci_.get_snav_cached_data();

  tf2::Transform est_tf(tf2::Transform(q, tf2::Vector3(
          snav_data.pos_vel.position_estimated[0],
          snav_data.pos_vel.position_estimated[1],
          snav_data.pos_vel.position_estimated[2])));

  est_transform_msg_.child_frame_id = base_link_frame_;
  est_transform_msg_.header.frame_id = estimation_frame_;

  ros::Time timestamp;
  timestamp = ros::Time((double)(snav_data.pos_vel.time + (dsp_offset_in_ns_/1e3))/1e6);
  est_transform_msg_.header.stamp = timestamp;
  timestamp_ = timestamp;

  tf2::convert(est_tf, est_transform_msg_.transform);

  tf2::toMsg(est_tf, est_pose_msg_.pose);
  est_pose_msg_.header.stamp = timestamp;
  est_pose_msg_.header.frame_id = est_transform_msg_.header.frame_id;

  tf2::Quaternion q_des;
  q_des.setEuler(0.0, 0.0, snav_data.pos_vel.yaw_desired);
  tf2::Transform des_tf(tf2::Transform(q_des, tf2::Vector3(
          snav_data.pos_vel.position_desired[0],
          snav_data.pos_vel.position_desired[1],
          snav_data.pos_vel.position_desired[2])));

  tf2::convert(des_tf, des_transform_msg_.transform);
  des_transform_msg_.child_frame_id = desired_frame_;
  des_transform_msg_.header.frame_id = estimation_frame_;
  des_transform_msg_.header.stamp = timestamp;

  tf2::toMsg(des_tf, des_pose_msg_.pose);
  des_pose_msg_.header.stamp = timestamp;
  des_pose_msg_.header.frame_id = des_transform_msg_.header.frame_id;

  tf2::Matrix3x3 R_eg(tf2::Matrix3x3(
        snav_data.pos_vel.R_eg[0],
        snav_data.pos_vel.R_eg[1],
        snav_data.pos_vel.R_eg[2],
        snav_data.pos_vel.R_eg[3],
        snav_data.pos_vel.R_eg[4],
        snav_data.pos_vel.R_eg[5],
        snav_data.pos_vel.R_eg[6],
        snav_data.pos_vel.R_eg[7],
        snav_data.pos_vel.R_eg[8]));

  tf2::Transform gps_enu_tf(tf2::Transform(R_eg, tf2::Vector3(
          snav_data.pos_vel.t_eg[0],
          snav_data.pos_vel.t_eg[1],
          snav_data.pos_vel.t_eg[2])));

  tf2::convert(gps_enu_tf, gps_enu_transform_msg_.transform);
  gps_enu_transform_msg_.child_frame_id = gps_enu_frame_;
  gps_enu_transform_msg_.header.frame_id = estimation_frame_;
  gps_enu_transform_msg_.header.stamp = timestamp;

  // base_link_no_rot and base_link_stab
  tf2::Matrix3x3 RR_est(q);
  tf2Scalar roll, pitch, yaw;
  RR_est.getRPY(roll, pitch, yaw);

  //tf2::Transform base_link_no_rot_tf(tf2::Transform(
  //      tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3(
  //        snav_data.pos_vel.position_estimated[0],
  //        snav_data.pos_vel.position_estimated[1],
  //        snav_data.pos_vel.position_estimated[2])));
  tf2::Transform base_link_no_rot_tf(tf2::Transform(
        RR_est.inverse(), tf2::Vector3(0.0, 0.0, 0.0)));

  base_link_no_rot_transform_msg_.child_frame_id = base_link_no_rot_frame_;
  base_link_no_rot_transform_msg_.header.frame_id = base_link_frame_;
  base_link_no_rot_transform_msg_.header.stamp = timestamp;
  tf2::convert(base_link_no_rot_tf, base_link_no_rot_transform_msg_.transform);

  tf2::Matrix3x3 RR_yaw;
  RR_yaw.setRPY(0, 0, yaw);
  tf2::Transform base_link_stab_tf(tf2::Transform(RR_yaw,
        tf2::Vector3(0.0, 0.0, 0.0)));

  base_link_stab_transform_msg_.child_frame_id = base_link_stab_frame_;
  base_link_stab_transform_msg_.header.frame_id = base_link_no_rot_frame_;
  base_link_stab_transform_msg_.header.stamp = timestamp;
  tf2::convert(base_link_stab_tf, base_link_stab_transform_msg_.transform);

  tf2::Quaternion q_launch;
  q_launch.setEuler(0.0, 0.0, snav_data.pos_vel.yaw_el);
  tf2::Transform launch_tf(tf2::Transform(q_launch, tf2::Vector3(
          snav_data.pos_vel.t_el[0],
          snav_data.pos_vel.t_el[1],
          snav_data.pos_vel.t_el[2])));

  tf2::convert(launch_tf, launch_transform_msg_.transform);
  launch_transform_msg_.child_frame_id = launch_frame_;
  launch_transform_msg_.header.frame_id = estimation_frame_;
  launch_transform_msg_.header.stamp = timestamp;

  Eigen::Quaternionf q_pw;
  Eigen::Vector3f t_pw;
  fci_.get_waypoint_frame_tf(q_pw, t_pw);
  tf2::Quaternion q_pw_tf2(q_pw.x(), q_pw.y(), q_pw.z(), q_pw.w());
  tf2::Vector3 t_pw_tf2(t_pw(0), t_pw(1), t_pw(2));
  tf2::Transform waypoint_tf(q_pw_tf2, t_pw_tf2);

  tf2::convert(waypoint_tf, waypoint_transform_msg_.transform);
  waypoint_transform_msg_.child_frame_id = waypoint_frame_;
  waypoint_transform_msg_.header.stamp = timestamp;
  if (tx_config_.waypoint_frame_parent == snav_fci::ReferenceFrame::ESTIMATION) {
    waypoint_transform_msg_.header.frame_id = estimation_frame_;
    valid_waypoint_frame_ = true;
  }
  else if (tx_config_.waypoint_frame_parent == snav_fci::ReferenceFrame::LAUNCH) {
    waypoint_transform_msg_.header.frame_id = launch_frame_;
    if (snav_data.pos_vel.launch_tf_is_valid) valid_waypoint_frame_ = true;
    else valid_waypoint_frame_ = false;
  }
  else {
    valid_waypoint_frame_ = false;
  }
}

void SnavInterface::UpdateSimMessages(){

  // Get Rotation Matrix from sn_cached_data_, convert to tf2 Matrix
  SnavCachedData snav_data = fci_.get_snav_cached_data();

  tf2::Matrix3x3 RR(
      snav_data.sim_ground_truth.R[0],
      snav_data.sim_ground_truth.R[1],
      snav_data.sim_ground_truth.R[2],
      snav_data.sim_ground_truth.R[3],
      snav_data.sim_ground_truth.R[4],
      snav_data.sim_ground_truth.R[5],
      snav_data.sim_ground_truth.R[6],
      snav_data.sim_ground_truth.R[7],
      snav_data.sim_ground_truth.R[8]);

  // Convert Rotation Matrix to quaternion
  tf2::Quaternion q;
  RR.getRotation(q);

  // Check for NAN in quaternion
  if(q.getX()!=q.getX() || q.getY()!=q.getY() ||
      q.getZ()!=q.getZ() || q.getW()!=q.getW())
  {
    ROS_WARN("Rotation Quaternion is NAN");
    valid_rotation_sim_gt_ = false;
  }
  else
  {
    valid_rotation_sim_gt_ = true;

    tf2::Transform sim_gt_tf(tf2::Transform(q, tf2::Vector3(
            snav_data.sim_ground_truth.position[0],
            snav_data.sim_ground_truth.position[1],
            snav_data.sim_ground_truth.position[2])));

    sim_gt_tf = sim_gt_tf.inverse();
    sim_gt_transform_msg_.child_frame_id = sim_gt_frame_;
    sim_gt_transform_msg_.header.frame_id = base_link_frame_;

    ros::Time timestamp;
    timestamp = ros::Time((double)(snav_data.sim_ground_truth.time + (dsp_offset_in_ns_/1e3))/1e6);
    sim_gt_transform_msg_.header.stamp = timestamp;

    tf2::convert(sim_gt_tf, sim_gt_transform_msg_.transform);

    tf2::toMsg(sim_gt_tf.inverse(), sim_gt_pose_msg_.pose);
    sim_gt_pose_msg_.header.stamp = timestamp;
    sim_gt_pose_msg_.header.frame_id = sim_gt_frame_;

  }
}

void SnavInterface::UpdateSnavData(){
  if(simulation_) {
    SnavCachedData snav_data = fci_.get_snav_cached_data();

    rosgraph_msgs::Clock simtime;
    simtime.clock = ros::Time((double)(snav_data.general_status.time)/1e6);
    clock_publisher_.publish(simtime);
  }
}

void SnavInterface::UpdateWaypointTf() {
  try {
    std::string parent = tx_config_.waypoint_frame_parent
      == snav_fci::ReferenceFrame::ESTIMATION ?  "odom" : "launch";
    geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(parent,
        "waypoint", ros::Time(0));
    Eigen::Quaternionf q_pw(tf.transform.rotation.w, tf.transform.rotation.x,
        tf.transform.rotation.y, tf.transform.rotation.z);
    Eigen::Vector3f t_pw(tf.transform.translation.x, tf.transform.translation.y,
        tf.transform.translation.z);
    fci_.set_waypoint_frame_tf(q_pw, t_pw);
  }
  catch (const tf2::TransformException& ex) {
    // Do nothing if no transform is found; that way FCI latches onto last one
    // (or the default)
  }
}

void SnavInterface::PublishBatteryVoltage(){
  std_msgs::Float32 voltage_msg;
  SnavCachedData snav_data = fci_.get_snav_cached_data();
  voltage_msg.data = snav_data.general_status.voltage;
  battery_voltage_publisher_.publish( voltage_msg );
}

void SnavInterface::PublishOnGroundFlag(){
  std_msgs::Bool on_ground_msg;
  SnavCachedData snav_data = fci_.get_snav_cached_data();
  on_ground_msg.data = snav_data.general_status.on_ground;
  on_ground_publisher_.publish( on_ground_msg );
}

void SnavInterface::PublishPropsState() {
  std_msgs::String props_state_msg;
  SnavCachedData snav_data = fci_.get_snav_cached_data();
  if ((SnPropsState)snav_data.general_status.props_state == SN_PROPS_STATE_NOT_SPINNING) {
    props_state_msg.data = "NOT_SPINNING";
  }
  else if ((SnPropsState)snav_data.general_status.props_state == SN_PROPS_STATE_STARTING) {
    props_state_msg.data = "STARTING";
  }
  else if ((SnPropsState)snav_data.general_status.props_state == SN_PROPS_STATE_SPINNING) {
    props_state_msg.data = "SPINNING";
  }
  else {
    props_state_msg.data = "UNKNOWN";
  }
  props_state_publisher_.publish(props_state_msg);
}

void SnavInterface::BroadcastEstTf(){
  if(valid_rotation_est_)
    tf_pub_.sendTransform(est_transform_msg_);
  else
    ROS_ERROR("Tried to broadcast invalid Est Tf");
}

void SnavInterface::BroadcastDesiredTf(){
  if(valid_rotation_est_)
    tf_pub_.sendTransform(des_transform_msg_);
  else
    ROS_ERROR("Tried to broadcast invalid Desired Tf");
}

void SnavInterface::BroadcastGpsEnuTf(){
  if(valid_rotation_est_)
    tf_pub_.sendTransform(gps_enu_transform_msg_);
  else
    ROS_ERROR("Tried to broadcast invalid GPS ENU Tf");
}

void SnavInterface::BroadcastBaseLinkNoRotTf(){
  if (valid_rotation_est_){
    tf_pub_.sendTransform(base_link_no_rot_transform_msg_);
  }
  else
    ROS_ERROR("Tried to broadcast invalid base link no rotation Tf");
}

void SnavInterface::BroadcastBaseLinkStabTf(){
  if (valid_rotation_est_){
    tf_pub_.sendTransform(base_link_stab_transform_msg_);
  }
  else
    ROS_ERROR("Tried to broadcast invalid base link stabilized Tf");
}

void SnavInterface::BroadcastLaunchTf(){
  SnavCachedData snav_data = fci_.get_snav_cached_data();

  if(valid_rotation_est_ && snav_data.pos_vel.launch_tf_is_valid)
    tf_pub_.sendTransform(launch_transform_msg_);
}

void SnavInterface::BroadcastWaypointTf() {
  if (valid_waypoint_frame_)
    tf_pub_.sendTransform(waypoint_transform_msg_);
}

void SnavInterface::BroadcastSimGtTf(){
  if (valid_rotation_sim_gt_){
    tf_pub_.sendTransform(sim_gt_transform_msg_);
  }
  else
    ROS_ERROR("Tried to broadcast invalid sim ground truth Tf");
}

void SnavInterface::PublishEstPose(){
  if(valid_rotation_est_)
    pose_est_publisher_.publish(est_pose_msg_);
  else
    ROS_ERROR("Tried to publish invalid Est Pose");
}

void SnavInterface::PublishDesiredPose(){
  if(valid_rotation_est_)
    pose_des_publisher_.publish(des_pose_msg_);
  else
    ROS_ERROR("Tried to publish invalid Desired Pose");
}

void SnavInterface::PublishSimGtPose(){
  if (valid_rotation_sim_gt_)
    pose_sim_gt_publisher_.publish(sim_gt_pose_msg_);
  else
    ROS_ERROR("Tried to publish invalid sim ground truth pose");
}

