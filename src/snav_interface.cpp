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

SnavInterface::SnavInterface(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh)
{
  if(sn_get_flight_data_ptr(sizeof(SnavCachedData),&cached_data_)!=0){
    ROS_ERROR("Error getting cached data.\n");
  };
  sn_update_data();
  last_sn_update_ = ros::Time::now();
  last_vel_command_time_ = ros::Time(0);

  // Setup the publishers
  pose_est_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 10);
  pose_des_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("pose_des", 10);
  battery_voltage_publisher_ = nh_.advertise<std_msgs::Float32>("battery_voltage", 10);
  on_ground_publisher_ = nh_.advertise<std_msgs::Bool>("on_ground", 10);

  cmd_vel_subscriber_ = nh_.subscribe("cmd_vel", 10, &SnavInterface::CmdVelCallback, this);
  start_props_subscriber_ = nh_.subscribe("start_props", 10, &SnavInterface::StartPropsCallback, this);
  stop_props_subscriber_ = nh_.subscribe("stop_props", 10, &SnavInterface::StopPropsCallback, this);

  pnh_.param("vio_frame", vio_frame_, std::string("/vio/odom"));
  pnh_.param("optic_flow_frame", optic_flow_frame_, std::string("/optic_flow/odom"));
  pnh_.param("gps_frame", gps_frame_, std::string("/gps/odom"));

  pnh_.param("base_link_frame", base_link_frame_, std::string("/base_link"));

  pnh_.param("vio_desired_frame",vio_base_link_des_frame_,std::string("/vio/base_link_des"));
  pnh_.param("optic_flow_desired_frame",of_base_link_des_frame_,std::string("/optic_flow/base_link_des"));
  pnh_.param("gps_desired_frame",gps_base_link_des_frame_,std::string("/gps/base_link_des"));

  pnh_.param("cmd_vel_type", cmd_vel_type_, 1); // Default Command Mode is VIO


  std::string root_tf_frame;
  pnh_.param("root_tf_frame", root_tf_frame, std::string("VIO"));
  vio_is_root_tf_ = false;
  gps_is_root_tf_ = false;
  of_is_root_tf_ = false;
  if( root_tf_frame.compare("VIO") == 0)
  {
    ROS_INFO("Root TF frame set to VIO.  GPS and OF transforms will be inverted to maintain tf tree structure");
    vio_is_root_tf_ = true;
  }
  else if( root_tf_frame.compare("GPS") == 0)
  {
    ROS_INFO("Root TF frame set to GPS.  VIO and OF transforms will be inverted to maintain tf tree structure");
    gps_is_root_tf_ = true;
  }
  else if( root_tf_frame.compare("OF") == 0)
  {
    ROS_INFO("Root TF frame set to OF (Optic Flow).  VIO and GPS transforms will be inverted to maintain tf tree structure");
    of_is_root_tf_ = true;
  }
  else
  {
    ROS_FATAL("root_tf_frame must be either VIO, GPS, or OF (optic flow), Exiting");
    exit(EXIT_FAILURE);
  }


  GetDSPTimeOffset();

  valid_rotation_est_ = false;
}

void SnavInterface::GetDSPTimeOffset()
{
  // get the adsp offset.
  int64_t dsptime;
  static const char qdspTimerTickPath[] = "/sys/kernel/boot_adsp/qdsp_qtimer";
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


void SnavInterface::PublishLowFrequencyData(const ros::TimerEvent& event)
{
  if( (ros::Time::now()-last_sn_update_) < ros::Duration(1.0) )
  {
    PublishBatteryVoltage();
    PublishOnGroundFlag();
  }
  else
  {
    ROS_ERROR("Tried to publish low frequency data, but sn_update_data() has not been called in at least 1 second");
  }
}

void SnavInterface::CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  commanded_vel_ = *msg;
  last_vel_command_time_ = ros::Time::now();
  SendVelocityCommand();
}

void SnavInterface::StartPropsCallback(const std_msgs::Empty::ConstPtr& msg)
{
  sn_spin_props();
}

void SnavInterface::StopPropsCallback(const std_msgs::Empty::ConstPtr& msg)
{
  sn_stop_props();
}

void SnavInterface::SendVelocityCommand()
{
  float snav_rc_cmd[4];
  SnRcCommandType cmd_type;

  switch (cmd_vel_type_){
    case 1:
      cmd_type = SN_RC_VIO_POS_HOLD_CMD;
      break;
    case 2:
      cmd_type = SN_RC_OPTIC_FLOW_POS_HOLD_CMD;
      break;
    case 3:
      cmd_type = SN_RC_GPS_POS_HOLD_CMD;
      break;
    default:
      ROS_ERROR("Attempted to send a command velocity to snav, but type of rc command was not set");
      return;
  }
  
  sn_apply_cmd_mapping(cmd_type, RC_OPT_LINEAR_MAPPING,
		       commanded_vel_.linear.x,
		       commanded_vel_.linear.y,
		       commanded_vel_.linear.z,
		       commanded_vel_.angular.z,
		       &snav_rc_cmd[0],
		       &snav_rc_cmd[1],
		       &snav_rc_cmd[2],
		       &snav_rc_cmd[3]);

  sn_send_rc_command(cmd_type, RC_OPT_LINEAR_MAPPING,
		     snav_rc_cmd[0],
		     snav_rc_cmd[1],
		     snav_rc_cmd[2],
		     snav_rc_cmd[3]);		     
}



void SnavInterface::UpdatePoseMessages()
{
  tf2::Quaternion q;
  GetRotationQuaternion(q);
  UpdateVIOMessages(q);
  UpdateOFMessages(q);
  UpdateGPSMessages(q);
}

void SnavInterface::GetRotationQuaternion(tf2::Quaternion &q)
{
  // Get Rotation Matrix from sn_cached_data_, convert to tf2 Matrix
  tf2::Matrix3x3 RR( cached_data_->attitude_estimate.rotation_matrix[0],
                     cached_data_->attitude_estimate.rotation_matrix[1],
                     cached_data_->attitude_estimate.rotation_matrix[2],
                     cached_data_->attitude_estimate.rotation_matrix[3],
                     cached_data_->attitude_estimate.rotation_matrix[4],
                     cached_data_->attitude_estimate.rotation_matrix[5],
                     cached_data_->attitude_estimate.rotation_matrix[6],
                     cached_data_->attitude_estimate.rotation_matrix[7],
                     cached_data_->attitude_estimate.rotation_matrix[8]);

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

void SnavInterface::UpdateVIOMessages(tf2::Quaternion q)
{
  // -------------------------------
  // -- VIO TFs and Pose Messages -- 
  // -------------------------------
  tf2::Transform vio_tf(tf2::Transform(q, tf2::Vector3(cached_data_->vio_pos_vel.position_estimated[0],
                                                       cached_data_->vio_pos_vel.position_estimated[1],
                                                       cached_data_->vio_pos_vel.position_estimated[2])));

  // The tf tree must maintain a strict tree structure.  As such, base_link cannont have multiple parents.
  // If you want to use (for example) GPS and VIO at the same time, you must choose one of them to be the
  // parent of base_link.  This is done throught the root_tf_frame param.  The other potential base_link parents
  // (for example gps and optic flow) will have their transform inverted such that base_link is their parent
  if(!vio_is_root_tf_)
    {
      vio_tf = vio_tf.inverse();
      vio_transform_.child_frame_id = vio_frame_;
      vio_transform_.header.frame_id = base_link_frame_;
    }
  else
    {
      vio_transform_.child_frame_id = base_link_frame_;
      vio_transform_.header.frame_id = vio_frame_;
    }

  ros::Time timestamp;
  timestamp = ros::Time((double)(cached_data_->vio_pos_vel.time + (dsp_offset_in_ns_/1e3))/1e6);
  vio_transform_.header.stamp = timestamp;

  tf2::convert(vio_tf, vio_transform_.transform);
    
  // Populate the VIO Pose Object
  tf2::toMsg(vio_tf, vio_pose_.pose);
  vio_pose_.header.stamp = timestamp;
  vio_pose_.header.frame_id = vio_transform_.header.frame_id;

  // Populate the VIO Desired Transform
  tf2::Quaternion q_des;
  q_des.setEuler(0.0, 0.0, cached_data_->vio_pos_vel.yaw_desired); // Set the quaternion for the desired yaw
  tf2::Transform vio_des_tf(tf2::Transform(q_des, tf2::Vector3(cached_data_->vio_pos_vel.position_desired[0],
                                                               cached_data_->vio_pos_vel.position_desired[1],
                                                               cached_data_->vio_pos_vel.position_desired[2])));
  tf2::convert(vio_des_tf, vio_desired_transform_.transform);
  vio_desired_transform_.child_frame_id = vio_base_link_des_frame_;
  vio_desired_transform_.header.frame_id = vio_frame_;
  vio_desired_transform_.header.stamp = timestamp;

  // Populate the VIO Desired Pose Object
  tf2::toMsg(vio_des_tf, vio_desired_pose_.pose);
  vio_desired_pose_.header.stamp = timestamp;
  vio_desired_pose_.header.frame_id = vio_desired_transform_.header.frame_id;
}

void SnavInterface::UpdateOFMessages(tf2::Quaternion q)
{
  // -------------------------------------- 
  // -- Optic Flow TFs and Pose Messages -- 
  // -------------------------------------- 

  tf2::Transform of_tf(tf2::Transform(q, tf2::Vector3(cached_data_->optic_flow_pos_vel.position_estimated[0],
                                                      cached_data_->optic_flow_pos_vel.position_estimated[1],
                                                      cached_data_->optic_flow_pos_vel.position_estimated[2])));
  if(!of_is_root_tf_)
    {
      of_tf = of_tf.inverse();
      of_transform_.child_frame_id = optic_flow_frame_;
      of_transform_.header.frame_id = base_link_frame_;
    }
  else
    {
      of_transform_.child_frame_id = base_link_frame_;
      of_transform_.header.frame_id = optic_flow_frame_;
    }
  ros::Time timestamp;
  timestamp = ros::Time((double)(cached_data_->optic_flow_pos_vel.time + (dsp_offset_in_ns_/1e3))/1e6);
  of_transform_.header.stamp = timestamp;

  tf2::convert(of_tf, of_transform_.transform);
    
  // Populate the Optic Flow Pose Object
  tf2::toMsg(of_tf, of_pose_.pose);
  of_pose_.header.stamp = timestamp;
  of_pose_.header.frame_id = of_transform_.header.frame_id;


  // Populate the OF Desired Transform
  tf2::Quaternion q_des;
  q_des.setEuler(0.0, 0.0, cached_data_->optic_flow_pos_vel.yaw_desired); // Set the quaternion for the desired yaw
  tf2::Transform of_des_tf(tf2::Transform(q_des, tf2::Vector3(cached_data_->optic_flow_pos_vel.position_desired[0],
                                                              cached_data_->optic_flow_pos_vel.position_desired[1],
                                                              cached_data_->optic_flow_pos_vel.position_desired[2])));

  tf2::convert(of_des_tf, of_desired_transform_.transform);
  of_desired_transform_.child_frame_id = of_base_link_des_frame_;
  of_desired_transform_.header.frame_id = optic_flow_frame_;
  of_desired_transform_.header.stamp = timestamp;

  // Populate the Optic Flow Desired Pose Object
  tf2::toMsg(of_des_tf, of_desired_pose_.pose);
  of_desired_pose_.header.stamp = timestamp;
  of_desired_pose_.header.frame_id = of_desired_transform_.header.frame_id;
}

void SnavInterface::UpdateGPSMessages(tf2::Quaternion q)
{
  // -------------------------------
  // -- GPS TFs and Pose Messages --
  // ------------------------------- 

  // Yaw correction for GPS w.r.t. True East
  tf2::Quaternion q_gps_corr;
  q_gps_corr.setEuler( 0.0,0.0,cached_data_->attitude_estimate.magnetic_yaw_offset + 
                       cached_data_->attitude_estimate.magnetic_declination);

  tf2::Transform gps_tf(tf2::Transform(q_gps_corr * q, tf2::Vector3(cached_data_->gps_pos_vel.position_estimated[0],
                                                                    cached_data_->gps_pos_vel.position_estimated[1],
                                                                    cached_data_->gps_pos_vel.position_estimated[2])));
  if(!gps_is_root_tf_)
    {
      gps_tf = gps_tf.inverse();
      gps_transform_.child_frame_id = gps_frame_;
      gps_transform_.header.frame_id = base_link_frame_;
    }
  else
    {
      gps_transform_.child_frame_id = base_link_frame_;
      gps_transform_.header.frame_id = gps_frame_;
    }
  ros::Time timestamp;
  timestamp = ros::Time((double)(cached_data_->gps_pos_vel.time + (dsp_offset_in_ns_/1e3))/1e6);
  gps_transform_.header.stamp = timestamp;

  tf2::convert(gps_tf, gps_transform_.transform);
    
  // Populate the GPS Pose Object
  tf2::toMsg(gps_tf, gps_pose_.pose);
  gps_pose_.header.stamp = timestamp;
  gps_pose_.header.frame_id = gps_transform_.header.frame_id;


  // Populate the GPS Desired Transform
  tf2::Quaternion q_des;
  q_des.setEuler(0.0, 0.0, cached_data_->gps_pos_vel.yaw_desired); // Set the quaternion for the desired yaw
  tf2::Transform gps_des_tf(tf2::Transform(q_des, tf2::Vector3(cached_data_->gps_pos_vel.position_desired[0],
                                                               cached_data_->gps_pos_vel.position_desired[1],
                                                               cached_data_->gps_pos_vel.position_desired[2])));

  tf2::convert(gps_des_tf, gps_desired_transform_.transform);
  gps_desired_transform_.child_frame_id = gps_base_link_des_frame_;
  gps_desired_transform_.header.frame_id = gps_frame_;
  gps_desired_transform_.header.stamp = timestamp;

  // Populate the Optic Flow Desired Pose Object
  tf2::toMsg(gps_des_tf, gps_desired_pose_.pose);
  gps_desired_pose_.header.stamp = timestamp;
  gps_desired_pose_.header.frame_id = gps_desired_transform_.header.frame_id;
}

void SnavInterface::UpdateSnavData(){
  if (sn_update_data() != 0)
  {
    ROS_WARN("sn_update_data failed, not publishing");
    return;
  }
  last_sn_update_ = ros::Time::now();
}

void SnavInterface::PublishBatteryVoltage(){
  std_msgs::Float32 voltage_msg;
  voltage_msg.data = cached_data_->general_status.voltage;
  battery_voltage_publisher_.publish( voltage_msg );
}

void SnavInterface::PublishOnGroundFlag(){
  std_msgs::Bool on_ground_msg;
  on_ground_msg.data = cached_data_->general_status.on_ground;
  on_ground_publisher_.publish( on_ground_msg );
}


// VIO Publish Functions

void SnavInterface::BroadcastVIOTf(){
  if(valid_rotation_est_)
    tf_pub_.sendTransform(vio_transform_);
  else
    ROS_ERROR("Tried to broadcast invalid VIO Tf");
}

void SnavInterface::BroadcastDesiredVIOTf(){
  if(valid_rotation_est_)
    tf_pub_.sendTransform(vio_desired_transform_);
  else
    ROS_ERROR("Tried to broadcast invalid VIO Desired Tf");
}

void SnavInterface::PublishVIOPose(){
  if(valid_rotation_est_)
    pose_est_publisher_.publish(vio_pose_);
  else
    ROS_ERROR("Tried to publish invalid VIO Pose");
}

void SnavInterface::PublishDesiredVIOPose(){
  if(valid_rotation_est_)
    pose_des_publisher_.publish(vio_desired_pose_);
  else
    ROS_ERROR("Tried to publish invalid Desired VIO Pose");
}


// Optic Flow Publish Functions

void SnavInterface::BroadcastOFTf(){
  if(valid_rotation_est_)
    tf_pub_.sendTransform(of_transform_);
  else
    ROS_ERROR("Tried to broadcast invalid Optic Flow Tf");
}

void SnavInterface::BroadcastDesiredOFTf(){
  if(valid_rotation_est_)
    tf_pub_.sendTransform(of_desired_transform_);
  else
    ROS_ERROR("Tried to broadcast invalid Optic Flow Desired Tf");
}

void SnavInterface::PublishOFPose(){
  if(valid_rotation_est_)
    pose_est_publisher_.publish(of_pose_);
  else
    ROS_ERROR("Tried to publish invalid Optic Flow Pose");
}

void SnavInterface::PublishDesiredOFPose(){
  if(valid_rotation_est_)
    pose_des_publisher_.publish(of_desired_pose_);
  else
    ROS_ERROR("Tried to publish invalid Desired Optic Flow Pose");
}


// GPS Publish Functions

void SnavInterface::BroadcastGPSTf(){
  if(valid_rotation_est_)
    tf_pub_.sendTransform(gps_transform_);
  else
    ROS_ERROR("Tried to broadcast invalid GPS Tf");
}

void SnavInterface::BroadcastDesiredGPSTf(){
  if(valid_rotation_est_)
    tf_pub_.sendTransform(gps_desired_transform_);
  else
    ROS_ERROR("Tried to broadcast invalid GPS Desired Tf");
}

void SnavInterface::PublishGPSPose(){
  if(valid_rotation_est_)
    pose_est_publisher_.publish(gps_pose_);
  else
    ROS_ERROR("Tried to publish invalid GPS Pose");
}

void SnavInterface::PublishDesiredGPSPose(){
  if(valid_rotation_est_)
    pose_des_publisher_.publish(gps_desired_pose_);
  else
    ROS_ERROR("Tried to publish invalid Desired GPS Pose");
}
