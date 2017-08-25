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
#ifndef _SNAV_INTERFACE_H_
#define _SNAV_INTERFACE_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Empty.h>

#include <snav/snapdragon_navigator.h>

class SnavInterface
{
public:
  /**
   * Constructor.
   * @param nh
   *   nodehandle to intialize the node.
   * @param pnh
   *   private namespace nodehandle for this node
   */
  SnavInterface(ros::NodeHandle nh, ros::NodeHandle pnh);

  /**
   * Pack VIO, Optic Flow, and GPS data into ros messages and tfs
   **/
  void UpdatePoseMessages();

  /**
   * Update snav cached_data from snav RPC call
   **/
  void UpdateSnavData();

  /**
   * Publish estimation_frame_ -> base_link_frame_ transform
   */
  void BroadcastEstTf();

  /**
   * Publish base_link_frame_ -> base_link_no_rot_frame_ transform
   */
  void BroadcastBaseLinkNoRotTf();

  /**
   * Publish base_link_no_rot_frame_ -> base_link_stab_frame_ transform
   */
  void BroadcastBaseLinkStabTf();

  /**
   * Publish estimation_frame_ -> desired_frame_ transform
   */
  void BroadcastDesiredTf();

  /**
   * Publish estimation_frame_ -> gps_enu_frame_ transform
   */
  void BroadcastGpsEnuTf();

  /**
   * Publish base_link_frame_ -> sim_gt_frame_ transform
   */
  void BroadcastSimGtTf();

  /**
   * Publish base_link pose in estimation_frame_ as geometry_msgs/PoseStamped
   */
  void PublishEstPose();

  /**
   * Publish desired pose in estimation_frame_ as geometry_msgs/PoseStamped
   */
  void PublishDesiredPose();

  /**
   * Publish base_link pose in sim_gt_frame_ as geometry_msgs/PoseStamped
   */
  void PublishSimGtPose();

  /**
   * Publish battery voltage and on_groung flag as Float32 and Bool msgs
   * @param event
   *   Required argument for a function passed to a ros timer, This function
   *   is intended to be attached via nodehandle::createtimer
   */
  void PublishLowFrequencyData(const ros::TimerEvent& event);

  /**
   * Callback function for velocity input
   * @param msg
   *   geometry_msgs/Twist ros message.  linear x, y, z and angular z
   *   are used. Note if this callback is active, it also maps and sends commdands to
   *   snav via RPC call
   */
  void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

  /**
   * Callback function to start propellers via ros message
   * @param msg
   *   Empty message
   */
  void StartPropsCallback(const std_msgs::Empty::ConstPtr& msg);

  /**
   * Callback function to stop propellers via ros message
   * @param msg
   *   Empty message
   */
  void StopPropsCallback(const std_msgs::Empty::ConstPtr& msg);

private:
  void GetRotationQuaternion(tf2::Quaternion &q);
  void UpdatePosVelMessages(tf2::Quaternion q);
  void UpdateSimMessages();

  void PublishBatteryVoltage();
  void PublishOnGroundFlag();

  void SendVelocityCommand();
  void GetDSPTimeOffset();

  void SetRcCommandType(std::string rc_cmd_type_string);
  void SetRcMappingType(std::string rc_cmd_mapping_string);
  
  ros::Publisher battery_voltage_publisher_;
  ros::Publisher pose_est_publisher_;
  ros::Publisher pose_des_publisher_;
  ros::Publisher pose_sim_gt_publisher_;
  ros::Publisher on_ground_publisher_;
  ros::Publisher clock_publisher_;

  ros::Subscriber cmd_vel_subscriber_;
  ros::Subscriber start_props_subscriber_;
  ros::Subscriber stop_props_subscriber_;

  //public namespace nodehandle
  ros::NodeHandle nh_;
  //private namespace nodehandle
  ros::NodeHandle pnh_;

  tf2_ros::TransformBroadcaster tf_pub_;

  geometry_msgs::PoseStamped est_pose_msg_;
  geometry_msgs::PoseStamped des_pose_msg_;
  geometry_msgs::PoseStamped sim_gt_pose_msg_;
  geometry_msgs::TransformStamped est_transform_msg_;
  geometry_msgs::TransformStamped des_transform_msg_;
  geometry_msgs::TransformStamped gps_enu_transform_msg_;
  geometry_msgs::TransformStamped sim_gt_transform_msg_;
  geometry_msgs::TransformStamped base_link_stab_transform_msg_;
  geometry_msgs::TransformStamped base_link_no_rot_transform_msg_;

  geometry_msgs::Twist commanded_vel_;

  SnavCachedData *cached_data_;

  bool valid_rotation_est_;
  bool valid_rotation_sim_gt_;

  ros::Time last_sn_update_;
  ros::Time last_vel_command_time_;

  int64_t dsp_offset_in_ns_;

  // Params
  std::string gps_enu_frame_;
  std::string estimation_frame_;
  std::string base_link_frame_;
  std::string base_link_stab_frame_;
  std::string base_link_no_rot_frame_;
  std::string desired_frame_;
  std::string sim_gt_frame_;

  SnRcCommandType rc_cmd_type_;
  SnRcCommandOptions rc_cmd_mapping_;

  bool simulation_;
};

#endif
