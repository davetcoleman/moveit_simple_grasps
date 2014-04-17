/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2014, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/* Authors: Bence Magyar
   Desc:    Function to load robot grasp data from param server
*/

#include <ros/ros.h>
#include <moveit_simple_grasps/simple_grasps.h> // has datastructure


namespace grasp_data_loader
{

std::string robot_description_="robot_description";
std::string base_link_ = "base_link";
std::vector<std::string> joint_names;
std::vector<double> pre_grasp_posture_;
std::vector<double> grasp_posture_;
std::vector<double> grasp_pose_to_eef_;
double pregrasp_time_from_start_;
double grasp_time_from_start_;
std::string end_effector_name_;
std::string end_effector_parent_link_;

// robot dimensions
double FLOOR_TO_BASE_HEIGHT;

moveit_simple_grasps::RobotGraspData loadRobotGraspData(const ros::NodeHandle& nh, const std::string& side)
{
  ROS_ASSERT(nh.hasParam("floor_to_base_height"));
  nh.getParam("floor_to_base_height", FLOOR_TO_BASE_HEIGHT);

  ROS_ASSERT(nh.hasParam("robot_description"));
  nh.getParam("robot_description", robot_description_);

  ROS_ASSERT(nh.hasParam("base_link"));
  nh.getParam("base_link", base_link_);

  ROS_ASSERT(nh.hasParam("pregrasp_time_from_start"));
  nh.getParam("pregrasp_time_from_start", pregrasp_time_from_start_);

  ROS_ASSERT(nh.hasParam("grasp_time_from_start"));
  nh.getParam("grasp_time_from_start", grasp_time_from_start_);

  ROS_ASSERT(nh.hasParam("end_effector_name"));
  nh.getParam("end_effector_name", end_effector_name_);

  ROS_ASSERT(nh.hasParam("end_effector_parent_link"));
  nh.getParam("end_effector_parent_link", end_effector_parent_link_);

  ROS_ASSERT(nh.hasParam("joints"));
  XmlRpc::XmlRpcValue joint_list;
  nh.getParam("joints", joint_list);
  ROS_ASSERT(joint_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < joint_list.size(); ++i)
  {
    ROS_ASSERT(joint_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    joint_names.push_back(static_cast<std::string>(joint_list[i]));
  }

  if(nh.hasParam("pregrasp_posture"))
  {
    XmlRpc::XmlRpcValue preg_posture_list;
    nh.getParam("pregrasp_posture", preg_posture_list);
    ROS_ASSERT(preg_posture_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < preg_posture_list.size(); ++i)
    {
      ROS_ASSERT(preg_posture_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      pre_grasp_posture_.push_back(static_cast<double>(preg_posture_list[i]));
    }
  }

  ROS_ASSERT(nh.hasParam("grasp_posture"));
  XmlRpc::XmlRpcValue grasp_posture_list;
  nh.getParam("grasp_posture", grasp_posture_list);
  ROS_ASSERT(grasp_posture_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < grasp_posture_list.size(); ++i)
  {
    ROS_ASSERT(grasp_posture_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    grasp_posture_.push_back(static_cast<double>(grasp_posture_list[i]));
  }

  ROS_ASSERT(nh.hasParam("grasp_pose_to_eef"));
  XmlRpc::XmlRpcValue g_to_eef_list;
  nh.getParam("grasp_pose_to_eef", g_to_eef_list);
  ROS_ASSERT(g_to_eef_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < g_to_eef_list.size(); ++i)
  {
    ROS_ASSERT(g_to_eef_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    grasp_pose_to_eef_.push_back(static_cast<double>(g_to_eef_list[i]));
  }

  moveit_simple_grasps::RobotGraspData grasp_data;

  // -------------------------------
  // Convert generic grasp pose to this end effector's frame of reference, approach direction for short

  // Orientation
  double angle = M_PI / 2;  // turn on Z axis //TODO parametrize this
  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitY()));
  grasp_data.grasp_pose_to_eef_pose_.orientation.x = quat.x();
  grasp_data.grasp_pose_to_eef_pose_.orientation.y = quat.y();
  grasp_data.grasp_pose_to_eef_pose_.orientation.z = quat.z();
  grasp_data.grasp_pose_to_eef_pose_.orientation.w = quat.w();

  // Position // approach vector?
  ROS_ASSERT(grasp_pose_to_eef_.size() == 3);
  grasp_data.grasp_pose_to_eef_pose_.position.x = grasp_pose_to_eef_[0];
  grasp_data.grasp_pose_to_eef_pose_.position.y = grasp_pose_to_eef_[1];
  grasp_data.grasp_pose_to_eef_pose_.position.z = grasp_pose_to_eef_[2];

  // -------------------------------
  // Create pre-grasp posture if specified
  if(!pre_grasp_posture_.empty())
  {
    grasp_data.pre_grasp_posture_.header.frame_id = base_link_;
    grasp_data.pre_grasp_posture_.header.stamp = ros::Time::now();
    // Name of joints:
    grasp_data.pre_grasp_posture_.joint_names = joint_names;
    // Position of joints
    grasp_data.pre_grasp_posture_.points.resize(1);
    grasp_data.pre_grasp_posture_.points[0].positions = pre_grasp_posture_;
    grasp_data.pre_grasp_posture_.points[0].time_from_start = ros::Duration(pregrasp_time_from_start_);
  }
  // -------------------------------
  // Create grasp posture
  grasp_data.grasp_posture_.header.frame_id = base_link_;
  grasp_data.grasp_posture_.header.stamp = ros::Time::now();
  // Name of joints:
  grasp_data.grasp_posture_.joint_names = joint_names;
  // Position of joints
  grasp_data.grasp_posture_.points.resize(1);
  grasp_data.grasp_posture_.points[0].positions = grasp_posture_;
  grasp_data.grasp_posture_.points[0].time_from_start = ros::Duration(grasp_time_from_start_);

  // -------------------------------
  // SRDF Info
  grasp_data.base_link_ = base_link_;
  grasp_data.ee_parent_link_ = end_effector_parent_link_;
  grasp_data.ee_group_ = end_effector_name_;

  // -------------------------------
  // Nums
  grasp_data.approach_retreat_desired_dist_ = 0.3;
  grasp_data.approach_retreat_min_dist_ = 0.06;
  // distance from center point of object to end effector
  grasp_data.grasp_depth_ = 0.06;// in negative or 0 this makes the grasps on the other side of the object! (like from below)

  // generate grasps at PI/angle_resolution increments
  grasp_data.angle_resolution_ = 16; //TODO parametrize this, or move to action interface

  // Debug
  //moveit_simple_grasps::SimpleGrasps::printObjectGraspData(grasp_data);

  return grasp_data;
}



} // namespace
