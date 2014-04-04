/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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
 *********************************************************************/

// Author: Dave Coleman
// Desc:   Generates grasps for a cube

#ifndef MOVEIT_SIMPLE_GRASPS__MOVEIT_SIMPLE_GRASPS_
#define MOVEIT_SIMPLE_GRASPS__MOVEIT_SIMPLE_GRASPS_

// ROS
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/PoseArray.h>
//#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/Grasp.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

// Rviz
#include <moveit_visualization_tools/visualization_tools.h>

// C++
#include <math.h>
#define _USE_MATH_DEFINES

namespace moveit_simple_grasps
{

static const double RAD2DEG = 57.2957795;

struct RobotGraspData
{
  RobotGraspData() :    
    // Fill in default values where possible:
    base_link_("/base_link"), 
    grasp_depth_(0.12), 
    angle_resolution_(16),
    approach_retreat_desired_dist_(0.6),
    approach_retreat_min_dist_(0.4),
    object_size_(0.04)
  {}
  geometry_msgs::Pose grasp_pose_to_eef_pose_; // Convert generic grasp pose to this end effector's frame of reference
  trajectory_msgs::JointTrajectory pre_grasp_posture_; // when the end effector is in "open" position
  trajectory_msgs::JointTrajectory grasp_posture_; // when the end effector is in "close" position
  std::string base_link_; // name of global frame with z pointing up
  std::string ee_parent_link_; // the last link in the kinematic chain before the end effector, e.g. "/gripper_roll_link"
  std::string ee_group_; // the end effector name
  std::string ee_joint_; // the joint to actuate for gripping
  double grasp_depth_; // distance from center point of object to end effector
  int angle_resolution_; // generate grasps at PI/angle_resolution increments
  double approach_retreat_desired_dist_; // how far back from the grasp position the pregrasp phase should be
  double approach_retreat_min_dist_; // how far back from the grasp position the pregrasp phase should be at minimum
  double object_size_; // for visualization
};


// Class
class MoveItSimpleGrasps
{
private:

  // Grasp axis orientation
  enum grasp_axis_t {X_AXIS, Y_AXIS, Z_AXIS};
  enum grasp_direction_t {UP, DOWN};

  // class for publishing stuff to rviz
  moveit_visualization_tools::VisualizationToolsPtr rviz_tools_;

  // Transform from frame of box to global frame
  Eigen::Affine3d object_global_transform_; 

  // Choose whether the end effector is animated and shown for each potential grasp
  bool animate_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen requires 128-bit alignment for the Eigen::Vector2d's array (of 2 doubles). With GCC, this is done with a attribute ((aligned(16))).

  // Constructor
  MoveItSimpleGrasps(moveit_visualization_tools::VisualizationToolsPtr rviz_tools);

  // Destructor
  ~MoveItSimpleGrasps();

  /**
   * \brief Choose whether the end effector is animated and shown for each potential grasp
            only really useful for debugging
   */
  void setAnimateGrasps(bool animate)
  {
    animate_ = animate;
  }

  // Create all possible grasp positions for a object
  bool generateGrasps(const geometry_msgs::Pose& object_pose, const RobotGraspData& grasp_data,
                      std::vector<moveit_msgs::Grasp>& possible_grasps);

  /**
   * \brief Show all grasps in Rviz
   * \param possible_grasps
   * \param object_pose
   * \param grasp_data - custom settings for a robot's geometry
   */ 
  void visualizeGrasps(const std::vector<moveit_msgs::Grasp>& possible_grasps,
    const geometry_msgs::Pose& object_pose, const RobotGraspData& grasp_data);

  /**
   * \brief Animate the pre grasp, grasp, and post-grasp process - for testing and visualization
   * \param grasp - a fully completed manipulation message that descibes a grasp
   */
  void animateGrasp(const moveit_msgs::Grasp &grasp, const RobotGraspData& grasp_data);

  static void printObjectGraspData(const RobotGraspData& data)
  {
    ROS_INFO_STREAM_NAMED("grasp","ROBOT GRASP DATA DEBUG OUTPUT ---------------------");
    ROS_INFO_STREAM_NAMED("grasp","Base Link: " << data.base_link_);
    ROS_INFO_STREAM_NAMED("grasp","EE Parent Link: " << data.ee_parent_link_);
    ROS_INFO_STREAM_NAMED("grasp","Grasp Depth: " << data.grasp_depth_);
    ROS_INFO_STREAM_NAMED("grasp","Angle Resolution: " << data.angle_resolution_);
    ROS_INFO_STREAM_NAMED("grasp","Approach Retreat Desired Dist: " << data.approach_retreat_desired_dist_);
    ROS_INFO_STREAM_NAMED("grasp","Approach Retreat Min Dist: " << data.approach_retreat_min_dist_);
    ROS_INFO_STREAM_NAMED("grasp","Pregrasp Posture: \n" << data.pre_grasp_posture_);
    ROS_INFO_STREAM_NAMED("grasp","Grasp Posture: \n" << data.grasp_posture_);
    ROS_INFO_STREAM_NAMED("grasp","---------------------------------------------------\n");
  }

private:

  // Create grasp positions in one axis
  bool generateAxisGrasps(std::vector<moveit_msgs::Grasp>& possible_grasps, grasp_axis_t axis,
                          grasp_direction_t direction, const RobotGraspData& grasp_data);
                          
}; // end of class

typedef boost::shared_ptr<MoveItSimpleGrasps> MoveItSimpleGraspsPtr;
typedef boost::shared_ptr<const MoveItSimpleGrasps> GraspGeneratorConstPtr;

} // namespace

#endif
