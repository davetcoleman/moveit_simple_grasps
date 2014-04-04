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

/* Author: Dave Coleman
   Desc:   Tests the grasp generator filter
*/

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// MoveIt
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

// Rviz
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Grasp 
#include <moveit_simple_grasps/moveit_simple_grasps.h>
#include <moveit_simple_grasps/grasp_filter.h>
#include <moveit_simple_grasps/visualization_tools.h>

// Baxter specific properties
#include <moveit_simple_grasps/baxter_data.h>
#include <moveit_simple_grasps/custom_environment2.h>

namespace moveit_simple_grasps
{

// Baxter specific
//static const std::string EE_PARENT_LINK = "right_wrist";
//static const std::string EE_GROUP = "right_hand";
//static const std::string EE_JOINT = "right_endpoint";
//static const std::string BASE_LINK = "/base";

// Table dimensions
static const double TABLE_HEIGHT = .92;
static const double TABLE_WIDTH = .85;
static const double TABLE_DEPTH = .47;
static const double TABLE_X = 0.66;
static const double TABLE_Y = 0;
static const double TABLE_Z = -0.9/2+0.01;


static const double OBJECT_SIZE = 0.04;

class GraspGeneratorTest
{
private:
  // A shared node handle
  ros::NodeHandle nh_;

  // Grasp generator
  moveit_simple_grasps::MoveItSimpleGraspsPtr moveit_simple_grasps_;

  // class for publishing stuff to rviz
  moveit_simple_grasps::VisualizationToolsPtr visual_tools_;

  // class for filter object
  moveit_simple_grasps::GraspFilterPtr grasp_filter_;

  // data for generating grasps
  moveit_simple_grasps::RobotGraspData grasp_data_;

  // which baxter arm are we using
  std::string arm_;
  std::string planning_group_name_;

public:

  // Constructor
  GraspGeneratorTest(int num_tests) 
    : nh_("~"),
      arm_("right"),
      planning_group_name_(arm_+"_arm")
  {

    // ---------------------------------------------------------------------------------------------
    // Load the Robot Viz Tools for publishing to Rviz
    visual_tools_.reset(new moveit_simple_grasps::VisualizationTools(baxter_pick_place::BASE_LINK));
    visual_tools_->setLifetime(40.0);
    visual_tools_->setMuted(false);
    visual_tools_->setEEGroupName(grasp_data_.ee_group_);
    visual_tools_->setPlanningGroupName(planning_group_name_);

    // ---------------------------------------------------------------------------------------------
    // Load grasp generator
    grasp_data_ = baxter_pick_place::loadRobotGraspData(arm_, OBJECT_SIZE); // Load robot specific data
    moveit_simple_grasps_.reset( new moveit_simple_grasps::MoveItSimpleGrasps(visual_tools_) );

    // ---------------------------------------------------------------------------------------------
    // Load grasp filter
    bool rviz_verbose = true;
    grasp_filter_.reset(new moveit_simple_grasps::GraspFilter(baxter_pick_place::BASE_LINK, 
        rviz_verbose, visual_tools_, planning_group_name_) );

    // ---------------------------------------------------------------------------------------------
    // Generate grasps for a bunch of random objects

    geometry_msgs::Pose object_pose;
    std::vector<moveit_msgs::Grasp> possible_grasps;

    // Loop
    for (int i = 0; i < num_tests; ++i)
    {
      ROS_INFO_STREAM_NAMED("test","Adding random object " << i+1 << " of " << num_tests);

      generateRandomObject(object_pose);
      //getTestObject(object_pose);
      visual_tools_->publishObject(object_pose, OBJECT_SIZE, false);

      possible_grasps.clear();

      // Generate set of grasps for one object
      //visual_tools_->setMuted(true); // we don't want to see unfiltered grasps
      moveit_simple_grasps_->generateGrasps( object_pose, grasp_data_, possible_grasps);
      visual_tools_->setMuted(false);

      // Filter the grasp for only the ones that are reachable
      grasp_filter_->filterGrasps(possible_grasps);

      // Visualize them
      moveit_simple_grasps_->visualizeGrasps(possible_grasps, object_pose, grasp_data_);

      // Make sure ros is still going
      if(!ros::ok())
        break;
    }


  }

  void getTestObject(geometry_msgs::Pose& object_pose)
  {
    // Position
    geometry_msgs::Pose start_object_pose;
    geometry_msgs::Pose end_object_pose;

    start_object_pose.position.x = 0.2;
    start_object_pose.position.y = 0.0;
    start_object_pose.position.z = 0.02;

    end_object_pose.position.x = 0.25;
    end_object_pose.position.y = 0.15;
    end_object_pose.position.z = 0.02;

    // Orientation
    double angle = M_PI / 1.5;
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    start_object_pose.orientation.x = quat.x();
    start_object_pose.orientation.y = quat.y();
    start_object_pose.orientation.z = quat.z();
    start_object_pose.orientation.w = quat.w();

    angle = M_PI / 1.1;
    quat = Eigen::Quaterniond(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    end_object_pose.orientation.x = quat.x();
    end_object_pose.orientation.y = quat.y();
    end_object_pose.orientation.z = quat.z();
    end_object_pose.orientation.w = quat.w();

    // Choose which object to test
    object_pose = start_object_pose;
  }

  void generateRandomObject(geometry_msgs::Pose& object_pose)
  {
    // Position
    object_pose.position.x = fRand(0.7,TABLE_DEPTH);
    object_pose.position.y = fRand(-TABLE_WIDTH/2,-0.1);
    object_pose.position.z = TABLE_Z + TABLE_HEIGHT / 2.0 + OBJECT_SIZE / 2.0;
  
    // Orientation
    double angle = M_PI * fRand(0.1,1);
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    object_pose.orientation.x = quat.x();
    object_pose.orientation.y = quat.y();
    object_pose.orientation.z = quat.z();
    object_pose.orientation.w = quat.w();
  }

  /**
   * \brief Get random double between min and max
   */
  double fRand(double fMin, double fMax)
  {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
  }

}; // end of class

} // namespace


int main(int argc, char *argv[])
{
  int num_tests = 10;

  ros::init(argc, argv, "grasp_generator_test");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(5);
  spinner.start();

  // Seed random
  srand(ros::Time::now().toSec());

  // Benchmark time
  ros::Time start_time;
  start_time = ros::Time::now();

  // Run Tests
  moveit_simple_grasps::GraspGeneratorTest tester(num_tests);

  // Benchmark time
  double duration = (ros::Time::now() - start_time).toNSec() * 1e-6;
  ROS_INFO_STREAM_NAMED("","Total time: " << duration);
  std::cout << duration << "\t" << num_tests << std::endl;

  ros::Duration(1.0).sleep(); // let rviz markers finish publishing

  return 0;
}

