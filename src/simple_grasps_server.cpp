/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
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

/* Author: Bence Magyar
   Desc:   Action server wrapper for object grasp generator. Currently only works for REEM robot, 
           needs to be changed to work with yaml configuration file instead.
*/

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <actionlib/server/simple_action_server.h>

// Grasp generation
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_simple_grasps/GenerateGraspsAction.h>


// Baxter specific properties
#include <moveit_simple_grasps/grasp_data_loader.h>
#include <moveit_simple_grasps/custom_environment2.h>

namespace moveit_simple_grasps
{

  class GraspGeneratorServer
  {
  private:
    // A shared node handle
    ros::NodeHandle nh_;

    // Action server
    actionlib::SimpleActionServer<moveit_simple_grasps::GenerateGraspsAction> as_;
    moveit_simple_grasps::GenerateGraspsResult result_;

    // Grasp generator
    moveit_simple_grasps::SimpleGraspsPtr simple_grasps_;

    // class for publishing stuff to rviz
    moveit_visual_tools::VisualToolsPtr visual_tools_;

    // robot-specific data for generating grasps
    moveit_simple_grasps::RobotGraspData grasp_data_;

    // which arm are we using
    std::string side_;
    std::string planning_group_name_;

  public:

    // Constructor
    GraspGeneratorServer(const std::string &name, const std::string &side)
      : nh_("~")
      , as_(nh_, name, boost::bind(&moveit_simple_grasps::GraspGeneratorServer::executeCB, this, _1), false)
      , side_(side)
      , planning_group_name_(side_+"_arm")
    {
      // ---------------------------------------------------------------------------------------------
      // Load grasp data specific to our robot
      grasp_data_ = grasp_data_loader::loadRobotGraspData(nh_, side_); // Load robot specific data

      // ---------------------------------------------------------------------------------------------
      // Load the Robot Viz Tools for publishing to Rviz
      visual_tools_.reset(new moveit_visual_tools::VisualTools(grasp_data_loader::base_link_));
      visual_tools_->setLifetime(120.0);
      visual_tools_->setMuted(false);
      visual_tools_->setEEGroupName(grasp_data_.ee_group_);
      visual_tools_->setPlanningGroupName(planning_group_name_);

      // ---------------------------------------------------------------------------------------------
      // Load grasp generator
      simple_grasps_.reset( new moveit_simple_grasps::SimpleGrasps(visual_tools_) );
      as_.start();
    }

    void executeCB(const moveit_simple_grasps::GenerateGraspsGoalConstPtr &goal)
    {
      // ---------------------------------------------------------------------------------------------
      // Remove previous results
      result_.grasps.clear();

      // ---------------------------------------------------------------------------------------------
      // Set object width and generate grasps
      grasp_data_.object_size_ = goal->width;
      simple_grasps_->generateBlockGrasps(goal->pose, grasp_data_, result_.grasps);

      // ---------------------------------------------------------------------------------------------
      // Publish results
      as_.setSucceeded(result_);
    }

  };
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "grasp_generator_server");
  moveit_simple_grasps::GraspGeneratorServer grasp_generator_server("generate", "right");
  ros::spin();
  return 0;
}
