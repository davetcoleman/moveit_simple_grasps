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

#include <moveit_simple_grasps/grasp_filter.h>
#include <moveit/transforms/transforms.h>

// Conversions
#include <eigen_conversions/eigen_msg.h> // add to pkg TODO

namespace moveit_simple_grasps
{

// Constructor
GraspFilter::GraspFilter( robot_state::RobotState robot_state, bool rviz_verbose,
                          moveit_visual_tools::VisualToolsPtr visual_tools, const std::string& planning_group ):
  robot_state_(robot_state),
  planning_group_(planning_group),
  rviz_verbose_(rviz_verbose),
  visual_tools_(visual_tools)
{
  ROS_INFO_STREAM_NAMED("filter","GraspFilter ready.");
}

GraspFilter::~GraspFilter()
{
}

bool GraspFilter::chooseBestGrasp( const std::vector<moveit_msgs::Grasp>& possible_grasps, moveit_msgs::Grasp& chosen )
{
  // TODO: better logic here
  if( possible_grasps.empty() )
  {
    ROS_ERROR_NAMED("filter","There are no grasps to choose from");
    return false;
  }
  chosen = possible_grasps[0]; // just choose first one
  return true;
}

// Return grasps that are kinematically feasible
bool GraspFilter::filterGrasps(std::vector<moveit_msgs::Grasp>& possible_grasps,
  std::vector<trajectory_msgs::JointTrajectoryPoint>& ik_solutions)
{
  // -----------------------------------------------------------------------------------------------
  // Error check
  if( possible_grasps.empty() )
  {
    ROS_ERROR_NAMED("filter","Unable to filter grasps because vector is empty");
    return false;
  }

  // -----------------------------------------------------------------------------------------------
  // how many cores does this computer have and how many do we need?
  int num_threads = boost::thread::hardware_concurrency();
  if( num_threads > possible_grasps.size() )
    num_threads = possible_grasps.size();

  if(false)
  {
    num_threads = 1;
    ROS_WARN_STREAM_NAMED("grasp_filter","Using only " << num_threads << " threads");
  }

  // -----------------------------------------------------------------------------------------------
  // Get the solver timeout from kinematics.yaml
  double timeout = robot_state_.getRobotModel()->getJointModelGroup( planning_group_ )->getDefaultIKTimeout();
  timeout = 0.05;
  ROS_INFO_STREAM_NAMED("grasp_filter","Planning timeout " << timeout);

  // -----------------------------------------------------------------------------------------------
  // Load kinematic solvers if not already loaded
  if( kin_solvers_.size() != num_threads )
  {
    kin_solvers_.clear();

    const robot_model::JointModelGroup* jmg = robot_state_.getRobotModel()->getJointModelGroup(planning_group_);

    // Create an ik solver for every thread
    for (int i = 0; i < num_threads; ++i)
    {
      //ROS_INFO_STREAM_NAMED("filter","Creating ik solver " << i);

      kin_solvers_.push_back(jmg->getSolverInstance());

      // Test to make sure we have a valid kinematics solver
      if( !kin_solvers_[i] )
      {
        ROS_ERROR_STREAM_NAMED("grasp_filter","No kinematic solver found");
        return false;
      }
    }
  }

  // Transform poses -------------------------------------------------------------------------------
  // bring the pose to the frame of the IK solver
  const std::string &ik_frame = kin_solvers_[0]->getBaseFrame();
  Eigen::Affine3d link_transform;
  if (!moveit::core::Transforms::sameFrame(ik_frame, robot_state_.getRobotModel()->getModelFrame()))
  {
    const robot_model::LinkModel *lm = robot_state_.getLinkModel((!ik_frame.empty() && ik_frame[0] == '/') ? ik_frame.substr(1) : ik_frame);
    if (!lm)
      return false;
    //pose = getGlobalLinkTransform(lm).inverse() * pose;
    link_transform = robot_state_.getGlobalLinkTransform(lm).inverse();
  }

  // Benchmark time
  ros::Time start_time;
  start_time = ros::Time::now();
  {

    // -----------------------------------------------------------------------------------------------
    // Loop through poses and find those that are kinematically feasible
    std::vector<moveit_msgs::Grasp> filtered_grasps;

    boost::thread_group bgroup; // create a group of threads
    boost::mutex lock; // used for sharing the same data structures

    ROS_INFO_STREAM_NAMED("filter", "Filtering possible grasps with " << num_threads << " threads");

    // split up the work between threads
    double num_grasps_per_thread = double(possible_grasps.size()) / num_threads;
    //ROS_INFO_STREAM("total grasps " << possible_grasps.size() << " per thead: " << num_grasps_per_thread);

    int grasps_id_start;
    int grasps_id_end = 0;

    for(int i = 0; i < num_threads; ++i)
    {
      grasps_id_start = grasps_id_end;
      grasps_id_end = ceil(num_grasps_per_thread*(i+1));
      if( grasps_id_end >= possible_grasps.size() )
        grasps_id_end = possible_grasps.size();
      //ROS_INFO_STREAM_NAMED("filter","low " << grasps_id_start << " high " << grasps_id_end);

      IkThreadStruct tc(possible_grasps, filtered_grasps, ik_solutions, link_transform, grasps_id_start, 
                        grasps_id_end, kin_solvers_[i], timeout, &lock, i);
      bgroup.create_thread( boost::bind( &GraspFilter::filterGraspThread, this, tc ) );
    }

    ROS_INFO_STREAM_NAMED("filter","Waiting to join threads...");
    bgroup.join_all(); // wait for all threads to finish
    ROS_INFO_STREAM_NAMED("filter","Done waiting to joint threads...");

    ROS_INFO_STREAM_NAMED("filter", "Found " << filtered_grasps.size() << " ik solutions out of " <<
                          possible_grasps.size() );

    possible_grasps = filtered_grasps;

  }
  // End Benchmark time
  double duration = (ros::Time::now() - start_time).toNSec() * 1e-6;
  ROS_INFO_STREAM_NAMED("filter","Grasp generator IK grasp filtering benchmark time:");
  std::cout << duration << "\t" << possible_grasps.size() << "\n";

  ROS_INFO_STREAM_NAMED("filter","Possible grasps filtered to " << possible_grasps.size() << " options.");

  return true;
}

// Thread for checking part of the possible grasps list
void GraspFilter::filterGraspThread(IkThreadStruct ik_thread_struct)
{
  // Seed state - start at zero
  std::vector<double> ik_seed_state(7); // fill with zeros 
  // TODO do not assume 7 dof

  std::vector<double> solution;
  moveit_msgs::MoveItErrorCodes error_code;
  geometry_msgs::Pose ik_pose;

  // Process the assigned grasps
  for( int i = ik_thread_struct.grasps_id_start_; i < ik_thread_struct.grasps_id_end_; ++i )
  {
    ROS_DEBUG_STREAM_NAMED("filter", "Checking grasp #" << i);

    // Clear out previous solution just in case - not sure if this is needed
    solution.clear();

    // Transform current pose to frame of planning group
    ik_pose = ik_thread_struct.possible_grasps_[i].grasp_pose.pose;
    Eigen::Affine3d eigen_pose;
    tf::poseMsgToEigen(ik_pose, eigen_pose);
    eigen_pose = ik_thread_struct.link_transform_ * eigen_pose;
    tf::poseEigenToMsg(eigen_pose, ik_pose);

    // Test it with IK
    ik_thread_struct.kin_solver_->
      searchPositionIK(ik_pose, ik_seed_state, ik_thread_struct.timeout_, solution, error_code);

    // Results
    if( error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS )
    {
      //ROS_INFO_STREAM_NAMED("filter","Found IK Solution");

      // Copy solution to seed state so that next solution is faster
      ik_seed_state = solution;

      // Lock the result vector so we can add to it for a second
      {
        boost::mutex::scoped_lock slock(*ik_thread_struct.lock_);
        ik_thread_struct.filtered_grasps_.push_back( ik_thread_struct.possible_grasps_[i] );

        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = solution;
        // Copy solution so that we can optionally use it later
        ik_thread_struct.ik_solutions_.push_back(point);
      }

      // TODO: is this thread safe? (prob not)
      if(rviz_verbose_)
        visual_tools_->publishArrow(ik_thread_struct.possible_grasps_[i].grasp_pose.pose);
    }
    else if( error_code.val == moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION )
      ROS_WARN_STREAM_NAMED("filter","Unable to find IK solution for pose.");
    else if( error_code.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT )
    {
      //ROS_INFO_STREAM_NAMED("filter","Unable to find IK solution for pose: Timed Out.");
      //std::copy(solution.begin(),solution.end(), std::ostream_iterator<double>(std::cout, "\n"));
    }
    else
      ROS_INFO_STREAM_NAMED("filter","IK solution error: MoveItErrorCodes.msg = " << error_code);
  }

  ROS_INFO_STREAM_NAMED("filter","Thread " << ik_thread_struct.thread_id_ << " finished");
}

} // namespace
