moveit_simple_grasps
====================

Note: this code is refactored and moved from [block_grasp_generator](https://github.com/davetcoleman/block_grasp_generator) to a more general library.

Generate basic grasp poses for simple objects such as blocks or cylinders for use with MoveIt! pick and place

Research code by [Dave Coleman](http://dav.ee) at the Correll Robotics Lab, University of Colorado Boulder

<img align="right" src="https://raw.github.com/davetcoleman/moveit_simple_grasps/hydro-devel/resources/demo.png" />

## Install

NEW: This package now depends on [moveit_visual_tools](https://github.com/davetcoleman/moveit_visual_tools)

## Tested Robots

 - [Baxter](https://github.com/davetcoleman/baxter)
 - [ClamArm](https://github.com/davetcoleman/clam)
 - [REEM](http://wiki.ros.org/Robots/REEM)

## Build Status

[![Build Status](https://travis-ci.org/davetcoleman/moveit_simple_grasps.png?branch=hydro-devel)](https://travis-ci.org/davetcoleman/moveit_simple_grasps)

## Testing and Example Code

There are currently test scripts and examples in:

 - [baxter_pick_place](https://github.com/davetcoleman/baxter/tree/hydro-devel/baxter_pick_place)
 - [reem_tabletop_grasping](https://github.com/pal-robotics/reem_tabletop_grasping).

## Grasp Filter

Creates several threads and tests a large number of potential grasps for kinematic feasibility.

<img align="right" src="https://raw.github.com/davetcoleman/moveit_simple_grasps/hydro-devel/resources/filter.png" />

To run the grasp filter test, add:
 - a RobotState display to Rviz subscribed to ``/move_group/robot_state``
 - a Marker display to Rviz subscribed to ``/end_effector_marker``
 - Run ``roslaunch moveit_simple_grasps grasp_filter_test.launch``