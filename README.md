# Simple Odometry and Costmap Simulator

Simple odometry and costmap simulator for testing local and global planners

## Usage

  - Publish a 2D nav goal using Rviz's "2D Nav Goal" tool and clicking somewhere on the map
  - Set the location & orientation of the robot using Rviz's "2D Nav Goal" tool. The robot will warp to wherever you click
  - Drive the robot manually using rqt's "Robot Steering" tool under `Plugins > Robot Tools`

## Defult Topics

  - By default, everything is launched under the "Actor" namespace
  - **Odometry**: `/Actor/odom`
  - **Map**: `/map`
  - **Steering**: `/Actor/cmd_vel`
  - **Nav goal**: `/Actor/move_base_simple/goal`
  - **Set pose of robot**: `/Actor/set_pose`

## Dynamic reconfigure

Launch dynamic reconfigure (`rqt > Plugins > Configuration`) to view the planner's dynamic parameters

## Launching

To run the simulator with ROS's default local planner:

```
roslaunch simple_sim test.launch planner:=base
```

To run the simulator with TEB local planner:

```
roslaunch simple_sim test.launch planner:=teb
```
