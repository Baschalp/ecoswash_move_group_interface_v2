/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
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
 *   * Neither the name of SRI International nor the names of its
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

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ecos_move_group_interface");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Starting the move_group_interface node...");

  // -------------
  // --- Setup ---
  // -------------

/* Moveit:
  MoveIt operates on sets of joints called "planning groups" and stores them in an object called the `JointModelGroup`. 
  Throughout MoveIt the terms "planning group" and "joint model group" are used interchangably.
*/

  static const std::string PLANNING_GROUP_RIGHT = "right_robot";
  static const std::string PLANNING_GROUP_LEFT = "left_robot";
  static const std::string PLANNING_GROUP_DUAL = "dual_robot";

  // static const std::string PLANNING_GROUP_LEFT = "left_staubli";
  // The :move_group_interface:`MoveGroupInterface` class can be setup using the name of the planning group to control/plan.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP_LEFT);

  // Use the :planning_scene_interface:`PlanningSceneInterface` class to add/remove collision objects in the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP_LEFT);

  // ---------------------
  // --- Visualization ---
  // ---------------------

/* => Use of rviz MarkerArray:
   User needs to add a new rviz MarkerArray display, and make sure that the MarkerArray is subscribing to the topic 
   that the markers are published on (that topic is a parameter to the MoveIt visual tools constructor in your C++ file). 
*/
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("cell_base");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows to step through a high level script via buttons in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().x() = 0;
  text_pose.translation().y() = 0;
  text_pose.translation().z() = 2.5;
  visual_tools.publishText(text_pose, "EcoSwash Cell", rvt::CYAN, rvt::XXLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // ---------------------------------
  // --- Getting basic information ---
  // ---------------------------------

  // Print the name of the reference frame for this robot
  ROS_INFO_NAMED("EcoSwash_ROS_cell", "Planning frame: %s", move_group.getPlanningFrame().c_str()); // cell_base

  // Print the name of the end-effector link for this group.
  ROS_INFO_NAMED("EcoSwash_ROS_cell", "End effector link: %s", move_group.getEndEffectorLink().c_str()); // left_tool0

  // Get a list of all the groups in the robot:
  ROS_INFO_NAMED("EcoSwash_ROS_cellorial", "Available planning groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), 
            move_group.getJointModelGroupNames().end(), 
            std::ostream_iterator<std::string>(std::cout, ", ")); // left_staubli

  // ----------------------------
  // --- Reading current pose ---
  // ----------------------------

  // Read the current pose from the robot model. Isometry3d contains position and orientation in matrix form.
  const Eigen::Isometry3d& end_effector_state = move_group.getCurrentState()->getGlobalLinkTransform(move_group.getEndEffectorLink().c_str());

  geometry_msgs::Pose start_pose = tf2::toMsg(end_effector_state);

  // Print end-effector pose. The pose is in the model frame
  ROS_INFO_STREAM("Start pose: \n" << start_pose << "\n");

  // -------------------------------
  // --- Planning to a pose goal ---
  // -------------------------------

  // This is only planning, not actually moving the robot
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success;

  visual_tools.prompt("Pause until 'Next' is pressed in Rviz - Stop #1");

  // Setting/Re-setting the start state if required
  // robot_state::RobotState start_state(*move_group.getCurrentState());
  // move_group.setStartState(start_state);

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pose); // add the start_pose into the Pose messages vector

  geometry_msgs::Pose target_poses1 = start_pose; // define target with a temporary assignment of start_pose
  geometry_msgs::Pose end_pose1 = start_pose;     // define the end_pose same as start_pose to bring the robot back to start
 
  // random assignment of displacements in x, y, z for meander movement like path planning trial
  double init_shift_x = 2.0;  // forward - backward (aasuming the start pose is near Home position, just a dirty implementation for now)
  double shift_x = 1.0;       // forward - backward
  double shift_y = 0.3;       // left - rigth
  double shift_z = 0.2;       // up - down

  target_poses1.position.x += init_shift_x;
  target_poses1.position.y -= shift_y;
  target_poses1.position.z -= shift_z;
  waypoints.push_back(target_poses1);  // initial move to start a randomly selected pose

  target_poses1.position.x += shift_x;
  waypoints.push_back(target_poses1);  // forward
  target_poses1.position.z -= shift_z;
  waypoints.push_back(target_poses1);  // down

  target_poses1.position.x -= shift_x;
  waypoints.push_back(target_poses1);  // backward
  target_poses1.position.z -= shift_z;
  waypoints.push_back(target_poses1);  // down

  target_poses1.position.x += shift_x;
  waypoints.push_back(target_poses1);  // forward

  waypoints.push_back(end_pose1);  // away from the hypothetical part, i.e. move back to end_pose = start_pose

/* => Issue: Execution of planned path depending on the initial pose of the robot
    100% path execution with start pose (~0.8m dispalced from Home position): 
    position: 
      x: -4.78001
      y: 1.59999
      z: 1.64379
    orientation: 
      x: 1
      y: -2.38037e-05
      z: 2.3096e-05
      w: 1.27502e-05

    37% path execution with start pose (Home position): 
    position: 
      x: -5.57999
      y: 1.59999
      z: 1.6438
    orientation: 
      x: -0.707113
      y: 0.707101
      z: -1.35674e-05
      w: 1.17404e-05
*/

  // Disabled the velocity scaling factor; but it can be used to limit the velocity by a given factor (0-1]
  // move_group.setMaxVelocityScalingFactor(0.1);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.00; // keep at 0 for now. Greater values did not result in a successful plan
  const double eef_step = 0.01;
  bool avoid_collision = true;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("EcoSwash_ROS_cell", "Visualizing plan (cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "End effector goal", rvt::WHITE, rvt::XXLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);

  bool b_visualize_waypoints_flag = true;
    // visualize the manually added waypoint poses
  if (b_visualize_waypoints_flag)
  {
    for (std::size_t i = 0; i < waypoints.size(); ++i) {
      visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    }  
  }
  
  visual_tools.trigger();
  visual_tools.prompt("Pause until 'Next' is pressed in Rviz - Stop #2");

  // ---------------------------------------------------------------
  // --- Adding/Removing objects and attaching/detaching objects ---
  // ---------------------------------------------------------------

  // Load mesh
  shapes::Mesh* c_mesh = shapes::createMeshFromResource("package://ecoswash/urdf/Cars/tesla_modelX.stl");
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(c_mesh, mesh_msg);
  shape_msgs::Mesh car_mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  // Define a collision object ROS message.  
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "car";

  // Add the car mesh to the collision object
  collision_object.meshes.resize(1);
  collision_object.mesh_poses.resize(1);
  collision_object.meshes[0] = car_mesh;
  collision_object.mesh_poses[0].position.x = 0.2;
  collision_object.mesh_poses[0].position.y = 0.0;
  collision_object.mesh_poses[0].position.z = 0.35;

  collision_object.meshes.push_back(car_mesh);
  collision_object.mesh_poses.push_back(collision_object.mesh_poses[0]);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Add the collision object into the world
  ROS_INFO_NAMED("EcoSwash_ROS_cell", "Add the object into the world");
  planning_scene_interface.applyCollisionObjects(collision_objects);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Car added", rvt::WHITE, rvt::XXXLARGE);
  visual_tools.trigger();

  visual_tools.prompt("Press 'Next' to terminate the node - Stop #3");

  ros::shutdown();
  return 0;
}
