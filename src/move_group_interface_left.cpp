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

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ecos_move_group_interface");
  ros::NodeHandle node_handle;

  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  ROS_INFO("Starting the move_group_interface node...");

  // =============================================================================================================
  // Setup
  // =============================================================================================================

/* Moveit:
  MoveIt operates on sets of joints called "planning groups" and stores them in an object called the `JointModelGroup`. 
  Throughout MoveIt the terms "planning group" and "joint model group" are used interchangably.
*/


/* ==> Note related to joint_state_publisher
  http://wiki.ros.org/urdf/Tutorials/Building%20a%20Movable%20Robot%20Model%20with%20URDF
  As you move the sliders around in the GUI, the model moves in Rviz. How is this done? 
  First the GUI parses the URDF and finds all the non-fixed joints and their limits. 
  Then, it uses the values of the sliders to publish sensor_msgs/JointState messages. 
  Those are then used by robot_state_publisher to calculate all of transforms between the different parts. 
  The resulting transform tree is then used to display all of the shapes in Rviz. 
*/

/* => Ideallly, only dual_robot to be used. Keep also individual robots for trials
  static const std::string PLANNING_GROUP_RIGHT = "right_robot";
  static const std::string PLANNING_GROUP_LEFT = "left_robot";
  static const std::string PLANNING_GROUP_DUAL = "dual_robot";
*/

  static const std::string PLANNING_GROUP_LEFT = "left_staubli";
  // The :move_group_interface:`MoveGroupInterface` class can be setup using the name of the planning group to control/plan.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP_LEFT);

  // Use the :planning_scene_interface:`PlanningSceneInterface` class to add/remove collision objects in the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP_LEFT);

  // =============================================================================================================
  // Visualization
  // =============================================================================================================

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

  // =============================================================================================================
  // Getting basic information
  // =============================================================================================================

  // Print the name of the reference frame for this robot
  ROS_INFO_NAMED("EcoSwash_ROS_cell", "Planning frame: %s", move_group.getPlanningFrame().c_str()); // cell_base
  // ROS_INFO_NAMED("EcoSwash_ROS_cell", "Pose reference frame: %s", move_group.getPoseReferenceFrame().c_str()); // cell_base

  // Print the name of the end-effector link for this group.
  ROS_INFO_NAMED("EcoSwash_ROS_cell", "End effector link: %s", move_group.getEndEffectorLink().c_str()); // left_tool0

  // Get a list of all the groups in the robot:
  ROS_INFO_NAMED("EcoSwash_ROS_cellorial", "Available planning groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), 
            move_group.getJointModelGroupNames().end(), 
            std::ostream_iterator<std::string>(std::cout, ", ")); // left_staubli

  // =============================================================================================================
  // Reading current pose
  // =============================================================================================================

  // Read the current pose from the robot model. Isometry3d contains position and orientation in matrix form.
  const Eigen::Isometry3d& end_effector_state1 = move_group.getCurrentState()->getGlobalLinkTransform(move_group.getEndEffectorLink().c_str());

  geometry_msgs::Pose start_pose1 = tf2::toMsg(end_effector_state1);

  // Print end-effector pose. The pose is in the model frame
  ROS_INFO_STREAM("Start pose of end effector before moving to Home: \n" << start_pose1 << "\n");

  // This is only planning, not actually moving the robot
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  my_plan.planning_time_ = 60;

  // =============================================================================================================
  // Moving the robot to the Home position (plan and move with Joints + ExternalAxis) (joint-space goal)
  // =============================================================================================================

  /* => Default Home position:
        ExternalAxis [m], Joint1 , Joint2  , Joint3 , Joint4, Joint5  , Joint6 [rad]
        0.7             , -1.5708, 0.261799, 2.35619, 0     , 0.523599, 1.5708
        0.7             , -90    , 15      , 135    , 0     , 30      , 90 [deg]
  */

  double init_posExtAxis = +0.7; // [m]
  double init_rotJ1 = -90.00;  // [deg]
  double init_rotJ2 = +15.00;  // [deg]
  double init_rotJ3 = +135.00; // [deg]
  double init_rotJ4 = 0.00;    // [deg]
  double init_rotJ5 = +30.00;  // [deg]
  double init_rotJ6 = +90.00;  // [deg]

  // Create a pointer that references the current robot's state. RobotState object contains all the current pos/vel/acc data.
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

  // Get the current set of joint values for the defined group
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  
  // Print the joint_group_positions for each joint before assignment
  ROS_INFO("joint_group_positions before moving to Home position");
  std::copy(joint_group_positions.begin(), 
            joint_group_positions.end(), 
            std::ostream_iterator<double>(std::cout, ", ")); 

  visual_tools.prompt("Pause until 'Next' is pressed in Rviz (start moving)");

  // Modify one of the joints, plan to the new joint space goal and visualize the plan. 
  joint_group_positions[0] = init_posExtAxis;
  move_group.setJointValueTarget(joint_group_positions);
  move_group.move();

  joint_group_positions[1] = init_rotJ1 * M_PI/180;  // deg->radians
  move_group.setJointValueTarget(joint_group_positions);
  move_group.move();

  joint_group_positions[2] = init_rotJ2 * M_PI/180;  
  move_group.setJointValueTarget(joint_group_positions);
  move_group.move();

  joint_group_positions[3] = init_rotJ3 * M_PI/180;  
  move_group.setJointValueTarget(joint_group_positions);
  move_group.move();

  joint_group_positions[4] = init_rotJ4 * M_PI/180;
  move_group.setJointValueTarget(joint_group_positions);
  move_group.move();    

  joint_group_positions[5] = init_rotJ5 * M_PI/180;  
  move_group.setJointValueTarget(joint_group_positions);
  move_group.move();

  joint_group_positions[6] = init_rotJ6 * M_PI/180;  
  move_group.setJointValueTarget(joint_group_positions);
  move_group.move();  

/* => Plan and visualize a JointStateGoal
  bool success_jointGoal = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("EcoSwash_ROS_cell", "Visualizing plan for joint space goal: %s", success_jointGoal ? "Success" : "Failed");
  
  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint space goal", rvt::WHITE, rvt::XXLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
*/

  // Print the joint_group_positions for each joint after assignment
  ROS_INFO("joint_group_positions after moving to Home position");
  std::copy(joint_group_positions.begin(), 
            joint_group_positions.end(), 
            std::ostream_iterator<double>(std::cout, ", ")); 

  // =============================================================================================================
  // Planning to a pose goal
  // =============================================================================================================

// Read the current pose from the robot model. Isometry3d contains position and orientation in matrix form.
  const Eigen::Isometry3d& end_effector_state_home = move_group.getCurrentState()->
                                                     getGlobalLinkTransform(move_group.getEndEffectorLink().c_str());

  geometry_msgs::Pose start_pose_home = tf2::toMsg(end_effector_state_home);

  // Print end-effector pose. The pose is in the model frame
  ROS_INFO_STREAM("Start pose of end effector after moving to Home: \n" << start_pose_home << "\n");

/* => Setting/Re-setting the start state if required
  robot_state::RobotState start_state2(*move_group.getCurrentState());
  start_state2.setFromIK(joint_model_group, start_pose_home);
  move_group.setStartState(start_state2);
*/

  // Re-setting the start state to start_pose_home instead of move_group.setStartState() method (Normally not required but just in case)
  move_group.setPoseTarget(start_pose_home);
  move_group.move(); 

  std::vector<geometry_msgs::Pose> waypoints;
  /* If the start_pose added as first waypoint, make sure to skip this point in the traj; o/w time at t[0]=t[1]=0 and robot doesnt move */
  waypoints.push_back(start_pose_home); // add the start_pose into the Pose messages vector

  geometry_msgs::Pose target_poses1 = start_pose_home; // define target with a temporary assignment of start_pose
  geometry_msgs::Pose end_pose1 = start_pose_home;     // define the end_pose same as start_pose to bring the robot back to start
 
  // random assignment of displacements in x, y, z for meander movement like path planning trial
  double init_shift_x = 2.0;  // forward - backward (assuming the start pose is near Home position, just a dirty implementation for now)
  double shift_x = 3.0;       // forward - backward
  double shift_y = 0.3;       // left - rigth
  double shift_z = 0.25;      // up - down
  double disturb_move = 0.05;    // random disturbance value

  target_poses1.position.x += init_shift_x - disturb_move;
  target_poses1.position.y -= shift_y;
  target_poses1.position.z -= shift_z - disturb_move;
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

/* // Append the home position as a final waypoint */
  // waypoints.push_back(end_pose1);      // away from the hypothetical part, i.e. move back to end_pose = start_pose

/* // Try adding a small disturbance to travel for eef = 0.01 so that execute can(?) succeed
  target_poses1.position.x -= disturb_move;
  waypoints.push_back(target_poses1);  // initial move to start a randomly selected pose
*/

/* => Directly setting the pose and moving the robot instead of planning and executing through computeCartesianPath 
  move_group.setPoseTarget(end_pose1);
  move_group.move();  
*/

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "End effector goal", rvt::WHITE, rvt::XXLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);

  // visualize the manually added waypoint poses
  bool b_visualize_waypoints_flag = true;
  if (b_visualize_waypoints_flag)
  {
    for (std::size_t i = 0; i < waypoints.size(); ++i) {
      visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    }  
  }
  ROS_INFO_NAMED("EcoSwash_ROS_cell", "Size of waypoints: %zu", waypoints.size());
  visual_tools.trigger();

  // =============================================================================================================
  // Planning and execution of a manually designed meander movement (through computeCartesianPath)
  // =============================================================================================================

  // Velocity scaling factor (0-1]
  // move_group.setMaxVelocityScalingFactor(0.1);

  // Planning with moveit::planning_interface::MoveGroupInterface::computeCartesianPath
  /* => Known fact related to <100% planning issue through computeCartesianPath
    https://answers.ros.org/question/332783/move_groupmove-vs-cartesian-paths/
    => " - Use tighter waypoints and fall back to move_group.move() if the cartesian path planning fails
         - Use named poses that are safe to move to, if you can"
    *****************
    https://groups.google.com/g/moveit-users/c/wsxMwps2V4w/m/1_O3_y2PVxoJ
    => "  Randomness in planning is true for calling a sampling based planner like OMPL. When "planning" cartesian paths with MoveIt!, OMPL is not used. 
        In fact, the desired path of the robot is already known through the cartesian motion constraints, so what happens behind the scenes is 
        calling IK for the waypoints of the cartesian path. This is also where potential randomness comes in when using the KDL plugin. 
        It will first try solving IK for the provided seed configuration. 
        If it fails with that, it will try again from random start configurations (depending on the parameters it is called with). 
        Given that the seed position should be the prior waypoint on the cartesian path, 
        it should be a good starting point for the solver and any random configuration should be far less likely to result in a solution afterwards, however.
          In short, the two sources of randomness in the described scenario are:
          - (Minor) variations in the configuration of the planning problem 
          - Randomness in the KDL solver
          Randomness due to sampling based planning does not apply in the given scenario"
    *****************
    https://groups.google.com/g/moveit-users/c/wdhnOOim4sI/m/C6f5_D8P2QkJ
    => "  The problem is that computeCartesianPath() just calls IK repeatedly, and if you are using a 6DOF IK solver, 
        it will only be happy if all poses are exactly reachable along the path.  With a 5DOF robot, 
        the chance is vanishingly small that any interpolated 6DOF cartesian pose will be reachable.  
        (That said, if your start and goal poses are both achievable and your max_step size is longer than the requested path, it should succeed.)
          The solution is to find or write an IK solver which returns success when the 5DOF robot gets as close as it "should" to a given 6DOF pose.  
        (Where you need to define "should" appropriately for your application.  
        Maybe position is more important than orientation and IK should return success if XYZ are correct and Yaw is correct but roll and/or pitch don't need to match.)"
    https://answers.ros.org/question/264134/moveit-incomplete-cartesian-path-execution/
    *****************   
    https://groups.google.com/g/moveit-users/c/I4sPhq_JGQk/m/53J-fAB_IFEJ   
    => "  Actually, numerical solvers are better at this Cartesian motion problem than the IKFast plugin, so I would stick with KDL.
        The fact that you can find IK for each point individually in no way guarantees that you can move between those points with straight-line motion."
       "  The Cartesian solver is actually implemented in the RobotState class. It does not use OMPL at all. 
        OMPL, in the context of MoveIt, is a set of libraries that is often used for complex planning tasks in joint space, not Cartesian space.
          MoveIt itself is a framework for motion planning. It establishes a common pipeline for managing planning tasks, 
        and it enables all the different pieces of planning, including collision detection, to work together.
          To help with Cartesian planning tasks, the moveit simple interface (that is what is typically used for these simpler problems) provides a service 
        to plan the Cartesian path. This enables Cartesian planning from python, because there is no python access to the robot state class. 
        It does not use the typical moveit pipeline, however, and can't really be compared to OMPL style planning tasks."
    *****************
      => plan() vs computeCartesianPath():
         - move_group.plan() tries to find a possible trajectory to reach the specified pose.
         - move_group.computeCartesianPath() creates a trajectory, connecting your waypoints in cartesian space with straight lines.
  */

  // If the current pose is the same as start pose, the robot doesn't move and trajectory[1].time = 0
  // Apparently this is an issue for the move/execute methods. So, initially disturb the tool pose and
  // Skip the start pose in the waypoints vector
  std::vector<geometry_msgs::Pose> sub_waypoints(&waypoints[1], &waypoints[waypoints.size()]);; // if waypoints[0] = start pose
    
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 5.0; // deafult = 0. All other values (0.0001 to 3) didn't result in a 1/1 plan with eef = 0.2
  const double eef_step = 0.1;    // 0.01
  bool avoid_collision = true;
  /* ==> IKFast
  https://ros-planning.github.io/moveit_tutorials/doc/ikfast/ikfast_tutorial.html
  rosrun moveit_kinematics auto_create_ikfast_moveit_plugin.sh --iktype Transform6D 
                           ~/catkin_ws/src/ecoswash/urdf/ecoswash_cell_moveit.urdf "left_staubli" "cell_base" "left_tool0"
  */

  /* Dynamic configurations: rosrun rqt_reconfigure rqt_reconfigure */
  ROS_INFO("Sleep 1 sec before computeCartesianPath");
  sleep(1.0);

  /* Note about planning time: Planning time limits are for planners. 
     computeCartesianPath(..) is not a planner and does not take those limits into account. - gvdhoorn  (Apr 12 '18)*/
  move_group.setPlanningTime(20); // doesn't actually have affect on computeCartesianPath but joint and pose move() commands
    move_group.clearPathConstraints();

  double fraction_success = 0.0;
  int attempt_count = 0;
  int max_attempt = 2000;
  while(fraction_success != 1.0 && attempt_count <= max_attempt)
  {
    // fraction_success = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, avoid_collision); // if waypoints[0] = next pose
    fraction_success = move_group.computeCartesianPath(sub_waypoints, eef_step, jump_threshold, trajectory, avoid_collision); // if waypoints[0] = next pose
    attempt_count++;
    ROS_INFO("Planning attempt: %d", attempt_count);
  }

  // double fraction = move_group.computeCartesianPath(sub_waypoints, eef_step, jump_threshold, trajectory); // if waypoints[0] = start pose
  // double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory); // if waypoints[0] = next pose
  // ROS_INFO_NAMED("EcoSwash_ROS_cell", "Visualizing plan (cartesian path) (%.2f%% acheived)", fraction * 100.0);

  ROS_INFO_NAMED("EcoSwash_ROS_cell", "Visualizing plan (cartesian path) after %d count (%.2f%% acheived)", attempt_count, fraction_success * 100.0);

  // Creare a vector array for trajectory object
  std::vector<double> pt_time_v(trajectory.joint_trajectory.points.size(), -1);
 
  // check the time stamt for each point in the trajectory
  for(std::size_t ind=0; ind < trajectory.joint_trajectory.points.size(); ++ind) {
    pt_time_v[ind] = trajectory.joint_trajectory.points[ind].time_from_start.toSec();
  }
  // Log the time info
  ROS_INFO_NAMED("EcoSwash_ROS_cell", "Size of joint_trajectory.points: %zu", pt_time_v.size());
  ROS_INFO("trajectory.joint_trajectory.points[ ].time_from_start.toSec() values: ");
  std::copy(pt_time_v.begin(), 
            pt_time_v.end(), 
            std::ostream_iterator<double>(std::cout, ", "));   
  
  visual_tools.prompt("Pause until 'Next' is pressed in Rviz (to actually move the robot along the trajectory)");

  // Planning and Execution of the plan after assigning the waypoints
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan2;


/* => This script section below executes the plan through computeCartesianPath
  // Create a RobotTrajectory object
  robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), "left_staubli");

  // Get a RobotTrajectory from trajectory
  rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
 
  // Create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;

  // Compute computeTimeStamps
  bool success_rt = iptp.computeTimeStamps(rt);
  ROS_INFO("Computed time stamp %s", success_rt? "computeTimeStamps SUCCEDED" : "computeTimeStamps FAILED");

  // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(trajectory);

  // Plan and execute the trajectory
  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
  my_plan2.trajectory_ = trajectory;
  ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",fraction * 100.0);   

  sleep(4.0);
  move_group.execute(my_plan2);

  visual_tools.prompt("Pause until 'Next' is pressed in Rviz - Stop #1.5");
*/
///

/* => Execute for long the cartesian trajectory with small eef mostly not successful */
  my_plan.trajectory_ = trajectory;

  ROS_INFO("Execution LOG-1");
  sleep(1.0);
  move_group.execute(my_plan);
  ROS_INFO("Execution LOG-2");
///

  // Move the enf effector to home position
  visual_tools.prompt("Pause until 'Next' is pressed in Rviz - Move the robot to home position");
  move_group.setPoseTarget(end_pose1);
  move_group.move();  
  // Move the base back to home position
  joint_group_positions[0] = init_posExtAxis;
  move_group.setJointValueTarget(joint_group_positions);
  move_group.move();


/*
  // =============================================================================================================
  // Adding/Removing objects and attaching/detaching objects
  // =============================================================================================================

  visual_tools.prompt("Pause until 'Next' is pressed in Rviz - Object will be added");

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

*/


  ros::shutdown();
  return 0;
}
