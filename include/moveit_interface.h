#ifndef MOVEIT_INTERFACE_H_
#define MOVEIT_INTERFACE_H_

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

// Global variables
extern const std::vector<std::string> active_joint_names;


class ESMoveItInterface
{
public:
  /// @brief Constructor
  explicit ESMoveItInterface();

  /// @brief Destructor
  ~ESMoveItInterface();

  // sequence of positions
  bool moveit_plan_joint_positions(const std::vector<double> joint_group_positions);

  // desired pose of the end-effector 
  bool moveit_plan_ee_pose(const geometry_msgs::Pose pose);

  // @brief Use MoveIt's planner to plan a trajectory to achieve the specified end-effector position
  bool moveit_plan_ee_position(double x, double y, double z);

  // desired end-effector orientation expressed as a quaternion
  bool moveit_plan_ee_orientation(const geometry_msgs::Quaternion quat);

  // sequence of goal poses for the end-effector to achieve; goal poses are relative to the 'world' frame
  bool moveit_plan_cartesian_path(const std::vector<geometry_msgs::Pose> waypoints);

  // Execute a Moveit plan on the robot arm
  bool moveit_execute_plan(void);

  /// @brief Set path constraints for a specific link
  /// @param constrained_link - name of the link to constrain as defined in the URDF
  /// @param reference_link - name of the link that the path of the 'constrained_link' is being constrained against
  /// @param quat - desired orientation of the 'constrained_link' relative to the 'reference_link'
  /// @param tolerance - allowable deviation [rad] from the constraint about the x, y, and z axes
  void moveit_set_path_constraint(const std::string constrained_link, const std::string reference_link, const geometry_msgs::Quaternion quat, const double tolerance);

  // Remove any path constraints
  void moveit_clear_path_constraints(void);

  // return the current end-effector pose relative to the 'world' frame
  geometry_msgs::Pose moveit_get_ee_pose(void);

  // Scale the end-effector velocity down from the max velocity specified in the robot model
  void moveit_scale_ee_velocity(const double factor);

  // fields
  static const std::string PLANNING_GROUP_LEFT;  
  static const std::string PLANNING_FRAME;
  const std::string ROSINFO_ES_CELL;
  const std::string eef_link_name;


  moveit::planning_interface::MoveGroupInterface *move_group;                  // MoveIt object to plan and execute desired trajectories
  moveit::core::RobotStatePtr current_state;                                   // RobotState object to contain all the current pos/vel/acc data.
  moveit_visual_tools::MoveItVisualTools *visual_tools;                        // Display text and markers in Rviz

  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;             // Plan object that holds the calculated trajectory

  robot_model_loader::RobotModelLoader *robot_model_loader;
  robot_model::RobotModelPtr kinematic_model;


private:
  Eigen::Isometry3d text_pose;                                                 // Pose of text
  const robot_state::JointModelGroup *joint_model_group;                       // Joints in the defined robot group
  // moveit_visual_tools::MoveItVisualTools *visual_tools;                        
  // moveit::planning_interface::MoveGroupInterface *move_group;                 
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface; // Add/remove collision objects in the scene

};

#endif