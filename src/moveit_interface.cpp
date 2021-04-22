#include "moveit_interface.h"

// Constructor
ESMoveItInterface::ESMoveItInterface()
  {
    // ================================
    // Setup & basic information
    // ================================
    ROS_INFO("Node constructed! \n");

    const std::string ROSINFO_ES_CELL = "EcoSwash_ROS_cell";

    // define planning group
    const std::string PLANNING_GROUP_LEFT = "left_staubli";   

    //  define move_group object as a pointer
    move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_LEFT);

    // get the planning frame
    const std::string PLANNING_FRAME = move_group->getPlanningFrame(); // cell_base
    ROS_INFO_NAMED(ROSINFO_ES_CELL, "Planning frame: %s", PLANNING_FRAME.c_str()); // cell_base

    // get current state of the robot
    current_state = move_group->getCurrentState();

    // raw pointer to refer to the planning group for improved performance
    joint_model_group = current_state->getJointModelGroup(PLANNING_GROUP_LEFT);

    // get the name of the active joints
    const std::vector<std::string>& active_joint_names = move_group->getActiveJoints();
    ROS_INFO("Active Joints: \n");
    std::copy(active_joint_names.begin(), active_joint_names.end(), std::ostream_iterator<std::string>(std::cout, ", "));      

    // print the name of the end-effector link for this group.
    const std::string eef_link_name = move_group->getEndEffectorLink();
    ROS_INFO_NAMED(ROSINFO_ES_CELL, "End effector link: %s", eef_link_name.c_str()); // left_tool0

    // get a list of all the groups in the robot:
    ROS_INFO_NAMED(ROSINFO_ES_CELL, "Available planning groups:");
    std::copy(move_group->getJointModelGroupNames().begin(), 
              move_group->getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", ")); // left_staubli   
              
    // not used for now but can be kept for additional information
    robot_model_loader = new robot_model_loader::RobotModelLoader("robot_description");
    kinematic_model = robot_model_loader->getModel();
    ROS_INFO("Kinematic model frame: %s \n", kinematic_model->getModelFrame().c_str()); // cell_base   


    // ================================
    // Visualization
    // ================================
    namespace rvt = rviz_visual_tools;
    visual_tools = new moveit_visual_tools::MoveItVisualTools("cell_base");
    visual_tools->deleteAllMarkers();
    visual_tools->loadRemoteControl(); // to step through a high level script via buttons in RViz

    text_pose = Eigen::Isometry3d::Identity(); // cell info
    text_pose.translation().x() = 0;
    text_pose.translation().y() = 0;
    text_pose.translation().z() = 2.5;    
    visual_tools->publishText(text_pose, "EcoSwash Cell", rvt::CYAN, rvt::XXLARGE);
    visual_tools->trigger();
  }

// Destructor
ESMoveItInterface::~ESMoveItInterface()
{
  delete move_group;
  delete visual_tools;
  delete joint_model_group;
  delete robot_model_loader;
}

bool ESMoveItInterface::moveit_plan_joint_positions(const std::vector<double> joint_group_positions)
{
  visual_tools->deleteAllMarkers();
  move_group->setJointValueTarget(joint_group_positions);
  bool success = (move_group->plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  visual_tools->publishText(text_pose, "Joint Space Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools->publishTrajectoryLine(my_plan2.trajectory_, joint_model_group);
  visual_tools->trigger();
  return success;
}

// desired pose of the eef
bool ESMoveItInterface::moveit_plan_ee_pose(const geometry_msgs::Pose pose)
{
  visual_tools->deleteAllMarkers();
  move_group->setPoseTarget(pose);
  bool success = (move_group->plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  visual_tools->publishAxisLabeled(pose, "ee_pose");
  visual_tools->publishText(text_pose, "Pose Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools->publishTrajectoryLine(my_plan2.trajectory_, joint_model_group);
  visual_tools->trigger();
  ROS_INFO("success: %d", success);
  return success;
}

bool ESMoveItInterface::moveit_plan_ee_position(double x, double y, double z)
{
  visual_tools->deleteAllMarkers();
  move_group->setPositionTarget(x, y, z);
  bool success = (move_group->plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  ROS_INFO("x, y, z: %f, %f, %f", pose.position.x, pose.position.y, pose.position.z);
  visual_tools->publishAxisLabeled(pose, "ee_pose");
  visual_tools->publishText(text_pose, "Position Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools->publishTrajectoryLine(my_plan2.trajectory_, joint_model_group);
  visual_tools->trigger();
  ROS_INFO("success: %d", success);
  return success;
}

bool ESMoveItInterface::moveit_plan_ee_orientation(const geometry_msgs::Quaternion quat)
{
  visual_tools->deleteAllMarkers();
  move_group->setOrientationTarget(quat.x, quat.y, quat.z, quat.w);
  bool success = (move_group->plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  geometry_msgs::Pose pose;
  pose = moveit_get_ee_pose();
  pose.orientation = quat;
  visual_tools->publishAxisLabeled(pose, "ee_pose");
  visual_tools->publishText(text_pose, "Orientation Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools->publishTrajectoryLine(my_plan2.trajectory_, joint_model_group);
  visual_tools->trigger();
  ROS_INFO("success: %d", success);
  return success;
}


bool ESMoveItInterface::moveit_plan_cartesian_path(const std::vector<geometry_msgs::Pose> waypoints)
{
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO("Visualizing (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
  visual_tools->deleteAllMarkers();
  visual_tools->publishText(text_pose, "Cartesian Path", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools->publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools->publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rviz_visual_tools::SMALL);
  visual_tools->trigger();

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  my_plan2 = plan;
  my_plan2.trajectory_ = trajectory;
  bool success = false;

  if (1.0 - fraction < 0.1)
    success = true;
  return success;
}

bool ESMoveItInterface::moveit_execute_plan(void)
{
  bool success = (move_group->execute(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  return success;
}

void ESMoveItInterface::moveit_set_path_constraint(const std::string constrained_link, const std::string reference_link, const geometry_msgs::Quaternion quat, const double tolerance)
{
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = constrained_link;
  ocm.header.frame_id = reference_link;
  ocm.orientation = quat;
  ocm.absolute_x_axis_tolerance = tolerance;
  ocm.absolute_y_axis_tolerance = tolerance;
  ocm.absolute_z_axis_tolerance = tolerance;

  // this parameter sets the importance of this constraint relative to other constraints that might be present. Closer to '0' means less important.
  ocm.weight = 1.0;

  // Set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group->setPathConstraints(test_constraints);

  // Since there is a constraint, it might take the planner a lot longer to come up with a valid plan - so give it some time
  move_group->setPlanningTime(30.0);
}

// remove any path constraints
void ESMoveItInterface::moveit_clear_path_constraints(void)
{
  move_group->clearPathConstraints();

  // Now that there are no constraints, reduce the planning time to the default
  move_group->setPlanningTime(5.0);
}

// Return the current end-effector pose relative to the 'world' frame
geometry_msgs::Pose ESMoveItInterface::moveit_get_ee_pose(void)
{
  return move_group->getCurrentPose().pose;
}

void ESMoveItInterface::moveit_scale_ee_velocity(const double factor)
{
  move_group->setMaxVelocityScalingFactor(factor);
}
