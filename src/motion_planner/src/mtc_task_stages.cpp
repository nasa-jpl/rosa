#include "motion_planner/mtc_task_stages.hpp"
#include <sstream>

// Define LOGGER in cpp file to avoid redefinition
const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_lab_logger");

MTCTaskStages::MTCTaskStages(rclcpp::Node::SharedPtr node) : node_(node)
{
    // Planners will be created fresh for each task in createTask()
    // This ensures they use the same robot model as the task
    
    // Initialize path constraints once (reused for all tasks)
    // Joint constraint: shoulder_pan_joint between -45° and +135°
    moveit_msgs::msg::JointConstraint joint_constraint;
    joint_constraint.joint_name = "shoulder_pan_joint";
    joint_constraint.position = 0.067078675646533;  // Center at ~0° (3.84°), matches initial position
    joint_constraint.tolerance_above = 2.29;   // Allow up to +135° (0.067 + 2.29 ≈ 2.36 rad = 135°)
    joint_constraint.tolerance_below = 0.85;   // Allow down to -45° (0.067 - 0.85 ≈ -0.78 rad = -45°)
    joint_constraint.weight = 1.0;
    path_constraints_.joint_constraints.push_back(joint_constraint);
    
    RCLCPP_INFO(LOGGER, "MTCTaskStages initialized - planners will be created per-task");
    RCLCPP_INFO(LOGGER, "Path constraints initialized (shoulder_pan: -45° to +135° centered at ~0°)");
}

geometry_msgs::msg::Point MTCTaskStages::getCylinderPosition(int cylinder_id)
{
  // Get cylinder positions from scene_objects based on cylinder_id
  auto cylinders = scene_objects::presets::getTargetCylinders();
  
  geometry_msgs::msg::Point pos;
  if (cylinder_id >= 1 && cylinder_id <= static_cast<int>(cylinders.size())) {
    pos = cylinders[cylinder_id - 1].pose.position;
  } else {
    RCLCPP_ERROR(LOGGER, "Invalid cylinder_id: %d, using default position", cylinder_id);
    pos.x = 0.45;
    pos.y = 0.0;
    pos.z = 0.075;
  }
  
  return pos;
}

void MTCTaskStages::doTask(int cylinder_id)
{
  task_ = createTask(cylinder_id);

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  // Try planning with more solutions and print detailed failure info
  RCLCPP_INFO(LOGGER, "Starting task planning...");
  //task_.introspection().publishTaskState(task_);

  if (!task_.plan(2))  // Reduced to 2 solutions for faster planning (was 5)
  {
    RCLCPP_ERROR(LOGGER, "Task planning failed!");
    
    // Print detailed failure information
    std::stringstream ss;
    ss << "\n=== Task State ===\n";
    task_.printState(ss);
    ss << "\n=== Failure Explanation ===\n";
    task_.explainFailure(ss);
    RCLCPP_ERROR_STREAM(LOGGER, ss.str());
    return;
  }
  RCLCPP_INFO(LOGGER, "Planning succeeded! Found %zu solutions", task_.solutions().size());
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}


void MTCTaskStages::doTaskReturn(int cylinder_id)
{
  taskReturn_ = createTaskReturn(cylinder_id);

  try
  {
    taskReturn_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  // Try planning with more solutions and print detailed failure info
  RCLCPP_INFO(LOGGER, "Starting task planning...");
  //taskReturn_.introspection().publishTaskState(taskReturn_);

  if (!taskReturn_.plan(2))  // Reduced to 2 solutions for faster planning (was 5)
  {
    RCLCPP_ERROR(LOGGER, "Task planning failed!");
    
    // Print detailed failure information
    std::stringstream ss;
    ss << "\n=== Task State ===\n";
    taskReturn_.printState(ss);
    ss << "\n=== Failure Explanation ===\n";
    taskReturn_.explainFailure(ss);
    RCLCPP_ERROR_STREAM(LOGGER, ss.str());
    return;
  }
  RCLCPP_INFO(LOGGER, "Planning succeeded! Found %zu solutions", taskReturn_.solutions().size());

  auto result = taskReturn_.execute(*taskReturn_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}



mtc::Task MTCTaskStages::createTask(int cylinder_id)
{
  mtc::Task task;
  task.stages()->setName("UR5e with Robotiq Hand-E Demo");
  // Load robot model fresh for each task - ensures we get the latest model from move_group
  task.loadRobotModel(node_);

  // Create planners fresh for this task with the correct robot model
  // Use RRTConnect for faster planning (much faster than RRTstar)
  auto sampling_planner_forward = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "move_group");
  sampling_planner_forward->setPlannerId("RRTConnectkConfigDefault");  // Faster planner
  sampling_planner_forward->setProperty("goal_joint_tolerance", 0.01);  // Relaxed for faster convergence (was 0.001)
  sampling_planner_forward->setProperty("num_planning_attempts", static_cast<unsigned int>(3));  // Limit attempts for faster failure
  
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setMaxAccelerationScalingFactor(0.5);
  cartesian_planner->setStepSize(.005);  // Even smaller step size to ensure at least 10 waypoints (was 0.01)
  cartesian_planner->setJumpThreshold(1.5);
  cartesian_planner->setMinFraction(0.02);  // Accept 2% completion (was 0.05) to allow very small movements when robot is constrained

  // UR5e with Robotiq Hand-E configuration
  const auto& arm_group_name = "ur_manipulator";
  const auto& hand_group_name = "gripper";
  const auto& tool_frame = "tool0";
  const auto& hand_frame = "robotiq_hande_end";  // Gripper TCP frame

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);
  task.setProperty("timeout", 30.0);  // Increased timeout to allow more planning time (was 15.0)

  // Get cylinder position based on cylinder_id
  auto cylinder_pos = getCylinderPosition(cylinder_id);
  std::string target_object_name = "target_object_" + std::to_string(cylinder_id);

  RCLCPP_INFO(LOGGER, "Creating MTC task for UR5e with Robotiq Hand-E for cylinder %d at (%.2f, %.2f, %.2f)", 
              cylinder_id, cylinder_pos.x, cylinder_pos.y, cylinder_pos.z);

  // Current state
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current state");
  task.add(std::move(stage_state_current));

  // Move to approach position - improved with IK sampling
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move to approach", sampling_planner_forward);
    stage->setGroup(arm_group_name);
    stage->setTimeout(10.0);  // Reduced timeout for faster planning (was 40.0)
    
    // Use pre-defined path constraints from constructor
    //stage->setPathConstraints(path_constraints_);
    
    // Define multiple goal poses - MTC will try all and pick best
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = cylinder_pos.x;
    target_pose.pose.position.y = cylinder_pos.y;
    target_pose.pose.position.z = 0.4;
    target_pose.pose.orientation.x = 1.0;  // Tool pointing down
    target_pose.pose.orientation.y = 0.0;
    target_pose.pose.orientation.z = 0.0;
    target_pose.pose.orientation.w = 0.0;
    
    stage->setGoal(target_pose);
    task.add(std::move(stage));
  }

  //Move down in Cartesian space (toward object)
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("move down", cartesian_planner);
    stage->setGroup(arm_group_name);
    stage->setIKFrame(tool_frame);
    
    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = "base_link";
    direction.vector.z = -0.12;  // Move down 15cm
    stage->setDirection(direction);
    task.add(std::move(stage));
  }

  // Open gripper before approaching object
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("gripper_open");
    task.add(std::move(stage));
  }


  // Before closing gripper
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collisions");
    stage->allowCollisions(target_object_name, 
                         task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNames(),
                         true);
    task.add(std::move(stage));
  }
  // Close gripper to grasp object
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("gripper_closed");
    task.add(std::move(stage));
  }

  // Attach the object to the gripper
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
    stage->attachObject(target_object_name, hand_frame);
    task.add(std::move(stage));
  }

  // Move up in Cartesian space (lift)
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("move up", cartesian_planner);
    stage->setGroup(arm_group_name);
    stage->setIKFrame(tool_frame);
    
    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = "base_link";
    direction.vector.z = 0.2;  // Move up 15cm
    stage->setDirection(direction);
    task.add(std::move(stage));
  }

  // Move to placement location with attached object
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move to place", sampling_planner_forward);
    stage->setGroup(arm_group_name);
    stage->setTimeout(20.0);  // Increased timeout for planning with attached object (was 10.0)
    
    // Use pre-defined path constraints from constructor
    //stage->setPathConstraints(path_constraints_);
    
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = -0.1;
    target_pose.pose.position.y = -0.5;
    target_pose.pose.position.z = 0.4;  // Approach position adjusted for lower table (table top at z=0.1, was 0.3)
    target_pose.pose.orientation.x = 1.0;  // Tool pointing down
    target_pose.pose.orientation.y = 0.0;
    target_pose.pose.orientation.z = 0.0;
    target_pose.pose.orientation.w = 0.0;
    
    stage->setGoal(target_pose);
    task.add(std::move(stage));
  }


      // Move down in Cartesian space (toward table to place object)
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("move down", cartesian_planner);
    stage->setGroup(arm_group_name);
    stage->setIKFrame(tool_frame);
    
    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = "base_link";
    direction.vector.z = -0.225;  // Move down 22.5cm (from z=0.4 to z=0.175 for cylinder center on table at z=0.1)
    stage->setDirection(direction);
    task.add(std::move(stage));
  }


    // Open gripper to grasp object
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("gripper_open");
    task.add(std::move(stage));
  }

  // Detach the object to the gripper
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
    stage->detachObject(target_object_name, hand_frame);
    task.add(std::move(stage));
  }

  // Move up in Cartesian space (lift) - reduced distance for final movement
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("move up", cartesian_planner);
    stage->setGroup(arm_group_name);
    stage->setIKFrame(tool_frame);
    
    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = "base_link";
    direction.vector.z = 0.10;  // Move up 10cm (reduced from 15cm to avoid obstacles)
    stage->setDirection(direction);
    task.add(std::move(stage));
  }

  return task;
}

mtc::Task MTCTaskStages::createTaskReturn(int cylinder_id)
{
  mtc::Task task;
  task.stages()->setName("UR5e with Robotiq Hand-E Demo");
  // Load robot model fresh for each task - ensures we get the latest model from move_group
  task.loadRobotModel(node_);

  // Create planners fresh for this task with the correct robot model
  auto sampling_planner_return = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "move_group");
  sampling_planner_return->setPlannerId("RRTConnectkConfigDefault");  // Faster planner
  sampling_planner_return->setProperty("goal_joint_tolerance", 0.01);  // Relaxed for faster convergence (was 0.001)
  sampling_planner_return->setProperty("num_planning_attempts", static_cast<unsigned int>(3));  // Limit attempts for faster failure
  
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setMaxAccelerationScalingFactor(0.5);
  cartesian_planner->setStepSize(.005);  // Even smaller step size to ensure at least 10 waypoints (was 0.01)
  cartesian_planner->setJumpThreshold(1.5);
  cartesian_planner->setMinFraction(0.02);  // Accept 2% completion (was 0.05) to allow very small movements when robot is constrained

  // UR5e with Robotiq Hand-E configuration
  const auto& arm_group_name = "ur_manipulator";
  const auto& hand_group_name = "gripper";
  const auto& tool_frame = "tool0";
  const auto& hand_frame = "robotiq_hande_end";  // Gripper TCP frame

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);
  task.setProperty("timeout", 30.0);  // Increased timeout to allow more planning time (was 15.0)

  // Get cylinder position based on cylinder_id
  auto cylinder_pos = getCylinderPosition(cylinder_id);
  std::string target_object_name = "target_object_" + std::to_string(cylinder_id);

  RCLCPP_INFO(LOGGER, "Creating MTC return task for UR5e with Robotiq Hand-E for cylinder %d at (%.2f, %.2f, %.2f)", 
              cylinder_id, cylinder_pos.x, cylinder_pos.y, cylinder_pos.z);

  // Current state
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current state");
  task.add(std::move(stage_state_current));

  
  // Move down in Cartesian space (toward object) - reduced distance since robot may already be close
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("move down", cartesian_planner);
    stage->setGroup(arm_group_name);
    stage->setIKFrame(tool_frame);
    
    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = "base_link";
    direction.vector.z = -0.08;  // Move down 8cm (reduced from 15cm - robot may already be close after placement)
    stage->setDirection(direction);
    task.add(std::move(stage));
  }

    // Before closing gripper - allow collisions only with gripper jaws (fingers)
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collisions");
    // Only allow collisions with the gripper finger links (jaws), not the entire gripper
    std::vector<std::string> finger_links = {
      "robotiq_hande_left_finger",
      "robotiq_hande_right_finger"
    };
    stage->allowCollisions(target_object_name, finger_links, true);
    task.add(std::move(stage));
  }
  // Close gripper to grasp object
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("gripper_closed");
    task.add(std::move(stage));
  }

  // Attach the object to the gripper
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
    stage->attachObject(target_object_name, hand_frame);
    task.add(std::move(stage));
  }

    // Move up in Cartesian space (lift) - smaller distance for return task
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("move up", cartesian_planner);
    stage->setGroup(arm_group_name);
    stage->setIKFrame(tool_frame);
    
    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = "base_link";
    direction.vector.z = 0.08;  // Move up 8cm (reduced from 15cm - robot may be constrained after picking from table)
    stage->setDirection(direction);
    task.add(std::move(stage));
  }



  // Move to approach position - improved with IK sampling
  // Using separate return planner for better planning success
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move to approach", sampling_planner_return);
    stage->setGroup(arm_group_name);
    stage->setTimeout(10.0);  // Reduced timeout for faster planning (was 25.0)
    
    // Use pre-defined path constraints from constructor
    //stage->setPathConstraints(path_constraints_);
    
    // Define multiple goal poses - MTC will try all and pick best
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = cylinder_pos.x;
    target_pose.pose.position.y = cylinder_pos.y;
    target_pose.pose.position.z = 0.4;
    target_pose.pose.orientation.x = 1.0;  // Tool pointing down
    target_pose.pose.orientation.y = 0.0;
    target_pose.pose.orientation.z = 0.0;
    target_pose.pose.orientation.w = 0.0;
    
    stage->setGoal(target_pose);
    task.add(std::move(stage));
  }

    // Move down in Cartesian space (toward object to place back)
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("move down", cartesian_planner);
    stage->setGroup(arm_group_name);
    stage->setIKFrame(tool_frame);
    
    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = "base_link";
    direction.vector.z = -0.325;  // Move down 32.5cm (from z=0.4 to z=0.075 for cylinder center at original position)
    stage->setDirection(direction);
    task.add(std::move(stage));
  }

      // Open gripper to grasp object
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("gripper_open");
    task.add(std::move(stage));
  }

  // Detach the object to the gripper
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
    stage->detachObject(target_object_name, hand_frame);
    task.add(std::move(stage));
  }

  // Move up in Cartesian space (lift)
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("move up", cartesian_planner);
    stage->setGroup(arm_group_name);
    stage->setIKFrame(tool_frame);
    
    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = "base_link";
    direction.vector.z = 0.15;  // Move up 15cm
    stage->setDirection(direction);
    task.add(std::move(stage));
  }



  return task;
}









