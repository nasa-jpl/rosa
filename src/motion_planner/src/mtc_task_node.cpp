#include "motion_planner/mtc_task_node.hpp"
#include "motion_planner/scene_objects.hpp"
#include <thread>
#include <chrono>



// ros service for triggering the task
MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }, 
    scene_setup_done_(false),
    task_stages_(node_)
{
  // Planners are now initialized in MTCTaskStages via task_stages_ constructor
  // This node focuses on ROS2 service routing and scene management
  
  // Create service to trigger task execution
  execute_service_ = node_->create_service<motion_planner::srv::ExecuteTask>(
    "execute_task",
    std::bind(&MTCTaskNode::executeTaskCallback, this, std::placeholders::_1, std::placeholders::_2));
  
      // Create service to trigger task execution
  execute_return_service_ = node_->create_service<motion_planner::srv::ExecuteTask>(
    "execute_return_task",
    std::bind(&MTCTaskNode::executeTaskReturnCallback, this, std::placeholders::_1, std::placeholders::_2));
  
  // Create service to execute all tasks automatically
  execute_all_service_ = node_->create_service<motion_planner::srv::ExecuteAllTasks>(
    "execute_all_tasks",
    std::bind(&MTCTaskNode::executeAllTasksCallback, this, std::placeholders::_1, std::placeholders::_2));
  
  RCLCPP_INFO(LOGGER, "MTC Task Node ready. Call service '/execute_task' to start task execution.");
  RCLCPP_INFO(LOGGER, "Service '/execute_all_tasks' is ready - will execute pick and return tasks for all cylinders automatically.");
}


rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}



void MTCTaskNode::executeTaskCallback(
  const std::shared_ptr<motion_planner::srv::ExecuteTask::Request> request,
  std::shared_ptr<motion_planner::srv::ExecuteTask::Response> response)
{
  if (!scene_setup_done_) {
    response->success = false;
    response->message = "Planning scene not set up yet. Please wait.";
    RCLCPP_ERROR(LOGGER, "Cannot execute task: planning scene not ready");
    return;
  }
  
  // Validate cylinder_id
  if (request->cylinder_id < 1 || request->cylinder_id > 7) {
    response->success = false;
    response->message = "Invalid cylinder_id. Must be between 1 and 7.";
    RCLCPP_ERROR(LOGGER, "Invalid cylinder_id: %d", request->cylinder_id);
    return;
  }
  
  RCLCPP_INFO(LOGGER, "Service triggered - starting task execution for cylinder %d...", request->cylinder_id);
  
  // Execute the task via task_stages_
  task_stages_.doTask(request->cylinder_id);
  
  response->success = true;
  response->message = "Task execution completed successfully for cylinder " + std::to_string(request->cylinder_id);
  RCLCPP_INFO(LOGGER, "Task execution service call completed");
}



void MTCTaskNode::executeTaskReturnCallback(
  const std::shared_ptr<motion_planner::srv::ExecuteTask::Request> request,
  std::shared_ptr<motion_planner::srv::ExecuteTask::Response> response)
{
  if (!scene_setup_done_) {
    response->success = false;
    response->message = "Planning scene not set up yet. Please wait.";
    RCLCPP_ERROR(LOGGER, "Cannot execute task: planning scene not ready");
    return;
  }
  
  // Validate cylinder_id
  if (request->cylinder_id < 1 || request->cylinder_id > 7) {
    response->success = false;
    response->message = "Invalid cylinder_id. Must be between 1 and 7.";
    RCLCPP_ERROR(LOGGER, "Invalid cylinder_id: %d", request->cylinder_id);
    return;
  }
  
  RCLCPP_INFO(LOGGER, "Service triggered - starting return task execution for cylinder %d...", request->cylinder_id);
  
  // Execute the task via task_stages_
  task_stages_.doTaskReturn(request->cylinder_id);
  
  response->success = true;
  response->message = "Return task execution completed successfully for cylinder " + std::to_string(request->cylinder_id);
  RCLCPP_INFO(LOGGER, "Task execution service call completed");
}

void MTCTaskNode::executeAllTasksCallback(
  const std::shared_ptr<motion_planner::srv::ExecuteAllTasks::Request> request,
  std::shared_ptr<motion_planner::srv::ExecuteAllTasks::Response> response)
{
  if (!scene_setup_done_) {
    response->success = false;
    response->message = "Planning scene not set up yet. Please wait.";
    RCLCPP_ERROR(LOGGER, "Cannot execute tasks: planning scene not ready");
    return;
  }
  
  RCLCPP_INFO(LOGGER, "Starting automated execution of all tasks for cylinders 1-7...");
  
  bool all_success = true;
  std::string status_message = "Completed tasks for cylinders: ";
  
  // Execute tasks for each cylinder (1-7)
  for (int cylinder_id = 1; cylinder_id <= 7; ++cylinder_id) {
    RCLCPP_INFO(LOGGER, "=== Processing cylinder %d ===", cylinder_id);
    
    // Execute pick task
    RCLCPP_INFO(LOGGER, "Executing pick task for cylinder %d...", cylinder_id);
    task_stages_.doTask(cylinder_id);
    
    // Wait 2 seconds between pick and return tasks
    RCLCPP_INFO(LOGGER, "Waiting 2 seconds before return task...");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Execute return task
    RCLCPP_INFO(LOGGER, "Executing return task for cylinder %d...", cylinder_id);
    task_stages_.doTaskReturn(cylinder_id);
    
    status_message += std::to_string(cylinder_id);
    if (cylinder_id < 7) {
      status_message += ", ";
    }
  }
  
  response->success = all_success;
  response->message = status_message;
  RCLCPP_INFO(LOGGER, "Automated task execution completed for all cylinders.");
}

void MTCTaskNode::setupPlanningScene()
{
  scene_objects::setupPlanningScene();
  scene_setup_done_ = true;
  RCLCPP_INFO(LOGGER, "Planning scene setup completed successfully");
}
