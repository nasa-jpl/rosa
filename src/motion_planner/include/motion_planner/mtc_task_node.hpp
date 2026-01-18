#ifndef MOTION_PLANNER_MTC_TASK_NODES_HPP
#define MOTION_PLANNER_MTC_TASK_NODES_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "motion_planner/srv/execute_task.hpp"
#include "motion_planner/srv/execute_all_tasks.hpp"
#include "motion_planner/mtc_task_stages.hpp"

namespace mtc = moveit::task_constructor;
extern const rclcpp::Logger LOGGER;

class MTCTaskNode{
  public:

    MTCTaskNode(const rclcpp::NodeOptions& options);
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
    void setupPlanningScene();

  private:

  // Service callback to trigger task execution
  void executeTaskCallback(
    const std::shared_ptr<motion_planner::srv::ExecuteTask::Request> request,
    std::shared_ptr<motion_planner::srv::ExecuteTask::Response> response);

  // Service callback to trigger task execution
  void executeTaskReturnCallback(
    const std::shared_ptr<motion_planner::srv::ExecuteTask::Request> request,
    std::shared_ptr<motion_planner::srv::ExecuteTask::Response> response);

  // Service callback to execute all tasks automatically
  void executeAllTasksCallback(
    const std::shared_ptr<motion_planner::srv::ExecuteAllTasks::Request> request,
    std::shared_ptr<motion_planner::srv::ExecuteAllTasks::Response> response);
  
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<motion_planner::srv::ExecuteTask>::SharedPtr execute_service_;
  rclcpp::Service<motion_planner::srv::ExecuteTask>::SharedPtr execute_return_service_;
  rclcpp::Service<motion_planner::srv::ExecuteAllTasks>::SharedPtr execute_all_service_;
  bool scene_setup_done_;
  
  // Task execution handler - owns and manages planners
  MTCTaskStages task_stages_;

};
#endif  // MOTION_PLANNER_MTC_TASK_NODES_HPP
