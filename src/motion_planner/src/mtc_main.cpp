#include <rclcpp/rclcpp.hpp>
#include "motion_planner/mtc_task_node.hpp"
#include "motion_planner/mtc_task_stages.hpp"
#include "motion_planner/scene_objects.hpp"

// LOGGER is declared extern in mtc_task_stages.hpp and defined in mtc_task_stages.cpp
// namespace mtc = moveit::task_constructor;


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  // Wait for controllers to be ready
  RCLCPP_INFO(LOGGER, "Waiting for controllers to be ready...");
  rclcpp::sleep_for(std::chrono::seconds(3));
  
  // Setup planning scene through the task node
  mtc_task_node->setupPlanningScene();
  
  RCLCPP_INFO(LOGGER, "Node ready! Call service to execute task:");
  RCLCPP_INFO(LOGGER, "  ros2 service call /execute_task motion_planner/srv/ExecuteTask \"{cylinder_id: 1}\"");
  RCLCPP_INFO(LOGGER, "  ros2 service call /execute_return_task motion_planner/srv/ExecuteTask \"{cylinder_id: 1}\"");
  RCLCPP_INFO(LOGGER, "  ros2 service call /execute_all_tasks motion_planner/srv/ExecuteAllTasks \"{}\"");

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}