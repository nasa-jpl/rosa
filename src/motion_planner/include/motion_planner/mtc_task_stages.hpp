#ifndef MOTION_PLANNER_MTC_TASK_STAGES_HPP
#define MOTION_PLANNER_MTC_TASK_STAGES_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <geometry_msgs/msg/point.hpp>
#include "motion_planner/scene_objects.hpp"
#include <moveit_msgs/msg/constraints.hpp>

namespace mtc = moveit::task_constructor;
extern const rclcpp::Logger LOGGER;

class MTCTaskStages
{
public:
    // Initialize task stages (creates planners internally)
    MTCTaskStages(rclcpp::Node::SharedPtr node);
    
    // Simple task execution - planners managed internally
    void doTask(int cylinder_id);
    void doTaskReturn(int cylinder_id);

private:
    // Compose an MTC task from a series of stages.
    mtc::Task createTask(int cylinder_id);
    mtc::Task createTaskReturn(int cylinder_id);
    
    // Helper function to get cylinder position from scene_objects
    geometry_msgs::msg::Point getCylinderPosition(int cylinder_id);
    
    // ROS2 node reference (for planner initialization)
    rclcpp::Node::SharedPtr node_;
    
    // Planner members (owned and managed by task stages)
    std::shared_ptr<mtc::solvers::PipelinePlanner> sampling_planner_forward_;
    std::shared_ptr<mtc::solvers::PipelinePlanner> sampling_planner_return_;
    std::shared_ptr<mtc::solvers::JointInterpolationPlanner> interpolation_planner_;
    std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner_;
    moveit::core::RobotModelConstPtr robot_model_;
    moveit_msgs::msg::Constraints path_constraints_;
    
    mtc::Task task_;
    mtc::Task taskReturn_;
};

#endif  // MOTION_PLANNER_MTC_TASK_STAGES_HPP
