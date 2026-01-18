#ifndef MOTION_PLANNER_SCENE_OBJECTS_HPP
#define MOTION_PLANNER_SCENE_OBJECTS_HPP

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <string>
#include <vector>

namespace scene_objects {

// Helper function to create colors easily
inline std_msgs::msg::ColorRGBA createColor(float r, float g, float b, float a = 1.0f) {
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

// Pre-defined colors
namespace colors {
  inline std_msgs::msg::ColorRGBA RED() { return createColor(1.0f, 0.0f, 0.0f); }
  inline std_msgs::msg::ColorRGBA GREEN() { return createColor(0.0f, 1.0f, 0.0f); }
  inline std_msgs::msg::ColorRGBA BLUE() { return createColor(0.0f, 0.0f, 1.0f); }
  inline std_msgs::msg::ColorRGBA YELLOW() { return createColor(1.0f, 1.0f, 0.0f); }
  inline std_msgs::msg::ColorRGBA ORANGE() { return createColor(1.0f, 0.5f, 0.0f); }
  inline std_msgs::msg::ColorRGBA PURPLE() { return createColor(0.5f, 0.0f, 0.5f); }
  inline std_msgs::msg::ColorRGBA CYAN() { return createColor(0.0f, 1.0f, 1.0f); }
  inline std_msgs::msg::ColorRGBA MAGENTA() { return createColor(1.0f, 0.0f, 1.0f); }
  inline std_msgs::msg::ColorRGBA WHITE() { return createColor(1.0f, 1.0f, 1.0f); }
  inline std_msgs::msg::ColorRGBA BLACK() { return createColor(0.0f, 0.0f, 0.0f); }
  inline std_msgs::msg::ColorRGBA GRAY() { return createColor(0.5f, 0.5f, 0.5f); }
  inline std_msgs::msg::ColorRGBA BROWN() { return createColor(0.6f, 0.4f, 0.2f); }
}

// Struct for cylinder objects
struct CylinderObject {
  std::string id;
  std::string frame_id;
  double height;
  double radius;
  geometry_msgs::msg::Pose pose;
  std_msgs::msg::ColorRGBA color;
  
  // Constructor with default values
  CylinderObject(const std::string& obj_id = "cylinder",
                 const std::string& frame = "base_link",
                 double h = 0.15,
                 double r = 0.01)
    : id(obj_id), frame_id(frame), height(h), radius(r) {
    pose.orientation.w = 1.0;  // Default orientation
    // Default color: light gray
    color.r = 0.5f;
    color.g = 0.5f;
    color.b = 0.5f;
    color.a = 1.0f;
  }
  
  // Method to convert to CollisionObject
  moveit_msgs::msg::CollisionObject toCollisionObject() const {
    moveit_msgs::msg::CollisionObject obj;
    obj.id = id;
    obj.header.frame_id = frame_id;
    obj.primitives.resize(1);
    obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    obj.primitives[0].dimensions = {height, radius};
    obj.pose = pose;
    return obj;
  }
};

// Struct for box objects (walls, containers)
struct BoxObject {
  std::string id;
  std::string frame_id;
  double length;  // x dimension
  double width;   // y dimension
  double height;  // z dimension
  geometry_msgs::msg::Pose pose;
  std_msgs::msg::ColorRGBA color;
  
  // Constructor with default values
  BoxObject(const std::string& obj_id = "box",
            const std::string& frame = "base_link",
            double l = 0.1,
            double w = 0.1,
            double h = 0.1)
    : id(obj_id), frame_id(frame), length(l), width(w), height(h) {
    pose.orientation.w = 1.0;  // Default orientation
    // Default color: light brown
    color.r = 0.6f;
    color.g = 0.4f;
    color.b = 0.2f;
    color.a = 1.0f;
  }
  
  // Method to convert to CollisionObject
  moveit_msgs::msg::CollisionObject toCollisionObject() const {
    moveit_msgs::msg::CollisionObject obj;
    obj.id = id;
    obj.header.frame_id = frame_id;
    obj.primitives.resize(1);
    obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    obj.primitives[0].dimensions = {length, width, height};
    obj.pose = pose;
    return obj;
  }
};

// Struct for container with walls
struct ContainerBox {
  std::string name_prefix;
  std::string frame_id;
  double center_x;
  double center_y;
  double size_x;
  double size_y;
  double wall_thickness;
  double wall_height;
  
  // Constructor
  ContainerBox(const std::string& prefix = "container",
               const std::string& frame = "base_link",
               double cx = 0.0, double cy = 0.0,
               double sx = 0.1, double sy = 0.1,
               double thickness = 0.02, double height = 0.08)
    : name_prefix(prefix), frame_id(frame),
      center_x(cx), center_y(cy),
      size_x(sx), size_y(sy),
      wall_thickness(thickness), wall_height(height) {}
  
  // Generate all four walls
  std::vector<BoxObject> generateWalls() const {
    std::vector<BoxObject> walls;
    
    // Front wall
    BoxObject front(name_prefix + "_front_wall", frame_id,
                    wall_thickness, size_y, wall_height);
    front.pose.position.x = center_x + size_x/2;
    front.pose.position.y = center_y;
    front.pose.position.z = wall_height/2;
    front.pose.orientation.w = 1.0;
    front.color = colors::GRAY();
    walls.push_back(front);
    
    // Back wall
    BoxObject back(name_prefix + "_back_wall", frame_id,
                   wall_thickness, size_y, wall_height);
    back.pose.position.x = center_x - size_x/2;
    back.pose.position.y = center_y;
    back.pose.position.z = wall_height/2;
    back.pose.orientation.w = 1.0;
    back.color = colors::GRAY();

    walls.push_back(back);
    
    // Left wall
    BoxObject left(name_prefix + "_left_wall", frame_id,
                   size_x, wall_thickness, wall_height);
    left.pose.position.x = center_x;
    left.pose.position.y = center_y + size_y/2;
    left.pose.position.z = wall_height/2;
    left.pose.orientation.w = 1.0;
    left.color = colors::GRAY();
    walls.push_back(left);
    
    // Right wall
    BoxObject right(name_prefix + "_right_wall", frame_id,
                    size_x, wall_thickness, wall_height);
    right.pose.position.x = center_x;
    right.pose.position.y = center_y - size_y/2;
    right.pose.position.z = wall_height/2;
    right.pose.orientation.w = 1.0;
    right.color = colors::GRAY();
    walls.push_back(right);
    
    return walls;
  }
};

// Pre-defined object configurations
namespace presets {

// Target cylinder objects
inline std::vector<CylinderObject> getTargetCylinders() {
  std::vector<CylinderObject> cylinders;
  
  
  // Object 1
  CylinderObject obj1("target_object_1", "base_link", 0.15, 0.01);
  obj1.pose.position.x = 0.5;  // Flipped to other side (was -0.5)
  obj1.pose.position.y = 0.15;  // Flipped along y-axis (was -0.15)
  obj1.pose.position.z = 0.075;
  obj1.color = colors::GREEN();
  cylinders.push_back(obj1);
  
  // Object 2
  CylinderObject obj2("target_object_2", "base_link", 0.15, 0.01);
  obj2.pose.position.x = 0.5;  // Flipped to other side (was -0.5)
  obj2.pose.position.y = 0.1;  // Flipped along y-axis (was -0.1)
  obj2.pose.position.z = 0.075;
  obj2.color = colors::GREEN();
  cylinders.push_back(obj2);
  
  // Object 3
  CylinderObject obj3("target_object_3", "base_link", 0.15, 0.01);
  obj3.pose.position.x = 0.5;  // Flipped to other side (was -0.5)
  obj3.pose.position.y = 0.05;  // Flipped along y-axis (was -0.05)
  obj3.pose.position.z = 0.075;
  obj3.color = colors::GREEN();
  cylinders.push_back(obj3);
  
  // Object 4
  CylinderObject obj4("target_object_4", "base_link", 0.15, 0.01);
  obj4.pose.position.x = 0.5;  // Flipped to other side (was -0.5)
  obj4.pose.position.y = 0.0;  // Flipped along y-axis (was -0.0)
  obj4.pose.position.z = 0.075;
  obj4.color = colors::GREEN();
  cylinders.push_back(obj4);


  // Object 5
  CylinderObject obj5("target_object_5", "base_link", 0.15, 0.01);
  obj5.pose.position.x = 0.5;  // Flipped to other side (was -0.5)
  obj5.pose.position.y = -0.05;  // Flipped along y-axis (was 0.05)
  obj5.pose.position.z = 0.075;
  obj5.color = colors::GREEN();
  cylinders.push_back(obj5);
  
  // Object 6
  CylinderObject obj6("target_object_6", "base_link", 0.15, 0.01);
  obj6.pose.position.x = 0.5;  // Flipped to other side (was -0.5)
  obj6.pose.position.y = -0.1;  // Flipped along y-axis (was 0.1)
  obj6.pose.position.z = 0.075;
  obj6.color = colors::GREEN();
  cylinders.push_back(obj6);
  
  // Object 7
  CylinderObject obj7("target_object_7", "base_link", 0.15, 0.01);
  obj7.pose.position.x = 0.5;  // Flipped to other side (was -0.5)
  obj7.pose.position.y = -0.15;  // Flipped along y-axis (was 0.15)
  obj7.pose.position.z = 0.075;
  obj7.color = colors::GREEN();
  cylinders.push_back(obj7);
  

  return cylinders;
}

// Container boxes
inline std::vector<ContainerBox> getContainers() {
  std::vector<ContainerBox> containers;
  
  // Container surrounding the cylinders (cylinders at x=0.5, y from -0.15 to 0.15)
  ContainerBox container1("container1", "base_link", 0.5, 0.0, 0.1, 0.4, 0.02, 0.08);
  containers.push_back(container1);
  
  // // Second container
  // ContainerBox container2("container2", "base_link", -0.6, 0.0, 0.4, 0.1, 0.02, 0.08);
  // containers.push_back(container2);
  
  return containers;
}

// Table objects
inline std::vector<BoxObject> getTables() {
  std::vector<BoxObject> tables;

  // Table: 0.2m x 0.2m footprint, 0.1m height (reduced by 20cm from 0.3m)
  BoxObject table("table_1", "base_link",
                  0.2,  // length (x)
                  0.2,  // width  (y)
                  0.1); // height (z) - reduced by 20cm
  // Place table so top surface is at z = 0.1, center z = 0.05 (adjusted for new height)
  // Table back to original position (behind robot)
  table.pose.position.x = -0.1; 
  table.pose.position.y = -0.5;  // Back to original position (behind robot)
  table.pose.position.z = 0.05;  // Center adjusted: was 0.15, now 0.05 (bottom stays at z=0.0)
  table.pose.orientation.w = 1.0;
  // Set a custom color - wooden brown
  table.color = colors::BROWN();
  tables.push_back(table);

  return tables;
}

}  // namespace presets

// Lab Scene Contruction
void setupPlanningScene()
{
  moveit::planning_interface::PlanningSceneInterface psi;
  
  // Use predefined scene objects from header file - much cleaner!
  auto cylinders = scene_objects::presets::getTargetCylinders();
  auto containers = scene_objects::presets::getContainers();
  auto tables = scene_objects::presets::getTables();
  
  // Collect all collision objects and their colors
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  std::vector<moveit_msgs::msg::ObjectColor> object_colors;
  
  // Add all cylinder objects
  for (const auto& cylinder : cylinders) {
    collision_objects.push_back(cylinder.toCollisionObject());

    moveit_msgs::msg::ObjectColor obj_color;
    obj_color.id = cylinder.id;
    obj_color.color = cylinder.color;
    object_colors.push_back(obj_color);
  }

  // Add all table objects
  for (const auto& table : tables) {
    collision_objects.push_back(table.toCollisionObject());
    
    moveit_msgs::msg::ObjectColor obj_color;
    obj_color.id = table.id;
    obj_color.color = table.color;
    object_colors.push_back(obj_color);
  }
  
  // Add all container walls
  for (const auto& container : containers) {
    auto walls = container.generateWalls();
    for (const auto& wall : walls) {
      collision_objects.push_back(wall.toCollisionObject());
      
      moveit_msgs::msg::ObjectColor obj_color;
      obj_color.id = wall.id;
      obj_color.color = wall.color;
      object_colors.push_back(obj_color);
    }
  }
  
  // Apply all collision objects at once
  psi.applyCollisionObjects(collision_objects);
  
  // Create a PlanningScene message to apply colors
  moveit_msgs::msg::PlanningScene planning_scene_msg;
  planning_scene_msg.is_diff = true;  // Only update colors, not the entire scene
  planning_scene_msg.object_colors = object_colors;
  
  // Apply the planning scene with colors
  psi.applyPlanningScene(planning_scene_msg);
  
  // RCLCPP_INFO(LOGGER, "Planning scene setup complete - added %zu cylinder objects and %zu container boxes",
  //             cylinders.size(), containers.size());
}



}  // namespace scene_objects




#endif  // MOTION_PLANNER_SCENE_OBJECTS_HPP
