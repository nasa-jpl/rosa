import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # UR type
    ur_type = "ur5e"
    
    # Robot description with Hand-E gripper attached
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur5e_hande_description"), "urdf", "ur5e_hande.urdf.xacro"]),
            " ",
            "ur_type:=", ur_type,
            " ",
            "name:=ur",
            " ",
            "use_fake_hardware:=true",
            " ",
            "fake_sensor_commands:=false",
            " ",
            "initial_positions_file:=",
            PathJoinSubstitution([FindPackageShare("ur_description"), "config", "initial_positions.yaml"]),
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
    
    # Robot description semantic (SRDF) - custom SRDF with gripper collision exemptions  
    # Read SRDF file content
    srdf_file_path = os.path.join(
        get_package_share_directory("motion_planner"),
        "srdf",
        "ur5e_hande.srdf"
    )
    
    try:
        with open(srdf_file_path, 'r') as f:
            robot_description_semantic_content = f.read()
    except FileNotFoundError:
        robot_description_semantic_content = ""
    
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    # Kinematics yaml - use improved configuration for better accuracy
    kinematics_yaml = os.path.join(
        get_package_share_directory("motion_planner"),
        "config",
        "kinematics_improved.yaml",
    )

    # Planning pipeline configuration - load from OMPL planning yaml file
    ompl_planning_yaml = os.path.join(
        get_package_share_directory("motion_planner"),
        "config",
        "ompl_planning.yaml",
    )
    
    # MoveIt controllers configuration
    controllers_yaml = os.path.join(
        get_package_share_directory("motion_planner"),
        "config",
        "moveit_controllers.yaml",
    )

    # Load ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Planning scene monitor parameters
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "planning_scene_monitor_options": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/joint_states",
            "attached_collision_object_topic": "/attached_collision_object",
            "publish_planning_scene_topic": "/planning_scene",
            "monitored_planning_scene_topic": "/monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
        },
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",          # <-- add
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_yaml,
            controllers_yaml,
            move_group_capabilities,
            planning_scene_monitor_parameters,
        ],
    )

    # RViz - using UR5e-specific config file with MTC panels
    rviz_config_file = os.path.join(
        get_package_share_directory("motion_planner"),
        "config",
        "ur5e_rviz.yaml"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("motion_planner"),
        "config",
        "ros2_controllers.yaml",
    )
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "scaled_joint_trajectory_controller",
        "gripper_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    # MTC demo node with gripper
    mtc_demo_node = Node(
        package="motion_planner",
        executable="mtc_node",
        name="mtc_node",            # <-- add
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_yaml,
        ],
    )

    return LaunchDescription(
        [
            static_tf,
            robot_state_publisher,
            ros2_control_node,
            run_move_group_node,
            rviz_node,
            mtc_demo_node,
        ]
        + load_controllers
    )
