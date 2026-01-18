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
    
    # Robot description with fake hardware for simulation
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
            " ",
            "ur_type:=", ur_type,
            " ",
            "name:=ur",
            " ",
            "prefix:=''",
            " ",
            "safety_limits:=true",
            " ",
            "safety_pos_margin:=0.15",
            " ",
            "safety_k_position:=20",
            " ",
            "use_fake_hardware:=true",
            " ",
            "fake_sensor_commands:=false",
            " ",
            #"initial_positions:='{shoulder_pan_joint: 3.14159, shoulder_lift_joint: -1.5708, elbow_joint: -1.5708, wrist_1_joint: -1.7453, wrist_2_joint: 1.5708, wrist_3_joint: -0.8727}'",
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
    
    # Robot description semantic (SRDF)
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
            " ",
            "name:=ur",
            " ",
            "prefix:=''",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)}

    # Kinematics yaml - use improved configuration for better accuracy
    # Option 1: Use custom improved kinematics (recommended)
    kinematics_yaml = os.path.join(
        get_package_share_directory("motion_planner"),
        "config",
        "kinematics_improved.yaml",
    )
    # Option 2: Use default UR kinematics (comment out above and uncomment below)
    # kinematics_yaml = os.path.join(
    #     get_package_share_directory("ur_moveit_config"),
    #     "config",
    #     "kinematics.yaml",
    # )

    # Planning pipeline configuration - inline OMPL config with time parameterization
    ompl_planning_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        },
        "ur_manipulator": {
            "default_planner_config": "RRTConnectkConfigDefault",
            "projection_evaluator": "joints(shoulder_pan_joint,shoulder_lift_joint)",
        },
        "planner_configs": {
            "RRTConnectkConfigDefault": {
                "type": "geometric::RRTConnect",
                "range": 0.1,  # Increased range for faster tree expansion (0.0 = auto, 0.1 = faster)
            }
        }
    }
    
    # MoveIt controllers configuration - using our own fixed version
    controllers_yaml = os.path.join(
        get_package_share_directory("motion_planner"),
        "config",
        "moveit_controllers.yaml",
    )

    # # Trajectory execution parameters - allow larger deviation for fake hardware
    # trajectory_execution = {
    #     "moveit_manage_controllers": True,
    #     "trajectory_execution.allowed_execution_duration_scaling": 1.2,
    #     "trajectory_execution.allowed_goal_duration_margin": 0.5,
    #     "trajectory_execution.allowed_start_tolerance": 0.1,  # Increased tolerance for fake hardware
    #     "trajectory_execution.execution_duration_monitoring": False,  # Disable strict timing checks for simulation
    # }

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Planning scene monitor parameters - ensure current state is used
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
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_config,
            #trajectory_execution,
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

    # ros2_control using FakeSystem as hardware - load from YAML file  
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
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    # #MTC demo node (optional - uncomment to run pick-and-place demo)
    mtc_demo_node = Node(
        package="motion_planner",
        executable="mtc_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_config,
        ],
    )

    return LaunchDescription(
        [
            static_tf,
            robot_state_publisher,
            ros2_control_node,
            run_move_group_node,
            rviz_node,
            mtc_demo_node,  # Uncomment to run pick-and-place demo
        ]
        + load_controllers
    )