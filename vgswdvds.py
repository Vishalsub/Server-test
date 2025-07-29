# #!/usr/bin/env python3

# import os
# from pathlib import Path
# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
# from launch.substitutions import EnvironmentVariable
# from launch_ros.actions import Node


# def generate_launch_description():
#     lerobot_description = get_package_share_directory('lerobot_description')
#     conveyor_model_path = os.path.join(lerobot_description, 'models', 'so101_conveyor', 'model.sdf')
#     conveyor_simple_conveyor_box = os.path.join(lerobot_description, 'models', 'simple_conveyor_box', 'model.sdf')


#     # Set GZ_SIM_RESOURCE_PATH for Gazebo to find textures and meshes
#     gz_resource_path = SetEnvironmentVariable(
#         name='GZ_SIM_RESOURCE_PATH',
#         value=str(Path(lerobot_description).resolve())
#     )

#     # Debug: Print resource path
#     print_env = ExecuteProcess(
#         cmd=['printenv', 'GZ_SIM_RESOURCE_PATH'],
#         output='screen'
#     )

#     # Launch Gazebo with empty world
#     gazebo = ExecuteProcess(
#         cmd=['gz', 'sim', '-r', '-v', '4', 'empty.sdf'],
#         output='screen'
#     )

#     # Spawn only the conveyor model
#     gz_spawn_conveyor = Node(
#         package='ros_gz_sim',
#         executable='create',
#         output='screen',
#         arguments=[
#             '-file', conveyor_model_path,
#             '-name', 'so101_conveyor',
#             '-x', '0', '-y', '0', '-z', '0'
#         ]
#     )

#     gz_spawn_conveyor_box = Node(
#         package='ros_gz_sim',
#         executable='create',
#         output='screen',
#         arguments=[
#             '-file', conveyor_simple_conveyor_box,
#             '-name', 'simple_conveyor_box',
#             '-x', '0', '-y', '0', '-z', '0'
#         ]
#     )

#     gz_ros2_bridge = Node(
#         package="ros_gz_bridge",
#         executable="parameter_bridge",
#         arguments=[
#             "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
#             "/conveyor/cmd_vel@std_msgs/msg/Float64@gz.msgs.Double"
#         ],
#         output="screen"
#     )

#     # Delay spawn to make sure Gazebo is ready
#     delayed_spawn = TimerAction(period=5.0, actions=[gz_spawn_conveyor])
#     delayed_spawn_box = TimerAction(period=7.0, actions=[gz_spawn_conveyor_box])

#     return LaunchDescription([
#         gz_resource_path,
#         print_env,
#         gazebo,
#         delayed_spawn,
#         delayed_spawn_box,
#         gz_ros2_bridge
#     ])

# ------------------------------------------------------------------------------------------------

# import os
# from pathlib import Path
# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
# from launch.substitutions import Command, LaunchConfiguration
# from launch.launch_description_sources import PythonLaunchDescriptionSource

# from launch_ros.actions import Node
# from launch_ros.parameter_descriptions import ParameterValue


# def generate_launch_description():
#     lerobot_description = get_package_share_directory("lerobot_description")

#     model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
#                                         lerobot_description, "urdf", "so101.urdf.xacro"
#                                         ),
#                                       description="Absolute path to robot urdf file"
#     )

#     gazebo_resource_path = SetEnvironmentVariable(
#         name="GZ_SIM_RESOURCE_PATH",
#         value=[
#             str(Path(lerobot_description).parent.resolve())
#             ]
#         )
    
#     robot_description = ParameterValue(Command([
#             "xacro ",
#             LaunchConfiguration("model"),
#         ]),
#         value_type=str
#     )

#     robot_state_publisher_node = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         parameters=[{"robot_description": robot_description,
#                      "use_sim_time": True}]
#     )

#     gazebo = IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource([os.path.join(
#                     get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
#                 launch_arguments=[
#                     ("gz_args", [" -v 4 -r empty.sdf "]
#                     )
#                 ]
#              )

#     gz_spawn_entity = Node(
#         package="ros_gz_sim",
#         executable="create",
#         output="screen",
#         arguments=["-topic", "robot_description",
#                    "-name", "so101"],
#     )

#     gz_ros2_bridge = Node(
#         package="ros_gz_bridge",
#         executable="parameter_bridge",
#         arguments=[
#             "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
#         ]
#     )

#     return LaunchDescription([
#         model_arg,
#         gazebo_resource_path,
#         robot_state_publisher_node,
#         gazebo,
#         gz_spawn_entity,
#         gz_ros2_bridge
#     ])

#!/usr/bin/env python3

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction
)
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # --- Package Path ---
    lerobot_description = get_package_share_directory("lerobot_description")

    # --- Robot URDF Argument ---
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(lerobot_description, "urdf", "so101.urdf.xacro"),
        description="Absolute path to robot URDF/XACRO"
    )

    # --- Conveyor Models ---
    conveyor_model_path = os.path.join(
        lerobot_description, 'models', 'so101_conveyor', 'model.sdf'
    )
    conveyor_box_model_path = os.path.join(
        lerobot_description, 'models', 'simple_conveyor_box', 'model.sdf'
    )

    realsense_path = os.path.join(
        lerobot_description, 'models', 'realsense_d435', 'model.sdf'
    )

    # --- Gazebo Resource Path ---
# Correct Gazebo Resource Path
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(lerobot_description).parent.resolve())
        ]
    )

    # --- Robot Description ---
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    # --- Robot State Publisher ---
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }]
    )

    # --- Include Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )]
        ),
        launch_arguments={
            "gz_args": "-v 4 -r empty.sdf"
        }.items()
    )

    # --- Spawn Robot ---
    gz_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "so101",
            "-x", "0.0", "-y", "0.0", "-z", "0.2"
        ],
    )

    # --- Spawn Conveyor ---
    gz_spawn_conveyor = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', conveyor_model_path,
            '-name', 'so101_conveyor',
            '-x', '0.05', '-y', '-0.23', '-z', '0.0'
        ]
    )

    # --- Spawn Conveyor Box ---
    gz_spawn_conveyor_box = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', conveyor_box_model_path,
            '-name', 'simple_conveyor_box',
            '-x', '0.04', '-y', '-0.24', '-z', '0.27'
        ]
    )

    gz_spawn_realsense = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file',realsense_path ,
            '-name', 'realsense_d435',
            '-x', '0.0', '-y', '0.0', '-z', '0.0'
        ]
    )

    

    # --- ROS-GZ Bridge ---
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/conveyor/cmd_vel@std_msgs/msg/Float64@gz.msgs.Double"
        ],
        output="screen"
    )

    # --- Delayed Spawns to ensure Gazebo is ready ---
    delayed_spawn_conveyor = TimerAction(period=5.0, actions=[gz_spawn_conveyor])
    delayed_spawn_box = TimerAction(period=7.0, actions=[gz_spawn_conveyor_box])
    delayed_spawn_robot = TimerAction(period=3.0, actions=[gz_spawn_robot])
    delayed_spawn_real = TimerAction(period=8.0, actions=[gz_spawn_realsense])

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_ros2_bridge,
        delayed_spawn_robot,
        delayed_spawn_conveyor,
        delayed_spawn_box,
        delayed_spawn_real
    ])
