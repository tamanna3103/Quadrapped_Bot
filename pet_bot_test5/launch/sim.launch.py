import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_name = 'pet_bot_test5'
    xacro_file_name = 'pet_bot_test5.urdf.xacro'

    pkg_share = get_package_share_directory(package_name)
    world_file = os.path.join(pkg_share, 'worlds', 'debug_world.sdf')
    controllers_yaml = os.path.join(pkg_share, 'config', 'pet_bot_controllers.yaml')

    info = LogInfo(msg='[sim_12_joints.launch] Launching Ignition + Whole Body Controller')

    # Launch Ignition
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '4', world_file],
        output='screen'
    )

    # Publish robot_description via xacro
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(
                Command([
                    'xacro ',
                    PathJoinSubstitution([pkg_share, 'urdf', xacro_file_name])
                ]),
                value_type=str
            )
        }]
    )

    # Spawn the robot after the world is up
    spawn_entity = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_ign_gazebo', 'create',
                    '-world', 'default',
                    '-name', 'pet_bot_test5',
                    '-topic', 'robot_description',
                    '-x', '0', '-y', '0', '-z', '0.3' 
                ],
                output='screen'
            )
        ]
    )

    # Spawner for joint_state_broadcaster
    load_joint_state_broadcaster = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen"
            )
        ]
    )

    # --- CHANGED: Spawner for WHOLE BODY controller ---
    load_forward_position_controller = TimerAction(
        period=16.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                # This matches the new YAML file name
                arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
                output="screen"
            )
        ]
    )

    # Bridge Gazebo pose to ROS
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/model/pet_bot_test5/pose@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V"
        ],
        output="screen"
    )
    pet_bot_test5_gazebo = get_package_share_directory("pet_bot_test5")
    pet_bot_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='rover_bridge',
        parameters=[{
            'config_file': os.path.join(pet_bot_test5_gazebo, 'config', 'bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'use_sim_time': True
        }],
        output='screen'
    )

    # camera_bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     arguments=[
    #         "/depth_camera@sensor_msgs/msg/Image@ignition.msgs.Image"
    #     ],
    #     output="screen"
    # )

    # point_cloud_bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     arguments=[
    #         "/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked"
    #     ],
    #     output="screen"
    # )

    camera_bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="rover_gz_bridge",
                # namespace="rover",  # Add namespace for bridge
                arguments=[
                    # "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                    "/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
                    "/camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image",
                    "/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
                    "/camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked",
                ],
                remappings=[
                    ("/camera/image", "/camera/image_raw"),
                    ("/camera/depth_image", "/camera/depth/image_raw"),
                    ("/camera/camera_info", "/camera/camera_info"),
                    ("/camera/points", "/camera/depth/points"),
                ],
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ]
    )


    camera_info_relay = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='topic_tools',
                executable='relay',
                name='camera_info_relay',
                # namespace='rover',  # Add namespace for relay
                arguments=['/camera/camera_info', '/camera/depth/camera_info'],
                output='screen',
                parameters=[{"use_sim_time": True}],
            )
        ]
    )


    return LaunchDescription([
        info,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        load_joint_state_broadcaster,
        load_forward_position_controller,
        bridge,
        camera_bridge,
        # point_cloud_bridge,
        camera_info_relay,
        pet_bot_bridge,
    ])