#!/usr/bin/env python3

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParametersFromFile


def _load_contract_defaults(contract_path):
    with open(contract_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    return data.get('/**', {}).get('ros__parameters', {})


def generate_launch_description():
    nartech_share = get_package_share_directory('nartech_ros')
    nav2_share = get_package_share_directory('nav2_bringup')
    contract_path = os.path.join(nartech_share, 'config', 'g1_ros_contract.yaml')
    contract = _load_contract_defaults(contract_path)

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_scan_pipeline = LaunchConfiguration('start_scan_pipeline')
    start_cmd_vel_adapter = LaunchConfiguration('start_cmd_vel_adapter')
    start_nav2 = LaunchConfiguration('start_nav2')
    slam = LaunchConfiguration('slam')
    start_nartech_node = LaunchConfiguration('start_nartech_node')
    enable_arm_controller = LaunchConfiguration('enable_arm_controller')
    publish_base_footprint_tf = LaunchConfiguration('publish_base_footprint_tf')
    contract_file = LaunchConfiguration('contract_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    slam_scan_mode = LaunchConfiguration('slam_scan_mode')

    livox_points_topic = LaunchConfiguration('livox_points_topic')
    d435_color_topic = LaunchConfiguration('d435_color_topic')
    d435_depth_topic = LaunchConfiguration('d435_depth_topic')
    d435_camera_info_topic = LaunchConfiguration('d435_camera_info_topic')
    scan_primary_topic = LaunchConfiguration('scan_primary_topic')
    scan_secondary_topic = LaunchConfiguration('scan_secondary_topic')
    scan_output_topic = LaunchConfiguration('scan_output_topic')
    scan_output_frame = LaunchConfiguration('scan_output_frame')
    restamp_scan = LaunchConfiguration('restamp_scan')
    scan_publish_rate_hz = LaunchConfiguration('scan_publish_rate_hz')
    max_input_msg_age_sec = LaunchConfiguration('max_input_msg_age_sec')
    max_future_offset_sec = LaunchConfiguration('max_future_offset_sec')
    pointcloud_target_frame = LaunchConfiguration('pointcloud_target_frame')
    pointcloud_transform_tolerance_sec = LaunchConfiguration('pointcloud_transform_tolerance_sec')
    base_frame = LaunchConfiguration('base_frame')
    camera_frame = LaunchConfiguration('camera_frame')

    declare_contract_file = DeclareLaunchArgument(
        'contract_file',
        default_value=contract_path,
        description='ROS2 parameter file containing the G1 topic/frame contract.'
    )
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='False')
    declare_start_scan_pipeline = DeclareLaunchArgument('start_scan_pipeline', default_value='True')
    declare_start_cmd_vel_adapter = DeclareLaunchArgument('start_cmd_vel_adapter', default_value='True')
    declare_start_nav2 = DeclareLaunchArgument('start_nav2', default_value='True')
    declare_slam = DeclareLaunchArgument('slam', default_value='True')
    declare_start_nartech_node = DeclareLaunchArgument('start_nartech_node', default_value='True')
    declare_enable_arm_controller = DeclareLaunchArgument('enable_arm_controller', default_value='False')
    declare_publish_base_footprint_tf = DeclareLaunchArgument('publish_base_footprint_tf', default_value='True')
    declare_autostart = DeclareLaunchArgument('autostart', default_value='True')
    declare_use_composition = DeclareLaunchArgument('use_composition', default_value='True')
    declare_use_respawn = DeclareLaunchArgument('use_respawn', default_value='False')
    declare_namespace = DeclareLaunchArgument('namespace', default_value='')
    declare_use_namespace = DeclareLaunchArgument('use_namespace', default_value='False')
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav2_share, 'maps', 'turtlebot3_world.yaml'),
    )
    declare_nav2_params_file = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(nav2_share, 'params', 'nav2_params.yaml'),
    )
    declare_slam_scan_mode = DeclareLaunchArgument(
        'slam_scan_mode',
        default_value='lidar_only',
        description="SLAM scan source mode: 'lidar_only' applies local slam_toolbox overrides "
                    "(including scan_topic=/scan), 'mux' uses Nav2 defaults."
    )

    declare_livox_points_topic = DeclareLaunchArgument(
        'livox_points_topic',
        default_value=str(contract.get('livox_points_topic', '/livox/points')),
    )
    declare_d435_depth_topic = DeclareLaunchArgument(
        'd435_depth_topic',
        default_value=str(contract.get('d435_depth_topic', '/intel/D435i/depth')),
    )
    declare_d435_color_topic = DeclareLaunchArgument(
        'd435_color_topic',
        default_value=str(contract.get('d435_color_topic', '/intel/D435i/color')),
    )
    declare_d435_camera_info_topic = DeclareLaunchArgument(
        'd435_camera_info_topic',
        default_value=str(contract.get('d435_camera_info_topic', '/intel/D435i/camera_info')),
    )
    declare_scan_primary_topic = DeclareLaunchArgument(
        'scan_primary_topic',
        default_value=str(contract.get('scan_primary_topic', '/scan/livox')),
    )
    declare_scan_secondary_topic = DeclareLaunchArgument(
        'scan_secondary_topic',
        default_value=str(contract.get('scan_secondary_topic', '/scan/depth')),
    )
    declare_scan_output_topic = DeclareLaunchArgument(
        'scan_output_topic',
        default_value=str(contract.get('scan_output_topic', '/scan')),
    )
    declare_scan_output_frame = DeclareLaunchArgument(
        'scan_output_frame',
        default_value=str(contract.get('scan_output_frame', 'base_link')),
    )
    declare_restamp_scan = DeclareLaunchArgument(
        'restamp_scan',
        default_value=str(contract.get('restamp_scan', True)).lower(),
    )
    declare_scan_publish_rate_hz = DeclareLaunchArgument(
        'scan_publish_rate_hz',
        default_value=str(contract.get('scan_publish_rate_hz', 10.0)),
    )
    declare_max_input_msg_age_sec = DeclareLaunchArgument(
        'max_input_msg_age_sec',
        default_value=str(contract.get('max_input_msg_age_sec', 0.8)),
    )
    declare_max_future_offset_sec = DeclareLaunchArgument(
        'max_future_offset_sec',
        default_value=str(contract.get('max_future_offset_sec', 0.25)),
    )
    declare_pointcloud_target_frame = DeclareLaunchArgument(
        'pointcloud_target_frame',
        default_value=str(contract.get('pointcloud_target_frame', '')),
    )
    declare_pointcloud_transform_tolerance_sec = DeclareLaunchArgument(
        'pointcloud_transform_tolerance_sec',
        default_value=str(contract.get('pointcloud_transform_tolerance_sec', 0.5)),
    )
    declare_base_frame = DeclareLaunchArgument(
        'base_frame',
        default_value=str(contract.get('base_frame', 'base_link')),
    )
    declare_camera_frame = DeclareLaunchArgument(
        'camera_frame',
        default_value=str(contract.get('camera_frame', 'd435i_depth_cam_optical')),
    )

    pointcloud_to_scan = Node(
        condition=IfCondition(start_scan_pipeline),
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[
            contract_file,
            {
                'use_sim_time': use_sim_time,
                'target_frame': pointcloud_target_frame,
                'transform_tolerance': pointcloud_transform_tolerance_sec,
                'min_height': 0.2,
                'max_height': 5.0,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,
                'scan_time': 1.0,
                'range_min': 0.3,
                'range_max': 30.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            },
        ],
        remappings=[
            ('cloud_in', livox_points_topic),
            ('scan', scan_primary_topic),
        ],
    )

    depth_to_scan = Node(
        condition=IfCondition(start_scan_pipeline),
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[
            contract_file,
            {
                'use_sim_time': use_sim_time,
                'output_frame': base_frame,
                'scan_time': 1.0,
                'range_min': 0.6,
                'range_max': 1.1,
                'scan_height': 320,
            },
        ],
        remappings=[
            ('depth', d435_depth_topic),
            ('depth_camera_info', d435_camera_info_topic),
            ('scan', scan_secondary_topic),
        ],
    )

    scan_mux = Node(
        condition=IfCondition(start_scan_pipeline),
        package='nartech_ros',
        executable='scan_mux_selector',
        name='scan_mux_selector',
        output='screen',
        parameters=[
            contract_file,
            {
                'use_sim_time': use_sim_time,
                'scan_primary_topic': scan_primary_topic,
                'scan_secondary_topic': scan_secondary_topic,
                'scan_output_topic': scan_output_topic,
                'scan_output_frame': scan_output_frame,
                'restamp_scan': restamp_scan,
                'scan_publish_rate_hz': scan_publish_rate_hz,
                'max_input_msg_age_sec': max_input_msg_age_sec,
                'max_future_offset_sec': max_future_offset_sec,
            },
        ],
    )

    cmd_vel_adapter = Node(
        condition=IfCondition(start_cmd_vel_adapter),
        package='nartech_ros',
        executable='cmd_vel_adapter',
        name='cmd_vel_adapter',
        output='screen',
        parameters=[contract_file, {'use_sim_time': use_sim_time}],
    )

    base_footprint_alias = Node(
        condition=IfCondition(
            PythonExpression(
                [publish_base_footprint_tf, " and '", base_frame, "' != 'base_footprint'"]
            )
        ),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_alias',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', base_frame, 'base_footprint'],
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
        condition=IfCondition(start_nav2),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'slam': slam,
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
        }.items(),
    )
    slam_toolbox_overrides = SetParametersFromFile(
        os.path.join(nartech_share, 'config', 'slam_toolbox_overrides.yaml')
    )
    nav2_with_slam_overrides = GroupAction(
        condition=IfCondition(
            PythonExpression(["'", slam_scan_mode, "' == 'lidar_only'"])
        ),
        actions=[
            slam_toolbox_overrides,
            nav2_bringup,
        ]
    )
    nav2_without_slam_overrides = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
        condition=IfCondition(
            PythonExpression(
                [start_nav2, " and '", slam_scan_mode, "' == 'mux'"]
            )
        ),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'slam': slam,
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
        }.items(),
    )

    nartech_node = Node(
        condition=IfCondition(start_nartech_node),
        package='nartech_ros',
        executable='nartech_main',
        name='nartech_main',
        output='screen',
        parameters=[
            contract_file,
            {
                'use_sim_time': use_sim_time,
                'enable_arm_controller': enable_arm_controller,
                'rgb_topic': d435_color_topic,
                'depth_topic': d435_depth_topic,
                'base_frame': base_frame,
                'camera_frame': camera_frame,
            },
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_contract_file)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_start_scan_pipeline)
    ld.add_action(declare_start_cmd_vel_adapter)
    ld.add_action(declare_start_nav2)
    ld.add_action(declare_slam)
    ld.add_action(declare_start_nartech_node)
    ld.add_action(declare_enable_arm_controller)
    ld.add_action(declare_publish_base_footprint_tf)
    ld.add_action(declare_autostart)
    ld.add_action(declare_use_composition)
    ld.add_action(declare_use_respawn)
    ld.add_action(declare_namespace)
    ld.add_action(declare_use_namespace)
    ld.add_action(declare_map)
    ld.add_action(declare_nav2_params_file)
    ld.add_action(declare_slam_scan_mode)
    ld.add_action(declare_livox_points_topic)
    ld.add_action(declare_d435_color_topic)
    ld.add_action(declare_d435_depth_topic)
    ld.add_action(declare_d435_camera_info_topic)
    ld.add_action(declare_scan_primary_topic)
    ld.add_action(declare_scan_secondary_topic)
    ld.add_action(declare_scan_output_topic)
    ld.add_action(declare_scan_output_frame)
    ld.add_action(declare_restamp_scan)
    ld.add_action(declare_scan_publish_rate_hz)
    ld.add_action(declare_max_input_msg_age_sec)
    ld.add_action(declare_max_future_offset_sec)
    ld.add_action(declare_pointcloud_target_frame)
    ld.add_action(declare_pointcloud_transform_tolerance_sec)
    ld.add_action(declare_base_frame)
    ld.add_action(declare_camera_frame)

    ld.add_action(pointcloud_to_scan)
    ld.add_action(depth_to_scan)
    ld.add_action(scan_mux)
    ld.add_action(cmd_vel_adapter)
    ld.add_action(base_footprint_alias)
    ld.add_action(nav2_with_slam_overrides)
    ld.add_action(nav2_without_slam_overrides)
    ld.add_action(nartech_node)
    return ld
