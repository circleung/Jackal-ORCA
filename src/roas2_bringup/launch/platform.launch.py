from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    launch_arg_diagnostic_updater_params = DeclareLaunchArgument(
        'diagnostic_updater_params',
        default_value='/home/jackal/colcon_ws/src/roas2_bringup/config/diagnostic_updater.yaml',
        description='')

    diagnostic_updater_params = LaunchConfiguration('diagnostic_updater_params')

    launch_arg_diagnostic_aggregator_params = DeclareLaunchArgument(
        'diagnostic_aggregator_params',
        default_value='/home/jackal/colcon_ws/src/roas2_bringup/config/diagnostic_aggregator.yaml',
        description='')

    diagnostic_aggregator_params = LaunchConfiguration('diagnostic_aggregator_params')

    launch_arg_foxglove_bridge_parameters = DeclareLaunchArgument(
        'foxglove_bridge_parameters',
        default_value='/home/jackal/colcon_ws/src/roas2_bringup/config/foxglove_bridge.yaml',
        description='')

    foxglove_bridge_parameters = LaunchConfiguration('foxglove_bridge_parameters')

    launch_arg_imu_filter = DeclareLaunchArgument(
        'imu_filter',
        default_value='/home/jackal/colcon_ws/src/roas2_bringup/config/imu_filter.yaml',
        description='')

    imu_filter = LaunchConfiguration('imu_filter')

    # Include Packages
    pkg_roas2_bringup = FindPackageShare('roas2_bringup')
    pkg_clearpath_common = FindPackageShare('clearpath_common')
    pkg_clearpath_firmware = FindPackageShare('clearpath_firmware')
    pkg_clearpath_diagnostics = FindPackageShare('clearpath_diagnostics')

    # Declare launch files
    launch_file_platform = PathJoinSubstitution([
        pkg_roas2_bringup, 'launch', 'jackal.launch.py'])
    launch_file_proton = PathJoinSubstitution([
        pkg_clearpath_firmware, 'launch', 'proton.launch.py'])
    launch_file_diagnostics = PathJoinSubstitution([
        pkg_clearpath_diagnostics, 'launch', 'diagnostics.launch.py'])
    launch_file_foxglove_bridge = PathJoinSubstitution([
        pkg_clearpath_diagnostics, 'launch', 'foxglove_bridge.launch.py'])
    launch_file_view_robot = PathJoinSubstitution([
        pkg_roas2_bringup, 'launch', 'view_robot.launch.py'])

    # Include launch files
    launch_platform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_platform]),
        launch_arguments=
            [
                (
                    'setup_path'
                    ,
                    '/home/jackal/colcon_ws/src/roas2_bringup'
                )
                ,
                (
                    'use_sim_time'
                    ,
                    'false'
                )
                ,
                (
                    'namespace'
                    ,
                    'j100_0915'
                )
                ,
                (
                    'enable_ekf'
                    ,
                    'false'
                )
                ,
            ]
    )

    launch_proton = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_proton]),
        launch_arguments=
            [
                (
                    'namespace'
                    ,
                    'j100_0915'
                )
                ,
                (
                    'platform'
                    ,
                    'j100'
                )
                ,
            ]
    )

    launch_diagnostics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_diagnostics]),
        launch_arguments=
            [
                (
                    'namespace'
                    ,
                    'j100_0915'
                )
                ,
                (
                    'updater_parameters'
                    ,
                    diagnostic_updater_params
                )
                ,
                (
                    'aggregator_parameters'
                    ,
                    diagnostic_aggregator_params
                )
                ,
            ]
    )

    launch_foxglove_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_foxglove_bridge]),
        launch_arguments=
            [
                (
                    'namespace'
                    ,
                    'j100_0915'
                )
                ,
                (
                    'parameters'
                    ,
                    foxglove_bridge_parameters
                )
                ,
            ]
    )

    launch_view_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_view_robot]),
        launch_arguments=
            [
                (
                    'namespace'
                    ,
                    'j100_0915'
                )
                ,
                (
                    'use_sim_time'
                    ,
                    'false'
                )
                ,
            ]
    )

    # Nodes
    node_battery_state_control = Node(
        name='battery_state_control',
        executable='battery_state_control',
        package='clearpath_hardware_interfaces',
        namespace='j100_0915',
        output='screen',
        arguments=
            [
                '-s'
                ,
                '/home/jackal/colcon_ws/src/roas2_bringup'
                ,
            ]
        ,
    )

    node_battery_state_estimator = Node(
        name='battery_state_estimator',
        executable='battery_state_estimator',
        package='clearpath_hardware_interfaces',
        namespace='j100_0915',
        output='screen',
        arguments=
            [
                '-s'
                ,
                '/home/jackal/colcon_ws/src/roas2_bringup'
                ,
            ]
        ,
    )

    # node_wireless_watcher = Node(
    #     name='wireless_watcher',
    #     executable='wireless_watcher',
    #     package='wireless_watcher',
    #     namespace='j100_0915',
    #     output='screen',
    #     remappings=
    #         [
    #             (
    #                 '/diagnostics'
    #                 ,
    #                 'diagnostics'
    #             )
    #             ,
    #         ]
    #     ,
    #     parameters=
    #         [
    #             {
    #                 'hz': 1.0
    #                 ,
    #                 'dev': ''
    #                 ,
    #                 'connected_topic': 'platform/wifi_connected'
    #                 ,
    #                 'connection_topic': 'platform/wifi_status'
    #                 ,
    #             }
    #             ,
    #         ]
    #     ,
    # )

    node_imu_filter_madgwick = Node(
        name='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        package='imu_filter_madgwick',
        namespace='j100_0915',
        output='screen',
        remappings=
            [
                (
                    'imu/data_raw'
                    ,
                    'sensors/imu_0/data_raw'
                )
                ,
                (
                    'imu/mag'
                    ,
                    'sensors/imu_0/magnetic_field'
                )
                ,
                (
                    'imu/data'
                    ,
                    'sensors/imu_0/data'
                )
                ,
                (
                    '/tf'
                    ,
                    'tf'
                )
                ,
            ]
        ,
        parameters=
            [
                imu_filter
                ,
            ]
        ,
    )

    node_nmea_topic_driver = Node(
        name='nmea_topic_driver',
        executable='nmea_topic_driver',
        package='nmea_navsat_driver',
        namespace='j100_0915',
        output='screen',
        remappings=
            [
                (
                    'nmea_sentence'
                    ,
                    'sensors/gps_0/nmea_sentence'
                )
                ,
                (
                    'fix'
                    ,
                    'sensors/gps_0/fix'
                )
                ,
                (
                    'heading'
                    ,
                    'sensors/gps_0/heading'
                )
                ,
                (
                    'time_reference'
                    ,
                    'sensors/gps_0/time_reference'
                )
                ,
                (
                    'vel'
                    ,
                    'sensors/gps_0/vel'
                )
                ,
            ]
        ,
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_arg_diagnostic_updater_params)
    ld.add_action(launch_arg_diagnostic_aggregator_params)
    ld.add_action(launch_arg_foxglove_bridge_parameters)
    ld.add_action(launch_arg_imu_filter)
    ld.add_action(launch_platform)
    ld.add_action(launch_proton)
    ld.add_action(launch_diagnostics)
    ld.add_action(launch_foxglove_bridge)
    # ld.add_action(launch_view_robot)  # Disabled: launch rviz2 manually
    ld.add_action(node_battery_state_control)
    ld.add_action(node_battery_state_estimator)
    # ld.add_action(node_wireless_watcher)  # Disabled: wireless_watcher uses deprecated wireless extensions API
    ld.add_action(node_imu_filter_madgwick)
    ld.add_action(node_nmea_topic_driver)
    return ld
