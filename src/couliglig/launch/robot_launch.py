import os
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def replace_stl_path_in_wbt(wbt_file, package_name):
    package_share_dir = get_package_share_directory(package_name)
    stl_dir = os.path.join(package_share_dir, 'worlds', 'stl_files')

    with open(wbt_file, 'r') as file:
        wbt_content = file.read()

    wbt_content = wbt_content.replace("stl_files", stl_dir)

    fixed_wbt_file = wbt_file.replace(".wbt", "_fixed.wbt")
    with open(fixed_wbt_file, 'w') as file:
        file.write(wbt_content)

    print(f"Updated Webots world saved as: {fixed_wbt_file}")
    return fixed_wbt_file

def generate_launch_description():
    package_name = 'couliglig'

    package_dir = get_package_share_directory(package_name)
    robot_description_path = os.path.join(package_dir, 'resource', 'couliglig_bot.urdf')

    original_wbt_path = os.path.join(package_dir, 'worlds', 'couliglig_bot.wbt')
    fixed_wbt_path = replace_stl_path_in_wbt(original_wbt_path, package_name)

    webots = WebotsLauncher(
        world=fixed_wbt_path,
    )

    couliglig_bot = WebotsController(
        robot_name='couliglig_bot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    odom_publisher = Node(
        package=package_name,
        executable='odom_publisher',
        output='screen'
    )

    imu_publisher = Node(
        package=package_name,
        executable='imu_publisher',
        output='screen'
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(package_dir, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time', default_value='True',
            description='Flag to enable use_sim_time'
        ),
        webots,
        couliglig_bot,
        odom_publisher,
        imu_publisher,
        robot_localization_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])