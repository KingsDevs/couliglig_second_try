import os
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
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

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    webots = WebotsLauncher(
        world=fixed_wbt_path,
    )

    couliglig_bot = WebotsController(
        robot_name='couliglig_bot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    base_link_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.15', '0', '0.18', '0', '0', '0', 'base_link', 'LDS-01'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    base_link_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'base_footprint'],
        parameters=[{'use_sim_time': use_sim_time}]
    )


    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(package_dir, 'config/ekf.yaml'), {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        webots,
        couliglig_bot,
        base_link_to_lidar,
        base_link_to_base_footprint,
        robot_localization_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])