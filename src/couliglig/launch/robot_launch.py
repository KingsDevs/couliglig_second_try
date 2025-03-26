import os
import launch
from launch import LaunchDescription
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

    # config_path = os.path.join(package_dir, 'config', 'couliglig_diff_drive.yaml')

    webots = WebotsLauncher(
        world=fixed_wbt_path,
    )

    couliglig_bot = WebotsController(
        robot_name='couliglig_bot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    return LaunchDescription([
        webots,
        couliglig_bot,
        # Node(
        #     package='controller_manager',
        #     executable='ros2_control_node',
        #     parameters=[{'robot_description': open(robot_description_path).read()}, config_path],
        #     output='screen'
        # ),
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['diff_drive_controller'],
        #     output='screen'
        # ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])