import subprocess
import atexit
import time
import os
from ament_index_python.packages import get_package_share_directory

processes = []

def run_node(package, executable, arguments=None, output='screen'):
    cmd = ['ros2', 'run', package, executable]
    if arguments:
        cmd.extend(arguments)
    if output == 'screen':
        p = subprocess.Popen(cmd)
    else:
        with open(output, 'w') as f:
            p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT)
    processes.append(p)


def run_launch(package, launch_file, arguments=None):
    cmd = ['ros2', 'launch', package, launch_file]
    if arguments:
        for key, value in arguments.items():
            cmd.append(f'{key}:={value}')
    p = subprocess.Popen(cmd)
    processes.append(p)

def terminate_processes():
    for p in processes:
        p.terminate()
        p.wait()


if __name__ == '__main__':
    atexit.register(terminate_processes)

    use_sim_time = 'true'
    slam_params_file = os.path.join('couliglig', 'config', 'mappers_online_params.yaml')
    nav2_params_file = os.path.join('couliglig', 'config', 'nav2_params2.yaml')

    run_launch('couliglig', 'robot_launch.py', {'use_sim_time': use_sim_time})
    time.sleep(5)  # Wait for the Webots simulation to start
    run_launch('slam_toolbox', 'online_async_launch.py', {'use_sim_time': use_sim_time, 'slam_params_file': slam_params_file})
    time.sleep(5)  # Wait for the SLAM Toolbox to start
    run_launch('nav2_bringup', 'navigation_launch.py', {'use_sim_time': use_sim_time, 'params_file': nav2_params_file})
    run_node('couliglig', 'keyboard_controller', output='screen')

    try:
        while True:
            pass  # Keep the script running
    except KeyboardInterrupt:
        print("Terminating processes...")
        terminate_processes()
    