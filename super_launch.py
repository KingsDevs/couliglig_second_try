import subprocess
import os
import signal
import atexit

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

    # Example usage
    run_node('couliglig', 'keyboard_controller')
    run_node('tf2_ros', 'static_transform_publisher', ['0', '0', '0.18', '0', '0', '0', 'base_link', 'LDS-01'])
    run_launch('slam_toolbox', 'online_async_launch.py', {'use_sim_time': 'true'})
    run_launch('nav2_bringup', 'navigation_launch.py', {'use_sim_time': 'true'})

    try:
        while True:
            pass  # Keep the script running
    except KeyboardInterrupt:
        print("Terminating processes...")
        terminate_processes()
    