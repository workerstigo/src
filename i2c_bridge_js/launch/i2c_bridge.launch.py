from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Use absolute path to the JS file
    js_file = os.path.join(
        '/home/user/ros2_ws/src/i2c_bridge_js',
        'i2c_bridge.js'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['node', js_file],
            output='screen'
        )
    ])
