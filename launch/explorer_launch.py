from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'launch', 'explorer_turtlebot', 'explorer_maze_world.launch.py'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'launch', 'nav2_bringup', 'navigation_launch.py', 'use_sim_time:=True'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py', 'use_sim_time:=True'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'],
            output='screen'
        )
    ])
