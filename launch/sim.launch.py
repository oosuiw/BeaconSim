"""비콘 시뮬레이터 실행: ros2 launch launch/sim.launch.py [rviz:=true] [camera:=true]"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))
sys.path.insert(0, ROOT)
from beacon_sim.common import build_urdf, load_yaml  # noqa: E402

PY = os.path.join(ROOT, '.venv', 'bin', 'python')


def generate_launch_description():
    urdf = build_urdf(load_yaml('config/robot.yaml'))
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('camera', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': urdf, 'use_sim_time': True}]),
        ExecuteProcess(cmd=[PY, '-u', '-m', 'beacon_sim.sim_core', '--markers', '--no-static-tf', '--ros-args', '-p', 'use_sim_time:=false'],
                       cwd=ROOT, output='screen', name='sim_core'),
        ExecuteProcess(cmd=[PY, '-u', '-m', 'beacon_sim.camera_node', '--ros-args', '-p', 'use_sim_time:=true'],
                       cwd=ROOT, output='screen', name='camera', condition=IfCondition(LaunchConfiguration('camera'))),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', os.path.join(ROOT, 'rviz', 'beacon_sim.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
