# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('social_navigation_actions')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={'model_file': example_dir + '/pddl/social_nav.pddl'}.items()
        )

    # Specify the actions
    scort_cmd = Node(
        package='social_navigation_actions',
        node_executable='escort_action_node',
        node_name='escort_action_node',
        output='screen',
        parameters=[])

    follow_cmd = Node(
        package='social_navigation_actions',
        node_executable='follow_action_node',
        node_name='follow_action_node',
        output='screen',
        parameters=[])
    approach_cmd = Node(
        package='social_navigation_actions',
        node_executable='approach_action_node',
        node_name='approach_action_node',
        output='screen',
        parameters=[])
    move_cmd = Node(
        package='social_navigation_actions',
        node_executable='move_action_node',
        node_name='move_action_node',
        output='screen',
        parameters=[])
    
    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(plansys2_cmd)
    ld.add_action(scort_cmd)
    ld.add_action(follow_cmd)
    ld.add_action(approach_cmd)
    ld.add_action(move_cmd)
    return ld
