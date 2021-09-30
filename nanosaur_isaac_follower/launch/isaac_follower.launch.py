# Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

import yaml
import os
import launch

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# detect all 36h11 tags
cfg_36h11 = {
    'image_transport': 'raw',
    'family': '36h11',
    'size': 0.162
}


def load_config(config):
    if os.path.isfile(config):
        
        with open(config, "r") as stream:
            try:
                return yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
    return {}


def generate_launch_description():
    pkg_nanosaur_isaac = FindPackageShare(package='nanosaur_isaac_follower').find('nanosaur_isaac_follower')
    # Load nanosaur configuration and check if are included extra parameters
    conf = load_config(os.path.join(pkg_nanosaur_isaac, 'param', 'robot.yml'))
    # Load namespace
    namespace = os.getenv("HOSTNAME") if conf.get("multirobot", False) else ""
    # Follower configuration
    follower_config = os.path.join(pkg_nanosaur_isaac, 'param', 'follower.yml')
    follower_dir = LaunchConfiguration('follower_dir', default=follower_config)
    
    follower_node = Node(
        package='nanosaur_isaac_follower',
        namespace=namespace,
        executable='nanosaur_follower',
        name='nanosaur_follower',
        parameters=[follower_dir] if os.path.isfile(follower_config) else [],
        output='screen'
    )    
    
    #rectify_node = ComposableNode(
    #    package='isaac_ros_image_proc',
    #    plugin='isaac_ros::image_proc::RectifyNode',
    #    name='rectify_node',
    #)

    #rectify_container = ComposableNodeContainer(
    #    name='rectify_container',
    #    namespace='',
    #    package='rclcpp_components',
    #    executable='component_container',
    #    composable_node_descriptions=[rectify_node],
    #    output='screen'
    #)

    apriltag_exe = Node(
        package='isaac_ros_apriltag',
        executable='isaac_ros_apriltag',
        name='apriltag_exe',
        #parameters=[cfg_36h11]
    )
    
    return launch.LaunchDescription([
        #rectify_container,
        # Isaac GEMs for ROS
        # April Tag node
        apriltag_exe,
        # Nanosaur follower driver
        # Take pictures from the apriltag node and follow a tag
        follower_node
    ])
# EOF