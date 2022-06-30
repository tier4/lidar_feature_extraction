# Copyright 2022 Takeshi Ishita
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Takeshi Ishita nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

scan_edge_topic = '/scan_edge'
colored_scan_topic = '/colored_scan'
curvature_scan_topic = '/curvature_scan'

input_sensor_points_topic = LaunchConfiguration(
    'input_sensor_points_topic',
    default='/points_raw'
)
input_pose_topic = LaunchConfiguration(
    'input_pose_topic',
    default='/pose'
)
output_path_topic = LaunchConfiguration(
    'output_path_topic',
    default='/path'
)


def generate_launch_description():
    extraction = Node(
        package='lidar_feature_extraction',
        executable='lidar_feature_extraction',
        name='lidar_feature_extraction',
        parameters=[
            'lidar_feature_launch/config/lidar_feature_extraction.param.yaml'
        ],
        remappings=[
            ('points_raw', input_sensor_points_topic),
            ('colored_scan', colored_scan_topic),
            ('curvature_scan', curvature_scan_topic),
            ('scan_edge', scan_edge_topic),
        ]
    )

    mapping = Node(
        package='lidar_feature_mapping',
        executable='lidar_feature_mapping',
        namespace='lidar_feature_mapping',
        remappings=[
            ('scan_edge', scan_edge_topic),
            ('pose', input_pose_topic),
        ]
    )

    path_generator = Node(
        package='path_generator',
        executable='path_generator',
        name='path_generator',
        remappings=[
            ('pose', input_pose_topic),
            ('path', output_path_topic),
        ]
    )

    return LaunchDescription([
        extraction,
        mapping,
        path_generator,
    ])
