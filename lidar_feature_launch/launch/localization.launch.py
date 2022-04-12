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
scan_surface_topic = '/scan_surface'
colored_scan_topic = '/colored_scan'
curvature_scan_topic = '/curvature_scan'

input_sensor_points_topic = LaunchConfiguration(
    'input_sensor_points_topic',
    default='/points_raw'
)
initial_pose_topic = LaunchConfiguration(
    'initial_pose_topic',
    default='/pose'
)
output_estimated_pose_topic = LaunchConfiguration(
    'output_estimated_pose_topic',
    default='/estimated_pose'
)
edge_map_topic = LaunchConfiguration(
    'edge_map_topic',
    default='/edge_map'
)
surface_map_topic = LaunchConfiguration(
    'surface_map_topic',
    default='/surface_map'
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
            ('scan_surface', scan_surface_topic),
        ]
    )

    localization = Node(
        package='lidar_feature_localization',
        executable='lidar_feature_localization',
        name='lidar_feature_localization',
        remappings=[
            ('scan_edge', scan_edge_topic),
            ('scan_surface', scan_surface_topic),
            ('initial_pose', initial_pose_topic),
            ('estimated_pose', output_estimated_pose_topic),
        ]
    )

    map_loader = Node(
        package='lidar_feature_map_loader',
        executable='lidar_feature_map_loader',
        namespace='lidar_feature_map_loader',
        remappings=[
            ('/edge_map', edge_map_topic),
            ('/surface_map', surface_map_topic),
        ]
    )

    map_tf_generator = Node(
        package='map_tf_generator',
        executable='map_tf_generator',
        name='map_tf_generator',
        parameters=[
            {
                'map_frame': 'map',
                'viewer_frame': 'viewer',
            }
        ],
        remappings=[
            ('/pointcloud_map', '/edge_map')
        ]
    )

    return LaunchDescription([extraction, localization, map_loader, map_tf_generator])
