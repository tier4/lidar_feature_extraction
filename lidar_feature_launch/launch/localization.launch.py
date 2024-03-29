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
converted_points_topic = '/points_converted'
edge_map_path = 'maps/edge.pcd'
surface_map_path = 'maps/surface.pcd'

ekf_initial_pose_topic = LaunchConfiguration(
    'input_initial_pose_topic',
    default='/initialpose3d'
)
input_twist_topic = LaunchConfiguration(
    'input_twist_topic',
    default='/twist'
)
input_sensor_points_topic = LaunchConfiguration(
    'input_sensor_points_topic',
    default='/points_raw'
)
lidar_feature_pose_topic = LaunchConfiguration(
    'lidar_feature_pose_topic',
    default='/lidar_feature_pose'
)
edge_map_topic = LaunchConfiguration(
    'edge_map_topic',
    default='/edge_map'
)
surface_map_topic = LaunchConfiguration(
    'surface_map_topic',
    default='/surface_map'
)
estimated_path_topic = LaunchConfiguration(
    'estimated_path_topic',
    default='/estimated_path'
)
ekf_odometry = LaunchConfiguration(
    'ekf_odometry',
    default='/ekf_odom'
)


def generate_launch_description():
    converter = Node(
        package='point_type_converter',
        executable='listener',
        name='point_type_converter',
        remappings=[
            ('points_raw', input_sensor_points_topic),
            ('points_converted', converted_points_topic),
        ]
    )

    extraction = Node(
        package='lidar_feature_extraction',
        executable='lidar_feature_extraction',
        name='lidar_feature_extraction',
        parameters=[
            'lidar_feature_launch/config/lidar_feature_extraction.param.yaml'
        ],
        remappings=[
            ('points_raw', converted_points_topic),
            ('colored_scan', colored_scan_topic),
            ('curvature_scan', curvature_scan_topic),
            ('scan_edge', scan_edge_topic),
            ('scan_surface', scan_surface_topic),
        ],
    )

    localization = Node(
        package='lidar_feature_localization',
        executable='lidar_feature_localization',
        name='lidar_feature_localization',
        remappings=[
            ('scan_edge', scan_edge_topic),
            ('optimization_start_odom', ekf_odometry),
            ('estimated_pose', lidar_feature_pose_topic),
        ],
    )

    edge_map_loader = Node(
        package='lidar_feature_map_loader',
        executable='lidar_feature_map_loader',
        name='edge_map_loader',
        parameters=[
            {'pcd_filename': edge_map_path}
        ],
        remappings=[
            ('map_topic', edge_map_topic)
        ]
    )

    surface_map_loader = Node(
        package='lidar_feature_map_loader',
        executable='lidar_feature_map_loader',
        name='surface_map_loader',
        parameters=[
            {'pcd_filename': surface_map_path}
        ],
        remappings=[
            ('map_topic', surface_map_topic)
        ]
    )

    ekf_localizer = Node(
        package='ekf_localizer',
        executable='ekf_localizer',
        name='ekf_localizer',
        remappings=[
            ('initialpose', ekf_initial_pose_topic),
            ('ekf_odom', ekf_odometry),
            ('in_pose_with_covariance', lidar_feature_pose_topic),
            ('in_twist_with_covariance', input_twist_topic),
        ],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        converter,
        extraction,
        localization,
        edge_map_loader,
        surface_map_loader,
        ekf_localizer,
    ])
