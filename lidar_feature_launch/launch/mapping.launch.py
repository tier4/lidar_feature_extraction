from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


scan_edge_topic = "/lidar_feature/scan_edge"
scan_surface_topic = "/lidar_feature/scan_surface"
colored_scan_topic = "/lidar_feature/colored_scan"
curvature_scan_topic = "/lidar_feature/curvature_scan"


def generate_launch_description():
    extraction = Node(
        package="lidar_feature_extraction",
        executable="lidar_feature_extraction",
        name="lidar_feature_extraction",
        parameters=["config/lidar_feature_library.param.yaml"],
        remappings=[
            ("points_raw", LaunchConfiguration("input_sensor_points_topic")),
            ("colored_scan", colored_scan_topic),
            ("curvature_scan", curvature_scan_topic),
            ("scan_edge", scan_edge_topic),
            ("scan_surface", scan_surface_topic),
        ]
    )

    mapping = Node(
        package="lidar_feature_mapping",
        executable="lidar_feature_mapping",
        name="lidar_feature_mapping",
        remappings=[
            ("scan_edge", scan_edge_topic),
            ("scan_surface", scan_surface_topic),
            ("pose", LaunchConfiguration("input_pose_topic")),
        ]
    )

    return LaunchDescription([extraction, mapping])
