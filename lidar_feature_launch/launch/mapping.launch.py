from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


scan_edge_topic = "/scan_edge"
scan_surface_topic = "/scan_surface"
colored_scan_topic = "/colored_scan"
curvature_scan_topic = "/curvature_scan"

input_sensor_points_topic = LaunchConfiguration(
    "input_sensor_points_topic",
    default="/points_raw"
)
input_pose_topic = LaunchConfiguration(
    "input_pose_topic",
    default="/pose"
)

def generate_launch_description():
    extraction = Node(
        package="lidar_feature_extraction",
        executable="lidar_feature_extraction",
        name="lidar_feature_extraction",
        parameters=["config/lidar_feature_library.param.yaml"],
        remappings=[
            ("points_raw", input_sensor_points_topic),
            ("colored_scan", colored_scan_topic),
            ("curvature_scan", curvature_scan_topic),
            ("scan_edge", scan_edge_topic),
            ("scan_surface", scan_surface_topic),
        ]
    )

    mapping = Node(
        package="lidar_feature_mapping",
        executable="lidar_feature_mapping",
        namespace="lidar_feature_mapping",
        remappings=[
            ("scan_edge", scan_edge_topic),
            ("scan_surface", scan_surface_topic),
            ("pose", input_pose_topic),
        ]
    )

    return LaunchDescription([extraction, mapping])
