LiDAR feature-based localization
================================

## Functionalities

This package has two functionalities:

* Map building module: constructs a map from undistorted LiDAR scans and their poses
* Localization module: localizes LiDAR on a map

## Benefits

* LOAM-like feature based algorithm enables localization in challenging environments such as tunnels, rice fields, etc.
* Localization on a pre-built map realizes stable and robust localization in dynamic environments

## Usage

### Building

```bash
colcon build
```

### Execution


#### Map building

```bash
source install/setup.bash
ros2 launch lidar_feature_launch mapping.launch.py
```

Open another terminal and play rosbag

```bash
source install/setup.bash
ros2 play lidar-feature-dataset.bag
```

#### Map visualization

```bash
sudo apt install pcl-tools
pcl_viewer maps/edge.pcd maps/surface.pcd
```

#### Localization

```bash
source install/setup.bash
ros2 launch lidar_feature_launch localization.launch.py
```

Open another terminal and play rosbag

```bash
source install/setup.bash
ros2 play lidar-feature-dataset.bag
```
