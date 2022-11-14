LiDAR feature-based localization
================================

## Functionalities

This package has two functionalities:

* Map building module: constructs a map from undistorted LiDAR scans and their poses
* Localization module: localizes LiDAR on a map

## Benefits

* LOAM-like feature based algorithm enables localization in challenging environments such as tunnels, rice fields, etc.
* Localization on a pre-built map realizes stable and robust localization in dynamic environments


## Citation

This code makes use of [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) and [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM).

```
@inproceedings{liosam2020shan,
  title={LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping},
  author={Shan, Tixiao and Englot, Brendan and Meyers, Drew and Wang, Wei and Ratti, Carlo and Rus Daniela},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={5135-5142},
  year={2020},
  organization={IEEE}
}
```

```
@inproceedings{legoloam2018shan,
  title={LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain},
  author={Shan, Tixiao and Englot, Brendan},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={4758-4765},
  year={2018},
  organization={IEEE}
}
```
