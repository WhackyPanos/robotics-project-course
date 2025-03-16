# ICP Package
This is a "secundary" package which will be used by the localization package. This package is responsible for taking the LiDAR scans, convert them into PointCloud and apply ICP to get the corrected PointCloud msg and the respective transformation. This transformation will be handled by the localization package.

#### **Topics**
| Topic | Type | Published in | Subscribed in| Description |
| --- | ---| --- | --- | --- |
| /transformed_point_cloud | PointCloud2 | icp_node.cpp -> icp_cpp pkg |  - | New point cloud, may be useful later on 
| /icp_transform | TransformStamped | icp_node.cpp -> icp_cpp pkg | - | Transformation to be handled by localizatin package
| /scan | LaserScan | LIDAR |  icp_node.cpp -> icp_cpp pkg| -
---

This is launched when the localization package nodes are launched, which can be done with
```
ros2 launch robp_launch localization_icp_launch.py
```

