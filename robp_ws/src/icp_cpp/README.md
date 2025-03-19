# ICP Package
This is a "secundary" package which will be used by the localization package. This package is responsible for taking the LiDAR scans, convert them into PointCloud and apply ICP to get the corrected PointCloud msg and the respective transformation. This transformation will be handled by the localization package.

The ``icp_node`` (in ``icp_node.cpp``) can perform ICP between two consecutive point clouds or between the very first point cloud and the current one. This is defined by the msg published in ``/icp/type``, which is controlled by the Localization node (in the localization pkg).

#### **Topics**
| Topic | Type | Published in | Subscribed in| Description |
| --- | ---| --- | --- | --- |
| /icp/transformed_point_cloud | PointCloud2 | icp_node.cpp -> icp_cpp pkg |  localization_transform.py | New point cloud, may be useful later on 
| /icp/transform | TransformStamped | icp_node.cpp -> icp_cpp pkg | localization_transform.py | Transformation between two point clouds
| /icp/type | String | localization_transform |  icp_node.cpp -> icp_cpp pkg| The msg can be "Normal" or "LoopClosure"
| /scan | LaserScan | LIDAR |  icp_node.cpp -> icp_cpp pkg| -
---

This is launched when the localization package nodes are launched, which can be done with
```
ros2 launch robp_launch localization_icp_launch.py
```
The launch file has to be completed since no movement is performed with the current launch file.

