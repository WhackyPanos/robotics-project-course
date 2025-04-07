# Localization Package
This package is responsible for taking the ICP result and computing the transform between map frame and odom frame. As of now, the ICP is triggered by the ``icp_master`` function (in the Localization node) and it's computed between a global point cloud and the most recent received point cloud.

The new point cloud is received and transformed to the odom frame. ICP is then performed between the global point cloud and the incoming point cloud, but only after applying some filtering and outlier rejection (functions from the PCL library). If the ICP was performed with a good fitness score, the new point cloud is added to the global one. At the end, the updated global point cloud is converted to the map frame and published in a topic so we can visualize in RVIZ2.

The localization node triggering can me made via ``main`` or via ```localization_bhv.py```. The former is used in the exploration phase and the latter is used in the collection phase. For the collection phase an additional topic will be defined to trigger the localization.

The transform comes from the icp package and this package is responsible for computing the transform between map and odom. 
#### **Topics**
| Topic | Type | Published in | Subscribed in| Description |
| --- | ---| --- | --- | --- |
| /icp/transformed_point_cloud | PointCloud2 | icp_node.cpp -> icp_cpp pkg |  localization_transform.py | New point cloud, may be useful later on 
| /icp/transform | TransformStamped | icp_node.cpp -> icp_cpp pkg | localization_transform.py | Transformation between two point clouds
| /icp/type | String | localization_transform |  icp_node.cpp -> icp_cpp pkg| The msg can be "Normal" or "LoopClosure"
| /icp/global_point_cloud | PointCloud2 | icp_node.cpp -> icp_cpp pkg |  rviz2 | pointCloud for visualization purposes
| /icp/activate | Bool | (1) |  localization_bhv | The boolean message will indicate if localization is needed or not, which is managed by the behavior

---
colcon build --symlink-install --packages-skip usb_cam 
source install/setup.bash

ros2 bag play --read-ahead-queue-size 100 -l -r 1.0 --clock 100 --start-paused ~/aREPO/robp_group3/robp_ws/bag/lidar_dynamic/lidar_dynamic.mcap 

ros2 run tf2_ros tf2_echo odom map
---
(1) Ideally, it should be published by the motion behavior when the point is reached, ie, when the robot stops completely and return SUCCESS. It should publish a FALSE message so the localization is not performed why the robot is standing still. Besides, the picking and placing behaviors should publish a TRUE message (when these behaviors return SUCCESS, the robot will start moving and localization should be re-initiated).

Launch file:

```
fastdds discovery -i 0 -t 192.168.128.107 -q 42100
```

```
ros2 launch robp_launch localization_icp_launch.py
```

