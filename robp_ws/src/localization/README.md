# Localization Package
This package is responsible for taking the ICP result and computing the transform between map frame and odom frame. As of now, the ICP is triggered by the ``icp_master`` function (in the Localization node) and it's computed between a global point cloud and the most recent received point cloud.

The new point cloud is received and transformed to the odom frame. ICP is then performed between the global point cloud and the incoming point cloud, but only after applying some filtering and outlier rejection (functions from the PCL library). If the ICP was performed with a good fitness score, the new point cloud is added to the global one. At the end, the updated global point cloud is converted to the map frame and published in a topic so we can visualize in RVIZ2.

The transform comes from the icp package and this package is responsible for computing the transform between map and odom. 
#### **Topics**
| Topic | Type | Published in | Subscribed in| Description |
| --- | ---| --- | --- | --- |
| /icp/transformed_point_cloud | PointCloud2 | icp_node.cpp -> icp_cpp pkg |  localization_transform.py | New point cloud, may be useful later on 
| /icp/transform | TransformStamped | icp_node.cpp -> icp_cpp pkg | localization_transform.py | Transformation between two point clouds
| /icp/type | String | localization_transform |  icp_node.cpp -> icp_cpp pkg| The msg can be "Normal" or "LoopClosure"
| /icp/global_point_cloud | PointCloud2 | icp_node.cpp -> icp_cpp pkg |  rviz2 | pointCloud for visualization purposes
---
*PROBLEM*: the point cloud that´s being sent to rviz2 it is actually in the odom frame, however the global point cloud remains constant (which it shouldn´t). If I send the global point cloud, it constantly moves in rviz, since the transform between map and odom changes substantially between icp runs. One suggestion would be to perform ICP in the map frame but I am not sure if that't the best approach.

*Solution to try*: do not transform the whole global point cloud as the points that are already on the point cloud should not be affected by the transform. Transform only the new points to map and add them to the global point cloud.

Launch file:

```
fastdds discovery -i 0 -t 192.168.128.107 -q 42100
```

```
ros2 launch robp_launch setup_localization_launch.py
```

```
ros2 launch robp_launch localization_icp_launch.py
```

