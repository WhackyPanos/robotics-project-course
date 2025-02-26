# **Detection Package**

## **Overview**
The `detection` package is a ROS2 C++ package for detecting and classifying objects in a 3D point cloud. It consists of two main nodes:

1. **`clustering_node`** - Segments objects using Euclidean clustering.
2. **`classifier_node`** - Analyzes the height of each cluster and classifies it.

---

## **Installation**
### **Dependencies**
Ensure you have the necessary dependencies installed that are specified in **package.xml**.
```bash
rosdep install -i --from-path src --rosdistro jazzy -y --as-root pip:false
```

### **Build**
In your robot workspace:
```bash
colcon build --packages-select detection
source install/setup.bash
---
```
## **Launching the Nodes**
The robp_launch package includes a launch file to start both nodes with their respective parameters along the RGB-D camera and the map file generator.
To start both `clustering_node` and `classifier_node`:
```bash
ros2 launch robp_launch detection_launch.py
```
This will:
- Subscribe to a raw point cloud topic 
- Publish clustered objects
- Process and classify the clusters
- Generate map file (see for more information in map_file package)

---

## **Nodes specification** 
### **`Clustering Node`**
**Purpose**: Segments objects from the raw point cloud using Euclidean clustering.

#### **Subscribed Topics**
| Topic | Type | Description |
| --- | ---| --- |
| /camera/camera/depth/color/points | sensor_msgs/msg/PointCloud2 | Raw point cloud data from the depth camera.|
| /cmd_vel | sensor_msgs/msg/Twist | Velocity sent by the controller. |
---
#### **Published Topics**
| Topic | Type | Description |
| --- | ---| --- |
| /camera/camera/depth/color/cluster_points | sensor_msgs/msg/PointCloud2 | Segmented object clusters. |
---
#### **Parameters**
| Parameter | Type | Default value | Description |
| --- | ---| --- | --- |
| cloud_topic | string | /camera/camera/depth/color/points | Input point cloud topic. |
| cluster_topic | string | /camera/camera/depth/color/cluster_points | Output clustered point cloud topic. |
| dist_filter_min | double | 0.0 | Minimum distance filter for points. |
| dist_filter_max | double | 1.0 | Maximum distance filter for points. |
| height_filter_min | double | -0.025 | Minimum height filter for points (above RGB-D cam). |
| height_filter_max | double | 0.075 | Maximum height filter for points (above ground). |
| cluster_min_size | int | 100 | Minimum number of points per cluster. |
| cluster_tolerance | double | 0.05 | Distance tolerance for clustering. |
| ang_velocity_threshold | double | 0.3 | Maximum angular velocity clustering is done. |

---

### **`Classifier Node`**
**Purpose**: Analyzes segmented clusters and classifies them based on height.

#### **Subscribed Topics**
| Topic | Type | Description |
| --- | ---| --- |
| /camera/camera/depth/color/cluster_points | sensor_msgs/msg/PointCloud2 | Segmented clusters from `clustering_node`. |
---
#### **Published Topics**
| Topic | Type | Description |
| --- | ---| --- |
| /detection/class | std_msgs/msg/String | Classification result. |
| /visualization_marker | visualization_msgs/msg/Marker | Visualization of the object bounding box (optional). |
---
#### **Parameters**
| Parameter | Type | Default value | Description |
| --- | ---| --- | --- |
| cloud_topic | string | /camera/camera/depth/color/cluster_points | Input point cloud topic. |
| classification_topic | string | /detection/class | Output classification topic. |
| box_filter_min | double | 0.0 | Minimum height threshold for box classification. |
| box_filter_max | double | 0.008 | Maximum height threshold for box classification. |
| box_filter_threshold | int | 50 | Minimum number of points required for box classification. |
| animal_filter_min | double | 0.045 | Minimum height threshold for animal classification. |
| animal_filter_max | double | 0.05 | Maximum height threshold for animal classification. |
| sphere_filter_min | double | 0.056 | Minimum height for sphere classification. |
| sphere_filter_max | double | 0.059 | Minimum height for sphere classification. |
| visualize_OBB | bool | false | Enables visualization of the oriented bounding box (OBB). |
