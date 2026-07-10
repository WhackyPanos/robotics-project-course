# **Detection Package**

## **Overview**
The `map_file` package is a ROS2 python package for creating and updating a txt map file containing objects classification and positions. It consists of one node:

1. **`map_file`** - Updates txt file and publishes array objects.

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
colcon build --packages-select map_file
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
### **`Map File Node`**
**Purpose**: Maps objects and removes duplicates.

#### **Subscribed Topics**
| Topic | Type | Description |
| --- | ---| --- |
| /detection/class | std_msgs/String.msg | Objects classification and position.|
---
#### **Published Topics**
| Topic | Type | Description |
| --- | ---| --- |
| /object_positions | visualization_msgs/MarkerArray.msg | Array containing all currently detected objects. Frame_id corresponds to classification, x and y is the position and z corresponds to the angle. |
---
#### **Parameters**
| Parameter | Type | Default value | Description |
| --- | ---| --- | --- |
| box_threshold | float | 20 | Dublicate threshold for box. |
| object_threshold | float | 5 | Dublicate threshold for objects. |
| msg_topic | string | /detection/class | Topic for detected objects. |


