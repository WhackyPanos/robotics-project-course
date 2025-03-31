# Behavior Tree Package
Package with the behavior trees for the Exploration and Collection phases. 

If detection build is not working, on /robp_group3 :
```
rm -rf build install log
colcon build --package-skip detection
source install/setup.bash
```
# Collection Phase

While being on robp_group3 (not robp_ws), build, source and then launch:

``` fastdds discovery -i 0 -t 192.168.128.107 -q 42100 ```

``` ros2 launch robp_launch collection_hw_tf_launch.py  ```

``` ros2 launch robp_launch arm_servo_launch.py ```

``` ros2 launch robp_launch collection_node_launch.py ```

#### **Important topics**
| Topic name | Published | Subscribed from | Description
| --- | ---| --- | --- |
| /motion/goal | ObjectsList behavior (collection_bhv.py)| A* node | The next goal is determined by the ObjectList node and published in this topic|
| /next_goal/object/need | (1), (2) | ObjectsList behavior (collection_bhv.py) | String msg to notify this behavior that a new object is to be computed (the closest one). It can be either 'Object', 'Box' or 'None'. |
| /next_goal/object/update | (1) | ObjectsList behavior (collection_bhv.py) | Point message to indicate which object should be removed from the list permanently |
| /icp/activate | (3), (4) | localization_bhv (localization package) | Boolean msg to indicate when localization needs to be stopped/activated |



(1) Nodes susceptible to failure that force the object/box to be eliminated from the list 

(2) Terminal nodes, *ie*, nodes that mark the end of a cycle (probably the placing node).

(3) Motion behavior: when this behavior returns success, the robot will stop to pick or place. Before returning success, this message has to be published (check localization README)

(4) Before picking and placing behavior return success, so localization is activated before the robot starts moving


![n](https://gits-15.sys.kth.se/storage/user/27281/files/7371e68a-d6fa-418b-9f8e-9a6e69d3cfaa)



#### **Important behaviors files**
| File | Package | Description |
| --- | ---| --- |
| pick_objects.py | handle_objects| File with the behaviors involved in the picking/placing motions, which includes IK, lifting arm, search for the object, etc |
| ppe.py | behavior_tree | Example file to serve as guide for the behaviors involved in the path planning and path execution |
---
#### **Behaviors (Panos, Sebastian and Dominik)**
| Behavior | Description |
| ---| --- |
| Plan Path to Object (1 in the image, **Sebastian**) | This behavior, when ticked, should give a list of points corresponding to the path. The list should have the minimum number of points possible and should be published on the topic "/goal/path". This list will actually be list of sublists: each sublist correspond to a path point, which will include its coordinates + indicator (Box, Object or Intermediate point). The goal point should be subscribed in the "/goal/point" topic and stored in a class attribute. |
| Move to (pick) point (2, in the image, **Panos**) | The behavior, when activated, should send duty cycle commands to the robot. It should subscribe to the "/goal/path" topic, storing the received goal points as a class attribute. These points will be processed in the update function, which will be called multiple times during movement. Flags may be needed to prevent restarting motion. The system should use the map grid, current odometry pose, and goal point to generate movement commands, progressing through the list as each point is reached. Success is determined by a threshold that varies based on the object type (Box, Object, or Intermediate), as indicated in the "/goal/path" messages. |
| Find object (3 in the image, **Dominik**) | This behavior, when ticked, perform the object detection and give the x and y coordinates. If something is found, return sucess, otherwise return failure. The x and y coordinates should be published in the topic "/pick/object_pos" |
