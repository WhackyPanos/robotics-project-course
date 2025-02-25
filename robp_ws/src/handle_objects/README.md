# Handle_objects package / Pick branch
For now, this README is for the collection phase. Therefore, the it does not launch only this package: it launches the behavior tree (see corresponding package) but has collection is mainly pick and place, it will be detailed here.

When launching this, the robot will move itself to an object with fixed position, try to pick it and lift the arm (25/02/2025). So, to use this correctly:

#### **Topics**
| Topic | Type | Published in | Subscribed in| Description |
| --- | ---| --- | --- | --- |
| /cmd_vel | Twist | collectObjectMs2.py -> path_planner pkg |  twist2duty.py -> joystick_teleop pkg | It is limited by maximum and minimum values 
| /temp_goal | PointStamped | goCollect_bhv.py -> behavior_tree pkg | collectObjectMs2.py -> path_planner pkg. | As for now, is fixed but the user can change it in the def __init_ of the goCollect class
| /goal_reached | Bool | collectObjectMs2.py -> path_planner pkg |  goCollect_bhv.py -> behavior_tree pkg| Sends True message when we are at a certain distance from the object
---

To run this properly, firstly open 3 terminals and run:
```
fastdds discovery -i 0 -t 192.168.128.107 -q 42100
```
```
ros2 launch robp_launch arm_servo_launch.py
```
```
py-trees-tree-viewer
```
These commands won't print anything useful in the terminal, so for simplicity you can keep them in the background. You may have to run the last one every one you run the program if you want to see the behavior tree.
To ensure that the robot starts at the origin of the map frame, run:
```
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```
Finally, launch the main launch file, which will launch the odometry, the path planner, the behavior tree and the arm manipulation nodes/scripts.
```
ros2 launch robp_launch pick_objects_launch.py
```

Check https://github.com/migsdigs/Hiwonder_xArm_ESP32 for more info regarding the arm
For the IK package, check http://docs.ros.org/en/diamondback/api/kdl/html/python/
