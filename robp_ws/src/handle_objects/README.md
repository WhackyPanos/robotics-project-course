# Handle_objects package / Pick branch
Open 4 terminals: 
```
fastdds discovery -i 0 -t 192.168.128.107 -q 42100
```
```
ros2 launch robp_launch arm_servo_launch.py
```
```
py-trees-tree-viewer
```
```
ros2 launch robp_launch pick_objects_launch.py
```

Check https://github.com/migsdigs/Hiwonder_xArm_ESP32 for more info regarding the arm
For the IK package, check http://docs.ros.org/en/diamondback/api/kdl/html/python/
