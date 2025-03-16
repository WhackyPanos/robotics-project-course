# Behavior Tree Package
Package with the behavior trees for the Exploration and Collection phases. 

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

# Collection Phase

![n](https://gits-15.sys.kth.se/storage/user/27281/files/7371e68a-d6fa-418b-9f8e-9a6e69d3cfaa)
