# Path planning

## Notice
The package `path_planner.py` is now deprecated and replaced by `motion.py`.

## General Info 
This is the (local) path planner that I have been working on. The idea is that you publish a *[!!]* _PoseStamped_ *[!!]*  message to `/motion/goal` and the system will pick it up and start navigating to it. It doensn't offer any 'shortest route' planning, it just takes the hypotenuse from the robot's current location to the goal and starts navigating to it. 

It also provides the option of publishing a _Path_ message to the `/motion/path` topic, with the specified path the robot has to follow.

In both cases, once the travel move is complete, the robot will rotate about itself until it reaches its final target yaw, given in the above stated message.

## How to use
To use the system, you may include the node in any launch file and just publish a *pose* to the `/motion/goal` topic. When it reaches its goal, then a 'True' message is published on the `/motion/goal_reached` topic. 

For convenience, we have created a couple of test nodes. First is the `point_generator`, which samples a random point in a given bounding box and navigates to it, and the `point_publisher_test` which moves around a couple predetermined points. 

## Extra functions / status
New functions have been implemented to faciliate the collection phase. 

- __adjust_yaw(angle)__: adjusts the yaw of the robot, to allow for fine-tuning of its yaw for easier collection. *untested*
- __reverse(distance)__: reverses the robot by `distance` amount, which may be useful for a plan B system.*untested*


## Use as an object (eg. for bhv-trees)
This node may also be used as an object for use with behaviour trees. One may use these parameters:

- __self.goal_threshold__: sets the slack distance that a goal is achieved.
- __self.goal_reached_flag__: set to TRUE when a goal is achieved
- __self.path_reached_flag__: set to TRUE when a path goal is achieved (one shouldn't use the goal_reached flag if they expect a path)
- *NEW* __self.do_yaw = False__: set to FALSE if you want to ignore orientation data in the give pose(s). Should be FALSE for exploration, and may need to be TRUE for collection. If set to TRUE and a 'goal' is given, then the robot will navigate to the given goal and once it reaches its threshold, will adjust its orientation to respect the given yaw in the pose data. If a 'path' is given, the robot will follow the path and adjust its yaw once it reaches the last goal in the given path. Should be used responsibly, since rotations may cause collisions with the back part of the robot if objects are nearby. 

## Status
- The system has been tested extensively and it seems to be working right.
- The extra functions (see above) have not yet been tested.