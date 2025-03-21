# Path planning

## Notice
The package `path_planner.py` is now deprecated and replaced by `motion.py`.

## General Info 
This is the (local) path planner that I have been working on. The idea is that you publish a _PointStamped_ message to `/motion/goal` and the system will pick it up and start navigating to it. It doensn't offer any 'shortest route' planning, it just takes the hypotenuse from the robot's current location to the goal and starts navigating to it. 

It also provides the option of publishing a _Path_ message to the `/motion/path` topic, with the specified path the robot has to follow.

## How to use
To use the system, you may include the node in any launch file and just publish a point to the `/motion/goal` topic. When it reaches its goal, then a 'True' message is published on the `/motion/goal_reached` topic. 

For convenience, we have created a couple of test nodes. First is the `point_generator`, which samples a random point in a given bounding box and navigates to it, and the `point_publisher_test` which moves around a couple predetermined points. 

## Use as an object (eg. for bhv-trees)
This node may also be used as an object for use with behaviour trees. One may use these parameters:

- __self.goal_threshold__: sets the slack distance that a goal is achieved.
- __self.goal_reached_flag__: set to TRUE when a goal is achieved
- __self.path_reached_flag__: set to TRUE when a path goal is achieved (one shouldn't use the goal_reached flag if they expect a path)

## Status
- The system has been tested extensively and it seems to be working right.
- However, some tuning in the speeds (linear & angular) may still be necessary, but that appear only when we start doing real-world tests.
- The controller has been tuned, but further fine-tuning may be proven necessary (hopefully not tho).