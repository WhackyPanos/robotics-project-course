# Path planning

## General Info 
This is the (local) path planner that I have been working on. The idea is that you publish a _Point()_ object to `/temp_goal` and the system will pick it up and start navigating to it. It doensn't offer any 'shortest route' planning, it just takes the hypotenuse from the robot's current location to the goal and starts navigating to it. For obstacle avoidance, the system incorporates the 'bug' algorithm, which navigates the robot around the obstacle until it reaches this line again. 

## How to use
To use the system, you may include the node in any launch file and just publish a point to the `/temp_goal` topic. The robot _should_ start moving. When it reaches its goal, then a 'True' message is published on the `/goal_reached` topic. 
For convenience, we have created a couple of test nodes. First is the `point_generator`, which samples a random point in a given bounding box and navigates to it, and the `point_publisher_test` which moves around a couple predetermined points. 

## Status
Currently WIP. 
Working:
- Navigation system
- Obstacle _detection_, 20 degrees in front of the robot (only with LiDAR)

Not working:
- Bug algorithm is untested, so debugging is necessary
- Should include detection of small objects through the camera, but that should be done in collaboration with the detection node.