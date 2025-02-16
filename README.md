# Group 3 Project Repo (DD2419)
This repo is subdivided into several branches. Generally, each branch corresponds to one package. When the package is terminated, the branch is merged in the development branch (devel).

## Steps to run any package
1) In 2 terminals, run
```
ssh robot
```
and insert *robot* as password

2) In one of them, run
```
fastdds discovery -i 0 -t 192.168.128.107 -q 42100
```

## Issues when building
If you encounter unforeseen issues when building, go to the root - you should have in the terminal

```
group3-robot@group3-robot:~/robp_group3$
```
Then run 
```
unset COLCON_PREFIX_PATH
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
source /opt/ros/jazzy/setup.bash
sudo apt update
rosdep update
```
Then install dependencies in each workspace by doing
```
cd arm_ws
rosdep install --from-paths src --ignore-src -r -y
```
```
cd ..
cd robp_ws
rosdep install --from-paths src --ignore-src -r -y
cd ..
```
Remove the instal, build and log folders and build
```
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

## Additional notes
Do not forget to build each package after changing setup.py or package.xml. Use the select command.

## pick branch

