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

## Additional notes
Do not forget to colcon build --symlink-install and source install/setup.bash after changing/creating a new package

## pick branch

