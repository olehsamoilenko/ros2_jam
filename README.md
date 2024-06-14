## Setup:
```bash
$ cat .bashrc
source /opt/ros/humble/setup.bash
$ source ~/.bashrc
```

<!-- 
$ source dev_ws/install/local_setup.bash
$ ros2 launch basic_mobile_robot basic_mobile_bot_v1.launch.py  -->

## Turtle Bot:
```bash
$ sudo apt install ros2-humble-turtlebot3*
$ export TURTLEBOT3_MODEL=waffle # or burger
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py # or turtlebot3_house.launch.py
$ ros2 run turtlebot3_teleop teleop_keyboard
```

## Lidar
Lidar should post to `/scan` topic. Topics are needed for inter-process communication (between modules)
```bash
$ ros2 topic list
$ ros2 topic info /scan # LaserScan means Lidar
$ ros2 topic echo /scan
```

## SLAM
```bash
$ sudo apt install ros-humble-slam-toolbox
$ ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True # online means live (no map), async means process the recent scan (works faster)
# use_sim_time: True for simulation, False for a real robot
```
Another options. Mode and parameters (e.g. map) can be configured there:
```bash
$ ls /opt/ros/humble/share/slam_toolbox/config/
mapper_params_lifelong.yaml
mapper_params_localization.yaml
mapper_params_offline.yaml
mapper_params_online_async.yaml
mapper_params_online_sync.yaml
```

`rviz` is odometry (perception of robot, what robot sees). If a real robot is used, we will not have simulation (Gazebo), but still will have `rviz` (odometry)

```bash
$ ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

<!-- TODO try no parameters (no globl planner?) -->

## Navigation
```bash
$ sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
$ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True # costmap appears: global planner and controller
# run "Nav2 Goal" (standard navigation is a bit stupid)
```

## Save map
Option 1:
```bash
$ ros2 run nav2_map_server map_saver_cli -f my_map
```
Option 2:<br>
`rviz` -> Panels -> Add new panel -> Save Map

<!--
$ ros2 service list # see slam_toolbox services  -->

<!-- 
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/osamoile/projects/ros2/dev_ws/src/basic_mobile_robot/models
 -->

## Algorithms
SLAM toolbox architecture: https://github.com/SteveMacenski/slam_toolbox/blob/ros2/images/slam_toolbox_sync.png<br>
SLAM toolbox article: https://joss.theoj.org/papers/10.21105/joss.02783

Graph-based SLAM: https://disco.ethz.ch/courses/fs14/seminar/paper/Pascal/1.pdf # instead of EKF SLAM<br>
Graph-based SLAM explanation: http://ais.informatik.uni-freiburg.de/teaching/ws11/robotics2/pdfs/rob2-10-landmark-slam.pdf

Karto SLAM (improvement of graph-based): http://www.yahboom.net/public/upload/upload-html/1665711621/8.karto%20mapping%20algorithm.html # rqt_graph
<!--  -->

## Other links
EKF SLAM from scratch: https://ngmor.github.io/projects/ekf-slam-from-scratch/
