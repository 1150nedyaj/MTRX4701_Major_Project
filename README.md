# MTRX4701_Major_Project

Run 
```
ros2 launch people_avoider_gui system.launch.py is_real:=false 
```
and it should automatically open gazebo sim you can change worlds by modifying /people_avoider/src/people_avoider_gui/launch/system.launch.py

## Setup Notes:
### Needed to install turtlebot3-msgs
```
sudo apt update
sudo apt install ros-jazzy-turtlebot3-msgs
``` 

# Nav2 command and setup for running

### Time syncing: 
```
sudo timedatectl set-ntp false
sudo systemctl stop chrony 2>/dev/null
export MASTER=ubuntu@10.42.0.1
sudo date -u -s "@$(ssh "$MASTER" date +%s)"
sudo systemctl start chrony 2>/dev/null
```

## Source terminal after build
### Need to colcon build --symlink-install 
```
source /opt/ros/jazzy/setup.bash
source ~/Desktop/MTRX4701_Major_Project/install/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=13
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Need slam tool box install
Run slam tool box
```
ros2 launch slam_toolbox online_async_launch.py
```

### Need nav2 install
Run nav2 bringup without a predefine map
```
ros2 launch nav2_bringup navigation_launch.py
```

### Rviz 
```
ros2 run rviz2 rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz
```

### For simulation of nav2 
```
Launch simulation world:
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

ros2 run rviz2 rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz
```
