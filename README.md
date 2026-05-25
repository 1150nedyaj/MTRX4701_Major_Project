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

# Running Nav2 w/ StampedTwist Commands
By default Nav2 will send command velocity messages as ```Twist``` messages, while the turtlebot will only accept ```TwistStamped```. This can be fixed by giving Nav2 Nodes the parameter ```enable_stamped_cmd_vel: True ``` in the Nav2 config file.\
In this case, its easy just to setup our own config and point Nav2 to it whenever it gets brought up.
```
ros2 launch nav2_bringup navigation_launch.py params_file:=<our-custom-params-file>
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
## mmWave Radar Module
### Firing it up
To bring up the radar with the default array config file run,
```
ros2 launch mmwave_radar array.bringup.launch.py
```
If you wish to specify a config run,
```
ros2 launch mmwave_radar array.bringup.launch.py array_config_file:=<array-config-file>.yaml
```
Every time the Array Bringup gets run the config file being used gets printed to the terminal window. Which will look something like this,
```
### Launching Array with config v0_frame_custom.yaml ###
```
Each module has it's node within the array's namespace, and the readings get published at a bit under 10Hz. If you were to be running 1 radar (id 0) the topics would look like more or less like this.
```
/mmWave_array/radar_0/detections
/mmWave_array/radar_0/pose
```

### Radar Message Types
The RD-03D mmWave Radar modules can give range, bearing and speed of up to 3 targets at a time. The ```RadarDetection``` gives each detection in cartesian coordinates relative to the frame of the sensor, as well as providing covariance values on the detection. A list of these get published with a stamp in the ```detections``` topic (see the example output below). Note that the covariance values are (x,y), and have been flattened.\
As Rviz doesn't know how to display ```RadarDetection```'s, the modules also publish a ```PoseWithCovarianceStamped``` for each detection to help with debugging. The orientation of this message is irrelevant and it doesn't contain the velocity readings for the radar signature.
```
---
header:
  stamp:
    sec: 1779089060
    nanosec: 16750330
  frame_id: radar0
detections:
- position:
    x: 3.353
    y: 1.353
    z: 0.0
  covariance:
  - 0.6573811523000166
  - -1.4060820426178537
  - -1.4060820426178537
  - 3.5745477375444676
  speed: -0.6800000071525574
- position:
    x: 0.202
    y: -0.045
    z: 0.0
  covariance:
  - 0.08581499991197789
  - -0.018786000395121464
  - -0.018786000395121464
  - 0.005671731559676978
  speed: -0.6800000071525574
- position:
    x: 3.947
    y: 4.239
    z: 0.0
  covariance:
  - 11.636879333365133
  - -10.751482125216368
  - -10.75148212521637
  - 10.100875194203589
  speed: 0.6800000071525574
---

```

### Array Config Files
These files describe how the radar array has been setup, labelling each radar, specifiying their serial port and their transform from a parent frame. They can be found in the config folder for the package, and will all look something like this,
```
launch_settings:
  radar_modules:
    - identifier: 0
      interface: /dev/ttyAMA0
      parent_frame: base_link
      translation_x: 0.1
      translation_y: 0.092
      translation_z: 0.092
      roll: 0.00
      pitch: 0.00
      yaw: 0.00
    - identifier: 1
      interface: /dev/ttyAMA1
      parent_frame: base_link
      translation_x: 0.2
      translation_y: 0.092
      translation_z: 0.092
      roll: 0.00
      pitch: 0.00
      yaw: 0.00
```

## Pi 5 UART Expander Board
![Preview](uart_breakout_board/UART_Expander_Preview.png)
This Pi 5 hat was built improve quality of life while using the mmWave Radar modules. Aside from maintaining the functionality of the supplied WiFi board, it breaks out all 5 of the UART interfaces available through the Pi's GPIO pins. Each interface includes a led on the transmit line for debugging. This doesn't appear to impact communication with the Pi, though the led's can be left disconnected if they start to cause issues.\
The Gerber file and Schematic for the board can be found in the uart_breakout_board folder.\
**NOTE**: The WiFi boards that come with the turtlebots have their status led hooked up to pin 16. This board uses pin 13 due to a mix-up during the design.\
**NOTE**: The current version provides the UART modules with 3.3v for power.
