# sensor_fusion

<<<<<<< HEAD
Confirms lidar people detections using radar. A person detected by the lidar
circle-detector is only forwarded if the radar also reports a detection nearby.
This removes false positives caused by circular static objects — traffic cones,
poles, wheels — that the lidar cannot distinguish from legs.

```
/scan ──► people_detect ──► /lidar/circle_candidates ──┐
                                                        ├──► sensor_fusion ──► /fusion/people
         radar_module_node ──► /mmWave_array/.../detections ─┘
```

---

## Dependencies

| Package | Why |
|---|---|
| `radar_messages` | Custom message type `StampedRadarDetections` |
| `lidar_radar` | Produces `/lidar/circle_candidates` that this node consumes |
| `mmwave_radar` | Produces radar detections and publishes the `radar0` TF frame |

---

## Building

`radar_messages` must be built before anything that imports its types.

```bash
cd ~/turtlebot3_ws

colcon build --packages-select radar_messages
source install/setup.bash

colcon build
source install/setup.bash
```

Confirm the package is visible:

```bash
ros2 pkg list | grep sensor_fusion
# sensor_fusion
```

---

## Running

### With the system launch file (recommended)

Launches `people_detect` and `sensor_fusion` together. Pass `is_real:=true` to
skip Gazebo when replaying a bag or using the real robot.

```bash
ros2 launch people_avoider_gui system.launch.py is_real:=true
```

### Standalone

```bash
ros2 run sensor_fusion sensor_fusion
```

### With a bag file

Open three terminals, each sourced with `source ~/turtlebot3_ws/install/setup.bash`.

**Terminal 1 — play the bag**
```bash
ros2 bag play ~/turtlebot3_ws/bags/test_recording/ --clock --loop
```

**Terminal 2 — launch the processing nodes**
```bash
ros2 launch people_avoider_gui system.launch.py is_real:=true use_sim_time:=true
```

**Terminal 3 — radar static transforms** *(only needed if `/tf_static` is absent from the bag)*
```bash
ros2 run mmwave_radar array_transforms_node.py \
  $(ros2 pkg prefix mmwave_radar)/share/mmwave_radar/config/v1_frame_solo_radar.yaml
```

---

## Parameters

All parameters can be set in the launch file or passed on the command line.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `radar_topics` | `string[]` | `["/mmWave_array/radar_0/detections"]` | Radar detection topics to subscribe to. Add one entry per radar module. |
| `fusion_distance_threshold` | `double` | `1.0` | Maximum distance in metres between a lidar person and a radar point for the detection to be confirmed. Increase if radar is noisy; decrease to be stricter. |
| `radar_timeout` | `double` | `0.5` | Radar readings older than this many seconds are discarded from the buffer. Should be at least 2× the radar publish period (~0.15 s). |
| `target_frame` | `string` | `"base_link"` | TF frame used as the common reference for spatial comparison. Both lidar and radar detections are transformed into this frame before matching. |

**Example — overriding parameters at the command line:**
```bash
ros2 run sensor_fusion sensor_fusion \
  --ros-args \
  -p fusion_distance_threshold:=0.75 \
  -p radar_timeout:=0.4 \
  -p radar_topics:='["/mmWave_array/radar_0/detections", "/mmWave_array/radar_1/detections"]'
```

**Example — two radar modules in the launch file:**
```python
parameters=[{
    "radar_topics": [
        "/mmWave_array/radar_0/detections",
        "/mmWave_array/radar_1/detections",
    ],
    "fusion_distance_threshold": 1.0,
    "radar_timeout": 0.5,
    "target_frame": "base_link",
}]
```

---

## Topics

### Subscribed

| Topic | Type | Description |
|---|---|---|
| `/lidar/circle_candidates` | `geometry_msgs/PoseArray` | Candidate people positions from the lidar circle-detector |
| `/mmWave_array/radar_0/detections` | `radar_messages/StampedRadarDetections` | Radar detections (one topic per module, configurable) |

### Published

| Topic | Type | Description |
|---|---|---|
| `/fusion/people` | `geometry_msgs/PoseArray` | Radar-confirmed people, same frame as input |
| `/fusion/people_markers` | `visualization_msgs/MarkerArray` | Green cylinders for confirmed people (RViz) |

---

## Verifying it works

**Check all nodes are running:**
```bash
ros2 node list
# /people_detect
# /sensor_fusion
```

**Check data is flowing at each stage:**
```bash
ros2 topic hz /scan                              # raw lidar
ros2 topic hz /mmWave_array/radar_0/detections  # raw radar
ros2 topic hz /lidar/circle_candidates           # lidar people candidates
ros2 topic hz /fusion/people                     # confirmed people
```

**Inspect a single fusion output message:**
```bash
ros2 topic echo /fusion/people --once
```

**Watch the fusion node's decisions in real time:**
```bash
ros2 run rqt_console rqt_console
```
Set the logger filter to `sensor_fusion` and the level to `Debug` to see a
line printed for every scan:
```
[sensor_fusion]: Fusion: 2 lidar candidates → 1 confirmed
[sensor_fusion]: No recent radar data; suppressing all lidar detections.
```

**Check the TF tree is complete** (radar0 must be connected to base_link):
```bash
ros2 run tf2_tools view_frames
# opens frames.pdf in the current directory
```

**Visualise in RViz2:**
```bash
rviz2
```

| Display | Topic | What you see |
|---|---|---|
| `LaserScan` | `/scan` | Raw lidar ring |
| `PoseArray` | `/lidar/circle_candidates` | All circular detections (including false positives) |
| `PoseArray` | `/fusion/people` | Only radar-confirmed people |
| `MarkerArray` | `/fusion/people_markers` | Green cylinders over confirmed people |
| `MarkerArray` | `/lidar/circle_markers` | Red ankle dots, blue person centres |

Set **Fixed Frame** to `base_link` in RViz's Global Options panel.

---

## Tuning

| Symptom | Likely cause | Fix |
|---|---|---|
| Real people are being dropped | `fusion_distance_threshold` too small or radar misaligned | Increase threshold; verify `base_link → radar0` TF matches physical mounting |
| Static objects still getting through | `fusion_distance_threshold` too large | Decrease threshold (try `0.5`) |
| All detections suppressed even with people present | Radar buffer emptying too fast | Increase `radar_timeout` |
| Node logs "TF unavailable" on every scan | `radar0` frame not in TF tree | Run `array_transforms_node.py` or ensure `/tf_static` is in the bag |
=======
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
>>>>>>> radarPackage
