# MTRX4701 Tortellini Major - QR Destinations
While the robot's driving around doing robot things it takes note of all the AruCo tags that it sees. Matching them up with some LiDAR points to generate a goal pose infront of each.\
Each tag's info get's published to list of destinations, where each destination has a pose the robot can be told to navigate to via. Nav2's ```/goal_pose``` topic.
## Message Types
### Summary
Every time there's an update to destination data, the node publishes a ```DestinationList``` to the ```/list``` topic.
### Destination Messages
Really just attaches some semantic information to a pose that can be handed to ```/goal_pose```. Every pose will be in the ***map frame***. At the moment the name string isn't being used, but tag is the id of the tag that the pose corresponds to.
```
string name
int32 tag
geometry_msgs/Pose pose
```
