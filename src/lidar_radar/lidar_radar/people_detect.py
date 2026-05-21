import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import MarkerArray, Marker
from builtin_interfaces.msg import Time

from lidar_radar import lidar_circle_detector
import numpy as np
import math


class PeopleDetectNode(Node):
    def __init__(self):
        super().__init__("people_detect")

        # Publishers
        self.circle_pub = self.create_publisher(
            PoseArray,
            "/lidar/circle_candidates",
            10,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/lidar/circle_markers",
            10,
        )

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10,
        )

        self.filtered_scan_pub = self.create_publisher(
            LaserScan,
            "/scan_filtered",
            10
        )

        self.previous_people = []
        self.get_logger().info("people_detect node started")

    def scan_callback(self, msg):

        #extract range/bearing and convert to x/y
        scan_points = []
        max_distance = 1 # Distance between two ankles to count as one person (m)

        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            # Check that the range is usable
            if not np.isfinite(distance):
                continue

            if distance < msg.range_min or distance > msg.range_max:
                continue

            x = distance * np.cos(angle)
            y = distance * np.sin(angle)

            scan_points.append([x, y])

        scan_points = np.array(scan_points)

        results = lidar_circle_detector.extract_circular_objects(scan_points)

        # Make a mutable copy of the original ranges tuple so we can edit it
        filtered_ranges = list(msg.ranges)
        
        # 3cm safety buffer to wipe out edge points near the legs
        padding = 0.03

        # Loop through every laser beam index and its measured distance
        for i, distance in enumerate(msg.ranges):
            # Skip points that are already infinite or invalid
            if not np.isfinite(distance):
                continue

            # Calculate where this specific laser point sits in 2D Cartesian space
            angle = msg.angle_min + i * msg.angle_increment
            x = distance * np.cos(angle)
            y = distance * np.sin(angle)

            # Compare this laser point against every leg circle found by the library
            for circle in results:
                cx, cy = circle.center
                
                # Compute straight-line distance from the laser point to the circle center
                dist_to_center = math.hypot(x - cx, y - cy)

                # If the point falls inside the leg radius (+ padding), erase it!
                if dist_to_center <= (circle.radius + padding):
                    filtered_ranges[i] = float('inf') # Set to infinity for Nav2 to ignore
                    break # Stop checking other circles for this beam and move to next point

        # Re-package everything back into a valid LaserScan message layout
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        filtered_msg.ranges = filtered_ranges
        
        if msg.intensities:
            filtered_msg.intensities = msg.intensities

        # Publish our human-free scan to the network
        self.filtered_scan_pub.publish(filtered_msg)
        ankles = []
        for circle in results:
            ankles.append([circle.center[0], circle.center[1]])

        people = []
        used = set()
        for i in range(len(ankles)):
            if i in used:
                continue

            x1, y1 = ankles[i]
            worked = False
            for j in range(len(ankles)):
                if i == j or j in used:
                    continue
                x2, y2 = ankles[j]
                manhatten = (abs(x1-x2)+abs(y1-y2))
                if manhatten < max_distance:
                    mid_x = (x1 + x2) / 2
                    mid_y = (y1 + y2) / 2
                    people.append([mid_x, mid_y, 0])
                    used.add(i)
                    used.add(j)
                    worked = True
                    break
            if worked == False:
                used.add(i)
                people.append([x1, y1, 0])
            
        if not self.previous_people:
            self.previous_people = people    
                    
        match_threshold = 2.0  # metres, tune this

        old_people = self.previous_people

        for person in people:
            xc, yc, _ = person

            closest_distance = match_threshold
            closest_previous = None

            for past_person in old_people:
                xp, yp, _ = past_person

                dx = xc - xp
                dy = yc - yp
                distance = math.hypot(dx, dy)

                if distance < closest_distance:
                    closest_distance = distance
                    closest_previous = past_person

            if closest_previous is not None and closest_distance > 0.001:
                xp, yp, _ = closest_previous

                dx = xc - xp
                dy = yc - yp

                theta = math.atan2(dy, dx)
                person[2] = theta

        # Store current people for next scan
        self.previous_people = [p.copy() for p in people]

        # Convert results into PoseArray
        message = PoseArray()
        message.header.stamp = msg.header.stamp
        message.header.frame_id = msg.header.frame_id

        i = 0
        for i in range(len(people)):
            p = Pose()
            p.position.x = float(people[i][0])
            p.position.y = float(people[i][1])
            p.position.z = 0.0

            theta = people[i][2]

            p.orientation.x = 0.0
            p.orientation.y = 0.0
            p.orientation.z = 0.0
            p.orientation.w = 1.0

            if theta is not None:
                p.orientation.z = math.sin(theta / 2.0)
                p.orientation.w = math.cos(theta / 2.0)

            message.poses.append(p)

        # Publish PoseArray
        self.circle_pub.publish(message)  
        
        #Make a MarkerArray of legs
        Detections = MarkerArray()

        leg = Marker()
        leg.header.frame_id = msg.header.frame_id
        leg.header.stamp = Time()

        leg.ns = "ankles"
        leg.id = 0

        leg.type = Marker.SPHERE_LIST
        leg.action = Marker.ADD

        leg.pose.orientation.w = 1.0

        leg.scale.x = 0.1
        leg.scale.y = 0.1
        leg.scale.z = 0.1

        leg.color.r = 1.0
        leg.color.g = 0.0
        leg.color.b = 0.0
        leg.color.a = 1.0

        for ankle in results:
            p = Point()
            p.x = float(ankle.center[0])
            p.y = float(ankle.center[1])
            p.z = 0.0
            leg.points.append(p)

        #Make Marker Array for People Centers
        center = Marker()
        center.header.frame_id = msg.header.frame_id
        center.header.stamp = Time()

        center.ns = "Person Centers"
        center.id = 1

        center.type = Marker.SPHERE_LIST
        center.action = Marker.ADD

        center.pose.orientation.w = 1.0

        center.scale.x = 0.1
        center.scale.y = 0.1
        center.scale.z = 0.1

        center.color.r = 0.0
        center.color.g = 0.0
        center.color.b = 1.0
        center.color.a = 1.0

        for person in people:
            p = Point()
            p.x = float(person[0])
            p.y = float(person[1])
            p.z = 0.0
            center.points.append(p)

        Detections.markers.append(leg)
        Detections.markers.append(center)


        self.marker_pub.publish(Detections)

def main(args=None):
    rclpy.init(args=args)

    node = PeopleDetectNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()