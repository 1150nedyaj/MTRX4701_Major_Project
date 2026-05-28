import sys
import threading
import math

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node

from PyQt6.QtCore import pyqtSignal,QTimer
from PyQt6.QtWidgets import (QApplication, QMainWindow,
                             QWidget, QVBoxLayout, QLabel,
                             QHBoxLayout, QStackedLayout, QGridLayout,
                             QPushButton, QGraphicsEllipseItem)

import pyqtgraph as pg
from collections import deque
from PyQt6.QtGui import QPen, QBrush, QColor

from .layout_colorwidget import Color

from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import BatteryState, LaserScan
from geometry_msgs.msg import PoseArray, PoseStamped
 
from destination_msgs.msg import DestinationListMsg

class DashboardWindow(QMainWindow):
    """
    GUI class.

    For widgets, layouts, labels, buttons, graphs, and Qt signals
    """

    speed_received = pyqtSignal(float)
    angle_received = pyqtSignal(float)
    battery_received = pyqtSignal(float)
    lidar_received = pyqtSignal(list)
    people_received = pyqtSignal(list)
    people_count_received = pyqtSignal(int)
    landmark_count_recieved = pyqtSignal(int)
    closest_person_received = pyqtSignal(float)
    closest_landmark_received = pyqtSignal(float)
    destinations_received = pyqtSignal(list)
    goal_button_pressed = pyqtSignal(str)
    goal_requested = pyqtSignal(str)
    valid_goals_received = pyqtSignal(list)
    plan_received = pyqtSignal(list)
    chosen_landmark_distances_received = pyqtSignal(float, float)

    def __init__(self):
        super().__init__()

        #All the things that are updated
        self.speed_received.connect(self.update_speed)
        self.angle_received.connect(self.update_angle)
        self.battery_received.connect(self.update_battery)
        self.lidar_received.connect(self.update_lidar_points)
        self.people_received.connect(self.update_people_points)
        self.people_count_received.connect(self.update_people_count)
        self.landmark_count_recieved.connect(self.update_landmark_count)
        self.closest_person_received.connect(self.update_closest_person)
        self.closest_landmark_received.connect(self.update_closest_landmark)
        self.destinations_received.connect(self.update_destinations)
        self.valid_goals_received.connect(self.update_goal_buttons)
        self.plan_received.connect(self.update_plan_points)
        self.chosen_landmark_distances_received.connect(self.update_chosen_landmark_graph)

        #Stuff to change the layout
        self.setWindowTitle("People Avoider 2000")
        self.resize(1000, 700)

        #Important Info (Top Left)
        infolayout = self.make_info_panel()

        
        #Graphs (Bottom Left)
        sectionlayout = QVBoxLayout()

        #This makes each of the graphs here
        self.graphlayout = QStackedLayout()

        self.graph0, self.person_distance_curve = self.make_line_graph(
            "Distance to Closest Person",
            y_label="Distance",
            y_units="m",
        )

        self.graph1 = self.make_bar_graph(
            "Distance to Each Landmark",
            y_label="Distance",
            y_units="m",
        )

        self.graph2, self.chosen_landmark_curve = self.make_line_graph(
            "Distance to Chosen Landmark",
            y_label="Distance",
            y_units="m",
        )
        
        self.chosen_path_curve = self.graph2.plot([], [])

        self.graph3, self.speed_curve = self.make_line_graph(
            "Speed vs Time",
            y_label="Speed",
            y_units="m/s",
        )
        self.graphlayout.addWidget(self.graph0)
        self.graphlayout.addWidget(self.graph1)
        self.graphlayout.addWidget(self.graph2)
        self.graphlayout.addWidget(self.graph3)

        self.landmark_bar_item = None
        self.person_x = deque(maxlen=200)
        self.person_y = deque(maxlen=200)

        self.chosen_x = deque(maxlen=200)
        self.chosen_y = deque(maxlen=200)

        self.speed_x = deque(maxlen=200)
        self.speed_y = deque(maxlen=200)

        self.person_sample = 0
        self.chosen_sample = 0
        self.speed_sample = 0

        self.chosen_tag = None
        
        #For path distance plot
        self.chosen_sample = 0

        self.chosen_x = deque(maxlen=200)
        self.chosen_direct_y = deque(maxlen=200)
        self.chosen_path_y = deque(maxlen=200)

        #This makes all the buttons that changes between graphs
        graphbuttonslayout = QGridLayout()

        btn = QPushButton("Distance to People")
        btn.pressed.connect(self.button0)
        graphbuttonslayout.addWidget(btn, 0, 0)
        
        btn = QPushButton("Distance to Landmarks")
        btn.pressed.connect(self.button1)
        graphbuttonslayout.addWidget(btn, 0, 1)

        btn = QPushButton("Distance from Chosen Landmark")
        btn.pressed.connect(self.button2)
        graphbuttonslayout.addWidget(btn, 1, 0)

        btn = QPushButton("Historical Speed")
        btn.pressed.connect(self.button3)
        graphbuttonslayout.addWidget(btn, 1, 1)

        #This combines the buttons and graphs in this section
        sectionlayout.addLayout(graphbuttonslayout, 1)
        sectionlayout.addLayout(self.graphlayout, 4)

        #Position Graph (Top Right)
        positionlayout = QVBoxLayout()

        self.position_graph = self.make_position_graph()
        positionlayout.addWidget(self.position_graph)

        #Command buttons (Bottom Right)
        buttonslayout = QGridLayout()

                #Command buttons (Bottom Right)
        buttonslayout = QGridLayout()

        self.button_a = QPushButton("A")
        self.button_a.pressed.connect(lambda: self.goal_requested.emit("A"))
        buttonslayout.addWidget(self.button_a, 0, 0)

        self.button_b = QPushButton("B")
        self.button_b.pressed.connect(lambda: self.goal_requested.emit("B"))
        buttonslayout.addWidget(self.button_b, 0, 1)

        self.button_c = QPushButton("C")
        self.button_c.pressed.connect(lambda: self.goal_requested.emit("C"))
        buttonslayout.addWidget(self.button_c, 0, 2)

        self.button_d = QPushButton("D")
        self.button_d.pressed.connect(lambda: self.goal_requested.emit("D"))
        buttonslayout.addWidget(self.button_d, 1, 0)

        self.button_e = QPushButton("E")
        self.button_e.pressed.connect(lambda: self.goal_requested.emit("E"))
        buttonslayout.addWidget(self.button_e, 1, 1)

        self.button_f = QPushButton("F")
        self.button_f.pressed.connect(lambda: self.goal_requested.emit("F"))
        buttonslayout.addWidget(self.button_f, 1, 2)

        #make all the buttons greyed out
        self.goal_buttons = {
            "A": self.button_a,
            "B": self.button_b,
            "C": self.button_c,
            "D": self.button_d,
            "E": self.button_e,
            "F": self.button_f,
        }
        self.update_goal_buttons([])

        btn = QPushButton("STOP")
        btn.pressed.connect(lambda: self.goal_requested.emit("STOP"))
        btn.setStyleSheet("background-color: red; color: white;")
        buttonslayout.addWidget(btn, 0, 3, 2, 1)


        #Overall Layout
        mainlayout = QHBoxLayout()
        leftlayout = QVBoxLayout()
        rightlayout = QVBoxLayout()

        leftlayout.addWidget(infolayout, 3)
        leftlayout.addLayout(sectionlayout, 4)
        rightlayout.addLayout(positionlayout, 6)
        rightlayout.addLayout(buttonslayout, 1)

        mainlayout.addLayout(leftlayout, 2)
        mainlayout.addLayout(rightlayout, 3)
        widget = QWidget()
        widget.setLayout(mainlayout)
        self.setCentralWidget(widget)


    #Info Panel in top right
    def make_info_panel(self):
        panel = QWidget()
        layout = QGridLayout(panel)

        layout.addWidget(QLabel("Battery"), 0, 0)
        self.battery_label = QLabel("-- %")
        layout.addWidget(self.battery_label, 0, 1)

        layout.addWidget(QLabel("Speed"), 1, 0)
        self.speed_label = QLabel("-- m/s")
        layout.addWidget(self.speed_label, 1, 1)

        layout.addWidget(QLabel("Angle"), 2, 0)
        self.angle_label = QLabel("--°")
        layout.addWidget(self.angle_label, 2, 1)

        layout.addWidget(QLabel("People Detected"), 3, 0)
        self.people_label = QLabel("--")
        layout.addWidget(self.people_label, 3, 1)

        layout.addWidget(QLabel("Closest Person"), 5, 0)
        self.closest_label = QLabel("--")
        layout.addWidget(self.closest_label, 5, 1)

        layout.addWidget(QLabel("Landmarks Detected"), 4, 0)
        self.landmarks_label = QLabel("--")
        layout.addWidget(self.landmarks_label, 4, 1)

        layout.addWidget(QLabel("Closest Landmark"), 6, 0)
        self.closest_landmark_label = QLabel("--")
        layout.addWidget(self.closest_landmark_label, 6, 1)
        
        return panel
    
    #Make the Graphs
    def make_line_graph(self, title, x_label="Sample", y_label="Value", y_units=None):
        graph = pg.PlotWidget()

        graph.setTitle(title)
        graph.setLabel("bottom", x_label)
        graph.setLabel("left", y_label, units=y_units)
        graph.showGrid(x=True, y=True)

        curve = graph.plot([], [])

        return graph, curve
    
    def make_bar_graph(self, title, x_label="Landmark", y_label="Value", y_units=None):
        graph = pg.PlotWidget()

        graph.setTitle(title)
        graph.setLabel("bottom", x_label)
        graph.setLabel("left", y_label, units=y_units)
        graph.showGrid(x=True, y=True)

        return graph

    # Graph Select Buttons
    def button0(self):
        self.graphlayout.setCurrentIndex(0)
    def button1(self):
        self.graphlayout.setCurrentIndex(1)
    def button2(self):
        self.graphlayout.setCurrentIndex(2)
    def button3(self):
        self.graphlayout.setCurrentIndex(3)

    def make_position_graph(self):
        graph = pg.PlotWidget()

        graph.setTitle("Local Position View")
        graph.setLabel("bottom", "X", units="m")
        graph.setLabel("left", "Y", units="m")
        graph.showGrid(x=True, y=True)
        graph.setMouseEnabled(x=False, y=False)
        graph.getViewBox().setMenuEnabled(False)
        graph.hideButtons()
        graph.setAspectLocked(True)

        # Keep robot centred for now
        graph.setXRange(-2.5, 2.5)
        graph.setYRange(-2.5, 2.5)

        # Lidar points
        self.lidar_scatter = pg.ScatterPlotItem(
            size=3,
            brush=pg.mkBrush(120, 120, 120, 160),
            pen=None,
        )
        graph.addItem(self.lidar_scatter)

        # People markers
        self.people_scatter = pg.ScatterPlotItem(
            size=12,
            brush=pg.mkBrush(255, 0, 0, 220),
            pen=pg.mkPen(255, 255, 255),
        )
        graph.addItem(self.people_scatter)

        # Goal markers
        self.goal_scatter = pg.ScatterPlotItem(
            size=14,
            brush=pg.mkBrush(0, 200, 80, 220),
            pen=pg.mkPen(255, 255, 255),
        )
        graph.addItem(self.goal_scatter)

        #the planned path
        self.plan_curve = graph.plot(
            [],
            [],
            pen=pg.mkPen(0, 120, 255, width=3),
        )

        # arrow in center
        self.turtlebot = pg.ArrowItem(
            angle=90,
            tipAngle=30,
            baseAngle=20,
            headLen=20,
            brush=pg.mkBrush(255, 0, 0),
            pen=pg.mkPen(255, 0, 0),
        )
        self.turtlebot.setPos(0, 0)
        graph.addItem(self.turtlebot)

        self.goal_labels = []
        return graph


    #Data Updates
    def update_speed(self, speed):

        #For Info Section
        self.speed_label.setText(f"{speed:.2f} m/s")

        #For Graph Section
        self.speed_sample += 1
        self.speed_x.append(self.speed_sample)
        self.speed_y.append(speed)

        self.speed_curve.setData(
            list(self.speed_x),
            list(self.speed_y),
        )

    def update_angle(self, angle_deg):
        self.angle_label.setText(f"{angle_deg:.1f}°")

    def update_battery(self, battery_percent):
        if battery_percent < 0:
            self.battery_label.setText("-- %")
        else:
            self.battery_label.setText(f"{battery_percent:.1f} %")

    def update_lidar_points(self, points):
        self.lidar_scatter.setData(
            x=[p[0] for p in points],
            y=[p[1] for p in points],
        )

    def update_plan_points(self, points):
        self.plan_curve.setData(
            [p[0] for p in points],
            [p[1] for p in points],
        )

    def update_people_count(self, count):
        self.people_label.setText(str(count))
    
    def update_landmark_count(self, count):
        self.landmarks_label.setText(str(count))

    def update_closest_person(self, distance):
        if math.isnan(distance):
            self.closest_label.setText("--")
            return

        self.closest_label.setText(f"{distance:.2f} m")

        self.person_sample += 1
        self.person_x.append(self.person_sample)
        self.person_y.append(distance)

        self.person_distance_curve.setData(
            list(self.person_x),
            list(self.person_y),
        )

    def update_closest_landmark(self, distance):
        if math.isnan(distance):
            self.closest_landmark_label.setText("--")
        else:
            self.closest_landmark_label.setText(f"{distance:.2f} m")

    def update_people_points(self, people):
        self.people_scatter.setData(
            x=[p[0] for p in people],
            y=[p[1] for p in people],
        )


    def update_chosen_landmark_graph(self, direct_distance, path_distance):
        self.chosen_sample += 1

        self.chosen_x.append(self.chosen_sample)
        self.chosen_direct_y.append(direct_distance)
        self.chosen_path_y.append(path_distance)

        self.chosen_direct_curve.setData(
            list(self.chosen_x),
            list(self.chosen_direct_y),
        )

        self.chosen_path_curve.setData(
            list(self.chosen_x),
            list(self.chosen_path_y),
        )
    def update_destinations(self, destinations):    
        self.goal_scatter.setData(
            x=[destination[2] for destination in destinations],
            y=[destination[3] for destination in destinations],
        )

        for label in self.goal_labels:
            self.position_graph.removeItem(label)

        self.goal_labels.clear()

        for name, tag, x, y in destinations:
            label = pg.TextItem(
                text=name,
                color=(0, 255, 100),
                anchor=(0.5, 1.5),
            )
            label.setPos(x, y)
            self.position_graph.addItem(label)
            self.goal_labels.append(label)

        names = []
        distances = []

        for name, tag, x, y in destinations:
            names.append(name)
            distances.append(math.sqrt(x ** 2 + y ** 2))

        x_positions = list(range(len(names)))

        if self.landmark_bar_item is not None:
            self.graph1.removeItem(self.landmark_bar_item)

        self.landmark_bar_item = pg.BarGraphItem(
            x=x_positions,
            height=distances,
            width=0.6,
        )

        self.graph1.addItem(self.landmark_bar_item)
        axis = self.graph1.getAxis("bottom")
        axis.setTicks([list(zip(x_positions, names))])

    #Makes the buttons greyed out when not found
    def update_goal_buttons(self, valid_goals):
        for name, button in self.goal_buttons.items():
            if name in valid_goals:
                button.setEnabled(True)
                button.setStyleSheet(
                    "QPushButton { background-color: white; color: black; }"
                )
            else:
                button.setEnabled(False)
                button.setStyleSheet(
                    "QPushButton { background-color: grey; color: black; }"
                )
        
class DashboardNode(Node):
    """
    ROS class.

    For publishers, subscribers, services, actions, and ROS callbacks
    """
        
    def __init__(self, window: DashboardWindow):
        super().__init__("people_avoider_dashboard")

        self.window = window
        self.robot_x = 0
        self.robot_y = 0
        self.yaw = 0
        self.global_destinations = []

        #button shit
        self.destination_poses = {}
        self.current_odom_pose = None

        self.goal_tags = {
            "A": 10,
            "B": 11,
            "C": 12,
            "D": 13,
            "E": 14,
            "F": 15,
        }

        self.tag_labels = {
            tag: label
            for label, tag in self.goal_tags.items()
        }
        
        #Distance to goal plots
        self.chosen_goal_tag = None
        self.latest_plan_points = []


        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 3)
        self.window.goal_requested.connect(self.publish_goal_pose)

        #subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )
        self.battery_sub = self.create_subscription(
            BatteryState,
            "/battery_state",
            self.battery_callback,
            10,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.people_sub = self.create_subscription(
            PoseArray,
            "/fusion/people",
            self.people_callback,
            10,
        )
        self.destinations_sub = self.create_subscription(
            DestinationListMsg,
            "/destination_advertiser/list",
            self.destinations_callback,
            10,
        )
        self.plan_sub = self.create_subscription(
            Path,
            "/plan",
            self.plan_callback,
            10,
        )
        

    def odom_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.sqrt(vx ** 2 + vy ** 2)

        q = msg.pose.pose.orientation

        # Quaternion to yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        angle_deg = math.degrees(yaw)
        self.yaw = yaw
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        #need for stop button
        self.current_odom_pose = msg.pose.pose

        self.window.speed_received.emit(speed)
        self.window.angle_received.emit(angle_deg)
        self.convert_global_destinations_to_graph()

    def battery_callback(self, msg):
        self.window.battery_received.emit(msg.percentage)

    def scan_callback(self, msg):
        points = []

        angle = msg.angle_min

        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                ros_x = r * math.cos(angle)  # forward
                ros_y = r * math.sin(angle)  # left

                graph_x = -ros_y
                graph_y = ros_x

                points.append((graph_x, graph_y))

            angle += msg.angle_increment

        self.window.lidar_received.emit(points)

    def people_callback(self, msg):
        people = []

        for pose in msg.poses:
            ros_x = pose.position.x
            ros_y = pose.position.y

            graph_x = -ros_y
            graph_y = ros_x

            people.append((graph_x, graph_y))

        self.window.people_received.emit(people)
        self.window.people_count_received.emit(len(people))

        if people:
            closest = min(math.sqrt(x ** 2 + y ** 2) for x, y in people)
        else:
            closest = float("nan")

        self.window.closest_person_received.emit(closest)

    def destinations_callback(self, msg):
        self.global_destinations = []
        self.destination_poses = {}
        

        for destination in msg.destinations:
            name = destination.name
            tag = int(destination.tag)
            label = self.tag_labels.get(tag, name)
            x_global = destination.pose.position.x
            y_global = destination.pose.position.y
            self.destination_poses[tag] = destination.pose
            self.global_destinations.append((label, name, x_global, y_global))
            
        self.convert_global_destinations_to_graph()

        self.window.landmark_count_recieved.emit(len(self.destination_poses))

        valid_goals = []

        for goal_name, tag in self.goal_tags.items():
            if tag in self.destination_poses:
                valid_goals.append(goal_name)

        self.window.valid_goals_received.emit(valid_goals)

    def plan_callback(self, msg):
        points = []
        self.latest_plan_points = []

        for pose_stamped in msg.poses:
            x_global = pose_stamped.pose.position.x
            y_global = pose_stamped.pose.position.y
            self.latest_plan_points.append((x_global, y_global))

            dx = x_global - self.robot_x
            dy = y_global - self.robot_y

            forward = math.cos(self.yaw) * dx + math.sin(self.yaw) * dy
            left = -math.sin(self.yaw) * dx + math.cos(self.yaw) * dy

            graph_x = -left
            graph_y = forward

            points.append((graph_x, graph_y))

        self.window.plan_received.emit(points)
        self.update_chosen_landmark_distances()


    def convert_global_destinations_to_graph(self):
        destinations = []

        for label, name, x_global, y_global in self.global_destinations:
            dx = x_global - self.robot_x
            dy = y_global - self.robot_y

            forward = math.cos(self.yaw) * dx + math.sin(self.yaw) * dy
            left = -math.sin(self.yaw) * dx + math.cos(self.yaw) * dy

            graph_x = -left
            graph_y = forward

            destinations.append((label, name, graph_x, graph_y))

        if destinations:
            closest = min(math.sqrt(x ** 2 + y ** 2) for _, _, x, y in destinations)
        else:
            closest = float("nan")

        self.window.closest_landmark_received.emit(closest)

        self.window.destinations_received.emit(destinations)

        
    def publish_goal_pose(self, goal_name):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "odom"
        goal_msg.header.stamp = self.get_clock().now().to_msg()

        if goal_name == "STOP":
            goal_msg.pose = self.current_odom_pose
        else:
            tag = self.goal_tags[goal_name]
            if tag not in self.destination_poses:
                return

            self.chosen_goal_tag = tag
            goal_msg.pose = self.destination_poses[tag]

        self.goal_pub.publish(goal_msg)

    def update_chosen_landmark_distances(self):
        if self.chosen_goal_tag is None:
            return

        if self.chosen_goal_tag not in self.destination_poses:
            return

        goal_pose = self.destination_poses[self.chosen_goal_tag]

        dx = goal_pose.position.x - self.robot_x
        dy = goal_pose.position.y - self.robot_y

        direct_distance = math.sqrt(dx ** 2 + dy ** 2)
        path_distance = self.get_remaining_path_distance()

        self.window.chosen_landmark_distances_received.emit(
            direct_distance,
            path_distance,
        )

    def get_remaining_path_distance(self):
        if not self.latest_plan_points:
            return float("nan")

        closest_index = 0
        closest_distance = float("inf")

        for i, point in enumerate(self.latest_plan_points):
            x, y = point
            distance = math.sqrt(
                (x - self.robot_x) ** 2 +
                (y - self.robot_y) ** 2
            )

            if distance < closest_distance:
                closest_distance = distance
                closest_index = i

        path_distance = closest_distance

        for i in range(closest_index, len(self.latest_plan_points) - 1):
            x1, y1 = self.latest_plan_points[i]
            x2, y2 = self.latest_plan_points[i + 1]

            path_distance += math.sqrt(
                (x2 - x1) ** 2 +
                (y2 - y1) ** 2
            )

        return path_distance
    
def main():
    rclpy.init()

    app = QApplication(sys.argv)

    window = DashboardWindow()
    ros_node = DashboardNode(window)

    ros_thread = threading.Thread(
        target=rclpy.spin,
        args=(ros_node,),
        daemon=True,
    )
    ros_thread.start()

    window.show()
    exit_code = app.exec()

    ros_node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()