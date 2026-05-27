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

from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, LaserScan
from geometry_msgs.msg import PoseArray


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
    closest_person_received = pyqtSignal(float)

    def __init__(self):
        super().__init__()

        #All the things that are updated
        self.speed_received.connect(self.update_speed)
        self.angle_received.connect(self.update_angle)
        self.battery_received.connect(self.update_battery)
        self.lidar_received.connect(self.update_lidar_points)
        self.people_received.connect(self.update_people_points)
        self.people_count_received.connect(self.update_people_count)
        self.closest_person_received.connect(self.update_closest_person)


        #Stuff to change the layout
        self.setWindowTitle("People Avoider 2000")
        self.resize(1000, 700)

        #Important Info (Top Left)
        infolayout = self.make_info_panel()

        
        #Graphs (Bottom Left)
        sectionlayout = QVBoxLayout()

        #This makes each of the graphs here
        self.graphlayout = QStackedLayout()

        self.graph0, self.speed_curve = self.make_line_graph("Title",y_label="Distance",y_units="m",)
        self.graph1, self.person_curve = self.make_line_graph("hfdgh",y_label="Distance",y_units="m",)
        self.graph2, self.goal_curve = self.make_line_graph("Titgfhfgshle",y_label="Distance",y_units="m",)
        self.graph3, self.goal_curve = self.make_line_graph("srths",y_label="Distance",y_units="m",)
        self.graphlayout.addWidget(self.graph0)
        self.graphlayout.addWidget(self.graph1)
        self.graphlayout.addWidget(self.graph2)
        self.graphlayout.addWidget(self.graph3)

        #This makes all the buttons that changes between graphs
        graphbuttonslayout = QGridLayout()

        btn = QPushButton("this and that")
        btn.pressed.connect(self.button0)
        graphbuttonslayout.addWidget(btn, 0, 0)
        
        btn = QPushButton("this and that")
        btn.pressed.connect(self.button1)
        graphbuttonslayout.addWidget(btn, 0, 1)

        btn = QPushButton("this and that")
        btn.pressed.connect(self.button2)
        graphbuttonslayout.addWidget(btn, 1, 0)

        btn = QPushButton("this and that")
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

        buttonslayout.addWidget(Color("orange"), 0, 0)
        buttonslayout.addWidget(Color("purple"), 0, 1)
        buttonslayout.addWidget(Color("orange"), 0, 2)
        buttonslayout.addWidget(Color("purple"), 0, 3)
        buttonslayout.addWidget(Color("orange"), 1, 0)
        buttonslayout.addWidget(Color("purple"), 1, 1)
        buttonslayout.addWidget(Color("orange"), 1, 2)
        buttonslayout.addWidget(Color("purple"), 1, 3)

        #Overall Layout
        mainlayout = QHBoxLayout()
        leftlayout = QVBoxLayout()
        rightlayout = QVBoxLayout()

        leftlayout.addWidget(infolayout, 3)
        leftlayout.addLayout(sectionlayout, 4)
        rightlayout.addLayout(positionlayout, 3)
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

        layout.addWidget(QLabel("Closest Person"), 4, 0)
        self.closest_label = QLabel("--")
        layout.addWidget(self.closest_label, 4, 1)
        
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
        graph.setXRange(-2, 2)
        graph.setYRange(-2, 2)

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
        self.speed_label.setText(f"{speed:.2f} m/s")

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

    def update_people_count(self, count):
        self.people_label.setText(str(count))


    def update_closest_person(self, distance):
        if math.isnan(distance):
            self.closest_label.setText("--")
        else:
            self.closest_label.setText(f"{distance:.2f} m")
    def update_people_points(self, people):
        self.people_scatter.setData(
            x=[p[0] for p in people],
            y=[p[1] for p in people],
        )

class DashboardNode(Node):
    """
    ROS class.

    For publishers, subscribers, services, actions, and ROS callbacks
    """
        
    def __init__(self, window: DashboardWindow):
        super().__init__("people_avoider_dashboard")

        self.window = window

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

        self.window.speed_received.emit(speed)
        self.window.angle_received.emit(angle_deg)

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