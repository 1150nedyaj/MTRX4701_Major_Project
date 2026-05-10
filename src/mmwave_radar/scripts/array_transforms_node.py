#!/usr/bin/env python3

import math
import sys
import os
import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import TransformStamped

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster



def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class RadarArrayStaticFramePublisher(Node):
    """
    Broadcast Pose data for mmwave radar modules over TF2
    """
    def __init__(self, array_config_path):
        super().__init__('static_turtle_tf2_broadcaster')

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        with open(array_config_path, 'r') as f:
            config = yaml.safe_load(f)

        # publish radar transforms once at start
        modules = config['launch_settings']['radar_modules']
        for m in modules:
            transformation = {}

            transformation['parent_name'] = m['parent_frame']
            transformation['name'] = f"radar{m['identifier']}"

            transformation['t_x'] = m['translation_x']
            transformation['t_y'] = m['translation_y']
            transformation['t_z'] = m['translation_z']

            transformation['roll'] = m['roll']
            transformation['pitch'] = m['pitch']
            transformation['yaw'] = m['yaw']

            self.make_transforms(transformation)

    def make_transforms(self, transformation):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = transformation['parent_name']
        t.child_frame_id = transformation['name']

        t.transform.translation.x = float(transformation['t_x'])
        t.transform.translation.y = float(transformation['t_y'])
        t.transform.translation.z = float(transformation['t_z'])
        quat = quaternion_from_euler(
            float(transformation['roll']), float(transformation['pitch']), float(transformation['yaw']))
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_static_broadcaster.sendTransform(t)



def main():
    try:
        print(sys.argv)

        if sys.argv[1].split('.')[-1] != 'yaml':
            print('Invalid config file type (must be .yaml), Usage: \n'
                        '$ ros2 run mmwave_radar arrray_transforms_node '
                        '<path-to-yaml-array-config-file>')
            sys.exit(1)

        supplied_config_path = sys.argv[1]
        if not os.path.isfile(supplied_config_path):
            print(f"Array configuration file {supplied_config_path} does not exist.")
            sys.exit(1)

        # pass parameters and initialize node
        rclpy.init()
        node = RadarArrayStaticFramePublisher(supplied_config_path)
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()