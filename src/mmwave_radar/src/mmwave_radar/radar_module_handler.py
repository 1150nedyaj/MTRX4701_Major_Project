"""
MTRX4701 2026 Assignment 4
File: radar_module_handler.py
Author(s): 530 499 451

Manages RD03D mmWave Radar Modules.
Takes the parsed output from the module and does some
filtering on the detections to get rid of known garbage readings.
"""

import serial.tools.list_ports
from threading import Lock
import time

from rclpy.node import Node

from mmwave_radar.rd03d import RD03D
from mmwave_radar.types import RadarSignature


class RadarModuleHandler(object):
    def __init__(self, node: Node, interface: str):
        self._node = node
        self._Lock = Lock()

        # interface is good?
        if not self._serial_interface_up(interface):
            self._node.get_logger().error(f"Can't find {interface}!")
            raise RuntimeError(f"Interface {interface} is not up.")
        else:
            self._node.get_logger().info(f"Interface {interface} is up!")
        self._interface = interface

        self.radar = RD03D(uart_port=interface)
    
    def get_signatures(self):
        """
        Public method to get radar's detections (that are worth seeing)
        """
        if self.radar.update():
            targets = [self.radar.get_target(n) for n in range(1,4)]
            detections = [t for t in targets if t.detection == True]
        else:
            self._node.get_logger().warning("Radar returned no detections!")
            return []
        
        # skip if the reading looks like a known false positive
        signatures = []
        for d in detections:
            # was getting false positives at the edges
            if abs(d.angle) > 45:
                continue
            
            if d.distance < 400:
                continue

            # annoying little fp at front
            if abs(d.angle) > 40 and d.distance < 400:
                continue

            # all tests passed; bring it through
            signatures.append(RadarSignature.from_RD03DMessage(d))

        return signatures
    
    def _serial_interface_up(self, interface):
        ports = serial.tools.list_ports.comports()
        port_names = [port for port, desc, hwid in ports]

        if interface in port_names:
            return True
        else:
            self._node.get_logger().info("Availiable Interfaces... ")

            for i in port_names:
                self._node.get_logger().info(f"\t-> {i}")
        return False

    @staticmethod
    def millis():
        return int(time.time() * 1000)




            
            
            

        


    




