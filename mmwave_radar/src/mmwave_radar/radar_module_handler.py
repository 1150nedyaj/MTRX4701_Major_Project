from serial import Serial
import serial.tools.list_ports
from threading import Lock
import binascii
import os
import time

import rclpy
from rclpy.node import Node

class RadarModuleHandler(object):
    def __init__(self, node: Node, interface: str):
        self._node = node
        self.serial_timeout = 1000 # millis
        self._Lock = Lock()

        ## Check interface is good
        if not self._serial_interface_up(interface):
            raise RuntimeError(f"Interface {interface} is not up.")
        else:
            self._node.get_logger().info(f"Interface {interface} is up!")
        self._interface = interface

        ## Setup port and bringup radar
        self.serial_port = Serial(self._interface, 115200, timeout=1)
        startup_hex  = "FDFCFBFA0800120000006400000004030201"
        hex_bytes = binascii.unhexlify(startup_hex)
        self.serial_port.write(hex_bytes)
    
    @staticmethod
    def millis():
        return int(time.time() * 1000)

    def _serial_interface_up(self, interface):
        ports = serial.tools.list_ports.comports()
        port_names = [port for port, desc, hwid in ports]

        if interface in port_names:
            return True
        else:
            print("Availiable Interfaces... ")
            for i in port_names:
                print("\t-> ", i)
        return False
        


    def read_radar_data(self):
        with self._Lock:
            self.serial_port.reset_input_buffer()

            # startup_hex  = "FDFCFBFA0800120000006400000004030201"
            # hex_bytes = binascii.unhexlify(startup_hex)
            # self.serial_port.write(hex_bytes)
            time.sleep(0.1)
            message_str = self.serial_port.readline().decode('utf-8', errors='ignore').strip()

            first_attempt = self.millis()
            success = False
            output = -1

            while not success:
                print(f"Message : [{message_str}]")

                if len(message_str.split(" ")) == 2:
                    # print(message_str.split(" "))
                    output = int(message_str.split(" ")[1])
                    success = True
                else:
                    
                    message_str = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    print('new message -> ', message_str," ##  output -> ", int(message_str.split(" ")[1]))

                if self.millis() - first_attempt > self.serial_timeout:
                    break
            
            return output
            
            

        


    




