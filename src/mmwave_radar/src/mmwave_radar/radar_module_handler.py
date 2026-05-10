from serial import Serial
import serial.tools.list_ports
from threading import Lock
import binascii
import time
import struct
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from mmwave_radar.types import RadarFrame

class RadarModuleHandler(object):
    def __init__(self, node: Node, interface: str):
        self._node = node
        self._Lock = Lock()

        ## Vars for pulling data from hex report frame
        self._buf = bytearray()
        self.frame_header = b'\xF4\xF3\xF2\xF1'
        self.frame_tail = b'\xF8\xF7\xF6\xF5'
        self.frame_payload_len = 35     # 1 detect + 2 distance + 32 (16 gates w/ 2 each)
        self.frame_len = 4 + 2 + self.frame_payload_len + 4     # why the extra 2?
        # Little-endian: '<', then uint16 length, uint8 detect, uint16 distance, 16x uint16 energies
        self.frame_struct = struct.Struct('<HBH16H')

        ## Check interface is good
        if not self._serial_interface_up(interface):
            self._node.get_logger().error(f"Can't find {interface}!")
            raise RuntimeError(f"Interface {interface} is not up.")
        else:
            self._node.get_logger().info(f"Interface {interface} is up!")
        self._interface = interface

        ## Setup port and bringup radar
        self.serial_port = Serial(self._interface, 115200, timeout=1)
        startup_hex  = "FDFCFBFA0800120000000400000004030201"   # Report MODE
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
            self._node.get_logger().info("Availiable Interfaces... ")

            for i in port_names:
                self._node.get_logger().info(f"\t-> {i}")
        return False
    
    def _parse_frame(self, payload: bytes) -> RadarFrame:
        """Unpack a 39-byte slice: length(2) + detect(1) + distance(2) + energies(32)."""
        length, detect, distance, *energies = self.frame_struct.unpack(payload)

        if length != self.frame_payload_len:
            raise ValueError(f"unexpected payload length {length}, expected {self.frame_payload_len}")
        
        return RadarFrame(
            present=bool(detect),
            distance=distance,
            gate_energies=tuple(energies),
        )

    def _find_frames(self, buf: bytearray):
        while True:
            idx = buf.find(self.frame_header)

            if idx < 0:
                # No header found, drop everything
                del buf[:]
                return
            
            if idx > 0:
                # Junk before header can be discarded
                del buf[:idx]

            if len(buf) < self.frame_len:
                # Need a full frame
                return
            
            if bytes(buf[self.frame_len - 4:self.frame_len]) != self.frame_tail:
                # Tail isn't where it should be -> corrupted frame
                del buf[:4]     # skip past header and keep looking
                continue

            # Frame's legit, unpack meaningful bytes
            try:
                frame = self._parse_frame(bytes(buf[4:self.frame_len - 4]))
            except ValueError:
                # issue reading -> corrupted -> skip to next
                del buf[:4]
                continue

            del buf[:self.frame_len]
            return frame


    def read_radar_data(self):
        with self._Lock:
                # populate buffer
                n = self.serial_port.in_waiting
                if n:
                    self._buf.extend(self.serial_port.read(n))

                # Pull all the complete frames from the buffer, keeping the last one
                latest = self._find_frames(self._buf)
                if latest == None:
                    self._node.get_logger().warning("Failed to get frame!")

                return latest

            
            
            

        


    




