import serial
import time

class ODriveMicro:
    def __init__(self, port, baudrate=115200, node_id=0x3f):
        """
        Initialize ODrive CAN interface via UART-to-CAN adapter.
        :param port: Serial port to which the UART-to-CAN adapter is connected (e.g., '/dev/ttyUSB0' or 'COM3').
        :param baudrate: Baud rate for the serial communication (default: 115200).
        :param node_id: ODrive CAN node ID (default: 0x3F).
        """
        self.serial = serial.Serial(port, baudrate, timeout=1)
        self.node_id = node_id
        self.rtr = 1
    
    def send_can_frame(self, cmd_id, data):
        """
        Send a CAN frame via the UART-to-CAN adapter.
        :param arbitration_id: CAN arbitration ID (11-bit).
        :param data: Data payload as bytes.
        """