import can
import time
import struct

get_version             =   0x000
heartbeat               =   0x001
estop                   =   0x002
get_Error               =   0x003
rxsdo                   =   0x004
txsdo                   =   0x005
address                 =   0x006
set_axis_state          =   0x007
get_encoder_estimate    =   0x009
set_controller_mode     =   0x00b
set_input_pos           =   0x00c
set_input_vel           =   0x00d
set_input_torque        =   0x00e
set_limit               =   0x00f
set_traj_vel_limit      =   0x011
set_traj_accel_limit    =   0x012
set_traj_inertia        =   0x013
get_lq                  =   0x014
get_temperature         =   0x015
reboot                  =   0x016
get_bus_voltage_current =   0x017
clear_errors            =   0x018
set_absolute_position   =   0x019
set_pos_gain            =   0x01a
set_vel_gain            =   0x01b
get_torque              =   0x01c
get_power               =   0x01d
enter_dfu_mode          =   0x01f

class ODriveMicro:

    def __init__(self, channel, node_id=0x3f, timeout = 0.1):
        """
        Initialize ODrive CAN interface via UART-to-CAN adapter.
        :param port: Serial port to which the UART-to-CAN adapter is connected (e.g., '/dev/ttyUSB0' or 'COM3').
        :param baudrate: Baud rate for the serial communication (default: 115200).
        :param node_id: ODrive CAN node ID (default: 0x3F).
        """
        self.bus = can.interface.Bus(interface="socketcan", channel=channel)
        self.node_id = node_id & 0x3f
        self.timeout = timeout

        self.flush_rx()
    
    def flush_rx(self):
        while not (self.bus.recv(timeout=0) is None): pass
    
    def send_can_frame(self, cmd_id, payload):
        """
        Send a CAN frame via the UART-to-CAN adapter.
        :param arbitration_id: CAN arbitration ID (11-bit).
        :param data: Data payload as bytes.
        """
        if type(payload) is list:
            msg = can.Message(
                arbitration_id= (self.node_id << 5) | (cmd_id & 0x1f),
                data= payload,
                is_extended_id=False
                )
            try:
                self.bus.send(msg)
                print(f"Message sent on {self.bus.channel_info}")
                time.sleep(0.01)
            except:
                print("Message NOT sent")
        else:
            print("The payload type is not list, Please makes sure that you input the list of data")
    
    def get_can_frame(self):
        """
        Receive the response from CAN Bus
        """
        return self.bus.recv(self.timeout)
    

    def disp_version(self):
        """
        Display the Odrive hardware version.
        Start Byte              Name                Type                Description
        0               Protocol_Version            uint8           Always reported as 2
        1               Hw_Version_Major            uint8           hw_version_major
        2               Hw_Version_Minor            uint8           hw_version_minor
        3               Hw_Version_Variant          uint8           hw_version_variant
        4               Fw_Version_Major            uint8           fw_version_major
        5               Fw_Version_Minor            uint8           fw_version_minor
        6               Fw_Version_Revision         uint8           fw_version_revision
        7               Fw_Version_Unreleased       uint8           fw_version_unreleased
        """

        self.send_can_frame(cmd_id=get_version, payload=b'')
        # Await reply
        for msg in self.bus:
            if msg.arbitration_id == (self.node_id << 5 | get_version):
                break
        _, hw_product_line, hw_version, hw_variant, fw_major, fw_minor, fw_revision, fw_unreleased = struct.unpack('<BBBBBBBB', msg.data)

        print(f"Hardware version : {hw_product_line}.{hw_version}.{hw_variant}.\n")
        print(f"Firmware version : {fw_major}.{fw_minor}.{fw_revision}.\n")

    def get_heartbeat(self):
        self.send_can_frame(cmd_id=heartbeat, payload=b'')
        #Await reply
        msg = self.get_can_frame()
        if msg.arbitration_id == (self.node_id << 5 | heartbeat):
            axis_error, axis_state, procedure_res, traj_done_flag = struct.unpack('<LBBB', msg.data)
            print(f"axis error is {axis_error}.     axis_state is {axis_state}.      procedure result is {procedure_res}.        trajectory done flag is {traj_done_flag}.\n")
            return axis_error, axis_state, procedure_res, traj_done_flag
        else:
            print("Arbitration ID is not match to the dataframe. Cannot communicate to Odrive.")
            return None