#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import Joy
from exomy.msg import RoverCommand
from locomotion_modes import LocomotionMode
import serial
import math
import time

# UART configuration for RC control
sbus_port = serial.Serial(
    port='/dev/ttyAMA2',
    baudrate=100000,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_EVEN,
    stopbits=serial.STOPBITS_TWO,
    timeout=0.005  # Short timeout for non-blocking read
)

# Global variables for locomotion modes and motor states
locomotion_mode = LocomotionMode.ACKERMANN.value
motors_enabled = True
sbus_buffer = bytearray()  # Buffer for SBUS data

# SBUS Protocol Constants
SBUS_HEADER_BYTE = b'\x0F'
SBUS_PACKET_LENGTH = 25

def read_sbus_packet():
    while True:
        if sbus_port.read() == b'\x0F':  # Look for the SBUS header byte
            packet = sbus_port.read(24)
            if len(packet) == 24:
                return b'\x0F' + packet


def decode_sbus_packet(packet):
    channels = []

    # Extract 16 channels, 11 bits each
    channels.append((packet[1] | packet[2] << 8) & 0x07FF)
    channels.append((packet[2] >> 3 | packet[3] << 5) & 0x07FF)
    channels.append((packet[3] >> 6 | packet[4] << 2 | packet[5] << 10) & 0x07FF)
    channels.append((packet[5] >> 1 | packet[6] << 7) & 0x07FF)
    channels.append((packet[6] >> 4 | packet[7] << 4) & 0x07FF)
    channels.append((packet[7] >> 7 | packet[8] << 1 | packet[9] << 9) & 0x07FF)
    channels.append((packet[9] >> 2 | packet[10] << 6) & 0x07FF)
    channels.append((packet[10] >> 5 | packet[11] << 3) & 0x07FF)
    channels.append((packet[12] | packet[13] << 8) & 0x07FF)
    channels.append((packet[13] >> 3 | packet[14] << 5) & 0x07FF)
    channels.append((packet[14] >> 6 | packet[15] << 2 | packet[16] << 10) & 0x07FF)
    channels.append((packet[16] >> 1 | packet[17] << 7) & 0x07FF)
    channels.append((packet[17] >> 4 | packet[18] << 4) & 0x07FF)
    channels.append((packet[18] >> 7 | packet[19] << 1 | packet[20] << 9) & 0x07FF)
    channels.append((packet[20] >> 2 | packet[21] << 6) & 0x07FF)
    channels.append((packet[21] >> 5 | packet[22] << 3) & 0x07FF)

    # Status flags
    channel17 = packet[23] & 0x01
    channel18 = packet[23] & 0x02
    frameLost = packet[23] & 0x04
    failsafe = packet[23] & 0x08
    return channels, channel17, channel18, frameLost, failsafe


def normalize_to_neg1_pos1(value, min_value=172, max_value=1811):
    """Normalize input value to range [-1, 1]."""
    return 2 * (value - min_value) / (max_value - min_value) - 1

def handle_rc_input():
    """Handle SBUS RC input."""
    global locomotion_mode, motors_enabled

    packet = read_sbus_packet()
    if packet:
        channels, channel17, channel18, frameLost, failsafe = decode_sbus_packet(packet)
        controller_funtion_map = {
            "x_axis": channels[3],  # Map channel 3 to x_axis
            "y_axis": channels[2],  # Map channel 2 to y_axis
            "mode_switch": channels[5],  # Mode switch on channel 5
            "motor_switch": channels[6]  # Motor enable/disable switch
        }

        x = normalize_to_neg1_pos1(controller_funtion_map["x_axis"])
        y = normalize_to_neg1_pos1(controller_funtion_map["y_axis"])

        # Locomotion mode switching
        if controller_funtion_map["mode_switch"] == 1811:
            locomotion_mode = LocomotionMode.POINT_TURN.value
        elif controller_funtion_map["mode_switch"] == 992:
            locomotion_mode = LocomotionMode.CRABBING.value
        elif controller_funtion_map["mode_switch"] == 172:
            locomotion_mode = LocomotionMode.ACKERMANN.value

        rover_cmd = RoverCommand()
        rover_cmd.locomotion_mode = locomotion_mode

        # Motor enable/disable
        if controller_funtion_map["motor_switch"] == 172 and motors_enabled:
            motors_enabled = False
            rospy.loginfo("Motors Disabled!")
        elif controller_funtion_map["motor_switch"] == 1811 and not motors_enabled:
            motors_enabled = True
            rospy.loginfo("Motors Enabled!")

        rover_cmd.motors_enabled = motors_enabled

        # Calculate velocity and steering
        if locomotion_mode == LocomotionMode.CRABBING.value:
            rover_cmd.vel = int(abs(y) * 100)
            rover_cmd.steering = int(math.copysign((x * 90) + 90,y))
            if rover_cmd.steering == 0:
                rover_cmd.steering = int(math.copysign(1,y))
        else:
            rover_cmd.vel = int(y * 100)
            rover_cmd.steering = int((x * 90) + 90)

        rover_cmd.connected = True

        # Publish the rover command
        pub.publish(rover_cmd)

def callback(data):

    global locomotion_mode
    global motors_enabled

    rover_cmd = RoverCommand()

    # Function map for joystick
    # Button on pad | function
    # --------------|----------------------
    # A             | Ackermann mode
    # X             | Point turn mode
    # Y             | Crabbing mode
    # Left Stick    | Control speed and direction
    # START Button  | Enable and disable motors

    # More info on mapping: https://wiki.ros.org/joy
    if data.header.frame_id == "webgui":
        # Logitech WebGUI
        controller_funtion_map = {
            "x_axis": 0,
            "y_axis": 1,
            "invert_x_axis": True,
            "X_button": 0,
            "Y_button": 3,
            "A_button": 1,
            "B_button": 2,
            "start_button": 9,
            "select_button": 8,
            "sensitivity": 0.15
        }
    elif rospy.get_param("controller") == "logitech-F710": 
        # Logitech F710 joystick
        controller_funtion_map = {
            "x_axis": 0,
            "y_axis": 1,
            "invert_x_axis": True,
            "X_button": 0,
            "Y_button": 3,
            "A_button": 1,
            "B_button": 2,
            "start_button": 9,
            "select_button": 8,
            "sensitivity": 0.1
        }
    elif rospy.get_param("controller") == "xbox-one": 
        #X-Box One controller
        controller_funtion_map = {
            "x_axis": 0,
            "y_axis": 1,
            "invert_x_axis": False,
            "X_button": 2,
            "Y_button": 3,
            "A_button": 0,
            "B_button": 1,
            "start_button": 7,
            "select_button": 6,
            "sensitivity": 0.11
        }
    else:
        rospy.logerr("No controller identified. Using fallback.")
        controller_funtion_map = {
            "x_axis": 0,
            "y_axis": 1,
            "invert_x_axis": True,
            "X_button": 0,
            "Y_button": 3,
            "A_button": 1,
            "B_button": 2,
            "start_button": 9,
            "select_button": 8,
            "sensitivity": 0.0
        }
   
    # Reading out joystick data
    y = data.axes[1]
    x = data.axes[0]

    if controller_funtion_map["invert_x_axis"] == True:
        x = x * -1

    # Reading out button data to set locomotion mode
    # X Button
    if (data.buttons[controller_funtion_map["X_button"]] == 1):
        locomotion_mode = LocomotionMode.POINT_TURN.value
    # A Button
    if (data.buttons[controller_funtion_map["A_button"]] == 1):
        locomotion_mode = LocomotionMode.ACKERMANN.value
    # B Button
    if (data.buttons[controller_funtion_map["B_button"]] == 1):
        pass
    # Y Button
    if (data.buttons[controller_funtion_map["Y_button"]] == 1):
        locomotion_mode = LocomotionMode.CRABBING.value

    rover_cmd.locomotion_mode = locomotion_mode

    # Enable and disable motors
    # START Button
    if (data.buttons[controller_funtion_map["start_button"]] == 1):
        if motors_enabled is True:
            motors_enabled = False
            rospy.loginfo("Motors disabled!")
            # Set a sleep timer, if not a button movement could be triggered falsely
            time.sleep(0.5)
        elif motors_enabled is False:
            motors_enabled = True
            rospy.loginfo("Motors enabled!")
            # Set a sleep timer, if not a button movement could be triggered falsely
            time.sleep(0.5)
        else:
            rospy.logerr(
                "Exceptional value for [motors_enabled] = {}".format(motors_enabled))
            motors_enabled = False

    rover_cmd.motors_enabled = motors_enabled

    # The velocity is decoded as value between 0...100
    # Sensitivity defines an area in the center, where no steering or speed commands are send to allow for lower speeds without loosing directions
    # Similar to "joy_node"-"deadzone"-parameter. The parameter is not touched, as every controller has its own sensitivity
    rover_cmd.vel = min(math.sqrt(x*x + y*y), 1.0)
    if rover_cmd.vel < controller_funtion_map["sensitivity"]:
        rover_cmd.vel = 0
    else:
        rover_cmd.vel = (rover_cmd.vel - controller_funtion_map["sensitivity"]) * (1 / (1 - controller_funtion_map["sensitivity"])) * 100

    # The steering is described as an angle between -180...180
    # Which describe the joystick position as follows:
    #   +90
    # 0      +-180
    #   -90
    #
    rover_cmd.steering = math.atan2(y, x)*180.0/math.pi
    rover_cmd.connected = True

    rover_cmd.vel = int(rover_cmd.vel)
    rover_cmd.steering = int(rover_cmd.steering)

    pub.publish(rover_cmd)


if __name__ == '__main__':
    global pub
    print("here")
    rospy.init_node('joystick_parser_node')
    rospy.loginfo('joystick_parser_node started')

    pub = rospy.Publisher('/rover_command', RoverCommand, queue_size=1)
    if rospy.get_param("controller") == "rc":
        # Timer for reading SBUS data
        #sbus_timer = rospy.Timer(rospy.Duration(0.02), lambda event: handle_rc_input())
        while not rospy.is_shutdown():
            handle_rc_input()
            #rospy.sleep(0.01)
    else:
        # Subscriber for joystick
        sub = rospy.Subscriber("/joy", Joy, callback, queue_size=1)
    rospy.spin()
