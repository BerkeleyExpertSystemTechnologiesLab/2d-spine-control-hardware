#!/usr/bin/python

# NOTE: on Ubuntu 18.04, specifying /usr/bin/env python uses python 3,
# while specifying /usr/bin/python uses python 2.7. We need 2.7.

# serial_tx_gait_commands writes Belka gait commands to the two PSoCs.

# Usage is:
#	rosrun belka_gait_commander serial_tx_gait_commands USB_DEVICE_HIPS USB_DEVICE_SHOULDERS

# ...where USB_DEVICE_HIPS and USB_DEVICE_SHOULDERS are the /dev/ttyACM0, /dev/ttyACM1, ... etc.

# Imports:
import rospy
# because we need the command-line arguments
import sys
# and for the serial/usb/uart via pyserial:
import serial
# need to manipulate arrays
# import numpy as np

# We'll use two messages, both strings, to differentiate what gets sent to hips vs. shoulders
from belka_gait_commander.msg import HipsGaitCommand
from belka_gait_commander.msg import ShouldersGaitCommand
# we'll need to parse the string itself too?
from std_msgs.msg import String

# Note: in order to do publishing within a callback, we need to pass around
# the publisher object, which is done much easier by making this a class.


class SerialTxGaitCommands:

    # The callback functions here do all the heavy lifting.

    def serial_tx_callback_hips(self, message):
        # When a message is received:
        # 1) format the string to send to the PSoC
        # 2) actually send the message

        # Now using our own message type: (this is just a string)
        hips_command = message.hips_command

        # 1) add on the required newline
        hips_command = hips_command + "\r\n"

        # We seem to be getting some mis-aligned commands.
        # So, before anything else, send out a "clear" every time.
        self.serial_port_hips.write("\n")
        clear_delay = 0.05
        # give the PSoC a moment
        # maybe 20 ms?
        rospy.sleep(clear_delay)
        self.serial_port_hips.write("c\n")
        rospy.sleep(clear_delay)
        self.serial_port_hips.write("c\n")
        rospy.sleep(clear_delay)

        # Send the message
        self.serial_port_hips.write(hips_command)
        # send debugging back to terminal
        print("Wrote hips command: " + message.hips_command)

    def serial_tx_callback_shoulders(self, message):
        # When a message is received:
        # 1) format the string to send to the PSoC
        # 2) actually send the message

        # Now using our own message type: (this is just a string)
        shoulders_command = message.shoulders_command

        # 1) add on the required newline
        shoulders_command = shoulders_command + "\r\n"

        # We seem to be getting some mis-aligned commands.
        # So, before anything else, send out a "clear" every time.
        self.serial_port_shoulders.write("\n")
        clear_delay = 0.05
        # give the PSoC a moment
        # maybe 20 ms?
        rospy.sleep(clear_delay)
        self.serial_port_shoulders.write("c\n")
        rospy.sleep(clear_delay)
        self.serial_port_shoulders.write("c\n")
        rospy.sleep(clear_delay)

        # Send the message
        self.serial_port_shoulders.write(shoulders_command)
        # send debugging back to the terminal
        print("Wrote shoulders command: " + message.shoulders_command)

    # The primary helper function here opens the serial device,
    # subscribes to a topic, writes when new data appears on the topic, and
    # echoes (publishes) its pushed data back out on *another* topic for debugging.
    def serial_tx_startup(self, device_name_hips, device_name_shoulders):
        # A welcome message
        # Hard-coding the topic name, doesn't make sense to need to pass it in each time.
        topic_name_hips = 'gait_commands_hips'
        topic_name_shoulders = 'gait_commands_shoulders'
        print("Running serial_tx_gait_commands node. Devices are:")
        print("Hips: " + device_name_hips)
        print("Shoulders: " + device_name_shoulders)
        #print(" and python version:")
        # print(sys.version)
        # Hard-code a timeout for pyserial. Seems recommended, even for tx?
        serial_timeout = 1
        # First, start up the single ROS node for these two messages.
        rospy.init_node('serial_tx_gait_commands', anonymous=False)
        # The main functionality here is a subscriber.
        #sub = rospy.Subscriber(topic_name, Float32MultiArray, self.serial_tx_callback)
        sub_hips = rospy.Subscriber(topic_name_hips, HipsGaitCommand, self.serial_tx_callback_hips)
        sub_shoulders = rospy.Subscriber(topic_name_shoulders, ShouldersGaitCommand, self.serial_tx_callback_shoulders)
        # We'll publish commands to a topic just in case someone else wants to use them
        # pub = rospy.Publisher('serial_tx_echo', String, queue_size=10)
        # Next, do the serial setup:
        # Hard-coded: our PSoC uses the following baud rate:
        psoc_baud = 115200
        # create the two serial port objects
        serial_port_hips = serial.Serial(device_name_hips, psoc_baud, timeout=serial_timeout)
        serial_port_shoulders = serial.Serial(device_name_shoulders, psoc_baud, timeout=serial_timeout)
        # flush out any old data
        serial_port_hips.reset_input_buffer()
        serial_port_hips.reset_output_buffer()
        serial_port_shoulders.reset_input_buffer()
        serial_port_shoulders.reset_output_buffer()

        # finishing setup.
        print("Opened ports, subscribers created. Now echoing from HipsGaitCommand and ShouldersGaitCommand to serial ports.")
        # and return the publisher so that the callback can use it.
        # also needs to have the serial port available to the callback.
        return serial_port_hips, serial_port_shoulders

    # The constructor calls a helper to initialize everything, and stores the
    # resulting publisher and serial port object that's created.
    def __init__(self, device_name_hips, device_name_shoulders):
        self.serial_port_hips, self.serial_port_shoulders = self.serial_tx_startup(device_name_hips, device_name_shoulders)
        # and that's all.


# the main function: create one of these objects, while parsing the serial port path
if __name__ == '__main__':
    # the 0-th arg is the name of the file itself, so we want the 1st and 2nd
    # We're making this a class now.
    s_tx = SerialTxGaitCommands(sys.argv[1], sys.argv[2])
    # We do the spin() in main. It's not appropriate for a constructor.
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
