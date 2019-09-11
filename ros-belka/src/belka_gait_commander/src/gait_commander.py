#!/usr/bin/python

# NOTE: on Ubuntu 18.04, specifying /usr/bin/env python uses python 3,
# while specifying /usr/bin/python uses python 2.7. We need 2.7.

# invkin_tx_totopic reads in an inverse kinematics calculation file from the tIK library
# and publishes those commands to a topic at a specified interval.
# Usage is:
#	rosrun spine_controller invkin_tx_totopic path-to-file

# gait_commander reads the .csv file (spreadsheet-like file) with the gait commands, 
# and publishes the commands at the specified times. Needs to be used with serial_tx_gait_commands.py 
# Usage is:
#   rosrun belka_gait_commander gait_commander.py PATH_TO_CSV_FILE


# Imports:
import rospy
# because we need the command-line arguments
import sys
# and for ease of importing data, numpy!
import numpy as np
# We'll use two messages, both strings, to differentiate what gets sent to hips vs. shoulders
from belka_gait_commander.msg import HipsGaitCommand
from belka_gait_commander.msg import ShouldersGaitCommand
# we'll need to parse the string itself too?
from std_msgs.msg import String

# The primary helper function here opens the csv file, creates a big list of all the times
# that commands should be published as well as when, and iteratively sleeps until each of those
# times have been met.

def tx_to_topic(file_name):
    # A welcome message
    print("Running gait_commander with file: " + file_name)
    #print("and python version:")
    # print(sys.version)
    # First, start up the ros node.
    rospy.init_node('gait_commander', anonymous=False)
    # We need a publisher. Note we're using the numpy message type, wrapping
    # around the standard message type
    #pub = rospy.Publisher('invkin_tx_commands', numpy_msg(Float32MultiArray), queue_size=10)
    # We'll need two publishers, one for the hips and one for the shoulders.
    pub_hips = rospy.Publisher('gait_commands_hips', HipsGaitCommand, queue_size=10)
    pub_shoulders = rospy.Publisher('gait_commands_shoulders', ShouldersGaitCommand, queue_size=10)

    # Then, read in the csv file.
    # The 3rd row (counting from 0) contains the parameters for this run, and is
    # of mixed type: int, string, int, int, int, string
    # let's try with autodetection of type. Only want 1 row.
    # invkin_header = np.genfromtxt(file_name, dtype=None, delimiter=",",
    #                               skip_header=3, max_rows=1)
    # print("Using an inverse kinematics file with the parameters:")
    # print(invkin_header)
    # We need to now pick out the number of cables. That's the 4th column
    # on the csv file. Numpy was giving me some trouble but turning the array into
    # a list seemed to help.
    #print(invkin_header.tolist()[0])
    # s = invkin_header.tolist()[3]

    # Then, read the data itself. Starts two rows down from header.
    # control_data = np.genfromtxt(file_name, dtype=float, delimiter=",",
                                # skip_header=5)

    # Read in the gait commands. They're of mixed type: float string string.
    # Starts at row 2, counting from 0 (third row.)
    # Read each individually for easy typing.
    gait_command_times = np.genfromtxt(file_name, dtype=float, delimiter=",",
                                       skip_header=2, usecols=0)
    gait_command_commands = np.genfromtxt(file_name, dtype=String, delimiter=",",
                                          skip_header=2, usecols=1)
    gait_command_hipsorshoulders = np.genfromtxt(file_name, dtype=String, delimiter=",",
                                                 skip_header=2, usecols=2)

    # because we'll need to use the times with numpy, just in case, confirm it's a numpy array and not a list:
    gait_command_times = np.array(gait_command_times)

    # Get the times at which to send each command.
    # command_times = gait_commands_raw_data.tolist()[0]
    # print(gait_command_times)
    # and the commands themselves, and which hip/shoulder to send to
    # commands = gait_commands_raw_data.tolist()[1]
    # print(gait_command_commands)
    # hips_or_shoulders = gait_commands_raw_data.tolist()[2]
    # print(gait_command_hipsorshoulders)

    # # Split the data into its two parts: 0:s-1 == invkin, remainder == states.
    # #print(control_data)
    # invkin_data = control_data[:, 0:s]
    # # this is numpy notation for MATLAB equivalent control_data(:, s:end)
    # state_data = control_data[:, s:]
    # # print(invkin_data.shape)
    # #print(state_data)

    # # Create a timer object that will sleep long enough to result in
    # # a reasonable publishing rate
    # r = rospy.Rate(0.5)  # hz
    # # We'll keep track of rows: we only want to publish until the end of the CSV file.
    # # max timestep is number of rows.
    # max_timestep = invkin_data.shape[0]
    # # initialize the counter
    # current_timestep = 0

    # finishing setup.
    print("File loaded into memory. Ctrl-C to stop output.")

    # # We iterate through the array until the end
    # # but also need to catch shutdown signals.
    # while (current_timestep < max_timestep) and not rospy.is_shutdown():
    #     # Create the message itself
    #     #to_publish = Float32MultiArray()
    #     to_publish = InvkinControlCommand(invkin_control = invkin_data[current_timestep, :], \
    #     	invkin_ref_state = state_data[current_timestep, :])
    #     # Put in the numpy array
    #     #to_publish.data = np.array([0.5, 0.5, 0.5, 0.5])
    #     #to_publish.data = invkin_data[current_timestep, :]
    #     # Publish the current_timestep-th message
    #     pub.publish(to_publish)
    #     # Echo to the terminal
    #     print("Timestep " + str(current_timestep) + ", publishing:")
    #     print(to_publish.invkin_control)
    #     # increment the counter
    #     current_timestep += 1
    #     # sleep until the next output
    #     r.sleep()


# the main function: just call the helper, while parsing the path to the gait commands file.
if __name__ == '__main__':
    try:
        # the 0-th arg is the name of this .py file, so the 1-th argument is the gait commands file
        tx_to_topic(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
