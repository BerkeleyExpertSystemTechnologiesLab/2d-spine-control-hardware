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

# this helper function is a terrible solution to a problem that should be easier.
# start_time = float, the time at the start of sending commands.
# wait_until_since_start = float, the time at which to unblock and continue, referenced to the start time
# dt = float, interval to sleep between checking if we're done yet
def sleepUntil(start_time, wait_until_since_start, dt):
    # the wall time when this function should complete:
    final_time = start_time + wait_until_since_start
    # then loop until we've waited long enough
    while rospy.get_time() < final_time:
        rospy.sleep(dt)

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
    # We'll need two publishers, one for the hips and one for the shoulders.
    pub_hips = rospy.Publisher('gait_commands_hips', HipsGaitCommand, queue_size=10)
    pub_shoulders = rospy.Publisher('gait_commands_shoulders', ShouldersGaitCommand, queue_size=10)

    # Then, read in the csv file / gait commands. 
    # They're of mixed type: float string string.
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

    # finishing setup.
    print("File loaded into memory. Ctrl-C to stop output.")

    # To start printing the commands, iterate through the lists and sleepUntil
    # the time for the next start. We'll want to use an index variable here,
    # which isn't pythonic, but is fine.
    i = 0

    # the starting time for counting gait timing.
    start_time = rospy.get_time()
    # a pre-specified interval for checking in sleepUntil. In sec.
    sleepUntil_dt = 0.001

    # We iterate through the array until the end
    # but also need to catch shutdown signals.
    # This assumes there are the same number of rows in ALL the three data arrays above!
    while (i < gait_command_times.shape[0]) and not rospy.is_shutdown():
        # Pull out the target time for the i-th message
        time_i = gait_command_times[i]
        # Wait until it's time to publish that command
        sleepUntil(start_time, time_i, sleepUntil_dt)
        # Now create and send the message as appropriate.
        # For either shoulders or hips:
        if gait_command_hipsorshoulders[i] == "hips":
            # create a hips message
            to_publish_hips = HipsGaitCommand(hips_command = gait_command_commands[i])
            # publish the command
            pub_hips.publish(to_publish_hips)
        elif gait_command_hipsorshoulders[i] == "shoulders":
            # create a shoulders message
            to_publish_shoulders = ShouldersGaitCommand(shoulders_command = gait_command_commands[i])
            # publish the command
            pub_shoulders.publish(to_publish_shoulders)
        else:
            # error, got to be hips or shoulders!
            raise Exception('Command did not match either hips or shoulders!')

        # and return a little debugging message
        print("Sent command " + gait_command_commands[i] + " to " + gait_command_hipsorshoulders[i] + " at time " + str(rospy.get_time()-start_time) + ", target time was " + str(time_i))
        # then increment the timestep
        i += 1


# the main function: just call the helper, while parsing the path to the gait commands file.
if __name__ == '__main__':
    try:
        # the 0-th arg is the name of this .py file, so the 1-th argument is the gait commands file
        tx_to_topic(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
