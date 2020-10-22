#!/usr/bin/env python

import rosbag
import csv
import sys

# Usage: python bag2csv.py robot_name joint_observe bagfile csvfile
#
# This script reads in the bagfile that was created during the 10 minute test. It parses the data
# and neatly formats it into a csv file that can be analyzed later by the user. It performs some stats
# like showing the min and max temperatures and effort
def main():
    robot_name = sys.argv[1]                        # Name of the robot that was tested
    joint_observe = str(sys.argv[2])                # The name of the joint to be analyzed
    bag = rosbag.Bag("../bag/" + sys.argv[3])       # Name of the bagfile that holds the data
    master_list = []                                # Contains rows where each row holds all the joint data at a specific time
    temp_list = []                                  # Contains rows with two pieces of data: Time [s] and the joint temperature [Celsius] at that time
    state_list = []                                 # Contains rows with four pieces of data: Time [s], and the joint position, velocity, and effort at that time
    command_list = []                               # Contains rows with two pieces of data: Time [s] and the joint position command at that time
    sum_pos_effort = 0                              # Sum of all positive efforts - this is used to calculate the average positive effort [mA]
    sum_neg_effort = 0                              # Sum of all negative efforts - this is used to calculate the average negative effort [mA]
    max_effort = 0                                  # Max observed effort [mA]
    min_effort = 3200                               # Min observed effort [mA]
    num_pos_effort = 0                              # Number of observed positive efforts - used to calculate the average positive effort [mA]
    num_neg_effort = 0                              # Number of observed negative efforts - used to calculate the average negative effort [mA]
    min_temp = 100                                  # Min observed temperature [C]
    max_temp = 0                                    # Max observed temperature [C]
    temp_start_time = None                          # Timestamp of the first Temperature message
    states_start_time = None                        # Timestamp of the first Joint state message
    command_start_time = None                       # Timestamp of the first Joint command message
    modulo_num = 9                                  # Number of messages to skip before including joint data in the csv - this is used to ensure the csv file is not too big
    temps_id = 0                                    # Index position of 'joint_observe' in the JointTemps 'names' list
    js_id = 0                                       # Index position of 'joint_observe' in the JointState 'name' list

    # Get initial starting times for all the messages. These times will be subtracted from all
    # timestamps so that it's as if they are all starting at '0'.
    for topic, msg, t in bag.read_messages(topics=['/'+robot_name+'/temperatures/joint_group']):
            temp_start_time = t.to_sec()
            temps_id = msg.names.index(joint_observe)
            break
    for topic, msg, t in bag.read_messages(topics=['/'+robot_name+'/joint_states']):
            states_start_time = t.to_sec()
            js_id = msg.name.index(joint_observe)
            break
    for topic, msg, t in bag.read_messages(topics=['/'+robot_name+'/commands/joint_single']):
            command_start_time = t.to_sec()
            break

    # Since there are not many joint_temperature messages (as they are only published at 5Hz),
    # all of them will be included in the csv file.
    for topic, msg, t in bag.read_messages(topics=['/'+robot_name+'/temperatures/joint_group']):
        data = []
        data.append(t.to_sec() - temp_start_time)
        data.append(msg.temps[temps_id])
        temp_list.append(data)
        min_temp = min(min_temp, msg.temps[temps_id])
        max_temp = max(max_temp, msg.temps[temps_id])

    # Since there are many joint_state messages, only one out of every nine messages will
    # be inputted into the csv file ('nine' was arbitrarily chosen)
    cntr = 0
    for topic, msg, t in bag.read_messages(topics=['/'+robot_name+'/joint_states']):
        if (cntr % modulo_num == 0):
            data = []
            data.append(t.to_sec() - states_start_time)
            data.append(msg.position[js_id])
            data.append(msg.velocity[js_id])
            data.append(msg.effort[js_id])
            state_list.append(data)
        if msg.effort[js_id] > 0:
            sum_pos_effort += msg.effort[js_id]
            num_pos_effort += 1
        elif msg.effort[js_id] < 0:
            sum_neg_effort += msg.effort[js_id]
            num_neg_effort += 1
        min_effort = min(min_effort, msg.effort[js_id])
        max_effort = max(max_effort, msg.effort[js_id])
        cntr += 1

    # Since there are many command/joint_single messages, only one out of every nine messages will
    # be inputted into the csv file ('nine' was arbitrarily chosen)
    cntr = 0
    for topic, msg, t in bag.read_messages(topics=['/'+robot_name+'/commands/joint_single']):
        if (cntr % modulo_num == 0):
            data = []
            data.append(t.to_sec() - command_start_time)
            data.append(msg.cmd)
            command_list.append(data)
        cntr += 1

    # Make sure there will be no 'divide by zero' errors
    if num_pos_effort == 0:
        avg_pos_effort = 0
    else:
        avg_pos_effort = sum_pos_effort/float(num_pos_effort)

    if num_neg_effort == 0:
        avg_neg_effort = 0
    else:
        avg_neg_effort = sum_neg_effort/float(num_neg_effort)

    # some stats that will be inputted on the top right of the csv file
    stats = [max_temp, min_temp, max_temp-min_temp, max_effort, min_effort, avg_pos_effort, avg_neg_effort]

    # Set the size of the 'master' list to the smaller of the following two lists. This is to make sure there is no 'orphan' data
    # or data that might include the joint states but not joint commands or visa versa
    max_size = max(len(state_list), len(command_list))

    # 'Knit' the three sub lists (joint temperatures, joint states, joint commands) into one row of data
    # This row will be added to the 'master' list
    for x in range(max_size):
        row = []
        if (x < len(temp_list)):
            for item in temp_list[x]:
                row.append(item)
        else:
            for i in range(len(temp_list[0])):
                row.append("")
        if (x < len(state_list)):
            for item in state_list[x]:
                row.append(item)
        else:
            for i in range(len(state_list[0])):
                row.append("")
        if (x < len(command_list)):
            for item in command_list[x]:
                row.append(item)
        else:
            for i in range(len(command_list[0])):
                row.append("")
        if x == 0:
            row.append("")
            for item in stats:
                row.append(item)
        master_list.append(row)

    # Write each row of data into the csv file
    with open(sys.argv[4], mode='w') as csv_file:
        message_writer = csv.writer(csv_file)
        message_writer.writerow(["Temperature Time [s]", "Temp [C]", "States Time [s]", "Position [rad]", "Velocity [rad/s]", "Effort [mA]", "Commands Time [s]", "Position Command [rad]","","Max Temp [C]","Min Temp [C]", "Temp Range [C]", "Max Effort [mA]", "Min Effort [mA]", "Avg Pos Effort [mA]", "Avg Neg Effort [mA]"])
        for line in master_list:
            message_writer.writerow(line)

if __name__=='__main__':
    main()
