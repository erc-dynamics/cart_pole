#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import pandas as pd

global data
data = []

# this ros node reads the robot state by a subscriber to '/state' topic
# and saves the data inside an excel file
    
# subscriber callback function
def state_callback(msg):
    global data
    # reads the current state
    #       [    t             ,     x      ,     phi    ,     xdot   ,    phidot  ,     u      ]
    state = [msg.data[0] / 1000, msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5]]
    # appends the current state to global variable
    data.append(state)

def shutdown_function():
    # on shutdown, it will save the data inside the global variable 'data' in an excel file.
    global data
    df = pd.DataFrame(data, columns=['t', 'x', 'phi', 'xdot', 'phidot', 'u'])
    excel_file_path = '~/validation/test(0).xlsx'
    df.to_excel(excel_file_path, index=False)

def main():
    # intializes a ros node
    rospy.init_node('plotter')
    # create a subscriber to '/state' topic
    rospy.Subscriber('/state', Float64MultiArray, state_callback)
    rospy.spin()
    # when the node is being shutdown it runs the function 'shutdown_function'
    rospy.on_shutdown(shutdown_function)

if __name__ == "__main__":
    main()
