#!/usr/bin/env python3

import rospy
import time, subprocess
from std_srvs.srv import Empty
from gazebo_msgs.srv import DeleteModel
from cart_pole.srv import LaunchParameters, LaunchParametersResponse
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import xacro
from std_msgs.msg import Float64MultiArray

# This ros node is responsible of creating the service '/start_simulation'
# this service upon any request, will start a simulation which will last 20000 iterations or 20 seconds
# after 20000 iterations the robot model will be deleted and the simulation will reset
# the node will not shutdown, but wait for a new request to start a new simulation

global iteration
iteration = 0


def handle_start_simulation(req):
    global iteration
    xacro_args = {
        'mc': req.mc,
        'mp': req.mp,
        'l': req.l,
        'k1': req.k1,
        'k2': req.k2,
        'k3': req.k3,
        'k4': req.k4
    }

    cmd_args = [f"{key}:={value}" for key, value in xacro_args.items()]

    robot_description_path = "/home/tofigh/catkin_ws/src/cart_pole/urdf/robot.xacro"

    xacro_cmd = ['xacro', robot_description_path] + cmd_args

    robot_description = subprocess.check_output(xacro_cmd).decode('utf-8')

    initial_pose = Pose()
    initial_pose.position.x = 0.0
    initial_pose.position.y = 0.0
    initial_pose.position.z = 0.0
    initial_pose.orientation.w = 1.0
    try:
        spawn_urdf_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp = spawn_urdf_model("robot", robot_description, "", initial_pose, "world")
        unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        unpause_physics()

        while not iteration == 20000:
            time.sleep(0.005)

        iteration = 0

        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model('robot')
        pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        pause_physics()

        reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        reset_simulation()

        rospy.loginfo('simulation is done')
        return resp.success
    
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return False
    
# subscriber callback function
# this function reads the current iteration of the simulation.
def state_callback(msg):
    global iteration
    iteration = msg.data[0]


if __name__ == '__main__':
    # initializes ros node
    rospy.init_node('simulation')
    # waits for the service '/gazebo/spawn_urdf_model', which is necessary to spawn robot model
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    # creates ros service '/start_simulation' with a handle function 'handle_start_simulation'
    rospy.Service('/start_simulation', LaunchParameters, handle_start_simulation)
    # a ros subscriber to '/state' topic to read the current iteration of the simulation
    rospy.Subscriber('/state', Float64MultiArray, state_callback)
    rospy.loginfo('ready to receive commands')
    rospy.spin()