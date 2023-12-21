#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud
from tf.transformations import euler_from_quaternion

# this is a global variable: all functions and methods in this file have access to it
cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

error_integral_x = 0
error_integral_y = 0
error_integral_z = 0
error_previous_x = 0
error_previous_y = 0
error_previous_z = 0
desired_x = 50
desired_y = 50
desired_z = 80


# this is our callback function: it is executed any time a message on the specified topic
# is received. In our case, the specified topic is /drone/gt_pose.
# The message received will be available in the function through the variable msg  
def pose_callback(msg):
    # to use a global variable in a function, you must explicitly
    # redefine with the global attribute
    global cmd_pub
    global error_integral_x
    global error_integral_y
    global error_integral_z
    global error_previous_x
    global error_previous_y
    global error_previous_z
    global desired_x
    global desired_y
    global desired_z

    print("[drone_command_velocities] Desired position: ", desired_x, desired_y)

    # Position of the drone
    px = msg.position.x
    py = msg.position.y
    pz = msg.position.z

    # Grab the quaternion values from the subscriber
    ox = msg.orientation.x
    oy = msg.orientation.y
    oz = msg.orientation.z
    ow = msg.orientation.w

    # convert quaternion to roll, pitch, and yaw
    (roll_drone, pitch_drone, yaw_drone) = euler_from_quaternion([ox, oy, oz, ow])

    # This is where I will include the desired drone poses from the drone_positioning_manager node
    # set the desired flight height here, and the gains of the controller
    # desired_x = -32.0  # in meters
    # desired_y = -12.0  # in meters
    # desired_z = 15.0  # in meters
    k_p = 1.0  # proportional gain
    k_i = 0.2  # integral gain
    k_d = 0.0  # derivative gain

    # compute the error for x
    error_x = desired_x - px
    error_integral_x = error_integral_x + error_x * 0
    if error_integral_x > 1:
        error_integral_x = 1
    if error_integral_x < -1:
        error_integral_x = -1
    error_derivative_x = (error_x - error_previous_x) / 0.001

    # compute the error for y
    error_y = desired_y - py
    error_integral_y = error_integral_y + error_y * 0
    if error_integral_y > 1:
        error_integral_y = 1
    if error_integral_y < -1:
        error_integral_y = -1
    error_derivative_y = (error_y - error_previous_y) / 0.001

    # compute the error for z
    error_z = desired_z - pz
    error_integral_z = error_integral_z + error_z * 0.001
    if error_integral_z > 1:
        error_integral_z = 1
    if error_integral_z < -1:
        error_integral_z = -1
    error_derivative_z = (error_z - error_previous_z) / 0.001

    # compute the command needed for the
    linear_velocity_x = k_p * error_x + k_i * error_integral_x + k_d * error_derivative_x
    # compute the command needed for the
    linear_velocity_y = k_p * error_y + k_i * error_integral_y + k_d * error_derivative_y
    # compute the command needed for the
    linear_velocity_z = k_p * error_z + k_i * error_integral_z + k_d * error_derivative_z

    # here I create a new Twist message (=linear velocity and angular velocity), I write
    # the value I computed for the linear velocity on z to achieve a flight height of 2m
    # and I publish it on the appropriate topic
    cmdmsg = Twist()
    cmdmsg.linear.z = linear_velocity_z
    cmdmsg.linear.y = (-1 * np.sin(yaw_drone) * linear_velocity_x) + (np.cos(yaw_drone) * linear_velocity_y)
    cmdmsg.linear.x = (np.cos(yaw_drone) * linear_velocity_x) + (np.sin(yaw_drone) * linear_velocity_y)
    cmd_pub.publish(cmdmsg)


def pointcloud_callback(msg):
    global desired_x
    global desired_y
    desired_coordinate = []
    for p in msg.points:
        desired_coordinate = [p.x, p.y]
    desired_x = desired_coordinate[0]
    desired_y = desired_coordinate[1]


def controller():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    # pub = rospy.Publisher("/drone_0/cmd_vel", Twist, queue_size=1)

    rospy.init_node('drone_command_velocities', anonymous=False)

    rospy.Subscriber("/desired_drone_pose", PointCloud, pointcloud_callback)
    rospy.Subscriber("/drone/gt_pose", Pose, pose_callback, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


# this is the main function: the program starts here
if __name__ == '__main__':
    controller()
