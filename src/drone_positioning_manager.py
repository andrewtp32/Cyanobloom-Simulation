#!/usr/bin/env python
import rospy
import time
import numpy as np
import std_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from cyanob_phd_filter.msg import diagnostics
import os

# this is a global variable: all functions and methods in this file have access to it
pointcloud_pub = rospy.Publisher("/desired_drone_pose", PointCloud, queue_size=1)
# subscribing to system dianostics topic
sdTopic = rospy.get_param('diaganostic_topicName', default="/system_Diagnostics")
sd_pub = rospy.Publisher(sdTopic, diagnostics, queue_size=10)
# load the desired drone positions into an array
with open(os.path.expanduser(
        "~/catkin_ws/src/uri_soft_wip/cyanobloom_simulation/drone_picture_coordinates/drone_desired_positioning_list.txt")) as txt_file: drone_desired_position_list = np.loadtxt(
    txt_file, delimiter=",")
# create a counter so the pose_callback can iterate through the position list
counter = 0
# Declare a boolean. This boolean will give the drone access to moving through it's path progression.
can_drone_fly = False
sd_msg = diagnostics
droneReturningHome = False  # this boolean used to inform drone to return stating point which is 1st point in thelist.


def pose_callback(msg):
    # define variables. The px, py, and pz are the pose of the drone.
    global drone_desired_position_list
    global counter
    global can_drone_fly
    global sd_msg
    global droneReturningHome
    if can_drone_fly:
        # print(counter)
        # print(counter)
        # obtain the position of the drone in the gazebo frame
        px = msg.position.x
        py = msg.position.y
        pz = msg.position.z
        # Obtain the point that we want the drone to be at
        drone_desired_position_point = drone_desired_position_list[counter]
        # Declare PointCloud
        coordinate_pointcloud = PointCloud()
        # Filling PointCloud header
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        coordinate_pointcloud.header = header
        # Fill and publish pointcloud
        coordinate_pointcloud.points.append(Point32(drone_desired_position_point[0],
                                                    drone_desired_position_point[1],
                                                    0.0))
        pointcloud_pub.publish(coordinate_pointcloud)
        # Calculate the error between the current position and the desired position
        px_error = abs(px - drone_desired_position_point[0])
        py_error = abs(py - drone_desired_position_point[1])
        # print("Errors: x=", px_error, " y=", py_error)
        # print("Desired position: ", drone_desired_position_point[0], drone_desired_position_point[1])
        # print("Current position: ", px, py, pz)
        # Determine if the position of the drone is within the allowed threshold
        if px_error <= 0.05 and py_error <= 0.05:
            print("[DronePosManger] Drone is in range. Moving on to next point.")
            time.sleep(1.5)
            if not droneReturningHome:
                counter += 1
                if counter >= len(drone_desired_position_list):  # this if statement checks to see if
                    sd_msg.measurement_update = 'stop'
                    sd_msg.simulation = "complete"
                    sd_pub.publish(sd_msg)
                    counter = 0
                    # can_drone_fly = False
                    print(
                        "[DronePosManger] drone reached all way points. measurement update stopped. Drone returning to Home")
                    droneReturningHome = True

            else:  # drone returned home
                sd_msg.measurement_update = 'stop'
                sd_msg.simulation = 'droneatHome'
                sd_pub.publish(sd_msg)
                droneReturningHome = False
                can_drone_fly = False
                print("[DronePosManger] Drone Reached Home. New measurement can begin")


def diagnostics_callback(msg2):
    # print("msg received inside drone pos manager")
    global can_drone_fly
    global sd_msg
    sd_msg = msg2
    # activate msg sent by MU
    if msg2.simulation == "activate":
        print("[DronePosManger] Drone simulation activated. Drone begin fly now")
        dronepoints_counter = 0
        can_drone_fly = True


def controller():
    rospy.init_node('drone_positioning_manager', anonymous=False)
    rospy.Subscriber("/drone_0/gt_pose", Pose, pose_callback, queue_size=1)
    rospy.Subscriber(sdTopic, diagnostics, diagnostics_callback)
    print("[DronePosManger] drone positioning manager launched...")
    # getting the desired drone positions from a csv file
    # rospy.Subscriber("/position_list_for_drone", PointCloud, pointcloud_callback)
    rospy.spin()


# this is the main function: the program starts here
if __name__ == '__main__':
    controller()
