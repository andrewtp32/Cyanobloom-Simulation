#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point32, PoseArray
from sensor_msgs.msg import PointCloud
import os

# this is a global variable: all functions and methods in this file have access to it
pointcloud_pub = rospy.Publisher("/desired_drone_pose", PointCloud, queue_size=1)
# publish an array of particles
particle_array_pub = rospy.Publisher("/particle_array", PoseArray, queue_size=1)

# directory of the package
package_directory = (os.path.relpath(os.path.dirname(__file__)))[:-4]
# load the list of waypoints from a file
with open(f'{package_directory}/drone_picture_coordinates/SabattusPond/drone_flight_path.txt') as waypoints_file:
    waypoints_array = np.loadtxt(waypoints_file, delimiter=' ')

# counter
counter = 0


def pose_callback(msg):
    # assign global
    global counter

    # obtain the position of the drone in the gazebo frame
    px = msg.position.x
    py = msg.position.y
    pz = msg.position.z

    # Declare PointCloud
    coordinate_pointcloud = PointCloud()
    # Filling PointCloud header
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    coordinate_pointcloud.header = header
    # Fill and publish pointcloud
    coordinate_pointcloud.points.append(Point32(waypoints_array[counter][0],
                                                waypoints_array[counter][1],
                                                0.0))
    pointcloud_pub.publish(coordinate_pointcloud)

    # calculate the error of the current drone position
    px_error = abs(px - waypoints_array[counter][0])
    py_error = abs(py - waypoints_array[counter][1])

    print(f"[waypoint_manager] Current position: [{px} {py}0]\n"
          f"[waypoint_manager] Desired position: [{waypoints_array[counter][0]} {waypoints_array[counter][1]}0]\n"
          f"[waypoint_manager] Error: [{px_error} {py_error}]\n")

    if (px_error <= 0.05) and (py_error <= 0.05):
        print("[waypoint_manager] Drone is in range. Moving on to next point.")

        # array
        arr = np.arange(12).reshape(6, 2)

        # array object
        published_array = PoseArray()
        # fill the header
        pub_arr_header = Header()
        pub_arr_header.stamp = rospy.Time.now()
        pub_arr_header.frame_id = 'map'
        published_array.header = pub_arr_header

        # run through each particle's coordinate and add to the ros message
        for i in arr:
            # create pose object
            published_pose = Pose()
            # add to the pose object
            published_pose.position.x = i[0]
            published_pose.position.y = i[1]
            published_pose.position.z = 0
            published_pose.orientation.x = 0
            published_pose.orientation.y = 0
            published_pose.orientation.z = 0
            published_pose.orientation.w = 0
            # add to the pose array object
            published_array.poses.append(published_pose)

        # publish array object
        particle_array_pub.publish(published_array)

        # add to the counter
        counter += 1

    if counter == len(waypoints_array):
        print("[waypoint_manager] reached all waypoints. Restarting.")
        counter = 0


def controller():
    rospy.init_node('waypoint_manager', anonymous=False)
    rospy.Subscriber("/drone/gt_pose", Pose, pose_callback, queue_size=1)
    print("[waypoint_manager] drone positioning manager launched...")
    # getting the desired drone positions from a csv file
    # rospy.Subscriber("/position_list_for_drone", PointCloud, pointcloud_callback)
    rospy.spin()


# this is the main function: the program starts here
if __name__ == '__main__':
    controller()
