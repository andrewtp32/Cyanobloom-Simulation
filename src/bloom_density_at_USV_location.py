#!/usr/bin/env python
import rospy
import std_msgs.msg
import numpy as np
import tf
import geonav_transform.geonav_conversions as gc  # Import geonav tranformation module
reload(gc)
import alvinxy.alvinxy as axy  # Import AlvinXY transformation module
reload(axy)

from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Temperature
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point32


class bloom_normal_distribution(object):
    def __init__(self):
        print("2")
        self.bloomCoordinateWorldFrameList = []
        self.bloom_filename = []  # this will store frame id of /particle_list_for_gazebo.
        self.pointcloud_sub = rospy.Subscriber("/particle_list_for_gazebo", PointCloud, self.pointcloud_callback)
        self.usv_gps_sub = rospy.Subscriber("/wamv/sensors/gps/gps/fix", NavSatFix, self.usv_gps_callback)
        self.pointcloud_pub = rospy.Publisher("/USV_coordinates_and_bloom_concentration_value_at_USV_location", PointCloud)
        print("3")

    def usv_gps_callback(self, NavSatFix):
        print("3")
        # Convert the latitude and longitude position of the usv into an xy coordinate

        # This is the set of coordinated that correlated to (0,0) in the gazebo world frame
        olat = 21.3099597542
        olon = -157.890104634

        # This is the lat and lon of the usv in the gazebo world frame
        latitude = NavSatFix.latitude
        longitude = NavSatFix.longitude
        print("Lat and Lon of the boat: ", latitude, longitude)

        # Convert the usv's position from lat/lon to xy
        xGazebo, yGazebo = gc.ll2xy(latitude, longitude, olat, olon)
        # place the x and y points into a list to create a 2D vector
        usvCoordinateWorldFrame = [[xGazebo],
                                   [yGazebo]]
        # print("X and Y position of the boat in the Gazebo world frame: ", usvCoordinateWorldFrame)

        # add up all the values within the list to get the concentration of blooms at the position of the usv
        sumOfPDF = 0
        pdfList = []

        # for loop to iterate through each bloom point
        for bloom in self.bloomCoordinateWorldFrameList:
            # turn these into arrays for PDF calculation
            usvXY = np.array(usvCoordinateWorldFrame)
            bloomXY = np.array(bloom)
            # if the bloom's position is within a 3m radius of the boat, calculate its PDF with respect to the USV
            if (usvXY[0]-3)<bloomXY[0]<(usvXY[0]+3) and (usvXY[1]-3)<bloomXY[1]<(usvXY[1]+3):
                # calculate the gaussian density function for the point with: test point = usv position, mean = each
                # bloom position
                pdf = np.exp((-1 / 2) * np.dot(np.transpose(usvXY - bloomXY), (usvXY - bloomXY)))
                # scale up the value of the pdf
                pdf = pdf * 1000
                # append each calculated value to a list
                pdfList.append(pdf)
                # sum all the pdfs
                sumOfPDF = sum(pdfList)

        # sumOfPDF is an array. These next two lines are here to obtain the float value that we want from inside the
        # sumOfPDF array
        sumOfPDF1 = sumOfPDF[0]
        sumOfPDF_value = sumOfPDF1[0]
        print("The concentration of blooms at ", usvCoordinateWorldFrame, " is: ", sumOfPDF_value)

        # --------------- Declaring and filling "PointCloud" ---------------

        # Declare "PointCloud" message
        boat_coordinate_and_bloom_concentration = PointCloud()

        # Filling "Temperature" header
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        boat_coordinate_and_bloom_concentration.header = header

        # Assign values to the message
        # The .x and the .y in the PointCloud are the coordinates of the boat in the gazebo world
        # The .z in the PointCloud is the value of concentration of blooms at the boat's coordinates
        boat_coordinate_and_bloom_concentration.points.append(Point32(xGazebo, yGazebo, sumOfPDF_value))

        # --------------- Publishing messages to topics ---------------

        # Publish the concentration value to a topic
        self.pointcloud_pub.publish(boat_coordinate_and_bloom_concentration)

        print("5")

    def pointcloud_callback(self, PointCloud):
        # Convert the pointcloud points into a 2D coordinate. Then place the 2D vectors into a list.
        if self.bloom_filename != PointCloud.header.frame_id: #prevents updating for each msg. only update when new points received
            self.bloomCoordinateWorldFrameList = []
            for p in PointCloud.points:
                singular_bloom_coordinate = [[p.x],
                                                [p.y]]
                self.bloomCoordinateWorldFrameList.append(singular_bloom_coordinate)
                    # run_once = 1
            print("new points updated to simulation")
            self.bloom_filename=PointCloud.header.frame_id


def main():
    print("1")
    bloomNormalDistribution = bloom_normal_distribution()
    rospy.init_node('bloom_normal_distribution', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
