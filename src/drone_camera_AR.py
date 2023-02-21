#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion
from cyanob_phd_filter.msg import diagnostics
import cv2
import numpy as np
import time
import math
import std_msgs.msg
from cyanob_phd_filter.msg import diagnostics


class CameraImageViewer(object):
    def __init__(self):
        print("2")
        self.px, self.py, self.pz, self.ox, self.oy, self.oz, self.ow = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        self.bloomPoseWorldFrame4DList = []
        self.bloomfilename = []  # this will store frame id of /particle_list_for_gazebo.
        self.orientation_q = []
        self.image_sub = rospy.Subscriber("/drone/front_camera/image_raw", Image, self.camera_callback)
        self.pose_sub = rospy.Subscriber("/drone_0/gt_pose", Pose, self.pose_callback)
        self.pointcloud_sub = rospy.Subscriber("/particle_list_for_gazebo", PointCloud, self.pointcloud_callback)
        # self.filterdiagnostics_sub = rospy.Subscriber("system_Diagnostics", diagnostics, self.callback_diagnostics)
        self.bridge_object = CvBridge()
        self.image_pub = rospy.Publisher("/drone/front_camera/image_raw/bloom", Image, queue_size=1)
        print("3")

    # def callback_diagnostics(self,msg):

    def camera_callback(self, Image):  # "data" is the image coming from the callback
        print("4")

        # Converting to a CV image
        cv_image = self.bridge_object.imgmsg_to_cv2(Image, desired_encoding="passthrough")

        # show the image for testing
        # cv2.imshow("Original image", cv_image)

        # Pose of the blooms in the world frame (homogeneous 4D vector)
        # for multiple blooms, the idea is to use a for-loop for each pose and add the variables to this vector
        bloom1PoseWorldFrame4D = [[-32.0],
                                  [-12.0],
                                  [0.75],
                                  [1.0]]

        bloom2PoseWorldFrame4D = [[-30.0],
                                  [-10.0],
                                  [0.75],
                                  [1.0]]

        bloom3PoseWorldFrame4D = [[-32.0],
                                  [-10.0],
                                  [0.75],
                                  [1.0]]

        bloom4PoseWorldFrame4D = [[-27.0],
                                  [-7.0],
                                  [0.75],
                                  [1.0]]

        bloom5PoseWorldFrame4D = [[-25.0],
                                  [-5.0],
                                  [0.75],
                                  [1.0]]

        bloom6PoseWorldFrame4D = [[-27.0],
                                  [-5.0],
                                  [0.75],
                                  [1.0]]

        # A list containing multiple bloom poses (homogeneous 4D vectors)
        bloomPoseWorldFrame4DList = [bloom1PoseWorldFrame4D, bloom2PoseWorldFrame4D, bloom3PoseWorldFrame4D,
                                     bloom4PoseWorldFrame4D, bloom5PoseWorldFrame4D, bloom6PoseWorldFrame4D]

        # declaring bloom to drone distance
        bloomToDroneDistance = []

        # declaring list for pixel locations
        bloomPoseImagePixelList = []

        # For loop iterates each of the bloom poses
        for bloomPoseWorldFrame4D in self.bloomPoseWorldFrame4DList:
            # print("bloom 4D positions in the world frame: ", bloomPoseWorldFrame4D)
            # -------------- 1. Get transformation matrix from world frame to drone frame -------------------

            # create a list containing the components of the drone's quaternion
            orientation_list = [self.ox, self.oy, self.oz, self.ow]

            # convert quaternion to roll, pitch, and yaw
            (rollDrone, pitchDrone, yawDrone) = euler_from_quaternion(orientation_list)

            # Compute components for rotation matrix (from world frame to drone frame)
            #     | r11  r12  r13 |
            # R = | r21  r22  r23 |
            #     | r31  r32  r33 |
            r11 = math.cos(pitchDrone) * math.cos(yawDrone)
            r12 = math.sin(rollDrone) * math.sin(pitchDrone) * math.cos(yawDrone) - math.cos(rollDrone) * math.sin(
                yawDrone)
            r13 = math.cos(rollDrone) * math.sin(pitchDrone) * math.cos(yawDrone) + math.sin(rollDrone) * math.sin(
                yawDrone)
            r21 = math.cos(pitchDrone) * math.sin(yawDrone)
            r22 = math.sin(rollDrone) * math.sin(pitchDrone) * math.sin(yawDrone) + math.cos(rollDrone) * math.cos(
                yawDrone)
            r23 = math.cos(rollDrone) * math.sin(pitchDrone) * math.sin(yawDrone) - math.sin(rollDrone) * math.cos(
                yawDrone)
            r31 = -1 * math.sin(pitchDrone)
            r32 = math.sin(rollDrone) * math.cos(pitchDrone)
            r33 = math.cos(rollDrone) * math.cos(pitchDrone)

            # create the rotation matrix from world to drone
            rotationMatrixWorldToDrone = np.array([[r11, r12, r13],
                                                   [r21, r22, r23],
                                                   [r31, r32, r33]])

            # pose of the drone in the world frame vector
            dronePoseWorldFrame = np.array([[self.px],
                                            [self.py],
                                            [self.pz]])

            # T is the transformation matrix. This is the T from the world to the drone
            T_world_drone = np.concatenate((rotationMatrixWorldToDrone, dronePoseWorldFrame), 1)
            T_world_drone = np.concatenate((T_world_drone, [[0, 0, 0, 1]]), 0)

            # -------------- 2. Get transformation matrix from drone frame to camera frame -------------------

            # Roll, pitch, and yaw angles from drone frame to camera frame
            # rotation values gathered from the camera portion of the .sdf file of the quadcopter
            rollCamera = 0.0
            pitchCamera = 1.57 + (math.pi / 2)  # camera initializes with pitch = pi/2 with respect to the camera
            yawCamera = 1.57

            cameraPoseDroneFrame = [[0.0],
                                    [0.0],
                                    [0.0]]

            # Compute components for rotation matrix (from drone frame to camera frame)
            #     | R11  R12  R13 |
            # r = | R21  R22  R23 |
            #     | R31  R32  R33 |
            R11 = math.cos(pitchCamera) * math.cos(yawCamera)
            R12 = math.sin(rollCamera) * math.sin(pitchCamera) * math.cos(yawCamera) - math.cos(rollCamera) * math.sin(
                yawCamera)
            R13 = math.cos(rollCamera) * math.sin(pitchCamera) * math.cos(yawCamera) + math.sin(rollCamera) * math.sin(
                yawCamera)
            R21 = math.cos(pitchCamera) * math.sin(yawCamera)
            R22 = math.sin(rollCamera) * math.sin(pitchCamera) * math.sin(yawCamera) + math.cos(rollCamera) * math.cos(
                yawCamera)
            R23 = math.cos(rollCamera) * math.sin(pitchCamera) * math.sin(yawCamera) - math.sin(rollCamera) * math.cos(
                yawCamera)
            R31 = -1 * math.sin(pitchCamera)
            R32 = math.sin(rollCamera) * math.cos(pitchCamera)
            R33 = math.cos(rollCamera) * math.cos(pitchCamera)

            # obtain the rotation matrix from the drone to the camera
            rotationMatrixDroneToCamera = np.array([[R11, R12, R13],
                                                    [R21, R22, R23],
                                                    [R31, R32, R33]])

            # T is the transformation matrix. This is the T from the drone to the camera
            T_drone_camera = np.concatenate((rotationMatrixDroneToCamera, cameraPoseDroneFrame), 1)
            T_drone_camera = np.concatenate((T_drone_camera, [[0, 0, 0, 1]]), 0)

            # -------------- 3. Get transformation matrix from world to camera -------------------

            # T is the transformation matrix. This is the T from the world to the camera frame
            T_world_camera = np.dot(T_world_drone, T_drone_camera)

            # -------------- 4. Compute pose of algae bloom in the camera frame -------------------

            # Obtain inverse of the T_world_camera
            T_camera_world = np.linalg.inv(T_world_camera)

            # Homogeneous 4D vector of the bloom pose in the camera frame
            bloomPoseCameraFrame4DVector = np.dot(T_camera_world, bloomPoseWorldFrame4D)

            # Converting 4D vector into a 3D vector
            bloomPoseCameraFrame = bloomPoseCameraFrame4DVector[0:3, 0:1]

            # -------------- 5. Obtain pinhole camera model from bloom pose in camera frame -------------------

            # parameters for the drone's camera
            alphaXF = 185.69
            x0 = 320.5
            alphaYF = 185.69
            y0 = 180.5

            # gather individual x, y, and z values from the bloom pose in camera frame vector
            bloomPoseCameraFrameX = bloomPoseCameraFrame[0][0]
            bloomPoseCameraFrameY = bloomPoseCameraFrame[1][0]
            bloomPoseCameraFrameZ = bloomPoseCameraFrame[2][0]

            # gather individual x, y, and z values from the drone pose in world frame vector
            dronePoseWorldFrameX = dronePoseWorldFrame[0][0]
            dronePoseWorldFrameY = dronePoseWorldFrame[1][0]
            dronePoseWorldFrameZ = dronePoseWorldFrame[2][0]

            # gather individual x, y, and z values from the bloom pose in world frame vector
            bloomPoseWorldFrameX = bloomPoseWorldFrame4D[0][0]
            bloomPoseWorldFrameY = bloomPoseWorldFrame4D[1][0]
            bloomPoseWorldFrameZ = bloomPoseWorldFrame4D[2][0]

            # get the x and y values of the pixel that corresponds to the bloom's pose in the camera frame
            xI = ((alphaXF * bloomPoseCameraFrameX) / bloomPoseCameraFrameZ) + x0
            yI = ((alphaYF * bloomPoseCameraFrameY) / bloomPoseCameraFrameZ) + y0

            pixelLocations = [int(xI), int(yI)]

            # compute the point-to-point distance from the bloom to the drone
            bloomToDroneDistance = (((dronePoseWorldFrameX - bloomPoseWorldFrameX) ** 2)
                                    + ((dronePoseWorldFrameY - bloomPoseWorldFrameY) ** 2)
                                    + ((dronePoseWorldFrameZ - bloomPoseWorldFrameZ) ** 2))

            # add all the pixel coordinates to a list
            bloomPoseImagePixelList.append(pixelLocations)

        # compute the radius of the bloom based upon how far away the bloom is from the drone
        bloomSize = 1
        focalLength = 1.738
        # radiusOfBloom = int(5500 / bloomToDroneDistance)
        alpha = 0.05

        # create two copies of the image
        cv_output = cv_image.copy()
        cv_overlay = cv_image.copy()

        # create a variable that will count the number of pixel values in "bloomPoseImagePixelList"
        count = 0

        for pixels in bloomPoseImagePixelList:
            count = 1.1 * (count + 1)
            cv_overlay = cv_output.copy()
            # draw a yellow circle on camera image at the augmented point of bloom
            cv2.circle(cv_overlay, (pixels[0], pixels[1]), 30, (255, 255, 0), -1)
            # apply the overlay
            cv2.addWeighted(cv_overlay, 0.3, cv_output, 0.7, 0, cv_output)

        # cv2.imshow("AR image", cv_output)
        cv2.waitKey(3)

        # Converting back to a ros image and publishing
        msg = self.bridge_object.cv2_to_imgmsg(cv_output, encoding="rgb8")  # converting crop_img from cv to ros msg
        self.image_pub.publish(msg)

        print("5")

    def pose_callback(self, Pose):
        self.px = Pose.position.x
        self.py = Pose.position.y
        self.pz = Pose.position.z
        self.ox = Pose.orientation.x
        self.oy = Pose.orientation.y
        self.oz = Pose.orientation.z
        self.ow = Pose.orientation.w

    def pointcloud_callback(self, PointCloud):
        if self.bloomfilename != PointCloud.header.frame_id:  # prevents updating for each msg. only update when new points received
            self.bloomPoseWorldFrame4DList = []
            # run_once = 0
            # while 1:
            #     if run_once == 0:
            for p in PointCloud.points:
                singular_bloom_coordinate = [[p.x],
                                             [p.y],
                                             [0.75],
                                             [1.0]]
                self.bloomPoseWorldFrame4DList.append(singular_bloom_coordinate)
                # run_once = 1
            print("new points updated to simulation")
            self.bloomfilename = PointCloud.header.frame_id


def main():
    print("1")
    bloom_filter_object = CameraImageViewer()
    rospy.init_node('bloom_filter_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
