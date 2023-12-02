#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion
import cv2
import numpy as np


class CameraImageViewer(object):
    def __init__(self):
        # print("2")
        self.px, self.py, self.pz, self.ox, self.oy, self.oz, self.ow = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        self.bloomPoseWorldFrame4DList = []
        self.bloomfilename = []  # this will store frame id of /particle_list_for_gazebo.
        self.orientation_q = []
        self.image_sub = rospy.Subscriber("/drone/down_camera/image_raw", Image, self.camera_callback)
        self.pose_sub = rospy.Subscriber("/drone/gt_pose", Pose, self.pose_callback)
        self.pointcloud_sub = rospy.Subscriber("/particle_list_for_gazebo", PointCloud, self.pointcloud_callback)
        # self.filterdiagnostics_sub = rospy.Subscriber("system_Diagnostics", diagnostics, self.callback_diagnostics)
        self.bridge_object = CvBridge()
        self.image_pub = rospy.Publisher("/drone/down_camera/bloom_filter", Image, queue_size=1)
        # print("3")

    # def callback_diagnostics(self,msg):

    def camera_callback(self, Image):  # "data" is the image coming from the callback
        # print("4")

        # Converting to a CV image
        cv_image = self.bridge_object.imgmsg_to_cv2(Image, desired_encoding="passthrough")

        # show the image for testing
        cv2.imshow("Original image", cv_image)

        # declaring bloom to drone distance
        bloomToDroneDistance = []

        # declaring list for pixel locations
        bloomPoseImagePixelArray = np.array([], np.int32)

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
            r11 = np.cos(pitchDrone) * np.cos(yawDrone)
            r12 = np.sin(rollDrone) * np.sin(pitchDrone) * np.cos(yawDrone) - np.cos(rollDrone) * np.sin(
                yawDrone)
            r13 = np.cos(rollDrone) * np.sin(pitchDrone) * np.cos(yawDrone) + np.sin(rollDrone) * np.sin(
                yawDrone)
            r21 = np.cos(pitchDrone) * np.sin(yawDrone)
            r22 = np.sin(rollDrone) * np.sin(pitchDrone) * np.sin(yawDrone) + np.cos(rollDrone) * np.cos(
                yawDrone)
            r23 = np.cos(rollDrone) * np.sin(pitchDrone) * np.sin(yawDrone) - np.sin(rollDrone) * np.cos(
                yawDrone)
            r31 = -1 * np.sin(pitchDrone)
            r32 = np.sin(rollDrone) * np.cos(pitchDrone)
            r33 = np.cos(rollDrone) * np.cos(pitchDrone)

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
            pitchCamera = 1.57 + (np.pi / 2)  # camera initializes with pitch = pi/2 with respect to the camera
            yawCamera = 1.57

            cameraPoseDroneFrame = [[0.0],
                                    [0.0],
                                    [0.0]]

            # Compute components for rotation matrix (from drone frame to camera frame)
            #     | R11  R12  R13 |
            # r = | R21  R22  R23 |
            #     | R31  R32  R33 |
            R11 = np.cos(pitchCamera) * np.cos(yawCamera)
            R12 = np.sin(rollCamera) * np.sin(pitchCamera) * np.cos(yawCamera) - np.cos(rollCamera) * np.sin(
                yawCamera)
            R13 = np.cos(rollCamera) * np.sin(pitchCamera) * np.cos(yawCamera) + np.sin(rollCamera) * np.sin(
                yawCamera)
            R21 = np.cos(pitchCamera) * np.sin(yawCamera)
            R22 = np.sin(rollCamera) * np.sin(pitchCamera) * np.sin(yawCamera) + np.cos(rollCamera) * np.cos(
                yawCamera)
            R23 = np.cos(rollCamera) * np.sin(pitchCamera) * np.sin(yawCamera) - np.sin(rollCamera) * np.cos(
                yawCamera)
            R31 = -1 * np.sin(pitchCamera)
            R32 = np.sin(rollCamera) * np.cos(pitchCamera)
            R33 = np.cos(rollCamera) * np.cos(pitchCamera)

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

            # Obtain inverse of the T_world_camera
            T_camera_world = np.linalg.inv(T_world_camera)

            # -------------- 4. Compute pose of cyanobloom corners in the camera frame -------------------
            # this is done so that the shapes change size based on what angle / how far away the camera is

            # each cell is a 30m x 30m square. calc the corners for the 2D projection
            # calc the "start point" (top-left corner) of the cyanobloom square
            top_left_bloomPoseWorldFrame4D = [[bloomPoseWorldFrame4D[0] - 15],
                                              [bloomPoseWorldFrame4D[1] + 15],
                                              [bloomPoseWorldFrame4D[2]],
                                              [bloomPoseWorldFrame4D[3]]]
            # calc the "end point" (bottom-right corner) of the cyanobloom square
            bottom_right_bloomPoseWorldFrame4D = [[bloomPoseWorldFrame4D[0] + 15],
                                                  [bloomPoseWorldFrame4D[1] - 15],
                                                  [bloomPoseWorldFrame4D[2]],
                                                  [bloomPoseWorldFrame4D[3]]]
            top_right_bloomPoseWorldFrame4D = [[bloomPoseWorldFrame4D[0] + 15],
                                               [bloomPoseWorldFrame4D[1] + 15],
                                               [bloomPoseWorldFrame4D[2]],
                                               [bloomPoseWorldFrame4D[3]]]
            # calc the "end point" (bottom-right corner) of the cyanobloom square
            bottom_left_bloomPoseWorldFrame4D = [[bloomPoseWorldFrame4D[0] - 15],
                                                 [bloomPoseWorldFrame4D[1] - 15],
                                                 [bloomPoseWorldFrame4D[2]],
                                                 [bloomPoseWorldFrame4D[3]]]

            # Homogeneous 4D vector of the bloom pose in the camera frame
            top_left_bloomPoseCameraFrame4DVector = np.dot(T_camera_world, top_left_bloomPoseWorldFrame4D)
            bottom_right_bloomPoseCameraFrame4DVector = np.dot(T_camera_world, bottom_right_bloomPoseWorldFrame4D)
            top_right_bloomPoseCameraFrame4DVector = np.dot(T_camera_world, top_right_bloomPoseWorldFrame4D)
            bottom_left_bloomPoseCameraFrame4DVector = np.dot(T_camera_world, bottom_left_bloomPoseWorldFrame4D)

            # Converting 4D vector into a 3D vector
            top_left_BCF = top_left_bloomPoseCameraFrame4DVector[0:3, 0:1]
            bottom_right_BCF = bottom_right_bloomPoseCameraFrame4DVector[0:3, 0:1]
            top_right_BCF = top_right_bloomPoseCameraFrame4DVector[0:3, 0:1]
            bottom_left_BCF = bottom_left_bloomPoseCameraFrame4DVector[0:3, 0:1]

            # -------------- 5. Obtain pinhole camera model from bloom pose in camera frame -------------------

            # parameters for the drone's camera
            alphaXF = 185.69
            x0 = 320.5
            alphaYF = 185.69
            y0 = 180.5

            # get the x and y values of the pixel that corresponds to the bloom's pose in the camera frame
            top_left_xI = ((alphaXF * top_left_BCF[0][0]) / top_left_BCF[0][2]) + x0
            top_left_yI = ((alphaYF * top_left_BCF[0][1]) / top_left_BCF[0][2]) + y0
            bottom_right_xI = ((alphaXF * bottom_right_BCF[0][0]) / bottom_right_BCF[0][2]) + x0
            bottom_right_yI = ((alphaYF * bottom_right_BCF[0][1]) / bottom_right_BCF[0][2]) + y0
            top_right_xI = ((alphaXF * top_left_BCF[0][0]) / top_right_BCF[0][2]) + x0
            top_right_yI = ((alphaYF * top_left_BCF[0][1]) / top_right_BCF[0][2]) + y0
            bottom_left_xI = ((alphaXF * bottom_right_BCF[0][0]) / bottom_left_BCF[0][2]) + x0
            bottom_left_yI = ((alphaYF * bottom_right_BCF[0][1]) / bottom_left_BCF[0][2]) + y0

            pixel_locations = np.array([top_left_xI, top_left_yI, bottom_right_xI, bottom_right_yI,
                                        top_right_xI, top_right_yI, bottom_left_xI, bottom_left_yI])

            # convert to np.int32
            pixel_locations = pixel_locations.astype(np.int32)

            # stack pixel locations to array
            np.vstack((bloomPoseImagePixelArray, pixel_locations))

        # -------------- draw shapes on image and publish -------------------

        # create two copies of the image
        cv_output = cv_image.copy()
        cv_overlay = cv_image.copy()

        # apply each pixel to an overlay
        for pixels in bloomPoseImagePixelArray:
            # reshape into an array of shape ROWSx1x2 (ROWS are number of vertices)
            pixels = pixels.reshape((-1, 1, 2))
            # draw a yellow circle on camera image at the augmented point of bloom
            cv2.fillPoly(cv_overlay, [pixels], (255, 255, 0))

        # apply the overlay
        cv2.addWeighted(cv_overlay, 0.3, cv_output, 0.7, 0, cv_output)

        # cv2.imshow("AR image", cv_output)
        cv2.waitKey(3)

        # Converting back to a ros image and publishing
        msg = self.bridge_object.cv2_to_imgmsg(cv_output, encoding="rgb8")  # converting crop_img from cv to ros msg
        self.image_pub.publish(msg)

        # print("5")

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
    # print("1")
    bloom_filter_object = CameraImageViewer()
    rospy.init_node('bloom_filter_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
