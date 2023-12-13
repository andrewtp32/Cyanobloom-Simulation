#!/usr/bin/env python
import rospy
from datetime import datetime
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import scipy
import std_msgs.msg
from scipy.spatial.transform import Rotation as R
from tf.transformations import euler_from_quaternion


class BloomLocator(object):
    def __init__(self):
        print("2")
        self.px, self.py, self.pz, self.ox, self.oy, self.oz, self.ow = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        self.orientation_q = []
        self.image_sub = rospy.Subscriber("/drone/front_camera/image_raw/bloom", Image, self.camera_callback)
        self.pose_sub = rospy.Subscriber("/drone_0/gt_pose", Pose, self.pose_callback)
        self.bridge_object = CvBridge()
        self.image_pub = rospy.Publisher("/drone/front_camera/image_raw/bloom_locator", Image, queue_size=1)
        self.pointcloud_pub = rospy.Publisher("/bloom_locations_topic", PointCloud)
        print("3")

    def camera_callback(self, Image):  # "data" is the image coming from the callback
        print("4")

        # Converting drone's filtered camera image to a CV image
        img = self.bridge_object.imgmsg_to_cv2(Image, desired_encoding="passthrough")

        # -------------- 1. Preform homogeneous transformations from camera frame to the world frame -------------------

        # -------------- 1a. Get translation matrix from world frame to drone frame -------------------

        # gather quaternion of the drone in the world frame
        quaternionWorldToDrone = R.from_quat([self.ox, self.oy, self.oz, self.ow])
        # convert the quaternion into a 3D rotation matrix from world to drone
        rotationMatrixWorldToDrone = quaternionWorldToDrone.as_dcm()
        # XYZ position vector of the drone in the world frame
        dronePoseWorldFrame = np.array([[self.px],
                                        [self.py],
                                        [self.pz]])
        # print("Drone position vector (world frame): ", dronePoseWorldFrame)
        # print("Drone rotation matrix: ", rotationMatrixWorldToDrone)
        # T is the homogeneous transformation matrix. This is the T from the world to the drone.
        # T is a 4x4 matrix.
        T_world_drone = np.concatenate((rotationMatrixWorldToDrone, dronePoseWorldFrame), 1)
        T_world_drone = np.concatenate((T_world_drone, [[0, 0, 0, 1]]), 0)

        # -------------- 1b. Get translation matrix from drone frame to camera frame -------------------

        # Roll, pitch, and yaw angles from drone frame to camera frame
        rotationMatrixDroneToCamera = np.array([[0, -1, 0],
                                                [-1, 0, 0],
                                                [0, 0, -1]])
        # XYZ position vector od the camera in the drone frame
        cameraPoseDroneFrame = [[0.0],
                                [0.0],
                                [0.0]]
        # print("Camera position vector (Drone frame): ", cameraPoseDroneFrame)
        # print("Camera rotation matrix: ", rotationMatrixDroneToCamera)
        # T is the translation matrix. This is the T from the drone to the camera.
        # T is a 4x4 matrix.
        T_drone_camera = np.concatenate((rotationMatrixDroneToCamera, cameraPoseDroneFrame), 1)
        T_drone_camera = np.concatenate((T_drone_camera, [[0, 0, 0, 1]]), 0)

        # -------------- 1c. Get transformation matrix from world to camera -------------------

        # T is the translation matrix. This is the T from the world to the camera frame
        T_world_camera = np.dot(T_world_drone, T_drone_camera)
        # Obtain inverse of the T_world_camera
        T_camera_world = np.linalg.inv(T_world_camera)

        # print("HTM from world to camera: ", T_world_camera)

        # -------------- 2. Use OpenCV to determine the X,Y position of the bloom in the camera image -------------------

        # show untouched image
        # cv2.imshow("Raw image", img)
        # Initialize a new image (used in line 115)
        imgWithBoundingRectangles = img.copy()
        # Initialize a new image (used in line 102)
        imgWithDrawnContours = img.copy()
        imgWithTracedPoints = img.copy()
        # Converting RGB to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Bloom color is yellow. In RGB, the color is (255, 255, 0)
        # Define range of yellow color in HSV
        lower_yellow = np.array([100, 30, 50])
        upper_yellow = np.array([140, 255, 255])

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        # cv2.imshow("Mask Image", mask)

        # Circumscribe the perimeter of each shape
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                          cv2.CHAIN_APPROX_SIMPLE)  # cv2.RETR_EXTERNAL is good for finding outer corners of canny image
        # Draw the contour lines onto "imgWithDrawnContours"
        cv2.drawContours(imgWithDrawnContours, contours, -1, (255, 0, 0), 2)

        # # Initialize list of center points
        # centerPoints = []
        #
        # # This for-loop's purpose is to isolate each bloom and calculate the center point
        # for cnt in contours:
        #     area = cv2.contourArea(cnt)  # calculate area of the given shape
        #     if area > 10:
        #         peri = cv2.arcLength(cnt, True)
        #         approx = cv2.approxPolyDP(cnt, 0.02*peri, True)  # count the amount of corners
        #         # object_corners = len(approx)  # count the amount of corners
        #         x, y, width, height = cv2.boundingRect(approx)  # create an imaginary rectangle around the shape
        #         cv2.rectangle(imgWithBoundingRectangles, (x,y), (x+width, y+height),(0, 0, 255), 2) # draw a rectangle about each detected shape onto "imgWithBoundingRectangles"
        #         point = [int(x+(width/2)), int(y+(height/2))] # calculate center point
        #         centerPoints.append(point)

        # distance between the drone and the water surface
        droneHeight = self.pz
        # Camera parameter for the focal in the x direction
        cameraAlphaXF = 185.69
        # The number of meters we want between each traced point
        tracedBloomDistanceMeters = 2

        # The number of pixels we want between each traced point
        tracedBloomDistancePixels = (cameraAlphaXF / droneHeight) * tracedBloomDistanceMeters

        # get all of the points that are within the calculated contours
        pointsWithinContoursList = []
        range_x = range(0, 640, int(tracedBloomDistancePixels))
        range_y = range(0, 360, int(tracedBloomDistancePixels))
        for cnt in contours:
            for x in range_x:
                for y in range_y:
                    result = cv2.pointPolygonTest(cnt, (x, y), False)
                    if result == 1.0:
                        pointsWithinContoursList.append([x, y])
                        cv2.circle(imgWithTracedPoints, (x, y), 2, (0, 0, 255), -1)

        # cv2.imshow("Image with traced points", imgWithTracedPoints)

        # cv2.imshow("Drawn Contours", imgWithDrawnContours)
        # cv2.imshow("Drawn Rectangles", imgWithBoundingRectangles)
        cv2.waitKey(3)

        # -------------- 3. Convert each X,Y point in the image into X,Y,Z coordinates with respect to the camera -------------------

        # parameters for the drone's camera
        alphaXF = 185.69
        x0 = 320.5
        alphaYF = 185.69
        y0 = 180.5
        cameraImageCornerPointsList = [[0, 0], [640, 0], [0, 360], [640, 360]]

        # Initialize list for all the corners of the camera image in the camera frame
        cornerPointsCameraFrame4DList = []
        # Initialize list for all the blooms' poses in the camera frame
        bloomPoseCameraFrame4DList = []

        cornerPoseCameraFrameZ = self.pz
        bloomPoseCameraFrameZ = self.pz

        # convert each corner point xy into xyz (camera frame) and then place into a 4x1 column vector
        for cornerPoseImagePoint in cameraImageCornerPointsList:
            cornerPoseImagePointX = cornerPoseImagePoint[0]
            cornerPoseImagePointY = cornerPoseImagePoint[1]

            cornerPoseCameraFrameX = (cornerPoseImagePointX - x0) * cornerPoseCameraFrameZ / alphaXF
            cornerPoseCameraFrameY = (cornerPoseImagePointY - y0) * cornerPoseCameraFrameZ / alphaYF

            # Create a vector representing the bloom's pose in the camera frame. The last row in the vector is necessary for translation later on.
            pose = [[cornerPoseCameraFrameX],
                    [cornerPoseCameraFrameY],
                    [cornerPoseCameraFrameZ],
                    [1]]
            cornerPointsCameraFrame4DList.append(pose)

        # convert each bloom xy into xyz (camera frame) and then place into a 4x1 column vector
        for bloomPoseImagePoint in pointsWithinContoursList:
            bloomPoseImagePointX = bloomPoseImagePoint[0]
            bloomPoseImagePointY = bloomPoseImagePoint[1]

            bloomPoseCameraFrameX = (bloomPoseImagePointX - x0) * bloomPoseCameraFrameZ / alphaXF
            bloomPoseCameraFrameY = (bloomPoseImagePointY - y0) * bloomPoseCameraFrameZ / alphaYF

            # Create a vector representing the bloom's pose in the camera frame. The last row in the vector is necessary for translation later on.
            pose = [[bloomPoseCameraFrameX],
                    [bloomPoseCameraFrameY],
                    [bloomPoseCameraFrameZ],
                    [1]]
            bloomPoseCameraFrame4DList.append(pose)

        # print("Each bloom's X,Y,Z position in the camera frame: ", bloomPoseCameraFrame4DList)

        # -------------- 4. Compute the X,Y,Z of the bloom in the world frame -------------------

        # Declare lists
        bloomPoseDroneFrame4DList = []
        bloomPoseWorldFrame4DList = []
        cornerPoseDroneFrame4DList = []
        cornerPoseWorldFrame4DList = []

        for cornerPoseCameraFrame4D in cornerPointsCameraFrame4DList:
            # Homogeneous 4D vector of the bloom position in the drone frame
            cornerPoseDroneFrame4DVector = np.dot(T_drone_camera, cornerPoseCameraFrame4D)
            cornerPoseDroneFrame4DList.append(cornerPoseDroneFrame4DVector)
            # Homogeneous 4D vector of the bloom position in the world frame
            cornerPoseWorldFrame4DVector = np.dot(T_world_drone, cornerPoseDroneFrame4DVector)
            cornerPoseWorldFrame4DList.append(cornerPoseWorldFrame4DVector)

        for bloomPoseCameraFrame4D in bloomPoseCameraFrame4DList:
            # Homogeneous 4D vector of the bloom position in the drone frame
            bloomPoseDroneFrame4DVector = np.dot(T_drone_camera, bloomPoseCameraFrame4D)
            bloomPoseDroneFrame4DList.append(bloomPoseDroneFrame4DVector)
            # Homogeneous 4D vector of the bloom position in the world frame
            bloomPoseWorldFrame4DVector = np.dot(T_world_drone, bloomPoseDroneFrame4DVector)
            bloomPoseWorldFrame4DList.append(bloomPoseWorldFrame4DVector)

        # print("Each corner of the camera's X,Y,Z position in the drone frame: ", cornerPoseDroneFrame4DList)
        # print("Each corner of the camera's X,Y,Z position in the world frame: ", cornerPoseWorldFrame4DList)
        # print("Each bloom's X,Y,Z position in the drone frame: ", bloomPoseDroneFrame4DList)
        # print("Each bloom's X,Y,Z position in the world frame: ", bloomPoseWorldFrame4DList)

        # -------------- 5. Convert the 4D lists of column vectors into a 3D list of row vectors -------------------

        # Declare new 3D lists
        cornerPoseWorldFrame3DList = []
        bloomPoseWorldFrame3DList = []

        for cornerPoseWorldFrame4DColumnVector in cornerPoseWorldFrame4DList:
            # transpose column vector into row vector
            cornerPoseWorldFrame4DRowVector = np.transpose(cornerPoseWorldFrame4DColumnVector)
            # Remove the last element of the 4D vector. The vector is now 3D.
            cornerPoseWorldFrame3DRowVector = cornerPoseWorldFrame4DRowVector[0, 0:3]
            # print(cornerPoseWorldFrame3DRowVector)
            cornerPoseWorldFrame3DList.append(cornerPoseWorldFrame3DRowVector)

        for bloomPoseWorldFrame4DColumnVector in bloomPoseWorldFrame4DList:
            # transpose column vector into row vector
            bloomPoseWorldFrame4DRowVector = np.transpose(bloomPoseWorldFrame4DColumnVector)
            # Remove the last element of the 4D vector. The vector is now 3D.
            bloomPoseWorldFrame3DRowVector = bloomPoseWorldFrame4DRowVector[0, 0:3]
            # print(bloomPoseWorldFrame3DRowVector)
            bloomPoseWorldFrame3DList.append(bloomPoseWorldFrame3DRowVector)

        # print("Checking to see if the 4D columns got changed to rows: ", cornerPoseWorldFrame3DList)

        # -------------- 6. Declaring and filling PointCloud -------------------

        # Declare PointCloud
        coordinate_pointcloud = PointCloud()

        # Filling PointCloud header
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        coordinate_pointcloud.header = header

        print("number of corners in list: ", len(cornerPoseWorldFrame3DList))
        print("number of blooms in list: ", len(bloomPoseWorldFrame3DList))

        # Place the tracked points and the four corner locations into pointclouds
        for corner in cornerPoseWorldFrame3DList:
            coordinate_pointcloud.points.append(Point32(corner[0], corner[1], corner[2]))
        for bloom in bloomPoseWorldFrame3DList:
            coordinate_pointcloud.points.append(Point32(bloom[0], bloom[1], bloom[2]))
        # # Filling in some arbitrary points for testing
        # coordinate_pointcloud.points.append(Point32(1.0, 1.0, 0.0))
        # coordinate_pointcloud.points.append(Point32(2.0, 2.0, 0.0))
        # coordinate_pointcloud.points.append(Point32(3.0, 3.0, 0.0))S
        # print("Number of points in pointcloud: ", coordinate_pointcloud)

        # -------------- 7. Publish messages to topics -------------------

        # Converting back to a ros image and publishing
        msg = self.bridge_object.cv2_to_imgmsg(mask, encoding="8UC1")  # converting crop_img from cv to ros msg
        self.image_pub.publish(msg)

        # Publishing point cloud to /bloom_locations_topic
        self.pointcloud_pub.publish(coordinate_pointcloud)

        print("5")

    def pose_callback(self, Pose):
        # Gathering pose information about the drone
        self.px = Pose.position.x
        self.py = Pose.position.y
        self.pz = Pose.position.z
        self.ox = Pose.orientation.x
        self.oy = Pose.orientation.y
        self.oz = Pose.orientation.z
        self.ow = Pose.orientation.w


def main():
    print("1")
    bloom_locator = BloomLocator()
    rospy.init_node('bloom_locator', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
