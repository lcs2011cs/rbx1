#!/usr/bin/env python

"""
    pos_calculator.py - Version 1.1 2013-12-20
    
    Track 3D position a target published on the /roi topic using depth from the depth image.
    
"""

import rospy
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo, PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Vector3
from math import isnan
import cv2
from cv2 import cv as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from rbx1_vision.msg import *

class PositionCalculator():
    def __init__(self):
        rospy.init_node("pos_calculator")
                        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)

        # How often should we update the robot's motion?
        self.rate = rospy.get_param("~rate", 10)
        r = rospy.Rate(self.rate)
        
        # Scale the ROI by this factor to avoid noisy background distance out of the object
        self.scale_roi = rospy.get_param("~scale_roi", 0.8)
        # The minimum and maximum distance a target can be from the robot for us to track
        self.max_z = rospy.get_param("~max_z", 5000)
        self.min_z = rospy.get_param("~min_z", 100)

        self.object_visible = False
        self.arm_visible = False        
        #position of the object and arm
        self.position = Vector3()
        self.position_arm = Vector3()
        self.direction = Vector3()
        self.pos_pub = rospy.Publisher("direction", Vector3, queue_size=1)
        
        # We will get the image width and height from the camera_info topic
        self.image_width = 0
        self.image_height = 0
        self.camera_matrix = None
        
        # We need cv_bridge to convert the ROS depth image to an OpenCV array
        self.cv_bridge = CvBridge()
        self.depth_array = None
        
        # Wait for the camera_info topic to become available
        rospy.loginfo("Waiting for camera_info topic...")
        rospy.wait_for_message('camera_info', CameraInfo)
        # Subscribe to the camera_info topic to get the image width and height
        rospy.Subscriber('camera_info', CameraInfo, self.get_camera_info, queue_size=1)
        # Wait until we actually have the camera data
        while self.image_width == 0 or self.image_height == 0:
            rospy.sleep(1)
            
        # Wait for the depth image to become available
        rospy.loginfo("Waiting for depth_image topic...")
        rospy.wait_for_message('depth_image', Image)           
        # Subscribe to the depth image
        self.depth_subscriber = rospy.Subscriber("depth_image", Image, self.convert_depth_image, queue_size=1)

        #rospy.loginfo("Waiting for point_cloud topic...")
        #rospy.wait_for_message('point_cloud', PointCloud2)
        #self.point_subscriber = rospy.Subscriber("point_cloud", PointCloud2, self.handle_points, queue_size=1)
        

        rospy.loginfo("Waiting for an ROI to track...")
        # Wait until we have an ROI to follow
        rospy.wait_for_message('object_pos', PosTrack)
        rospy.wait_for_message('arm_pos', PosTrack)
        # Subscribe to the ROI topic and set the callback to update the robot's motion
        rospy.Subscriber('object_pos',PosTrack, self.calculate_pos, queue_size=1)
        rospy.Subscriber('arm_pos', PosTrack, self.calculate_pos_arm, queue_size=1)

        rospy.loginfo("ROI messages detected. Starting Calculation...")
        # Begin the tracking loop
        while not rospy.is_shutdown():
            if self.object_visible and self.arm_visible:
                print self.position_arm.x, self.position_arm.y, self.position_arm.z
                #self.pub_position()
            else:
                print "Arm or Object is not visible."
            
            # Sleep for 1/self.rate seconds
            r.sleep()

    def handle_points(self, msg):

        for point in point_cloud2.read_points(msg, skip_nans = False):
            n += 1
            if(n < (max_y + 1) * 640 and n > min_y * 640):
                m = n % 640
                if(m >= min_x and m <= max_x):
                    if(isnan(point[2])):
                        pass
                    else:
                        if(point[2] > 3.2):
                            l = n / 640
                            r = n % 640
                            self.outlier.append( ((r-min_x) * 1.0/scaled_width, (l-min_y) * 1.0 / scaled_height) )
                        sum_z = sum_z + point[2]
                        npoints += 1.0

                        
    def calculate_pos(self, msg):
        
        # If the ROI has a width or height of 0, we have lost the target
        if msg.width == 0 or msg.height == 0:
            print "Object loses track"
            self.object_visible = False
            return
        else:
            self.object_visible = True

        track_box = ((msg.x,msg.y),((int)(self.scale_roi*msg.width),(int)(self.scale_roi*msg.height)),msg.angle)
        vertices = np.int0(cv2.cv.BoxPoints(track_box))
        if(len(vertices) != 4):
            print "object track box error"
        
        min_x = min_y = 640
        max_x = max_y = 0
        for i in range(0,len(vertices)):
            min_x = min(min_x,vertices[i][0])
            min_y = min(min_y,vertices[i][1])
            max_x = max(max_x,vertices[i][0])
            max_y = max(max_y,vertices[i][1])
        
        sum_z = npoints = 0.0
        for x in range(min_x, max_x):
            for y in range(min_y, max_y):
                if( self.inarea(x,y,vertices) ):
                    try:
                        # Get a depth value in meters
                        z = self.depth_array[y, x]
                    
                        # Check for NaN values returned by the camera driver
                        if isnan(z):
                            continue

                        if(z > self.max_z or z < self.min_z):
                            continue

                        #freenect get z in millimeters
                        sum_z += z / 1000
                        npoints += 1.0

                    except:
                        continue

        if(npoints > 0.5):
            self.position.z = sum_z / npoints
            self.position.x = (1.0 * msg.x - self.camera_matrix[0,2]) * self.position.z / self.camera_matrix[0,0]
            self.position.y = (1.0 * msg.y - self.camera_matrix[1,2]) * self.position.z / self.camera_matrix[1,1]

    def calculate_pos_arm(self, msg):
        # If the ROI has a width or height of 0, we have lost the target
        if msg.width == 0 or msg.height == 0:
            print "Arm loses track"
            self.arm_visible = False
            return
        else:
            self.arm_visible = True

        track_box = ((msg.x,msg.y),((int)(self.scale_roi*msg.width),(int)(self.scale_roi*msg.height)),msg.angle)
        vertices = np.int0(cv2.cv.BoxPoints(track_box))
        if(len(vertices) != 4):
            print "object track box error"
        
        min_x = min_y = 640
        max_x = max_y = 0
        for i in range(0,len(vertices)):
            min_x = min(min_x,vertices[i][0])
            min_y = min(min_y,vertices[i][1])
            max_x = max(max_x,vertices[i][0])
            max_y = max(max_y,vertices[i][1])
        
        sum_z = npoints = 0.0
        for x in range(min_x, max_x):
            for y in range(min_y, max_y):
                if( self.inarea(x,y,vertices) ):
                    try:
                        # Get a depth value in meters
                        z = self.depth_array[y, x]
                    
                        # Check for NaN values returned by the camera driver
                        if isnan(z):
                            continue

                        if(z > self.max_z or z < self.min_z):
                            continue

                        #freenect get z in millimeters
                        sum_z += z / 1000
                        npoints += 1.0

                    except:
                        continue

        if(npoints > 0.5):
            self.position_arm.z = sum_z / npoints
            self.position_arm.x = (1.0 * msg.x - self.camera_matrix[0,2]) * self.position.z / self.camera_matrix[0,0]
            self.position_arm.y = (1.0 * msg.y - self.camera_matrix[1,2]) * self.position.z / self.camera_matrix[1,1]
        

    def inarea(self,x,y,vertices):
        vec1 = (vertices[1][0]-vertices[0][0], vertices[1][1]-vertices[0][1])
        vec2 = (vertices[2][0]-vertices[1][0], vertices[2][1]-vertices[1][1])

        nvec1 = (x-vertices[0][0], y-vertices[0][1])
        nvec2 = (x-vertices[1][0], y-vertices[1][1])

        if(self.dotvalue(nvec1,vec1) >= 0 and self.dotvalue(nvec1,vec1) <= self.dotvalue(vec1,vec1) and self.dotvalue(nvec2,vec2) >= 0 and self.dotvalue(nvec2,vec2) <= self.dotvalue(vec2,vec2) ):
            return True
        else:
            return False


    def dotvalue(self, vec1, vec2):
        return (vec1[0]*vec2[0] + vec1[1]*vec2[1])

    def convert_depth_image(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            # Convert the depth image using the default passthrough encoding
            depth_image = self.cv_bridge.imgmsg_to_cv2(ros_image, "passthrough")
        except CvBridgeError, e:
            print e
        # Convert the depth image to a Numpy array
        self.depth_array = np.array(depth_image, dtype=np.float32)

    def get_camera_info(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height
        P = np.asarray(msg.P)
        self.camera_matrix = np.reshape(P,(3,4))

    def pub_position(self):
        try:
            self.direction.x = self.position.x - self.position_arm.x
            self.direction.y = self.position.y - self.position_arm.y
            self.direction.z = self.position.z - self.position_arm.z
            self.pos_pub.publish(self.direction)
        except:
            rospy.loginfo("Publishing 3D direction Failed")

    def shutdown(self):
        rospy.loginfo("Stopping Calculating the 3D Position...")
        # Unregister the subscriber
        self.depth_subscriber.unregister()
        rospy.sleep(1)      

if __name__ == '__main__':
    try:
        PositionCalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Position Calculator node terminated.")

