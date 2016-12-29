#!/usr/bin/env python

"""
    pos_calculator.py - Version 1.1 2013-12-20
    
    Get the Tracking ROI info of Object and 3rd Arm published by camshift(2) nodes
    
    Calculate the 3D corrdinates of them and do some denoising 
    
    Keep track of:
    	1. the vector pointing from 3rd Arm to Object (self.direction) and publish it to /direction topic
    	2. the moving vector(the tracks) of the 3rd Arm(self.moving_direction), you could use it later
    
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
        # The minimum and maximum distance a target(object or arm) can by for us to track
        self.max_z = rospy.get_param("~max_z", 5000)
        self.min_z = rospy.get_param("~min_z", 100)
        # If some points' z coordinates deviate more than this threshold from mean, we will drop them and recalculate the mean 
        self.dev_threshold = rospy.get_param("~dev_threshold", 0.1)

        # Whether we are tracking the object and arm or not
        self.object_visible = False
        self.arm_visible = False        
        # Position of the object and arm
        self.position = Vector3()
        self.position_arm = Vector3()

        # Previous position of the 3rd arm used for recording the moving track of it
        self.pre_position_arm = Vector3()
        self.pre_position_arm.x = 0.0
        self.pre_position_arm.y = 0.0
        self.pre_position_arm.z = 0.0

        self.moving_direction = Vector3()
        self.direction = Vector3()
        
        # Vector from 3rd Arm pointing to the Object. Unit is meter.
        self.pos_pub = rospy.Publisher("direction", Vector3, queue_size=1)
        
        # We will get the image width and height from the camera_info topic
        self.image_width = 0
        self.image_height = 0

        # The camera projection matrix. The default or calibrated one.
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

        # Subscribe to the depth image, the depth value is just the z corrdinates
        # The unit is millimeter or meter as you may use different drivers.
        # Here we use freenect, and the depth image is gotten from depth_registered topic, so the unit is millimeter
        self.depth_subscriber = rospy.Subscriber("depth_image", Image, self.convert_depth_image, queue_size=1)

        #rospy.loginfo("Waiting for point_cloud topic...")
        #rospy.wait_for_message('point_cloud', PointCloud2)
        #self.point_subscriber = rospy.Subscriber("point_cloud", PointCloud2, self.handle_points, queue_size=1)
        

        rospy.loginfo("Waiting for an ROI to track...")
        # Wait until we have an ROI to follow
        rospy.wait_for_message('object_pos', PosTrack)
        rospy.wait_for_message('arm_pos', PosTrack)
        # Subscribe to the ROI topic and set the callback to update the corrdinates of the object and arm
        rospy.Subscriber('object_pos',PosTrack, self.calculate_pos, queue_size=1)
        rospy.Subscriber('arm_pos', PosTrack, self.calculate_pos_arm, queue_size=1)

        rospy.loginfo("ROI messages detected. Starting Calculation...")
        
        cnt = 0
        # Begin the tracking loop
        while not rospy.is_shutdown():

        	# When we have both tracking info for arm and object, publish their corrdinates info we needed
            if self.object_visible and self.arm_visible:
            	# Update the moving track of the arm every 10*1/self.rate seconds
            	cnt += 1
            	if(cnt > 10):
        			self.moving_direction.x = self.position_arm.x - self.pre_position_arm.x
        			self.moving_direction.y = self.position_arm.y - self.pre_position_arm.y
        			self.moving_direction.z = self.position_arm.z - self.pre_position_arm.z
        			
        			self.pre_position_arm.x = self.position_arm.x
        			self.pre_position_arm.y = self.position_arm.y
        			self.pre_position_arm.z = self.position_arm.z
        			cnt = 0
        		# Publish the vector (3rd arm -> object)
                self.pub_position()
            else:
                print "Arm or Object is not visible."
            
            # Sleep for 1/self.rate seconds
            r.sleep()

    # You could also use pointcloud to get the corrdinates, just another way to get corrdinates
    # Currently we don't use this one
    def handle_points(self, msg):

        n = npoints = sum_z = 0
        for point in point_cloud2.read_points(msg, skip_nans = False):
            n += 1
            x = n % 640
            y = n / 640

            if(self.inarea(x,y,self.obj_vertices)):
                if(isnan(point[2])):
                    pass
                else:
                    npoints += 1.0
                    sum_z += point[2] * 1.0
        if(npoints > 0.5):
            self.position.z = sum_z / npoints  
    
    # Calculate the 3D corrdinates of the object                
    def calculate_pos(self, msg):
        
        # If the width or height of ROI is 0, we have lost the tracking info of arm
        if msg.width == 0 or msg.height == 0:
            print "Object loses track"
            self.object_visible = False
            return
        else:
            self.object_visible = True

 		# Get four vertices of the rotated rectangle
        track_box = ((msg.x,msg.y),((int)(self.scale_roi*msg.width),(int)(self.scale_roi*msg.height)),msg.angle)
        vertices = np.int0(cv2.cv.BoxPoints(track_box))
        if(len(vertices) != 4):
            print "object track box error"
        
        # Since we want to get every z-value within the rotated rectangle, we first decide the range of x and y we need to iterate
        min_x = min_y = 640
        max_x = max_y = 0
        for i in range(0,len(vertices)):
            min_x = min(min_x,vertices[i][0])
            min_y = min(min_y,vertices[i][1])
            max_x = max(max_x,vertices[i][0])
            max_y = max(max_y,vertices[i][1])
        
        zlist = list()

        for x in range(min_x, max_x):
            for y in range(min_y, max_y):

            	# Judge whether current (x,y) is in our rotated rectangle
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
                        zlist.append(z / 1000.0)

                    except:
                        continue

        # zarr is all the depth values within the tracking rotated rectangle
        # Note the not all these pixels is belonging to the object as the object we track can be any shape, but the convex hull we use is limited as rotated rectangle
        # So drop those pixels which z values are far from mean and calculate the mean again
        # You could do it of many iterations until you think it reachs convergence.
        zarr = np.array(zlist);
        zdev = abs(zarr - np.mean(zarr))
        zindex = np.argsort(zdev)

        # Drop the coordinates which are far away from mean and calculate the mean again.
        # Increase the accurancy of z-corrdinate
        npoints = sum_z = 0.0
        for i  in range(0,len(zindex)):
        	if(zdev[zindex[i]] > self.dev_threshold or i > self.scale_roi*len(zdev)):
        		break
        	else:
        		npoints += 1.0
        		sum_z += zarr[zindex[i]]

        # Get the 3D corrdinates using z value and camera projection matrix
        # Note the 3D corrdinates is within camera corrdinate system. If you want to use another corrdinate system, do transformations. 
        if(npoints > 0.5):
            self.position.z = sum_z / npoints
            self.position.x = (1.0 * msg.x - self.camera_matrix[0,2]) * self.position.z / self.camera_matrix[0,0]
            self.position.y = (1.0 * msg.y - self.camera_matrix[1,2]) * self.position.z / self.camera_matrix[1,1]

    # Calculate the 3D corrdinates of the object, almost same as calculate_pos
    def calculate_pos_arm(self, msg):
        # If the width or height of ROI is 0, we have lost the tracking info of arm
        if msg.width == 0 or msg.height == 0:
            print "Arm loses track"
            self.arm_visible = False
            return
        else:
            self.arm_visible = True

        track_box = ((msg.x,msg.y),((int)(self.scale_roi*msg.width),(int)(self.scale_roi*msg.height)),msg.angle)
        vertices = np.int0(cv2.cv.BoxPoints(track_box))
        if(len(vertices) != 4):
            print "arm track box error"
        
        min_x = min_y = 640
        max_x = max_y = 0
        for i in range(0,len(vertices)):
            min_x = min(min_x,vertices[i][0])
            min_y = min(min_y,vertices[i][1])
            max_x = max(max_x,vertices[i][0])
            max_y = max(max_y,vertices[i][1])
        
        zlist = list()

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
                        zlist.append(z / 1000.0)

                    except:
                        continue

        zarr = np.array(zlist);
        zdev = abs(zarr - np.mean(zarr))
        zindex = np.argsort(zdev)

        npoints = sum_z = 0.0
        for i  in range(0,len(zindex)):
        	if(zdev[zindex[i]] > self.dev_threshold or i > self.scale_roi*len(zdev)):
        		break
        	else:
        		npoints += 1.0
        		sum_z += zarr[zindex[i]]


        if(npoints > 0.5):
            self.position_arm.z = sum_z / npoints
            self.position_arm.x = (1.0 * msg.x - self.camera_matrix[0,2]) * self.position.z / self.camera_matrix[0,0]
            self.position_arm.y = (1.0 * msg.y - self.camera_matrix[1,2]) * self.position.z / self.camera_matrix[1,1]
        
    # Judge whether a 2D points is within a rotated given rectangle 
    def inarea(self,x,y,vertices):
        vec1 = (vertices[1][0]-vertices[0][0], vertices[1][1]-vertices[0][1])
        vec2 = (vertices[2][0]-vertices[1][0], vertices[2][1]-vertices[1][1])

        nvec1 = (x-vertices[0][0], y-vertices[0][1])
        nvec2 = (x-vertices[1][0], y-vertices[1][1])

        if(self.dotvalue(nvec1,vec1) >= 0 and self.dotvalue(nvec1,vec1) <= self.dotvalue(vec1,vec1) and self.dotvalue(nvec2,vec2) >= 0 and self.dotvalue(nvec2,vec2) <= self.dotvalue(vec2,vec2) ):
            return True
        else:
            return False

    # Calculate the dot product of two vectors
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

