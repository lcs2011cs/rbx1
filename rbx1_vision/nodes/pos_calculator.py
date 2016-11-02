#!/usr/bin/env python

"""
    pos_calculator.py - Version 1.1 2013-12-20
    
    Track 3D position a target published on the /roi topic using depth from the depth image.
    
"""

import rospy
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from geometry_msgs.msg import Vector3
from math import isnan,sqrt
from cv2 import cv as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class PositionCalculator():
    def __init__(self):
        rospy.init_node("pos_calculator")
                        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        self.flag = True
        # How often should we update the robot's motion?
        self.rate = rospy.get_param("~rate", 10)
        r = rospy.Rate(self.rate)
        
        # Scale the ROI by this factor to avoid background distance values around the edges
        self.scale_roi = rospy.get_param("~scale_roi", 0.6)
                # The maximum distance a target can be from the robot for us to track
        self.max_z = rospy.get_param("~max_z", 4000)
        self.min_z = rospy.get_param("~min_z", 1000)
        self.roi_visible = False
        self.roi_arm_visible = False
        
        # Initialize the global ROI
        self.roi = RegionOfInterest()
        self.roi_arm = RegionOfInterest()
        self.position = Vector3()
        self.position_arm = Vector3()
        self.direction = Vector3()
        self.distance = 0.0
        
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
        
        # Subscribe to the ROI topic and set the callback to update the robot's motion
        rospy.Subscriber('roi', RegionOfInterest, self.calculate_pos, queue_size=1)
        rospy.Subscriber('roi_arm', RegionOfInterest, self.calculate_pos_arm, queue_size=1)
        
        # Wait until we have an ROI to follow
        rospy.loginfo("Waiting for an ROI to track...")
        rospy.wait_for_message('roi', RegionOfInterest)
        rospy.wait_for_message('roi_arm', RegionOfInterest)

        rospy.loginfo("ROI messages detected. Starting Calculation...")


        # Begin the tracking loop
        while not rospy.is_shutdown():
            
            if(self.roi_visible and self.roi_arm_visible):
                #self.direction.x = self.position.x - self.position_arm.x
                #self.direction.y = self.position.y - self.position_arm.y
                #self.direction.z = self.position.z - self.position_arm.z
                #self.distance = sqrt(self.direction.x**2 + self.direction.y**2 + self.direction.z**2)
                print self.position.x, self.position.y, self.position.y 
            else:
                print "Arm or Object is not visible."
            
            # Sleep for 1/self.rate seconds
            r.sleep()
                        
    def calculate_pos(self, msg):
        
        # If the ROI has a width or height of 0, we have lost the target
        if msg.width == 0 or msg.height == 0:
            print "Object is not visible now"
            self.roi_visible = False
            return
        else:
            self.roi_visible = True

        self.roi = msg

        sum_x = sum_y = sum_z = npoints = 0.0
             
        # Shrink the ROI to try to only care about the object and avoid the noise
        scaled_width = int(self.roi.width * self.scale_roi)
        scaled_height = int(self.roi.height * self.scale_roi)
            
        # Get the min/max x and y values from the scaled ROI
        min_x = int(self.roi.x_offset + self.roi.width * (1.0 - self.scale_roi) / 2.0)
        max_x = min_x + scaled_width
        min_y = int(self.roi.y_offset + self.roi.height * (1.0 - self.scale_roi) / 2.0)
        max_y = min_y + scaled_height
            
        # Get the average depth value over the ROI
        for x in range(min_x, max_x):
            for y in range(min_y, max_y):
                try:
                    # Get a depth value in meters
                    z = self.depth_array[y, x]
                    
                    # Check for NaN values returned by the camera driver
                    if isnan(z):
                        continue
                                               
                except:
                    # It seems to work best if we convert exceptions to 0
                    continue
                    
                # A hack to convert millimeters to meters for the freenect driver
                    
                # Check for values outside max range
                if z > self.max_z or z < self.min_z:
                    continue
                
                # Get the 3D coordinates of thw world
                z_w = z / 1000.0
                x_w = (x + self.roi.width / 2 - self.camera_matrix[0,2]) * z_w / self.camera_matrix[0,0]
                y_w = (y + self.roi.height / 2 - self.camera_matrix[1,2]) * z_w / self.camera_matrix[1,1]
                
                # Increment the sum and count
                npoints += 1.0
                sum_z = sum_z + z_w
                sum_x = sum_x + x_w
                sum_y = sum_y + y_w

        # Get the center of the object
        if npoints > 1.0:
            self.position.z = sum_z / npoints
            self.position.x = sum_x / npoints
            self.position.y = sum_y / npoints

    def calculate_pos_arm(self, msg):
        
        # If the ROI has a width or height of 0, we have lost the target
        if msg.width == 0 or msg.height == 0:
            print "Arm is not visible now"
            self.roi_arm_visible = False
            return
        else:
            self.roi_arm_visible = True

        self.roi_arm = msg

        sum_x = sum_y = sum_z = npoints = 0.0
             
        # Shrink the ROI to try to only care about the object and avoid the noise
        scaled_width = int(self.roi_arm.width * self.scale_roi)
        scaled_height = int(self.roi_arm.height * self.scale_roi)
            
        # Get the min/max x and y values from the scaled ROI
        min_x = int(self.roi_arm.x_offset + self.roi_arm.width * (1.0 - self.scale_roi) / 2.0)
        max_x = min_x + scaled_width
        min_y = int(self.roi_arm.y_offset + self.roi_arm.height * (1.0 - self.scale_roi) / 2.0)
        max_y = min_y + scaled_height
            
        # Get the average depth value over the ROI
        for x in range(min_x, max_x):
            for y in range(min_y, max_y):
                try:
                    # Get a depth value in meters
                    z = self.depth_array[y, x]
                    
                    # Check for NaN values returned by the camera driver
                    if isnan(z):
                        continue
                                               
                except:
                    # It seems to work best if we convert exceptions to 0
                    continue
                    
                # A hack to convert millimeters to meters for the freenect driver
                    
                # Check for values outside max range
                if z > self.max_z or z < self.min_z:
                    continue
                
                # Increment the sum and count

                z_w = z / 1000.0
                x_w = (x + self.roi.width / 2 - self.camera_matrix[0,2]) * z_w / self.camera_matrix[0,0]
                y_w = (y + self.roi.height / 2 - self.camera_matrix[1,2]) * z_w / self.camera_matrix[1,1]

                npoints += 1.0
                sum_z = sum_z + z_w
                sum_x = sum_x + x_w
                sum_y = sum_y + y_w

        # Calculate the center of the arm
        if npoints > 1.0:
            self.position_arm.z = sum_z / npoints
            self.position_arm.x = sum_x / npoints
            self.position_arm.y = sum_y / npoints

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

