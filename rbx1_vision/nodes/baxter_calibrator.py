#!/usr/bin/env python
import argparse
import sys
import csv
import numpy as np
from copy import copy
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo, PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Vector3
import rospy
 
import actionlib
 
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)
 
import baxter_interface
from baxter_core_msgs.msg import EndpointState
from baxter_interface import CHECK_VERSION

class BaxterCalibrator():
    def __init__(self):
        rospy.init_node("baxter_calib_kinect")
        #rospy.on_shutdown(self.shutdown)
                # How often should we update the robot's motion?
        self.rate = rospy.get_param("~rate", 5)
        r = rospy.Rate(self.rate)

        self.pos_baxter = Vector3()
        self.pos_kinect = Vector3()

        self.bax_write_data = np.array([0,0,0])
        self.kin_write_data = np.array([0,0,0])

        rospy.loginfo("Waiting for position from Kinect...")
        rospy.wait_for_message('/pos_2', Vector3 )


        rospy.loginfo("Waiting for position from Baxter Positioning...")
        rospy.wait_for_message('/robot/limb/right/endpoint_state', EndpointState )

        while not rospy.is_shutdown():
            
            rospy.Subscriber('/pos_2', Vector3, self.get_pos_kinect, queue_size=1)            
            rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState, self.get_pos_baxter, queue_size=1)

            rospy.loginfo('The baxter pos is {0} and \n Kinect pos is {1}'.format(self.pos_baxter, self.pos_kinect))

            #Write to CSV file
            with open('pos_baxter_2.csv','a') as f1:
                writer = csv.writer(f1)
                writer.writerow(self.bax_write_data)

            with open('pos_kinect_2.csv','a') as f2:
                writer = csv.writer(f2)
                writer.writerow(self.kin_write_data)

            r.sleep()

    def get_pos_kinect(self,data):
        x = data.x
        y = data.y
        z = data.z

        self.pos_kinect = Vector3(x,y,z)

        self.kin_write_data = np.array([x,y,z])

        # #Write to CSV file
        # with open('pos_baxter_2.csv','a') as f:
        #     writer = csv.writer(f)
        #     writer.writerow(self.kin_write_data)



    def get_pos_baxter(self,data):
        x = data.pose.position.x
        y = data.pose.position.y
        z = data.pose.position.z

        self.pos_baxter = Vector3(x,y,z)

        #Write to CSV file
        self.bax_write_data = np.array([x,y,z])
        
        # with open('pos_baxter_2.csv','a') as f1:
        #     writer = csv.writer(f1)
        #     writer.writerow(self.bax_write_data)


        #rospy.Subscriber('/pos_2', Vector3, self.get_pos_kinect, queue_size=1)


if __name__ == '__main__':
    try:
        BaxterCalibrator()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Position Calculator node terminated.")
            
                        