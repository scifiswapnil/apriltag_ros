#!/usr/bin/env python
import rospy
import tf
import time
import socket
import json
import cv2
import numpy as np
from geometry_msgs.msg import Point
from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import TagImageCorner
from image_geometry import PinholeCameraModel
from camera_calibration_parsers import readCalibration

camera_name, camera_info = readCalibration("head_camera.yaml")
camera = PinholeCameraModel()
camera.fromCameraInfo(camera_info)

def callbackAP(data):
    global camera
    global camera_info
    if len(data.detections) > 0:
        p_image = camera.project3dToPixel((data.detections[0].pose.pose.pose.position.x, data.detections[0].pose.pose.pose.position.y,data.detections[0].pose.pose.pose.position.z))
        # print(p_image)
        # quaternion = (data.detections[0].pose.pose.pose.orientation.x, data.detections[0].pose.pose.pose.orientation.y, data.detections[0].pose.pose.pose.orientation.z,data.detections[0].pose.pose.pose.orientation.w)
        # euler = tf.transformations.euler_from_quaternion(quaternion)
        # print data.detections[0].id[0]
        # print data.detections[0].pose.pose.pose.position.x
        # print data.detections[0].pose.pose.pose.position.y
        # print euler[2]
#         a = {
#           "message": data.detections[0].id[0],
#           "orinetation": euler[2],
#           "x": data.detections[0].pose.pose.pose.position.x,
#           "y": data.detections[0].pose.pose.pose.position.y
#         }
#         jsondump = json.dumps(a)
    

def callback(data):
    global camera
    global camera_info
    # print(camera_info.K)
    # print(data.points[0].x)
    xdismtrx = camera_info.D
    cmatrx = np.asarray(camera_info.K)
    cmatrx = cmatrx.reshape(3,3)
    # print (cmatrx) 
    # print (xdismtrx) 
    World = np.array([[-0.015, -0.015,  0.0 ],
                      [ 0.015, -0.015,  0.0 ],
                      [ 0.015,  0.015,  0.0 ],
                      [-0.015,  0. ,  0.0 ]])
    keyPoints = np.array([ [ data.points[1].x,  data.points[1].y ,    1.        ],
                           [ data.points[2].x,  data.points[2].y ,    1.        ],
                           [ data.points[3].x,  data.points[3].y ,    1.        ],
                           [ data.points[0].x,  data.points[0].y ,    1.        ]])

    objectPoints = World
    # imagePoints = keyPoints[:,:2] # <--- THIS SLICE IS A PROBLEM CAUSER!!!
    imagePoints = np.ascontiguousarray(keyPoints[:,:2]).reshape((4,1,2)) # Now OpenCV is HAPPY!
    retval, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, cmatrx, xdismtrx)
    # print (retval)
    # print (rvec)
    print (tvec)
    # # cv2.solvePnP(objectPoints, imagePoints, np.eye(3), np.zeros(5))
    # vec = camera.projectPixelTo3dRay((data.x,data.y))
    # vec = [x * (z/vec[2]) for x in vec]
    # ray_z = [el / vec[2] for el in vec]
    print ()

if __name__ == '__main__' :
    rospy.init_node('pose_generator')
    rospy.loginfo("ready ..!")
    rospy.Subscriber("/tag_center_image_point", TagImageCorner, callback)
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callbackAP)
    rospy.spin()

