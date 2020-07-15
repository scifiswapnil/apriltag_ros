#!/usr/bin/env python
import rospy
import tf
import time
import socket
import json
from apriltag_ros.msg import AprilTagDetectionArray
localIP     = "localhost"
localPort   = 20001
bufferSize  = 1024

server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)



def callback(data):
    if len(data.detections) > 0:
        quaternion = (data.detections[0].pose.pose.pose.orientation.x, data.detections[0].pose.pose.pose.orientation.y, data.detections[0].pose.pose.pose.orientation.z,data.detections[0].pose.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        print data.detections[0].id[0]
        print data.detections[0].pose.pose.pose.position.x
        print data.detections[0].pose.pose.pose.position.y
        print euler[2]
        a = {
          "message": data.detections[0].id[0],
          "orinetation": euler[2],
          "x": data.detections[0].pose.pose.pose.position.x,
          "y": data.detections[0].pose.pose.pose.position.y
        }
        jsondump = json.dumps(a)
        server.sendto(jsondump, ('<broadcast>', 37020))
    
        

if __name__ == '__main__' :
    rospy.init_node('pose_generator')
    rospy.loginfo("ready ..!")
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback)
    rospy.spin()

