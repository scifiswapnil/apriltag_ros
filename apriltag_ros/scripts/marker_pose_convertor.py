#!/usr/bin/env python
import rospy
import tf
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import String
from tf import TransformListener
import threading
from nav_msgs.msg import Odometry

pub = 0
# trans=0
# rot=0

def thread_function(name):
    # global trans,rot
    while not rospy.is_shutdown():
        listener = TransformListener()
        listener.waitForTransform('/odom','/base_link',rospy.Time(), rospy.Duration(0.5))
        (trans, rot) = listener.lookupTransform('/odom', 'base_link', rospy.Time(0))   
        print trans, rot
        throw_data = Odometry()
        # throw_data.header = data.detections[0].pose.header
        throw_data.pose.pose.position.x = trans[0]
        throw_data.pose.pose.position.y = trans[1]
        throw_data.pose.pose.position.z = trans[2]
        throw_data.pose.pose.orientation.x = rot[0] 
        throw_data.pose.pose.orientation.y = rot[1] 
        throw_data.pose.pose.orientation.z = rot[2] 
        throw_data.pose.pose.orientation.w = rot[3] 
        pub.publish(throw_data) 


# def callback(data):
#     global pub
#     number_marker = len(data.detections)
#     for i in range(0,number_marker):
#         if (data.detections[i].id[0] == 1): # check for odometry marker
#             throw_data = Odometry()
#             throw_data.header = data.detections[0].pose.header
#             throw_data.pose = data.detections[0].pose.pose
#             pub.publish(throw_data) 
#             # print trans, rot



    # quaternion = (data.detections[0].pose.pose.pose.orientation.x,data.detections[0].pose.pose.pose.orientation.y,data.detections[0].pose.pose.pose.orientation.z,data.detections[0].pose.pose.pose.orientation.w)
    # if (data.detections[0].id[0] == 1) : 
    #     # rospy.loginfo("got robot marker : ")
    #     throw_data = Odometry()

    #     euler = tf.transformations.euler_from_quaternion(quaternion)
    #     print (str(euler[0]) + "\t"+ str(euler[1]) + "\t"+ str(euler[2]+3.14))
    #     quaternion = tf.transformations.quaternion_from_euler(euler[0],euler[1],euler[2]+3.14)
    #     throw_data.pose.pose.position = data.detections[0].pose.pose.pose.position
    #     throw_data.pose.pose.orientation.z = quaternion[2]
    #     throw_data.pose.pose.orientation.w = quaternion[3]
    #     pub.publish(throw_data)
    # if (data.detections[1].id[0] == 1) : 
    #     # rospy.loginfo("got robot marker : ")
    #     throw_data = Odometry()
    #     # throw_data.pose = data.detections[1].pose.pose
    #     # throw_data.twist = data.detections[0].pose.pose
    #     euler = tf.transformations.euler_from_quaternion(quaternion)
    #     print (str(euler[0]) + "\t"+ str(euler[1]) + "\t"+ str(euler[2]))
    #     throw_data.pose.pose.position = data.detections[0].pose.pose.pose.position
    #     throw_data.pose.pose.orientation.z = quaternion[2]
    #     throw_data.pose.pose.orientation.w = quaternion[3]
    #     pub.publish(throw_data)
    

if __name__ == '__main__' :
    pub = rospy.Publisher('/marker_pose_odom', Odometry, queue_size=1)
    rospy.init_node('odom_pose_generator')
    # rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback)
    x = threading.Thread(target=thread_function, args=(1,))
    x.start()
    rospy.spin()