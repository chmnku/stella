#! /usr/bin/env python3

import time
import rospy
import cv2
import os

import sys, select, termios, tty

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from ji_func_0109 import Orchestrator


def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def testDrive():
    rospy.loginfo("testDrive")

    global settings
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('stella_line_tracking_node')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)

    twist = Twist()

    for _ in range(3):

        twist.linear.x = 0.3
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        pub.publish(twist)

        rate.sleep()
        pass

    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0

    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0

    pub.publish(twist)




def drive():
    angle_list = []

    global settings
    settings = termios.tcgetattr(sys.stdin)

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,160)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,120)

    frame_width = int(cap.get(3))
    frame_height = int(cap.get(4))
    # fourcc = cv2.VideoWriter_fourcc('D','I','V','X')
    out = cv2.VideoWriter('/home/odroid/catkin_ws/src/STELLA_ODROID_C4/stella_line_tracking/src/SaveVideo.avi', cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width, frame_height))

    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    
    rospy.init_node('stella_line_tracking_node')
    bridge = CvBridge()
    pub1 = rospy.Publisher('stella_line_tracking', Image, queue_size=1)
    pub2 = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)
    
    try:
        
        while not rospy.is_shutdown():
            # rospy.loginfo("run")
            # print('run')

            twist = Twist()

            # begin = time.time()

            ret,frame = cap.read()
            
            if not ret:
                rospy.loginfo("Not Found Devices")

                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.linear.z = 0.0

                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = 0.0

                pub2.publish(twist)

                break

            final_frame, angle = Orchestrator(frame)
            image_msg = bridge.cv2_to_imgmsg(final_frame,"bgr8")
            # cv2.imshow('final_frame', final_frame)
            print('angle: ',angle)

            # save frame to file..
            out.write(final_frame)
            
            
            if angle == -1:
                angle_list.append(angle)

            if len(angle_list) == 5:
                if target_linear_vel == target_linear_vel:
                    target_linear_vel = 0.0
                    # target_linear_vel = 0.2
                    target_angular_vel = 0.3 * -1.57
                    # rospy.loginfo('Right')
                    angle_list.clear()
                        
            if -0.2 < angle < 0.5:
                if target_angular_vel == target_angular_vel:
                    target_angular_vel = 0.0
                    target_linear_vel = 0.3
                    # rospy.loginfo('Straight')
                    
            # elif angle <= -1:     # back
            #     target_linear_vel = -1 * 0.3

            # elif angle >= 15 : # angle #left
                # target_angular_vel = 1 * 0.7 * 1.57

            # elif angle > 0.5 and angle != -1 :   # angle #right
            #     target_angular_vel = 1 * 0.3 * -1.57

            # elif angle == -1 :
            #     target_linear_vel  = 0.0
            #     control_linear_vel = 0.0
            #     target_angular_vel = 0.0
            #     control_angular_vel = 0.0



            # if rospy.is_shutdown():
            #     target_linear_vel  = 0.0
            #     target_angular_vel = 0.0

            

            twist.linear.x = target_linear_vel
            # twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = target_angular_vel
            # twist.angular.z = 0.0

            pub1.publish(image_msg)
            pub2.publish(twist)

            # delta = time.time() - begin
            # print("delta: ", delta)

            key = get_key()
            # print('key: ', key)
            # rospy.loginfo("Key", key)
            if key == '\x03': 
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.linear.z = 0.0

                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = 0.0

                pub2.publish(twist)
                break

            cv2.waitKey(40) #33ms == 30fps

    except KeyboardInterrupt:
        rospy.loginfo("Exiting Program")
    
    except Exception as exception_error:
        rospy.loginfo("Error occurred. Exiting Program")
        rospy.loginfo("Error: " + str(exception_error))
    
    finally:
        rospy.loginfo("Finally")
        cap.release()
        out.release()
    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__=="__main__":
    
    testDrive()
    # drive()

    pass
