#!/usr/bin/env python

from dis import dis
import time
from turtle import distance, pd

from cv2 import resize
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:

        def __init__(self):

                self.bridge = cv_bridge.CvBridge()

                self.image_sub = rospy.Subscriber('camera/image',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=10)

                self.twist = Twist()
        
        def markdetect(self , camera):

             this_aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_100)
             this_aruco_parameters = cv2.aruco.DetectorParameters_create()
             frame = camera
             #import pdb; pdb.set_trace()
    # Detect ArUco markers in the video frame
             (corners, ids, rejected) = cv2.aruco.detectMarkers(
               frame, this_aruco_dictionary, parameters=this_aruco_parameters)
             cameraMatrix = numpy.matrix(numpy.array([[-3.313061098815712, 0.0, 160.5],
                                              [0.0, -3.313061098815712, 120.5],
                                              [0.0, 0.0, 1.0]]))
             distCoeffs = numpy.array([0.0, 0.0, 0.0, 0.0, 0.0])
             R = numpy.matrix(numpy.array([[1, 0, 1],
                                              [0, 0, 0],
                                              [0, -1, 0]]))
             T = numpy.matrix(numpy.array([0, 1, 1]).reshape(3, 1))
             n = numpy.matrix(numpy.array([0, -1, 0]).reshape(3, 1))
             d = 0.115
             
    # Check that at least one ArUco marker was detected
             if len(corners) > 0:
      # Flatten the ArUco IDs list
                    print("detected!")
                    ids = ids.flatten()
                    for (marker_corner, marker_id) in zip(corners, ids):
       
                        # Extract the marker corners
                        r, t, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corner, 0.05, cameraMatrix, distCoeffs)
                        dis_z = numpy.abs((t[0][0][2])*100)
                        if dis_z >=0.999 and dis_z<=1.05:    
                                print("detected!")
                                print("stop!")                  
                                self.twist.linear.x = 0
                                self.twist.angular.z = 0
                                self.cmd_vel_pub.publish(self.twist)
                                time.sleep(10)
                                
                                
        def image_callback(self, msg):

                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                #image = cv2.resize(image , (640,480) , interpolation = cv2.INTER_LANCZOS4)
                #print(image.shape)
               # cv2.imwrite("src.png" , image)
               # print("srcsaved")
                self.markdetect(image)
            
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

                lower_yellow = numpy.array([ 10, 10, 10])
                upper_yellow = numpy.array([255, 255, 250])

                lower_white = numpy.array([0, 0, 240])
                upper_white = numpy.array([150, 43, 255])
                
                mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask2 = cv2.inRange(hsv, lower_white, upper_white)
                #import pdb; pdb.set_trace()
                h, w, d = image.shape
                search_top = 2*h/3
                mask1[0:search_top, 0:w] = 0
                mask2[0:search_top, 0:w] = 0

                M1 = cv2.moments(mask1)
                M2 = cv2.moments(mask2)

                if M1['m00'] > 0:
                    cx1 = int(M1['m10']/M1['m00'])
                    cy1 = int(M1['m01']/M1['m00'])

                    cx2 = int(M2['m10']/M2['m00'])
                    cy2 = int(M2['m01']/M2['m00'])

                    fpt_x = (cx1 + cx2)/2
                    fpt_y = (cy1 + cy2)/2 + 2*h/3

                    cv2.circle(image, (cx1, cy1), 10, (0,255,255), -1)
                    cv2.circle(image, (cx2, cy2), 10, (255,255,255), -1)
                    cv2.circle(image, (fpt_x, fpt_y), 10, (128,128,128), -1)

                    err = w/2 - fpt_x

                    self.twist.linear.x = 0.3
                    self.twist.angular.z = (err*90.0/160)/15
                    self.cmd_vel_pub.publish(self.twist)
                cv2.imshow("window", image)
                cv2.waitKey(1)
                #import pdb; pdb.set_trace()

rospy.init_node('lane_follower')
follower = Follower()
rospy.spin()
#import pdb;pdb.set_trace()