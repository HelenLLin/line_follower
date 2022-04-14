#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge() # creates a cv_bridge object to communicate between ROS and OpenCV image formats
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw/compressed', CompressedImage, self.image_cb) # subscribes to image
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1) # publishes to cmd_vel
        self.twist = Twist()
        self.logcount = 0
        self.lostcount = 0

    def image_cb(self, msg):

        # get image from camera
        np_arr = numpy.fromstring(msg.data, numpy.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # detects only the yellow line, gets rid of all other "noise"
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([ 40, 0, 0]) # lower bound of yellow line
        upper_yellow = numpy.array([ 120, 255, 255]) # upper bound of yellow line
        mask = cv2.inRange(hsv,  lower_yellow, upper_yellow) # mask of yellow line
        masked = cv2.bitwise_and(image, image, mask=mask)

        # clear a 20 pixel area by the top of the image
        h, w, d = image.shape
        search_top = 3 * h /4
        search_bot = search_top + 20
        mask[0:int(search_top), 0:w] = 0
        mask[int(search_bot):h, 0:w] = 0
        cv2.imshow("band", mask) # mask of 20 pixel area

        # find the center and mark it with a red dot
        M = cv2.moments(mask)
        self.logcount += 1
        print("M00 %d %d" % (M['m00'], self.logcount))

        if M['m00'] > 0: 
            cx = int(M['m10']/M['m00']) + 100
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

            # linear x = 0.2 m/s
            # Add a turn using err if the center dot is off
            err = cx - w/2
            self.twist.linear.x = 0.2
            # divides err by 1000 to create a reasonable turning distance
            self.twist.angular.z = -float(err) / 1000
            self.cmd_vel_pub.publish(self.twist)
        # shows the image of the mask and the red dot in a cv2 window
        cv2.imshow("image", image)
        cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
