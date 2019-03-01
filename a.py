import numpy

import cv2
import cv_bridge
import rospy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from laser_geometry import LaserProjection

kernel = numpy.ones((5, 5), numpy.uint8)

global mask, red, blu, yel, gren, M, found
search_top = 0

class Follower:
	def __init__(self):
		self.cvBridge = cv_bridge.CvBridge()
		self.imgSub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
		self.velPub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
		self.img = Image()

	def imageMaskMaker(self, msg):
		cv2.namedWindow("Image",1)
		image = self.cvBridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')		
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		
		red = cv2.inRange(hsv, numpy.array([0, 100, 100]), numpy.array([6, 255, 255]))
		yel = cv2.inRange(hsv, numpy.array([20, 100, 100]), numpy.array([40, 255, 255]))
		gre = cv2.inRange(hsv, numpy.array([50, 100, 100]), numpy.array([70, 255, 255]))
		blu = cv2.inRange(hsv, numpy.array([110, 100, 100]), numpy.array([140, 255, 255]))

		mask = red + yel + blu + gre

		found = 0

		h, w, d = image.shape

		search_bot = h
		
		mask[0:search_top, 0:w] = 0
		mask[search_bot:h, 0:w] = 0

		mask = cv2.erode(mask, kernel)
		mask = cv2.medianBlur(mask,7)
		
		return (image, mask, red, yel, gre, blu, found)


	def image_callback(self, msg):
		#mask = follower.image.mask
		(img, mask, r, y, g, b, found) = self.imageMaskMaker(msg)
		M = cv2.moments(mask)
		h, w, _ = img.shape
		t = Twist()
		if found <= 5:
			if numpy.all(mask == 0):
				t.angular.z = 0.5
				self.velPub.publish(t)	
			else:
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				cv2.circle(img, (cx,cy), 10, (0,255,0), -1)
				err = cx - w/2
				if numpy.any(mask == y):	
					if mask[h-1, w/2] != 0:
						cv2.circle(img, (cx,cy), 10, (0,0,255), -1)
						t.linear.x = 0
						found = 1
						if found == 1:
							mask = mask-y					
					else:
						t.linear.x = 0.5				
						if numpy.all(mask == 0):
							t.angular.z = 0.5
							self.velPub.publish(t)

				t.angular.z = -float(err)/100
				self.velPub.publish(t)

			print("here...")

		print("there...")

		cv2.imshow("Image", img)
		cv2.imshow("Mask", mask)
		cv2.waitKey(3)

		return mask

cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()
