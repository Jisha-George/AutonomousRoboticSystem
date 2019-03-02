import numpy
import cv2
import cv_bridge
import rospy
import actionlib

from time import sleep
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal

kernel = numpy.ones((5, 5), numpy.uint8)


class Follower:
	
	def __init__(self):
		self.cvBridge = cv_bridge.CvBridge()
		self.imgSub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
		self.scanSub = rospy.Subscriber('/scan', LaserScan, self.scan)
		self.odomSub = rospy.Subscriber('/odom', Odometry, self.odom)
		self.movePub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size = 1)
		self.velPub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
		self.found = {"red": False, "yellow": False, "green": False, "blue": False}
		self.dist = 0.0
		self.imageobj = Image()
		self.pose = Odometry()
		sleep(0.1)
		
		sleep(10)
		self.move(2.0, 1, 0.99)
#		self.main()

	def scan(self, msg):
		self.dist = msg.ranges[320]
		
	def image_callback(self, msg):
		self.imageobj = msg
		
	def odom(self, msg):
		self.pose = {msg.pose.pose.orientation.w, msg.pose.pose.orientation.z}
	
	def move(self, x, y, o):
		moveplz = MoveBaseActionGoal()
		moveplz.goal.target_pose.pose.position.x = x
		moveplz.goal.target_pose.pose.position.y = y
		moveplz.goal.target_pose.pose.orientation.z = o
		moveplz.goal.target_pose.pose.orientation.w = 1
		moveplz.goal.target_pose.header.stamp = rospy.Time.now()
		moveplz.goal.target_pose.header.frame_id = 'map'
		moveplz.goal.target_pose.header.seq = 101
		
		self.movePub.publish(moveplz)
		
	def imageMaskMaker(self, msg):
		cv2.namedWindow("Image",1)
		image = self.cvBridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')		
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

		global mask, red, blu, yel, gre, M
		
		red = cv2.inRange(hsv, numpy.array([0, 100, 100]), numpy.array([6, 255, 255]))
		yel = cv2.inRange(hsv, numpy.array([20, 100, 100]), numpy.array([40, 255, 255]))
		gre = cv2.inRange(hsv, numpy.array([50, 100, 100]), numpy.array([70, 255, 255]))
		blu = cv2.inRange(hsv, numpy.array([110, 100, 100]), numpy.array([140, 255, 255]))

		mask = red * 0
		
		if not (self.found["red"]):
			mask += red
		if not (self.found["yellow"]):
			mask += yel
		if not (self.found["green"]):
			mask += gre
		if not (self.found["blue"]):
			mask += blu		

		h, w, d = image.shape

		search_top = 0		
		search_bot = h
		
		mask[0:search_top, 0:w] = 0
		mask[search_bot:h, 0:w] = 0

		mask = cv2.erode(mask, kernel)
		mask = cv2.medianBlur(mask,5)

		return (image, mask, red, yel, gre, blu, self.found)
	
	
	def main(self):
		while not rospy.is_shutdown():

			(img, mask, r, y, g, b, self.found) = self.imageMaskMaker(self.imageobj)
			M = cv2.moments(mask)
			h, w, _ = img.shape
			t = Twist()
			
			if (self.found["red"] == False or self.found["yellow"] == False or self.found["green"] == False or self.found["blue"] == False):

				t.angular.z = 0.5

				if r[239, 319] and self.dist < 1:
					self.found["red"] = True
					print "found red"				
				elif y[239, 319] and self.dist < 1:
					self.found["yellow"] = True
					print "found yellow"
				elif g[239, 319] and self.dist < 1:
					self.found["green"] = True
					print "found green"
				elif b[239, 319] and self.dist < 1:
					self.found["blue"] = True
					print "found blue"
				else:
					if numpy.sum(mask[:,320]) == 0 :
						print numpy.sum(mask[:,320])
				if numpy.all(mask == 0):
					t.angular.z = 0.5
					self.velPub.publish(t)	
				else:
					cx = int(M['m10']/M['m00'])
					cy = int(M['m01']/M['m00'])
					err = cx - w/2
					t.linear.x = 0.5
					t.angular.z = -float(err)/100
					self.velPub.publish(t)

			else:
				break

			cv2.imshow("Image", img)
			cv2.imshow("Mask", mask)
			cv2.waitKey(3)

cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()
