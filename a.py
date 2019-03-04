import numpy
import cv2
import cv_bridge
import rospy
import math

from time import sleep
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from random import randint

kernel = numpy.ones((5, 5), numpy.uint8)

class Follower:
	
	def __init__(self):
		self.cvBridge = cv_bridge.CvBridge()
		self.imgSub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
		self.scanSub = rospy.Subscriber('/scan', LaserScan, self.scan)
		self.odomSub = rospy.Subscriber('/odom', Odometry, self.odom)
		self.mapSub = rospy.Subscriber('/map', OccupancyGrid, self.mapp)
#		self.statSub = rospy.Subscriber('/move_base/status', )
		self.movePub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size = 1)
		self.velPub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
		self.found = {"red": False, "yellow": False, "green": False, "blue": False}
		self.dist = 0.0
		self.imageobj = Image()
		self.epose = []
		self.odomObj = Odometry()
		sleep(0.1)
		self.main()

	def scan(self, msg):
		self.dist = msg.ranges[320]
		
	def image_callback(self, msg):
		self.imageobj = msg
		
	def odom(self, msg):
		self.orient = [0,0, msg.pose.pose.orientation.w, msg.pose.pose.orientation.z]
		self.ePose = euler_from_quaternion(self.orient)
		self.qPose = quaternion_from_euler(self.ePose[0],self.ePose[1], self.ePose[2])
		
		return (self.ePose)	

#create random co-ordinates		
	def node_gen(self, quantity):
#		return [[randint(-4,4), randint(-5,5), False] for i in range(quantity)]
		return [[randint(-4,3) + 0.5, randint(-5,5) + 0.5, False] for i in range(quantity)]
			
	
	def move(self, x, y):
	
		moveplz = MoveBaseActionGoal()
		moveplz.goal.target_pose.pose.position.x = x
		moveplz.goal.target_pose.pose.position.y = y
		moveplz.goal.target_pose.pose.orientation.w = 1
		moveplz.goal.target_pose.header.stamp = rospy.Time.now()
		moveplz.goal.target_pose.header.frame_id = 'map'
		moveplz.goal.target_pose.header.seq = 101
		print "here"				
		self.movePub.publish(moveplz)					
		
	def spin(self, x):
		r,p,y = self.ePose
		t = Twist()	
		t.angular.z = y
		self.velPub.publish(t)
		r1,p1,y1 = self.ePose
		print y
		print y1
		while y != y1:
			self.velPub.publish(t)
			
	def mapp(self,msg):
		cv2.imwrite('map.png', cv2.flip(cv2.rotate(numpy.reshape(msg.data, newshape = (msg.info.height, msg.info.width)),cv2.ROTATE_90_COUNTERCLOCKWISE),1))
		
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
			(ePose) = self.odom(self.odomObj)
			M = cv2.moments(mask)
			h, w, _ = img.shape
			t = Twist()

			co = self.node_gen(15)
			hn = 10
			wn = 12
			mapy = cv2.imread('map.png')
#			xP = (numpy.array([x[0] for x in co]) - (hn/2)) * mapy.shape[0]/-hn
	#		yP = (numpy.array([y[1] for y in co]) - (wn/2)) * mapy.shape[1]/-wn
			xP = numpy.round((numpy.array([x[0] for x in co]) - (hn/2)) * mapy.shape[0]/-hn)
			yP = numpy.round((numpy.array([y[1] for y in co]) - (wn/2)) * mapy.shape[1]/-wn)
			
			#add co-ordinates to map
			for i in range(len(xP)):
				cv2.circle(mapy,(int(yP[i]),int(xP[i])),4,(255,255,0),-1)

			cv2.imshow("map", mapy)
			cv2.waitKey(1)
			
			while numpy.any(numpy.array(co) == False):
				cv2.waitKey(60)
			#move to nearest co
				#calculate the distance nearest co using the euclidian distance from current goal
				
				temp = [dist for dist in co if dist[2] == False]
				xval = numpy.array([x[0] for x in temp])
				yval = numpy.array([y[1] for y in temp])
				xstart = 0
				ystart = 0
				edist = numpy.sqrt((xval-xstart)**2+(yval-ystart)**2)
				next_id = numpy.argmin(edist)
				print "temp stuff"
				self.move(temp[next_id][0], temp[next_id][1])
				print "setting co-ordinates" + str(temp)
				print"checking: " + str(temp[next_id][0]) + ", " + str(temp[next_id][1]) + " ..."
				
				self.spin()
				
			
					#spin 360 at current co (x)
				
				
						#if a colour is found while spinning 
							#add new node
							#move towards colour
							#mark as found
						#	break

					#	if x = 360
							#set current node as complete

						#if no nodes are reachable
							#generate forest again		
				sleep(20)
			
		#	if (self.found["red"] == False or self.found["yellow"] == False or self.found["green"] == False or self.found["blue"] == False):

#				t.angular.z = 0.5

#				if r[239, 319] and self.dist < 1:
#					self.found["red"] = True
#					print "found red"				
#				elif y[239, 319] and self.dist < 1:
#					self.found["yellow"] = True
#					print "found yellow"
#				elif g[239, 319] and self.dist < 1:
#					self.found["green"] = True
#					print "found green"
#				elif b[239, 319] and self.dist < 1:
#					self.found["blue"] = True
#					print "found blue"
	#			else:
#					if numpy.sum(mask[:,320]) == 0 :
#						print numpy.sum(mask[:,320])
#				if numpy.all(mask == 0):
#					t.angular.z = 0.5
#					self.velPub.publish(t)	
#				else:
#					cx = int(M['m10']/M['m00'])
#					cy = int(M['m01']/M['m00'])
	#				err = cx - w/2
#					t.linear.x = 0.5
#					t.angular.z = -float(err)/100
#					self.velPub.publish(t)
#
			#else:
#				break

			cv2.imshow("Image", img)
			cv2.imshow("Mask", mask)
			cv2.waitKey(3)

cv2.startWindowThread()
cv2.destroyAllWindows()
rospy.init_node('follower')
follower = Follower()
rospy.spin()
cv2.destroyAllWindows()
