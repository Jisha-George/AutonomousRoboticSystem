import numpy
import cv2
import cv_bridge
import rospy
import math
import time

from actionlib_msgs.msg import GoalStatusArray
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from random import randint
from time import sleep

kernel = numpy.ones((5, 5), numpy.uint8)

#######################################################################################################################################

class Follower:
	
	def __init__(self):
		self.cvBridge = cv_bridge.CvBridge()
		self.imgSub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
		self.scanSub = rospy.Subscriber('/scan', LaserScan, self.scanning)
		self.odomSub = rospy.Subscriber('/odom', Odometry, self.odom)
		self.mapSub = rospy.Subscriber('/map', OccupancyGrid, self.mapp)
#		self.statSub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status)
		
		self.movePub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size = 1)
		self.velPub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size =1)
		self.maskPub = rospy.Publisher('/13488071/images/mask', Image, queue_size = 1)
		
		self.found = {"red": False, "yellow": False, "green": False, "blue": False}
		self.dist = 0.0
		self.imageobj = Image()
		self.epose = []
		self.odomObj = Odometry()
		#self.ab = True
		self.path = []
		global mask, red, blu, yel, gre, M
		sleep(0.1)

		self.main()
		
#######################################################################################################################################
	#def status(self, msg):
		#try:
			#self.stat = msg.status_list[len(msg.status_list)-1].status
		#	self.ab = True
#		except:
			#if self.ab == True:
	#			print "Error..."
			#	self.ab = False
			
#######################################################################################################################################

	def scanning(self, msg):
		self.dist = msg.ranges[320]
		
#######################################################################################################################################
		
	def image_callback(self, msg):
		self.imageobj = msg
		image = self.cvBridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')		
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		
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

		mask = cv2.erode(mask, kernel)
		mask = cv2.medianBlur(mask,5)
		
		self.maskPub.publish(self.cvBridge.cv2_to_imgmsg(mask, encoding = 'mono8'))
		
		return (image, mask, red, yel, gre, blu, self.found)

#######################################################################################################################################
		
	def odom(self, msg):
		self.orient = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.w, msg.pose.pose.orientation.z]
		self.ePose = euler_from_quaternion(self.orient)
		self.qPose = quaternion_from_euler(self.ePose[0],self.ePose[1], self.ePose[2])
			
		self.path.append([msg.pose.pose.position.x, msg.pose.pose.position.y])

#######################################################################################################################################

#create random co-ordinates		
	def node_gen(self, quantity):
		print("generating nodes...")
		return [[randint(-4,3) + 0.5, randint(-5,4) + 0.5, False] for i in range(quantity)]
		


#######################################################################################################################################

	def move(self, x, y):
		co_move = MoveBaseActionGoal()
		co_move.goal.target_pose.pose.position.x = x
		co_move.goal.target_pose.pose.position.y = y
		co_move.goal.target_pose.pose.orientation.w = 1
		co_move.goal.target_pose.header.stamp = rospy.Time.now()
		co_move.goal.target_pose.header.frame_id = 'map'
		co_move.goal.target_pose.header.seq = 102
		self.movePub.publish(co_move)
		
#######################################################################################################################################

	def colour_move(self, x, z):
		colour_found = MoveBaseActionGoal()
		colour_found.goal.target_pose.pose.position.x = x
		colour_found.goal.target_pose.pose.orientation.z = z
		colour_found.goal.target_pose.pose.orientation.w = 1
		colour_found.goal.target_pose.header.stamp = rospy.Time.now()
		colour_found.goal.target_pose.header.frame_id = 'base_link'
		colour_found.goal.target_pose.header.seq = 101		
		self.movePub.publish(colour_found)
			
#######################################################################################################################################
		
	def mapp(self,msg):
		cv2.imwrite('map.png', cv2.flip(cv2.rotate(numpy.reshape(msg.data, newshape = (msg.info.height, msg.info.width)),cv2.ROTATE_90_COUNTERCLOCKWISE),1))	
		
#######################################################################################################################################

	def main(self):
		self.path = []
		start = time.time()
		co = self.node_gen(20)
		done = 3
		fail = 4
		sleep(3)
		self.move(1,0)
		sleep(5)
		self.move(0,0)
		sleep(5)
		t = Twist()			
		hn = 10
		wn = 12
	
		#while there are unvisited co-ordinates or there are unfound poles
		while not([self.found[k] == True for k in self.found] == [True]*4):
			
			mapy = cv2.imread('map.png')
			
			#create array containing unvisited co-ordinates
			temp = [dist for dist in co if dist[2] == False]

			#add pixel co-ordinates for map
			xP = numpy.round((numpy.array([x[0] for x in temp]) - (hn/2)) * mapy.shape[0]/-hn)
			yP = numpy.round((numpy.array([y[1] for y in temp]) - (wn/2)) * mapy.shape[1]/-wn)
			for i in range(len(xP)):
				cv2.circle(mapy,(int(yP[i]),int(xP[i])),4,(255,255,0),-1)
				cv2.imshow("map", mapy)
				cv2.waitKey(10)

			#move to nearest co:
			xval = numpy.array([x[0] for x in temp])
			yval = numpy.array([y[1] for y in temp])
			xstart = self.orient[0]
			ystart = self.orient[1]
			#calculate the euclidian distance from current goal to each node
			edist = numpy.sqrt((xval-xstart)**2+(yval-ystart)**2)
			next_id = numpy.argmin(edist)
			self.move(temp[next_id][0], temp[next_id][1])
			print "setting co-ordinates" + str(co)
			print"checking: " + str(temp[next_id][0]) + ", " + str(temp[next_id][1]) + " ..." + str(temp[next_id][2])
			sleep(15)
			
			
			#spin 360 at current co (x)
			t0 = time.time()
			t1 = time.time()
			print self.found
			
			while t1-t0 < 20:
				(img, mask, r, yel, g, b, self.found) = self.image_callback(self.imageobj)
				
				M = cv2.moments(mask)
				h, w, _ = img.shape
				t1 = time.time()
				
				#if there are lost nodes
				if (not([self.found[k] == True for k in self.found] == [True]*4)):			
					
					#if the mask is empty then spin
					if numpy.all(mask[240, :] == 0):
						t.angular.z = -1
						self.velPub.publish(t)
					

					#if an object is focused
					elif mask[239,319]:
						print("Object Centred " + str(self.dist))
						
						#if dist < 0.5
						if numpy.isnan(self.dist):
							self.colour_move(-0.5, self.orient[2])
							sleep(0.5)
						
						
						#if the object is  0.5<dist<1m  away
						elif self.dist < 1.1:
				
							print("!!!something detected!!!")
					
							#find the colour of the object and set it to found
							if r[239, 319] and not self.found["red"]:
								print "Found Red"	
								self.found["red"] = True	
								break	
							elif yel[239, 319] and not self.found["yellow"]:
								print "Found Yellow"	
								self.found["yellow"] = True	
								break		
							elif g[239, 319] and not self.found["green"]:
								print "Found Green"	
								self.found["green"] = True	
								break	
							elif b[239, 319] and not self.found["blue"]:
								print "Found Blue"	
								self.found["blue"] = True	
								break

						#if object is >1m away
						else:
							print "move forward"
							self.colour_move(0.5,0)
							sleep(0.5)

					#otherwise spin till the object is in the middle of the screen & move forward
					else:
					 	print("Twist to Focus")
						
						maskL = numpy.sum(mask[239, 0:213])
						maskC = numpy.sum(mask[239, 213:426])
						maskR = numpy.sum(mask[239, 426:640])
						
						if numpy.argmax([maskL, maskC, maskR]) == 0:
							print("	- Twist left")
							self.colour_move(0,0.3)
							sleep(0.5)
						elif numpy.argmax([maskL, maskC, maskR]) == 1:
							print("	- Move forward")
							self.colour_move(0.65,0)
							sleep(0.5)
						elif numpy.argmax([maskL, maskC, maskR]) == 2:
							print("	- Twist right")
							self.colour_move(0,-0.3)
							sleep(0.5)
						
						print '================'
						print str(self.dist)+"|"+str(r[239, 319])+"|"+str(yel[239, 319])+"|"+str(g[239, 319])+"|"+str(b[239, 319])
				


				#if spin finished, current node complete
				
			else:
				print("spin finished")
				temp[next_id][2] = True
				print temp
				
				if (not([self.found[k] == True for k in self.found] == [True]*4)) and numpy.all(numpy.array(co)[:,2] == True):
					co = self.node_gen(10)	
					
		else:
					end = (time.time() - start)/60 
					print end


		xP = numpy.round((numpy.array([x[0] for x in self.path]) - (hn/2)) * mapy.shape[0]/-hn)
		yP = numpy.round((numpy.array([y[1] for y in self.path]) - (wn/2)) * mapy.shape[1]/-wn)					
		for pos in range(len(xP)):
			if numpy.mod(pos,20) == 0:
				cv2.circle(mapy,(int(yP[pos]),int(xP[pos])),1,(255,255,0),-1)
		cv2.imshow("path", mapy)
		cv2.waitKey(10)
				
						
						#move towards colour
						#mark as found
						#break
					#if x = 360
						#set current node as complete
					#if no nodes are reachable
						#generate new forest
						
				#if temp		
					#if colours not found and all nodes reached
						#generate new forest
						
		

cv2.startWindowThread()
#cv2.namedWindow("Image",1)
#cv2.namedWindow("Mask",2)
rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()

#######################################################################################################################################
'''
	
	gen 1st
	for i in range 10
		while True
			gen next
	
	
			
#######################################################################################################################################

def new_node(self, quant, minD):
	c = node_gen(1)
	for i in range(quant):
		while True:
			next = node_gen(1)
			if min(euclidian_distance(c,next)) > minD:
				concat(c,next)
			else:
				break

	
	
	
	
	
	def colourfinder(self):
		(img, mask, r, y, g, b, self.found) = self.imageMaskMaker(self.imageobj)			
		M = cv2.moments(mask)
		h, w, _ = img.shape
		t = Twist()
		
		if (self.found["red"] == False or self.found["yellow"] == False or self.found["green"] == False or self.found["blue"] == False):

			t.angular.z = 0.5
			if r[239, 319] and self.dist < 1:
				self.found["red"] = True
				print "found red"
				t.linear.x = 0.5
				t.angular.z = -float(err)/100					
				self.velPub.publish(t)			
			elif y[239, 319] and self.dist < 1:
				self.found["yellow"] = True
				print "found yellow"
			elif g[239, 319] and self.dist < 1:
				self.found["green"] = True
				print "found green"
			elif b[239, 319] and self.dist < 1:
				self.found["blue"] = True
				print "found blue"

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
					
		cv2.imshow("Image", img)
		cv2.imshow("Mask", mask)
		cv2.waitKey(3)		

#######################################################################################################################################
		
	def spin(self, spinTime):
		t0 = time.time()
		sleep(1)
		t1 = time.time()
		t = Twist()
		
		while t1-t0 < spinTime:
		
			self.colourfinder()		
			t1 = time.time()

(img, mask, r, y, g, b, self.found) = self.imageMaskMaker(self.imageobj)
M = cv2.moments(mask)
h, w, _ = img.shape
xP = (numpy.array([x[0] for x in co]) - (hn/2)) * mapy.shape[0]/-hn
yP = (numpy.array([y[1] for y in co]) - (wn/2)) * mapy.shape[1]/-wn
#		return [[randint(-4,4), randint(-5,5), False] for i in range(quantity)]
#		self.statSub = rospy.Subscriber('/move_base/status', )
				t0 = time.time()
				sleep(1)
				t1 = time.time()
				t = Twist()
		
				while t1-t0 < 17:
					
					#if a colour is found while spinning 
					if (self.found["red"] == False or self.found["yellow"] == False or self.found["green"] == False or self.found["blue"] == False):			
						t.angular.z = 0.5

						if r[239, 319] and self.dist < 1:
							self.found["red"] = True
							print "found red"	
							self.co.append([self.orient[0],self.orient[1],False])			
						elif y[239, 319] and self.dist < 1:
							self.found["yellow"] = True
							print "found yellow"
							self.co.append([self.orient[0],self.orient[1],False])
						elif g[239, 319] and self.dist < 1:
							self.found["green"] = True
							print "found green"
							self.co.append([self.orient[0],self.orient[1],False])
						elif b[239, 319] and self.dist < 1:
							self.found["blue"] = True
							print "found blue"
							self.co.append([self.orient[0],self.orient[1],False])

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
						c[next_id][2] = True
						break
				
					cv2.imshow("Image", img)
					cv2.imshow("Mask", mask)
					cv2.waitKey(3)
					
					t1 = time.time()
					
					
					
					
						#	self.velPub.publish(t2)
							
							
							#print "Moments"
							#cx = int(M['m10']/M['m00'])
							#cy = int(M['m01']/M['m00'])
							#err = cx - w/2
							#t.angular.z = -float(err)/200
							#t.linear.x = 0.75
							#self.velPub.publish(t)
							#self.colour_move(0.5,0)
							
							#sleep(1)
					
					
					
					
					
'''
