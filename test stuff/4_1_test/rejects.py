
'''
	
	gen 1st
	for i in range 10
		while True
			gen next
	
			
#######################################################################################################################################

		xP = numpy.round((numpy.array([x[0] for x in self.path]) - (hn/2)) * mapy.shape[0]/-hn)
		yP = numpy.round((numpy.array([y[1] for y in self.path]) - (wn/2)) * mapy.shape[1]/-wn)					
		for pos in range(len(xP)):
			if numpy.mod(pos,20) == 0:
				cv2.circle(mapy,(int(yP[pos]),int(xP[pos])),1,(255,255,0),-1)
		cv2.imshow("path", mapy)
		cv2.waitKey(10)

self.path.append([msg.pose.pose.position.x, msg.pose.pose.position.y])

		self.path = []


						
t0 = time.time()
						t1 = time.time()
				
						while t1-t0 < 15:	
							t1 = time.time()

	
	_,_,yaw=epose#once only
	while epose[2]!=yaw+-tol
	#julie do the thing
	
	t.angular.z = ((speed * 2) * Pi) /360
	
							
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
							self.co.append([self.pos[0],self.pos[1],False])			
						elif y[239, 319] and self.dist < 1:
							self.found["yellow"] = True
							print "found yellow"
							self.co.append([self.pos[0],self.pos[1],False])
						elif g[239, 319] and self.dist < 1:
							self.found["green"] = True
							print "found green"
							self.co.append([self.pos[0],self.pos[1],False])
						elif b[239, 319] and self.dist < 1:
							self.found["blue"] = True
							print "found blue"
							self.co.append([self.pos[0],self.pos[1],False])

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
