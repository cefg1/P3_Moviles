#!/usr/bin/env python

# INICIO PROGRAMA
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:

  def __init__(self):
  
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    
    # Indicadores de detección de señales
    parking_state = False
    stop_state = False
    self.parking_state(parking_state)
    self.stop_state(stop_state)
    
    self.image_sub = rospy.Subscriber('camera/image', Image, self.principal)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    self.twist = Twist()
  
  def parking_state (self, state):
  	global parking_detected
  	parking_detected = state

  def stop_state (self, state):
  	global stop_detected
  	stop_detected = state

  	
  def principal(self, msg):
  
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    h, w, d = image.shape
    
    # Detección de Lineas
    left = self.detect_line(image, numpy.array([10,10,10]), numpy.array([255,255,250]), 40, h, w, hsv)
    right = self.detect_line(image, numpy.array([0,0,250]), numpy.array([0,0,255]), 280, h, w, hsv)
    
    cv2.imshow("window", image)
    cv2.waitKey(3)
	
	# Detección de Señales
    if parking_detected == False:
    	parking = self.detect_signal (image, numpy.array([90,50,70]), numpy.array([128,255,255]), h, w, hsv)
    else:
    	parking = 0
    
    # Señal de parking cerca
    if parking > 3600 and parking_detected == False:
    	self.parking_state(True)
    	self.control (left, right, w, 0.0, 0.0)
    	print()
    	input("Bienbenido, por favor pulse 'ENTER' para comenzar el trayecto ...")
    	
    cv2.imshow("window", image)
    cv2.waitKey(3)
    	
    if stop_detected == False:
    	stop = self.detect_signal (image, numpy.array([159,50,70]), numpy.array([180,255,255]), h, w, hsv)
    else:
    	stop = 0
    
    # Señal de STOP CERCA
    if stop > 1300 and stop_detected == False:
    	self.stop_state(True)
    	self.control (left, right, w, 0.0, 0.0)
    	print()
    	print ("Detectado 'STOP', final del trayecto. Gracias por viajar con nosotros")
    	
    if stop_detected == True:
    	self.control (left, right, w, 0.0, 0.0)
    else:
    	self.control (left, right, w, 0.1, 1)
    
    cv2.imshow("window", image)
    cv2.waitKey(3)
  
  # Detección de Linea
  def detect_line (self, image, lower, upper, ideal_position, h, w, hsv):
  
  	mask = cv2.inRange(hsv, lower, upper)
  	
  	x = ideal_position
  	y = 190
  	
  	# Zona de búsqueda
  	search_top = 3*h/4
  	search_bot = 3*h/4 + 20
  	
  	mask [0:int(search_top), 0:w] = 0
  	mask[int(search_bot):h, 0:w] = 0
  	M = cv2.moments(mask)
  	if M['m00'] > 0:
  		x = int(M['m10']/M['m00'])
  		y = int(M['m01']/M['m00'])
  		cv2.circle(image, (x, y), 20, (0,0,255), -1)
  	else:
  		cv2.circle(image, (x, y), 20, (0,255,0), -1) 	
  	
  	return x
  
  # Detección de Señales
  def detect_signal (self, image, lower, upper, h, w, hsv):
  	
  	mask = cv2.inRange(hsv, lower, upper)

  	# Zona de búsqueda: margen superior derecho
  	mask[0:int(h/2), 0:int(w/2)] = 0
  	mask[int(h/2):h, 0:w] = 0
  	
  	# Detección del contorno y Cálculo del área
  	kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
  	opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
  	
  	contours = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  	contours = contours[0] if len (contours) == 2 else contours[1]
  	
  	area = 0
  	for pixel in contours:
  		area += cv2.contourArea(pixel)
  	
  	return area
  
  # Control del Robot
  def control (self, left, right, w, vel, ang):
  
  	err = ((left+right)/2) - (w/2)
  	self.twist.linear.x = vel
  	self.twist.angular.z = -float(err) / 100 * ang
  	self.cmd_vel_pub.publish(self.twist)
  

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# FINAL DE PROGRAMA
