#! /usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist
from math import radians
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
import serial
import numpy as np

serial_port = '/dev/ttyACM0'
baud_rate = 9600 #In arduino, Serial.begin(baud_rate)
SENSORES = int(6)
import time

def sensor(ser,distances):
	#num_caracters = 30
	#muestra = np.arange(num_caracters)
	#while num_caracters > 0:
	line = ser.readline()
	beta = 0.8
	line = line.decode("utf-8", "ignore")
	sensores = line.split(",")
	if(len(sensores)!=6):
		return distances
	for l in sensores:
		i=l.split(":")
		if(len(i)!=2):
			continue
		try:
			if(i[0]=="Sensor 1"):
				distances[0]=beta*float(i[1])+(1-beta)*distances[0]
			elif(i[0]=="Sensor 2"):
				distances[1]=beta*float(i[1])+(1-beta)*distances[1]
			elif(i[0]=="Sensor 3"):
				distances[2]=beta*float(i[1])+(1-beta)*distances[2]
			elif(i[0]=="Sensor 4"):
				distances[3]=beta*float(i[1])+(1-beta)*distances[3]
			elif(i[0]=="Sensor 5"):
				distances[4]=beta*float(i[1])+(1-beta)*distances[4]
			elif(i[0]=="Sensor 6"):
				distances[5]=beta*float(i[1])+(1-beta)*distances[5]
		except:
			print("El sensor envio un error")
			print(i)
	return distances

def moveSq():
	rospy.init_node('cuadrado', anonymous=True)
	velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
	distance_publisher = rospy.Publisher('distance',Float32MultiArray,queue_size = 1)
	distances = []
	for i in range(0,SENSORES):
		distances.append(0)
	ser = serial.Serial(serial_port, baud_rate)
	vel_msg = Twist()
	vel_msg.linear.x = 0.1
	ang_msg = Twist()
	ang_msg.angular.z = radians(45)
	distance = 0.5
	isFwd = 0
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	ang_msg.angular.x = 0
	ang_msg.angular.y = 0
	ang_msg.angular.x = 0
	msg = Float32MultiArray()
	msg.layout.dim.append(MultiArrayDimension())
	msg.layout.dim[0].label = "distnacias"
	msg.layout.dim[0].size = 6
	msg.layout.dim[0].stride = 6
	msg.layout.data_offset = 0
	print ("Preparado")
	while not rospy.is_shutdown():
		mills1 = int(round(time.time() * 1000))
		obst= sensor(ser,distances)
		#print(obst)
		#vel_msg.linear.x = 0.1
#		for i in obst:
#			if(i<10.0):
#				vel_msg.linear.x = 0.0
#				velocity_publisher.publish(vel_msg)
		msg.data = obst
		#print(msg)
		distance_publisher.publish(msg)
		
if __name__ == '__main__':
	try:
		moveSq()
	except rospy.ROSInterruptException: pass



