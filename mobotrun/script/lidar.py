#!/usr/bin/env python
import rospy
import os
import math
import time
import json
from math import sin,cos
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import paho.mqtt.client as mqtt

Q1 = 'OK'
Q2 = 'OK'
Q3 = 'OK'
Q4 = 'OK'
Q5 = 'OK'
Q6 = 'OK'
Q7 = 'OK'
Q8 = 'OK'
Time = 0
dt = 0.1
send_to_unity = {}
flag_pub1 = 1
flag_pub2 = 1
flag_pub3 = 1
flag_pub4 = 1
flag_pub5 = 1
flag_pub6 = 1
flag_pub7 = 1
flag_pub8 = 1
flag_time = 1

class DemoNode():
	def __init__(self):
		self.timer = rospy.Timer(rospy.Duration(dt), self.demo_callback)
	def demo_callback(self, timer):
		global Time
		global flag_time
		global flag_pub1
		global flag_pub2
		global flag_pub3
		global flag_pub4
		global flag_pub5
		global flag_pub6
		global flag_pub7
		global flag_pub8
		Time += dt
		if Q1 == "BEWARE" or Q2 == "BEWARE" or Q3 == "BEWARE" or Q4 == "BEWARE" or Q5 == "BEWARE" or Q6 == "BEWARE" or Q7 == "BEWARE" or Q8 == "BEWARE":
			client.publish("Mobot/lidar","SLOW")

		if Q2 == "OK" and Q3 == "OK" and Q4 == "OK" and Q5 == "OK" and Q1 == "OK" and Q6 == "OK" and Q7 == "OK" and Q8 == "OK":
			client.publish("Mobot/lidar","OK")

		send_to_unity = {'lidar_q1':Q1, 'lidar_q2':Q2, 'lidar_q3':Q3, 'lidar_q4':Q4, 'lidar_q5':Q5,'lidar_q6':Q6, 'lidar_q7':Q7, 'lidar_q8':Q8}
		client.publish("mobot/unity/lidar",json.dumps(send_to_unity,sort_keys=True))

		if Q2 != "STOP":
			flag_pub2 = 1
		if Q3 != "STOP":
			flag_pub3 = 1
		if Q4 != "STOP":
			flag_pub4 = 1
		if Q5 != "STOP":
			flag_pub5 = 1
		if Q6 != "STOP":
			flag_pub6 = 1
		if Q7 != "STOP":
			flag_pub7 = 1
		if Q8 != "STOP":
			flag_pub8 = 1
		if Q1 != "STOP":
			flag_pub1 = 1



def callback_ridar1(data):
	global Q2
	global Q3
	global Q4
	global Q5
	global flag_pub
	Q2 = "OK"
	Q3 = "OK"
	Q4 = "OK"
	Q5 = "OK"
	temp = ""
	send_msg = {"mobi_vx":0,"mobi_vy":0,"mobi_w":0}
	count = int(math.floor(data.scan_time/data.time_increment))
	q2_stop = 0
	q2_beware =0
	q3_stop = 0
	q3_beware =0
	q4_stop = 0
	q4_beware = 0
	q5_stop = 0
	q5_beware =0
	for i in range (0,count):
		if data.ranges[i] == None:
			continue
		degree = (data.angle_min + data.angle_increment * i)*57.2958
		if(degree >= 90 and degree <= 180):
			degree = degree - 90
		elif(degree >= -180 and degree < 0):
			degree = (180-abs(degree)) + 90
		else:
			degree =  270 + degree
			
		lidar_y = (data.ranges[i])*sin(degree/57.2958)
		lidar_x = (data.ranges[i])*cos(degree/57.2958)

		if degree > 0.5 and degree <= 270:

			if lidar_x >= 0.64 and lidar_x < 1.1 and lidar_y > 0 and lidar_y <= 0.5:
				q2_beware += 1
				if lidar_x <= 0.89 and lidar_y <= 0.25:
					q2_stop += 1


			elif lidar_x >= 0 and lidar_x < 0.64 and lidar_y > 0 and lidar_y <= 0.5:
				q3_beware += 1
				if lidar_y <= 0.25:
					q3_stop += 1


			elif lidar_x >= -0.5 and lidar_x < 0 and lidar_y >= 0 and lidar_y < 0.45:
				q4_beware += 1
				if lidar_x >= -0.25 and lidar_y <= 0.25:
					q4_stop += 1

			elif lidar_x >= -0.5 and lidar_x < 0 and lidar_y > -1.2 and lidar_y < 0:
				q5_beware += 1
				if lidar_y <= -0.95 and lidar_x >= -0.25 and lidar_x <= -0.05:
					q5_stop += 1
					
				if lidar_x >= -0.25 and lidar_x <= -0.05:
					q5_stop += 1

	if(q2_beware > 10):
		Q2 = "BEWARE"
	if(q2_stop > 10):
		Q2 = "STOP"
	if(q3_beware > 10):
		Q3 = "BEWARE" 
	if(q3_stop > 10):
		Q3 = "STOP" 
	if(q5_beware > 10):
		Q5 = "BEWARE" 
	if(q5_stop > 10):
		Q5 = "STOP" 
	if(q4_beware > 10):
		Q4 = "BEWARE" 
	if(q4_stop > 10):
		Q4 = "STOP"
		
def callback_ridar2(data):
	global Q1
	global Q6
	global Q7
	global Q8
	global flag_pub
	Q1 = "OK"
	Q6 = "OK"
	Q7 = "OK"
	Q8 = "OK"
	send_msg = {"mobi_vx":0,"mobi_vy":0,"mobi_w":0}
	count = int(math.floor(data.scan_time/data.time_increment))
	q6_stop = 0
	q6_beware =0
	q7_stop = 0
	q7_beware = 0
	q8_stop = 0
	q8_beware = 0
	q1_stop = 0
	q1_beware = 0
	for i in range (0,count):
		degree = (data.angle_min + data.angle_increment * i)*57.2958

		if(degree >= 90 and degree <= 180):
			degree = degree - 90
		elif(degree >= -180 and degree < 0):
			degree = (180-abs(degree)) + 90
		else:
			degree =  270 + degree
		
		lidar_x = (data.ranges[i])*cos(degree/57.2958)
		lidar_y = (data.ranges[i])*sin(degree/57.2958)

		if degree >= 0.5 and degree <= 270:
				q6_beware += 1
				if lidar_x <= 1.0 and lidar_y <= 0.25:
					q6_stop += 1


			elif lidar_x >= 0 and lidar_x < 0.64 and lidar_y > 0 and lidar_y <= 0.5:
				q7_beware += 1
				if lidar_y <= 0.25:
					q7_stop += 1

			elif lidar_x >= -0.5 and lidar_x < 0 and lidar_y >= 0 and lidar_y < 0.45:
				q8_beware += 1
				if lidar_x >= -0.25 and lidar_y <= 0.25:
					q8_stop += 1

			elif lidar_x >= -0.5 and lidar_x < 0 and lidar_y > -1.2 and lidar_y < 0:
				q1_beware += 1
				if lidar_y <= -0.95 and lidar_x >= -0.25 and lidar_x <= -0.1:
					q1_stop += 1
				if lidar_x >= -0.25 and lidar_x <= -0.1:
					q1_stop += 1

	if(q6_beware > 10):
		Q6 = "BEWARE" 
	if(q6_stop > 10):
		Q6 = "STOP" 
	if(q7_beware > 10):
		Q7 = "BEWARE" 
	if(q7_stop > 10):
		Q7 = "STOP" 
	if(q8_beware > 10):
		Q8 = "BEWARE" 
	if(q8_stop > 10):
		Q8 = "STOP" 
	if(q1_beware > 7):
		Q1 = "BEWARE" 
	if(q1_stop > 7):
		Q1 = "STOP" 

def talker():
	global send_str
	global flag_pub1
	global flag_pub2
	global flag_pub3
	global flag_pub4
	global flag_pub5
	global flag_pub6
	global flag_pub7
	global flag_pub8
	pub = rospy.Publisher('chatter', String, queue_size=10)
	rate = rospy.Rate(50) # 10hz
	while not rospy.is_shutdown():
		send_msg = {"mobi_vx":0,"mobi_vy":0,"mobi_w":0}
		send_str = Q1+' '+Q2+' '+Q3+' '+Q4+' '+Q5+' '+Q6+' '+Q7+' '+Q8
		if Q1 == "STOP":		
			if flag_pub1 == 1:
				client.publish("unity/mobot/mobility",json.dumps(send_msg,sort_keys=True))
				client.publish("Mobot/lidar","SLOW")
				flag_pub1 = 0
			pub.publish(send_str)

		if Q2 == "STOP":
			if flag_pub2 == 1:
				client.publish("unity/mobot/mobility",json.dumps(send_msg,sort_keys=True))
				client.publish("Mobot/lidar","SLOW")
				flag_pub2 = 0
			pub.publish(send_str)

		if Q3 == "STOP":
			if flag_pub3 == 1:
				client.publish("unity/mobot/mobility",json.dumps(send_msg,sort_keys=True))
				client.publish("Mobot/lidar","SLOW")
				flag_pub3 = 0
			pub.publish(send_str)

		if Q4 == "STOP":
			if flag_pub4 == 1:
				client.publish("unity/mobot/mobility",json.dumps(send_msg,sort_keys=True))
				client.publish("Mobot/lidar","SLOW")
				flag_pub4 = 0
			pub.publish(send_str)

		if Q5 == "STOP":
			if flag_pub5 == 1:
				client.publish("unity/mobot/mobility",json.dumps(send_msg,sort_keys=True))
				client.publish("Mobot/lidar","SLOW")
				flag_pub5 = 0
			pub.publish(send_str)

		if Q6 == "STOP":
			if flag_pub6 == 1:
				client.publish("unity/mobot/mobility",json.dumps(send_msg,sort_keys=True))
				client.publish("Mobot/lidar","SLOW")
				flag_pub6 = 0
			pub.publish(send_str)

		if Q7 == "STOP":
			if flag_pub7 == 1:
				client.publish("unity/mobot/mobility",json.dumps(send_msg,sort_keys=True))
				client.publish("Mobot/lidar","SLOW")
				flag_pub7 = 0
			pub.publish(send_str)

		if Q8 == "STOP":
			if flag_pub8 == 1:
				client.publish("unity/mobot/mobility",json.dumps(send_msg,sort_keys=True))
				client.publish("Mobot/lidar","SLOW")
				flag_pub8 = 0
			pub.publish(send_str)


		rate.sleep()
		
def listener_ridar():
    rospy.Subscriber('/lidar0/scan', LaserScan, callback_ridar1)
    rospy.Subscriber('/lidar1/scan', LaserScan, callback_ridar2)
    DemoNode()
    talker()
    rospy.spin()

def myhook():
    print("Shutdown and Reintialize")



if __name__ == '__main__':

	rospy.init_node('listener_test')
	client = mqtt.Client()
	client.connect("192.168.111.2",1883)
	try:
		listener_ridar()
	except rospy.ROSInterruptException:
		print("exception thrown")
		pass
