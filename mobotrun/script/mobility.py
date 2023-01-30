#!/usr/bin/env python
# -*- coding: utf-8 -*-
import json
import os
import numpy
import math
import time
import rospy
from std_msgs.msg import String
import paho.mqtt.client as mqtt

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *  # Uses Dynamixel SDK library

####################################################################################################################################################

# Control table ADDRess for MX-106
# EEPROM REGISTER ADDRESSES - Permanently stored in memory once changed
ADDR_MX_MODEL_NUMBER           = 0
ADDR_MX_FIRMWARE_VERSION       = 2
ADDR_MX_ID                     = 3
ADDR_MX_BAUD_RATE              = 4
ADDR_MX_RETURN_DELAY_TIME      = 5
ADDR_MX_CW_ANGLE_LIMIT         = 6
ADDR_MX_CCW_ANGLE_LIMIT        = 8
ADDR_MX_DRIVE_MODE             = 10
ADDR_MX_LIMIT_TEMPERATURE      = 11
ADDR_MX_MIN_VOLTAGE_LIMIT      = 12
ADDR_MX_MAX_VOLTAGE_LIMIT      = 13
ADDR_MX_MAX_TORQUE             = 14
ADDR_MX_STATUS_RETURN_LEVEL    = 16
ADDR_MX_ALARM_LED              = 17
ADDR_MX_SHUTDOWN               = 18
ADDR_MX_MULTI_TURN_OFFSET      = 20
ADDR_MX_RESOLUTION_DIVIDER     = 22

# RAM REGISTER ADDRESSES - resets after shut down
ADDR_MX_TORQUE_ENABLE          = 24
ADDR_MX_LED                    = 25
ADDR_MX_D_GAIN                 = 26
ADDR_MX_I_GAIN                 = 27
ADDR_MX_P_GAIN                 = 28
ADDR_MX_GOAL_POSITION          = 30
ADDR_MX_MOVING_SPEED           = 32
ADDR_MX_TORQUE_LIMIT           = 34
ADDR_MX_PRESENT_POSITION       = 36
ADDR_MX_PRESENT_SPEED          = 38
ADDR_MX_PRESENT_LOAD           = 40
ADDR_MX_PRESENT_VOLTAGE        = 42
ADDR_MX_PRESENT_TEMPERATURE    = 43
ADDR_MX_REGISTERED             = 44
ADDR_MX_MOVING                 = 46
ADDR_MX_LOCK                   = 47
ADDR_MX_PUNCH_L                = 48
ADDR_REALTIME_TICK             = 50
ADDR_CURRENT                   = 68
ADDR_TORQUE_CTRL_MODE_ENABLE   = 70
ADDR_GOAL_TORQUE               = 71
ADDR_GOAL_ACCELERATION         = 73

# Protocol version
PROTOCOL_VERSION               = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                        = 1                 # Dynamixel#1 ID : 1
DXL2_ID                        = 2                 # Dynamixel#2 ID : 2
DXL3_ID                        = 3                 # Dynamixel#3 ID : 3
DXL4_ID                        = 4                 # Dynamixel#4 ID : 4
BAUDRATE                       = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                     = '/dev/ttyUSB1'    # Port connected to controller
TORQUE_ENABLE                  = 1                 # Value for enabling the torque
TORQUE_DISABLE                 = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD    = 20                # Dynamixel moving status threshold

# Defined Parameter
DEBUG                          = True
DXL_MINIMUM_SPEED_VALUE        = 0           # Dynamixel will rotate between this value
DXL_MAXIMUM_SPEED_VALUE        = 1023            # and this value (note that the Dynamixel would not
DXL_GOAL_SPEED                 = [DXL_MINIMUM_SPEED_VALUE, DXL_MAXIMUM_SPEED_VALUE]

# Data Byte Length
LEN_MX_MOVING_SPEED            = 4
LEN_MX_PRESENT_SPEED           = 4

DXL_CW_ANGLE_TO_Z              = 0
DXL_CCW_ANGLETO_Z              = 0

mes = ''
Wheel_stop_left = 0
Wheel_stop_right = 0
Wheel_stop_up = 0
Wheel_stop_down = 0
Wheel_stop_ur = 0
Wheel_stop_ul = 0
Wheel_stop_dr = 0
Wheel_stop_dl = 0
Wheel_stop_rt = 0
Time = 0
dt = 0.1
Flag_OK = 1
Flag_CH = 1
Flag_stop = 0
count = 0
Vx = 0
Vy = 0
Wz = 0
preVx = 0
preVy = 0
preWz = 0
Lx = 0.45
Ly = 0.3
R = 0.04
rad_to_ms = 0.5
K = 40.92
sendvel = [0,0,0]
slow_ch = 0
prevTime = 0
Timestamp = 0
count0 = 0
ch = 0
eqm = numpy.array([[1, 1, -(Lx+Ly)], [1, -1, (Lx+Ly)], [1, -1, -(Lx+Ly)], [1, 1, (Lx+Ly)]])

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_MOVING_SPEED, LEN_MX_MOVING_SPEED )
MQTT_topic = [("unity/mobot/mobility",0),("Mobot/stop",0),("Mobot/shutdown",0),("Mobot/lidar",0),("Mobot/lidar/Q1_8",0)]

###################################################################################################################################################

if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

for id in range(1, 5):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d torque has been enable" % id)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, ADDR_MX_CW_ANGLE_LIMIT, DXL_CW_ANGLE_TO_Z)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, ADDR_MX_CCW_ANGLE_LIMIT, DXL_CCW_ANGLETO_Z)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully set to wheel mode" % id)

def Feedback():
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

def WriteDXL_Feedback(W1,W2,W3,W4):
	GetVelocity()
	client.publish("mobility/debug","Moving",2)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 1, ADDR_GOAL_ACCELERATION, 5)
	dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MX_MOVING_SPEED, W1)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 2, ADDR_GOAL_ACCELERATION, 5)
	dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 2, ADDR_MX_MOVING_SPEED, W2)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 4, ADDR_GOAL_ACCELERATION, 5)
	dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 4, ADDR_MX_MOVING_SPEED, W3)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 3, ADDR_GOAL_ACCELERATION, 5)
	dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 3, ADDR_MX_MOVING_SPEED, W4)
    
def WriteDXL_Feedback_stop(W1,W2,W3,W4):
	GetVelocity()
	client.publish("mobility/debug","Stop",2)
	dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MX_MOVING_SPEED, W1)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 1, ADDR_GOAL_ACCELERATION, 0)
	dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 2, ADDR_MX_MOVING_SPEED, W2)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 2, ADDR_GOAL_ACCELERATION, 0)
	dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 4, ADDR_MX_MOVING_SPEED, W3)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 4, ADDR_GOAL_ACCELERATION, 0)
	dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 3, ADDR_MX_MOVING_SPEED, W4)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 3, ADDR_GOAL_ACCELERATION, 0)


def GetVelocity():
	global sendvel
	vel1 = [0,0,0]
	vel2 = [0,0,0]
	vel3 = [0,0,0]
	vel4 = [0,0,0]
	velcal1 = 0
	velcal2 = 0
	velcal3 = 0
	velcal4 = 0
	vel1 = packetHandler.read2ByteTxRx(portHandler, 1, ADDR_MX_PRESENT_SPEED)
	if vel1[0] > 1023:
		velcal1 = (vel1[0]-1023)*-1*0.11*2*3.14/60
	else:
		velcal1 = vel1[0]*0.11*2*3.14/60
	vel2 = packetHandler.read2ByteTxRx(portHandler, 2, ADDR_MX_PRESENT_SPEED)
	if vel2[0] > 1023:
		if(vel2[0] == 0):
			velcal2 = 0
		else:
			velcal2 = (vel2[0]-1023)*0.11*2*3.14/60
	else:
		velcal2 = vel2[0]*-1*0.11*2*3.14/60
	vel3 = packetHandler.read2ByteTxRx(portHandler, 4, ADDR_MX_PRESENT_SPEED)
	if vel3[0] > 1023:
		velcal3 = (vel3[0]-1023)*-1*0.11*2*3.14/60
	else:
		velcal3 = vel3[0]*0.11*2*3.14/60
	vel4 = packetHandler.read2ByteTxRx(portHandler, 3, ADDR_MX_PRESENT_SPEED)
	if vel4[0] > 1023:
		if(vel4[0] == 0):
			velcal4 = 0
		else:
			velcal4 = (vel4[0]-1023)*0.11*2*3.14/60
	else:
		velcal4 = vel4[0]*-1*0.11*2*3.14/60
		# ~ vel4 = vel4[0]*0.11*2*3.14/60
		
	#print(vel1,vel2,vel3,vel4)
	allvel = numpy.array([[velcal1], [velcal2], [velcal3], [velcal4]])
	tra_mat = numpy.array([[1, 1, 1, 1],[1, -1, -1, 1],[-1/(Lx+Ly), 1/(Lx+Ly), -1/(Lx+Ly), 1/(Lx+Ly)]])
	sendvel = ((R/4)*tra_mat).dot(allvel)
####################################################################################################################################################

###########################################################
#########################################################################################

# The callback for when the client receives a CONNACK response from the server.

class DemoNode(): #Timer
	def __init__(self):
		self.timer = rospy.Timer(rospy.Duration(dt), self.demo_callback)
		self.timer2 = rospy.Timer(rospy.Duration(0.005), self.timer_callback)

	def timer_callback(self, timer2):
		global Time
		Time += dt

	def demo_callback(self, timer):
		global count
		global Timestamp
		global Flag_CH
		global send_temp
		global send_status

		client.publish("mobot/unity/temp",send_temp)
		client.publish("mobot/unity/status",send_status)

		send_to_unity = {'mobi_vx': float(sendvel[0]), 'mobi_vy' : float(sendvel[1])*-1, 'mobi_w': float(sendvel[2])}

		client.publish("mobot/unity/mobility",json.dumps(send_to_unity,sort_keys=True))

		if(count == 1):
			if(Time-Timestamp >= 0.35):
				Flag_CH = 1
				count = 0

####################################################################################################################################################



def callback_ridar(data):
	global Wheel_stop_up
	global Wheel_stop_left
	global Wheel_stop_down
	global Wheel_stop_right
	global Wheel_stop_ur
	global Wheel_stop_ul
	global Wheel_stop_dr
	global Wheel_stop_dl
	global Wheel_stop_rt
	global ridar_mes
	global Flag_CH
	global Flag_OK
	global Time
	msg = data.data
	ridar_mes = msg.split()
	Q1 = ridar_mes[0]
	Q2 = ridar_mes[1]
	Q3 = ridar_mes[2]
	Q4 = ridar_mes[3]
	Q5 = ridar_mes[4]
	Q6 = ridar_mes[5]
	Q7 = ridar_mes[6]
	Q8 = ridar_mes[7]

	if(Flag_CH == 1):

		if  Q5 == "STOP":
			print("Detect LEFT")
			Wheel_stop_left = 1
			Flag_OK = 0
			Flag_CH = 0
			Time = 0


		elif  Q1 == "STOP":
			print("Detect RIGHT")
			Wheel_stop_right = 1
			Flag_CH = 0
			Flag_OK = 0
			Time = 0


		elif  Q7 == "STOP":
			print("Detect DOWN")
			Wheel_stop_down = 1
			Flag_CH = 0
			Flag_OK = 0
			Time = 0


		elif  Q3 == "STOP":
			print("Detect UP")
			Wheel_stop_up = 1
			Flag_CH = 0
			Flag_OK = 0
			Time = 0


		elif  Q2 == "STOP":
			print("Detect UP")
			Wheel_stop_ur = 1
			Flag_CH = 0
			Flag_OK = 0
			Time = 0


		elif  Q4 == "STOP":
			print("Detect UP")
			Wheel_stop_ul = 1
			Flag_CH = 0
			Flag_OK = 0
			Time = 0


		elif  Q6 == "STOP":
			print("Detect UP")
			Wheel_stop_dl = 1
			Flag_CH = 0
			Flag_OK = 0
			Time = 0

		elif  Q8 == "STOP":
			print("Detect UP")
			Wheel_stop_dr = 1
			Flag_CH = 0
			Flag_OK = 0
			Time = 0


def listener():
    rospy.Subscriber("chatter", String, callback_ridar)
    DemoNode()

def on_connect(client, userdata, flags, rc):
	global flag_connected
	print("Connected with result code "+str(rc))
	listener()
	flag_connected = 1
	client.subscribe(MQTT_topic)

####################################################################################################################################################

# The callback for when a PUBLISH message is received from the server.

def on_message(client, userdata, msg):
	global mes
	global Flag_OK
	global Flag_CH
	global Flag_stop
	global Wheel_stop_up
	global Wheel_stop_left
	global Wheel_stop_down
	global Wheel_stop_right
	global Wheel_stop_ur
	global Wheel_stop_ul
	global Wheel_stop_dr
	global Wheel_stop_dl
	global Wheel_stop_rt
	global ridar_mes
	global count
	global Timestamp
	global Time
	global slow_ch
	global Vx
	global Vy
	global Wz
	global prevTime

	topic = msg.topic
	mes = msg.payload

	
	
	if(topic == 'Mobot/lidar'):
		if mes == "SLOW":
			slow_ch = 1
		else:
			slow_ch = 0

	elif(topic == 'unity/mobot/mobility'):
		st = json.loads(mes)
		Vx = (st["mobi_vx"])/3
		Vy = (st["mobi_vy"])/3
		Vy = (Vy * -1)
		Wz = (st["mobi_w"])/1.5
		Timestamp = Time

		if Time > 1000:
			Time = 0
			prevTime = 0
			Timestamp = 0


		if slow_ch == 1:
			Vx = Vx/1.5
			Vy = Vy/1.5
			Wz = Wz

	elif(topic == 'Mobot/stop'):
		if mes == "STOP":
			Vx = 0
			Vy = 0
			Wz = 0
			Flag_stop = 1

		if mes == "OK": 
			Flag_stop = 0




def main():
	global Flag_OK
	global Flag_CH
	global Flag_stop
	global Wheel_stop_up
	global Wheel_stop_left
	global Wheel_stop_down
	global Wheel_stop_right
	global Wheel_stop_ur
	global Wheel_stop_ul
	global Wheel_stop_dr
	global Wheel_stop_dl
	global Wheel_stop_rt
	global ridar_mes
	global count
	global count0
	global Timestamp
	global prevTime
	global Time
	global slow_ch
	global Vx
	global Vy
	global Wz
	global preVx
	global preVy
	global preWz
	global send_temp
	global send_status
	global ch

	GetVelocity()
	temp = [0,0,0,0]
	temp[0] = packetHandler.read1ByteTxRx(portHandler, 1, ADDR_MX_PRESENT_TEMPERATURE)
	temp[1] = packetHandler.read1ByteTxRx(portHandler, 2, ADDR_MX_PRESENT_TEMPERATURE)
	temp[2] = packetHandler.read1ByteTxRx(portHandler, 4, ADDR_MX_PRESENT_TEMPERATURE)
	temp[3] = packetHandler.read1ByteTxRx(portHandler, 3, ADDR_MX_PRESENT_TEMPERATURE)
	status = [0,0,0,0]
	status[0] = packetHandler.read2ByteTxRx(portHandler, 1, ADDR_MX_PRESENT_LOAD)
	status[1] = packetHandler.read2ByteTxRx(portHandler, 2, ADDR_MX_PRESENT_LOAD)
	status[2] = packetHandler.read2ByteTxRx(portHandler, 4, ADDR_MX_PRESENT_LOAD)
	status[3] = packetHandler.read2ByteTxRx(portHandler, 3, ADDR_MX_PRESENT_LOAD)
	send_temp = str(temp[0]) + str(temp[1]) + str(temp[2]) + str(temp[3])
	send_status = str(status[0]) + ' ' + str(status[1])+ ' ' + str(status[2])+ ' ' + str(status[3])

	if Timestamp - prevTime == 0:
		count0 += 1
	else:
		count0 = 0
	prevTime = Timestamp

	if count0 > 10:
		Vx = 0
		Vy = 0
		Wz = 0

	if Wheel_stop_up == 1:
		if(Vx < 0 and Vy > -0.2 and Vy < 0.2):
			Flag_OK = 1
			Timestamp = Time
			count = 1
			Wheel_stop_up = 0

			
	elif Wheel_stop_down == 1:
		if(Vx > 0 and Vy > -0.2 and Vy < 0.2):
			Flag_OK = 1
			Timestamp = Time
			count = 1
			Wheel_stop_down = 0


	elif Wheel_stop_left == 1:
		if(Vy < 0 and Vx > -0.2 and Vx < 0.2):
			Flag_OK = 1
			Timestamp = Time
			count = 1
			Wheel_stop_left = 0



	elif Wheel_stop_right == 1:
		if(Vy > 0 and Vx > -0.2 and Vx < 0.2):
			Flag_OK = 1
			Timestamp = Time
			count = 1
			Wheel_stop_right = 0



	elif Wheel_stop_dr == 1:
		if(Vx > 0 and Vy > 0):
			Flag_OK = 1
			Timestamp = Time
			count = 1
			Wheel_stop_dr = 0
		else:
			Flag_OK = 0


	elif Wheel_stop_dl == 1:
		if(Vx > 0 and Vy < 0):
			Flag_OK = 1
			Timestamp = Time
			count = 1
			Wheel_stop_dl = 0
		else:
			Flag_OK = 0



	elif Wheel_stop_ur == 1:
		if(Vx < 0 and Vy > 0):
			Flag_OK = 1
			Timestamp = Time
			count = 1
			Wheel_stop_ur = 0
		else:
			Flag_OK = 0


	elif Wheel_stop_ul == 1:
		if(Vx < 0 and Vy < 0):
			Flag_OK = 1
			Timestamp = Time
			count = 1
			Wheel_stop_ul = 0
		else:
			Flag_OK = 0


	if(Flag_OK == 1 and Flag_stop == 0):

		robot_vel = numpy.array([[Vx], [Vy], [Wz]])
		wheel_vel = ((1/R)*eqm).dot((robot_vel))
		wheel_vel = numpy.absolute(wheel_vel)
		findK = numpy.max(wheel_vel)
		if findK != 0:
			K = 1023/findK 
		else:
			K = 0
		wheel_vel = ((1/R)*eqm).dot((robot_vel))*K*math.sqrt((Vx*Vx)+(Vy*Vy))
		if Vx == 0 and Vy == 0 and Wz != 0:
			wheel_vel = ((1/R)*eqm).dot((robot_vel))*10


		W1 = int(math.floor(wheel_vel[0])) #0-1023
		if(W1 > 1023):
			W1 = 1023
		elif(W1 < -1023):
			W1 = -1023
		W2 = int(math.floor(wheel_vel[1])) #1024-2046
		if(W2 > 1023):
			W2 = 1023
		elif(W2 < -1023):
			W2 = -1023
		W3 = int(math.floor(wheel_vel[2])) #0-1023
		if(W3 > 1023):
			W3 = 1023
		elif(W3 < -1023):
			W3 = -1023
		W4 = int(math.floor(wheel_vel[3])) #1024-2046
		if(W4 > 1023):
			W4 = 1023
		elif(W4 < -1023):
			W4 = -1023
			
			
		if(W1 < 0):
			W1 = abs(W1)+1023

		if(W2 < 0):
			W2 = abs(W2)
		else:
			W2 = W2+1023

		if(W3 < 0):
			W3 = abs(W3) +1023

		if(W4 < 0):
			W4 = abs(W4)
		else:
			W4 = W4+1023


		if Vx != 0 or Vy != 0 or Wz != 0:
			for id in range(1, 5):
				dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
				if dxl_comm_result != COMM_SUCCESS:
					print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
				elif dxl_error != 0:
					print("%s" % packetHandler.getRxPacketError(dxl_error))
				
			WriteDXL_Feedback(W1,W2,W3,W4)
			

	if Vx == 0 and Vy == 0 and Wz == 0:

		W1 = 0
		W2 = 0
		W3 = 0
		W4 = 0
		WriteDXL_Feedback_stop(W1,W2,W3,W4)
		if ch == 1:
			for id in range(1, 5):
				dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
				if dxl_comm_result != COMM_SUCCESS:
					print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
				elif dxl_error != 0:
					print("%s" % packetHandler.getRxPacketError(dxl_error))
			ch = 0


if __name__ == '__main__':
	client = mqtt.Client()
	client.on_connect = on_connect
	client.on_message = on_message
	client.on_disconnect = on_disconnect
	rospy.init_node('Mobility_listener')

	client.connect("192.168.111.2",1883)

	client.loop_start()
	while True:
		main()

