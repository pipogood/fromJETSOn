#!/usr/bin/env python
import paho.mqtt.client as mqtt #import the client1
import rospy
import json
import sys
import os
import time
import math
from math import sin, cos
from open_manipulator_msgs.msg import KinematicsPose
from sensor_msgs.msg import JointState
from open_manipulator_msgs.msg import OpenManipulatorState
from open_manipulator_msgs.srv import *

home = "2 0 0 0 0 0 0 5"
set_stand = "2 0 0 -1.57 0 0 0 5"
shutdown = ""
stop = 0
stopend = 0
mode = 1
pos_x = 0.659
pos_y = 0
pos_z = 0.25
pitch = 0
prev_x = 0.659
prev_y = 0
prev_z = 0.25
prev_yaw = 0
prev_pitch = 0
prev_roll = 0
grip_joint = 0
prev_grip = 0
prev_j1 = 0
prev_j2 = 0
prev_j3 = 0
prev_j4 = 0
prev_j5 = 0
prev_j6 = 0
j1 =0
j2 =0
j3 =0
j4 =0
j5 =0
j6 =0
flag_reset = 0
dt = 0.1
Time = 0
topic = ""
flag_stop = 0
flag_overw = 0
lock = 0
prevTime = 0
Timestamp = 0
count0 = 0
over_limit = 0
statouni = "yes"
MQTT_topic = [("unity/mobot/manipulator",0),("unity/mobot/gripper",0),("unity/mobot/manual",0),("Mobot/stop",0),("Mobot/shutdown",0),("Mobot/smart",0)]

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("connected OK")
        client.subscribe(MQTT_topic)
        client.publish("manipulator/debug","connect ok",2)
        print("Idle State")
        client.publish("manipulator/debug","Idle State",2)
    else:
        print("Bad connection Returned code=", rc)

def on_disconnect(client, userdata, flags,rc=0):
    print("DisConnected result code" +str(rc))

def on_message(client,userdata,msg):
	global mes
	global stop
	global topic
	global shutdown
	global mode
	global grip_joint
	global prev_grip
	global flag_stop
	global flag_reset
	global prev_j1
	global prev_j2
	global prev_j3
	global prev_j4
	global prev_j5
	global prev_j6
	global over_limit
	global j1
	global j2
	global j3
	global j4
	global j5
	global j6   
	global lock
	global count0
	global Timestamp
	global prevTime
	global Time
	global grip_enter

	topic = msg.topic
	m_decode=str(msg.payload.decode("utf-8","ignore"))
	mes = m_decode
	over_limit = 0
	
	if topic == 'unity/mobot/gripper' and flag_stop == 0 and flag_reset == 0:
		st = json.loads(mes)
		grip_enter = 1
		grip_joint = (st["gripper"])
		Gripper_Control(grip_joint,1)
		stop = 0
		
	elif topic == 'unity/mobot/manipulator' and flag_stop == 0 and flag_reset == 0:
		Timestamp = Time
		if Time > 1000:
			Time = 0
			prevTime = 0
			Timestamp = 0
		st = json.loads(mes)
		mode = 1
		y = (st["mani_y"])*0.02
		z = (st["mani_z"])*0.015
		if over_limit == 0:
			if z == 0:
				prev_j2 = joint2
				prev_j5 = joint5
			if y == 0:
				prev_j1 = joint1
			j2 = prev_j2 + (z*-1)
			j5 = prev_j5 + (z*-1)
			j1 = prev_j1 + y
			
	elif topic == 'unity/mobot/manual' and flag_stop == 0 and flag_reset == 0:
		st = json.loads(mes)
		mode = 0
		over_limit = 0
		get = (st['manual'])
		if get == "mani_home":
			set_forward_client(0,0,0,0,0,0,3)
			flag_reset = 1
			j1 = 0
			j2 = 0
			j5 = 0
			prev_j1 = 0
			prev_j2 = 0
			prev_j5 = 0
			
	elif topic == 'Mobot/stop':
		if mes == "STOP":
			flag_stop = 1
			set_forward_client(joint1,joint2,joint3,joint4,joint5,joint6,0.1)
			stop = 1
		if mes == "OK":
			flag_stop = 0
			stop = 0
	
	elif topic == 'Mobot/smart':
		if mes == "LOCK":
			lock = 1
			j1 = 0
			j2 = 0
			j5 = 0
			client.publish("unity/mobot/manual",json.dumps({'manual':'mani_home'}))
			client.publish("Mobot/smart","NOW is LOCK")
		elif mes == "UNLOCK":
			lock = 0
			client.publish("Mobot/smart","NOW is UNLOCK")

	elif topic == 'Mobot/shutdown':
		stop = 1
		shutdown = "SHUTDOWN"


def run_begin():
    client.publish("manipulator/debug","Initialize",2)
    service_name = '/goal_joint_space_path'
    target_angle = [0,0,0,0,0,0]
    joint_name = ["Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6"]
    rospy.wait_for_service(service_name)
    try:
        set_position = rospy.ServiceProxy(service_name, SetJointPosition)
        arg = SetJointPositionRequest()
        for i in range(0,6):
            arg.joint_position.joint_name.append(joint_name[i])
            arg.joint_position.position.append(target_angle[i])
            arg.path_time = 5
        resp1 = set_position(arg)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return False

def callback_joint(data):
    global joint1
    global joint2
    global joint3
    global joint4
    global joint5
    global joint6

    data_j = data.position
    joint1 = data_j[0]
    joint2 = data_j[1]
    joint3 = data_j[2]
    joint4 = data_j[3]
    joint5 = data_j[4]
    joint6 = data_j[5]

class DemoNode(): #Timer
	def __init__(self):
		self.timer = rospy.Timer(rospy.Duration(dt), self.demo_callback)
		self.timer2 = rospy.Timer(rospy.Duration(0.005), self.timer_callback)

	def timer_callback(self, timer2):
		global Time
		Time += dt

	def demo_callback(self, timer):
		global statouni
		global flag_reset
		send_to_unity = {'mani_x': 0, 'mani_y' : joint1*57.2, 'mani_z': joint2*57.2, 'workspace': 'yes'}
		if(joint1 > -0.01 and joint1 < 0.01 and joint2 > -0.01 and joint2 < 0.01 and joint5 > -0.01 and joint5 < 0.01):
			flag_reset = 0
		client.publish("mobot/unity/manipulator",json.dumps(send_to_unity,sort_keys=True))


def callback_position(data):
    global data_x
    global data_y
    global data_z
    global data_ox
    global data_oy
    global data_oz
    global data_ow
    data_x = float(data.pose.position.x)
    data_y = float(data.pose.position.y)
    data_z = float(data.pose.position.z)
    data_ox = data.pose.orientation.x
    data_oy = data.pose.orientation.y
    data_oz = data.pose.orientation.z
    data_ow = data.pose.orientation.w

def callback_moving_status(data):
    global status
    status = data.open_manipulator_moving_state


def listener_joint_position():
    rospy.Subscriber('/joint_states', JointState, callback_joint)
    rospy.Subscriber('/gripper/kinematics_pose', KinematicsPose, callback_position)
    rospy.Subscriber('/states', OpenManipulatorState, callback_moving_status)
    DemoNode()
    rospy.sleep(0.05)

def set_state(state):
    service_name2 = '/set_actuator_state'
    rospy.wait_for_service(service_name2)
    set_actuator = rospy.ServiceProxy(service_name2, SetActuatorState)
    arg = SetActuatorStateRequest()
    try:
	arg.set_actuator_state = state
	resp1 = set_actuator(arg)
	return resp1

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return False

def set_inverse_client(x, y, z, yaw ,pitch, roll, grip_joint, dt):
	global flag_overw
	global statouni
	flag_overw = 0
	service_name1 = '/goal_joint_space_path_to_kinematics_pose'
	rospy.wait_for_service(service_name1)
	set_position = rospy.ServiceProxy(service_name1, SetKinematicsPose)
	arg = SetKinematicsPoseRequest()
	arg.end_effector_name = 'gripper'
	try:
		arg.kinematics_pose.pose.position.x = x
		arg.kinematics_pose.pose.position.y = y
		arg.kinematics_pose.pose.position.z = z
		cy = cos(yaw * 0.5)
		sy = sin(yaw * 0.5)
		cp = cos(pitch * 0.5)
		sp = sin(pitch * 0.5)
		cr = cos(roll * 0.5)
		sr = sin(roll * 0.5)
		ow = cr * cp * cy + sr * sp * sy
		ox = sr * cp * cy - cr * sp * sy
		oy = cr * sp * cy + sr * cp * sy
		oz = cr * cp * sy - sr * sp * cy
		arg.kinematics_pose.pose.orientation.w = ow
		arg.kinematics_pose.pose.orientation.x = ox
		arg.kinematics_pose.pose.orientation.y = oy
		arg.kinematics_pose.pose.orientation.z = oz
		arg.path_time = dt
		resp1 = set_position(arg)
		rospy.sleep(0.1)
		if status == '"STOPPED"':
			statouni = "no"
			flag_overw = 1
		if status == '"IS_MOVING"' and flag_overw == 0:
			statouni = "yes"

	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
		return False


def Gripper_Control(grip_joint,time):
    service_name = '/goal_tool_control'
    rospy.wait_for_service(service_name)
    try:
        set_position = rospy.ServiceProxy(service_name, SetJointPosition)
        arg = SetJointPositionRequest()
        arg.joint_position.joint_name.append("gripper")
        arg.joint_position.position.append(grip_joint)
        arg.path_time = time
        resp1 = set_position(arg)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return False


def set_forward_client(j1,j2,j3,j4,j5,j6,time):
    service_name = '/goal_joint_space_path'
    target_angle = [j1,j2,j3,j4,j5,j6]
    joint_name = ["Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6"]
    rospy.wait_for_service(service_name)
    try:
		set_position = rospy.ServiceProxy(service_name, SetJointPosition)
		arg = SetJointPositionRequest()
		for i in range(0,6):
			arg.joint_position.joint_name.append(joint_name[i])
			arg.joint_position.position.append(target_angle[i])
			arg.path_time = time
		resp1 = set_position(arg)
		return resp1

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return False


def run_mode():
	global dt
	global grip_joint
	global prev_grip
	global prev_j1
	global prev_j2
	global prev_j3
	global prev_j4
	global prev_j5
	global prev_j6
	global j1
	global j2
	global j3
	global j4
	global j5
	global j6   
	global mode
	global stop
	global statouni
	global lock
	global over_limit
	global count0
	global Timestamp
	global prevTime
	global Time
	global grip_enter
	over_limit = 0
	grip_enter = 0
	if Timestamp - prevTime == 0:
		count0 += 1
	else:
		count0 = 0
	prevTime = Timestamp
	if count0 > 10:
		pass
	
	if stop == 0 and flag_stop == 0:
		if mode == 1:
			dt = 1
			if(abs(j1) > 0.6): #0.58
				over_limit = 1

			if(j2 > 0.68):
				over_limit = 1

			if(j2 < -0.05):
				over_limit = 1
				
			if lock == 1:
				if(j2 > 0.2):
					over_limit = 1

			if(over_limit == 1):
				print("Over Limit Workspace")
				client.publish("manipulator/debug","Over Limit Workspace",2)
				prev_j1 = j1
				prev_j2 = j2
				prev_j3 = j3
				prev_j4 = j4
				prev_j5 = j5
				prev_j6 = j6
				prev_grip = grip_joint
			else:						
				if((j1 != prev_j1) or (j2 != prev_j1) or (j5 != prev_j5) or (grip_joint != prev_grip)):
					response = set_forward_client(j1,j2,0,0,j5,0,dt)
					prev_j1 = j1
					prev_j2 = j2
					prev_j3 = j3
					prev_j4 = j4
					prev_j5 = j5
					prev_j6 = j6
					prev_grip = grip_joint

		elif mode == 2:
			split = set2.split()
			j1 = float(split[1])
			j2 = float(split[2])
			j3 = float(split[3])
			j4 = float(split[4])
			j5 = float(split[5])
			j6 = float(split[6])
			dt = float(split[7])
			if((j1 != prev_j1) or (j2 != prev_j2) or (j3 != prev_j3) or (j4 != prev_j4) or (j5 != prev_j5) or (j6 != prev_j6)):
				prev_j1 = j1
				prev_j2 = j2
				prev_j3 = j3
				prev_j4 = j4
				prev_j5 = j5
				prev_j6 = j6
				prev_grip = grip_joint

def myhook():
	client.publish("manipulator/debug","Shutdown and Reinitialize",2)
	print("Shutdown and Reintialize")

rospy.init_node('master_publisher')
broker_address="broker.hivemq.com"

client = mqtt.Client("master")
client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.on_message = on_message
print("Conecting to broker",broker_address)
client.connect("192.168.111.2",1883)
run_begin()
listener_joint_position()
client.loop_start()

while True:
    time.sleep(0.1)
    if shutdown == "SHUTDOWN":
		set_forward_client(0,0,-1.57,0,0,-1.57,0,5)
		break
    else:
        run_mode()

os.system("rosnode kill master_publisher")
rospy.on_shutdown(myhook)
client.loop_stop()
client.disconnect()

