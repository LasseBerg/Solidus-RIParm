# ArmPrintBridge
#	Send Controll commands to arm controller (Arduino Mega)
#	Recive Joint Controll Commands from mqtt (Retain)
# 
# V1.0
# 
# 24.4.2023
# Benjamin Zbinden


import rclpy
from rclpy.node import Node

from std_msgs.msg import String

#Custom imports
import paho.mqtt.client as mqtt
import threading
import serial
import time
import os

## Global Variables
port = "/dev/ttyACM" #Port Arduino Mega
msg_array = ["0","0","0","0","0","0"]

class ArmPrintBridge(Node):

	#self.notstop_gui = False
	#notstop_basis = False

	def __init__(self, port):
		super().__init__('ArmPrintBridge')

		self.lastCommand = ""

		self.ser = openSerial(port)
		self.z = 0

		##def startMqtt():
##
		##	root_topic="solidus/arm/stepper/"
		##	topics_to_subscribe=["wrist_c","wrist_y","elbow_y","shoulder_y","shoulder_c","gripper"]
		##	broker="192.168.50.210"
##
		##	### Thread for MQTT Subscribing ###
		##	def subscribing():
		##		def on_connect(client, userdata, flags, rc):
		##			print("CONNECTED")
		##			print("Connected with result code: ", str(rc))
##
		##		def on_message(client, userdata, msg):
		##			#print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
##
		##			##if "solidus/station/gui/notstop" in msg.topic:
		##			##	global notstop_gui
		##			##	state = msg.payload.decode()
		##			##	if(state == "0"):
		##			##		notstop_gui = False
		##			##	else:
		##			##		notstop_gui = True
		##			##	return
		##			##
		##			##if "solidus/basis/notstop" in msg.topic:
		##			##	global notstop_basis
		##			##	state = msg.payload.decode()
		##			##	if(state == "0"):
		##			##		notstop_basis = False
		##			##	else:
		##			##		notstop_basis = True
		##			##	return
##
		##			if "solidus/arm/stepper/" in msg.topic:
		##				print("new data :D")
		##				for i in range(len(topics_to_subscribe)):
		##					if(msg.topic == root_topic + topics_to_subscribe[i]):
		##						msg_array[i]=msg.payload.decode()
##
		##		def on_disconnect(client, userdata, rc):
		##			print("DISCONNECTED")
##
		##		
		##		### MQTT ###
		##		client = mqtt.Client()
		##		client.connect(broker, 1883, 60)
		##		client.subscribe("solidus/arm/stepper/#")
		##		#client.subscribe("solidus/basis/sensor/notstop")
		##		#client.subscribe("solidus/station/gui/notstop")
		##		client.on_connect = on_connect
		##		client.on_disconnect = on_disconnect
		##		client.on_message = on_message
		##		client.loop_start()
##
		##	sub=threading.Thread(target=subscribing)
		##	sub.start()

		#startMqtt()
	 
		################################### TIMERS FOR CALLING ARM BOARD ###########################################
		self.timer_period_arm_command = float(100)/1000  # seconds
		self.arm_command_timer = self.create_timer(self.timer_period_arm_command, self.send_arm_command_callback)

	
		self.arm_command_sub = self.create_subscription(
			String,
			'arm_command',
			self.arm_command_lis,
			10)
		self.arm_command_sub  # prevent unused variable warning

	def arm_command_lis(self, msg):
		data = json.dump(msg.data)
		index = data["joint"]
		value = data["dir"]
		msg_array[index] = value





	def assamble_msg(self):
		message = "m "
		for msg in msg_array:
			message = message + msg + " "
		
		# strip last char from message (space)
		message = message[:-1]

		return message



	################################### SEND ARM COMMANDS (CYCLIC FROM TIMER) ################################################

	def send_arm_command_callback(self):

		#if(self.notstop_gui or notstop_basis):
		#	print("NOTSTOP")
		#	writeSerial(self.ser, "m 0 0 0 0 0 0" + '\r')
		#else:
		currentCommand = self.assamble_msg()
		writeSerial(self.ser, currentCommand + '\r')

		self.z = self.z + 1
		if(self.z >= 1):
			print(f"Send Command: {currentCommand}")
			self.z = 0



	################################### Serial Stuff for Communication width Arduino Mega ################################################

def writeSerial(ser, msg):
	ser.write(msg.encode('utf-8'))

def openSerial(port):
	ser = serial.Serial()
	ser.timeout = 2
	ser.baudrate = 115200
	ser.port = port
	ser.open()
	ser.reset_input_buffer()
	ser.reset_output_buffer()
	print('Opening Serial Port: ' + port)
	time.sleep(2.5)
	print('Serial Ready')
	return ser

#################################### LOAD PARAMETERS ########################################################

def loadParameters():

	def getVarFromFile(filename):
			import imp
			f = open(filename)
			data = f.readline()
			f.close()
			return data

	# path to "config" file
	data = getVarFromFile('/home/robot/parameters/ArmPrintBridge.txt')
	global port
	port = port + data.strip('\n')



#################################### MAIN ########################################################

def main(args=None):
	# Set Default Values for Message Array (Mittelposition of Joints)
	for i in range(len(msg_array)):
		msg_array[i] = "0"

	loadParameters()
	print(f"Port: {port}")
	
	rclpy.init(args=args)
	armPrintBridge = ArmPrintBridge(port)
	rclpy.spin(armPrintBridge)
	armPrintBridge.destroy_node()
	rclpy.shutdown()




