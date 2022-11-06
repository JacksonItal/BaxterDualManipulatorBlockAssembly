#!/usr/bin/env python

from robotiq.msg import Service
from robotiq.srv import Actions

import rospy
import numpy as np
import serial
import binascii as ba

def crcCalc(data):
	
	crc =np.ushort(0xFFFF)

	for pos in range(len(data)):
		# print(ord(data[pos]))
		crc ^= np.ushort(ord(data[pos]))
		for i in range(8):
			if (crc & 0x0001) != 0:
				crc >>= 1
				crc ^= 0xA001
			else:
				crc >>= 1
	
	return ba.hexlify(crc)

class RobIQ:
	def __init__(self):
		self.ros_initialized = False
		
		if self.ros_initialized:
			return 0

		#self._enable_disable_service = rospy.Service('%s/enable_disable'%rospy.get_name(), enable_disable, self.enableDisable)
		# SERVICES
		self._hand_service = rospy.Service('%s/actions'%rospy.get_name(), Actions, self.handActions)
		# self._set_control_mode_service = rospy.Service('%s/set_control_mode'%rospy.get_name(), SetControlMode)
	
		# List where saving the required actions
		self.actions_list = []

		self.ros_initialized = True
		rospy.loginfo('%s::RobotIQ: Ready...'%(rospy.get_name()))
		self.com = b'\x00'
		
		# self.publishROSstate()
			
	def serialCom(self, com):
		ser = serial.Serial()
		ser.timeout = 10
		ser.baudrate = 115200
		ser.port = '/dev/ttyUSB0'
		ser.open()

		ser.write(com)
		self.res = ser.read(8)
		# print(repr(res))
		ser.close()

	def handActions(self, req):
		# rospy.wait_for_service('%s/actions'%rospy.get_name())
		self.actions_list.append(req.action)

		rospy.loginfo('%s::handActions: Received new action %s'%(rospy.get_name(), req.action))
		self.robotiqGripper(str(req.action))
		rospy.loginfo('%s::handActions: Serial Send %s'%(rospy.get_name(), repr(self.com)))
		rospy.loginfo('%s::handActions: Serial Return %s'%(rospy.get_name(), repr(self.res)))


		return self.ret


	def robotiqGripper(self, action):
		rate = rospy.Rate(1)

		codest = b'\x09\x10\x03\xe8\x00\x03\x06\x09\x00\x00'
		codeen = b'\xff\xff'
		pos = b'\x00'

		try:
			if len(self.actions_list) > 0:
				action = self.actions_list[0]
				self.actions_list.remove(action)

				if action == Service.OPEN_GRASP:
					pos = b'\x00'
				elif action == Service.CLOSE_GRASP:
					pos = b'\xff'
				elif action == Service.HALF_OPEN_GRASP:
					pos = b'\x7f'
				else:
					rospy.logerr("invlid action >> opening")

				c = str(ba.hexlify(codest))
				v = str(ba.hexlify(pos))
				b = str(ba.hexlify(codeen))

				code = c + v + b
				code = ba.unhexlify(code)

				ret = crcCalc(code)
				ret = ba.hexlify(code) + ret[0:4]
				self.com = ba.unhexlify(ret)

				self.serialCom(self.com)


				rate.sleep()
			self.ret = True
		except (rospy.ServiceException) as e:
			rospy.logerr("**Service Call Failed: %s**" %e)
			self.ret = False

		# ser.write(b'\x09\x03\x07\xd0\x00\x02\xc5\xce')
		# ser.write(b'\x09\x10\x03\xe8\x00\x03\x06\x09\x00\x00\xff\xff\xff\x42\x29') # close full speed full forse
		# ser.write(b'\x09\x10\x03\xe8\x00\x03\x06\x09\x00\x00\x00\xff\xff\x72\x19') #open full speed full forse


def main():
	"""
	controls robotIQ gripper through serail commands
	"""
	rospy.init_node("robotIQ_node")

	riq = RobIQ()
	# riq.robotiqGripper()

	rospy.spin()

	rospy.loginfo('%s::RobotIQ: Done.'%(rospy.get_name()))
	return 0

if __name__ == '__main__':
  main()