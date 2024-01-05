#!/usr/bin/python
# -*- coding:utf-8 -*-

import RPi.GPIO as GPIO

import serial
import time

import traceback
import sys

class GPS:
	def __init__(self, debug_mode = False):
		self.ser = serial.Serial('/dev/ttyS0', 115200)
		self.ser.flushInput()

		self.POWER_KEY = 6
		self.debug_mode = debug_mode
		self.rec_buff = ''

		self.latitude = 0
		self.longitude = 0

		self.activate()
	
	def __del__(self):
		if self.ser != None:
			self.ser.close()
			GPIO.cleanup()

	def activate(self):
		try:
			self.power_on()
			self.fetch_gps_position()
			self.power_down()
		except Exception:
			print(traceback.format_exc())
			if self.ser != None:
				self.ser.close()
			self.power_down()
			GPIO.cleanup()

	def power_on(self):
		if self.debug_mode:
			print('SIM7600X is starting:')
			
		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup(self.POWER_KEY, GPIO.OUT)
		time.sleep(0.1)
		GPIO.output(self.POWER_KEY, GPIO.HIGH)
		time.sleep(2)
		GPIO.output(self.POWER_KEY, GPIO.LOW)
		time.sleep(20)
		self.ser.flushInput()

		if self.debug_mode:
			print('SIM7600X is ready')

	def fetch_gps_position(self):
		rec_null = True
		answer = 0

		if self.debug_mode:
			print('Start GPS session...')
		
		self.rec_buff = ''
		self.send_at('AT+CGPS=1,1', 'OK', 1)
		time.sleep(2)

		while rec_null:
			answer = self.send_at('AT+CGPSINFO', '+CGPSINFO: ', 1)
			if 1 == answer:
				answer = 0
				if ',,,,,,' in self.rec_buff.decode():

					if self.debug_mode:
						print('GPS is not ready')
					
					rec_null = False
					time.sleep(1)
			else:
				if self.debug_mode:
					print('error %d' %answer)

				self.rec_buff = ''
				self.send_at('AT+CGPS=0', 'OK', 1)
				return False
			time.sleep(1.5)

	# Returns 0 if failed, 1 if success
	def send_at(self, command, back, timeout):
		self.rec_buff = ''
		self.ser.write((command + '\r\n').encode())
		time.sleep(timeout)

		if self.ser.inWaiting():
			time.sleep(0.01)
			self.rec_buff = self.ser.read(self.ser.inWaiting())
		
		if self.rec_buff != '':
			if back not in self.rec_buff.decode():
				if self.debug_mode:
					print(command + ' ERROR')
					print(command + ' back:\t' + self.rec_buff.decode())
				return 0
			else:
				if self.debug_mode:
					print(self.rec_buff.decode())
				
				self.decode_lat_long()
				print("Latitude: %f, Longitude: %f" %(self.latitude, self.longitude))

				return 1
		else:
			if self.debug_mode:
				print('GPS is not ready')
			return 0

	def decode_lat_long(self):
		lat = ''
		long = ''

		decoded_string = self.rec_buff.decode()

		# Get the latitude string
		i = 12
		while decoded_string[i] != ',':
			lat = lat + str(decoded_string[i])
			i += 1
		i += 1

		# If the latitude is S, make the string negative
		if decoded_string[i] == 'S':
			lat = '-' + lat
		i += 2

		# Get the longitude string
		while decoded_string[i] != ',':
			long = long + str(decoded_string[i])
			i += 1
		i += 1

		# If the longitude is S, make the string negative
		if decoded_string[i] == 'W':
			long = '-' + long

		# Normalize the decimal system
		lat_dot_ind = 0
		long_dot_ind = 0

		for j in range(0, len(lat)):
			if lat[j] == '.':
				lat_dot_ind = j

		left_str = lat[: lat_dot_ind]
		right_str = lat[lat_dot_ind + 1 :]
		moved_str = left_str[len(left_str) - 2 :]
		lat = left_str[: len(left_str) - 2] + '.' + moved_str + right_str

		for j in range(0, len(long)):
			if long[j] == '.':
				long_dot_ind = j

		left_str = long[: long_dot_ind]
		right_str = long[long_dot_ind + 1 :]
		moved_str = left_str[len(left_str) - 2 :]
		long = left_str[: len(left_str) - 2] + '.' + moved_str + right_str

		self.latitude = float(lat)
		self.longitude = float(long)


	def power_down(self):
		if self.debug_mode:
			print('SIM7600X is logging off:')
		GPIO.output(self.POWER_KEY,GPIO.HIGH)
		time.sleep(3)
		GPIO.output(self.POWER_KEY,GPIO.LOW)
		time.sleep(18)
		if self.debug_mode:
			print('Good bye')


	# Getters: Return '' if not ready or if there's no reception
	def get_latitude(self):
		return self.latitude
	def get_longitude(self):
		return self.longitude