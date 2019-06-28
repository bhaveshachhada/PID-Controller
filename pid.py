import numpy as np
import math
from time import time

class PID(object):

	def __init__(kp,ki,kd,sampleTime,out_min,out_max,time=time):
		
		# PID Constants
		self.kp 				= kp
		self.kd 				= kd
		self.ki 				= ki

		# Different Errors
		self.error_propotional  = 0.0
		self.error_integral 	= 0.0
		self.error_differential = 0.0

		# Sampling time
		self.sampleTime			= sampleTime

		# Remembering past results
		self.last_timeStamp 	= 0.0
		self.last_input			= 0.0
		self.last_output 		= 0.0

		# Boundary values for output
		self.out_min			= out_min
		self.out_max 			= out_max
		self._time				= time

		# Values to be used in calculation
		self._kp				= self.kp
		self._kd				= self.kd/self.sampleTime
		self._ki				= self.ki*self.sampleTime
		#self.integral 			


	def calculate_PID(self,currentValue,targetValue):

		now = time()

		if now - self.last_timeStamp < self.sampleTime:
			return self.last_output

		error = targetValue - currentValue
		input_difference = currentValue - self.last_input

		# In order to prevent windup, integrate only if process is not saturated
		if self.last_output<self.out_max and self.last_output>self.out_min:
			self.error_integral += error * self._ki
			self.error_integral = min(self.error_integral,self.out_max)
			self.error_integral = max(self.error_integral,self.out_min)

		p = error * self._kp
		i = self.error_integral
		d = -(self._kd * input_difference)

		self.last_output = p + i + d
		self.last_timeStamp = now
		self.last_input = currentValue
		return self.last_output


