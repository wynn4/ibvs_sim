#!/usr/bin/env python
# Python implementation of rosflight_utils SimplePID class

from math import *
import numpy as np

class PID:

	# Init function
	def __init__(self, p=0.0, i=0.0, d=0.0, max_=None, min_=None, tau=0.05):
		self.kp_ = p
		self.ki_ = i
		self.kd_ = d
		self.tau_ = tau
		self.max_ = max_
		self.min_ = min_
		self.integrator_ = 0.0
		self.differentiator_ = 0.0
		self.last_error_ = 0.0
		self.last_state_ = 0.0

	def computePID(self, desired, current, dt, x_dot=None):
		error = desired - current

		# Don't do stupid things (like divide by nearly zero, gigantic control jumps)
		if (dt < 0.00001 or abs(error) > 9999999):
			return 0.0
		if (dt > 1.0):
			# This means that this is a ''stale'' controller and needs to be reset.
			# This would happen if we have been operating in a different mode for a while
			# and will result in some enormous integrator.
			# Or, it means we are disarmed and shouldn't integrate
			# Setting dt for this loop will mean that the integrator and dirty derivative
			# doesn't do anything this time but will keep it from exploding.
			dt = 0.0
			self.differentiator_ = 0.0

		p_term = error*self.kp_
		i_term = 0.0
		d_term = 0.0

		# Caluclate Derivative Term
		if (abs(self.kd_) > 0.0):
			if x_dot != None:
				d_term = self.kd_ * x_dot
			elif (dt > 0.0):
				# Noise reduction (See "Small Unmanned Aircraft". Chapter 6. Slide 31/33)
				# d/dx w.r.t. error:: differentiator_ = (2*tau_ - dt)/(2*tau_ + dt)*differentiator_ + 2/(2*tau_ + dt)*(error -
				# last_error_);
				self.differentiator_ = (2.0 * self.tau_ - dt) / (2.0 * self.tau_ + dt) * self.differentiator_ + 2.0 / (2.0 * self.tau_ + dt) * (current - self.last_state_)
				d_term = self.kd_* self.differentiator_

		# Calc Integrator Term
		if (self.ki_ > 0.0):
			self.integrator_ += dt / 2.0 * (error + self.last_error_)
			i_term = self.ki_ * self.integrator_

		# Save off this state for next loop
		self.last_error_ = error
		self.last_state_ = current

		u_unsat = p_term + i_term - d_term
		u = self.saturate(u_unsat, self.max_, self.min_)
		# anti windup
		if (self.ki_ != 0.0):
			self.integrator_ = self.integrator_ + 1/self.ki_*(u - u_unsat)
		return u

		# TODO anti windup

	def setGains(self,p,i,d,tau):
		self.kp_ = p
		self.ki_ = i
		self.kd_ = d
		self.tau_ = tau
		# print "New Gains:\nP:", p, "\nI:", i, "\nD:", d

	def saturate(self, value, up_limit, low_limit):
		if(value > up_limit):
			rVal = up_limit
		elif(value < low_limit):
			rVal = low_limit
		else:
			rVal = value
		if (up_limit == None) or (low_limit == None):
			rVal = value

		return rVal

	def clearIntegrator(self):

		self.integrator_ = 0.0