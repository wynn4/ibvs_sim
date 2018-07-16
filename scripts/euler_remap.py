#! /usr/bin/env python

import numpy as np
import time
from scipy.optimize import fsolve

class EulerRemap(object):

    def __init__(self):

    	# Original Euler Angles
    	self.phi = np.radians(15.0)
    	self.theta = np.radians(-10.0)
    	self.psi = np.radians(45.0)

        # Desired Heading angle
    	# self.psi_des = np.radians(-45.0)

    	# Pre-evaluate sines and cosines
    	sphi = np.sin(self.phi)
    	cphi = np.cos(self.phi)

    	stheta = np.sin(self.theta)
    	ctheta = np.cos(self.theta)

    	spsi = np.sin(self.psi)
    	cpsi = np.cos(self.psi)

    	# The z-axis unit vector resulting from the euler angles. 
    	self.z_unit = np.array([[cphi*stheta*cpsi + sphi*spsi],
    		                    [cphi*stheta*spsi - sphi*cpsi],
    		                    [cphi*ctheta]])

    def find_remaped_euler(self, euler_angles, psi_des):

    	# print euler_angles
    	self.psi_des = psi_des

    	# Euler angles passed in
    	phi = euler_angles[0]
    	theta = euler_angles[1]
    	psi = psi_des

    	# Pre-evaluate sines and cosines
    	sphi = np.sin(phi)
    	cphi = np.cos(phi)

    	stheta = np.sin(theta)
    	ctheta = np.cos(theta)

    	spsi = np.sin(psi)
    	cpsi = np.cos(psi)

    	vec = np.array([[cphi*stheta*cpsi + sphi*spsi],
    		            [cphi*stheta*spsi - sphi*cpsi],
    		            [cphi*ctheta]])

    	F = vec - self.z_unit

    	F = F.flatten()

    	return F


def main():

    # create instance of LevelFrameMapper class
    mapper = EulerRemap()

    then = time.time()
    x = fsolve(mapper.find_remaped_euler, np.zeros(3), (np.radians(-20.0)))
    now = time.time()

    print "dt: %f" % (now-then)

    phi = np.degrees(x[0])
    theta = np.degrees(x[1])



    print "Original Euler Angles:"
    print "Phi: %f" % np.degrees(mapper.phi)
    print "Theta: %f" % np.degrees(mapper.theta)
    print "Psi: %f" % np.degrees(mapper.psi)
    print "\n"
    print "New Euler Angles:"
    print "Phi: %f" % phi
    print "Theta: %f" % theta
    print "Psi: %f" % np.degrees(mapper.psi_des)

    

    # suff here

if __name__ == '__main__':
    main()
