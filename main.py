 #!/usr/bin/python

# PRODUCTION VERSION

################################################################
# more_video_control_cb.py 
# Lab 2
#
# Authors: Alexander Bernard, Timothy Duggan, Christian Welling
# NetIDs: arb392, trd44, csw73
# Date Written: 11/17/17
# Last Revision: 11/30/17
#
################################################################
#
## Description: This is the main code, and the only thing that needs 
#               to be run to start all seven processes. The other files
#               need to be in the same folder

from __future__ import division

import RPi.GPIO as GPIO
import time
import subprocess
import time
import Adafruit_PCA9685
import os
import stat
import select
import math
import numpy
from camera_tracker_beta import Tracker
import psutil
	

class autoPilot():
	def __init__(self):
		
		p = psutil.Process(os.getpid())
		
		# define pins going to use
		self.pin_relays = 4
		self.pin_TFT1 = 17
		self.pin_TFT2 = 22
		self.pin_TFT3 = 23
		self.pin_TFT4 = 27
		
		# set up GPIO stuff
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.pin_TFT1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
		GPIO.setup(self.pin_TFT2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
		GPIO.setup(self.pin_TFT3, GPIO.IN, pull_up_down=GPIO.PUD_UP)
		GPIO.setup(self.pin_TFT4, GPIO.IN, pull_up_down=GPIO.PUD_UP)
		GPIO.setup(self.pin_relays, GPIO.OUT)

		
		# create subprocesses for reading from pin
		p.cpu_affinity([1])
		subprocess.call('gcc -o read_SWA read_SWA.c -lwiringPi', shell = True)
		self.swa_process = subprocess.Popen(['sudo', 'taskset', '-c', '1', './read_SWA'], stdout=subprocess.PIPE)
		self.SWA = 0
		
		subprocess.call('gcc -o read_SWB read_SWB.c -lwiringPi', shell = True)
		self.swb_process = subprocess.Popen(['sudo', 'taskset', '-c', '1', './read_SWB'], stdout=subprocess.PIPE)
		self.SWB = 0
		
		subprocess.call('gcc -o read_sonic read_sonic.c -lwiringPi', shell = True)
		self.sonic_process = subprocess.Popen(['sudo', 'taskset', '-c', '1', './read_sonic'], stdout=subprocess.PIPE)
		self.distance = 0
		

		p.cpu_affinity([2])
		# initialize the camera tracking
		self.ptb = Tracker()
		self.ptb.start()
		self.targetX = 0.
		self.targetY = 0.
		
		# set up the PWM hat to work with the library
		self.pwm_freq = 51.3
		self.pwm = Adafruit_PCA9685.PCA9685()
		self.pwm.set_pwm_freq(self.pwm_freq)
		
		
		
		# servo channel definitions
		self.strafe_command	    = 0 # channel 1 for receiver
		self.fly_command        = 1	# channel 2 for receiver
		self.throttle_command   = 2 # channel 3 for receiver
		self.turn_command       = 3 # channel 4 for receiver 

		# TIM'S CONTROLLER STUFF
		m = 2
		
		self.A = numpy.matrix([[0,1],[0,0]])
		self.B = numpy.matrix([[0],[1/m]])
		self.K = numpy.matrix([1,2.2361])
		
		self.DesiredHeight = 100
		
		
		# cpu stuff
		p.cpu_affinity([3])
	# -------------------- SEND PWM FUNCTIONS -------------------------------------------
	def set_servo_radians(self, channel, radians):
		# map between 1 and 2 ms
		# 0 radians = 1 ms
		# pi radians = 2 ms
		wide = 1000. / self.pwm_freq
		ms = radians / math.pi
		bits = int( (1 + ms) / wide * 4096.)
		self.pwm.set_pwm(channel, 0, bits)
	
		
	def fly(self, percent):
		percent = min(max(0,percent), 1)
		wide = 1000. / self.pwm_freq
		zero = 1.5
		ms = 0.5 * percent
		bits = int( (zero - ms) / wide * 4096.)
		self.pwm.set_pwm(self.fly_command, 0, bits)
		
	def throttle(self, percent):
		percent = min(max(0,percent), 1)
		wide = 1000. / self.pwm_freq
		zero = 1
		ms = 1 * percent
		bits = int( (zero + ms) / wide * 4096.)
		self.pwm.set_pwm(self.throttle_command, 0, bits)
	
	def strafe(self, percent):
		percent = min(max(0,percent), 1)
		wide = 1000. / self.pwm_freq
		zero = 1.5
		ms = 0.5 * percent
		bits = int( (zero - ms) / wide * 4096.)
		self.pwm.set_pwm(self.turn_command, 0, bits)
	
	# -------------------- READ AND UPDATE FROM EXTERNAL WORLD -------------------------------
	def poll_SWA(self):
		# read the state of the switch, change sides
		self.polling_booth_A = select.poll()
		self.polling_booth_A.register(self.swa_process.stdout, select.POLLIN)
		it_happened = self.polling_booth_A.poll(0)
		if it_happened:
			self.SWA = int(self.swa_process.stdout.readline())
		print("Switch A State = " + str(self.SWA))
		
	def poll_SWB(self):
		# read the state of the switch, change sides
		self.polling_booth_B = select.poll()
		self.polling_booth_B.register(self.swb_process.stdout, select.POLLIN)
		it_happened = self.polling_booth_B.poll(0)
		if it_happened:
			self.SWB = int(self.swb_process.stdout.readline())
		print("Switch B State = " + str(self.SWB))

		
	def poll_sonic(self):
		# read the state of the switch, change sides
		self.polling_booth_sonic = select.poll()
		self.polling_booth_sonic.register(self.sonic_process.stdout, select.POLLIN)
		it_happened = self.polling_booth_sonic.poll(0)
		if it_happened:
			self.distance = float(self.sonic_process.stdout.readline())
			self.distance = min(max(self.distance, 0.), 400.)
			self.distance = self.distance
		print("Current Altitude = " + str(self.distance))


	def poll_camera(self):
		# messed up in translation
		self.targetX = -self.ptb.readY()
		self.targetY = self.ptb.readX()
		print("XY Target Location= " + str(self.targetX) + " | " + str(self.targetY))
		

	# ------------------------------ MAIN LOOOOOOP --------------------------------------
	def main(self):
		t = 0.0
		dt = 0.1
		oldHeight = self.distance
		oldX = self.targetX
		oldY = self.targetY
		
		while True:
			os.system('clear')
			print "------------- NEW ITERATION ------------- "
			
			
			
			
			start = time.time()
			
			# poll and update from the switch, sonic sensor, camera
			print "" 
			print ""
			self.poll_SWA()
			self.poll_SWB()
			self.poll_sonic()
			self.poll_camera()			
			
			# Relay controller
			GPIO.output(self.pin_relays, not self.SWA)
			
			
			##LQR CONTROLLER
            #HEIGHT
			heightErr = self.DesiredHeight - self.distance
			heightVelErr = 0. #(self.distance - oldHeight)/dt
			HErrState = numpy.matrix([[heightErr],[heightVelErr]])
			tu = self.K*HErrState
			tu = tu/600.

            #X
			XErr = self.targetX
			XVelErr = 0. #(self.targetX - oldX)/dt
			XerrState = numpy.matrix([[XErr],[XVelErr]])
			fu = self.K*XerrState
			fu = fu/200.
			
			print "X Velocity Error= " + str(XVelErr)

            #Y
			YErr = self.targetY
			YVelErr = 0. #(self.targetY - oldY)/dt
			YerrState = numpy.matrix([[YErr],[YVelErr]])
			su = self.K*YerrState
			su = su/200.
			
			print "Y Velocity Error= " + str(YVelErr)
			
			# only track red when SWB is 1
			
			fu = self.SWB*fu
			su = self.SWB*su
			
			self.fly(fu)
			self.strafe(su)
			self.throttle(0.5 + tu)
			print ""
			print ""
			print "Fly Command = " + str(fu)
			print "Strafe Command = " + str(su)
			print "Throttle Command = " + str(0.5 + tu)
			
			oldHeight = self.distance
			oldX = self.targetX
			oldY = self.targetY
			
			
			et = time.time() - start
			
			print ""
			print ""
			print "Elapsed time is " + str(et)
			print "Going to sleep for " + str(max(dt-et,0))
			time.sleep(max(dt - et,0))
			
			
	def cleanup(self):
		print "Exiting Program"
		self.ptb.stop()
		GPIO.cleanup()
		print "Program Exitted Successfully"
		
if __name__ == "__main__":

	x = autoPilot()
	
	try:
		x.main()
	finally:
		x.cleanup()	
