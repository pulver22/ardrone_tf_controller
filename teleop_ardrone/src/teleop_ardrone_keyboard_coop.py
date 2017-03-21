#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_ardrone')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
up/down:       move forward/backward
left/right:    move left/right
w/s:           increase/decrease altitude
a/d:           turn left/right
t/l:           takeoff/land
r:             reset (toggle emergency state)
anything else: stop
y/n : increase/decrease max speeds by 10%
u/m : increase/decrease only linear speed by 10%
i/, : increase/decrease only angular speed by 10%
anything else : stop
CTRL-C to quit
"""
#each button is associated with a 6-dim tuple: (x,th_x,y,th_y,z,th_z)

#(linear,angular) velocity on the three axis
moveBindingsAxis = {
		67:(0,0,-1,0,0,0,0), #left
		68:(0,0,1,0,0,0,0), #right
		65:(1,0,0,0,0,0,0), #forward
		66:(-1,0,0,0,0,0), #back
		'w':(0,0,0,0,1,0), #increase altitude
		's':(0,0,0,0,-1,0), #decrease altitude
		'a':(0,0,0,0,0,1), #rotate left
		'd':(0,0,0,0,0,-1), #rotate right
	       }

#increase/decrease velocity on X axis
speedBindingsAxis={
		'y':(1.1,1.1), #increase linear and angular velocity
		'n':(.9,.9), #decrease linear and angular velocity
		'u':(1.1,1), #increase only linear vel
		'm':(.9,1), #decrease only linear vel
		'i':(1,1.1), #increase only rot vel
		',':(1,.9), #decrease only rot vel
	      }

specialManoeuvre={
		't':(0,0),
		'l':(0,0),
		'r':(0,0),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

speed = .5
turn = 1

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)

	pub_twist = rospy.Publisher('/uav1/cmd_vel', Twist, queue_size=1)
	pub_empty_takeoff = rospy.Publisher('/uav1/takeoff', Empty, queue_size=1)
	pub_empty_landing = rospy.Publisher('/uav1/land', Empty,queue_size=1)
	rospy.init_node('teleop_ardrone_keyboard')



	try:
		print msg
		print vels(speed,turn)
		while(1):
			x = y = z = 0
			th_x = th_y = th_z = 0
			status = 0
			key = getKey()
			if ((ord(key) in moveBindingsAxis.keys()) or (key in moveBindingsAxis.keys())):
				# x is linear speed, th is the angular one

				if (ord(key) in moveBindingsAxis.keys()):
					key = ord(key)

				x = moveBindingsAxis[key][0]
				th_x = moveBindingsAxis[key][1]
				y = moveBindingsAxis[key][2]
				th_y = moveBindingsAxis[key][3]
				z = moveBindingsAxis[key][4]
				th_z = moveBindingsAxis[key][5]


			elif key in speedBindingsAxis.keys():
				# increase or decrease linear or angular speed
				speed = speed * speedBindingsAxis[key][0]
				turn = turn * speedBindingsAxis[key][1]

				print vels(speed,turn)
				if (status == 14):
					print msg
				status = (status + 1) % 15

			elif key in specialManoeuvre.keys():

				if (key == 't'):
					# publish mex to take off
					pub_empty_takeoff.publish(Empty());
					continue;
				elif (key == 'l'):
					#publish mex to landing
					pub_empty_landing.publish(Empty());
					continue;
				else:
					# publish mex to reset velocity and hoovering
					twist = Twist()
					twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
					twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
					pub_twist.publish(twist)
					continue;
			else:
				x = 0
				th = 0
				if (key == '\x03'):
					break


			twist = Twist()
			twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed
			twist.angular.x = th_x*turn; twist.angular.y = th_y*turn; twist.angular.z = th_z*turn
			pub_twist.publish(twist)

			#print key

	except:
		print e

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub_twist.publish(twist)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
