
import numpy as np
import scipy.io as sio

f=open('Same_orientation_4.txt','r')
g=[]

for line in f:
		tmp=line[0:3]
		if tmp=="vx:":
			g.append(float(line[4:-1]))
		if tmp=="vy:":
			g.append(float(line[4:-1]))
		if tmp=="alt":
			g.append(float(line[6:-1]))
			
sio.savemat('Same_orientation_4.mat', {'vect':g})
