
import numpy as np
import scipy.io as sio

f=open('UGV_straight_line_2_cmd.txt','r')
g=[]

for line in f:
		#tmp=line[0:3]
		#if tmp=="lin":	
			#if f.next()=="  x":
		if line.startswith("lin"):
			tmp = f.next()
			#print tmp[4:-1]
			for i in range(1,40):
				g.append(float(tmp[4:-1]))
			
sio.savemat('UGV_straight_line_2_cmd.mat', {'ugv_vel':g[400:-1]})
