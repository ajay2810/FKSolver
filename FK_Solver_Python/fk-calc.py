#!/usr/bin/python
# Author: Ajay kumar                                            
# KUKA K210 Forward Kinematics Transformation Matrix Calculator   
# TODO: Try to read the x and z values from the model definition  
# and then update the HTM table given below                        
# To run this file, go to command prompt/Python shell and execute 
# $> python fk_calc.py <model-file-name>                                            

import numpy as np
from numpy import array, radians
from sympy import symbols, cos, sin, pprint
from sympy.matrices import Matrix
import sys

filename = sys.argv[1]
x_val = []
z_val = []

def parse_file():
	with open(filename) as f:
		joint_info_offset = 3
		xy_offset = 3
		print ("Processing file: ", filename)
		lines =  list(f) 
		total_links = int(lines[0].rstrip("\n"))
		print (total_links)
		total_joints = int(lines[int(total_links)+joint_info_offset].rstrip("\n"))
		print (int(total_joints))

		for i in range(0,total_joints):
			coordinates = lines[total_links+joint_info_offset+xy_offset].rstrip("\n")
			coordinates = coordinates.rsplit(" ")
			global x_val
			x_val.append(float(coordinates[0]))
			global z_val
			z_val.append(float(coordinates[2]))
			xy_offset = 3 + xy_offset
		
		print (x_val, z_val)
	return

parse_file()
	
# Create symbols for Homogeneous Transform Matrix (HTM)
x_value0, x_value1, x_value2, x_value3, x_value4, x_value5 = symbols('x_value0:6')  # Link offset in X-axis
z_value0, z_value1, z_value2, z_value3, z_value4, z_value5 = symbols('z_value0:6')  # Link offset in Y-axis
a0, a1, a2, a3, a4, a5 = symbols('a0:6') 

a0 = 0
a1 = radians(-90)
a2 = 0
a3 = 0
a4 = 0
a5 = 0

HTM = {a0: a0, x_value0: x_value0, z_value0: z_value0, 
       a1: a1, x_value1: x_value1, z_value1: z_value1, 
       a2: a2, x_value2: x_value2, z_value2: z_value2, 
       a3: a3, x_value3: x_value3, z_value3: z_value3, 
       a4: a4, x_value4: x_value4, z_value4: z_value4, 
       a5: a5, x_value5: x_value5, z_value5: z_value5 }	  
	  
# Function to return homogeneous transform matrix

def TF_Mat_Y(angle, x_value, z_value):
    TF = Matrix([[cos(angle),0,sin(angle),-cos(angle)*(x_value)+(x_value)-(z_value)*sin(angle)],
	 [0,1,0,0],
	 [-sin(angle),0,cos(angle),-cos(angle)*(z_value)+(z_value)+(x_value)*sin(angle)],
	 [0,0,0,1]])
    return TF

def TF_Mat_Z(angle, x_value, z_value):
    TF = Matrix([[cos(angle),-sin(angle),0,-cos(angle)*(x_value)+(x_value)+(z_value)*sin(angle)],
				 [sin(angle),cos(angle),0,0],
				 [0,0,1,-cos(angle)*(z_value)+(z_value)-(x_value)*sin(angle)],
				 [0,0,0,1]])
    return TF

print("\nCalculating transforms for joint angles (0,-90,0,0,0,0)\n")	

## Substiute HTM_Table

T0_1 = TF_Mat_Z(a0, x_val[0], z_val[0]).subs(HTM)
print("\nLocal coordinate TF matrix for Link1 (z-rotation): TF0_1\n")
pprint(T0_1)
T1_2 = TF_Mat_Y(a1, x_val[1], z_val[1]).subs(HTM)
print("\nTF matrix for Link2 (y-rotation): TF1_2\n")
pprint(T1_2)
T2_3 = TF_Mat_Y(a2, x_val[2], z_val[2]).subs(HTM)
print("\nTF matrix for Link3 (y-rotation): TF2_3\n")
pprint(T2_3)
T3_4 = TF_Mat_Z(a3, x_val[3], z_val[3]).subs(HTM)
print("\nTF matrix for Link4 (z-rotation): TF3_4\n")
pprint(T3_4)
T4_5 = TF_Mat_Y(a4, x_val[4], z_val[4]).subs(HTM)
print("\nTF matrix for Link5 (y-rotation): TF4_5\n")
pprint(T4_5)
T5_6 = TF_Mat_Z(a5, x_val[5], z_val[5]).subs(HTM)
print("\nTF matrix for Gripper  (z-rotation): TF5_G\n")
pprint(T5_6)
T6_7 = Matrix(4,1,[1945.,0.,-855.,1.])

# Composition of Homogeneous Transforms
# Transform from Base link to end effector (Gripper)
T0_2 = (T0_1 * T1_2) ## (Base) Link_0 to Link_2
T0_3 = (T0_2 * T2_3) ## (Base) Link_0 to Link_3
T0_4 = (T0_3 * T3_4) ## (Base) Link_0 to Link_4
T0_5 = (T0_4 * T4_5) ## (Base) Link_0 to Link_5
T0_6 = (T0_5 * T5_6) ## (Base) Link_0 to Link_6

print ("\nLocal coordinate frame of gripper\n")
pprint (T0_6)

print("\nWorld coordinate frame of Gripper for joint angles (",a0*180/np.pi,a1*180/np.pi,a2*180/np.pi,a3*180/np.pi,a4*180/np.pi,a5*180/np.pi, "\n")
pprint (T0_6.dot(T6_7))
