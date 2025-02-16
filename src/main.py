import numpy as np
from math import sqrt, atan2

#Step 1: Draw kinematic diagram of first three joints and do inverse kinematiics for position
#Step 2: Forware kinematics on the first three joints to tget the rotation part of r0_3
#Step 3: Find the inverse of the r0_3 matrix
#Step 4: Do the forwared kinematics on the last three joints and pull out the rotaiton part r3_6
#Step 5: Specify what you want the rotation matrix r0_6 to be
#Step 6: Given a desired X,Y,Z position solve for the first three joints using the inverse kinematic equations from step 1
#Step 7: Plug the joint values from step 6 into the rotation matrix to solve for the last three joint angles.

#Link lengths
a3 = 0.0
a4 = 0.0
a5 = 0.0

#Desired position from user
x = float(input("Desired X position: "))
y = float(input("Desired y position: "))
z = float(input("Desired z position: "))

#Inverse Kinematics
d3 = (sqrt((x^2)+(y^2))) - a3 - a4 - a5

theta_2 = np.arctan2(y/x)

#Forward Kinematics R0_3
r0_1 = np.eye(3)

r1_2 = np.dot(np.array([[np.cos(theta_2), -np.sin(theta_2), 0],
                [np.sin(theta_2), np.cos(theta_2), 0]
                [0,0,1]]),r0_1) 

r2_3 = np.eye(3)

r0_3 = np.dot(r0_1,r1_2,r2_3)

i_r0_3 = np.linalg.inv(r0_3)

