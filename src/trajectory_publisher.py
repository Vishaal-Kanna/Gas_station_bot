#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64
import math
 
def talker():
     pub_theta1 = rospy.Publisher('/robot_arm/theta1_controller/command', Float64, queue_size=10) 
     pub_theta2 = rospy.Publisher('/robot_arm/theta2_controller/command', Float64, queue_size=10)
     pub_theta3 = rospy.Publisher('/robot_arm/theta3_controller/command', Float64, queue_size=10) 
     pub_theta4 = rospy.Publisher('/robot_arm/theta4_controller/command', Float64, queue_size=10)
     pub_theta5 = rospy.Publisher('/robot_arm/theta5_controller/command', Float64, queue_size=10) 
     pub_theta6 = rospy.Publisher('/robot_arm/theta6_controller/command', Float64, queue_size=10)
     pub_grasp_angle1 = rospy.Publisher('/robot_arm/grasp_angle1_controller/command', Float64, queue_size=10) 
     pub_grasp_angle2 = rospy.Publisher('/robot_arm/grasp_angle2_controller/command', Float64, queue_size=10)
     
     T = 500              #Time taken to draw the circle 
     delT = 0.1        
     n= int(T/delT)

     a = 0*3.14/180    #Initial joint angles
     b = 0*3.14/180
     c = 0*3.14/180
     d = -90*3.14/180
     e = 0
     f = 0*3.14/180
     g = 0

     q = [a,b,c,d,f,g]
     
     d1 = 70
     d3 = 600	
     d5 = 570
     d7 = 105.5
     i=0

     A = np.matrix([0,0,0,1])

     rospy.init_node('talker', anonymous=True)
     rate = rospy.Rate(1/delT) # 10hz
     
     while not rospy.is_shutdown():
         #Using the updated joint angle values to get the position of end effector at each time instant from the forward kinematics equations
         T01 = np.matrix([[math.cos(q[0]), 0, math.sin(q[0]), 0], [math.sin(q[0]), 0, -math.cos(q[0]), 0], [0, 1, 0, d1], [0, 0, 0, 1]])
         T12 = np.matrix([[math.cos(q[1]), 0, -math.sin(q[1]), 0], [math.sin(q[1]), 0, math.cos(q[1]), 0], [0, -1, 0, 0], [0, 0, 0, 1]])
         T23 = np.matrix([[math.cos(q[2]), 0, math.sin(q[2]), 0], [math.sin(q[2]), 0, -math.cos(q[2]), 0], [0, 1, 0, d3], [0, 0, 0, 1]])
         T35 = np.matrix([[math.cos(q[3]), -math.sin(q[3]), 0, d5*math.cos(q[3])], [math.sin(q[3]), math.cos(q[3]),0, d5*math.sin(q[3])], [0,0,0,0], [0, 0, 0, 1]])
         T56 = np.matrix([[math.cos(q[4]), 0, -math.sin(q[4]), 0], [math.sin(q[4]), 0, math.cos(q[4]), 0], [0, -1, 0, 0], [0, 0, 0, 1]])
         T67 = np.matrix([[math.cos(q[5]), -math.sin(q[5]), 0, 0], [math.sin(q[5]), math.cos(q[5]),0, 0], [0, 0, 0, d7], [0, 0, 0, 1]])


         T02 = T01*T12
         T03 = T01*T12*T23
         T05 = T01*T12*T23*T35
         T06 = T01*T12*T23*T35*T56
         T07 = T01*T12*T23*T35*T56*T67

         O0 = np.matrix([[0], [0], [0]])
         O1 = T01[0:3,3]
         O2 = T02[0:3,3]
         O3 = T03[0:3,3]
         O5 = T05[0:3,3]
         O6 = T06[0:3,3]
         O7 = T07[0:3,3]

         Z0 = np.matrix([[0], [0], [1]])
         Z1 = T01[0:3,2]
         Z2 = T02[0:3,2]
         Z3 = T03[0:3,2]
         Z5 = T05[0:3,2]
         Z6 = T06[0:3,2]

         B1 = np.cross(Z0.T, (O7-O0).T)
         B2 = np.cross(Z1.T, (O7-O1).T)
         B3 = np.cross(Z2.T, (O7-O2).T)
         B4 = np.cross(Z3.T, (O7-O3).T)
         B5 = np.cross(Z5.T, (O7-O5).T)
         B6 = np.cross(Z6.T, (O7-O6).T)

         J1 = np.zeros(6)
         J2 = np.zeros(6)
         J3 = np.zeros(6)
         J4 = np.zeros(6)
         J5 = np.zeros(6)
         J6 = np.zeros(6)

         J1[0:3] = B1
         J1[3:6] = Z0.T
         J2[0:3] = B2
         J2[3:6] = Z1.T
         J3[0:3] = B3
         J3[3:6] = Z2.T
         J4[0:3] = B4
         J4[3:6] = Z3.T
         J5[0:3] = B5
         J5[3:6] = Z5.T
         J6[0:3] = B6
         J6[3:6] = Z6.T

         JT = np.zeros((6,6))
         JT[0:6][0] = J1.T
         JT[0:6][1] = J2.T
         JT[0:6][2] = J3.T
         JT[0:6][3] = J4.T
         JT[0:6][4] = J5.T
         JT[0:6][5] = J6.T

         J = JT.T     #Final jacobian matrix

         Jinv = np.linalg.pinv(J)
         if i<=50:
             V = np.matrix([[0], [0], [0], [0], [0], [0]])
             pub_theta1.publish(0)
             pub_theta2.publish(3.14)
             pub_theta3.publish(0)
             pub_theta4.publish(3.14)
             pub_theta5.publish(0)
             pub_theta6.publish(0)
             pub_grasp_angle1.publish(0)
             pub_grasp_angle2.publish(0)
         
         elif i<= 55:
             V = np.matrix([[50], [0], [50], [0], [0], [0]])
             pub_theta1.publish(0)
             pub_theta2.publish(3.14-0.1)
             pub_theta3.publish(0)
             pub_theta4.publish(3.14-0.1)
             pub_theta5.publish(0)
             pub_theta6.publish(0)
             pub_grasp_angle1.publish(0)
             pub_grasp_angle2.publish(0)

         elif i<= 60:
             V = np.matrix([[50], [0], [50], [0], [0], [0]])
             pub_theta1.publish(0)
             pub_theta2.publish(3.14-0.2)
             pub_theta3.publish(0)
             pub_theta4.publish(3.14-0.2)
             pub_theta5.publish(0)
             pub_theta6.publish(0)
             pub_grasp_angle1.publish(0)
             pub_grasp_angle2.publish(0)

         elif i<= 65:
             V = np.matrix([[50], [0], [50], [0], [0], [0]])
             pub_theta1.publish(0)
             pub_theta2.publish(3.14-0.3)
             pub_theta3.publish(0)
             pub_theta4.publish(3.14-0.3)
             pub_theta5.publish(0)
             pub_theta6.publish(0)
             pub_grasp_angle1.publish(0)
             pub_grasp_angle2.publish(0)

         elif i<= 70:
             V = np.matrix([[50], [0], [50], [0], [0], [0]])
             pub_theta1.publish(0)
             pub_theta2.publish(3.14-0.4)
             pub_theta3.publish(0)
             pub_theta4.publish(3.14-0.5)
             pub_theta5.publish(0)
             pub_theta6.publish(0)
             pub_grasp_angle1.publish(0)
             pub_grasp_angle2.publish(0)

         elif i<= 75:
             V = np.matrix([[50], [0], [50], [0], [0], [0]])
             pub_theta1.publish(0)
             pub_theta2.publish(3.14-0.4)
             pub_theta3.publish(0)
             pub_theta4.publish(3.14-0.7)
             pub_theta5.publish(0)
             pub_theta6.publish(0)
             pub_grasp_angle1.publish(0)
             pub_grasp_angle2.publish(0)

         elif i<= 80:
             V = np.matrix([[50], [0], [50], [0], [0], [0]])
             pub_theta1.publish(0)
             pub_theta2.publish(3.14-0.4)
             pub_theta3.publish(0)
             pub_theta4.publish(3.14-0.9)
             pub_theta5.publish(0)
             pub_theta6.publish(0)
             pub_grasp_angle1.publish(0)
             pub_grasp_angle2.publish(0)

         elif i<= 85:
             V = np.matrix([[50], [0], [50], [0], [0], [0]])
             pub_theta1.publish(0.2)
             pub_theta2.publish(3.14-0.4)
             pub_theta3.publish(0)
             pub_theta4.publish(3.14-1.05)
             pub_theta5.publish(0)
             pub_theta6.publish(0)
             pub_grasp_angle1.publish(0)
             pub_grasp_angle2.publish(0)

         elif i<= 90:
             V = np.matrix([[50], [0], [50], [0], [0], [0]])
             pub_theta1.publish(0.27)
             pub_theta2.publish(3.14-0.4)
             pub_theta3.publish(0)
             pub_theta4.publish(3.14-1.05)
             pub_theta5.publish(0)
             pub_theta6.publish(0)
             pub_grasp_angle1.publish(0.2)
             pub_grasp_angle2.publish(0.2)


         elif i<= 120:
             V = np.matrix([[50], [0], [50], [0], [0], [0]])
             pub_theta1.publish(0.27)
             pub_theta2.publish(3.14-0.4)
             pub_theta3.publish(0)
             pub_theta4.publish(3.14-1.05)
             pub_theta5.publish(0)
             pub_theta6.publish(0.1)
             pub_grasp_angle1.publish(0.2)
             pub_grasp_angle2.publish(0.2)

         elif i<= 122:
             V = np.matrix([[50], [0], [50], [0], [0], [0]])
             pub_theta1.publish(0.35)
             pub_theta2.publish(3.14-0.4)
             pub_theta3.publish(0)
             pub_theta4.publish(3.14-1.05)
             pub_theta5.publish(0)
             pub_theta6.publish(0)
             pub_grasp_angle1.publish(0.2)
             pub_grasp_angle2.publish(0.2)

         
         elif i<= 125:
             V = np.matrix([[50], [0], [50], [0], [0], [0]])
             pub_theta1.publish(0.4)
             pub_theta2.publish(3.14-0.4)
             pub_theta3.publish(0)
             pub_theta4.publish(3.14-1.1)
             pub_theta5.publish(0)
             pub_theta6.publish(0)
             pub_grasp_angle1.publish(0.2)
             pub_grasp_angle2.publish(0.2)

         elif i<= 128:
             V = np.matrix([[50], [0], [50], [0], [0], [0]])
             pub_theta1.publish(0.5)
             pub_theta2.publish(3.14-0.35)
             pub_theta3.publish(0)
             pub_theta4.publish(3.14-1.1)
             pub_theta5.publish(0)
             pub_theta6.publish(0)
             pub_grasp_angle1.publish(0.2)
             pub_grasp_angle2.publish(0.2)

         elif i<= 130:
             V = np.matrix([[50], [0], [50], [0], [0], [0]])
             pub_theta1.publish(0.6)
             pub_theta2.publish(3.14-0.3)
             pub_theta3.publish(0)
             pub_theta4.publish(3.14-1.2)
             pub_theta5.publish(0)
             pub_theta6.publish(0)
             pub_grasp_angle1.publish(0.2)
             pub_grasp_angle2.publish(0.2)

         elif i<= 133:
             V = np.matrix([[50], [0], [50], [0], [0], [0]])
             pub_theta1.publish(0.65)
             pub_theta2.publish(3.14-0.25)
             pub_theta3.publish(0)
             pub_theta4.publish(3.14-1.2)
             pub_theta5.publish(0)
             pub_theta6.publish(0)
             pub_grasp_angle1.publish(0.2)
             pub_grasp_angle2.publish(0.2)

         elif i<= 135:
             V = np.matrix([[50], [0], [50], [0], [0], [0]])
             pub_theta1.publish(0.7)
             pub_theta2.publish(3.14-0.2)
             pub_theta3.publish(0)
             pub_theta4.publish(3.14-1.4)
             pub_theta5.publish(0)
             pub_theta6.publish(0)
             pub_grasp_angle1.publish(0.2)
             pub_grasp_angle2.publish(0.2)

         elif i<= 138:
             V = np.matrix([[50], [0], [50], [0], [0], [0]])
             pub_theta1.publish(0.7)
             pub_theta2.publish(3.14-0.25)
             pub_theta3.publish(0)
             pub_theta4.publish(3.14-1.2)
             pub_theta5.publish(0)
             pub_theta6.publish(0)
             pub_grasp_angle1.publish(0.2)
             pub_grasp_angle2.publish(0.2)

         elif i<= 140:
             V = np.matrix([[50], [0], [50], [0], [0], [0]])
             pub_theta1.publish(0.7)
             pub_theta2.publish(3.14-0.2)
             pub_theta3.publish(0)
             pub_theta4.publish(3.14-1.4)
             pub_theta5.publish(0)
             pub_theta6.publish(0)
             pub_grasp_angle1.publish(0)
             pub_grasp_angle2.publish(0)

         elif i<= 143:
             V = np.matrix([[50], [0], [50], [0], [0], [0]])
             pub_theta1.publish(0.7)
             pub_theta2.publish(3.14-0.2)
             pub_theta3.publish(0)
             pub_theta4.publish(3.14-1.4)
             pub_theta5.publish(0)
             pub_theta6.publish(0)
             pub_grasp_angle1.publish(0)
             pub_grasp_angle2.publish(0)

         elif i<= 145:
             V = np.matrix([[50], [0], [50], [0], [0], [0]])
             pub_theta1.publish(0.7)
             pub_theta2.publish(3.14-0.2)
             pub_theta3.publish(0)
             pub_theta4.publish(3.14-1.4)
             pub_theta5.publish(0)
             pub_theta6.publish(0)
             pub_grasp_angle1.publish(0)
             pub_grasp_angle2.publish(0)

         elif i<= 145:
             V = np.matrix([[50], [0], [50], [0], [0], [0]])
             pub_theta1.publish(0.65)
             pub_theta2.publish(3.14-0.2)
             pub_theta3.publish(0)
             pub_theta4.publish(3.14-1.4)
             pub_theta5.publish(0)
             pub_theta6.publish(0)
             pub_grasp_angle1.publish(0)
             pub_grasp_angle2.publish(0)

         elif i<= 150:
             V = np.matrix([[50], [0], [50], [0], [0], [0]])
             pub_theta1.publish(0.6)
             pub_theta2.publish(3.14-0.3)
             pub_theta3.publish(0)
             pub_theta4.publish(3.14-1.2)
             pub_theta5.publish(0)
             pub_theta6.publish(0)
             pub_grasp_angle1.publish(0)
             pub_grasp_angle2.publish(0)

         elif i<= 150:
             V = np.matrix([[50], [0], [50], [0], [0], [0]])
             pub_theta1.publish(0.5)
             pub_theta2.publish(3.14-0.3)
             pub_theta3.publish(0)
             pub_theta4.publish(3.14-1.2)
             pub_theta5.publish(0)
             pub_theta6.publish(0)
             pub_grasp_angle1.publish(0)
             pub_grasp_angle2.publish(0)

         elif i<= 155:
             V = np.matrix([[50], [0], [50], [0], [0], [0]])
             pub_theta1.publish(0.4)
             pub_theta2.publish(3.14-0.4)
             pub_theta3.publish(0)
             pub_theta4.publish(3.14-1.1)
             pub_theta5.publish(0)
             pub_theta6.publish(0)
             pub_grasp_angle1.publish(0)
             pub_grasp_angle2.publish(0)

         elif i<= 158:
             V = np.matrix([[50], [0], [50], [0], [0], [0]])
             pub_theta1.publish(0.35)
             pub_theta2.publish(3.14-0.4)
             pub_theta3.publish(0)
             pub_theta4.publish(3.14-1.1)
             pub_theta5.publish(0)
             pub_theta6.publish(0)
             pub_grasp_angle1.publish(0)
             pub_grasp_angle2.publish(0)

         elif i<= 160:
             V = np.matrix([[50], [0], [50], [0], [0], [0]])
             pub_theta1.publish(0.27)
             pub_theta2.publish(3.14-0.4)
             pub_theta3.publish(0)
             pub_theta4.publish(3.14-1.05)
             pub_theta5.publish(0)
             pub_theta6.publish(0)
             pub_grasp_angle1.publish(0)
             pub_grasp_angle2.publish(0)



         q_dot = np.dot(Jinv,V)

         print('theta1 : ', q[0])
         print('theta2 : ', q[1])
         print('theta3 : ', q[2])
         print('theta4 : ', q[3])
         print('theta5 : ', q[4])
         print('theta6 : ', q[5])
         
         
         #Updating q values using joint angle velocity obtained in the previous step
         q[0] += (delT*q_dot[0])
         q[1] += (delT*q_dot[1])
         q[2] += (delT*q_dot[2])
         q[3] += (delT*q_dot[3])
         q[4] += (delT*q_dot[4])
         q[5] += (delT*q_dot[5])


         rate.sleep()
         print(i)
         i=i+1
 
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
