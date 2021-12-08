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


     rospy.init_node('talker', anonymous=True)
     rate = rospy.Rate(1/delT) # 10hz
     
     while not rospy.is_shutdown():

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

         rate.sleep()
         print(i)
         i=i+1
 
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
