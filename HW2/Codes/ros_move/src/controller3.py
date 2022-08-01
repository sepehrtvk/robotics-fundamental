#!/usr/bin/python3

import numpy as np
import rospy
import tf
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from math import atan2, pi, radians,dist

class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=False)
        
        sub = rospy.Subscriber("/odometry/filtered", Odometry, self.get_heading)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)

        # getting specified parameters
        self.linear_speed = rospy.get_param("/controller/linear_speed") # m/s
        self.angular_speed = rospy.get_param("/controller/angular_speed") # rad/s
        self.goal_angle = radians(rospy.get_param("/controller/goal_angle")) # rad
        self.epsilon = rospy.get_param("/controller/epsilon")
        self.twist = Twist()
        
        # defining the states of our robot
        self.GO, self.ROTATE = 0, 1
        self.state = self.GO 

        self.kp_distance = 10
        self.ki_distance = 20
        self.kd_distance = 2

        self.kp_angle = 1
        self.ki_angle = 0.03
        self.kd_angle = 0.05

        self.shape = []
        
    def getCurrentPosition(self):
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        return  msg.pose.pose.position
    
    def get_heading(self):
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        orientation = msg.pose.pose.orientation
        
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        return yaw

    def rotate(self):
        self.cmd_publisher.publish(Twist())
        rospy.sleep(1)

        remaining = self.goal_angle
        prev_angle = self.get_heading()
        
        twist = Twist()
        twist.angular.z = self.angular_speed
        self.cmd_publisher.publish(twist)
        
        # rotation loop
        while remaining >= self.epsilon:
            current_angle = self.get_heading()
            delta = abs(prev_angle - current_angle)
            remaining -= delta
            prev_angle = current_angle
        
        self.cmd_publisher.publish(Twist())

    def makeArchimedean(self):
        archi = []
        growth_factor = 0.1
        for i in range(400):
            t = i / 20 * math.pi
            dx = (1 + growth_factor * t) * math.cos(t)
            dy = (1 + growth_factor * t) * math.sin(t)
            archi.append([dx,dy])
        self.shape.append(archi)

    def makeLogarithem(self):
        log = []
        a = 0.17
        k = math.tan(a)
        for i in range(150):
            t = i / 20 * math.pi
            dx = a * math.exp(k * t) * math.cos(t)
            dy = a * math.exp(k * t) * math.sin(t)
            log.append([dx,dy])
        self.shape.append(log)

    def makeEightees(self):
        eight = []
        X2 = np.linspace(1, 1 + 2**(1/2) , 10)
        Y2 = - (2**(1/2)) * (X2 - 1) + 3
        for i,x in enumerate(X2):
            eight.append([x,Y2[i]])

        Y3 = np.linspace(1, -1 , 10)
        X3 = np.array([1 + 2**(1/2)]*10)
        for i,x in enumerate(X3):
            eight.append([x,Y3[i]])

        X4 = np.linspace(1 + 2**(1/2), 1, 10)
        Y4 = (2**(1/2)) * (X4 - 1 - 2**(1/2)) -1 
        for i,x in enumerate(X4):
            eight.append([x,Y4[i]])

        X5 = np.linspace(1, -1 , 10)
        for i,x in enumerate(X5):
            eight.append([x,-3])

        X6 = np.linspace(-1, -1 - 2**(1/2) , 10)
        Y6 = - (2**(1/2)) * (X6 + 1) - 3 
        for i,x in enumerate(X6):
            eight.append([x,Y6[i]])


        Y7 = np.linspace(-1, 1 , 10)
        X7 = np.array([- 1 - 2**(1/2)]*10)
        for i,x in enumerate(X7):
            eight.append([x,Y7[i]])


        X8 = np.linspace(-1 - 2**(1/2), -1, 10)
        Y8 = (2**(1/2)) * (X8 + 1 + 2**(1/2)) + 1
        for i,x in enumerate(X8):
            eight.append([x,Y8[i]])

        
        X1 = np.linspace(-1, 1 , 10)
        for x in X1:
            eight.append([x,3])

        self.shape.append(eight)

    def makeHalfCircles(self):
        circles = []
        X1 = np.linspace(-6., -2 , 20)
        for x in X1:
            circles.append([x, 0.0])

        x_dim, y_dim = 2,2
        t = np.linspace(np.pi, 0, 30)
        for t1 in t:
            circles.append([x_dim * np.cos(t1), y_dim * np.sin(t1)])

        X3 = np.linspace(2, 6 , 20)
        Y3 = np.zeros((50,))
        for x in X3:
            circles.append([x, 0.0])

        x_dim, y_dim = 6,6
        t = np.linspace(np.pi*2, np.pi, 40)
        for t2 in t:
            circles.append([x_dim * np.cos(t2), y_dim * np.sin(t2)])

        self.shape.append(circles)

    def run(self):
        self.makeArchimedean()
        self.makeLogarithem()
        self.makeHalfCircles()
        self.makeEightees()

        while not rospy.is_shutdown():
            last_rotation = 0
            for i,goal in enumerate(self.shape[2]):
                self.cmd_publisher.publish(Twist())                
                
                previous_distance = 0
                total_distance = 0
                
                current_pose = self.getCurrentPosition()
                distance = dist([goal[0],goal[1]],[current_pose.x,current_pose.y])

                while distance > 0.25:

                    rotation = self.get_heading()
                    current_pose = self.getCurrentPosition()
                    path_angle = atan2(goal[1]-current_pose.y , goal[0] - current_pose.x) 
                    if path_angle < -pi/4 or path_angle > pi/4:
                        if goal[1] < 0 and current_pose.y < goal[1]:
                            path_angle = -2*pi + path_angle
                        elif goal[1] >= 0 and current_pose.y > goal[1]:
                            path_angle = 2*pi + path_angle
                    if last_rotation > pi-0.1 and rotation <= 0:
                        rotation = 2*pi + rotation
                    elif last_rotation < -pi+0.1 and rotation > 0:
                        rotation = -2*pi + rotation

                    diff_distance = distance - previous_distance
                    distance = dist([goal[0],goal[1]],[current_pose.x,current_pose.y])

                    control_signal_distance = self.kp_distance*distance + self.ki_distance*total_distance + self.kd_distance*diff_distance

                    control_signal_angle = self.kp_angle*(path_angle - rotation)

                    self.twist.angular.z = (control_signal_angle)

                    self.twist.linear.x = min(control_signal_distance,0.1)

                    if self.twist.angular.z > 0:
                        self.twist.angular.z = min(self.twist.angular.z, 1.5)
                    else:
                        self.twist.angular.z = max(self.twist.angular.z, -1.5)
                    
                    last_rotation = rotation
                    self.cmd_publisher.publish(self.twist)

                    rospy.sleep(1)
                    previous_distance = distance
                    total_distance = total_distance + distance

                
if __name__ == "__main__":
    controller = Controller()
    controller.run()
