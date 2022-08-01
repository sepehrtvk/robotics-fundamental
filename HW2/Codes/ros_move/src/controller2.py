#!/usr/bin/python3

import numpy as np
import rospy
import tf

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
        
        self.rectangle = self.makeRectangle()


        # defining the states of our robot
        self.GO, self.ROTATE = 0, 1
        self.state = self.GO 

        self.kp_distance = 0.2
        self.ki_distance = 0.02
        self.kd_distance = 2

        self.kp_angle = 1
        self.ki_angle = 0.03
        self.kd_angle = 0.05

        self.errors =[]
        
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

    def makeRectangle(self):
        rectangle = []
        
        X1 = np.linspace(0, 4 , 5)
        for x in X1:
            rectangle.append([x,0.0])

        X3 = np.linspace(4, 0 , 5)
        for x in X3:
            rectangle.append([x,6.0])


        Y2 = np.linspace(0, 4 , 5)
        for y in Y2:
            rectangle.append([0.0,y])

        Y4 = np.linspace(4, 0 , 5)
        for y in Y4:
            rectangle.append([y,6.0])
        return rectangle

    def run(self):

        while not rospy.is_shutdown():
            last_rotation = 0
            for goal in self.rectangle:
                self.cmd_publisher.publish(Twist())
                
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

                    distance = dist([goal[0],goal[1]],[current_pose.x,current_pose.y])

                    control_signal_distance = self.kp_distance*distance + self.ki_distance*total_distance 

                    control_signal_angle = self.kp_angle*(path_angle - rotation)

                    self.twist.angular.z = (control_signal_angle)

                    self.twist.linear.x = min(control_signal_distance,0.1)

                    if self.twist.angular.z > 0:
                        self.twist.angular.z = min(self.twist.angular.z, 1.5)
                    else:
                        self.twist.angular.z = max(self.twist.angular.z, -1.5)
                    
                    last_rotation = rotation
                    self.cmd_publisher.publish(self.twist)

                    total_distance +=  distance

                
if __name__ == "__main__":
    controller = Controller()
    controller.run()
