#!/usr/bin/python3

from cmath import sqrt
from dis import dis
import rospy, tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import radians,dist
import matplotlib.pyplot as plt
import numpy as np

class Controller:
    def __init__(self) -> None:
        rospy.init_node("controller", anonymous=False)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)

        # getting specified parameters
        self.linear_speed = rospy.get_param("/controller/linear_speed") # m/s
        self.angular_speed = rospy.get_param("/controller/angular_speed") # rad/s
        self.goal_angle = radians(rospy.get_param("/controller/goal_angle")) # rad
        self.epsilon = rospy.get_param("/controller/epsilon")

        # define rectangle
        self.length = 6
        self.width = 4

        self.spendTime = 0

        self.pose_x = []
        self.pose_y = []
        self.pose_x.append(0)
        self.pose_y.append(0)

        # defining the states of our robot
        self.GO, self.DIRECTION = 1, 1

        # do some cleanup on shutdown
        rospy.on_shutdown(self.clean_shutdown)

    def get_heading(self):
    # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry) 
        orientation = msg.pose.pose.orientation
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((orientation.x ,orientation.y ,orientation.z ,orientation.w))
        return yaw
    
    def clean_shutdown(self):
        rospy.loginfo("System is shutting down. Stopping robot...")
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_publisher.publish(twist)
    
    def rotate(self):
        rospy.loginfo("Robot is rotating to left ...")
        remaining = self.goal_angle
        prev_angle = self.get_heading()          
        twist = Twist()
        twist.angular.z = self.angular_speed
        self.cmd_publisher.publish(twist)            
        # rotation loop
        while remaining >= 0:
            current_angle = self.get_heading()
            delta = abs(prev_angle - current_angle)
            remaining -= delta
            prev_angle = current_angle
        rospy.loginfo("Robot rotating completed.")

    def moveForward(self):
        rospy.loginfo("Robot is moving forward...")
        traveled = 0
        steps = 0
        if self.DIRECTION == 1:
                steps = self.length
                self.DIRECTION = 0
        else:
                steps = self.width
                self.DIRECTION = 1
        msg = rospy.wait_for_message("/odom" , Odometry) 
        start_x = msg.pose.pose.position.x
        start_y = msg.pose.pose.position.y
        while steps > traveled:
            twist = Twist()
            twist.linear.x = self.linear_speed
            msg = rospy.wait_for_message("/odom" , Odometry) 
            traveled = abs(msg.pose.pose.position.y - start_y) + abs(msg.pose.pose.position.x - start_x) 

            self.pose_x.append(msg.pose.pose.position.x)
            self.pose_y.append(msg.pose.pose.position.y)

            self.cmd_publisher.publish(twist)
        rospy.loginfo("Robot moving completed. Steps: %d ",steps)

    def run(self):
        while not rospy.is_shutdown():

            self.moveForward()
            self.cmd_publisher.publish(Twist())
            rospy.sleep(1)            

            # 1 complete rectangle round and then shutdown
            if self.spendTime == 3 :
                rospy.signal_shutdown("We are done here!")
                break
            self.spendTime += 1
            
            self.rotate() 
            self.cmd_publisher.publish(Twist())
            rospy.sleep(1)

        # show plots    
        self.ploot()

    def ploot(self):

        plt.xlabel("X-Axis")
        plt.ylabel("Y-Axis")
        plt.title("Robot Error")

        # draw rectangle
        num = len(self.pose_x)
        X1 = np.linspace(0, self.length , num)
        Y1 = np.array([self.width]*num)


        Y2 = np.linspace(self.width, 0 , num)
        X2 = np.array([self.length]*num)

        X3 = np.linspace(self.length, 0 , num)
        Y3 = np.array([0]*num)

        Y4 = np.linspace(0, self.width , num)
        X4 = np.array([0]*num)

        plt.figure(1)
        plt.plot(np.concatenate([X1,X2, X3 , X4]), np.concatenate([Y1,Y2,Y3,Y4]), color='r', label='Default Path')
        plt.plot(self.pose_x,self.pose_y, color='b', label='Robot Path')

        zip_object1 = np.concatenate([X1,X2, X3 , X4])

        zip_object2 = np.concatenate([Y1,Y2,Y3,Y4])

        points = []
        for X2, Y2 in zip(self.pose_x , self.pose_y):
            minn = 1000
            for X1, Y1 in zip(zip_object1,zip_object2):
                distance = dist([X2,Y2],[X1,Y1])
                if(distance < minn ):
                    minn = distance
            points.append(minn)
        plt.figure(2)
        plt.plot(points, color='g', label='diff')

        plt.legend()
        plt.savefig("plots.png")
        plt.show()  

if __name__ == "__main__" :
    controller = Controller()
    controller.run()
