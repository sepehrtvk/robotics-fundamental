#!/usr/bin/python3

import rospy
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import atan2
from cmath import sqrt

class PIDController():
    def __init__(self):
        
        rospy.init_node('goal_fallow', anonymous=False)
        
        self.goal_x = 3
        self.goal_y = -1

        self.currentX = 0
        self.currentY = 0

        self.ki_angle = 0.08
        self.kp_angle = 0.4
        self.kd_angle = 8
        
        self.kd_distance = 0.005
        self.ki_distance = 0.02
        self.kp_distance = 0.9

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


    def distance_from_wall(self):
        laser_data = rospy.wait_for_message("/scan" , LaserScan)
        rng = laser_data.ranges[10:190]
        return min(rng)
    
    def front_distance_from_wall(self):
        laser_data = rospy.wait_for_message("/scan" , LaserScan)
        rng1 = laser_data.ranges[0:10]
        rng2 = laser_data.ranges[350:359]
        return  min(min(rng1),min(rng2))

    def distance_from_goal(self):
        msg = rospy.wait_for_message("/odom" , Odometry) 
        self.currentX = msg.pose.pose.position.x
        self.currentY = msg.pose.pose.position.y
        deltaX = self.currentX - self.goal_x
        deltaY = self.currentY - self.goal_y
        return sqrt(deltaX**2 + deltaY**2)

    def angle_to_goal(self):
        inc_x = self.goal_x - self.currentX
        inc_y = self.goal_y - self.currentY
        return atan2(inc_y, inc_x)

    def get_heading(self):
        msg = rospy.wait_for_message("/odom" , Odometry)
        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        return yaw   

    def move_to_goal(self):

            move_cmd = Twist()
            angle = self.angle_to_goal()
            delta = angle - self.get_heading()

            if abs(delta) > 0.2:
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 0.3
            else:
                move_cmd.linear.x = self.ki_distance
                move_cmd.angular.z = 0.0

            self.cmd_vel.publish(move_cmd)

    def follow_wall(self):
        
        d = self.distance_from_wall()    
        sum_i_theta = 0
        prev_theta_error = 0
            
        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = self.ki_distance

        front_d = self.front_distance_from_wall()
        if front_d <= (self.kp_distance + 0.1):
            front_d = self.front_distance_from_wall()
            twist = Twist()
            twist.angular.z = -0.2
            self.cmd_vel.publish(twist)
            
        else:
            self.cmd_vel.publish(move_cmd)
            err = d - self.kp_distance
            self.errs.append(err)
            sum_i_theta += err * self.kd_distance
            
            P = self.kp_angle * err
            I = self.ki_angle * sum_i_theta
            D = self.kd_angle * (err - prev_theta_error)

            move_cmd.angular.z = P + I + D 
            prev_theta_error = err
            move_cmd.linear.x = self.ki_distance                
             
    def run(self):
        goal_d = self.distance_from_goal()
        while not rospy.is_shutdown() :
            goal_d = self.distance_from_goal()
            if goal_d.real > 0.35:
                d = self.distance_from_wall()
                if d > self.kp_distance + 0.2:
                    self.move_to_goal()
                else:
                    self.follow_wall()
            else:
                move_cmd = Twist()
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 0.0
                break
            
if __name__ == '__main__':
    start = PIDController()
    start.run()
