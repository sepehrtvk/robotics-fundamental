#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class PIDController():
    def __init__(self):
        
        rospy.init_node('maze_fallow', anonymous=False)
        
        self.ki_angle = 0.1
        self.kp_angle = 0.6
        self.kd_angle = 7
        
        self.kd_distance = 0.005
        self.ki_distance = 0.09
        self.kp_distance = 0.5
        
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def distance_from_wall(self):
        laser_data = rospy.wait_for_message("/scan" , LaserScan)
        rng = laser_data.ranges[5:180]
        return min(rng)

    def front_distance_from_wall(self):
        laser_data = rospy.wait_for_message("/scan" , LaserScan)
        rng = laser_data.ranges[0:5]
        return min(rng)

    def follow_wall(self):
        
        d = self.distance_from_wall()    
        sum_i_theta = 0
        prev_theta_error = 0
        
        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = self.ki_distance


        while not rospy.is_shutdown():
            front_d = self.front_distance_from_wall()
            if front_d <= self.kp_distance :
                front_d = self.front_distance_from_wall()
                twist = Twist()
                twist.angular.z = -0.2
                self.cmd_vel.publish(twist)
            
            else:
                self.cmd_vel.publish(move_cmd)

                err = d - self.kp_distance
                sum_i_theta += err * self.kd_distance
                
                P = self.kp_angle * err
                I = self.ki_angle * sum_i_theta
                D = self.kd_angle * (err - prev_theta_error)

                prev_theta_error = err
                move_cmd.angular.z = P + I + D 
                move_cmd.linear.x = self.ki_distance            
                                
                d = self.distance_from_wall()
            
if __name__ == '__main__':
    start = PIDController()
    start.follow_wall()
