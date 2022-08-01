#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from distance_caculator.msg import Obst
import math

class ClosestObstacle:
    
    def __init__(self) -> None:
        rospy.init_node("closest_obstacle_node" , anonymous=True)
        self.odom_subscriber = rospy.Subscriber("/odom" , Odometry , callback = self.calculate_distance)
        self.obstacle_publisher = rospy.Publisher('/ClosestObstacle' , Obst , queue_size=10)
        
        # obstacle details
        self.obstacles = {
            "bookshelf" : (2.64, -1.55),
            "dumpster" :  (1.23, -4.57),
            "barrel" :    (-2.51, -3.08),
            "postbox" :   (-4.47, -0.57),
            "brick_box"	: (-3.44, 2.75),
            "cabinet" :	  (-0.45, 4.05),
            "cafe_table": (1.91, 3.37),
            "fountain" :  (4.08, 1.14)
        }
        
    def calculate_distance(self,odom):
        pose_x = odom.pose.pose.position.x
        pose_y = odom.pose.pose.position.y

        minDist = 10 ** 6

        for obstacle,position in self.obstacles.items():    
            distance = math.sqrt(((pose_x - position[0]) ** 2) + ((pose_y - position[1]) ** 2))

            if distance < minDist:
                closest = obstacle
                minDist = distance

        obs = Obst()
        obs.obstacle_name = closest
        obs.distance = minDist

        rospy.loginfo(f"Closest Obstacle Published: {closest} Distance : {minDist}")

        self.obstacle_publisher.publish(obs)

if __name__ == "__main__":
    startNode = ClosestObstacle()
    rospy.spin()