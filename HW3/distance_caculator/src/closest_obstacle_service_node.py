#!/usr/bin/python3

import rospy
from distance_caculator.msg import Obst
from distance_caculator.srv import GetDistance

class ClosestObstacle:
    
    def __init__(self) -> None:
        rospy.init_node("closest_obstacle_node" , anonymous=True)
        # prepare service
        rospy.wait_for_service('get_distance')
        self.getDistService = rospy.ServiceProxy('get_distance', GetDistance)
        self.obstacle_publisher = rospy.Publisher('/ClosestObstacle' , Obst , queue_size=10)
        
        # obstacle details
        self.obstacles={
            "bookshelf" : (2.64, -1.55),
            "dumpster" :  (1.23, -4.57),
            "barrel" :    (-2.51, -3.08),
            "postbox" :   (-4.47, -0.57),
            "brick_box"	: (-3.44, 2.75),
            "cabinet" :	  (-0.45, 4.05),
            "cafe_table": (1.91, 3.37),
            "fountain" :  (4.08, 1.14)
        }
        
    def calculate_distance(self):
        while True :
            minDist = 10 ** 6
            for obstacle in self.obstacles.keys():    
                res = self.getDistService(obstacle)
                currentDist = res.distance
                if currentDist < minDist:
                    closest = obstacle
                    minDist = currentDist
            obs = Obst()
            obs.obstacle_name = closest
            obs.distance = minDist

            rospy.loginfo(f"Closest Obstacle Published: {closest} Distance : {minDist}")
            self.obstacle_publisher.publish(obs)

if __name__ == "__main__":
    startNode = ClosestObstacle()
    startNode.calculate_distance()
    rospy.spin()