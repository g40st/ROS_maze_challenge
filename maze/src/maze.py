#! /usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class MazeSolver:

    def __init__(self):
        rospy.init_node('MazeSolverNode')

        self.rate = rospy.Rate(10)

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.velPub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 5)

        self.vel = Twist()
        self.odom_x = 0


    def odom_callback(self, data):
        self.odom_x = data.pose.pose.position.x

    def startSolver(self):
        rospy.loginfo("start Maze Solver Node")

        while not rospy.is_shutdown():
            rospy.loginfo(self.odom_x)

            self.vel.linear.x = 0.1
            self.velPub.publish(self.vel)

            self.rate.sleep()
        rospy.spin()

if __name__ == "__main__":
        maze_solver = MazeSolver()
        maze_solver.startSolver()
