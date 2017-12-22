#! /usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from pid import PID

class MazeSolver:

    def __init__(self):
        rospy.init_node('MazeSolverNode')
        self.rate = rospy.Rate(10)

        rospy.Subscriber('/laserscan', LaserScan, self.laserscan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.velPub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 5)

        self.vel = Twist()
        self.laser = None
        self.odom = None

        # set up wall follower
        self.distanceToWall = 0.5

        # set up PID
        kp = 6
        ki = 2
        kd = 1.5
        outMin = -0.4
        outMax = 0.4
        iMin = -0.4
        iMax = 0.4
        self.pid = PID(kp, ki, kd, outMin, outMax, iMin, iMax)

    def odom_callback(self, odom_msg):
        self.odom = odom_msg

    def laserscan_callback(self, laser_msg):
        self.laser = laser_msg

    def startSolver(self):
        rospy.loginfo("start Maze Solver Node")

        while not rospy.is_shutdown():

            if(self.laser and self.odom): # laser and odom data received from callback
                pidValue = self.pid.pidExecute(self.distanceToWall, self.laser.ranges[359])
                #rospy.loginfo(pidValue)
                self.wallFollower(pidValue)

                self.velPub.publish(self.vel)

            self.rate.sleep()
        rospy.spin()

    def wallFollower(self, pidValue):
        if(pidValue == 0):
            self.vel.linear.x = 0.3
            self.vel.angular.z = 0.0
        elif(pidValue != 0):
            self.vel.linear.x = 0.15
            self.vel.angular.z = pidValue

if __name__ == "__main__":
        maze_solver = MazeSolver()
        maze_solver.startSolver()
