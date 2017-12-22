#! /usr/bin/env python

import rospy

from threading import Thread, Lock

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

        # actual drive state
        self.driveState = "WallFollow"

        # set up wall follower
        self.distanceToWall = 0.4
        self.angle = 1.57
        self.mutex = Lock()

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

            if(self.laser and self.odom): # laser and odom data arrived from callback
                # take a bunch of lasers in front of the robot (obstacles detection)
                lasers = 0
                for i in range(160, 200):
                    if(str(self.laser.ranges[i]) != 'nan'):
                        lasers += self.laser.ranges[i]
                actLasers = lasers / 40

                if(self.driveState == "WallFollow"):
                    pidValue = self.pid.pidExecute(self.distanceToWall, self.laser.ranges[359])
                    #rospy.loginfo(pidValue)
                    self.wallFollower(actLasers, pidValue)
                elif(self.driveState == "NeedToTurn"): # rote by an angle
                    self.rotate_angle(self.angle)

                self.velPub.publish(self.vel)

            self.rate.sleep()
        rospy.spin()

    def wallFollower(self, actLasers, pidValue):
        if(actLasers <= self.distanceToWall + 0.05): # obstacle in front of the robot
            self.driveState = "NeedToTurn"
        elif(pidValue == 0):
            self.vel.linear.x = 0.3
            self.vel.angular.z = 0.0
        elif(pidValue != 0):
            self.vel.linear.x = 0.15
            self.vel.angular.z = pidValue

    # obstacle in front of the robot, the robot needs to rotate
    def rotate_angle(self, angle):
        self.mutex.acquire()
        try:
            angular_speed = -0.3
            relative_angle = angle
            self.vel.linear.x = 0.0
            self.vel.angular.z = angular_speed

            # Setting the current time for distance calculus
            t0 = rospy.Time.now().to_sec()
            current_angle = 0

            while(current_angle < relative_angle):
                self.velPub.publish(self.vel)
                t1 = rospy.Time.now().to_sec()
                current_angle = abs(angular_speed)*(t1-t0)

            # Forcing the robot to stop
            self.vel.angular.z = 0
            self.velPub.publish(self.vel)
            self.driveState = "WallFollow"
        finally:
            self.mutex.release()

if __name__ == "__main__":
        maze_solver = MazeSolver()
        maze_solver.startSolver()
