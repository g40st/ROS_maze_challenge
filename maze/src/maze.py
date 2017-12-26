#! /usr/bin/env python

import rospy

from threading import Thread, Lock
import math

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

        # set up wall detection
        self.mutex2 = Lock()
        self.minLaserValues = 60
        self.intervallEpsilon = 1.5

        # set up look detection
        self.knownPoints = []
        self.epsilionAroundPoints = 0.25
        self.timeoutForDetection = 12 # seconds

        # actual drive state
        self.driveState = "WallFollow"

        # set up wall follower
        self.leftHand = True        # True if the robot uses the left sensor for wall follow
        self.laserIndex = 359
        self.turnSpeed = -0.3
        self.distanceToWall = 0.4
        self.angle = 1.57
        self.minLasersSide = [150,170]
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
        if(not self.leftHand):
            self.laserIndex = 0
            self.minLasersSide = [190, 210]
            self.turnSpeed *= (-1)

        while not rospy.is_shutdown():
            if(self.laser and self.odom): # laser and odom data arrived from callback
                # append the origin to the known points
                if(len(self.knownPoints) == 0):
                    self.knownPoints.append([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, rospy.Time.now().to_sec()])

                # take the min laser value in front of the robot (obstacles detection)
                actMinLaserValue = min(min(self.laser.ranges[170:190], self.laser.ranges[self.minLasersSide[0] : self.minLasersSide[1]]))

                if(self.driveState == "WallDetection"):
                    self.vel.linear.x = 0.0
                    self.vel.angular.z = 0.0
                    self.wallDetection(actMinLaserValue)
                elif(self.driveState == "driveToWall"):
                    if(self.mutex.locked() == False):
                        if(actMinLaserValue <= self.distanceToWall): # obstacle in front of the robot
                            # save the actual position for loop detection
                            self.knownPoints.append([self.odom.pose.pose.position.x,self.odom.pose.pose.position.y, rospy.Time.now().to_sec()])

                            self.vel.linear.x = 0.0
                            self.vel.angular.z = 0.0
                            self.rotate_angle(self.angle, self.turnSpeed)
                            self.driveState = "WallFollow"
                        else:
                            self.vel.linear.x = 0.3
                            self.vel.angular.z = 0.0
                elif(self.driveState == "WallFollow"):
                    if(self.mutex.locked() == False):
                        # check known points (loop detection)
                        for i in range(0, len(self.knownPoints)):
                            if(self.odom.pose.pose.position.x - self.epsilionAroundPoints <= self.knownPoints[i][0] <= self.odom.pose.pose.position.x + self.epsilionAroundPoints and
                            self.odom.pose.pose.position.y - self.epsilionAroundPoints <= self.knownPoints[i][1] <= self.odom.pose.pose.position.y + self.epsilionAroundPoints and
                            self.knownPoints[i][2] + self.timeoutForDetection <  rospy.Time.now().to_sec()):
                                rospy.loginfo("Loop detected")
                                self.driveState = "WallDetection"

                        self.wallFollower(actMinLaserValue)

                self.velPub.publish(self.vel)

            self.rate.sleep()
        rospy.spin()

    def wallDetection(self, actMinLaserValue):
        self.mutex2.acquire()
        try:
            if(actMinLaserValue < self.distanceToWall):
                self.rotate_angle(self.angle, self.turnSpeed)
            else:
                # search for a wall to follow
                arrValues = []
                i = 0
                while(i < len(self.laser.ranges)):
                    valCounter = 0
                    for k in range(i+1, len(self.laser.ranges)):
                        # check if actual value is in interval
                        if(self.laser.ranges[i] - self.intervallEpsilon) <= self.laser.ranges[k] <= (self.laser.ranges[i] + self.intervallEpsilon):
                            valCounter += 1
                        else:
                            if(valCounter > self.minLaserValues):
                                arrValues.append([self.laser.ranges[i], i, k])
                            valCounter = 0
                            i = k
                        if(k == len(self.laser.ranges) - 1):
                            i = len(self.laser.ranges)

                # turn the robot to the wall (max distance)
                getLaserIndex = 180;
                rospy.loginfo(arrValues)
                if(arrValues):
                    getLaserIndex = int(math.floor((max(arrValues)[1] + max(arrValues)[2]) / 2))
                if(getLaserIndex <= 170 or getLaserIndex >= 180):
                    tmpAngle = self.laser.angle_min + (getLaserIndex * self.laser.angle_increment)
                    if(getLaserIndex < 170): # rotation to the right
                        self.rotate_angle(abs(tmpAngle), -0.25)
                    else:   # rotation to the left
                        self.rotate_angle(abs(tmpAngle), 0.25)
                self.driveState = "driveToWall"
        finally:
            self.mutex2.release()

    def wallFollower(self, actMinLaserValue):
        pidValue = self.pid.pidExecute(self.distanceToWall, self.laser.ranges[self.laserIndex])

        if(self.leftHand):
            pidValue = pidValue * (-1)

        if(actMinLaserValue < self.distanceToWall): # obstacle in front of the robot
            self.rotate_angle(self.angle, self.turnSpeed)
        elif(pidValue == 0):
            self.vel.linear.x = 0.3
            self.vel.angular.z = 0.0
        elif(pidValue != 0):
            self.vel.linear.x = 0.15
            self.vel.angular.z = pidValue

    # obstacle in front of the robot, the robot needs to rotate
    def rotate_angle(self, angle, speed):
        self.mutex.acquire()
        try:
            angular_speed = speed
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
            # fixes issue #6
            self.pid.resetValues()
        finally:
            self.mutex.release()

if __name__ == "__main__":
        maze_solver = MazeSolver()
        maze_solver.startSolver()
