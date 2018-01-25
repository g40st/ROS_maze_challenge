import rospy

class PID:

    def __init__(self, kp, ki, kd, outMin, outMax, iMin, iMax):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.outMin = outMin
        self.outMax = outMax
        self.intMin = iMin
        self.intMax = iMax

        self.lastError = 0
        self.sumError = 0
        self.lastTime = 0

    def resetValues(self):
        self.lastError = 0
        self.sumError = 0
        self.lastTime = 0

    def pidExecute(self, should, actValue):
        now = rospy.Time.now().to_nsec()
        timeChange = now - self.lastTime
        error = should - actValue
        newErrorSum = self.sumError + (error * timeChange)

        if((newErrorSum >= self.intMin) and (newErrorSum <= self.intMax)):
            self.sumError = newErrorSum
        dError = (error - self.lastError) / timeChange
        output = (self.kp * error) + (self.ki * self.sumError) + (self.kd * dError)
        self.lastError = error
        self.last = now
        if(output > self.outMax):
            output = self.outMax
        if(output < self.outMin):
            output = self.outMin
        return output
