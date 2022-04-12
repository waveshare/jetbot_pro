#!/usr/bin/env python
import roslib
import sys
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from jetbot_pro.cfg import laserTrackingConfig

class LaserFilter:
    def __init__(self):
        self.start = False;
        self.Dist = 1
        self.Angle = 120
        self.Priority_angle = 40
        self.linear_kp = 0
        self.linear_kd = 0
        self.angular_kp = 0
        self.angular_kd = 0
        self.last_linear_err = 0
        self.last_angular_err = 0
        self.linear_Max = 1
        self.angular_Max = 6
        self.cmd = Twist()
        self.sub = rospy.Subscriber("scan",LaserScan,self.callback)
        self.pub = rospy.Publisher("filteredscan",LaserScan,queue_size=10)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
        Server(laserTrackingConfig, self.config_callback)
        rospy.on_shutdown(self.cancel)

    def cancel(self):
        self.cmd_pub.publish(Twist())
        self.sub.unregister()
        self.pub.unregister()
        self.cmd_pub.unregister()
        
    def callback(self,data):
        newdata = data

        #Convert tuple data to list
        newdata.ranges = list(data.ranges)
        newdata.intensities = list(data.intensities)
        #data len
        length = len(data.ranges)
        Index = int(self.Angle/2*length/360)                        #laser Angle range
        PIndex = int(self.Priority_angle/2*length/360)              #Priority Angle range
        if(PIndex >= Index):PIndex = Index - 1

        #Find the minimum distance and index 
        PminDist = min(data.ranges[0:PIndex])
        tmp = min(data.ranges[(length - PIndex):])
        
        if(PminDist < tmp):
            PminIndex = data.ranges[0:PIndex].index(PminDist)
        else:
            PminDist = tmp
            PminIndex = data.ranges[(length - PIndex):].index(PminDist)
            PminIndex = PminIndex + length - PIndex

        minDist = min(data.ranges[PIndex:Index])
        tmp = min(data.ranges[(length - Index):(length - PIndex)])

        if(minDist < tmp):
            minIndex = data.ranges[PIndex:Index].index(minDist)
            minIndex = minIndex + PIndex
        else:
            minDist = tmp
            minIndex = data.ranges[(length - Index):(length - PIndex)].index(minDist)
            minIndex = minIndex + length - Index

        if ((PminDist - minDist) < 0.3):
            minDist  = PminDist
            minIndex = PminIndex

        if(newdata.ranges[minIndex] < self.Dist):
            linear_err = newdata.ranges[minIndex] - 0.4     #Follow distance
            self.cmd.linear.x = (self.linear_kp * linear_err + self.linear_kd * (linear_err - self.last_linear_err))
            self.last_linear_err = linear_err
            #Limit maximum output
            if(self.cmd.linear.x > self.linear_Max):self.cmd.linear.x = self.linear_Max
            elif(self.cmd.linear.x < -self.linear_Max):self.cmd.linear.x = -self.linear_Max

            if (self.cmd.linear.x < 0) and (min(data.ranges[int(length/2)-Index:int(length/2)+Index]) < 0.4):
                self.cmd.linear.x = 0

            if(minIndex < length/2):angular_err = minIndex
            else:angular_err = minIndex - length    
            self.cmd.angular.z = (self.angular_kp * angular_err + self.angular_kd * (angular_err - self.last_angular_err)) * 0.01
            self.last_angular_err = angular_err
            #Limit maximum output
            if(self.cmd.angular.z > self.angular_Max):self.cmd.angular.z = self.angular_Max
            elif(self.cmd.angular.z < -self.angular_Max):self.cmd.angular.z = -self.angular_Max

            if(self.start):
                self.cmd_pub.publish(self.cmd)
            else:
                self.cmd_pub.publish(Twist())
        else:
            self.cmd_pub.publish(Twist())
            
        #Angle filtering
        for i in range(Index,length -Index):
            newdata.ranges[i] = 0
            newdata.intensities[i] = 0
            
        self.pub.publish(newdata)
        
    def config_callback(self, config, level):
        self.start = config['start']
        self.Angle = config['laserAngle']
        self.Priority_angle = config['PriorityAngle']
        self.Dist = config['distance']
        self.linear_kp = config['linear_kp']
        self.linear_kd = config['linear_kd']
        self.angular_kp = config['angular_kp']
        self.angular_kd = config['angular_kd']
        return config
        
if __name__ == '__main__':

    rospy.init_node('LidarFilter',anonymous=False)
    laser = LaserFilter()

    rospy.spin()
