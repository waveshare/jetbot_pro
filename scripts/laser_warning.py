#!/usr/bin/env python
import roslib
import sys
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from jetbot_pro.cfg import laserWarningConfig

class LaserFilter:
    def __init__(self):
        self.start = False;
        self.Dist = 0.5
        self.Angle = 180
        self.kp = 0
        self.kd = 0
        self.last_err = 0
        self.Max = 6

        self.cmd = Twist()
        self.sub = rospy.Subscriber("scan",LaserScan,self.callback)
        self.pub = rospy.Publisher("filteredscan",LaserScan,queue_size=10)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
        Server(laserWarningConfig, self.config_callback)
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
        #Angle range
        Index = int(self.Angle/2*length/360)

        minDist = min(data.ranges[0:Index])
        tmp = min(data.ranges[(length - Index):])
        
        # Find the minimum distance and index
        if(minDist < tmp):
            minIndex = data.ranges[0:Index].index(minDist)
        else:
            minDist = tmp
            minIndex = data.ranges[(length - Index):].index(minDist)
            minIndex = minIndex + length - Index

        for i in range(Index,length -Index):
            newdata.ranges[i] = 0
            newdata.intensities[i] = 0 
            
        self.pub.publish(newdata)
        
        if(newdata.ranges[minIndex] < self.Dist):
            if(minIndex < length/2):err = minIndex
            else:err = minIndex - length    
            
            self.cmd.angular.z = (self.kp * err + self.kd * (err - self.last_err)) * 0.01
            self.last_err = err
      
            if(self.cmd.angular.z > self.Max):self.cmd.angular.z = self.Max
            elif(self.cmd.angular.z < -self.Max):self.cmd.angular.z = -self.Max
            if(self.start):
                self.cmd_pub.publish(self.cmd)
            else:
                self.cmd_pub.publish(Twist())
        else:
            self.cmd_pub.publish(Twist())

    def config_callback(self, config, level):
        self.start = config['start']
        self.Angle = config['laserAngle']
        self.Dist = config['distance']
        self.kp = config['kp']
        self.kd = config['kd']
        return config
        
if __name__ == '__main__':

    rospy.init_node('LidarFilter',anonymous=False)
    laser = LaserFilter()

    rospy.spin()
