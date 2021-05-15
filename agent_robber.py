#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Int8
from std_msgs.msg import Int8MultiArray

class Agent:
    def __init__(self):
        self.pub = rospy.Publisher('/environment/robber/act', Int8, queue_size=10)
        self.terminate = False

    def action(self,choice):
        self.pub.publish(choice)
    
    def observation(self,msg):
        #print(msg)
        print(self.terminate)

    def getTerminate(self,msg):
        self.terminate = msg.data[8]
        
if __name__ == '__main__':
    rospy.init_node('robber', anonymous=True)
    robber = Agent()
    while not rospy.is_shutdown():
        robber.action(choice=np.random.randint(0,4))
        rospy.Subscriber('/environment/robber/obs', Int8MultiArray, robber.observation)
        rospy.Subscriber('/environment', Int8MultiArray, robber.getTerminate)
        rospy.Rate(1).sleep()
