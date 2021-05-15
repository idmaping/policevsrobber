#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Int8
from std_msgs.msg import Int8MultiArray
import pickle

class Agent:
    def __init__(self, qtable = None):
        self.pub = rospy.Publisher('/environment/police/act', Int8, queue_size=10)
        self.terminate = False
        
        self.size = 10
        self.dim_action = 4
        self.qtable_directory = qtable
        if self.qtable_directory is None:
            print("NEW QTABLE CREATED")
            q_table = {}
            for i in range(-self.size+1, self.size):
                for ii in range(-self.size+1, self.size):
                    for iii in range(-self.size+1, self.size):
                            for iiii in range(-self.size+1, self.size):
                                q_table[((i, ii), (iii, iiii))] = [np.random.uniform(-5, 0) for i in range(self.dim_action)]
        else:
            print("LOAD QTABLE")
            with open(self.qtable_directory, "rb") as f:
                q_table = pickle.load(f)
        

    def action(self,choice):
        self.pub.publish(choice)
    
    def observation(self,msg):
        
        print(self.terminate)

    def getTerminate(self,msg):
        self.terminate = msg.data[8]
        
if __name__ == '__main__':
    rospy.init_node('police', anonymous=True)
    police = Agent()
    while not rospy.is_shutdown():
        police.action(choice=2)
        rospy.Subscriber('/environment/police/obs', Int8MultiArray, police.observation)
        rospy.Subscriber('/environment', Int8MultiArray, police.getTerminate)
        rospy.Rate(1).sleep()
