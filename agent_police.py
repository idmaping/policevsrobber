#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Int8
from std_msgs.msg import Int8MultiArray
import pickle
import time

class Agent:
    def __init__(self, qtable = None):
        self.pub = rospy.Publisher('/environment/police/act', Int8, queue_size=10)
        self.terminate = False
        self.state = ((0,0),(0,0))
        self.reward = 0
        self.nextstate = ((0,0),(0,0))
        
        #Q-LEARNING HYPERPARAMETER
        self.CHECKPOINT = 1000
        self.EPSILON = 0.9
        self.EPSILON_DECAY = 0.9998
        self.LEARNING_RATE = 0.1
        self.DISCOUNT = 0.95
        
        self.size = 10
        self.dim_action = 4
        self.qtable_directory = qtable
        
        if self.qtable_directory is None:
            print("NEW QTABLE CREATED")
            self.q_table = {}
            for i in range(-self.size+1, self.size):
                for ii in range(-self.size+1, self.size):
                    for iii in range(-self.size+1, self.size):
                            for iiii in range(-self.size+1, self.size):
                                self.q_table[((i, ii), (iii, iiii))] = [np.random.uniform(-5, 0) for i in range(self.dim_action)]
        else:
            print("LOAD QTABLE")
            with open(self.qtable_directory, "rb") as f:
                self.q_table = pickle.load(f)
        
    def action(self,choice):
        print("action = ",choice)
        self.pub.publish(choice)
    
    def observation(self,msg):
        self.state = ((msg.data[1],msg.data[2]),(msg.data[3],msg.data[4]))

    def next_observation(self,msg):
        self.reward = msg.data[0]
        self.nextstate = ((msg.data[1],msg.data[2]),(msg.data[3],msg.data[4]))

    def getTerminate(self,msg):
        self.terminate = msg.data[9]
        
if __name__ == '__main__':
    rospy.init_node('police', anonymous=True)
    police = Agent()
    episode = 0
    while not rospy.is_shutdown():
        print("======== EPISODE : ",episode,"  ================")
        rospy.Subscriber('/environment/police/obs', Int8MultiArray, police.observation)
        print("state: ",police.state)
        if np.random.random() > police.EPSILON:
            print("epsilon: ",police.EPSILON, " EXPLOIT")
            action=0
        else:
            print("epsilon: ",police.EPSILON, " EXPLORE")
            action=np.random.randint(0,4)
        police.action(choice=action)
        rospy.Subscriber('/environment/police/next_obs', Int8MultiArray, police.next_observation)
        rospy.Subscriber('/environment', Int8MultiArray, police.getTerminate)
        print("action: ", action)
        print("_state: ", police.nextstate)
        print("reward: ", police.reward)
        print("terminate: ", police.terminate)
        print("table_state :",police.q_table[police.state])
        print("table_next_state :",police.q_table[police.nextstate])
        
        current_q = police.q_table[police.state][action]
        max_future_q = np.max(police.q_table[police.nextstate])
        
        print("current_q", current_q)
        print("max_future_q", max_future_q)

        if police.reward == 25:
            new_q = police.reward
        else :
            new_q = (1 - police.LEARNING_RATE) * current_q + police.LEARNING_RATE * (police.reward + police.DISCOUNT * max_future_q)

        police.q_table[police.state][action] = new_q

        if police.terminate:
            episode +=1
            if episode!=0 and episode%police.CHECKPOINT == 0:
                print("SAVING IN EPISODE ",episode)
                with open(f"police-qtable-{int(time.time())}-{episode}.pickle", "wb") as f:
                    pickle.dump(police.q_table, f)

        police.EPSILON *= police.EPSILON_DECAY
        rospy.Rate(100).sleep()

