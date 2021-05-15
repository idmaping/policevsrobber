#!/usr/bin/env python3

import numpy as np
from PIL import Image
import time
import rospy
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Int8

class Blob:
    def __init__(self, SIZE):
        self.size = SIZE
        self.x = np.random.randint(0,self.size)
        self.y = np.random.randint(0,self.size)
        self.step = 0
        
    def __str__(self):
        return f"{self.x},{self.y},{self.step}"
    
    def move(self, x=False, y=False):
        self.x +=x
        self.y +=y
        
        #BOUNDARY
        if self.x < 0:
            self.x = 0
        elif self.x > self.size-1:
            self.x = self.size-1    
        if self.y < 0:
            self.y = 0
        elif self.y > self.size-1:
            self.y = self.size-1
        
    def action(self, choice):
        if choice == 0: #ATAS
            self.move(x=0, y=-1)
            self.step +=1
        elif choice == 1: #KANAN
            self.move(x=1, y=0)
            self.step +=1
        elif choice == 2: #KIRI
            self.move(x=-1, y=0)
            self.step +=1
        elif choice == 3: #BAWAH
            self.move(x=0, y=1)
            self.step +=1
        else: #DO NOTHING
            self.move(x=0, y=0)
        
class Environment:
    def __init__(self,max_step = 200, env_size = 10):
        self.size = env_size
        self.max_step = max_step
        self.pub_police = rospy.Publisher('/environment/police/obs', Int8MultiArray, queue_size=10)
        self.pub_robber = rospy.Publisher('/environment/robber/obs', Int8MultiArray, queue_size=10)
        self.pub_environment = rospy.Publisher('/environment', Int8MultiArray, queue_size=10)
        #RESET
        self.police = Blob(self.size)
        self.robber = Blob(self.size)
        self.bank = Blob(self.size)
        self.safehouse = Blob(self.size)
        self.terminate = False
        
    def reset(self):
        self.police = Blob(self.size)
        self.robber = Blob(self.size)
        self.bank = Blob(self.size)
        self.safehouse = Blob(self.size)
        self.terminate = False

    def terminate_condition(self):
        if self.police.x == self.robber.x and self.police.y == self.robber.y:
            self.terminate = True
        if self.bank.x == self.robber.x and self.bank.y == self.robber.y:
            self.terminate = True
        if self.police.step >= self.max_step or self.robber.step >= self.max_step:
            self.terminate = True

####---POLICE---------------------------------------------------------------------------------------------
    def police_reward(self):
        reward = -1
        if self.police.x == self.robber.x and self.police.y == self.robber.y: #BERHASIL MENANGKAP ROBBER
            reward = 25
        if self.bank.x == self.robber.x and self.bank.y == self.robber.y: #GAGAL MENANGKAP ROBBER
            reward = -200
        return reward

    def police_observe(self):
        self.terminate_condition()
        deltax_pr = self.police.x - self.robber.x
        deltay_pr = self.police.y - self.robber.y
        deltax_pb = self.robber.x - self.bank.x
        deltay_pb = self.robber.y - self.bank.y
        reward = self.police_reward()
        return [reward, deltax_pr, deltay_pr, deltax_pb, deltay_pb]

    def cb_police_act(self, msg):
        self.police.action(msg.data)
        array = Int8MultiArray(data=self.police_observe())
        self.pub_police.publish(array)
        data = [self.size,self.police.x,self.police.y,self.robber.x,self.robber.y,self.bank.x,self.bank.y,self.safehouse.x,self.safehouse.y,self.terminate]
        array = Int8MultiArray(data=data)
        self.pub_environment.publish(array)
####------------------------------------------------------------------------------------------------------

####---ROBBER---------------------------------------------------------------------------------------------
    def robber_reward(self):
        reward = -1
        if self.police.x == self.robber.x and self.police.y == self.robber.y: #BERHASIL MENANGKAP ROBBER
            reward = -200
        if self.bank.x == self.robber.x and self.bank.y == self.robber.y: #GAGAL MENANGKAP ROBBER
            reward = 25
        return reward

    def robber_observe(self):
        self.terminate_condition()
        deltax_rp = self.robber.x - self.police.x
        deltay_rp = self.robber.y - self.police.y
        deltax_rb = self.robber.x - self.bank.x
        deltay_rb = self.robber.y - self.bank.y
        reward = self.robber_reward()
        return [reward, deltax_rp, deltay_rp, deltax_rb, deltay_rb]

    def cb_robber_act(self, msg):
        self.robber.action(msg.data)
        array = Int8MultiArray(data=self.robber_observe())
        self.pub_robber.publish(array)
        data = [self.size,self.police.x,self.police.y,self.robber.x,self.robber.y,self.bank.x,self.bank.y,self.safehouse.x,self.safehouse.y,self.terminate]
        array = Int8MultiArray(data=data)
        self.pub_environment.publish(array)
####------------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    rospy.init_node('environment')
    env = Environment(max_step=200, env_size=20)
    rospy.Subscriber('/environment/police/act', Int8, env.cb_police_act)
    rospy.Subscriber('/environment/robber/act', Int8, env.cb_robber_act)
    rospy.spin()


    '''
    env = Environment(max_step=200, env_size=20)
    for _ in range(100):
        env.police.action(np.random.randint(0,4))
        env.robber.action(np.random.randint(0,4))
        env.render()
        if cv2.waitKey(500) & 0xFF == ord('q'):
            break
    '''
    
    
    
        
