#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Int8
from std_msgs.msg import Int8MultiArray
import image_resize
import cv2

color = {
        "POLICE":(255,0,0), #BIRU
        "ROBBER":(0,0,255), #MERAH
        "BANK":(43,181,48), #HIJAU
        "SAFEHOUSE":(115,87,87) #NILA
        }

def callback(msg):
    size = msg.data[0]
    police_x = msg.data[1]
    police_y = msg.data[2]
    robber_x = msg.data[3]
    robber_y = msg.data[4]
    bank_x = msg.data[5]
    bank_y = msg.data[6]
    safehouse_x = msg.data[7]
    safehouse_y = msg.data[8]

    env = np.zeros((size, size, 3), dtype=np.uint8)
    env[bank_x][bank_y] = color["BANK"]
    #env[safehouse_x][safehouse_y] = color["SAFEHOUSE"]
    env[robber_x][robber_y] = color["ROBBER"]
    env[police_x][police_y] = color["POLICE"]
    img = image_resize.resize(env,300,300)
    cv2.imshow("image", np.array(img))

    if cv2.waitKey(1) & 0xFF == ord('q'):
        pass

def render():
    rospy.init_node('environment_render' ,anonymous=True)
    rospy.Subscriber('/environment', Int8MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    render()