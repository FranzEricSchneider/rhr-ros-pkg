#!/usr/bin/env python
import rospy
from math import sin, pi
from rhr_visualization.msg import Hand

def spoof_rhr_data():
    pub = rospy.Publisher('spoof_hand_data', Hand, queue_size=10)
    rospy.init_node('spoof_rhr_data', anonymous=True)
    r = rospy.Rate(50)

    hand = Hand()
    counter = 0.0
    rate_of_cycle = 40.0

    while not rospy.is_shutdown():
        sine = sin(counter/rate_of_cycle)
        fing_ang = (1.4*sine) + 1.4
        pre_ang = (-(pi/4)*sine)+pi/4
        if (sine > 0):
            contact = True
        else:
            contact = False
        scalar = (200*sine) + 200

        for i in range(3):
            hand.finger[i].proximal = fing_ang       # Proximal joints
            hand.finger[i].distal = 0.5*fing_ang     # Distal joints
            for j in range(9):                  
                hand.finger[i].contact[j] = contact     # Finger tactile contact
                hand.finger[i].pressure[j] = scalar     # Finger pressure scalar
        for i in range(2):
            hand.palm.preshape[i] = pre_ang
        for i in range(6):
            hand.palm.contact[i] = contact
            hand.palm.pressure[i] = scalar

        pub.publish(hand)
        counter+=1
        r.sleep()

if __name__ == '__main__':
    try:
        spoof_rhr_data()
    except rospy.ROSInterruptException: pass