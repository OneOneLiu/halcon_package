#!/usr/bin/env python3
from __future__ import print_function 
import sys
import rospy
from halcon_package.srv import *    #注意是功能包名.srv

def registration_client():
    rospy.wait_for_service('registration')
    try:
        registration = rospy.ServiceProxy('registration', RegistratePose)
        resp1 = registration()
        return resp1
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)

if __name__ == "__main__":
    rospy.init_node("registration_client")
    response = registration_client()
    rospy.loginfo("Registration is OK !")
    print("final_pose is :",response.final_pose)
    print("matched_model is :",response.matched_model)
    

