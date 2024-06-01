#!/usr/bin/env python3
from __future__ import print_function
from halcon_package.srv import * #注意是功能包名.srv
import rospy
from std_msgs.msg import Int32

from halcon_registration import Registrate

#请求信息虽然为空，但服务仍会接受到一个参数，但对回调函数无意义，因此为回调函数添加一个不会使用到的形参
def handle_registration(req):
    final_pose, matched_model= Registrate()
    final_pose = final_pose.reshape(-1).tolist()
    return RegistratePoseResponse(final_pose, matched_model)

if __name__ == "__main__":
    rospy.init_node('registration_server')
    server = rospy.Service('registration',RegistratePose, handle_registration)
    rospy.loginfo("Ready to registration.")
    rospy.spin()
