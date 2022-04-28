#!/usr/bin/env python

import os
import sys
import rospy
import rospkg
from gazebo_msgs.srv import (
    GetModelState,
)

def get_gazebo_models():
    model_name = "fetch"
    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    try:
        resp = get_model_state(model_name, 'world')
        return resp.pose.position
    except rospy.ServiceException as exc:
        print("get_link_properties did not process request: " + str(exc))

def save_config():
    path = rospkg.RosPack().get_path('gazebo_my') + '/config/'
    with open(path + 'conf.yaml', 'w') as conf_file:
        conf_file.truncate()
        if os.path.exists(path + 'mymap.pgm'):
            conf_file.write('map: True\n')
        else:
            conf_file.write('map: False\n')
        if os.path.exists(path + 'pic1.jpg'):
            conf_file.write('step1: True\n')
        else:
            conf_file.write('step1: False\n')
        if os.path.exists(path + 'pic2.jpg'):
            conf_file.write('step2: True\n')
        else:
            conf_file.write('step2: False\n')
        if os.path.exists(path + 'pic3.jpg'):
            conf_file.write('step3: True\n')
        else:
            conf_file.write('step3: False\n')

def main():
    rospy.init_node('finish')
    loc = get_gazebo_models()
    if loc.x > 0.75 or loc.x < -0.75 or loc.y > 0.75 or loc.y < -0.75:
        print("Please adjust Fetch to the direction of the arrow.")
    else:
        save_config()

if __name__ == '__main__':
    sys.exit(main())