#!/usr/bin/env python

import os
import cv2
import sys
import rospy
import rospkg
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from gazebo_msgs.srv import (
    GetModelState,
)
from fetch_actions import (
    PointHeadClient,
    FollowTrajectoryClient,
    GraspingClient,
)

def quaternion_to_euler(x, y, z, w):
    r = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    p = math.asin(2*(w*y - z*x))
    y = math.atan2(2*(w*z + x*y), 1 - 2*(z*z + y*y))
    return [y, p, r]

def get_gazebo_models():
    model_name = "fetch"
    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    try:
        resp = get_model_state(model_name, 'world')
        return resp.pose
    except rospy.ServiceException as exc:
        print("get_link_properties did not process request: " + str(exc))

def detect_area():
    global pos
    pos = get_gazebo_models()
    if 11.56 < pos.position.x < 12.56 and -12.34 < pos.position.y < -11.34:
        return 1
    elif 16.92 < pos.position.x < 17.92 and -3.94 < pos.position.y < -2.94:
        return 2
    elif 24 < pos.position.x < 25 and 8.66 < pos.position.y < 9.66:
        return 3
    else:
        return 0

def detect_orie():
    global area, pos
    [yaw, pitch, roll] = quaternion_to_euler(pos.orientation.x, pos.orientation.y, pos.orientation.z, pos.orientation.w)
    if area == 1 and (yaw > -0.785 or yaw < -2.355):
        return False
    elif area == 2 and (yaw < 0.785 or yaw > 2.355):
        return False
    elif area == 3 and -1.57 < yaw < 1.57:
        return False
    else:
        return True

def callback(data):
    global area
    bridge = CvBridge()
    cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
    image_path = rospkg.RosPack().get_path('gazebo_my') + '/config/'
    image_name = 'pic' + str(area) + '.jpg'
    cv2.imwrite(image_path + image_name, cv_img)
    if area == 1:
        print("Fetch takes a picture of instrument 1 successfully.")
    else:
        torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
        torso_action.move_to([0, ])
        print("Fetch takes a picture of instrument 2 successfully.")
    os.system("rosnode kill operate_" + str(area))

def main(argv):
    global area, ope
    area = detect_area()
    if area == 0:
        print("Please move Fetch inside the yellow wireframe area.")
        exit(0)
    elif area == 1 and len(argv) > 1:
        print("Please perform the correct operation.")
        exit(0)
    elif area == 2:
        if len(argv) == 1 or argv[1] != 'rise':
            print("Please perform the correct operation.")
            exit(0)
    elif area == 3:
        if len(argv) == 1 or argv[1] != 'button':
            print("Please perform the correct operation.")
            exit(0)
    if len(argv) == 1:
        ope = 'default'
    else:
        ope = argv[1]
    
    rospy.init_node('operate_' + str(area))
    torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
    torso_action.move_to([0, ])
    if not detect_orie():
        print("Please adjust Fetch to the direction of the arrow.")
    else:
        if ope == 'rise':
            torso_action.move_to([0.4, ])
            head_action = PointHeadClient()
            head_action.look_at(17.4218, -2.573, 1.4536, "map")
        elif ope == 'button':
            head_action = PointHeadClient()
            head_action.look_at(23.5, 9.154, 0.75, "map")
            os.system("rosrun gazebo_my press.py")
        else:
            head_action = PointHeadClient()
            head_action.look_at(12.0592, -12.9038, 0.75, "map")
    rospy.Subscriber('/head_camera/rgb/image_raw', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    sys.exit(main(sys.argv))