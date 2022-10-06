from cmath import nan
import threading
import rospy
from pyzbar import pyzbar
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from clover import srv
from std_srvs.srv import Trigger
import math
from clover.srv import SetLEDEffect
import numpy as np
import cv2
import tf
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)
from mavros_msgs.srv import CommandBool
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
bridge = CvBridge()

msg = rospy.wait_for_message("/main_camera/camera_info", CameraInfo)
dC = np.array(msg.D, dtype="float64")
cM = np.reshape(np.array(msg.K, dtype="float64"), (3, 3))

target_qr = dict(x=nan, y=nan, counter=0)
found = False
qrs_lock = threading.Lock()
qr_size = 0.23
objPoint = np.array([(-qr_size/2, -qr_size/2, 0), (-qr_size/2, qr_size/2, 0), (qr_size/2, qr_size/2, 0), (qr_size/2, -qr_size/2, 0)])
listener = tf.TransformListener()

def navigate_wait_original(x=0, y=0, z=0, speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def navigate_wait(x=0, y=0, z=0, speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    while not rospy.is_shutdown():
        global found
        with qrs_lock:
            if found:
                if target_qr["counter"] < 10:
                    set_effect(r=255, g=255, b=0, effect="blink")
                    navigate_wait_original(x=target_qr["x"], y=target_qr["y"], z=1, frame_id='aruco_map')
                    rospy.sleep(1)
                    target_qr["counter"] += 1
                else:
                    set_effect(r=0, g=255, b=0, effect="fill")
                    navigate_wait_original(x=target_qr["x"], y=target_qr["y"], z=1, frame_id='aruco_map')
                    land()
                    rospy.sleep(3)
                    print("landed")
                    rospy.sleep(3)
                    navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)
                    rospy.sleep(3)
                    navigate_wait_original(x=target_qr["x"], y=target_qr["y"], z=1, frame_id='aruco_map')
                    navigate(x=x, y=y, z=z, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
                    found = False
            else:
                set_effect(r=0, g=0, b=0, effect="fill")
                navigate(x=x, y=y, z=z, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
                telem = get_telemetry(frame_id='navigate_target')
                if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
                    break
        rospy.sleep(0.2)

def image_callback(data):
    global found
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    barcodes = pyzbar.decode(cv_image)
    for barcode in barcodes:
        data = barcode.data.decode("utf-8")
        if data != qr:
            print("ignoring", data)
            continue
        with qrs_lock:
            if target_qr["x"] != nan and target_qr["y"] != nan:
                if target_qr["counter"] >= 10:
                    continue

        _, _, tvec = cv2.solvePnP(objPoint, np.array(barcode.polygon, dtype="float64"), cM, dC)

        p = PoseStamped()
        p.header.frame_id = "main_camera_optical"
        p.pose.position.x = tvec[0][0]
        p.pose.position.y = tvec[1][0]
        p.pose.position.z = tvec[2][0]
        pose_local = listener.transformPose("aruco_map", p)
        target_x = pose_local.pose.position.x
        target_y = pose_local.pose.position.y
        print(target_x, target_y)
        with qrs_lock:
            target_qr["x"] = target_x
            target_qr["y"] = target_y
            found = True

image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback, queue_size=1)

qr = input()

navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)
rospy.sleep(3)
navigate_wait(x=0, y=0, z=1, frame_id='aruco_map')
navigate_wait(x=4, y=4, z=1, frame_id='aruco_map')
navigate_wait(x=0, y=0, z=1, frame_id='aruco_map')
land()