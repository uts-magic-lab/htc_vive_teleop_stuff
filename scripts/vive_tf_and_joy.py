#!/usr/bin/env python

import time
import openvr
from math import sqrt, copysign
import pprint
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

"""
Getting poses and buttons into ROS.

Poses are published in TF.
Button presses in Joy topics /vive_left /vive_right .

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


def get_controller_ids(vrsys=None):
    if vrsys is None:
        vrsys = openvr.VRSystem()
    else:
        vrsys = vrsys
    left = None
    right = None
    for i in range(openvr.k_unMaxTrackedDeviceCount):
        device_class = vrsys.getTrackedDeviceClass(i)
        if device_class == openvr.TrackedDeviceClass_Controller:
            role = vrsys.getControllerRoleForTrackedDeviceIndex(i)
            if role == openvr.TrackedControllerRole_RightHand:
                right = i
            if role == openvr.TrackedControllerRole_LeftHand:
                left = i
    return left, right


def get_lighthouse_ids(vrsys=None):
    if vrsys is None:
        vrsys = openvr.VRSystem()
    else:
        vrsys = vrsys
        lighthouse_ids = []
    for i in range(openvr.k_unMaxTrackedDeviceCount):
        device_class = vrsys.getTrackedDeviceClass(i)
        if device_class == openvr.TrackedDeviceClass_TrackingReference:
            lighthouse_ids.append(i)
    return lighthouse_ids


def get_generic_tracker_ids(vrsys=None):
    if vrsys is None:
        vrsys = openvr.VRSystem()
    else:
        vrsys = vrsys
        generic_tracker_ids = []
    for i in range(openvr.k_unMaxTrackedDeviceCount):
        device_class = vrsys.getTrackedDeviceClass(i)
        if device_class == openvr.TrackedDeviceClass_GenericTracker:
            generic_tracker_ids.append(i)
    return generic_tracker_ids


def from_matrix_to_pose_dict(matrix):
    pose = {}
    # From http://steamcommunity.com/app/358720/discussions/0/358417008714224220/#c359543542244499836
    position = {}
    position['x'] = matrix[0][3]
    position['y'] = matrix[1][3]
    position['z'] = matrix[2][3]
    q = {}
    q['w'] = sqrt(max(0, 1 + matrix[0][0] + matrix[1][1] + matrix[2][2])) / 2.0
    q['x'] = sqrt(max(0, 1 + matrix[0][0] - matrix[1][1] - matrix[2][2])) / 2.0
    q['y'] = sqrt(max(0, 1 - matrix[0][0] + matrix[1][1] - matrix[2][2])) / 2.0
    q['z'] = sqrt(max(0, 1 - matrix[0][0] - matrix[1][1] + matrix[2][2])) / 2.0
    q['x'] = copysign(q['x'], matrix[2][1] - matrix[1][2])
    q['y'] = copysign(q['y'], matrix[0][2] - matrix[2][0])
    q['z'] = copysign(q['z'], matrix[1][0] - matrix[0][1])
    pose['position'] = position
    pose['orientation'] = q
    return pose


def from_matrix_to_transform(matrix, stamp, frame_id, child_frame_id,
                             to_ros_reference_frame=True):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = frame_id
    t.child_frame_id = child_frame_id
    # From http://steamcommunity.com/app/358720/discussions/0/358417008714224220/#c359543542244499836
    t.transform.translation.x = matrix[0][3]
    t.transform.translation.y = matrix[1][3]
    t.transform.translation.z = matrix[2][3]
    t.transform.rotation.w = sqrt(
        max(0, 1 + matrix[0][0] + matrix[1][1] + matrix[2][2])) / 2.0
    t.transform.rotation.x = sqrt(
        max(0, 1 + matrix[0][0] - matrix[1][1] - matrix[2][2])) / 2.0
    t.transform.rotation.y = sqrt(
        max(0, 1 - matrix[0][0] + matrix[1][1] - matrix[2][2])) / 2.0
    t.transform.rotation.z = sqrt(
        max(0, 1 - matrix[0][0] - matrix[1][1] + matrix[2][2])) / 2.0
    t.transform.rotation.x = copysign(
        t.transform.rotation.x, matrix[2][1] - matrix[1][2])
    t.transform.rotation.y = copysign(
        t.transform.rotation.y, matrix[0][2] - matrix[2][0])
    t.transform.rotation.z = copysign(
        t.transform.rotation.z, matrix[1][0] - matrix[0][1])

    if to_ros_reference_frame:
        tr = t.transform.translation
        rot = t.transform.rotation
        tr.z, tr.y, tr.x = tr.y, -tr.x, -tr.z
        rot.z, rot.y, rot.x = rot.y, -rot.x, -rot.z
    return t


def from_controller_to_joy(prev_unPacketNum,
                           pControllerState,
                           stamp,
                           frame_id):
    # docs: https://github.com/ValveSoftware/openvr/wiki/IVRSystem::GetControllerState

    d = {}
    d['unPacketNum'] = pControllerState.unPacketNum
    # on trigger .y is always 0.0 says the docs
    d['trigger'] = pControllerState.rAxis[1].x
    # 0.0 on trigger is fully released
    # -1.0 to 1.0 on joystick and trackpads
    d['trackpad_x'] = pControllerState.rAxis[0].x
    d['trackpad_y'] = pControllerState.rAxis[0].y
    # These are published and always 0.0
    # for i in range(2, 5):
    #     d['unknowns_' + str(i) + '_x'] = pControllerState.rAxis[i].x
    #     d['unknowns_' + str(i) + '_y'] = pControllerState.rAxis[i].y
    d['ulButtonPressed'] = pControllerState.ulButtonPressed
    d['ulButtonTouched'] = pControllerState.ulButtonTouched
    # To make easier to understand what is going on
    # Second bit marks menu button
    d['menu_button'] = bool(pControllerState.ulButtonPressed >> 1 & 1)
    # 32 bit marks trackpad
    d['trackpad_pressed'] = bool(pControllerState.ulButtonPressed >> 32 & 1)
    d['trackpad_touched'] = bool(pControllerState.ulButtonTouched >> 32 & 1)
    # third bit marks grip button
    d['grip_button'] = bool(pControllerState.ulButtonPressed >> 2 & 1)
    # System button can't be read, if you press it
    # the controllers stop reporting

    j = Joy()
    j.header.frame_id = frame_id
    j.header.stamp = stamp
    # Axes
    # Trigger, Trackpad X, Trackpad Y
    j.axes = [
        d['trigger'],
        d['trackpad_x'],
        d['trackpad_y']
    ]
    # Buttons
    # Trigger, Trackpad touched, trackpad pressed, menu, grip
    j.buttons = [
        d['trigger'] == 1.0,
        d['trackpad_touched'],
        d['trackpad_pressed'],
        d['menu_button'],
        d['grip_button']
    ]
    new_msg = prev_unPacketNum != d['unPacketNum']
    return new_msg, j


if __name__ == '__main__':
    print("===========================")
    print("Initializing OpenVR...")
    retries = 0
    max_init_retries = 4
    while retries < max_init_retries:
        try:
            openvr.init(openvr.VRApplication_Scene)
            break
        except openvr.OpenVRError as e:
            print("Error when initializing OpenVR (try {} / {})".format(
                  retries + 1, max_init_retries))
            print(e)
            retries += 1
            time.sleep(2.0)
    else:
        print("Could not initialize OpenVR, aborting.")
        print("Make sure the system is correctly plugged, you can also try")
        print("to do:")
        print("killall -9 vrcompositor vrmonitor vrdashboard")
        print("Before running this program again.")
        exit(0)

    print("Success!")
    print("===========================")

    print("VRSystem...")
    vrsystem = openvr.VRSystem()

    left_id, right_id = None, None
    print("===========================")
    print("Waiting for controllers...")
    try:
        while left_id is None or right_id is None:
            left_id, right_id = get_controller_ids(vrsystem)
            if left_id and right_id:
                break
            print("Waiting for controllers...")
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("Control+C pressed, shutting down...")
        openvr.shutdown()

    print("Left controller ID: " + str(left_id))
    print("Right controller ID: " + str(right_id))
    print("===========================")

    lighthouse_ids = get_lighthouse_ids(vrsystem)
    print("Lighthouse IDs: " + str(lighthouse_ids))

    generic_tracker_ids = get_generic_tracker_ids(vrsystem)
    print("Generic tracker IDs:" + str(generic_tracker_ids))

    poses_t = openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount
    poses = poses_t()

    pp = pprint.PrettyPrinter(indent=4)

    print("Initializing ROS...")
    rospy.init_node('HTCViveROS')
    print("Creating TransformBroadcaster...")

    br = tf2_ros.TransformBroadcaster()
    joy_left_pub = rospy.Publisher('vive_left', Joy, queue_size=1)
    prev_unPacketNum_left = 0
    joy_right_pub = rospy.Publisher('vive_right', Joy, queue_size=1)
    prev_unPacketNum_right = 0
    # Give a bit of time to initialize...
    rospy.sleep(3.0)
    print("Running!")

    # Vibration topic for each controller
    def vibration_cb(data, controller_id):
        # strength 0-3999, data contains a float expected
        # to be in between 0.0 and 1.0
        if data.data > 1.0:
            strength = 3999
        elif data.data < 0.0:
            strength = 0
        else:
            strength = int(data.data * 3999)
        vrsystem.triggerHapticPulse(controller_id, 0, strength)

    vib_left = rospy.Subscriber('vive_left_vibration', Float64, vibration_cb,
                                callback_args=left_id, queue_size=1)

    vib_right = rospy.Subscriber('vive_right_vibration', Float64, vibration_cb,
                                 callback_args=right_id, queue_size=1)

    # Internet says the tracking can be up until 250Hz
    r = rospy.Rate(250)
    while not rospy.is_shutdown():
        r.sleep()
        poses = poses_t()
        vrsystem.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding,
            0,
            len(poses),
            poses)

        now = rospy.Time.now()
        transforms = []
        # Hmd is always 0
        matrix = poses[0].mDeviceToAbsoluteTracking
        hmd_pose = from_matrix_to_transform(matrix, now, "world", "hmd")
        transforms.append(hmd_pose)

        # print("Hmd:")
        # pp.pprint(hmd_pose)

        for idx, _id in enumerate(lighthouse_ids):
            matrix = poses[_id].mDeviceToAbsoluteTracking
            lhouse_pose = from_matrix_to_transform(matrix,
                                                   now,
                                                   "world",
                                                   "lighthouse_" + str(idx))
            transforms.append(lhouse_pose)
            # print("Lighthouse #" + str(idx) + " :")
            # pp.pprint(lhouse_pose)

        if left_id:
            matrix = poses[left_id].mDeviceToAbsoluteTracking
            left_pose = from_matrix_to_transform(matrix,
                                                 now,
                                                 "world",
                                                 "left_controller")
            transforms.append(left_pose)
            result, pControllerState = vrsystem.getControllerState(left_id)
            new_msg, j = from_controller_to_joy(prev_unPacketNum_left,
                                                pControllerState,
                                                now,
                                                "left_controller")
            prev_unPacketNum_left = pControllerState.unPacketNum
            if new_msg:
                joy_left_pub.publish(j)
            # print("Left controller:")
            # # pp.pprint(d)
            # pp.pprint(left_pose)

        if right_id:
            matrix = poses[right_id].mDeviceToAbsoluteTracking
            right_pose = from_matrix_to_transform(matrix,
                                                  now,
                                                  "world",
                                                  "right_controller")
            transforms.append(right_pose)
            result, pControllerState = vrsystem.getControllerState(right_id)
            new_msg, j = from_controller_to_joy(prev_unPacketNum_right,
                                                pControllerState,
                                                now,
                                                "right_controller")
            prev_unPacketNum_right = pControllerState.unPacketNum
            if new_msg:
                joy_right_pub.publish(j)
            # print("Right controller:")
            # # pp.pprint(d)
            # pp.pprint(right_pose)

        for idx, _id in enumerate(generic_tracker_ids):
            matrix = poses[_id].mDeviceToAbsoluteTracking
            gen_track_pose = from_matrix_to_transform(matrix,
                                                      now,
                                                      "world",
                                                      "generic_tracker_" + str(idx))
            transforms.append(gen_track_pose)
            # print("Generic tracker #" + str(idx) + " :")
            # pp.pprint(gen_track_pose)

        br.sendTransform(transforms)

    openvr.shutdown()
