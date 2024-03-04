#!/usr/bin/env python
#
# Copyright (c) 2024, RoboVerse community

import json
import numpy as np
from go2_driver.webrtc_driver import Go2Connection
from go2_driver.constants import ROBOT_CMD, RTC_TOPIC
from geometry_msgs.msg import Vector3

# joint lengths from urdf
L_HIP = 0.0955
L_THIGH = 0.213
L_CALF = 0.2135
BASE_TO_LEG_ORIGIN_X = 0.1934
BASE_TO_LEG_ORIGIN_Y = 0.0465
BASE_TO_LEG_ORIGIN_Z = 0.0

def gen_mov_command(x: float, y: float, z: float):

    command = {
        "type": "msg",
        "topic": RTC_TOPIC["SPORT_MOD"],
        "data": {
            "header": {"identity": {"id": Go2Connection.generate_id(), "api_id": ROBOT_CMD['Move']}},
            "parameter": json.dumps({"x": x, "y": y, "z": z}),
        },
    }

    return command


def gen_pose_command(roll: float, pitch: float, yaw: float):

    command = {
        "type": "msg",
        "topic": RTC_TOPIC["SPORT_MOD"],
        "data": {
            "header": {"identity": {"id": Go2Connection.generate_id(), "api_id": ROBOT_CMD['Euler']}},
            "parameter": json.dumps({"x": roll, "y": pitch, "z": yaw}),
        },
    }

    return command


def gen_command(cmd: int):
    command = {
        "type": "msg",
        "topic": RTC_TOPIC["SPORT_MOD"],
        "data": {
            "header": {"identity": {"id": Go2Connection.generate_id(), "api_id": cmd}},
            "parameter": json.dumps(cmd),
        },
    }

    return command


def pt_dist(p1: Vector3, p2: Vector3):
    return np.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2)


def xyz_foot_position_body_to_joint_angles(x: float, y: float, z: float, foot_index: int):

    foot_position = Vector3(x, y, z)

    base_tf_offset_hip_joint = Vector3(BASE_TO_LEG_ORIGIN_X, BASE_TO_LEG_ORIGIN_Y, BASE_TO_LEG_ORIGIN_Z)

    if foot_index > 1:
        base_tf_offset_hip_joint.x *= -1
    if foot_index % 2 == 1:
        base_tf_offset_hip_joint.y *= -1

    d = pt_dist(foot_position, base_tf_offset_hip_joint)

    # cos theory
    E = np.sqrt(d ** 2 - L_HIP ** 2)
    y = np.arccos((E ** 2 + L_THIGH ** 2 - L_CALF ** 2) / (2 * E * L_THIGH))
    S = np.arccos((L_CALF ** 2 + L_THIGH ** 2 - E ** 2) / (2 * L_CALF * L_THIGH)) - np.pi
    C = foot_position.x - base_tf_offset_hip_joint.x
    R = foot_position.y - base_tf_offset_hip_joint.y

    if foot_position.z < 0:
        A = np.arcsin(-C / E) + y
    else:
        A = -np.pi + np.arcsin(C / E) + y

    O = np.sqrt(d ** 2 - C ** 2)
    L = np.arcsin(R / O)

    if foot_position.z > 0:
        P = -1
    else:
        P = 1

    if foot_index % 2 == 0:
        J = P * (L - np.arcsin(L_HIP / O))
    else:
        J = P * (L + np.arcsin(L_HIP / O))

    if np.isnan(J + A + S):
        return 0, 0, 0

    return J, A, S

# # inverse kinematics from end-effector (foot) position x, y, z in body frame
# # to joint angles theta_hip, theta_thigh, theta_calf
# def xyz_foot_position_body_to_joint_angles(x: float, y: float, z: float):

#     x_leg = np.sign(x) * (abs(x) - BASE_TO_LEG_ORIGIN_X)
#     y_leg = np.sign(y) * (abs(y) - BASE_TO_LEG_ORIGIN_Y)
#     z_leg = np.sign(z) * (abs(z) - BASE_TO_LEG_ORIGIN_Z)

#     # Calculate distance and angle from leg origin to end-effector projected on yz-plane
#     d = np.linalg.norm([y_leg, z_leg])
#     alpha = np.arcsin(z_leg / d)

#     # calculate hip joint angle
#     theta_hip = (alpha + (np.pi/2 - np.arcsin(L_HIP / d))) * np.sign(y_leg)

#     r = R.from_euler('x', theta_hip, degrees=False)
#     x_rot, y_rot, z_rot = r.apply([x_leg, y_leg, z_leg])

#     d2 = np.linalg.norm([x_rot, z_rot])
#     beta = np.arctan2(z_rot, x_rot) + np.pi
#     beta2 = np.arccos((L_THIGH ** 2 + d2 ** 2 - L_CALF ** 2) / (2 * L_THIGH * d2))
#     beta3 = np.arccos((L_THIGH ** 2 + L_CALF ** 2 - d2 ** 2) / (2 * L_THIGH * L_CALF))

#     # calculate thigh joint angle
#     theta_thigh = beta - beta2

#     # calculate calf joint angle
#     theta_calf = np.pi - beta3

#     return theta_hip, np.pi / 2 - theta_thigh, -theta_calf
