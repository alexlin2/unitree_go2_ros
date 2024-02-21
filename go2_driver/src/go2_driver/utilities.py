#!/usr/bin/env python
#
# Copyright (c) 2024, RoboVerse community

import json
import numpy as np
from go2_webrtc import Go2Connection, ROBOT_CMD, RTC_TOPIC

# joint lengths from urdf
L_HIP = 0.0955
L_THIGH = 0.213
L_CALF = 0.21
BASE_TO_HIP_X = 0.193
BASE_TO_HIP_Y = 0.12
BASE_TO_HIP_Z = 0.0

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


# inverse kinematics from end-effector (foot) position x, y, z in body frame
# to joint angles theta_hip, theta_thigh, theta_calf
def xyz_foot_position_body_to_joint_angles(x: float, y: float, z: float):

    x_leg = -np.sign(x) * (abs(x) - BASE_TO_HIP_X)
    y_leg = np.sign(y) * (abs(y) - BASE_TO_HIP_Y)
    z_leg = -np.sign(z) * (abs(z) - BASE_TO_HIP_Z)

    # calculate hip joint angle
    theta_hip = np.arctan2(z_leg, y_leg)

    # Calculate distance from hip joint to end-effector projected on yz-plane
    d = np.sqrt(y_leg**2 + z_leg**2)

    length_sq = x_leg**2 + y_leg**2 + z_leg**2

    # calculate thigh joint angle
    theta_thigh = np.arctan2(x_leg, d) - np.arccos((L_THIGH**2 - L_CALF**2 - length_sq) / (-2.0 * L_THIGH * np.sqrt(length_sq)))

    # calculate calf joint angle
    theta_calf = np.arccos((L_THIGH**2 + L_CALF**2 - length_sq) / (2.0 * L_THIGH * L_CALF))

    return theta_hip - np.pi / 2, theta_thigh + np.pi / 2, theta_calf - np.pi
