#!/usr/bin/env python
#
# Copyright (c) 2024, RoboVerse community

import json
from go2_webrtc import Go2Connection, ROBOT_CMD, RTC_TOPIC


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