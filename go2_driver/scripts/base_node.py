#!/usr/bin/env python
#
# Copyright (c) 2024, RoboVerse community

import rospy
import asyncio
import json
import os
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from go2_webrtc import Go2Connection, ROBOT_CMD, RTC_TOPIC
from go2_driver.utilities import gen_mov_command, gen_command, gen_pose_command

JOY_SENSITIVITY = 0.3
ENABLE_BUTTON = 4

class Go2BaseNode:

    def __init__(self):

        self._conn = None
        self.robot_cmd_vel = None
        self._joy_state = Joy()
        self._cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self._cmd_vel_cb)
        self._joy_sub = rospy.Subscriber('joy', Joy, self._joy_cb)

        self.rtc_topic_subs = RTC_TOPIC.values()

    def _cmd_vel_cb(self, msg):

        # TODO (alexlin): find the scaling from joy to actual velocity
        x = msg.linear.x
        y = msg.linear.y
        z = msg.angular.z

        self.robot_cmd_vel = gen_mov_command(x, y, z)
        self.robot_cmd_vel['stamp'] = rospy.Time.now()

    def _joy_cb(self, msg):
        self._joy_state = msg

    def on_validated(self):
        for topic in self.rtc_topic_subs:
            conn.data_channel.send(json.dumps({"type": "subscribe", "topic": topic}))

    def on_data_channel_message(self, message, msgobj):
        if msgobj.get('topic') == RTC_TOPIC['LF_SPORT_MOD_STATE']:
            rospy.loginfo(f"Received multiple state: {message}")

    async def run(self, conn):

        self._conn = conn

        await self._conn.connect_robot()
        rospy.loginfo("Connected to Go2 !!!")

        while True:

            stamp = rospy.Time.now()

            if rospy.is_shutdown():
                break

            if self.robot_cmd_vel and stamp - self.robot_cmd_vel['stamp'] < rospy.Duration(0.2):
                self._conn.publish(self.robot_cmd_vel['topic'], self.robot_cmd_vel['data'], self.robot_cmd_vel['type'])

            if self._joy_state.header.stamp - stamp < rospy.Duration(0.2) and self._joy_state.buttons[ENABLE_BUTTON]:
                pitch = self._joy_state.axes[4] * JOY_SENSITIVITY
                pose_cmd = gen_pose_command(0.0, pitch, 0.0)
                self._conn.publish(pose_cmd['topic'], pose_cmd['data'], pose_cmd['type'])

            if self._joy_state.buttons[1]:
                rospy.loginfo("Stand down")
                stand_down_cmd = gen_command(ROBOT_CMD['StandDown'])
                self._conn.publish(stand_down_cmd['topic'], stand_down_cmd['data'], stand_down_cmd['type'])

            if self._joy_state.buttons[0]:
                rospy.loginfo("Stand up")
                stand_up_cmd = gen_command(ROBOT_CMD['StandUp'])
                self._conn.publish(stand_up_cmd['topic'], stand_up_cmd['data'], stand_up_cmd['type'])
                balance_stand_cmd = gen_command(ROBOT_CMD['BalanceStand'])
                self._conn.publish(balance_stand_cmd['topic'], balance_stand_cmd['data'], balance_stand_cmd['type'])

            await asyncio.sleep(0.1)


if __name__ == '__main__':

    rospy.init_node('go2_base_node')

    base_node = Go2BaseNode()

    conn = Go2Connection(
        os.getenv("GO2_IP"),
        os.getenv("GO2_TOKEN"),
        on_validated=base_node.on_validated,
        on_message=base_node.on_data_channel_message,
    )

    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(base_node.run(conn))
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down...")
    finally:
        loop.run_until_complete(conn.pc.close())

    rospy.spin()
