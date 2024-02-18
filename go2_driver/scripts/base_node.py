#!/usr/bin/env python
#
# Copyright (c) 2024, RoboVerse community

import rospy
import asyncio
import json
import os
from geometry_msgs.msg import Twist

from go2_webrtc import Go2Connection, ROBOT_CMD
from go2_driver.utilities import gen_mov_command


class Go2BaseNode:

    def __init__(self, conn):

        self._conn = conn
        self.robot_cmd = None
        self._cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self._cmd_vel_cb)

    def _cmd_vel_cb(self, msg):

        # TODO (alexlin): find the scaling from joy to actual velocity
        x = msg.linear.x
        y = msg.linear.y
        z = msg.angular.z

        self.robot_cmd = gen_mov_command(x, y, z)
        self.robot_cmd['stamp'] = rospy.Time.now()

    async def run(self):

        await self._conn.connect_robot()

        while True:

            stamp = rospy.Time.now()

            if rospy.is_shutdown():
                break

            if self.robot_cmd and stamp - self.robot_cmd['stamp'] < rospy.Duration(0.2):
                self._conn.publish(self.robot_cmd['topic'], self.robot_cmd['data'], self.robot_cmd['type'])

            await asyncio.sleep(0.1)


if __name__ == '__main__':

    rospy.init_node('go2_base_node')

    conn = Go2Connection(
        os.getenv("GO2_IP"),
        os.getenv("GO2_TOKEN"),
    )

    base_node = Go2BaseNode(conn)

    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(base_node.run())
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down...")
    finally:
        loop.run_until_complete(conn.pc.close())

    rospy.spin()
