#!/usr/bin/env python
#
# Copyright (c) 2024, RoboVerse community

import rospy
import asyncio
import json
import os
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import Joy, JointState
from nav_msgs.msg import Odometry

from go2_webrtc import Go2Connection, ROBOT_CMD, RTC_TOPIC
from go2_driver.utilities import gen_mov_command, gen_command, gen_pose_command, xyz_foot_position_body_to_joint_angles

JOY_SENSITIVITY = 0.3
ENABLE_BUTTON = 4


class Go2BaseNode:

    def __init__(self):

        self._conn = None
        self.robot_cmd_vel = None
        self._joy_state = Joy()
        self._cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self._cmd_vel_cb)
        self._joy_sub = rospy.Subscriber('joy', Joy, self._joy_cb)
        self._joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self._odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._last_message_stamp = rospy.Time.now()

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

    def on_data_channel_message(self, _, msgobj):
        if msgobj.get('topic') == RTC_TOPIC['LF_SPORT_MOD_STATE']:
            self.publish_joint_state(msgobj)

        if msgobj.get('topic') == RTC_TOPIC['ROBOTODOM']:
            self.publish_odom(msgobj)

    async def run(self, conn):

        self._conn = conn

        await self._conn.connect_robot()
        rospy.loginfo("Connected to Go2 !!!")

        while True:

            stamp = rospy.Time.now()

            if rospy.is_shutdown():
                break

            self.joy_cmd(stamp)

            await asyncio.sleep(0.1)

    def publish_odom(self, msg):

        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base'
        odom_msg.pose.pose.position.x = msg['data']['pose']['position']['x']
        odom_msg.pose.pose.position.y = msg['data']['pose']['position']['y']
        odom_msg.pose.pose.position.z = msg['data']['pose']['position']['z']
        odom_msg.pose.pose.orientation.x = msg['data']['pose']['orientation']['x']
        odom_msg.pose.pose.orientation.y = msg['data']['pose']['orientation']['y']
        odom_msg.pose.pose.orientation.z = msg['data']['pose']['orientation']['z']
        odom_msg.pose.pose.orientation.w = msg['data']['pose']['orientation']['w']

        self._odom_pub.publish(odom_msg)

        # Publish transform
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = 'odom'
        transform_stamped.child_frame_id = 'base_link'
        transform_stamped.transform.translation.x = msg['data']['pose']['position']['x']
        transform_stamped.transform.translation.y = msg['data']['pose']['position']['y']
        transform_stamped.transform.translation.z = msg['data']['pose']['position']['z']
        transform_stamped.transform.rotation.x = msg['data']['pose']['orientation']['x']
        transform_stamped.transform.rotation.y = msg['data']['pose']['orientation']['y']
        transform_stamped.transform.rotation.z = msg['data']['pose']['orientation']['z']
        transform_stamped.transform.rotation.w = msg['data']['pose']['orientation']['w']

        self._tf_broadcaster.sendTransform(transform_stamped)

    def publish_joint_state(self, msg):
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        FL_x = msg["data"]["foot_position_body"][0]
        FL_y = msg["data"]["foot_position_body"][1]
        FL_z = msg["data"]["foot_position_body"][2]

        FL_hip_joint, FL_thigh_joint, FL_calf_joint = xyz_foot_position_body_to_joint_angles(FL_x, FL_y, FL_z)

        FR_x = msg["data"]["foot_position_body"][3]
        FR_y = msg["data"]["foot_position_body"][4]
        FR_z = msg["data"]["foot_position_body"][5]

        FR_hip_joint, FR_thigh_joint, FR_calf_joint = xyz_foot_position_body_to_joint_angles(FR_x, FR_y, FR_z)

        RL_x = msg["data"]["foot_position_body"][6]
        RL_y = msg["data"]["foot_position_body"][7]
        RL_z = msg["data"]["foot_position_body"][8]

        RL_hip_joint, RL_thigh_joint, RL_calf_joint = xyz_foot_position_body_to_joint_angles(RL_x, RL_y, RL_z)

        RR_x = msg["data"]["foot_position_body"][9]
        RR_y = msg["data"]["foot_position_body"][10]
        RR_z = msg["data"]["foot_position_body"][11]

        RR_hip_joint, RR_thigh_joint, RR_calf_joint = xyz_foot_position_body_to_joint_angles(RR_x, RR_y, RR_z)

        joint_state.name = [
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
            ]
        joint_state.position = [
            FL_hip_joint, FL_thigh_joint, FL_calf_joint,
            FR_hip_joint, FR_thigh_joint, FR_calf_joint,
            RL_hip_joint, RL_thigh_joint, RL_calf_joint,
            RR_hip_joint, RR_thigh_joint, RR_calf_joint,
            ]

        self._joint_pub.publish(joint_state)

    def joy_cmd(self, stamp):

        if self.robot_cmd_vel and stamp - self.robot_cmd_vel['stamp'] < rospy.Duration(0.2):
            self.publish_command(self.robot_cmd_vel)

        if self._joy_state.header.stamp - stamp < rospy.Duration(0.2) and self._joy_state.buttons[ENABLE_BUTTON]:
            pitch = self._joy_state.axes[4] * JOY_SENSITIVITY
            pose_cmd = gen_pose_command(0.0, pitch, 0.0)
            self.publish_command(pose_cmd)

        if self._joy_state.buttons[1]:
            rospy.loginfo("Stand down")
            stand_down_cmd = gen_command(ROBOT_CMD['StandDown'])
            self.publish_command(stand_down_cmd)

        if self._joy_state.buttons[0]:
            rospy.loginfo("Stand up")
            stand_up_cmd = gen_command(ROBOT_CMD['StandUp'])
            self.publish_command(stand_up_cmd)
            balance_stand_cmd = gen_command(ROBOT_CMD['BalanceStand'])
            self.publish_command(balance_stand_cmd)

    def publish_command(self, cmd):
        self._conn.publish(cmd['topic'], cmd['data'], cmd['type'])

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
