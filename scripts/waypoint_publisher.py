#!/usr/bin/env python
#
#   Copyright (c) 2018 John A. Dougherty. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name ATLFlight nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# In addition Supplemental Terms apply.  See the SUPPLEMENTAL file.
#

import rospy
import sys
import yaml

from snav_msgs.msg import WaypointWithConfig
from snav_msgs.msg import WaypointWithConfigArray


def publish_waypoints():
    pub = rospy.Publisher('input_waypoints', WaypointWithConfigArray,
                          queue_size=10, latch=True)
    rospy.init_node('waypoint_publisher', anonymous=False)
    wp_file = sys.argv[1]
    with open(wp_file, 'r') as stream:
        try:
            waypoint_list = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    msg = WaypointWithConfigArray()
    msg.header.stamp = rospy.Time.now()
    msg.header.seq += 1
    msg.header.frame_id = '/waypoint'

    for num, wp in enumerate(waypoint_list):
        wp_with_config_msg = WaypointWithConfig()

        waypoint = wp.get('waypoint')
        if waypoint is None:
            print("waypoint field missing")
            return

        header = waypoint.get('header')
        if header is not None:
            for entry in header:
                if type(header[entry]) is not dict:
                    setattr(wp_with_config_msg.waypoint.header, entry,
                            header[entry])
                else:
                    for item in header[entry]:
                        if type(header[entry][item]) is not dict:
                            setattr(wp_with_config_msg.waypoint.header.stamp,
                                    item, header[entry][item])
        wp_with_config_msg.waypoint.header.seq = num

        position = waypoint['position']
        for entry in position:
            setattr(wp_with_config_msg.waypoint.position, entry,
                    position[entry])

        velocity = waypoint.get('velocity')
        if velocity is not None:
            for entry in velocity:
                setattr(wp_with_config_msg.waypoint.velocity, entry,
                        velocity[entry])

        acceleration = waypoint.get('acceleration')
        if acceleration is not None:
            for entry in acceleration:
                setattr(wp_with_config_msg.waypoint.acceleration, entry,
                        acceleration[entry])

        jerk = waypoint.get('jerk')
        if jerk is not None:
            for entry in jerk:
                setattr(wp_with_config_msg.waypoint.jerk, entry, jerk[entry])

        yaw = waypoint.get('yaw')
        if yaw is not None:
            wp_with_config_msg.waypoint.yaw = yaw

        yaw_rate = waypoint.get('yaw_rate')
        if yaw_rate is not None:
            wp_with_config_msg.waypoint.yaw_rate = yaw_rate

        yaw_acceleration = waypoint.get('yaw_acceleration')
        if yaw_acceleration is not None:
            wp_with_config_msg.waypoint.yaw_acceleration = yaw_acceleration

        yaw_type = waypoint.get('yaw_type')
        if yaw_type is not None:
            wp_with_config_msg.waypoint.yaw_type = yaw_type

        constrained = waypoint.get('constrained')
        if constrained is not None:
            wp_with_config_msg.waypoint.constrained = constrained

        use_config = wp.get('use_config')
        if use_config is not None:
            wp_with_config_msg.use_config = use_config

        config = wp.get('config')
        if config is not None:
            max_lin_vel_norm = config.get('max_linear_velocity_norm')
            if max_lin_vel_norm is not None:
                wp_with_config_msg.config.max_linear_velocity_norm = \
                        max_lin_vel_norm

            max_lin_acc_norm = config.get('max_linear_acceleration_norm')
            if max_lin_acc_norm is not None:
                wp_with_config_msg.config.max_linear_acceleration_norm = \
                        max_lin_acc_norm

            max_yaw_velocity_norm = config.get('max_yaw_velocity_norm')
            if max_yaw_velocity_norm is not None:
                wp_with_config_msg.config.max_yaw_velocity_norm = \
                        max_yaw_velocity_norm

        msg.waypoints.append(wp_with_config_msg)

    pub.publish(msg)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        publish_waypoints()
    except rospy.ROSInterruptException:
        pass
