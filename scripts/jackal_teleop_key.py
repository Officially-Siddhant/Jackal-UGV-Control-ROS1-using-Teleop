#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Modified structure to conform to Publisher template (Wiki ROS) by Siddhant Baroth, 2025

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
msg = """
Controlling Jackal
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


speed = .2
turn = 1

def vels(speed, turn):
    return f"currently:\tspeed {speed}\tturn {turn}"
def smoother(speed, turn, x, th, ctrl_speed, ctrl_turn):
    target_speed = 0
    target_turn = 0
    target_speed = speed * x
    target_turn = turn * th

    if target_speed > ctrl_speed:
        ctrl_speed = min(target_speed, ctrl_speed + 0.02)
    elif target_speed < ctrl_speed:
        ctrl_speed = max(target_speed, ctrl_speed - 0.02)
    else:
        ctrl_speed = target_speed

    if target_turn > ctrl_turn:
        ctrl_turn = min(target_turn, ctrl_turn + 0.1)
    elif target_turn < ctrl_turn:
        ctrl_turn = max(target_turn, ctrl_turn - 0.1)
    else:
        ctrl_turn = target_turn

    return [ctrl_speed, ctrl_turn]
def publisher():
    global speed, turn
    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1

    control_speed = 0
    control_turn = 0
    print(msg)
    print(vels(speed, turn))
    while (1):
        key = getKey()
        if key in moveBindings.keys():
            x = moveBindings[key][0]
            th = moveBindings[key][1]
            count = 0
        elif key in speedBindings.keys():
            speed = speed * speedBindings[key][0]
            turn = turn * speedBindings[key][1]
            count = 0

            print(vels(speed, turn))
            if status == 14:
                print(msg)
            status = (status + 1) % 15
        elif key == ' ' or key == 'k':
            x = 0
            th = 0
            control_speed = 0
            control_turn = 0
        else:
            count = count + 1
            if count > 4:
                x = 0
                th = 0
            if (key == '\x03'):
                break

        control_speed, control_turn = smoother(speed, turn, x, th, control_speed, control_turn)

        twist = Twist()
        twist.linear.x = control_speed;
        twist.linear.y = 0;
        twist.linear.z = 0
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = control_turn
        pub.publish(twist)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin) # Save current terminal settings to restore later
    rospy.init_node('jackal_teleop')
    pub = rospy.Publisher('~cmd_vel', Twist, queue_size=5)
    try:
        publisher()

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
