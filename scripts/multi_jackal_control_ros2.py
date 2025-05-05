#!/usr/bin/env python3

import os
import select
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

# Max limits and step sizes
MAX_LIN_VEL = 1.0
MAX_ANG_VEL = 2.0
LIN_STEP = 0.05
ANG_STEP = 0.1

# Key bindings: (robot, lin, ang)
keymap = {
    # Jackal 1: u i o j k l m , .
    'i': ('jackal1', 1.0, 0.0),
    'k': ('jackal1', 0.0, 0.0),
    ',': ('jackal1', -1.0, 0.0),
    'j': ('jackal1', 0.0, 1.0),
    'l': ('jackal1', 0.0, -1.0),
    'u': ('jackal1', 1.0, 1.0),
    'o': ('jackal1', 1.0, -1.0),
    'm': ('jackal1', -1.0, 1.0),
    '.': ('jackal1', -1.0, -1.0),

    # Jackal 2: q w e a s d z x c
    'w': ('jackal2', 1.0, 0.0),
    's': ('jackal2', 0.0, 0.0),
    'x': ('jackal2', -1.0, 0.0),
    'a': ('jackal2', 0.0, 1.0),
    'd': ('jackal2', 0.0, -1.0),
    'q': ('jackal2', 1.0, 1.0),
    'e': ('jackal2', 1.0, -1.0),
    'z': ('jackal2', -1.0, 1.0),
    'c': ('jackal2', -1.0, -1.0),

    # Jackal 3: r t y f g h v b n
    't': ('jackal3', 1.0, 0.0),
    'g': ('jackal3', 0.0, 0.0),
    'b': ('jackal3', -1.0, 0.0),
    'f': ('jackal3', 0.0, 1.0),
    'h': ('jackal3', 0.0, -1.0),
    'r': ('jackal3', 1.0, 1.0),
    'y': ('jackal3', 1.0, -1.0),
    'v': ('jackal3', -1.0, 1.0),
    'n': ('jackal3', -1.0, -1.0),
}

def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def smoother(target, current, step):
    if target > current:
        return min(target, current + step)
    elif target < current:
        return max(target, current - step)
    return target

def main():
    settings = termios.tcgetattr(sys.stdin) if os.name != 'nt' and sys.stdin.isatty() else None
    rclpy.init()
    node = rclpy.create_node('jackal_multi_teleop_key')

    publishers = {
        'jackal1': node.create_publisher(Twist, '/jackal1/cmd_vel', 10),
        'jackal2': node.create_publisher(Twist, '/jackal2/cmd_vel', 10),
        'jackal3': node.create_publisher(Twist, '/jackal3/cmd_vel', 10),
    }

    # Per-robot velocity state
    targets = {
        ns: {'lin': 0.0, 'ang': 0.0, 'ctrl_lin': 0.0, 'ctrl_ang': 0.0}
        for ns in publishers
    }

    print("Multi-Jackal Teleop Node Started.")
    print("Jackal1: u/i/o/j/k/l/m/,/.\nJackal2: q/w/e/a/s/d/z/x/c\nJackal3: r/t/y/f/g/h/v/b/n\n")

    try:
        while True:
            key = get_key(settings)
            if key == '\x03':
                break

            if key in keymap:
                ns, lin, ang = keymap[key]
                targets[ns]['lin'] = lin * MAX_LIN_VEL
                targets[ns]['ang'] = ang * MAX_ANG_VEL

            for ns, state in targets.items():
                state['ctrl_lin'] = smoother(state['lin'], state['ctrl_lin'], LIN_STEP)
                state['ctrl_ang'] = smoother(state['ang'], state['ctrl_ang'], ANG_STEP)

                twist = Twist()
                twist.linear.x = state['ctrl_lin']
                twist.angular.z = state['ctrl_ang']
                publishers[ns].publish(twist)

    except Exception as e:
        node.get_logger().error(f"Exception: {e}")

    finally:
        for pub in publishers.values():
            pub.publish(Twist())  # Stop all robots
        node.destroy_node()
        rclpy.shutdown()
        if settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
