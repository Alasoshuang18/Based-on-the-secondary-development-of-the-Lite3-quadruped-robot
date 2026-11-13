#!/usr/bin/env python3
"""Interactive simulation keyboard input; requires a separate gait controller."""
import math
import select
import sys
import termios
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist


def main():
    rclpy.init()
    node = None
    saved = None
    fd = None
    publisher = None
    try:
        if not sys.stdin.isatty():
            raise RuntimeError('请在交互终端使用 ros2 run，不要使用 ros2 launch 或输入重定向。')
        node = Node('lite3_only_keyboard', namespace='lite3_sim',
                    parameter_overrides=[Parameter('use_sim_time', value=True)])
        if not node.get_parameter('use_sim_time').value:
            raise RuntimeError('Lite3 keyboard requires use_sim_time=true')
        speed = node.declare_parameter('linear_speed', 0.003).value
        turn = node.declare_parameter('angular_speed', 0.01).value
        timeout = node.declare_parameter('key_timeout', 0.5).value
        for name, value, maximum in [('linear_speed', speed, 0.3),
                                     ('angular_speed', turn, 1.0),
                                     ('key_timeout', timeout, 2.0)]:
            if not math.isfinite(value) or not 0 < value <= maximum:
                raise ValueError(f'{name} 必须大于 0 且不超过 {maximum}')
        publisher = node.create_publisher(
            Twist, '/lite3_sim/cmd_vel',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE))
        fd = sys.stdin.fileno()
        saved = termios.tcgetattr(fd)
        settings = termios.tcgetattr(fd)
        # Keep ISIG enabled so Ctrl+C retains its normal meaning.
        settings[3] &= ~(termios.ICANON | termios.ECHO)
        settings[6][termios.VMIN] = 1
        settings[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, settings)
        print('W 前进 | S 后退 | A 左转 | D 右转 | 空格停止 | Q 退出')
        print(f'无方向键输入 {timeout} 秒自动发布停止；长按依赖键盘重复。')
        print('自由机身完成四腿抬落测试、lift_passed=True 且状态 READY 后，WASD 才能启用行走。')
        command = Twist()
        last_input = time.monotonic()
        quit_requested = False
        while rclpy.ok() and not quit_requested:
            if select.select([fd], [], [], 0.02)[0]:
                # os.read bypasses buffered text input; drain currently available bytes.
                import os
                keys = os.read(fd, 256)
                if not keys:
                    break
                for key in keys:
                    command = Twist()
                    key = chr(key).lower()
                    if key == 'w':
                        command.linear.x = speed
                    elif key == 's':
                        command.linear.x = -speed
                    elif key == 'a':
                        command.angular.z = turn
                    elif key == 'd':
                        command.angular.z = -turn
                    elif key == 'q':
                        quit_requested = True
                        break
                    last_input = time.monotonic()
            if quit_requested or time.monotonic() - last_input > timeout:
                command = Twist()
            if rclpy.ok():
                publisher.publish(command)
                rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        if saved is not None:
            termios.tcsetattr(fd, termios.TCSANOW, saved)
        if publisher is not None and rclpy.ok():
            publisher.publish(Twist())
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
