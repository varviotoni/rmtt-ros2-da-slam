#!/usr/bin/env python3
"""Interactive keyboard teleoperation node for DJI RoboMaster TT (Continuous & Step Modes)."""

import os
import select
import sys
import termios
import time
import tty
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

HELP_MSG = """
=====================================================
         DJI RoboMaster TT Teleoperation (Dual Mode) 
=====================================================
  Flight Controls:
    [T] : Takeoff
    [G] : Land
    [SPACE] : Emergency Motor Stop (immediate cut)
    [M] : Motor On (cooling)
    [N] : Motor Off

  Mode Switching:
    [C] : Continuous Smooth Mode (Hold keys to fly like app joystick)
    [V] : Step Motion Mode (Tap key for discrete small steps)

  Movement Keys:
        [W]
    [A] [S] [D]
      W / S : Forward / Backward (+X / -X)
      A / D : Left / Right (+Y / -Y)

  Altitude & Yaw:
        [I]
    [J] [K] [L]
      I / K : Up / Down (+Z / -Z)
      J / L : Turn Counter-Clockwise / Turn Clockwise

  Tuning & Safety:
    1 - 5 : Speed / Step size preset (20% to 100%)
    X     : Hover / Stop motion immediately

  Press Ctrl+C to exit.
=====================================================
"""


class RmttTeleopNode(Node):
    def __init__(self) -> None:
        super().__init__('rmtt_teleop_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.flight_cmd_pub = self.create_publisher(String, '/flight_cmd', 10)

        # Mode: 'continuous' (default, smooth like app) or 'step'
        self.mode = 'continuous'

        # Speed and step parameters
        self.speed_level = 0.40  # 40% speed
        self.step_duration = 0.40  # 400ms pulse for step mode

        # Continuous state
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        self.angular_z = 0.0

        # Timing and state tracking
        self.last_key_time = 0.0
        self.step_deadline = 0.0
        self.active_step_twist = Twist()
        self._was_moving = False

        # High-rate control loop (20 Hz)
        self.timer = self.create_timer(0.05, self._timer_callback)

    def trigger_step(self, lx=0.0, ly=0.0, lz=0.0, az=0.0) -> None:
        """Trigger a discrete timed velocity step."""
        self.active_step_twist = Twist()
        self.active_step_twist.linear.x = float(lx)
        self.active_step_twist.linear.y = float(ly)
        self.active_step_twist.linear.z = float(lz)
        self.active_step_twist.angular.z = float(az)

        self.step_deadline = time.monotonic() + self.step_duration
        self.cmd_vel_pub.publish(self.active_step_twist)

    def update_continuous(self, lx=0.0, ly=0.0, lz=0.0, az=0.0) -> None:
        """Update continuous velocity while key is held down."""
        self.linear_x = float(lx)
        self.linear_y = float(ly)
        self.linear_z = float(lz)
        self.angular_z = float(az)
        self.last_key_time = time.monotonic()

    def _timer_callback(self) -> None:
        now = time.monotonic()

        if self.mode == 'continuous':
            twist = Twist()
            # If a key has been repeated within 180ms, transmit the active velocity
            if (now - self.last_key_time) < 0.18:
                twist.linear.x = self.linear_x
                twist.linear.y = self.linear_y
                twist.linear.z = self.linear_z
                twist.angular.z = self.angular_z
                self.cmd_vel_pub.publish(twist)
                self._was_moving = True
            elif self._was_moving:
                # Key released: publish neutral once to halt motion
                self.linear_x = 0.0
                self.linear_y = 0.0
                self.linear_z = 0.0
                self.angular_z = 0.0
                self.cmd_vel_pub.publish(twist)
                self._was_moving = False

        elif self.mode == 'step':
            if now < self.step_deadline:
                self.cmd_vel_pub.publish(self.active_step_twist)
            else:
                if (
                    self.active_step_twist.linear.x != 0.0
                    or self.active_step_twist.linear.y != 0.0
                    or self.active_step_twist.linear.z != 0.0
                    or self.active_step_twist.angular.z != 0.0
                ):
                    self.zero_velocities()
                    self.cmd_vel_pub.publish(self.active_step_twist)

    def send_flight_cmd(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self.flight_cmd_pub.publish(msg)
        sys.stdout.write(f"\r\033[K>> Flight Command: {cmd.upper()}\r\n")
        sys.stdout.flush()

    def zero_velocities(self) -> None:
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        self.angular_z = 0.0
        self.last_key_time = 0.0
        self.step_deadline = 0.0
        self.active_step_twist = Twist()
        self._was_moving = False


def print_status(msg: str) -> None:
    """Print an in-place status line in raw terminal mode."""
    sys.stdout.write(f"\r\033[K>> {msg}")
    sys.stdout.flush()


def get_key():
    rlist, _, _ = select.select([sys.stdin], [], [], 0.02)
    if rlist:
        return sys.stdin.read(1)
    return ''


def main(args=None):
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(args=args)
    node = RmttTeleopNode()

    # Print help banner in cooked mode before raw mode
    print(HELP_MSG)

    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = get_key()

            if not key:
                continue

            k = key.lower()

            if k == '\x03':  # Ctrl+C
                break
            elif k == 't':
                node.send_flight_cmd('takeoff')
            elif k == 'g':
                node.zero_velocities()
                node.send_flight_cmd('land')
            elif k == ' ':
                node.zero_velocities()
                node.send_flight_cmd('emergency')
            elif k == 'm':
                node.send_flight_cmd('motoron')
            elif k == 'n':
                node.send_flight_cmd('motoroff')
            # Mode toggle
            elif k == 'c':
                node.zero_velocities()
                node.mode = 'continuous'
                print_status("MODE: Continuous Smooth (Hold to fly)")
            elif k == 'v':
                node.zero_velocities()
                node.mode = 'step'
                print_status("MODE: Step Motion (Single taps)")
            # Speed presets
            elif k in ['1', '2', '3', '4', '5']:
                preset = int(k) * 0.2
                node.speed_level = preset
                node.step_duration = 0.20 + (int(k) * 0.10)
                print_status(f"Speed / Step: {int(preset * 100)}%")
            elif k == 'x':
                node.zero_velocities()
                print_status("HOVER / STOP (Zeroed velocities)")

            # Directional commands handled according to active mode
            elif k == 'w':
                if node.mode == 'continuous':
                    node.update_continuous(lx=node.speed_level)
                    print_status(f"[Continuous] FORWARD ({node.speed_level:.1f})")
                else:
                    node.trigger_step(lx=node.speed_level)
                    print_status("[Step] FORWARD")

            elif k == 's':
                if node.mode == 'continuous':
                    node.update_continuous(lx=-node.speed_level)
                    print_status(f"[Continuous] BACKWARD ({-node.speed_level:.1f})")
                else:
                    node.trigger_step(lx=-node.speed_level)
                    print_status("[Step] BACKWARD")

            elif k == 'a':
                if node.mode == 'continuous':
                    node.update_continuous(ly=node.speed_level)
                    print_status(f"[Continuous] LEFT ({node.speed_level:.1f})")
                else:
                    node.trigger_step(ly=node.speed_level)
                    print_status("[Step] LEFT")

            elif k == 'd':
                if node.mode == 'continuous':
                    node.update_continuous(ly=-node.speed_level)
                    print_status(f"[Continuous] RIGHT ({-node.speed_level:.1f})")
                else:
                    node.trigger_step(ly=-node.speed_level)
                    print_status("[Step] RIGHT")

            elif k == 'i':
                if node.mode == 'continuous':
                    node.update_continuous(lz=node.speed_level)
                    print_status(f"[Continuous] UP ({node.speed_level:.1f})")
                else:
                    node.trigger_step(lz=node.speed_level)
                    print_status("[Step] UP")

            elif k == 'k':
                if node.mode == 'continuous':
                    node.update_continuous(lz=-node.speed_level)
                    print_status(f"[Continuous] DOWN ({-node.speed_level:.1f})")
                else:
                    node.trigger_step(lz=-node.speed_level)
                    print_status("[Step] DOWN")

            elif k == 'j':
                if node.mode == 'continuous':
                    node.update_continuous(az=node.speed_level)
                    print_status(f"[Continuous] TURN LEFT ({node.speed_level:.1f})")
                else:
                    node.trigger_step(az=node.speed_level)
                    print_status("[Step] TURN LEFT")

            elif k == 'l':
                if node.mode == 'continuous':
                    node.update_continuous(az=-node.speed_level)
                    print_status(f"[Continuous] TURN RIGHT ({-node.speed_level:.1f})")
                else:
                    node.trigger_step(az=-node.speed_level)
                    print_status("[Step] TURN RIGHT")

    except Exception as e:
        sys.stdout.write(f"\r\nTeleop error: {e}\r\n")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        sys.stdout.write("\r\n>> Teleop stopped cleanly.\r\n")
        sys.stdout.flush()
        node.zero_velocities()
        node._timer_callback()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
