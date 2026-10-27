#!/usr/bin/env python3
"""Simple, debug-friendly ROS 2 driver node for DJI RoboMaster TT (Tello SDK)."""

import math
import os
import signal
import socket
import sys
import threading
import time
from typing import Dict, Optional

import av
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, Range
from std_msgs.msg import String


class RmttDriverNode(Node):
    """Minimal UDP driver for DJI RoboMaster TT with explicit TX/RX debug logging."""

    DRONE_IP = '192.168.10.1'
    CMD_PORT = 8889
    STATE_PORT = 8890
    VIDEO_PORT = 11111

    def __init__(self) -> None:
        super().__init__('rmtt_driver_node')

        self.declare_parameter('bench_cooling_mode', False)
        self.bench_cooling_mode = bool(
            self.get_parameter('bench_cooling_mode').value
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom_raw', 10)
        self.tof_btm_pub = self.create_publisher(Range, '/tof_btm', 10)
        self.image_pub = self.create_publisher(Image, '/image_raw', 1)
        self.telemetry_pub = self.create_publisher(String, '/telemetry_raw', 10)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10
        )
        self.flight_cmd_sub = self.create_subscription(
            String, '/flight_cmd', self._flight_cmd_callback, 10
        )

        self.bridge = CvBridge()
        self._running = True
        self._last_rc_cmd = 'rc 0 0 0 0'
        self._last_rc_send_time = 0.0
        self._last_tx_time = 0.0
        self._last_state_time = 0.0
        self._latest_state: Dict[str, float] = {}

        # UDP command socket (port 8889)
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.cmd_sock.bind(('0.0.0.0', self.CMD_PORT))
        self.cmd_sock.settimeout(0.5)

        # UDP state socket (port 8890)
        self.state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.state_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.state_sock.bind(('0.0.0.0', self.STATE_PORT))
        self.state_sock.settimeout(0.5)

        # Background receiver threads
        threading.Thread(target=self._cmd_recv_loop, daemon=True).start()
        threading.Thread(target=self._state_recv_loop, daemon=True).start()
        threading.Thread(target=self._video_decode_loop, daemon=True).start()

        # Run initial SDK handshake in a short background thread so spacing is clean
        threading.Thread(target=self._init_sdk, daemon=True).start()

        # 5-second keepalive & connection status timer
        self.keepalive_timer = self.create_timer(5.0, self._keepalive_callback)

        self.get_logger().info(
            f'Simple RMTT Driver started. Target={self.DRONE_IP}:{self.CMD_PORT} '
            f'(bench_cooling_mode={self.bench_cooling_mode})'
        )

    def _init_sdk(self) -> None:
        """Send spaced initialization commands so the drone SDK parser does not drop them."""
        self._send_cmd('command')
        time.sleep(0.3)
        self._send_cmd('setresolution low')
        time.sleep(0.2)
        self._send_cmd('setbitrate 2')
        time.sleep(0.2)
        self._send_cmd('streamon')
        if self.bench_cooling_mode:
            time.sleep(0.3)
            self._send_cmd('motoron')

    def _send_cmd(self, cmd: str, log_tx: bool = True) -> bool:
        """Send a UDP command string to the drone and log TX or socket errors."""
        try:
            self.cmd_sock.sendto(
                cmd.encode('utf-8'), (self.DRONE_IP, self.CMD_PORT)
            )
            self._last_tx_time = time.monotonic()
            if log_tx:
                self.get_logger().info(
                    f'[TX -> {self.DRONE_IP}:{self.CMD_PORT}] "{cmd}"'
                )
            return True
        except OSError as exc:
            self.get_logger().error(
                f'[TX ERROR -> {self.DRONE_IP}:{self.CMD_PORT}] Failed to send "{cmd}": {exc}'
            )
            return False

    def _keepalive_callback(self) -> None:
        """Keep SDK mode alive and log drone connection health."""
        now = time.monotonic()
        age = now - self._last_state_time if self._last_state_time > 0 else -1.0
        if age < 0 or age > 3.0:
            self.get_logger().warn(
                'No telemetry received from drone on UDP 8890 yet '
                '(check Wi-Fi connection to RMTT-XXXXXX). Re-sending SDK "command"...'
            )
            self._send_cmd('command', log_tx=True)
            self._send_cmd('streamon', log_tx=False)
        else:
            bat = int(self._latest_state.get('bat', -1))
            temph = int(self._latest_state.get('temph', -1))
            tof = int(self._latest_state.get('tof', -1))
            self.get_logger().info(
                f'[STATUS] Connected | bat={bat}% | temp_max={temph}C | tof={tof}cm'
            )
            # Only send 'command' keepalive if no command was sent in the last 8 seconds,
            # so it never collides with user teleop commands while preventing the 15s SDK timeout.
            if (now - self._last_tx_time) >= 8.0:
                self._send_cmd('command', log_tx=False)

    def _flight_cmd_callback(self, msg: String) -> None:
        """Immediately forward discrete flight commands from teleop to the drone."""
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'[SUB /flight_cmd] Received: "{cmd}"')
        if not cmd:
            return
        self._send_cmd(cmd, log_tx=True)

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """Convert /cmd_vel Twist (ROS FLU) to Tello 'rc a b c d' and transmit."""
        now = time.monotonic()

        def clamp_rc(val: float) -> int:
            return int(max(-100, min(100, round(val * 100.0))))

        # ROS FLU -> Tello RC (a: right+, b: fwd+, c: up+, d: cw+)
        a = clamp_rc(-msg.linear.y)
        b = clamp_rc(msg.linear.x)
        c = clamp_rc(msg.linear.z)
        d = clamp_rc(-msg.angular.z)

        rc_cmd = f'rc {a} {b} {c} {d}'
        is_neutral = (a == 0 and b == 0 and c == 0 and d == 0)

        if not is_neutral:
            # Log when the RC command value changes, throttle identical repeats to ~15 Hz
            if rc_cmd != self._last_rc_cmd:
                self._send_cmd(rc_cmd, log_tx=True)
                self._last_rc_cmd = rc_cmd
                self._last_rc_send_time = now
            elif (now - self._last_rc_send_time) >= 0.066:
                self._send_cmd(rc_cmd, log_tx=False)
                self._last_rc_send_time = now
        elif self._last_rc_cmd != 'rc 0 0 0 0':
            self._send_cmd('rc 0 0 0 0', log_tx=True)
            self._last_rc_cmd = 'rc 0 0 0 0'
            self._last_rc_send_time = now

    def _cmd_recv_loop(self) -> None:
        """Listen on UDP 8889 and log every response from the drone."""
        while self._running and rclpy.ok():
            try:
                data, addr = self.cmd_sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break

            text = data.decode('utf-8', errors='ignore').strip()
            if text:
                self.get_logger().info(f'[RX <- {addr[0]}:{addr[1]}] "{text}"')

    @staticmethod
    def _parse_state_packet(packet: str) -> Dict[str, float]:
        parsed: Dict[str, float] = {}
        for item in packet.strip().split(';'):
            if ':' not in item:
                continue
            key, val_str = item.split(':', 1)
            try:
                parsed[key.strip()] = float(val_str.strip())
            except ValueError:
                continue
        return parsed

    def _state_recv_loop(self) -> None:
        """Listen on UDP 8890 for telemetry and publish /odom_raw and /tof_btm."""
        while self._running and rclpy.ok():
            try:
                data, _ = self.state_sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break

            text = data.decode('utf-8', errors='ignore')
            fields = self._parse_state_packet(text)
            if not fields:
                continue

            self._last_state_time = time.monotonic()
            self._latest_state = fields
            now_msg = self.get_clock().now().to_msg()

            telemetry_msg = String()
            telemetry_msg.data = text.strip()
            self.telemetry_pub.publish(telemetry_msg)

            vgx = fields.get('vgx', 0.0)
            vgy = fields.get('vgy', 0.0)
            yaw_deg = fields.get('yaw', 0.0)
            tof_cm = fields.get('tof', 0.0)

            vx = vgx * 0.1
            vy = -vgy * 0.1
            psi = -math.radians(yaw_deg)
            z_btm = tof_cm * 0.01

            odom_msg = Odometry()
            odom_msg.header.stamp = now_msg
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_stabilized'
            odom_msg.pose.pose.position.z = z_btm
            half_psi = 0.5 * psi
            odom_msg.pose.pose.orientation.z = math.sin(half_psi)
            odom_msg.pose.pose.orientation.w = math.cos(half_psi)
            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.linear.y = vy
            self.odom_pub.publish(odom_msg)

            btm_msg = Range()
            btm_msg.header.stamp = now_msg
            btm_msg.header.frame_id = 'base_stabilized'
            btm_msg.radiation_type = Range.INFRARED
            btm_msg.field_of_view = 0.47
            btm_msg.min_range = 0.10
            btm_msg.max_range = 8.00
            btm_msg.range = float(z_btm)
            self.tof_btm_pub.publish(btm_msg)

    def _video_decode_loop(self) -> None:
        """Decode H.264 stream on UDP 11111 and publish /image_raw throttled to 10 Hz."""
        last_pub_time = 0.0
        while self._running and rclpy.ok():
            container: Optional[av.container.InputContainer] = None
            try:
                container = av.open(
                    f'udp://0.0.0.0:{self.VIDEO_PORT}',
                    options={
                        'fflags': 'nobuffer+discardcorrupt',
                        'flags': 'low_delay',
                        'probesize': '32',
                        'analyzeduration': '0',
                        'max_delay': '0',
                    },
                    timeout=(5.0, 5.0),
                )
                for frame in container.decode(video=0):
                    if not (self._running and rclpy.ok()):
                        break
                    now = time.monotonic()
                    # Throttle ROS 2 image publishing to 10 Hz so depth_to_cloud_node
                    # and FastDDS never back-pressure or starve the main executor thread.
                    if (now - last_pub_time) < 0.10:
                        continue
                    last_pub_time = now
                    bgr_img = frame.to_ndarray(format='bgr24')
                    img_msg = self.bridge.cv2_to_imgmsg(bgr_img, encoding='bgr8')
                    img_msg.header.stamp = self.get_clock().now().to_msg()
                    img_msg.header.frame_id = 'camera_link'
                    self.image_pub.publish(img_msg)
            except Exception:
                time.sleep(1.0)
            finally:
                if container is not None:
                    try:
                        container.close()
                    except Exception:
                        pass

    def destroy_node(self) -> bool:
        self._running = False
        try:
            self.cmd_sock.sendto(b'rc 0 0 0 0', (self.DRONE_IP, self.CMD_PORT))
            self.cmd_sock.sendto(b'land', (self.DRONE_IP, self.CMD_PORT))
            self.cmd_sock.sendto(b'motoroff', (self.DRONE_IP, self.CMD_PORT))
            self.cmd_sock.close()
        except OSError:
            pass
        try:
            self.state_sock.close()
        except OSError:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(args=args)
    node = RmttDriverNode()

    def _sig_handler(sig, frame):
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
