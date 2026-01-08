
#!/usr/bin/env python3
import time
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class CmdVelToMDDS30(Node):
    def __init__(self):
        super().__init__("cmdvel_to_mdds30")

        # Parameters you can override at runtime
        self.declare_parameter("port", "/dev/serial0")  # matches your test
        self.declare_parameter("baud", 9600)
        self.declare_parameter("track_width", 0.18)     # meters
        self.declare_parameter("max_wheel_speed", 1.5)  # m/s at full command (tune)
        self.declare_parameter("cmd_timeout", 0.6)      # seconds
        self.declare_parameter("output_limit", 1)
        self.output_limit = float(self.get_parameter("output_limit").value)

        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baud").value)
        self.track_width = float(self.get_parameter("track_width").value)
        self.max_wheel_speed = float(self.get_parameter("max_wheel_speed").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)

        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=8,
            timeout=1
        )

        time.sleep(0.5)
        self.get_logger().info(f"MDDS30 serial opened on {port} @ {baud}")

        self.last_cmd_time = time.time()
        self.left = 0.0
        self.right = 0.0

        self.create_subscription(Twist, "/cmd_vel", self.cb, 10)
        self.create_timer(0.05, self.loop)  # 20 Hz send + watchdog

    def cb(self, msg: Twist):
        self.last_cmd_time = time.time()

        v = float(msg.linear.x)
        w = float(msg.angular.z) * 2.0


        # Wheel linear speeds (m/s)
        left_v = v - (w * self.track_width / 2.0)
        right_v = v + (w * self.track_width / 2.0)

        #Scaling both wheels
        max_mag = max(abs(left_v), abs(right_v))
        if max_mag > self.max_wheel_speed:
            scale = self.max_wheel_speed / max_mag
            left_v *= scale
            right_v *= scale

        # Normalize [-1..1]
        self.left = clamp(left_v / self.max_wheel_speed, -1.0, 1.0)
        self.right = clamp(right_v / self.max_wheel_speed, -1.0, 1.0)

    def loop(self):
        # Watchdog stop
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            self.left = 0.0
            self.right = 0.0

        self.send_motor(is_right=False, value=self.left)
        self.send_motor(is_right=True, value=self.right)

    def send_motor(self, is_right: bool, value: float):
        # MDDS30 Serial Simplified:
        # bit7 channel (0=left,1=right), bit6 direction (0=CW,1=CCW), bits0-5 speed (0-63)
        # value = clamp(value, -1.0, 1.0)
        value *= self.output_limit

        speed = int(round(abs(value) * 63.0))
        speed = clamp(speed, 0, 63)

        direction_bit = 0 if value >= 0 else 1
        channel_bit = 1 if is_right else 0

        cmd = (channel_bit << 7) | (direction_bit << 6) | speed
        self.ser.write(bytes([cmd]))


def main():
    rclpy.init()
    node = CmdVelToMDDS30()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
