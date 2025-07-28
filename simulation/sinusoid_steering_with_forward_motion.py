#!/usr/bin/env python3
"""
Slow sinusoidal steering test for Agrorob.

Publishes geometry_msgs/Twist on /cmd_vel:
  linear.x = 0          (no forward motion)
  angular.z =  A · sin(2π f t)   [rad]

CLI parameters
--------------
--amp       Amplitude  (rad)    default: 0.75  (≈ 43 ° each side)
--freq      Frequency  (Hz)     default: 0.02  (one sweep every 50 s)
--duration  Duration   (s)      default: 0     (run until Ctrl‑C)

Example
-------
ros2 run agrorob_utils sinusoid_steering --amp 1.5 --freq 0.01 --duration 180
"""
import math, argparse, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SinusoidSteering(Node):
    def __init__(self, amp, freq, duration):
        super().__init__("sinusoid_steering")
        self.amp       = min(abs(amp), math.pi/2) * (1 if amp >= 0 else -1)  # clip to ±π/2
        self.freq      = abs(freq)
        self.duration  = max(duration, 0.0)

        self.pub       = self.create_publisher(Twist, "/cmd_vel", 10)
        self.t0        = self.get_clock().now().nanoseconds * 1e-9          # seconds
        self.timer     = self.create_timer(1.0/30.0, self._tick)            # 30 Hz

        info = (f"A={self.amp:.3f} rad  ({self.amp*180/math.pi:.1f} °) "
                f"f={self.freq:.3f} Hz  duration="
                f"{'∞' if self.duration==0 else self.duration}s")
        self.get_logger().info(f"Starting sinusoid steering: {info}")

    # ------------------------------------------------------------------ #
    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        t   = now - self.t0
        if self.duration and t > self.duration:
            self.get_logger().info("Test finished — shutting down.")
            rclpy.shutdown()
            return

        cmd = Twist()
        cmd.angular.z = self.amp * math.sin(2.0 * math.pi * self.freq * t)
        cmd.linear.x = 1.0
        self.pub.publish(cmd)

# ---------------------------------------------------------------------- #
def main():
    p = argparse.ArgumentParser()
    p.add_argument("--amp",      type=float, default=1.57,
                   help="Amplitude in radians (default 0.75 ≈ ±43 °)")
    p.add_argument("--freq",     type=float, default=0.02,
                   help="Frequency in Hz (default 0.02 Hz → 50 s period)")
    p.add_argument("--duration", type=float, default=0.0,
                   help="Total run time in seconds (0 = run forever)")
    args = p.parse_args()

    rclpy.init()
    node = SinusoidSteering(args.amp, args.freq, args.duration)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

