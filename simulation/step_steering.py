#!/usr/bin/env python3
"""
Publishes a ±step steering sequence on /cmd_vel.
"""

import math, argparse, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

rad = lambda d: d * math.pi / 180.0

class StepSteering(Node):
    def __init__(self, inc, max_, t_acc, t_hold, t_pause, rate):
        super().__init__("step_steering")

        # Build + then – sequence
        pos = [rad(d) for d in range(inc, max_ + 1, inc)]
        neg = [-a for a in pos]
        self.seq = []
        for a in pos + neg:
            self.seq += [(a, t_acc), (a, t_hold), (0.0, t_acc), (0.0, t_pause)]

        self.idx, self.t0 = 0, self._now()
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"Step‑steering sequence ready: ±{max_}° in {inc}° steps "
            f"[approach {t_acc}s, hold {t_hold}s, pause {t_pause}s]"
        )

    # --- helpers ----------------------------------------------------- #
    def _now(self):  # seconds (float)
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        angle, dur = self.seq[self.idx]
        if self._now() - self.t0 >= dur:
            self.idx += 1
            if self.idx >= len(self.seq):
                self.get_logger().info("Sequence finished – shutting down.")
                rclpy.shutdown(); return
            angle, dur = self.seq[self.idx]
            self.t0 = self._now()

        twist = Twist()
        twist.angular.z = angle
        self.pub.publish(twist)

# -------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--inc_deg",  type=int,   default=10)
    ap.add_argument("--max_deg",  type=int,   default=90)
    ap.add_argument("--approach", type=float, default=3.0)
    ap.add_argument("--hold",     type=float, default=2.0)
    ap.add_argument("--pause",    type=float, default=3.0)
    ap.add_argument("--rate",     type=float, default=30.0)
    a = ap.parse_args()

    rclpy.init()
    node = StepSteering(a.inc_deg, a.max_deg, a.approach, a.hold, a.pause, a.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()

