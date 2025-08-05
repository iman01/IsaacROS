#!/usr/bin/env python3
"""
Step-speed profile  (0 → 1 → 0)

Defaults now match the synthetic dataset:
    inc  = 0.2
    hold = 6.0 s
"""
import argparse, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SlowStepSpeed(Node):
    def __init__(self, inc, hold, rate_hz):
        super().__init__("slow_step_speed")

        up   = [round(i*inc,2) for i in range(int(1/inc)+1)]   # 0,0.2,…1
        down = up[-2::-1]                                      # 0.8,…0
        self.sequence = up + down
        self.idx      = 0
        self.hold     = hold
        self.t_last   = self.get_clock().now().nanoseconds*1e-9

        self.pub   = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(1.0/rate_hz, self._tick)

        self.get_logger().info(f"Step profile — {inc=}, {hold=}, {rate_hz=} Hz")

    def _tick(self):
        now = self.get_clock().now().nanoseconds*1e-9
        if now - self.t_last >= self.hold:
            self.idx += 1
            if self.idx >= len(self.sequence):
                self.get_logger().info("Cycle complete — shutting down.")
                rclpy.shutdown(); return
            self.t_last = now

        msg = Twist();  msg.linear.x = self.sequence[self.idx]
        self.pub.publish(msg)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--inc",  type=float, default=0.2)   # NEW default ▶
    ap.add_argument("--hold", type=float, default=6.0)   # NEW default ▶
    ap.add_argument("--rate", type=float, default=30.0)
    args = ap.parse_args()

    rclpy.init()
    node = SlowStepSpeed(args.inc, args.hold, args.rate)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()

