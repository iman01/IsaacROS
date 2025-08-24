#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# edit these
INC           = 0.2    # step incremenrts
HOLD_S        = 6.0    # seconds to hold each plateau
RATE_HZ       = 30.0   
START_DELAY_S = 1.0    # wait at 0 before starting


class ReverseStep(Node):
    def __init__(self):
        super().__init__("reverse_step_speed")
        inc = float(max(0.01, min(1.0, INC)))
        up  = [round(i * inc, 2) for i in range(int(1 / inc) + 1)]  # 0..1
        self.seq  = [-v for v in up] + [-v for v in up[-2:0:-1]]    # 0..-1..0
        self.hold = float(HOLD_S)
        self.dt   = 1.0 / float(RATE_HZ)

        self.idx    = 0
        self.started = False
        self.t0     = self.get_clock().now().nanoseconds * 1e-9
        self.tlast  = self.t0

        self.pub   = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(self.dt, self._tick)

        self.get_logger().info(
            f"Reverse step  inc={inc}  hold={self.hold}s  count={len(self.seq)}  "
            f"start_delay={START_DELAY_S:.1f}s  rate={RATE_HZ:.0f}Hz"
        )

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        msg = Twist()

        if not self.started:
            msg.linear.x = 0.0
            if now - self.t0 >= START_DELAY_S:
                self.started = True
                self.tlast = now
        else:
            if now - self.tlast >= self.hold:
                self.idx += 1
                if self.idx >= len(self.seq):
                    self.get_logger().info("Finshed")
                    rclpy.shutdown(); return
                self.tlast = now
            msg.linear.x = self.seq[self.idx]

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ReverseStep()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()

