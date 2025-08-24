#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# edit these
TARGET        = 1.0    # final throttle max 1
RISE_S        = 3.0    # seconds to ramp 0 -> TARGET
HOLD_S        = 15.0    # seconds to hold at TARGET
RATE_HZ       = 30.0   
START_DELAY_S = 1.0    # wait at 0 before starting


class BrakeTest(Node):
    def __init__(self):
        super().__init__("brake_test_speed")
        self.target = float(max(-1.0, min(1.0, TARGET)))
        self.rise   = max(0.01, float(RISE_S))
        self.hold   = max(0.0,  float(HOLD_S))
        self.dt     = 1.0 / float(RATE_HZ)

        self.pub   = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(self.dt, self._tick)

        self.t0      = self.get_clock().now().nanoseconds * 1e-9
        self.phase   = "delay" 
        self.t_phase = 0.0

        self.get_logger().info(
            f"Brake test  target={self.target:.2f}  rise={self.rise:.2f}s  hold={self.hold:.2f}s  "
            f"start_delay={START_DELAY_S:.1f}s  rate={RATE_HZ:.0f}Hz"
        )

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        since_start = now - self.t0
        msg = Twist()

        if self.phase == "delay":
            msg.linear.x = 0.0
            if since_start >= START_DELAY_S:
                self.phase = "rise"
                self.t_phase = 0.0
        elif self.phase == "rise":
            self.t_phase += self.dt
            u = min(1.0, self.t_phase / self.rise) * self.target
            msg.linear.x = u
            if self.t_phase >= self.rise:
                self.phase = "hold"; self.t_phase = 0.0
        elif self.phase == "hold":
            self.t_phase += self.dt
            msg.linear.x = self.target
            if self.t_phase >= self.hold:
                self.phase = "brake"
        else:  # brake
            msg.linear.x = 0.0
            self.pub.publish(msg)
            self.get_logger().info("Brake command sent (0). Shutting down.")
            rclpy.shutdown(); return

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = BrakeTest()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()

