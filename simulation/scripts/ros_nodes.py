# ros_nodes.py
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
import math, threading

class JointStateListener(Node):
    def __init__(self):
        super().__init__("joint_state_listener")
        self.subscription = self.create_subscription(JointState, "/joint_states", self.cb, 10)
        self.fl  = self.fr = self.rl = self.rr = 0.0
        self.fl_vel = self.fr_vel = self.rl_vel = self.rr_vel = 0.0
    def cb(self, msg):
        pos = dict(zip(msg.name, msg.position))
        vel = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        self.fl = pos.get("front_left", pos.get("front", 0.0))
        self.fr = pos.get("front_right", pos.get("front", 0.0))
        self.rl = pos.get("rear_left", pos.get("back", 0.0))
        self.rr = pos.get("rear_right", pos.get("back", 0.0))
        self.fl_vel = vel.get("front_left", 0.0)
        self.fr_vel = vel.get("front_right", 0.0)
        self.rl_vel = vel.get("rear_left", 0.0)
        self.rr_vel = vel.get("rear_right", 0.0)

class GhostJointStateListener(Node):
    def __init__(self):
        super().__init__("ghost_joint_state_listener")
        from agrorob_msgs.msg import RobotState
        self.subscription = self.create_subscription(RobotState, "/agrorob/robot_state", self.cb, 10)
        self.fl = self.fr = self.rl = self.rr = 0.0
        self.fl_vel = self.fr_vel = self.rl_vel = self.rr_vel = 0.0
    def cb(self, msg):
        self.fl = msg.left_front_wheel_turn_angle_rad
        self.fr = msg.right_front_wheel_turn_angle_rad
        self.rl = msg.left_rear_wheel_turn_angle_rad
        self.rr = msg.right_rear_wheel_turn_angle_rad
        self.fl_vel = impulses_to_rad_s(msg.left_front_wheel_encoder_imp)
        self.fr_vel = impulses_to_rad_s(msg.right_front_wheel_encoder_imp)
        self.rl_vel = impulses_to_rad_s(msg.left_rear_wheel_encoder_imp)
        self.rr_vel = impulses_to_rad_s(msg.right_rear_wheel_encoder_imp)

def impulses_to_rad_s(impulses_scaled, scale=100.0, cpr=54.0):
    rev_s = (impulses_scaled/scale) / cpr
    return rev_s * 2 * math.pi

class RosRuntime:
    def __init__(self, nodes):
        self.exec = MultiThreadedExecutor()
        for n in nodes: self.exec.add_node(n)
        self._t = threading.Thread(target=self.exec.spin, daemon=True)
    def start(self):
        self._t.start()
    def stop(self):
        self.exec.shutdown()
        for n in list(self.exec.get_nodes()): n.destroy_node()
