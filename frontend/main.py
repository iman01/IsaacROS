import pygame
import sys
from math import sin, cos, pi,atan2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
import random



class Game:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()

        self.WIDTH, self.HEIGHT = 300, 330
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption("Agrorob Controller")
        self.font = pygame.font.SysFont(None, 36)

        self.square_rect = pygame.Rect(50, 80, 40, 40)
        self.crab = False

        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
        else:
            self.joystick = None

        self.clock = pygame.time.Clock()
        self.node = StatePublisher()

        self.front_rot = 0
        self.back_rot = 0 
        self.speed = 0.0

    def run(self):
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 0:  # Replace 0 with your button index
                        self.crab = not self.crab


            if self.joystick:
                self.handle_gamepad_input()

            self.draw()
            self.node.send(self.crab)
            self.clock.tick(60)

    def draw(self):
        self.screen.fill((255,255,255))

        text = "Crab" if self.crab else "Normal"
        drive_label = self.font.render(f"Drive mode: {text}", True, (0,0,0))
        drive_rect = drive_label.get_rect()
        drive_rect.centerx = 150
        drive_rect.y = 0 + 10  
        self.screen.blit(drive_label, drive_rect)

        speed_label = self.font.render(f"Speed: {self.speed:.2f}", True, (0,0,0))
        speed_rect = speed_label.get_rect()
        speed_rect.centerx = 150
        speed_rect.y = 250 + 10 
        self.screen.blit(speed_label, speed_rect)

        bar_y = 300
        bar_height = 12
        pygame.draw.rect(self.screen, (220, 220, 220), pygame.Rect(20, bar_y, 260, bar_height), 0)
        if self.speed >= 0:   pygame.draw.rect(self.screen, (0, 200, 10), pygame.Rect(150, bar_y, 130 * self.speed, bar_height), 0)
        else:  pygame.draw.rect(self.screen, (200, 5, 5), pygame.Rect(150 + 130 * self.speed, bar_y, -130 * self.speed, bar_height), 0)

        def draw_arrow(center, angle):
            angle = angle - pi/2  # Convert angle to radians
            tail_length = 30  # length of the arrow tail
            head_length = 15  # length of the arrowhead
            head_width = 30   # width of the arrowhead

            x_tail, y_tail = center

            x_base = x_tail + tail_length * cos(angle)
            y_base = y_tail + tail_length * sin(angle)
            x_head = x_base + head_length * cos(angle)
            y_head = y_base + head_length * sin(angle)

            # Arrowhead points (triangle)
            left_x = x_base  - head_width * sin(angle) / 2
            left_y = y_base  + head_width * cos(angle) / 2
            right_x = x_base + head_width * sin(angle) / 2
            right_y = y_base  - head_width * cos(angle) / 2

            color = (200, 50, 50) if self.crab else (10, 50, 10)
            pygame.draw.line(self.screen, color, (x_tail, y_tail), (x_base, y_base), 3)
            pygame.draw.polygon(self.screen, color, [(x_head, y_head), (left_x, left_y), (right_x, right_y)])
            # pygame.draw.line(self.screen, (0, 0, 0), (x_tail, y_tail), (x_base, y_base), 3)
            # pygame.draw.polygon(self.screen, (0, 0, 0), [(x_head, y_head), (left_x, left_y), (right_x, right_y)])


        arrow_spacing_x, arrow_spacing_y = 30, 50
        center_x, center_y = 150, 150  
        arrow_centers = [
            (center_x - arrow_spacing_x, center_y - arrow_spacing_y), 
            (center_x + arrow_spacing_x, center_y - arrow_spacing_y),  
            (center_x - arrow_spacing_x, center_y + arrow_spacing_y),  
            (center_x + arrow_spacing_x, center_y + arrow_spacing_y), 
        ]
        for i in range(4):
            draw_arrow(arrow_centers[i], self.front_rot if i < 2 else self.back_rot)
        pygame.display.flip()

    

    def handle_gamepad_input(self):
        dt = 0.1
        if self.joystick:
            # Get the axes values
            if self.crab:
                deadzone = 0.3
                x = self.joystick.get_axis(0) 
                y = -self.joystick.get_axis(1)

                if abs(x) < deadzone and abs(y) < deadzone:
                    x, y = 0, 0
                else:
                    self.front_rot = -atan2(y,x) + pi/2
                self.back_rot = self.front_rot
            else:
                
                self.front_rot += (self.joystick.get_axis(0) if abs(self.joystick.get_axis(0)) > 0.1 else 0 ) * dt
                self.front_rot = max(-pi/2, min(self.front_rot, pi/2))
                self.back_rot = -self.front_rot / 3
            # axois 2 l2 axis 5 r2 

            self.speed = (self.joystick.get_axis(5) - self.joystick.get_axis(2))/2.0



class StatePublisher(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        self.degree = pi / 180.0
        self.loop_rate = self.create_rate(60)

        # robot state
        tilt = 0.
        tinc = self.degree
        swivel = 0.
        self.angle = 0.
        height = 0.
        hinc = 0.005


        []

        body_shin_FR = 0.
        shin_wheel_FR = 0.
        body_shin_FL = 0.
        shin_wheel_FL = 0.
        body_shin_RR = 0.
        shin_wheel_RR = 0.
        body_shin_RL = 0.
        shin_wheel_RL = 0.




        # message declarations
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'odom'
        self.odom_trans.child_frame_id = 'base_link'
        self.joint_state = JointState()
    def send(self, positions): 
        def euler_to_quaternion(roll, pitch, yaw):
            qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
            qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
            qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
            qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
            return Quaternion(x=qx, y=qy, z=qz, w=qw)

        try:
            if rclpy.ok():
                rclpy.spin_once(self)

                body_shin_FR = random.uniform(-0.5, 0.5)
                shin_wheel_FR = random.uniform(-0.5, 0.5)
                body_shin_FL = random.uniform(-0.5, 0.5)
                shin_wheel_FL = random.uniform(-0.5, 0.5)
                body_shin_RR = random.uniform(-0.5, 0.5)
                shin_wheel_RR = random.uniform(-0.5, 0.5)
                body_shin_RL = random.uniform(-0.5, 0.5)
                shin_wheel_RL = random.uniform(-0.5, 0.5)

                now = self.get_clock().now()
                self.joint_state.header.stamp = now.to_msg()
                self.joint_state.name = ['body_shin_FR', 'shin_wheel_FR', 'body_shin_FL', 'shin_wheel_FL',
                    'body_shin_RR', 'shin_wheel_RR', 'body_shin_RL', 'shin_wheel_RL']
                self.joint_state.position = [body_shin_FR, shin_wheel_FR, body_shin_FL, shin_wheel_FL,
                    body_shin_RR, shin_wheel_RR, body_shin_RL, shin_wheel_RL]




                # update transform
                # (moving in a circle with radius=2)
                self.odom_trans.header.stamp = now.to_msg()
                self.odom_trans.transform.translation.x = cos(self.angle)*2
                # self.odom_trans.transform.translation.y = sin(angle)*2
                self.odom_trans.transform.translation.z = 1.6
                self.odom_trans.transform.rotation = \
                    euler_to_quaternion(0, 0, self.angle + pi/2) # roll,pitch,yaw

                # print(self.odom_trans)
                self.joint_pub.publish(self.joint_state)
                self.broadcaster.sendTransform(self.odom_trans)

                # Create new robot state
                # tilt += tinc
                # if tilt < -0.5 or tilt > 0.0:
                #     tinc *= -1
                # height += hinc
                # if height > 0.2 or height < 0.0:
                #     hinc *= -1
                # swivel += self.degree
                # angle += self.degree/4


                self.loop_rate.sleep()
            else:
                return False
        except KeyboardInterrupt:
            return False
        
        return True


if __name__ == '__main__':
    game = Game()
    game.run()