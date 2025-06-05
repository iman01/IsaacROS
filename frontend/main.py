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

import threading
from sensor_msgs.msg import Image
import cv2
import numpy as np
from rclpy.executors import MultiThreadedExecutor

from cv_bridge import CvBridge

class Game:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()


        # just th
        self.WIDTH, self.HEIGHT = 330 + 330, 330 + 330
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption("Agrorob Controller")
        self.font = pygame.font.SysFont(None, 36)

        self.square_rect = pygame.Rect(50, 80, 40, 40)
        self.crab = False

        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
        else:
            print("No joystick found, no alternate control available, dispalying only camera feed")
            self.joystick = None

        self.clock = pygame.time.Clock()



        self.front_rot = 0.0
        self.back_rot = 0.0
        self.speed = 0.0


        self.bridge = CvBridge()

        self.publisher = StatePublisher()   
        self.reciever = StateReciever(self)

    def run(self):
        print('Running Agrorob Controller...')
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 0:  # Replace 0 with your button index
                        self.crab = not self.crab
                    if event.button == 1:  # Replace 0 with your button index
                        pygame.quit()
                        sys.exit()


            if self.joystick:
                self.handle_gamepad_input()

            self.draw()

            if not self.publisher.send(self.front_rot, self.back_rot, self.speed):
                print("Node is not running, exiting...")
                pygame.quit()
                sys.exit()
                break

            self.clock.tick(60)

    def draw(self):
        pygame.draw.rect(self.screen, (255, 255, 255), pygame.Rect(0, 0, 330, 330))


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
            angle = angle - pi/2  
            tail_length = 30
            head_length = 15
            head_width = 30

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

            color = (80, 20, 20) if self.crab else (20, 80, 20)
            pygame.draw.line(self.screen, color, (x_tail, y_tail), (x_base, y_base), 3)
            pygame.draw.polygon(self.screen, color, [(x_head, y_head), (left_x, left_y), (right_x, right_y)])


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
        # border
        pygame.draw.rect(self.screen, (20, 20, 20), pygame.Rect(0, 0, 330, 330), 2)
        pygame.display.flip()


    
    def draw_camera_left(self, msg):
        self.draw_camera(msg,0,1)

    def draw_camera_right(self, msg):
        self.draw_camera(msg,1,1)

    def draw_camera_front(self, msg):
        self.draw_camera(msg,1,0)

    def draw_camera(self, msg,x,y):
        width, height = 330, 330
        x = x * width
        y = y * height

        if msg is not None:
            try:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                img = cv2.resize(img, (width, height))

                surface = pygame.surfarray.make_surface(img.swapaxes(0, 1))  # Convert to pygame surface
                self.screen.blit(surface, (x, y))
                # border
                pygame.draw.rect(self.screen, (20, 20, 20), pygame.Rect(x, y, width, height), 2)



            except Exception as e:
                print(f"Image conversion error: {e}")
            else:
                return


    def handle_gamepad_input(self):
        dt = 0.1
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
            self.back_rot = -self.front_rot 
        # axois 2 l2 axis 5 r2 

        self.speed = (self.joystick.get_axis(5) - self.joystick.get_axis(2))/2.0



class StateReciever(Node):
        def __init__(self, game:Game=None):  
            super().__init__('state_reciever')
            if game is None:
                return 
            
            self.create_subscription(Image, '/camera_front_rgb', game.draw_camera_front, 10)
            self.create_subscription(Image, '/camera_left_rgb', game.draw_camera_left, 10)
            self.create_subscription(Image, '/camera_right_rgb', game.draw_camera_right, 10)
            



class StatePublisher(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=5)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        self.degree = pi / 180.0
        self.loop_rate = self.create_rate(60)

      

        # message declarations
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'odom'
        self.odom_trans.child_frame_id = 'base_link'
        self.joint_state = JointState()



        


    def send(self, front, back, speed): 
        def euler_to_quaternion(roll, pitch, yaw):
            qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
            qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
            qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
            qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
            return Quaternion(x=qx, y=qy, z=qz, w=qw)

        try:
            if rclpy.ok():
                rclpy.spin_once(self)

                now = self.get_clock().now()
                self.joint_state.header.stamp = now.to_msg()
                self.joint_state.name = ['front','back','speed']
                self.joint_state.position = [front,back,speed]




                # update transform
                # (moving in a circle with radius=2)
                self.odom_trans.header.stamp = now.to_msg()
                self.odom_trans.transform.translation.x = cos(1)*2
                self.odom_trans.transform.translation.y = sin(1)*2
                self.odom_trans.transform.translation.z = 1.6
                self.odom_trans.transform.rotation = \
                    euler_to_quaternion(0, 0, 1 + pi/2) # roll,pitch,yaw
# 
                self.joint_pub.publish(self.joint_state)
                # self.broadcaster.sendTransform(self.odom_trans)

                # Create new robot state
                # tilt += tinc
                # if tilt < -0.5 or tilt > 0.0:
                #     tinc *= -1
                # height += hinc
                # if height > 0.2 or height < 0.0:
                #     hinc *= -1
                # swivel += self.degree
                # angle += self.degree/4

                print(f"\rFront: {front}, Back: {back}, Speed: {speed}",end='')
                self.loop_rate.sleep()
            else:
                return False
        except KeyboardInterrupt:
            return False
        
        return True


def ros_spin_thread(executor):
    executor.spin()

if __name__ == '__main__':
    game = Game()
    executor = MultiThreadedExecutor()
    executor.add_node(game.publisher)
    executor.add_node(game.reciever)

    spin_thread = threading.Thread(target=ros_spin_thread, args=(executor,), daemon=True)
    spin_thread.start()


    try:
        game.run()
    finally:
        executor.shutdown()
        game.publisher.destroy_node()
        game.reciever.destroy_node()
        rclpy.shutdown()


    # TODO move this to keyboard interrupt from the gaem 
    rclpy.shutdown()