import pygame
import sys
from math import sin, cos, pi,atan2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from geometry_msgs.msg import Twist

from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import DescribeParameters
from rcl_interfaces.msg import ParameterValue, ParameterType



import threading
from sensor_msgs.msg import Image
import cv2
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

        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
        else:
            print("No joystick found, no alternate control available, dispalying only camera feed")
            self.joystick = None

        self.clock = pygame.time.Clock()



        self.fl_angle = 0.0
        self.fr_angle = 0.0
        self.rl_angle = 0.0
        self.rr_angle = 0.0
        self.wheel_angle = 0.0

        self.speed = 0.0

        self.steering_mode = "unknown"
        self.available_modes = []

        self.rpm = 0

        self.bridge = CvBridge()

        self.publisher = StatePublisher()   
        self.reciever = StateReciever(self)
        self.mode_reader = ParameterReader(self, param_name="steering_mode")
        self.rpm_reader = ParameterReader(self, param_name="RPM")
        self.param_setter = ParameterSetter()

    def run(self):
        print('Running Agrorob Controller...')
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 1:  # Replace 0 with your button index
                        pygame.quit()
                        sys.exit()
                    elif event.button == 2:  # Example: Button 3 = cycle steering mode
                        self.cycle_steering_mode()
                    elif event.button == 3: # RPM up
                        self.add_rpm(5)
                    elif event.button == 0:
                        self.add_rpm(-5)


            if self.joystick:
                self.handle_gamepad_input()

            self.draw()

            if not self.publisher.send(self.wheel_angle, self.speed):
                print("Node is not running, exiting...")
                pygame.quit()
                sys.exit()
                break

            self.clock.tick(60)

    def draw(self):
        pygame.draw.rect(self.screen, (255, 255, 255), pygame.Rect(0, 0, 330, 330))


        speed_label = self.font.render(f"Speed: {self.speed:.2f}", True, (0,0,0))
        speed_rect = speed_label.get_rect()
        speed_rect.centerx = 150
        speed_rect.y = 250 + 10 
        self.screen.blit(speed_label, speed_rect)

        mode_label = self.font.render(f"Mode: {self.steering_mode}", True, (0, 0, 0))
        mode_rect = mode_label.get_rect()
        mode_rect.centerx = 150
        mode_rect.y = 10
        self.screen.blit(mode_label, mode_rect)

        rpm_label = self.font.render(f"RPM: {self.rpm}", True, (0,0,0))
        rpm_rect = rpm_label.get_rect()
        rpm_rect.centerx = 150
        rpm_rect.y = 250 - 20
        self.screen.blit(rpm_label, rpm_rect)

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

            color = (20, 80, 20)
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

        # Determine angles to draw per mode
        if self.steering_mode == 'car':
            # rear wheels straight (0)
            angles = [self.wheel_angle, self.wheel_angle, 0.0, 0.0]
        elif self.steering_mode == '4ws':
            # rear wheels opposite front wheels
            angles = [self.wheel_angle, self.wheel_angle, -self.wheel_angle, -self.wheel_angle]
        elif self.steering_mode == 'crab':
            # rear wheels same as front wheels
            angles = [self.wheel_angle, self.wheel_angle, self.wheel_angle, self.wheel_angle]
        elif self.steering_mode == 'pivot':
            # rear_left opposite front_left, rear_right same as front_right
            angles = [self.wheel_angle, self.wheel_angle, -self.wheel_angle, self.wheel_angle]
        else:
            angles = [0.0, 0.0, 0.0, 0.0]  # fallback

        for i in range(4):
            draw_arrow(arrow_centers[i], angles[i])
        # border
        pygame.draw.rect(self.screen, (20, 20, 20), pygame.Rect(0, 0, 330, 330), 2)
        pygame.display.flip()
    
    def update_wheel_angles(self, fl, fr, rl, rr, mode):
        self.fl_angle = fl
        self.fr_angle = fr
        self.rl_angle = rl
        self.rr_angle = rr
        self.steering_mode = mode


    
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

        self.wheel_angle =    self.joystick.get_axis(0) * pi/2

        self.speed = (self.joystick.get_axis(5) - self.joystick.get_axis(2))/2.0


    def cycle_steering_mode(self):
        if not self.available_modes:
            print("No steering modes available.")
            return

        try:
            current_index = self.available_modes.index(self.steering_mode)
        except ValueError:
            current_index = -1

        next_index = (current_index + 1) % len(self.available_modes)
        new_mode = self.available_modes[next_index]

        self.steering_mode = new_mode
        print(f"Switching to steering mode: {new_mode}")
        self.param_setter.set_parameter('steering_mode', new_mode)

    def add_rpm(self, value):
        new_rpm = self.rpm + value

        def on_rpm_set(success):
            if success:
                self.rpm = new_rpm
                print(f"\nRPM successfully set to {new_rpm}\n")
            else:
                print("\nFailed to change RPM.\n")

        self.param_setter.set_numeric_parameter('RPM', new_rpm, on_result=on_rpm_set)
            
    



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
        super().__init__('cmd_vel_publisher')

        qos_profile = QoSProfile(depth=5)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)

        self.nodeName = self.get_name()
        self.get_logger().info(f"{self.nodeName} started")
        self.loop_rate = self.create_rate(60)  # 60Hz

    def send(self, front, speed): 
        try:
            if rclpy.ok():

                # Create and populate Twist message
                twist = Twist()
                twist.linear.x = speed  # Forward speed
                twist.angular.z = front  # Steering mapped directly

                # Publish to /cmd_vel
                self.cmd_vel_pub.publish(twist)
                print(f"\rSent cmd_vel -> linear.x: {speed:.2f}, angular.z: {front:.2f}", end='')

                self.loop_rate.sleep()
            else:
                return False
        except KeyboardInterrupt:
            return False

        return True

        


    def send(self, front, speed): 
        try:
            if rclpy.ok():


                twist = Twist()
                twist.linear.x = speed
                twist.angular.z = front  # map front_rot directly to angular velocity

                self.cmd_vel_pub.publish(twist)
                print(f"\rSent cmd_vel -> linear.x: {speed:.2f}, angular.z: {front:.2f}", end='')

                self.loop_rate.sleep()
            else:
                return False
        except KeyboardInterrupt:
            return False

        return True




class ParameterReader(Node):
    def __init__(self, game: Game, target_node='steering_emulator', param_name='steering_mode'):
        super().__init__('parameter_reader')
        self.game = game
        self.target_node = target_node
        self.param_name = param_name

        # Clients for getting parameter value and descriptor
        self.get_param_client = self.create_client(GetParameters, f'/{target_node}/get_parameters')
        self.desc_param_client = self.create_client(DescribeParameters, f'/{target_node}/describe_parameters')

        # Wait for services
        self.get_logger().info(f"Waiting for services from '{target_node}'...")
        while not self.get_param_client.wait_for_service(timeout_sec=1.0) or \
              not self.desc_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Waiting for parameter services...")

        # Step 1: Get parameter value
        get_req = GetParameters.Request()
        get_req.names = [self.param_name]
        self.get_future = self.get_param_client.call_async(get_req)
        self.get_future.add_done_callback(self.handle_value_response)

        # Step 2: Get parameter descriptor
        if (self.param_name == "steering_mode"):
            desc_req = DescribeParameters.Request() 
            desc_req.names = [self.param_name]
            self.desc_future = self.desc_param_client.call_async(desc_req)
            self.desc_future.add_done_callback(self.handle_descriptor_response)

    def handle_value_response(self, future):
        try:
            response = future.result()
            if response.values:
                value = response.values[0]
                if value.type == ParameterType.PARAMETER_INTEGER:
                    self.game.rpm = value.integer_value
                    self.get_logger().info(f"RPM set to {value.integer_value}")
                elif value.type == ParameterType.PARAMETER_STRING:
                    self.game.steering_mode = value.string_value
                    self.get_logger().info(f"Steering mode set to '{value.string_value}'")
                else:
                    self.get_logger().warn(f"Unsupported parameter type: {value.type}")
            else:
                self.get_logger().warn(f"Parameter '{self.param_name}' not found in node '{self.target_node}'")
        except Exception as e:
            self.get_logger().error(f"Failed to get parameter value: {e}")

    def handle_descriptor_response(self, future):
        try:
            response = future.result()
            if not response.descriptors:
                self.get_logger().warn(f"No descriptor returned for '{self.param_name}'")
                return

            desc = response.descriptors[0].description
            if desc:
                modes = [s.strip() for s in desc.split('|') if s.strip()]
                self.game.available_modes = modes
                self.get_logger().info(f"Available steering modes: {modes}")
            else:
                self.get_logger().warn("Descriptor is empty.")
        except Exception as e:
            self.get_logger().error(f"Failed to get parameter descriptor: {e}")

class ParameterSetter(Node):
    def __init__(self, target_node_name='steering_emulator'):
        super().__init__('parameter_setter')
        self.target_node_name = target_node_name
        self.client = self.create_client(SetParameters, f'/{target_node_name}/set_parameters')

        self.get_logger().info(f"Waiting for '{self.target_node_name}' parameter service...")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Waiting for '{self.target_node_name}' parameter service...")

    def set_parameter(self, name: str, value: str):
        # Build request
        param = Parameter()
        param.name = name
        param.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)


        request = SetParameters.Request()
        request.parameters = [param]

        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)

    def set_numeric_parameter(self, name: str, value: int, on_result: callable = None):
        param = Parameter()
        param.name = name
        param.value = ParameterValue(
            type=ParameterType.PARAMETER_INTEGER,
            integer_value=value
        )

        request = SetParameters.Request()
        request.parameters = [param]

        future = self.client.call_async(request)

        # Wrap the callback to pass result to user-defined callback if provided
        def wrapped_callback(fut):
            try:
                response = fut.result()
                success = response.results[0].successful if response.results else False
                #self.get_logger().info(f"Set parameter '{name}' result: {success}")
                if on_result:
                    on_result(success)
            except Exception as e:
                self.get_logger().error(f"Error while setting parameter: {e}")
                if on_result:
                    on_result(False)

        future.add_done_callback(wrapped_callback)

        
    def handle_response(self, future):
        try:
            response = future.result()
            if response.results and response.results[0].successful:
                self.get_logger().info(f"Parameter set successfully.")
            else:
                self.get_logger().warn(f"Failed to set parameter.")
        except Exception as e:
            self.get_logger().error(f"Error while setting parameter: {e}")


def ros_spin_thread(executor):
    executor.spin()

if __name__ == '__main__':
    game = Game()
    executor = MultiThreadedExecutor()
    executor.add_node(game.publisher)
    executor.add_node(game.reciever)
    executor.add_node(game.mode_reader)
    executor.add_node(game.rpm_reader)
    executor.add_node(game.param_setter)


    spin_thread = threading.Thread(target=ros_spin_thread, args=(executor,), daemon=True)
    spin_thread.start()


    try:
        game.run()
    except KeyboardInterrupt:
        print("Keyboard interrupt received, shutting down.")
    finally:
        executor.shutdown()
        game.publisher.destroy_node()
        game.reciever.destroy_node()
        game.mode_reader.destroy_node()
        game.rpm_reader.destroy_node()
        game.param_setter.destroy_node()
        rclpy.shutdown()