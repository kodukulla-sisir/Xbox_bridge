import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from rclpy.qos import qos_profile_system_default
import serial
import time

class JoyToServo(Node):
    def __init__(self):
        super().__init__('joy_to_servo')
        
        # xARM setup
        self.subscription = self.create_subscription(
            Joy, '/custom_joy', self.joy_callback, 10
        )
        self.publisher = self.create_publisher(
            TwistStamped, '/servo_server/delta_twist_cmds', qos_profile_system_default
        )
        self.frame_id = 'link_base' 

        # Motor state
        self.motor_state = False      
        self.current_speed = 0.0      
        self.prev_action_button = 0  
        self.last_sent_rpm = -1 

        # Arduino setup
        self.arduino_port = '/dev/ttyACM0'
        self.baud_rate = 115200
        try:
            self.arduino = serial.Serial(self.arduino_port, self.baud_rate, timeout=0.05)
            time.sleep(2) 
            self.get_logger().info(f"Successfully connected to Arduino on {self.arduino_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.arduino = None

    def joy_callback(self, msg):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'link_base'

        def filter_stick(value):
            return 0.0 if abs(value) < 0.1 else value
        
        # Controller inputs
        deadman = msg.buttons[5]    # R1
        left_y = filter_stick(msg.axes[1])  # Left joystick up/down
        left_x = filter_stick(msg.axes[0])  # Left joystick right/left
        right_y = filter_stick(msg.axes[4]) # Right joystick up/down
        right_x = filter_stick(msg.axes[3]) # Right joystick right/left
        dpad_up_down = filter_stick(msg.axes[7])
        dpad_left_right = filter_stick(msg.axes[6])
        
        action_button = msg.buttons[0]  # A button
        l2_trigger = msg.axes[2]    # L2
        r2_trigger = msg.axes[5]    # R2


        # Arduino logic
        # Motor toggle
        if action_button == 1 and self.prev_action_button == 0:
            self.motor_state = not self.motor_state
            self.get_logger().info(f"Motor Toggled: {'ON' if self.motor_state else 'OFF'}")
        self.prev_action_button = action_button

        # Speed control
        if r2_trigger < 0.0: 
            self.current_speed += 2.0  
        if l2_trigger < 0.0: 
            self.current_speed -= 2.0  

        self.current_speed = max(0.0, min(300.0, self.current_speed))

        # Final output
        final_output = int(self.current_speed) if self.motor_state else 0
        
        if self.arduino is not None:
            if final_output != self.last_sent_rpm:
                command_str = f"{final_output}\n"
                self.arduino.write(command_str.encode('utf-8'))
                self.last_sent_rpm = final_output
            
            if self.arduino.in_waiting > 0:
                try:
                    response = self.arduino.readline().decode('utf-8', errors='ignore').strip()
                    if response:
                        self.get_logger().info(f"ARDUINO SAYS: {response}")
                except Exception:
                    pass


        # xARM logic
        if deadman == 1:
            twist.twist.linear.x = left_y * 0.02   
            twist.twist.linear.y = left_x * 0.02   
            twist.twist.linear.z = right_y * 0.02  
            twist.twist.angular.z = right_x * 0.1          
            twist.twist.angular.y = dpad_up_down * 0.1    
            twist.twist.angular.x = dpad_left_right * 0.1
        else:
            # Safety toggle
            twist.twist.linear.x = 0.0
            twist.twist.linear.y = 0.0
            twist.twist.linear.z = 0.0
            twist.twist.angular.x = 0.0
            twist.twist.angular.y = 0.0
            twist.twist.angular.z = 0.0
                    
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToServo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.arduino:
            node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()