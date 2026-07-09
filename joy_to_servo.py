import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from rclpy.qos import qos_profile_system_default
import serial

class JoyToServo(Node):
    def __init__(self):
        super().__init__('joy_to_servo')
        
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(TwistStamped, '/servo_server/delta_twist_cmds', qos_profile_system_default)
        
        self.current_twist = TwistStamped()
        self.current_twist.header.frame_id = 'link_base'
        
        self.motor_state = False      
        self.current_speed = 0.0      
        self.prev_action_button = 0  
        self.last_sent_rpm = -1 

        self.arduino_port = '/dev/ttyACM0'
        self.baud_rate = 115200
        try:
            self.arduino = serial.Serial(self.arduino_port, self.baud_rate, timeout=0.05)
            self.get_logger().info("Connected to Arduino")
        except Exception:
            self.arduino = None

        self.create_timer(0.02, self.publish_twist)
        self.create_timer(0.1, self.read_arduino)

    def read_arduino(self):
        if self.arduino and self.arduino.in_waiting > 0:
            try:
                resp = self.arduino.readline().decode('utf-8', errors='ignore').strip()
                if resp:
                    self.get_logger().info(f"ARDUINO: {resp}")
            except Exception:
                pass

    def publish_twist(self):
        self.current_twist.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.current_twist)

    def joy_callback(self, msg):
        def filter_stick(value):
            return 0.0 if abs(value) < 0.1 else value
        
        deadman = msg.buttons[5]
        left_y = filter_stick(msg.axes[1])
        left_x = filter_stick(msg.axes[0])
        right_y = filter_stick(msg.axes[4])
        right_x = filter_stick(msg.axes[3])
        dpad_up_down = filter_stick(msg.axes[7])
        dpad_left_right = filter_stick(msg.axes[6])
        
        action_button = msg.buttons[0]
        l2_trigger = msg.axes[2]
        r2_trigger = msg.axes[5]

        if action_button == 1 and self.prev_action_button == 0:
            self.motor_state = not self.motor_state
        self.prev_action_button = action_button

        target_speed = 0.0
        if r2_trigger < 1.0:
            target_speed = ((1.0 - r2_trigger) / 2.0) * 400.0
        elif l2_trigger < 1.0:
            target_speed = -(((1.0 - l2_trigger) / 2.0) * 400.0)
        
        self.current_speed = max(-400.0, min(400.0, target_speed))
        final_output = int(self.current_speed) if self.motor_state else 0
        
        if self.arduino and final_output != self.last_sent_rpm:
            self.arduino.write(f"{final_output}\n".encode('utf-8'))
            self.last_sent_rpm = final_output

        if deadman == 1:
            self.current_twist.twist.linear.x = left_y * 0.02   
            self.current_twist.twist.linear.y = left_x * 0.02   
            self.current_twist.twist.linear.z = right_y * 0.02  
            self.current_twist.twist.angular.z = right_x * 0.1          
            self.current_twist.twist.angular.y = dpad_up_down * 0.1    
            self.current_twist.twist.angular.x = dpad_left_right * 0.1
        else:
            self.current_twist.twist.linear.x = 0.0
            self.current_twist.twist.linear.y = 0.0
            self.current_twist.twist.linear.z = 0.0
            self.current_twist.twist.angular.x = 0.0
            self.current_twist.twist.angular.y = 0.0
            self.current_twist.twist.angular.z = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = JoyToServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.arduino:
            node.arduino.write("0\n".encode('utf-8'))
            node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()