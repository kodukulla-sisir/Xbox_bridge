import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from rclpy.qos import qos_profile_system_default

class JoyToServo(Node):
    def __init__(self):
        super().__init__('joy_to_servo')
        
        self.subscription = self.create_subscription(
            Joy, 
            '/joy', 
            self.joy_callback, 
            10
        )
        
        self.publisher = self.create_publisher(
            TwistStamped, 
            '/servo_server/delta_twist_cmds', 
            qos_profile_system_default
        )
        
        self.frame_id = 'link_base' 

    def joy_callback(self, msg):
        twist = TwistStamped()
        
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'link_base'

        def filter_stick(value):
            return 0.0 if abs(value) < 0.1 else value
        
        # Controller mapping
        deadman = msg.buttons[5]
        left_y = filter_stick(msg.axes[1])
        left_x = filter_stick(msg.axes[0])
        right_y = filter_stick(msg.axes[4])
        right_x = filter_stick(msg.axes[3])
        dpad_up_down = filter_stick(msg.axes[7])
        dpad_left_right = filter_stick(msg.axes[6])

        if deadman == 1: # Movement conditional
            # Linear
            twist.twist.linear.x = left_y * 0.02   
            twist.twist.linear.y = left_x * 0.02   
            twist.twist.linear.z = right_y * 0.02  

            # Angular
            twist.twist.angular.z = right_x * 0.1          
            twist.twist.angular.y = dpad_up_down * 0.1    
            twist.twist.angular.x = dpad_left_right * 0.1
            
            if left_y != 0.0 or left_x != 0.0:
                self.get_logger().info(f"Speed -> X: {twist.twist.linear.x:.3f}, Y: {twist.twist.linear.y:.3f}")
                
        else:
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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()