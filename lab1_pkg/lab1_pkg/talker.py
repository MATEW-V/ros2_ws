import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        
        # Declare parameters with default fallback values
        self.declare_parameter('v', 0.0)
        self.declare_parameter('d', 0.0)
        
        # Create publisher on the 'drive' topic
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        
        # Publish as fast as possible by running a loop timer with a 0-second interval
        self.timer = self.create_timer(0.0, self.timer_callback)

    def timer_callback(self):
        # Fetch current parameter values on every iteration
        v_val = self.get_parameter('v').get_parameter_value().double_value
        d_val = self.get_parameter('d').get_parameter_value().double_value
        
        # Populate the AckermannDriveStamped message
        msg = AckermannDriveStamped()
        msg.drive.speed = v_val
        msg.drive.steering_angle = d_val
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
