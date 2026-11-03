import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class RelayNode(Node):
    def __init__(self):
        super().__init__('relay')
        
        # Subscribe to incoming data on 'drive'
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            'drive',
            self.listener_callback,
            10
        )
        
        # Publish relay modified data to 'drive_relay'
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive_relay', 10)

    def listener_callback(self, msg):
        # Take incoming speed/steering measurements and triple them
        relayed_msg = AckermannDriveStamped()
        relayed_msg.drive.speed = msg.drive.speed * 3.0
        relayed_msg.drive.steering_angle = msg.drive.steering_angle * 3.0
        
        self.publisher_.publish(relayed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

