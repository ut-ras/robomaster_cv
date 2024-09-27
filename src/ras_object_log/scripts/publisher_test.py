import rclpy
from rclpy.node import Node
from stampede_msgs.msg import BoundingBox, ObjectLogInput

class ObjectLogInputPublisher(Node):
    def __init__(self):
        super().__init__('object_log_input_publisher')
        self.publisher_ = self.create_publisher(ObjectLogInput, '/object_log_input', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_message)
        
    def publish_message(self):
        msg = ObjectLogInput()
        box1 = BoundingBox()
        box1.center_x = 1.0
        box1.center_y = 2.0
        box1.width = 3.0
        box1.height = 4.0
        box1.depth = 5.0
        
        box2 = BoundingBox()
        box2.center_x = 6.0
        box2.center_y = 7.0
        box2.width = 8.0
        box2.height = 9.0
        box2.depth = 10.0
        
        msg.boxes = [box1, box2]
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectLogInputPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
