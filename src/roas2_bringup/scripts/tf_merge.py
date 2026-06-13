#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)


class Tfmerge(Node):
    def __init__(self):
        super().__init__('tf_merge_node')

        self.tf_sub = self.create_subscription(
            TFMessage,
            '/j100_0915/tf',   
            self.tf_cb,
            100,
        )
        self.tf_pub = self.create_publisher(
            TFMessage,
            '/tf',             
            100,
        )

        qos_static = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.tf_static_sub = self.create_subscription(
            TFMessage,
            '/j100_0915/tf_static',  
            self.tf_static_cb,
            qos_profile=qos_static,
        )
        self.tf_static_pub = self.create_publisher(
            TFMessage,
            '/tf_static',            
            qos_profile=qos_static,
        )

        self.get_logger().info(
            'TF Relay started: /j100_0915/tf → /tf, /j100_0915/tf_static → /tf_static'
        )

    def tf_cb(self, msg: TFMessage):
        self.tf_pub.publish(msg)

    def tf_static_cb(self, msg: TFMessage):
        self.tf_static_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Tfmerge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

