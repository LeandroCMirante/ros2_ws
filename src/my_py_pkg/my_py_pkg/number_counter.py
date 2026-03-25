#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool

class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.counter_ = 0
        self.number_count_publisher = self.create_publisher(Int64, "number_count", 10)
        self.number_subscriber_ = self.create_subscription(Int64, "number", self.number_callback, 10)
        self.reset_counter_service_ = self.create_service(SetBool, "reset_counter", self.reset_counter_callback)

    def reset_counter_callback(self, request, response):
        if request.data:
            self.counter_ = 0
            response.success = True
            response.message = "Counter reset successfully."
            self.get_logger().info("Counter reset to 0.")
        else:
            response.success = False
            response.message = "Counter not reset."
            self.get_logger().info("Counter not reset.")
        return response

    def number_callback(self, msg: Int64):
        self.counter_ += msg.data
        count_msg = Int64()
        count_msg.data = self.counter_
        self.number_count_publisher.publish(count_msg)
        self.get_logger().info(f"Received: {msg.data}, Count: {self.counter_}")

def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
