import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests

class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')
        self.subscription = self.create_subscription(
            String,
            'robot_prompt',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(String, 'robot_action', 10)

    def listener_callback(self, msg):
        prompt = msg.data

        response = requests.post(
            "http://localhost:11434/api/generate",
            json={
                "model": "llama3",
                "prompt": prompt,
                "stream": False
            }
        )

        result = response.json()["response"]

        action_msg = String()
        action_msg.data = result
        self.publisher.publish(action_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()