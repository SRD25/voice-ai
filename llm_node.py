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
            10
        )

        self.publisher = self.create_publisher(
            String,
            'robot_action',
            10
        )

        self.get_logger().info("LLM Node Started Successfully")


    def listener_callback(self, msg):

        prompt = msg.data
        self.get_logger().info(f"Received prompt: {prompt}")

        try:
            response = requests.post(
                "http://localhost:11434/api/generate",
                json={
                    "model": "llama3",
                    "prompt": prompt,
                    "stream": False
                },
                timeout=60
            )

            if response.status_code != 200:
                self.get_logger().error(
                    f"Ollama HTTP error: {response.status_code}"
                )
                return

            data = response.json()

            if "response" not in data:
                self.get_logger().error("No 'response' field from Ollama")
                return

            result = data["response"]

            self.get_logger().info(f"LLM response: {result}")

            action_msg = String()
            action_msg.data = result
            self.publisher.publish(action_msg)

            self.get_logger().info("Published response to /robot_action")

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Request failed: {str(e)}")

        except Exception as e:
            self.get_logger().error(f"Unexpected error: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
