import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ollama
from duckduckgo_search import DDGS


class RobotAI(Node):

    def __init__(self):
        super().__init__('robot_ai')

        self.subscription = self.create_subscription(
            String,
            'robot_question',
            self.question_callback,
            10)

        self.publisher = self.create_publisher(
            String,
            'robot_answer',
            10)

        self.get_logger().info("Robot AI Node Started")


    def search_internet(self, query):

        context = ""

        with DDGS() as ddgs:
            results = ddgs.text(query, max_results=3)

        for r in results:
            context += r["body"] + "\n"

        return context


    def question_callback(self, msg):

        question = msg.data
        self.get_logger().info(f"Question received: {question}")

        internet_data = self.search_internet(question)

        prompt = f"""
Use the following internet data to answer.

{internet_data}

Question: {question}
"""

        response = ollama.chat(
            model="llama3:8b",
            messages=[{"role": "user", "content": prompt}]
        )

        answer = response["message"]["content"]

        out = String()
        out.data = answer

        self.publisher.publish(out)

        self.get_logger().info("Answer published")


def main(args=None):

    rclpy.init(args=args)

    node = RobotAI()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()