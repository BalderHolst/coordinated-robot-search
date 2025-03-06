import rclpy
from rclpy.node import Node
from search_agent_msgs.msg import AgentMessage

class AgentMsgRelay(Node):
    def __init__(self):
        super().__init__('agent_msg_relay')

        self.declare_parameter('global_topic', '')
        global_topic = self.get_parameter('global_topic').get_parameter_value().string_value
        if global_topic == '':
            self.get_logger().error('Namespace parameter not set')
            return

        self.declare_parameter('robot_namespace', '')
        robot_namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value
        if robot_namespace == '':
            self.get_logger().error('Robot namespace parameter not set')
            return


        self.send_topic = f'/{robot_namespace}/msgs_send'
        self.recv_topic = f'/{robot_namespace}/msgs_recv'
        self.channel_topic = f'/{global_topic}'

        self.get_logger().info(f"Relaying from {self.send_topic} to {self.recv_topic} via {self.channel_topic}")

        self.send_sub = self.create_subscription(
            AgentMessage,
            self.send_topic,
            self.send_callback,
            10
        )

        self.recv_pub = self.create_publisher(
            AgentMessage,
            self.recv_topic,
            10
        )

        self.channel_sub = self.create_subscription(
            AgentMessage,
            self.channel_topic,
            self.channel_callback,
            10
        )

        self.channel_pub = self.create_publisher(
            AgentMessage,
            self.channel_topic,
            10
        )

    def send_callback(self, msg):
        self.channel_pub.publish(msg)

    def channel_callback(self, msg):
        self.recv_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AgentMsgRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
