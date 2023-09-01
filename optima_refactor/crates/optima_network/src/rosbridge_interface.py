import socket
import json
import yaml
from roslibpy import Ros, Topic


class UDPServer:
    """
    UDP Server class that listens for UDP messages and forwards them to ROS topics.

    Args:
        udp_server_address (tuple): The address (host, port) for the UDP server.
        ros_host (str): The hostname of the ROS server.
        ros_port (int): The port number of the ROS server.
        config_file (str): Path to the YAML configuration file.
    """

    def __init__(self, udp_server_address, ros_host="localhost", ros_port=9090, config_file="configs/ros_config.yaml"):
        self.udp_server_address = udp_server_address
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind(udp_server_address)

        self.ros = Ros(host=ros_host, port=ros_port)
        self.ros.run()
        self.topic_handlers = {}

        # Load configuration from YAML file
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)

        self.load_topics_from_config(config)

    def load_topics_from_config(self, config):
        """
        Load topics from the configuration and add them to topic_handlers.

        Args:
            config (dict): Configuration dictionary loaded from YAML.
        """
        for topic in config['topics']:
            topic_name = topic['name']
            message_type = topic['message_type']
            is_publisher = topic['publish']

            self.add_topic(topic_name, message_type, is_publisher)

    def add_topic(self, topic_name, message_type, is_publisher):
        """
        Add a ROS topic to the topic_handlers dictionary.

        Args:
            topic_name (str): Name of the ROS topic.
            message_type (str): Type of the ROS message.
            is_publisher (bool): True if the server should publish, False if it should subscribe.
        """
        topic = Topic(self.ros, topic_name, message_type)

        if not is_publisher:
            topic.subscribe(self.get_message_handler(topic_name))

        self.topic_handlers[topic_name] = topic if is_publisher else None

    def get_message_handler(self, topic_name):
        def handle_topic_message(message):
            """
            Handler function for receiving messages from ROS topics and forwarding to UDP clients.
            """
            udp_message = json.dumps({"topic": topic_name, "content": message["data"]}).encode()
            for handler in self.topic_handlers.values():
                if handler is not None and handler.is_publishing:
                    self.udp_socket.sendto(udp_message, self.udp_server_address)
                    print(f"Published to UDP: {udp_message}")
        return handle_topic_message

    def start(self):
        """
        Start the UDP server to listen for messages and forward to ROS topics.
        """
        try:
            while True:
                try:
                    data, _ = self.udp_socket.recvfrom(1024)
                    if not data:
                        continue

                    received_message = json.loads(data.decode())

                    for topic_name, handler in self.topic_handlers.items():
                        if handler is not None:
                            ros_message = {'data': received_message['content']}
                            handler.publish(ros_message)
                            print(f"Published to {topic_name}: {ros_message}")

                except json.JSONDecodeError:
                    print("Received invalid JSON data")
                except Exception as e:
                    print(f"An error occurred: {e}")

        except KeyboardInterrupt:
            pass
        finally:
            self.udp_socket.close()
            self.ros.terminate()


if __name__ == "__main__":
    config_file = 'config/ros_config.yaml'

    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    udp_server_address = (config['udp_server']['host'], config['udp_server']['port'])
    ros_host = config['ros_server']['host']
    ros_port = config['ros_server']['port']

    udp_server = UDPServer(udp_server_address, ros_host, ros_port, config_file)
    udp_server.start()
