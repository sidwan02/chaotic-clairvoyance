# https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#write-the-subscriber-node

import rclpy
from rclpy.node import Node
import json
from tf2_msgs.msg import TFMessage
import pickle

# from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__("minimal_subscriber")
        self.subscription = self.create_subscription(
            TFMessage, "tf", self.listener_callback, 1
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)
        # msg_json = json.load(msg)
        # print(msg_json)

        afile = open("log_positions.pkl", "ab+")
        pickle.dump(msg, afile)
        afile.close()


# TODO: figure out how to store this


def main(args=None):
    afile = open("log_positions.pkl", "wb")
    afile.close()

    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


def load_and_print_pickle():
    msgs = []
    try:
        afile = open("log_positions.pkl", "rb")
        while 1:
            try:
                msg = pickle.load(afile)
                msgs.append(msg)
            except EOFError:
                break
        afile.close()
    except FileNotFoundError:
        print("Pickle file not found.")

    print(msgs)
    print(len(msgs))


if __name__ == "__main__":
    # main()
    load_and_print_pickle()
