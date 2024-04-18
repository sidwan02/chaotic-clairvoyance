# https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#write-the-subscriber-node

import rclpy
from rclpy.node import Node
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

        # TODO: this doesn't really work since we need to append to the existing array rather than add a new thing
        # Might want to change this to be a list of arrays, and then work from there.
        # If you change this, also change the load_pickle function to reflect the change.
        for drone_tf in msg.transforms:
            positions.append(
                (
                    drone_tf.child_frame_id,
                    drone_tf.header.stamp.sec,
                    drone_tf.header.stamp.nanosec,
                    drone_tf.transform.translation.x,
                    drone_tf.transform.translation.y,
                    drone_tf.transform.translation.z,
                )
            )

        afile = open("log_positions_processed.pkl", "ab+")
        pickle.dump(positions, afile)
        afile.close()


# TODO: figure out how to store this


def main(args=None):
    # clear the pickle file
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


if __name__ == "__main__":
    main()
    # load_pickle()
