# https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#write-the-subscriber-node

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Joy
import pickle
from brew_chaos import *
from pydub import AudioSegment

import pygame

# from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__("minimal_subscriber")
        self.subscription = self.create_subscription(
            TFMessage, "tf", self.listener_callback, 1
        )
        self.joy_subscription = self.create_subscription(
            Joy, "joy", self.joy_callback, 1
        )
        self.subscription  # prevent unused variable warning

        self.setup_processing()

    def setup_processing(self):
        # TODO: first, modulate all wav files to be middle C. this will help with harmonization better. use the midi_pivots to help with this.

        # whether the normalized sound starts of in octave 4 (eg, as middle C), or in octave 5 (eg, as C5)
        self.base_octave_delta = [0, 0]
        self.key = "E"
        self.scale = "minor"
        possible_modes = ["generate_wav", "play_realtime"]
        self.set_modes = set(possible_modes)
        # self.set_modes = set(["play_realtime"])

        self.sounds_normalized = None

        if "play_realtime" in self.set_modes:
            pygame.mixer.init()
            pygame.mixer.set_num_channels(8)

            self.realtime_play_start_ns = None

        # print(msgs)

        # 60 second clip
        self.final_wav = AudioSegment.silent(duration=100 * 1000)

        self.drone_sounds = None
        # print("drone sounds: ", self.drone_sounds)

        # pygame.mixer.Channel(0).play(pygame.mixer.Sound(self.drone_sounds["cf16"].raw_data))
        # time.sleep(5)
        # pygame.mixer.Channel(0).play(pygame.mixer.Sound(self.drone_sounds["cf13"].raw_data))

        # time.sleep(5)

        # raise Exception("stop here")

        self.drone_buffers = defaultdict(lambda: defaultdict(partial(Queue, maxlen=30)))
        self.temp_buffers = defaultdict(lambda: defaultdict(deque))

        self.tf_prev_trigger_sec = None

        self.length_so_far = 0
        self.prev_sound_len = 0

        self.cur_channel = 0

        self.tf_start_sec = None

        self.play_queue = deque()

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg)
        # msg_json = json.load(msg)
        # print(msg_json)

        # ============= store the data for later use =============
        afile = open("four_curtain_log_positions.pkl", "ab+")
        pickle.dump(msg, afile)
        afile.close()

        # TODO: this doesn't really work since we need to append to the existing array rather than add a new thing
        # Might want to change this to be a list of arrays, and then work from there.
        # If you change this, also change the load_pickle function to reflect the change.
        positions = []

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

        afile = open("four_curtain_log_positions_processed.pkl", "ab+")
        pickle.dump(positions, afile)
        afile.close()
        print("FILE CLOSED ===================================")

        # =========== live processing =================

        if self.sounds_normalized is None:
            num_cf = len([drone_tf[0] for drone_tf in positions])
            print("num_cf: ", num_cf)
            self.sounds_normalized = generate_normalized_sounds(self.key, num_cf)

        if self.tf_start_sec is None:
            self.tf_start_sec = int(positions[0][1]) + int(positions[0][2]) / 1e9

        if self.drone_sounds is None:
            self.drone_sounds = allocate_sounds(self.sounds_normalized, positions)

        # print("calling process_positions")

        (
            self.tf_prev_trigger_sec,
            self.realtime_play_start_ns,
            self.length_so_far,
            self.prev_sound_len,
            self.cur_channel,
            self.drone_buffers,
            self.temp_buffers,
            self.final_wav,
            self.drone_sounds,
            self.play_queue,
        ) = process_positions(
            positions,
            self.scale,
            self.set_modes,
            self.tf_start_sec,
            self.tf_prev_trigger_sec,
            self.realtime_play_start_ns,
            self.length_so_far,
            self.prev_sound_len,
            self.cur_channel,
            self.drone_buffers,
            self.temp_buffers,
            self.final_wav,
            self.drone_sounds,
            self.play_queue,
        )

        # # for this to be detected, will need to hold down the escape key
        # if keyboard.is_pressed("esc"):
        #     print("Escape key pressed: generating wav")
        #     if "generate_wav" in self.set_modes:
        #         self.final_wav.export("mixed_sounds.wav", format="wav")

    def joy_callback(self, msg):
        # print(msg.buttons[0])
        if msg.buttons[0] == 1:  # keyboard.is_pressed("esc"):
            print("A pressed: generating wav")
            if "generate_wav" in self.set_modes:
                self.final_wav.export("mixed_sounds_live.wav", format="wav")


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
