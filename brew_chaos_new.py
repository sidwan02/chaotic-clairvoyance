# https://batulaiko.medium.com/how-to-pitch-shift-in-python-c59b53a84b6d
# https://github.com/jiaaro/pydub/issues/160#issuecomment-497953546


from log_positions.load_pos import load_pickle
from pitch_change.find_freq import wav_to_midi

import pygame

from itertools import chain

from functools import partial
import bisect

import random
import numpy as np
from pydub import AudioSegment

from collections import defaultdict, deque

import simpleaudio as sa
import time


def change_semitones(sound, semitones, length=-1):
    octave_delta = semitones / 12
    new_sample_rate = int(sound.frame_rate * (2.0**octave_delta))
    hipitch_sound = sound._spawn(
        sound.raw_data, overrides={"frame_rate": new_sample_rate}
    )
    hipitch_sound = hipitch_sound.set_frame_rate(44100)

    hipitch_sound = hipitch_sound[0:length]

    hipitch_sound = hipitch_sound.fade_in(500).fade_out(500)

    return hipitch_sound


def allocate_sounds(sounds_normalized, msg_transforms):
    # print(msg_transforms)
    drone_names = [drone_tf[0] for drone_tf in msg_transforms]
    # print(drone_names)

    drone_sounds = {}
    available_sounds = list(sounds_normalized)
    print(available_sounds)
    random.shuffle(available_sounds)

    for drone_name in drone_names:
        drone_sounds[drone_name] = available_sounds.pop()

    return drone_sounds


class Queue:
    def __init__(self, maxlen=20):
        self.q = deque()
        self.maxlen = maxlen

    def append(self, item):
        if len(self.q) == self.maxlen:
            self.q.popleft()
        self.q.append(item)

    def get_std(self):
        return np.std(self.q)

    def get_mean(self):
        return np.mean(self.q)

    def __str__(self):
        return str(self.q)


def metric_1(arr, pivot):
    return np.mean(np.array(arr) - pivot)


def bias_semitone(semitone, scale):
    scale_config = scale_configs[scale]
    i = bisect.bisect_left(scale_config, semitone)
    return (
        scale_config[i]
        if abs(scale_config[i] - semitone) < abs(scale_config[i - 1] - semitone)
        else scale_config[i - 1]
    )


scale_configs = {
    "major": list(
        chain.from_iterable(
            [
                np.array([0, 2, 4, 5, 7, 9, 11, 12]) + offset * 12
                for offset in range(-4, 4)
            ]
        )
    ),
    "minor": list(
        chain.from_iterable(
            [
                np.array([0, 2, 3, 5, 7, 8, 10, 12]) + offset * 12
                for offset in range(-4, 4)
            ]
        )
    ),
}


def setup_processing():
    # TODO: first, modulate all wav files to be middle C. this will help with harmonization better. use the midi_pivots to help with this.
    filenames = [
        # "./sounds/guitar.wav",
        # "./sounds/sound1.wav",
        # "./sounds/sound2.wav",
        # "./sounds/violin.wav",
        "./sounds/violin2.wav",
        "./sounds/violin2.wav",
        # "./sounds/violin3.wav",
    ]
    sounds = [
        AudioSegment.from_file(filename, format=filename[-3:]) for filename in filenames
    ]

    # whether the normalized sound starts of in octave 4 (eg, as middle C), or in octave 5 (eg, as C5)
    base_octave_delta = [0, 0]

    key = "E"
    scale = "minor"

    # these are notes in the 4th octave
    key_to_midi = {
        "C": 60,
        "C♯": 61,
        "D♭": 61,
        "D": 62,
        "D♯": 63,
        "E♭": 63,
        "E": 64,
        "F": 65,
        "F♯": 66,
        "G♭": 66,
        "G": 67,
        "G♯": 68,
        "A♭": 68,
        "A": 69,
        "A♯": 70,
        "B♭": 70,
        "B": 71,
    }

    possible_modes = ["generate_wav", "play_realtime"]

    set_modes = set(possible_modes)
    # set_modes = set(["play_realtime"])

    # we build the major and minor semitone configurations over 8 octaves. There are duplicate semitone values at each octave boundary but that's fine since we use this later in the bisect operation to bias semitones generated by the drones to semitones on the major/minor key.

    midi_original = [wav_to_midi(fn, 0, 1000) for fn in filenames]

    sounds_normalized = [
        change_semitones(sound, key_to_midi[key] - midi + oc_del * 12, 5 * 1000)
        for sound, midi, oc_del in zip(sounds, midi_original, base_octave_delta)
    ]

    # test the sounds
    """
    for sound in sounds_normalized:
        playback = sa.play_buffer(
            sound.raw_data,
            num_channels=sound.channels,
            bytes_per_sample=sound.sample_width,
            sample_rate=sound.frame_rate,
        )

        time.sleep(len(sound) / 1000)
        playback.stop()

    # raise Exception("stop here")
    """

    if "play_realtime" in set_modes:
        pygame.mixer.init()
        pygame.mixer.set_num_channels(8)

        start_ns = None

    msgs = load_pickle("./log_positions/log_positions_processed.pkl")

    # print(msgs)

    # TODO: once we migrate to the new format, remove these comments
    # for some reson this makes everything strings

    msgs = np.reshape(msgs, (-1, 2, 6))

    # print(msgs[10])
    # raise Error("stop here")

    # 60 second clip
    final_wav = AudioSegment.silent(duration=60 * 1000)

    drone_sounds = allocate_sounds(sounds_normalized, msgs[0])
    # print("drone sounds: ", drone_sounds)

    # pygame.mixer.Channel(0).play(pygame.mixer.Sound(drone_sounds["cf16"].raw_data))
    # time.sleep(5)
    # pygame.mixer.Channel(0).play(pygame.mixer.Sound(drone_sounds["cf13"].raw_data))

    # time.sleep(5)

    # raise Exception("stop here")

    drone_buffers = defaultdict(lambda: defaultdict(partial(Queue, maxlen=30)))
    temp_buffers = defaultdict(lambda: defaultdict(deque))

    start_sec = None

    length_so_far = 0
    prev_sound_len = 0

    cur_channel = 0

    for msg in msgs:
        (
            start_sec,
            start_ns,
            length_so_far,
            prev_sound_len,
            cur_channel,
            drone_buffers,
            temp_buffers,
            final_wav,
            drone_sounds,
        ) = process_positions(
            msg,
            scale,
            set_modes,
            start_sec,
            start_ns,
            length_so_far,
            prev_sound_len,
            cur_channel,
            drone_buffers,
            temp_buffers,
            final_wav,
            drone_sounds,
        )

    if "generate_wav" in set_modes:
        final_wav.export("mixed_sounds.wav", format="wav")


# process drone positions corresponding to single timestep
def process_positions(
    msg_transforms,
    scale,
    set_modes,
    start_sec,
    start_ns,
    length_so_far,
    prev_sound_len,
    cur_channel,
    drone_buffers,
    temp_buffers,
    final_wav,
    drone_sounds,
):
    # print("msg_transforms: ", msg_transforms)

    # once we change the data format, we won't need to ccast anymore
    sec = int(msg_transforms[0][1])

    if start_sec is None:
        start_sec = sec

    # print("sec: ", sec)
    # print("start_sec: ", start_sec)

    should_play_flag = (sec - start_sec) >= 1

    for drone_tf in msg_transforms:
        # print(msg)
        # print("drone_tf: ", drone_tf)
        drone_name, sec, nsec, x, y, z = drone_tf

        # TODO: remove these casts when we migrate to the new format
        sec = int(sec)
        nsec = int(nsec)
        x = float(x)
        y = float(y)
        z = float(z)

        temp_buffers[drone_name]["x_temp"].append(x)
        temp_buffers[drone_name]["y_temp"].append(y)
        temp_buffers[drone_name]["z_temp"].append(z)

        # print(temp_buffers[drone_name]["x_temp"])
        # print(temp_buffers[drone_name]["y_temp"])
        # print(temp_buffers[drone_name]["z_temp"])

        if not should_play_flag:
            continue

        print("=====================================")
        print("drone name: ", drone_name)
        print("=====================================")

        err = [
            metric_1(temp_buffers[drone_name][temp], temp_buffers[drone_name][temp][0])
            for temp in ["x_temp", "y_temp", "z_temp"]
        ]

        print("err: ", err)

        # TODO: make use of these buffers at some point.
        drone_buffers[drone_name]["x"].append(err[0])
        drone_buffers[drone_name]["y"].append(err[1])
        drone_buffers[drone_name]["z"].append(err[2])

        temp_buffers[drone_name]["x_temp"] = deque()
        temp_buffers[drone_name]["y_temp"] = deque()
        temp_buffers[drone_name]["z_temp"] = deque()

        sound = drone_sounds[drone_name]
        # print("=====================================")
        # print("drone name: ", drone_name)
        # print("=====================================")

        print(err)
        # semitone_delta = random.choice(scale_configs[scale])
        semitone_delta = np.mean(err) * 120
        print("semitone delta: ", semitone_delta)

        semitone_delta = bias_semitone(semitone_delta, scale)
        print("semitone delta after bias: ", semitone_delta)

        # proportion of length
        length_prop = np.random.uniform(0.5, 1)

        # clip can't be more than 3 seconds
        length = min(len(sound), 3000) * length_prop

        hipitch_sound = change_semitones(sound, semitone_delta, length)

        # get the insert time of the next sound
        # ==============
        # TODO: clear up the delay prop logic such that insert time is any time between length_so_far and beginning of the previous sound.
        delay_prop = np.random.uniform(0.5, 1)

        # start the next sound 3 seconds before the previous sound ends
        insert_time = length_so_far - prev_sound_len * delay_prop

        # print(len(hipitch_sound))
        prev_sound_len = len(hipitch_sound)
        # you might overlay a sound that extends the current length, or it might be within the current length if it's short
        length_so_far = max(length_so_far, prev_sound_len + insert_time)
        # ==============
        print("insert time: ", insert_time / 1000)
        if start_ns is not None:
            print("elapsed time: ", (time.time_ns() - start_ns) / 1e9)

        if "play_realtime" in set_modes:
            while True:
                if start_ns is None or ((time.time_ns() - start_ns) / 1e9) >= (
                    insert_time / 1000
                ):
                    # start_ns = time.time_ns()
                    print("inserting sound")
                    pygame.mixer.Channel(cur_channel).play(
                        pygame.mixer.Sound(hipitch_sound.raw_data)
                    )

                    if start_ns is None:
                        start_ns = time.time_ns()

                    cur_channel = (cur_channel + 1) % 8

                    break

        if "generate_wav" in set_modes:
            # https://stackoverflow.com/questions/43406129/python-overlay-more-than-3-wav-files-end-to-end

            # print("baba", insert_time)
            final_wav = final_wav.overlay(hipitch_sound, position=insert_time)

            # print(length_so_far)
            # print(length_so_far + insert_time)

            print(
                f"inserted sound of duration {prev_sound_len} at position {insert_time}"
            )
            print(f"new total length: {length_so_far}")

    # raise Exception("stop here")

    # we could play the final wav file after generation
    # if "play_wav" in set_modes:
    #     playback = sa.play_buffer(
    #         final_wav.raw_data,
    #         num_channels=final_wav.channels,
    #         bytes_per_sample=final_wav.sample_width,
    #         sample_rate=final_wav.frame_rate,
    #     )

    #     time.sleep(len(final_wav) / 100)
    #     playback.stop()

    # time.sleep(10)

    if should_play_flag:
        start_sec = sec

    return (
        start_sec,
        start_ns,
        length_so_far,
        prev_sound_len,
        cur_channel,
        drone_buffers,
        temp_buffers,
        final_wav,
        drone_sounds,
    )


if __name__ == "__main__":
    setup_processing()
