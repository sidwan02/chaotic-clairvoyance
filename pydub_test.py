# https://batulaiko.medium.com/how-to-pitch-shift-in-python-c59b53a84b6d
# https://github.com/jiaaro/pydub/issues/160#issuecomment-497953546


from log_positions.load_pos import load_pickle
from pitch_change.find_freq import wav_to_midi

from functools import partial

import random
import numpy as np
from pydub import AudioSegment

from collections import defaultdict, deque

import simpleaudio as sa
import time

# TODO: first, modulate all wav files to be middle C. this will help with harmonization better. use the midi_pivots to help with this.
filenames = [
    # "./sounds/guitar.wav",
    "./sounds/sound1.wav",
    # "./sounds/sound2.wav",
    # "./sounds/violin.wav",
    # "./sounds/violin1.wav",
    "./sounds/violin2.wav",
    # "./sounds/violin3.wav",
]
sounds = [
    AudioSegment.from_file(filename, format=filename[-3:]) for filename in filenames
]

key = "C"
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

scale_configs = {
    "major": [0, 2, 4, 5, 7, 9, 11, 12],
    "minor": [0, 2, 3, 5, 7, 8, 10, 12],
}


midi_original = [wav_to_midi(fn, 0, 1000) for fn in filenames]


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


sounds_normalized = [
    change_semitones(sound, key_to_midi[key] - midi, 5 * 1000)
    for sound, midi in zip(sounds, midi_original)
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

msgs = load_pickle("./log_positions/log_positions_processed.pkl")

# 30 second clip
final_wav = AudioSegment.silent(duration=30 * 1000)

length_so_far = 0
prev_sound_len = 0


def allocate_sounds():
    drone_names = set()
    for msg in msgs:
        drone_name = msg[0]

        if drone_name in drone_names:
            break

        drone_names.add(drone_name)

    drone_sounds = {}
    available_sounds = list(sounds_normalized)
    random.shuffle(available_sounds)

    for drone_name in drone_names:
        drone_sounds[drone_name] = available_sounds.pop()

    return drone_sounds


drone_sounds = allocate_sounds()
print("drone sounds: ", drone_sounds)

# raise Exception("stop here")


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


drone_buffers = defaultdict(lambda: defaultdict(partial(Queue, maxlen=30)))
temp_buffers = defaultdict(lambda: defaultdict(deque))

start_sec = None

for msg in msgs:
    # print(msg)
    drone_name, sec, nsec, x, y, z = msg

    if start_sec is None:
        start_sec = sec

    temp_buffers[drone_name]["x_temp"].append(x)
    temp_buffers[drone_name]["y_temp"].append(y)
    temp_buffers[drone_name]["z_temp"].append(z)

    if sec - start_sec >= 1:
        start_sec = sec

        err = [
            metric_1(temp_buffers[drone_name][temp], temp_buffers[drone_name][temp][0])
            for temp in ["x_temp", "y_temp", "z_temp"]
        ]

        # TODO: make use of these buffers at some point.
        drone_buffers[drone_name]["x"].append(err[0])
        drone_buffers[drone_name]["y"].append(err[1])
        drone_buffers[drone_name]["z"].append(err[2])

        temp_buffers[drone_name]["x_temp"] = deque()
        temp_buffers[drone_name]["y_temp"] = deque()
        temp_buffers[drone_name]["z_temp"] = deque()

        sound = drone_sounds[drone_name]

        print(err)
        # semitone_delta = random.choice(scale_configs[scale])
        semitone_delta = np.mean(err) * 120
        print("semitone delta: ", semitone_delta)

        # proportion of length
        length_prop = np.random.uniform(0.5, 1)

        # song can't be more than 3 seconds
        length = min(len(sound), 3000) * length_prop

        hipitch_sound = change_semitones(sound, semitone_delta, length)
        # https://stackoverflow.com/questions/43406129/python-overlay-more-than-3-wav-files-end-to-end

        delay_prop = np.random.uniform(0.5, 1)

        # start the next sound 3 seconds before the previous sound ends
        insert_time = length_so_far - prev_sound_len * delay_prop
        # print("baba", insert_time)
        final_wav = final_wav.overlay(hipitch_sound, position=insert_time)

        # print(len(hipitch_sound))
        prev_sound_len = len(hipitch_sound)
        # you might overlay a sound that extends the current length, or it might be within the current length if it's short
        length_so_far = max(length_so_far, prev_sound_len + insert_time)
        # print(length_so_far)
        # print(length_so_far + insert_time)

        print(f"inserted sound of duration {prev_sound_len} at position {insert_time}")
        print(f"new total length: {length_so_far}")

final_wav.export("mixed_sounds.wav", format="wav")

playback = sa.play_buffer(
    final_wav.raw_data,
    num_channels=final_wav.channels,
    bytes_per_sample=final_wav.sample_width,
    sample_rate=final_wav.frame_rate,
)

time.sleep(len(final_wav) / 100)
playback.stop()
