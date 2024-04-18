# https://batulaiko.medium.com/how-to-pitch-shift-in-python-c59b53a84b6d
# https://github.com/jiaaro/pydub/issues/160#issuecomment-497953546


from log_positions.load_pos import load_pickle
from pitch_change.find_freq import wav_to_midi

import random
import numpy as np
from pydub import AudioSegment

# from pydub.playback import _play_with_simpleaudio
import simpleaudio as sa
import time

sounds = ["./sounds/sound1.wav", "./sounds/violin2.wav"]
midi_pivots = [wav_to_midi(sound, 0, 1000) for sound in sounds]


msgs = load_pickle("./log_positions/log_positions_processed.pkl")
# print(msgs)
# raise Exception("Stop here.")


# octaves = 0.5
# for octaves in np.linspace(-1, 1, 21):
for _ in range(10):
    filename = random.choice(sounds)
    sound = AudioSegment.from_file(filename, format=filename[-3:])
    delay = random.uniform(0, 3)

    octave_delta = np.random.uniform(-3, 1)

    new_sample_rate = int(sound.frame_rate * (2.0**octave_delta))
    hipitch_sound = sound._spawn(
        sound.raw_data, overrides={"frame_rate": new_sample_rate}
    )
    hipitch_sound = hipitch_sound.set_frame_rate(44100)

    hipitch_sound = hipitch_sound[0 : 3 * 1000]

    hipitch_sound = hipitch_sound.fade_in(500).fade_out(500)

    playback = sa.play_buffer(
        hipitch_sound.raw_data,
        num_channels=hipitch_sound.channels,
        bytes_per_sample=hipitch_sound.sample_width,
        sample_rate=hipitch_sound.frame_rate,
    )

    # end playback after 3 seconds
    time.sleep(delay)
    # if we want overlapping sounds, comment this out. But still need a time sleep that's longer than 0 seconds.
    # playback.stop()

time.sleep(10)
