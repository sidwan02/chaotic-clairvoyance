# https://batulaiko.medium.com/how-to-pitch-shift-in-python-c59b53a84b6d
# https://github.com/jiaaro/pydub/issues/160#issuecomment-497953546

import numpy as np
from pydub import AudioSegment
from pydub.playback import _play_with_simpleaudio
import simpleaudio as sa
import time
import random

filename = "./sounds/sound1.wav"
sound = AudioSegment.from_file(filename, format=filename[-3:])

octaves = 0.5

def randomOctave():
    random.uniform(-1.5, 1.5)

for octaves in np.linspace(-1, 1, 21):
    new_sample_rate = int(sound.frame_rate * (2.0**octaves))
    hipitch_sound = sound._spawn(
        sound.raw_data, overrides={"frame_rate": new_sample_rate}
    )
    hipitch_sound = hipitch_sound.set_frame_rate(44100)

    hipitch_sound = hipitch_sound[0 : 3 * 1000]

    # playback = _play_with_simpleaudio(audio_segment)

    hipitch_sound = hipitch_sound.fade_in(500).fade_out(500)

    playback = sa.play_buffer(
        hipitch_sound.raw_data,
        num_channels=hipitch_sound.channels,
        bytes_per_sample=hipitch_sound.sample_width,
        sample_rate=hipitch_sound.frame_rate,
    )

    # end playback after 3 seconds
    time.sleep(3)
    playback.stop()