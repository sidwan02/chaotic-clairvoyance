# https://batulaiko.medium.com/how-to-pitch-shift-in-python-c59b53a84b6d
# https://github.com/jiaaro/pydub/issues/160#issuecomment-497953546


from log_positions.load_pos import load_pickle
from pitch_change.find_freq import wav_to_midi

from collections import deque
from collections import defaultdict

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

position_history = defaultdict()

x_history = deque([])
y_history = deque([])
z_history = deque([])



for cf_name, sec, nsec, x, y, z in msgs:
    if cf_name == "cf13":
        if len(x_history) == 10:
            x_history.popleft()
            y_history.popleft()
            z_history.popleft()
            
        x_history.append(x)
        y_history.append(y)
        z_history.append(z)
        
        play_sound(sounds[0], np.std(x_history), 0)
    else:
        if len(x_history) == 10:
            x_history.popleft()
            y_history.popleft()
            z_history.popleft()
            
        x_history.append(x)
        y_history.append(y)
        z_history.append(z)
        play_sound(sounds[1], np.std(x_history), 0)
        


def play_sound(filename, octave_delta, duration):
    sound = AudioSegment.from_file(filename, format=filename[-3:])
    
        new_sample_rate = int(sound.frame_rate * (2.0**octaves_delta))
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
        time.sleep(1)
        # if we want overlapping sounds, comment this out. But still need a time sleep that's longer than 0 seconds.
        # playback.stop()
