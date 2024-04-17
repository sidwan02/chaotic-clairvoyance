# https://batulaiko.medium.com/how-to-pitch-shift-in-python-c59b53a84b6d
# https://github.com/jiaaro/pydub/issues/160#issuecomment-497953546

import numpy as np
import time
import random
from pydub import AudioSegment
from pydub.playback import _play_with_simpleaudio
from numpy.random import uniform

filename = 'violin4.wav'

sound = AudioSegment.from_file(filename, format=filename[-3:])

def randomOctave():
    return random.uniform(-1.5, 1.5)

def randomTime():
    return random.uniform(1.0,3.0)

for i in range(20):
#def playNote():
    octave_shift = randomOctave()
    new_sample_rate = int(sound.frame_rate * (2.0 ** octave_shift))
    hipitch_sound = sound._spawn(sound.raw_data, overrides={'frame_rate': new_sample_rate})
    hipitch_sound = hipitch_sound.set_frame_rate(44100)
    p = _play_with_simpleaudio(hipitch_sound)
    print("played sound")
    time.sleep(randomTime())
    p.stop()