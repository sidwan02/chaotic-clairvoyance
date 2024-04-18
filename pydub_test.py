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

sounds = [
    # "./sounds/guitar.wav",
    # "./sounds/sound1.wav",
    # "./sounds/sound2.wav",
    # "./sounds/violin.wav",
    # "./sounds/violin1.wav",
    "./sounds/violin2.wav",
    # "./sounds/violin3.wav",
]

# midi_pivots = [wav_to_midi(sound, 0, 1000) for sound in sounds]

msgs = load_pickle("./log_positions/log_positions_processed.pkl")

# 30 second clip
final_wav = AudioSegment.silent(duration=30 * 1000)

length_so_far = 0
prev_sound_len = 0

for _ in range(30):
    filename = random.choice(sounds)
    sound = AudioSegment.from_file(filename, format=filename[-3:])
    # proportion of length
    length_prop = np.random.uniform(0.5, 1)
    original_length = len(sound)
    delay_prop = np.random.uniform(0.5, 1)

    # octave_delta = np.random.uniform(-2, 0)
    # major scale
    # major
    octave_delta = random.choice(np.array([0, 2, 4, 5, 7, 9, 11, 12]) / 12)
    # minor
    # octave_delta = random.choice(np.array([0, 2, 3, 5, 7, 8, 10, 12]) / 12)

    new_sample_rate = int(sound.frame_rate * (2.0**octave_delta))
    hipitch_sound = sound._spawn(
        sound.raw_data, overrides={"frame_rate": new_sample_rate}
    )
    hipitch_sound = hipitch_sound.set_frame_rate(44100)

    # song can't be more than 5 seconds
    hipitch_sound = hipitch_sound[0 : min(original_length, 3000) * length_prop]

    hipitch_sound = hipitch_sound.fade_in(500).fade_out(500)

    # print(type(hipitch_sound))

    # playback = sa.play_buffer(
    #     hipitch_sound.raw_data,
    #     num_channels=hipitch_sound.channels,
    #     bytes_per_sample=hipitch_sound.sample_width,
    #     sample_rate=hipitch_sound.frame_rate,
    # )

    # https://stackoverflow.com/questions/43406129/python-overlay-more-than-3-wav-files-end-to-end

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

    # end playback after 3 seconds
    # time.sleep(delay)
    # if we want overlapping sounds, comment this out. But still need a time sleep that's longer than 0 seconds.
    # playback.stop()

# time.sleep(10)

final_wav.export("mixed_sounds.wav", format="wav")

playback = sa.play_buffer(
    final_wav.raw_data,
    num_channels=final_wav.channels,
    bytes_per_sample=final_wav.sample_width,
    sample_rate=final_wav.frame_rate,
)

time.sleep(len(final_wav) / 100)
playback.stop()
