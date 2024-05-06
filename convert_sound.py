from pydub import AudioSegment
import simpleaudio as sa
import time
from pydub.playback import play
import soundfile as sf

filenames = [
    # "./sounds/violin2.wav",
    # "./wavfiles/shakuhachi.wav",
    # "./wavfiles/shakuhachi.wav",
    # "./wavfiles/spirit_cello_cropped.wav",
    # "./wavfiles/spirit_cello_cropped.wav",
    # "./wavfiles/guitar_3rd_string_open_e.wav",
    # "./wavfiles/orch-002-boom.wav",
    "./wavfiles/windchime1.wav",
    # "./wavfiles/wind-chime-gamelan-gong-a.wav",  # needs attribution
    # "./wavfiles/large-wind-chime-2-eb575.wav",
    # "./wavfiles/synth-pad-molten-iron-in-f3.wav",
    # "./wavfiles/hand-crank-music-box-g5-note.flac",
    # "./wavfiles/orch-002-boom_16bd.wav",
    # "./wavfiles/windchime1_16bd.wav",
    # "./wavfiles/wind-chime-gamelan-gong-a_16bd.wav",  # needs attribution
    # "./wavfiles/large-wind-chime-2-eb575_16bd.wav",
    # "./wavfiles/synth-pad-molten-iron-in-f3_16bd.wav",
    # "./wavfiles/hand-crank-music-box-g5-note_16bd.flac",
]


# print(len(sounds))

# midi_original = [wav_to_midi(fn, 0, 1000) for fn in filenames]

# sounds_normalized = [
#     change_semitones(sound, key_to_midi[key] - midi + oc_del * 12, 5 * 1000)
#     for sound, midi, oc_del in zip(sounds, midi_original, base_octave_delta)
# ]

# print("normie: ", len(sounds_normalized))

# https://github.com/jiaaro/pydub/issues/90
# https://github.com/jiaaro/pydub/blob/master/API.markdown#audiosegmentdbfs
# sounds_normalized = [
#     match_target_amplitude(sound, -40) for sound in sounds_normalized
# ]

# test the sounds

for fn in filenames:
    data, samplerate = sf.read(fn)
    a, b, c = fn.rpartition(".")  # split the filename
    # bd i.e. bit depth
    sf.write(a + "_16bd" + b + c, data, samplerate, subtype="PCM_16")

# sounds = [
#     AudioSegment.from_file(filename, format=filename.split(".")[-1])
#     for filename in filenames
# ]


# for sound in sounds:
#     # playback = sa.play_buffer(
#     #     sound.raw_data,
#     #     num_channels=sound.channels,
#     #     bytes_per_sample=sound.sample_width,
#     #     sample_rate=sound.frame_rate,
#     # )

#     # time.sleep(len(sound) / 1000)
#     # playback.stop()
#     play(sound)

# raise Exception("stop here")
