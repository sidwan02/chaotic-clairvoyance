from pydub import AudioSegment
import numpy as np
import random

def add_tremolo_with_random_volume(input_file, output_file, depth=0.5, frequency=5.0, volume_range=(-10, 10)):
    sound = AudioSegment.from_wav(input_file)

    samples = np.array(sound.get_array_of_samples(), dtype=np.int16)

    time = np.linspace(0, len(samples) / sound.frame_rate, len(samples))
    tremolo = 0.5 * depth * np.sin(2 * np.pi * frequency * time)

    tremolo_samples = (1.0 + tremolo) * samples

    tremolo_samples = np.clip(tremolo_samples, -32768, 32767).astype(np.int16)
    tremolo_samples = tremolo_samples.reshape(-1, sound.channels)
    sound_with_tremolo = AudioSegment(tremolo_samples.tobytes(), frame_rate=sound.frame_rate, sample_width=sound.sample_width, channels=sound.channels)
    volume_adjustments = [random.randint(*volume_range) for _ in range(3)]

    sliced_audio_with_volume = AudioSegment.empty()
    for i in range(len(volume_adjustments)):
        segment = sound_with_tremolo[i * len(sound_with_tremolo) // len(volume_adjustments): (i + 1) * len(sound_with_tremolo) // len(volume_adjustments)]
        segment = segment.apply_gain(volume_adjustments[i])
        sliced_audio_with_volume += segment

    sliced_audio_with_volume.export(output_file, format="wav")

input_file = "test3.wav"
output_file = "output.wav"
depth = 0.6  # Depth of tremolo effect (0.0 to 1.0)
frequency = 5.0  # Frequency of tremolo modulation (in Hz)
volume_range = (-10, 10)  # Range of volume adjustments (in dB)

add_tremolo_with_random_volume(input_file, output_file, depth, frequency, volume_range)
