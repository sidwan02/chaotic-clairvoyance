import numpy as np
from scipy.fft import *
from scipy.io import wavfile
import math

# https://stackoverflow.com/a/66892766


def wav_to_freq(file, start_time, end_time):

    # Open the file and convert to mono
    sr, data = wavfile.read(file)
    if data.ndim > 1:
        data = data[:, 0]
    else:
        pass

    # Return a slice of the data from start_time to end_time
    dataToRead = data[int(start_time * sr / 1000) : int(end_time * sr / 1000) + 1]

    # Fourier Transform
    N = len(dataToRead)
    yf = rfft(dataToRead)
    xf = rfftfreq(N, 1 / sr)

    # Uncomment these to see the frequency spectrum as a plot
    # plt.plot(xf, np.abs(yf))
    # plt.show()

    # Get the most dominant frequency and return it
    idx = np.argmax(np.abs(yf))
    freq = xf[idx]
    return freq


# https://newt.phys.unsw.edu.au/jw/notes.html#:~:text=m%20for%20the%20note%20A4,)%2F12(440%20Hz).
def freq_to_midi(f_m):
    return 12 * math.log2(f_m / 440) + 69


def wav_to_midi(file, start_time, end_time):
    f_m = wav_to_freq(file, start_time, end_time)
    print(f_m)
    return freq_to_midi(f_m)


if __name__ == "__main__":
    file = "./sounds/sound1.wav"
    start_time = 0
    end_time = 1000
    print(wav_to_midi(file, start_time, end_time))
