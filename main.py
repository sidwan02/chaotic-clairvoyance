import pygame
import numpy as np
from pygame.locals import *

# Initialize Pygame
pygame.mixer.pre_init(44100, -16, 2, 512)
pygame.init()

# Constants
MIXING_RATE = 44100  # Sampling frequency in Hertz

# Load sounds
sound1 = pygame.mixer.Sound('violin.wav')
sound2 = pygame.mixer.Sound('sound2.wav')

# Convert to arrays for mixing
sound_array1 = pygame.sndarray.array(sound1)
sound_array2 = pygame.sndarray.array(sound2)

# Main mixing function
def dynamic_mix(sound_arr1, sound_arr2, mix_ratio):
    '''
    Dynamically mixes two sound arrays.
    :param sound_arr1: Numpy array of the first sound
    :param sound_arr2: Numpy array of the second sound
    :param mix_ratio: Ratio for mixing where 1 is only sound_arr1 and 0 is only sound_arr2.
    :return: Resulting mixed sound as a Numpy array
    '''
    arr1_mix = sound_arr1 * mix_ratio
    arr2_mix = sound_arr2 * (1 - mix_ratio)

    print(arr1_mix.shape)
    print(arr2_mix.shape)
    print("Checkpoint 1")

    arr2_mix = np.resize(arr2_mix, arr1_mix.shape)

    print(arr1_mix.shape)
    print(arr2_mix.shape)
    print("Checkpoint 2")
    
    return_val = arr1_mix + arr2_mix
    print("Checkpoint 3")
    return return_val.astype(sound_arr1.dtype)

# Event loop
running = True
mix_val = .5  # Start with an even mix

while running:
    # Check for quit event
    for event in pygame.event.get():
        if event.type == QUIT:
            running = False
    
    # Update mix_val based on real-time input or other conditions
    mix_val = (mix_val + 0.01) % 1  # Simple example to change mix over time
    
    # Mix sounds dynamically
    mixed_sound_array = dynamic_mix(sound_array1, sound_array2, mix_val)
    print("Success")
    
    # Convert array back to a sound object
    mixed_sound = pygame.mixer.Sound(mixed_sound_array)
    
    # Play the mixed sound
    mixed_sound.play()

    # Small delay to prevent overwhelming the CPU
    pygame.time.delay(100)

# Quit Pygame
pygame.quit()