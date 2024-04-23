import pygame
import time

# # https://stackoverflow.com/questions/42393916/how-can-i-play-multiple-sounds-at-the-same-time-in-pygame
pygame.mixer.init()

start_ns = time.time_ns()

pygame.mixer.set_num_channels(10)  # default is 8

pygame.mixer.Channel(0).play(pygame.mixer.Sound("./sounds/guitar.wav"))

while True:
    if (time.time_ns() - start_ns) / 1e9 > 3.453:
        print("in here")
        pygame.mixer.Channel(1).play(pygame.mixer.Sound("./sounds/violin1.wav"))
        # pygame.mixer.Channel(1).pause()
        # pygame.mixer.Channel(1).unpause()

        break

# import pygame

# pygame.mixer.init()
# sound = pygame.mixer.Sound("./sounds/guitar.wav")
# sound.play()
# # Pausing the Sound
# pygame.mixer.pause()
# sound.play()  # Resuming the sound
# pygame.mixer.music.load("./sounds/violin.wav")
# pygame.mixer.music.play(-1)
# # Pausing the Music
# pygame.mixer.music.pause()
# pygame.mixer.music.unpause()  # Resuming the music
time.sleep(40)
