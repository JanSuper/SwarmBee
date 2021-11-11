"""Allows for Drone Keyboard Control
Created a window that allow the function to be called anywhere inside my project"""
# Copyright (c) 2021 https://github.com/HKagiri/DroneProgramming-OpenCV-Python
import pygame


# func_create window
def init():
    pygame.init()
    pygame.display.set_mode((400, 400))


# func get Key Presses

def get_key(key_name):
    ans = False
    # check the events
    for _ in pygame.event.get():
        pass
    key_input = pygame.key.get_pressed()  # Create input
    my_key = getattr(pygame, 'K_{}'.format(key_name))  # Format to be output for a key pressed e.g K_LEFT
    if key_input[my_key]:
        ans = True
    pygame.display.update()

    return ans


def main():
    if get_key("LEFT"):
        print("Left Key Pressed")
    if get_key("RIGHT"):
        print("Right Key Pressed")
    # print(getKey("a"))


if __name__ == '__main__':
    init()
    while True:
        main()
