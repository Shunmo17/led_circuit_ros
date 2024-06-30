#! /usr/bin/env python3
from std_msgs.msg import ColorRGBA
import rospy
import neopixel_spi as neopixel
import board
import time
import numpy as np
import math
from typing import Tuple
dir(time)


PIXEL_ORDER = neopixel.GRB


def rgb_to_hex(rgba: ColorRGBA):
    r = int(rgba.r * 255)
    g = int(rgba.g * 255)
    b = int(rgba.b * 255)
    return (r << 16) + (g << 8) + b


def rosmsg_to_rgb(color_msg: ColorRGBA):
    return np.array((int(color_msg.r * 255),
                     int(color_msg.g * 255),
                     int(color_msg.b * 255)))


class NeoPixel():
    def __init__(self, n_pixels: int = 16):
        spi = board.SPI()
        self.pixels_ = neopixel.NeoPixel_SPI(spi,
                                             n_pixels,
                                             pixel_order=PIXEL_ORDER,
                                             auto_write=False)
        self.abort_ = False

    def light_off(self):
        self.pixels_.fill(0)
        self.pixels_.show()

    def light_static(self,
                     color_msg: ColorRGBA = ColorRGBA(1, 1, 1, 1)):
        self.clear()
        n = self.pixels_.n
        color = rosmsg_to_rgb(color_msg)

        if self.abort_:
            self.abort_ = False

        for i in range(n):
            self.pixels_[i % n] = color
        self.pixels_.show()

    def light_breathing(self,
                        color_msg: ColorRGBA = ColorRGBA(1, 1, 1, 1),
                        cycle_sec: float = 1.0):
        STEP = 32

        self.clear()
        n = self.pixels_.n
        color = rosmsg_to_rgb(color_msg)
        rate = rospy.Rate(1.0 / (cycle_sec / (STEP * 2)))

        if self.abort_:
            self.abort_ = False

        for j in range(STEP * 2):
            brightness = 0.0
            if (j // STEP) % 2 == 0:
                brightness = (j % STEP) / float(STEP)
            else:
                brightness = 1.0 - (j % STEP) / float(STEP)
            for i in range(n):
                self.pixels_[i % n] = color * brightness

            # if a new command is received, abort
            if self.abort_:
                return

            self.pixels_.show()
            rate.sleep()

    def light_spin(self,
                   color_msg: ColorRGBA = ColorRGBA(1, 1, 1, 1),
                   cycle_sec: float = 1.0,
                   width: int = 4,
                   brightness_range: Tuple[int] = (0.0, 0.8)):
        n = self.pixels_.n
        color = rosmsg_to_rgb(color_msg)
        diff_brightness = (brightness_range[1] - brightness_range[0]) / width
        rate = rospy.Rate(1.0 / (cycle_sec / n))

        if self.abort_:
            self.abort_ = False

        for i in range(n):
            self.clear()

            self.pixels_[i % n] = color

            for j in range(1, width):
                brightness = brightness_range[1] - diff_brightness * j
                self.pixels_[(i - j) % n] = color * brightness
                self.pixels_[(i + j) % n] = color * brightness

            # if a new command is received, abort
            if self.abort_:
                return

            rate.sleep()
            self.pixels_.show()

    def clear(self):
        for i in range(self.pixels_.n):
            self.pixels_[i] = (0, 0, 0)
