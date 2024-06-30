#! /usr/bin/env python3

import rospy
import copy
import threading
from led_circuit_ros.neo_pixel import NeoPixel
from led_circuit_msgs.msg import Command, Pattern
from std_msgs.msg import ColorRGBA

DEFAULT_CYCLE_SEC = 1.0


class LedCircuit:
    def __init__(self) -> None:
        # current command
        self.pattern_: int = Pattern.OFF
        self.color_: ColorRGBA = ColorRGBA()
        self.cycle_sec_ = 1.0

        n_pixels = rospy.get_param("~n_pixels", default=16)
        self.demo_ = rospy.get_param("~demo", default=False)
        self.neo_pixel_ = NeoPixel(n_pixels)

        self.lock_ = threading.Lock()

        self.command_sub_ = rospy.Subscriber(
            "command", Command, self.command_callback, queue_size=1)

    def command_callback(self, cmd: Command) -> None:
        if self.pattern_ != cmd.pattern.id or self.color_ != cmd.color or abs(self.cycle_sec_ - cmd.cycle_sec) > 1e-10:
            self.pattern_ = cmd.pattern.id
            self.color_ = cmd.color
            self.cycle_sec_ = cmd.cycle_sec if cmd.cycle_sec > 0 else DEFAULT_CYCLE_SEC
            self.neo_pixel_.abort_ = True

    def run(self) -> None:
        if self.demo_:
            rospy.loginfo("////////////// Demo mode //////////////")
            self.neo_pixel_.light_breathing(ColorRGBA(1, 1, 0, 1), 2.0)
            self.neo_pixel_.light_breathing(ColorRGBA(0, 1, 1, 1), 2.0)
            self.neo_pixel_.light_breathing(ColorRGBA(1, 0, 1, 1), 2.0)
            self.neo_pixel_.light_breathing(ColorRGBA(1, 1, 1, 1), 2.0)
            self.neo_pixel_.light_spin(ColorRGBA(1, 1, 0, 1))
            self.neo_pixel_.light_spin(ColorRGBA(0, 1, 1, 1))
            self.neo_pixel_.light_spin(ColorRGBA(1, 0, 1, 1))
            self.neo_pixel_.light_spin(ColorRGBA(1, 1, 1, 1))
            self.neo_pixel_.light_off()
            return

        rospy.loginfo("Receiving led circuit command...")
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.pattern_ == Pattern.OFF:
                self.neo_pixel_.light_off()
            elif self.pattern_ == Pattern.STATIC:
                self.neo_pixel_.light_static(self.color_)
            elif self.pattern_ == Pattern.BREATHING:
                self.neo_pixel_.light_breathing(
                    self.color_, self.cycle_sec_)
            elif self.pattern_ == Pattern.SPIN:
                self.neo_pixel_.light_spin(self.color_, self.cycle_sec_)
            rate.sleep()
