#! /usr/bin/env python3

import rospy
from led_circuit_ros.led_circuit import LedCircuit


def main():
    rospy.init_node("led_circuit_ros")
    led_circuit = LedCircuit()
    led_circuit.run()


if __name__ == "__main__":
    main()
