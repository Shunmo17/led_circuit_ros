LED Circuit ROS
===

![demo](https://raw.github.com/wiki/Shunmo17/led_circuit_ros/images/demo.gif)

## Description

This is a simple ROS wrapper of [Adafruit CircuitPython NeoPixel SPI](https://github.com/adafruit/Adafruit_CircuitPython_NeoPixel_SPI) library.
You can control your LED board connected to Adafruit FT232H via a ROS topic.

## Preparation

1. Prepare workspace

    ```bash
    mkdir -p catkin_ws/src
    cd catkin_ws/src
    git clone git@github.com:Shunmo17/led_circuit_ros.git
    git clone git@github.com:Shunmo17/led_circuit_msgs.git
    ```

2. Install dependencies

    ```bash
    sudo rosdep init
    rosdep update
    rosdep install -iy --from-paths src/
    ```

3. Build

    ```bash
    catkin build
    ```

4. Set udev rules to use USB board without root

    ```bash
    sudo vim /etc/udev/rules.d/11-ftdi.rules
    ```

    Write the following contents.

    ```
    SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6001", GROUP="plugdev", MODE="0666"
    SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6011", GROUP="plugdev", MODE="0666"
    SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6010", GROUP="plugdev", MODE="0666"
    SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6014", GROUP="plugdev", MODE="0666"
    SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6015", GROUP="plugdev", MODE="0666"
    ```

    Apply updated udev rules.

    ```bash
    sudo udevadm control --reload-rules
    ```

5. Connect your LED board to the FT232H

    `DIN` of the LED board must be connect to the `D1 (MOSI)` port of the FT232H. Then, connect the FT232H to your computer.

## Demo

Launch a demo application (without ROS server)

```bash
roslaunch led_circuit_ros led_circuit_server.launch demo:=true
```

## Usage

Launch ROS server

```bash
roslaunch led_circuit_ros led_circuit_server.launch
```

Send command via ROS topic

```bash
rostopic pub /command led_circuit_msgs/Command "
pattern:
  id: 2
color:
  r: 1.0
  g: 0.0
  b: 1.0
  a: 0.0
cycle_sec: 2.0"
```

## Parameters

| Name     | Type | Default | Description                                 |
| -------- | ---- | ------- | ------------------------------------------- |
| n_pixels | int  | 16      | The number of LED pixels on your LED board. |
| demo     | bool | false   | Set true if you launch a demo application.  |

## Subscribed topics

| Name    | Type                     | Description                |
| ------- | ------------------------ | -------------------------- |
| command | led_circuit_msgs/Command | LED control command topic. |

## Dependencies

- pyftdi
- adafruit-blinka
- adafruit-circuitpython-neopixel-spi
- [led_circuit_msgs](https://github.com/Shunmo17/led_circuit_msgs)

## License

MIT

## Maintainer

[Shunmo17](https://github.com/Shunmo17)
