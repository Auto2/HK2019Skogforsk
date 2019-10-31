# Testrig
This package allows driving of the testrig.

## Reqs
Apart from ROS, you need ```rosserial```, ```rosserial_arduino``` and the Python package ```pynput``` for the keyboard nodes.

## Connecting it all
Make sure the arduino is connected correctly. Check so that pins in ```arduino_code/controller/controller.ino``` are correct. Motors need power, obviously.

## Keyboard nodes
There are two keyboard nodes available.

The first is ```keyboard_wsad_node.py``` which allows WSAD steering. Easy. It publishes an action message that is then translated into pin output by the motor controller node and subsequentially fed to the Arduino. Quit with ```q```.

The second is ```keyboard_pins_node.py```. There you input how you want your motors to drive. For example ```10 10 00 00``` would mean that motors one and two are enabled and drive in CCW direction. Motors 3 and 4 are not enabled.

## Running the testrig
1. Make sure that the Arduino port in ```launch/testrig.launch``` is correct.
2. Start the testrig with ROS:
```console
roslaunch testrig testrig.launch
```
3. Open a new terminal and start a chosed keyboard node:
```console
rosrun testrig keyboard_xxxx_node.py
```
4. Drive!
