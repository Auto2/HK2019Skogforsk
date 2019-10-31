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

## Info om filer
```startrig.sh```  
Kör foljande launchfiler:
testrig.launch (från paket testrig), startar testriggen
rplidar.launch (från paket rplidar_ros), startar lidarn 
sonic.launch (från paket sonic), startar ultraljudssensorer
zed.launch (från paet zed_wrapper), startar stereokameran 

```Testrig.launch```  
startar testriggen med följande noder:  
motor_controller_node  
arduino_motor_node
waist_twist_control
pendulum_arm_control

  
```arduino_motor_node (program: Controller.ino, Arduino Mega ADK)```  
    • Tar in meddelanden från motorkontroller, topic ”pins”, läser av dem och kör önskade motorer i önskad riktning  
    • Visualiserar även information om vilka motorer som körs och åt vilket håll på den lilla displayen (OLED).  
  
    
```keyboard_wsad_node.py (Från laptop)```  
    • Gör att riggen går att kontrollera med wasd-tangenterna genom att publicera meddelanden till topic ”motor_action”  
    • Gör att pendelarmarna går att kontrollera från tangentbord genom att publicera meddelanden till topic ”pendulum_action”
      
      
```motor_controller_node (program: motor_controller_node.cpp, NVIDIA)```  
    • hämtar meddelanden från topic ”motor_action”  
    • publicerar meddelanden till topic ”pins” samt ”motor_override” varav den senare inte används   
    • Skriver ut lite information i terminalen (om motorn är på och vilket håll den roterar åt)  

```waist_twist_control (program: Waists2RG_Scaling_Tuned_ROS.ino, Arduino UNO (waists))```  
    • hämtar meddelanden från topic ”motor_action”  
    • Midjorna svänger då fram/bak och höger/vänster trycks samtidigt   
    • Används tangentbordskontroll styrs testriggen med WASD-knapparna, ex WD för högersväng  
    • Denna nod (arduino) måste kopplas in efter Controller.ino men före, annars hamnar noderna fel i ROS vilket medför att Testriggen inte går att köra.  
   
```pendulum_arm_control (program: PendulumArmsSeparated_ROS.ino, Arduino UNO (pendulum))```  
  • hämtar meddelanden från topic ”pendulum_acton”, svänger då fram/bak och höger/vänster trycks samtidigt   
  • Används tangentbordskontroll styrs pendelarmarna genom att först välja vilken som ska styras (siffra 1-6) och sedan höjas (8) och sänkas (9)  
  • Denna nod (arduino) måste kopplas in efter arduino_motor_node och waist_twist_control, annars hamnar noderna fel i ROS 
    vilket medför att Testriggen inte går att köra  
