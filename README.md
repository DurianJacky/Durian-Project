# Durian-Project

NTU Robotx challange \
Propeller system: \
No.1 to 2 is Navy 3.0 \
No.3 to 4 is Minn kota \
Battery: E80 \
this repository includes 3 Arduino codes (one for controller and two for PWM and RS485) and 1 Python code (pyserial). \


Currently, there are 4 Arduino Uno used in the on-water test:
- 1 for controller
- 1 for PWM control
- 2 for RS485 control


\



Manual Control Process:

$$
Controller \rightarrow Arduino(controller) \rightarrow PC/RPi \rightarrow Arduino(propeller)
$$

- The controller sends channel values to the Arduino(controller) by radio
- Arduino(controller) sends control message to PC/RPi by serial
- PC/RPi receives control messages and implements thrust allocation 
- PC/RPi sends corresponding thrust allocation message to each Arduino(propeller) by serial
- Arduinos(propeller) control the propellers with RS485 or PWM 
