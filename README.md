# Modbus-Stepper-Controller
Step/Dir Controller based on teensy. Acts as a Modbus tcp slave, so a modbus master is needed for the setup. the master will need a library issuing different commands over the modbus registers.
For the motion control, following arduino library is used:
https://github.com/luisllamasbinaburo/Arduino-AsyncStepper

So this project consists of two parts:

1 - The MCU (teensy 4.1) "firmware" (including AsyncStepper Lib)  
2 - The accompanying library for the PLC (OpenPLC or Codesys)  

![Test](https://user-images.githubusercontent.com/101837284/159583834-0b0fe905-bf78-46a9-b57c-8987204d72fe.png)

First Prototype for testing communication via modbus and implementing the slave firmware in arduino:

videos: https://youtube.com/shorts/akJ87RP2c9Q?feature=share  
        https://youtu.be/qdu1bqlBr8Q  
        https://youtu.be/bMGx0n8RxtE  
        
![FIrst Prototype](https://user-images.githubusercontent.com/101837284/158897530-49c3237d-0893-4524-8a08-eb9b87bab08a.jpg)
