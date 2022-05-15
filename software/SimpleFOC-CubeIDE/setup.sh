#!/bin/bash

curl -OL https://github.com/stm32duino/Arduino_Core_STM32/releases/download/2.1.0/STM32-2.1.0.tar.bz2
curl -OL https://github.com/stm32duino/ArduinoModule-CMSIS/releases/download/5.7.0/CMSIS-5.7.0.tar.bz2
curl -OL https://downloads.arduino.cc/libraries/github.com/askuric/Simple_FOC-2.2.0.zip
tar xf STM32-2.1.0.tar.bz2
tar xf CMSIS-5.7.0.tar.bz2
unzip Simple_FOC-2.2.0.zip
mv Simple_FOC-2.2.0 Simple_FOC

#https://github.com/stm32duino/Arduino_Tools/releases/download/2.1.0/STM32Tools-2.1.0-windows.tar.bz2
#https://github.com/stm32duino/Arduino_Tools/releases/download/2.1.0/STM32Tools-2.1.0-mac.tar.bz2
#https://github.com/stm32duino/Arduino_Tools/releases/download/2.1.0/STM32Tools-2.1.0-linux.tar.bz2
