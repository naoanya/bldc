
# SimpleFOC on CubeIDE

# Environment

* B-G431B-ESC1
* STM32CubeIDE Version: 1.7.0

# How to Setup

## Run setup.sh

Run setup.sh.

``` bash
./setup.sh
```

This scripts will download and unpack some libraries.

* https://github.com/stm32duino/Arduino_Core_STM32
* https://github.com/stm32duino/ArduinoModule-CMSIS
* https://github.com/simplefoc/Arduino-FOC

## Import Projects

1. Run STM32CubeIDE
2. Select "Import projects...".
3. Select "General"->"Existing Projects into Workspace".
4. Set "SimpleFOC-CubeIDE" directory path to "Select root directory".
5. Check "simplefoc" project.
6. "Finish".

## Build

Run "Build All" on STM32CubeIDE.

## Debug

Connect "B-G431B-ESC1" board via USB.

Run "Debug" on STM32CubeIDE.
