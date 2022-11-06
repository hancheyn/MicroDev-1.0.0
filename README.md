# MicroDev Development Board Tester Setup

## Raspbian Setup
- Download Raspbian OS Imager (https://www.raspberrypi.com/software/)
- Insert and locate SD card on computer
- Install Recommended Raspberry Pi OS (w/ Software) to SD card.
- Boot-up Raspi
- Username: microdev | Password: microdev
- Connect to Internet
- Terminal: git clone https://github.com/hancheyn/MicroDev.git
- Terminal: chmod 777 MicroDev/setup.sh
- Terminal: ./MicroDev/setup.sh
- File Manager > Edit > Preferences > Volume Management > 
[uncheck] Show available options for removable media when they are inserted
- Raspberry Pi Configuration > [disable] Screen Blanking


## Arduino CLI Installation & Application
Start up raspberry pi using an hdmi monitor, keyboard, and mouse.

    Download in Add/Remove Software:
- Raspberry OS Start Menu > Preferences > Add/Remove Software
- Search “arduino”
- Check AVR development board IDE
- Check Command Line Tool for compiling Arduino Sketches
- Click Apply and Wait for Download. 


    Download in Terminal:
- Terminal: curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
- Put arduino-cli into a runnable folder so the command will run on its own ($PATH).
- Terminal Command: arduino-cli core install arduino:avr
- Terminal Command View Connected Boards: arduino-cli board list
- Terminal Command Compile .ino example: arduino-cli compile -b arduino:avr:uno Blink.ino -e
- Terminal Command Upload .ino example: arduino-cli upload -b arduino:avr:uno -p /dev/ttyACM0 Blink.ino


### FLASH TO ARDUINO NOTES:
The Arduino CLI does not automatically create a permanent bit file for uploading. In order to do this, one must add the command ‘-e’ while compiling. This looks like the following: arduino-cli compile -b arduino:avr:uno Blink.ino -e.This will create a new folder within the arduino project folder that contains a bit file, “[Project name].ino.with_bootloader.bin”. This should be in a folder with a file path ./build/arduino.avr.uno/. In Terminal, the command arduino-cli upload -b arduino:avr:uno -p /dev/ttyACM0 -i [binary file path] will upload the file.
    
STM32 CLI Installation & Application
Start up raspberry pi using an hdmi monitor, keyboard, and mouse.

    Download in Add/Remove Software:
- Raspberry OS Start Menu  > Preferences > Add/Remove Software
- Search “stlink”
- Check OpenSource ST-Link tools replacement… (x3)
  - …libstlink1-1.6.1+ds-3
  - …stlink-gui-1.6.1+ds-3
  - …stlink-tools-1.6.1+ds-3

- Click Apply and Wait for Download 


### FLASH TO STM32 NOTES:
The STMCubeIDE does not automatically create a permanent bit file for uploading. Since it uses a .elf file (located in the debug folder) this must first be converted to a .bin file before flashing can occur. The .elf file can be converted using the following command from terminal arm-none-eabi-objcopy -O binary F401RE_T.elf main.bin. In the previous example, F401RE_T.elf becomes a binary file called main.bin. Flashing can then occur using the command st-flash write main.bin 0x08000000.
