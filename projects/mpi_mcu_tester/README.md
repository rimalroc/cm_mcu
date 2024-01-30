# tester project for the MPI CM microcontroller.


To be completed.
This should be running in a devboard, the idea is that it can test a devboard using physical connections between two boards, running in a CI.

## dependencies
### almalinux9
`sudo dnf install arm-none-eabi-binutils-cs.x86_64 arm-none-eabi-gcc-cs.x86_64 arm-none-eabi-gcc-cs-c++.x86_64 arm-none-eabi-newlib.noarch`

## Make build
In the root directory set the RTOS directory e.g. for test using the devboard you can run the following
```bash
export FREERTOS_ROOT=/home/rrojas/l0mdt/mcu/FreeRTOS-Kernel
make DEVBOARD=1 DEBUG=1 PART=TM4C129ENCPDT
```
## Note
Currently the project is built with no bootloader
### no bootloader
The project originally expects a bootloader on address 0, so the bootloader will jumps to 0x00004000 for the firmware we build.
for debugging is easier to use no bootloader, simply start the firmware at address 0x0, for production we simply change the origin again to 0x00004000 and load the firmware using bootloader.
change the memory region to start becuase the defualt is expecting a bootloader
```
MEMORY
{
    FLASH (rx) : ORIGIN = 0x00004000, LENGTH = 0x000fc000
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00040000
}
```
 to 
 ```
 MEMORY
{
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x000fc000
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00040000
}
 ```
 

## Code structure

To be completed.

## List of Tasks

To be completed.

## Building FreeRTOS

To be completed.
