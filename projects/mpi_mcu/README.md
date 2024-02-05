# Main project for the MPI CM microcontroller.


To be completed.

## Make build
In the root directory set the RTOS directory e.g. for test using the devboard you can run the following
```bash
export FREERTOS_ROOT=/home/rrojas/l0mdt/mcu/FreeRTOS-Kernel
make DEVBOARD=1 DEBUG=1 PART=TM4C1294NCPDT
```
Others examples for the L0MDTTP project building for the Demonstrator or the Prototype will be available in future.


## Install the devboard
No need to install the software from Tivaware. Just need to add the following udev rule to be able to access the device
```bash
echo 'ATTRS{idVendor}=="1cbe", ATTRS{idProduct}=="00fd", GROUP="users", MODE="0666"' | sudo tee /etc/udev/rules.d/99-stellaris-launchpad.rules
echo 'KERNEL=="ttyACM[0-9]*",MODE:="0666"' | sudo tee -a /etc/udev/rules.d/99-stellaris-launchpad.rules
```

### requirements
Additionally, if you want to test from a pc or configure your gtlab-runner

```bash
sudo dnf install -y screen python3-pyserial.noarch
```


## Code structure

To be completed.

## List of Tasks

To be completed.

## Building FreeRTOS

To be completed.
