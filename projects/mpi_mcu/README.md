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
For compiling this firmware check the `.base_docker:` label on the `.gitlab-ci.yml` to understand the requirements on almalinux9
Additionally, if you want to test from a pc or configure your gitlab-runner

```bash
sudo dnf install -y screen python3-pyserial.noarch
```

## CI
A gitlab runner hs been set with all the requirements. If it is no longer availabe, enable the docker excecution for the build job in `.gitlab-ci.yml`.
You will be able to get the binaries from the artifacts, but no testing until you setup a new runner.

The umassminipc02 has attached a [Tiva™ C Series TM4C1294 Connected LaunchPad Evaluation Kit](https://www.ti.com/lit/ug/spmu365c/spmu365c.pdf)
The test-job will load your firmware in it and run an automated test using a simple python script.
This is a very simple test to make sure that the RTOS is not corrupted and some commands are properly working.

## Code structure

To be completed.

## List of Tasks

To be completed.

## Building FreeRTOS

To be completed.
