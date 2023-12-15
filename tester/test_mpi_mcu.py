#!/usr/bin/env python3
import serial
import argparse

def options():
    parser = argparse.ArgumentParser(
        usage=__doc__,
        description='This sripts test the mpi_mcu firmware running on the mcu',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-v", help="verbose", action='store_true')
    parser.add_argument("-d", help="device", default='/dev/ttyACM0')
    return parser.parse_args()

ops = options()
# Configure the USB device port
usb_device_port = ops.d

# List of commands to send
commands = [
    'pwr status\r\n',
    'pwr off\r\n',
    'pwr status\r\n',
    'pwr on\r\n',
    'pwr status\r\n',
    'adc\r\n',
    'gpio\r\n',
    'help\r\n',
# do not test restart command    
#    'restart_mcu\r\n',
    'simple_sensor\r\n',
    'stack_usage\r\n',
    'taskinfo\r\n',
    'taskstats\r\n',
    'uptime\r\n',
    'version\r\n',
    'watchdog\r\n'
]

def simple_test_command (command):
    exit_code = 0
    ser.write(command.encode())
    response = ''
    while True:
        line = ser.readline().decode().strip()
        if line == '':
            break
        response += line + '\n'

    if response == '' \
        or response == 'Command unknown' \
        or 'Error' in response \
        or 'error' in response \
        or 'ERROR' in response :
        print(f"Test failed for command {command} -> response: {response}")
        exit_code = 1
    else:
        if ops.v:
            print(f"Response for command {command}: {response}")
        else:
            print(f"OK")
    return exit_code
    
# Establish connection to the USB device
try:
    ser = serial.Serial(usb_device_port, baudrate=115200, timeout=1)
    print(f"Connected to {usb_device_port}")
except serial.SerialException:
    print(f"Failed to connect to {usb_device_port}")
    exit(1)

exit_code = 0
# Send commands and check for response
for command in commands:
    exit_code = simple_test_command(command)
    if exit_code != 0:
        print("failing command: " + command)
        break



# Close the connection
ser.close()
if exit_code == 0:
    print("Success!")
else:
    print("Fail")
exit(exit_code)
