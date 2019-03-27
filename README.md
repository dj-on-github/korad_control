# korad_control
A linux command line python program to control the Korad KA6003P and other Korad bench power supplies.

Connect the PSU to a computer via a serial port or USB and use the program to control the PSU in place of the front panel buttons. It includes an option to monitor the CC-CV charging of batteries and turn off the output when the minimum current is reached (E.G. typically 10ma per cell for 18650 Li-Ion cells).

This has been tested on a Rasberry-Pi with a Korad KA6003P.

It steals liberally from https://github.com/Tamagotono/Korad-KA6003P-Software . But that is a Tk GUI application and I wanted a CLI. The firmware on the KA6003P is buggy and often gives spurious data on the wire. I've added some robustification to deal with that.

Invoking with -h gives:

# ./korad_control.py -h
usage: korad_control.py [-h] [-s] [-n] [--verbose] [-d DEVICE] [-v VOLTAGE]
                        [-i CURRENT] [--output OUTPUT] [--charge]
                        [--imin IMIN] [-o OUTPUT_FILE]

Program to Control Konrad KA6003P Power Supply

optional arguments:
  -h, --help            show this help message and exit
  -s, --status          Just ask the PSU for its status and print it. The
                        default with no voltage, current, ocp, ovp, monitor,
                        on or off command.
  -n, --no_write        Just ask the PSU for its status and print it
  --verbose             Print details of comms with PSU
  -d DEVICE, --device DEVICE
                        Select the serial device. Defaults to searching for
                        /dev/ttyUSB* or /dev/ttyACM*
  -v VOLTAGE, --voltage VOLTAGE
                        Voltage to Set
  -i CURRENT, --current CURRENT
                        Current limit to Set
  --output OUTPUT       Enable or disable power output with on or off, or 1 or
                        0
  --charge              Charge a LiIon type battery and monitor current and
                        voltage. Use with --imin and set voltage to the
                        charging CV and current to charging CC for the battery
                        being charged.
  --imin IMIN           Minimum charging current at which to cancel the charge
                        and turn off the output. Default is 0.01A.
  -o OUTPUT_FILE, --output_file OUTPUT_FILE
                        Filename prefix to put the results in. Otherwise
                        output to stdout


