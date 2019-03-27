#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct  9 23:05:05 2014

@author: jason
"""

#from tkinter import *
#from tkinter import ttk
#from tkinter import messagebox
import serial
import time
import binascii
import argparse
import sys
from pathlib import Path

VMAX=4.2
IMAX=3.1
#==============================================================================
# Define protocol commands
#==============================================================================
REQUEST_STATUS = b"STATUS?"  # Request actual status.
    # 0x40 (Output mode: 1:on, 0:off)
    # 0x20 (OVP and/or OCP mode: 1:on, 0:off)
    # 0x01 (CV/CC mode: 1:CV, 0:CC)
REQUEST_ID = b"*IDN?"

REQUEST_SET_VOLTAGE = b"VSET1?"  # request the set voltage
REQUEST_ACTUAL_VOLTAGE = b"VOUT1?"  # Request output voltage

REQUEST_SET_CURRENT = b"ISET1?"  # Request the set current
REQUEST_ACTUAL_CURRENT = b"IOUT1?"  # Requst the output current

SET_VOLTAGE = b"VSET1:"  # Set the maximum output voltage
SET_CURRENT = b"ISET1:"  # Set the maximum output current

SET_OUTPUT = b"OUT"  # Enable the power output

SET_OVP = b"OVP"  # Enable(1)/Disable(0) OverVoltageProtection

SET_OCP = b"OCP"  # Enable(1)/Disable(0) OverCurrentProtection

#==============================================================================
# Methods
#==============================================================================

def print_sent(s):
    global verbose
    if verbose:
        print("  SENT :", s)
        print("  As hex: ", binascii.hexlify(s))

def print_got(s):
    global verbose
    if verbose:
        print("   GOT :", s)
        print("  As hex: ", binascii.hexlify(s))


def GetID(PS):
    PS.flushInput()
    PS.write(REQUEST_ID)  # Request the ID from the Power Supply
    print_sent(REQUEST_ID)
    PSID = PS.read(16)
    print_got(PSID)
    #print(b'PSID = '+PSID)
    PS.flushInput()
    return(PSID)


def Get_I_Set(PS):
    PS.flushInput()
    PS.write(REQUEST_SET_CURRENT)  # Request the target current
    print_sent(REQUEST_SET_CURRENT)
    I_set = PS.read(5)
    print_got(I_set)
    if (I_set == b''):
        I_set = b'0'
    I_set = float(I_set)
    #print(str('Current is set to ')+str(I_set))
    PS.flushInput()
    return(I_set)


def Get_V_Set(PS):
    PS.flushInput()
    PS.write(REQUEST_SET_VOLTAGE)  # Request the target voltage
    print_sent(REQUEST_SET_VOLTAGE)
    #try:
    vs = PS.read(5)
    print_got(vs)
    astr = vs.decode()
    V_set = float(astr)
    #except:
    #    print("Funny VS = ",vs)
    #    V_set = -1.0
    #print(str('Voltage is set to ')+str(V_set))
    PS.flushInput()
    return(V_set)


def Get_Status(PS):
    PS.flushInput()
    PS.write(REQUEST_STATUS)  # Request the status of the PS
    print_sent(REQUEST_STATUS)
    Stat = PS.read(1)
    print_got(Stat)
    PS.flushInput()
    try:
        astr = Stat.decode()
        x = ord(astr[0])
    except:
        print("Bad reponse :",astr)
        x = 0x00
    os=""
    if x & 0x40 == 0x40:
        os += "Output ON "
    else:
        os += "Output OFF"
    
    if x & 0x20 == 0x20:
        os += ", OVP/OCP ON "
    else:
        os += ", OVP/OCP OFF"

    if x & 0x01 == 0x01:
        os += ", CV "
    else:
        os += ", CC"

    

    #print('Status = '+Stat)
    return(os)


def SetVoltage(PS,Voltage):
    #PS = serial.Serial("/dev/ttyACM0",
    #                   baudrate=9600,
    #                   bytesize=8,
    #                   parity='N',
    #                   stopbits=1,
    #                   timeout=1)
    PS.flushInput()
    if (float(Voltage) > float(VMAX)):
        Voltage = VMAX
    Voltage = "{:2.2f}".format(float(Voltage))
    Output_string = SET_VOLTAGE + bytes(Voltage, "utf-8")
    PS.write(Output_string)
    print_sent(Output_string)
    PS.flushInput()
    time.sleep(0.2)
    VeriVolt = "{:2.2f}".format(float(Get_V_Set(PS)))  # Verify PS accepted
        # the setting
#    print(VeriVolt)
#    print(Voltage)
    if (VeriVolt != Voltage):
        print("verivolt:",VeriVolt," != Voltage:",Voltage)
        #PS.write(Output_string)  # Try one more time
        #print_sent(Output_string)
    #vEntry.delete(0, 5)
    #vEntry.insert(0, "{:2.2f}".format(float(VeriVolt)))
    return(Output_string)


def SetCurrent(PS,Current):
    #PS = serial.Serial("/dev/ttyACM0",
    #                   baudrate=9600,
    #                   bytesize=8,
    #                   parity='N',
    #                   stopbits=1,
    #                   timeout=1)
    PS.flushInput()
    if (float(Current) > float(IMAX)):
        Current = IMAX
    Current = "{:2.3f}".format(float(Current))
    Output_string = SET_CURRENT + bytes(Current, "utf-8")
    PS.write(Output_string)
    print_sent(Output_string)
    print(Output_string)
    PS.flushInput()
    time.sleep(0.2)
    VeriAmp = "{:2.3f}".format(float(Get_I_Set(PS)))
    if (VeriAmp != Current):
        VeriAmp = 0.00
    #iEntry.delete(0, 5)
    #iEntry.insert(0, "{:2.3f}".format(float(VeriAmp)))
    return(Output_string)


def V_Actual(PS):
    #PS = serial.Serial("/dev/ttyACM0",
    #                   baudrate=9600,
    #                   bytesize=8,
    #                   parity='N',
    #                   stopbits=1,
    #                   timeout=1)
    PS.flushInput()
    PS.write(REQUEST_ACTUAL_VOLTAGE)  # Request the actual voltage
    print_sent(REQUEST_ACTUAL_VOLTAGE)
    time.sleep(0.015)
    V_actual = PS.read(5)
    V_actual_str = V_actual.decode("utf-8")
    #print(V_actual)
    #print(V_actual_str)

    #print("A_actual string =",V_actual)
    if (V_actual == b''):
            V_actual = b'0.0'  # deal with the occasional NULL from PS
    V_actual = float(V_actual.decode())
    PS.flushInput()
    return(V_actual)


def I_Actual(PS):
    #PS = serial.Serial("/dev/ttyACM0",
    #                   baudrate=9600,
    #                   bytesize=8,
    #                   parity='N',
    #                   stopbits=1,
    #                   timeout=1)
    PS.flushInput()
    PS.write(REQUEST_ACTUAL_CURRENT)  # Request the actual current
    print_sent(REQUEST_ACTUAL_CURRENT)
    time.sleep(0.2)
    I_actual = PS.read(5)
    print_got(I_actual)
    #print("I_actual string =",I_actual)
    if (I_actual == b''):
            I_actual = b'0'  # deal with the occasional NULL from PS
    I_actual = float(I_actual)
    PS.flushInput()
    return(I_actual)


def SetOP(PS,OnOff):
    #PS = serial.Serial("/dev/ttyACM0",
    #                   baudrate=9600,
    #                   bytesize=8,
    #                   parity='N',
    #                   stopbits=1,
    #                   timeout=1)
    PS.flushInput()

    Output_string = SET_OUTPUT + bytes(OnOff, "utf-8")

    PS.write(Output_string)
    print_sent(Output_string)
    #print(Output_string)
    PS.flushInput()
    return(Output_string)


def SetOVP(PS,OnOff):
    #PS = serial.Serial("/dev/ttyACM0",
    #                   baudrate=9600,
    #                   bytesize=8,
    #                   parity='N',
    #                   stopbits=1,
    #                   timeout=1)
    PS.flushInput()
    Output_string = SET_OVP + OnOff
    PS.write(Output_string)
    print_sent(Output_string)
    PS.flushInput()
    return(Output_string)


def SetOCP(PS,OnOff):
    #PS = serial.Serial("/dev/ttyACM0",
    #                   baudrate=9600,
    #                   bytesize=8,
    #                   parity='N',
    #                   stopbits=1,
    #                   timeout=1)
    PS.flushInput()
    Output_string = SET_OCP + OnOff
    PS.write(Output_string)
    print_sent(Output_string)
    PS.flushInput()
    return(Output_string)


def MemSet(MemNum):
    print(MemNum)



# Argument Parsing

parser = argparse.ArgumentParser(description='Program to Control Konrad KA6003P Power Supply')
parser.add_argument('-s','--status', action='store_true', default=None, help='Just ask the PSU for its status and print it. The default with no voltage, current, ocp, ovp, monitor, on or off command.')
parser.add_argument('-n','--no_write', action='store_true', default=False, help='Just ask the PSU for its status and print it')
parser.add_argument('--verbose', action='store_true', default=False, help='Print details of comms with PSU')
parser.add_argument('-d','--device', default=None,help='Select the serial device. Defaults to searching for /dev/ttyUSB* or /dev/ttyACM*')
parser.add_argument('-v','--voltage', type=float,default=None, help='Voltage to Set')
parser.add_argument('-i','--current', type=float,default=None, help='Current limit to Set')
#parser.add_argument('--ocp', type=str,default=None, help='Enable or disable Over Current Protection with on or off, or 1 or 0')
#parser.add_argument('--ovp', type=str,default=None, help='Enable or disable Over Voltage Protection with on or off, or 1 or 0')
parser.add_argument('--output', type=str,default=None, help='Enable or disable power output with on or off, or 1 or 0')
parser.add_argument('--charge', action='store_true',default=None, help='Charge a LiIon type battery and monitor current and voltage. Use with --imin and set voltage to the charging CV and current to charging CC for the battery being charged.')
parser.add_argument('--imin', type=float,default=0.01, help='Minimum charging current at which to cancel the charge and turn off the output. Default is 0.01A.')
parser.add_argument('-o', '--output_file', type=str, default=None, help='Filename prefix to put the results in. Otherwise output to stdout')
args = parser.parse_args()

#==============================================================================
# Serial Port
#==============================================================================

#nowrite means don't actually send any commands. Just go through the motions.

nowrite = args.no_write
dowrite = not(nowrite)

imin = float(args.imin)

verbose = args.verbose

# search for the right port
if dowrite and args.device==None:
    print("serching for ports")
    ports = ["/dev/ttyACM0","/dev/ttyUSB0","/dev/ttyACM1","/dev/ttyUSB1"]

    filename = None
    for port in ports:
        if Path(port).exists():
            filename = port
            print("Found port ",filename)
            break
    if filename == None:
        print("Error: Didn't find a port")
        print("Exiting")
        exit()
elif dowrite and args.device != None:
    filename = str(args.device)

if dowrite:
    print("opening serial port as ",filename)
    PS = serial.Serial(filename,
                   baudrate=9600,
                   bytesize=8,
                   parity='N',
                   stopbits=1,
                   xonxoff=False,
                   rtscts=False,
                   dsrdtr=False,
                   write_timeout=1,
                   timeout=1)
else:
    PS=None

do_status = True

if args.voltage != None:
    volts = float(args.voltage)
    SetVoltage(PS,volts)
    time.sleep(0.5)
    do_status = False


if args.current != None:
    current = float(args.current)
    SetCurrent(PS,current)
    time.sleep(0.5)
    do_status = False


if args.output != None:
    if args.output == "1" or args.output.lower()=="on":
        SetOP(PS,'1')
        time.sleep(1)
        do_status = False
    elif args.output == "0" or args.output.lower()=="off":
        SetOP(PS,'0')
        time.sleep(1)
        do_status = False

if do_status or args.status:
    PSID = GetID(PS)
    V = V_Actual(PS)
    I = I_Actual(PS)
    VS = Get_V_Set(PS)
    IS = Get_I_Set(PS)
    status = Get_Status(PS)
    print("PSID = ",str(PSID.decode()))
    print("V = {0:1.3f}V  I = {1:1.3f}A  Vsetting={2:1.3f}  Isetting={3:1.3f} status={4:s}".format(V,I,VS,IS,str(status)))
    if do_status == True:
        PS.closr()
        quit()

if args.charge == True:
    SetOP(PS,'1')
    time.sleep(1)
    while True:
        V = V_Actual(PS)
        I = I_Actual(PS)
        VS = Get_V_Set(PS)
        IS = Get_I_Set(PS)
        status = Get_Status(PS)
        print("V = {0:1.3f}V  I = {1:1.3f}A  Vsetting={2:1.3f}  Isetting={3:1.3f} status={4:s}".format(V,I,VS,IS,str(status)))
        if (I != 0.0):
            if I < imin:
                break
        time.sleep(0.5)

    print("Finished charging. Turning off output")
    SetOP(PS,'0')
    time.sleep(1)


PS.close()


