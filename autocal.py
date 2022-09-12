#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 16 15:10:09 2022

@author: jboss
"""

import numpy as np
from matplotlib import pyplot as plt
import uvfunc as uv
import subprocess
#Import packages
#Note that the Digilent Waveforms SDK must be installed
from ctypes import *
from dwfconstants import *
import time
import os
import sys
import numpy as np
from matplotlib import pyplot as plt
#import uvfunc as uv
import subprocess
import serial

cwd = os.getcwd()
datDirectory  = cwd+"/./uvdma.dat"


from serial.tools import list_ports

port = list(serial.tools.list_ports.comports())
portName =""
for p in port:
	print(p.device)
	print(p.description)
	if str.__contains__(p.description,"FT230X"):
		portName = (p.device)
print(portName)
ser=serial.Serial(portName,3000000,timeout=1)

def tpipacket(bod):
    header=np.array([0xAA,0x55])
    body=np.array(bod)
    numbyte=np.int16(np.size(body))
    bytebin=bin(numbyte)[2:].zfill(16)
    qual1=int(bytebin[0:8],2)
    qual2=int(bytebin[8:17],2)
    checksum=~np.uint8(np.sum(body)+np.sum(numbyte))
    pkt=bytearray(np.hstack([header,qual1,qual2,body,checksum]).tolist())
    return pkt

def tpirx(raw_rx):
    data=list(raw_rx)
    print(data)
    if data[0:2]!=[170,85]:
        print('RX Header Bad')
    cksm=~np.uint8(sum(data[2:-1]))
    if data[-1]!=cksm:
        print('RX Checksum Bad')
    rx_hex=[hex(x) for x in data[4:-1]]
    return rx_hex
    
#Enable User Control
ser.write(tpipacket([0x08,0x01]))
rx=tpirx(ser.read(7))
print(rx)

#Write Frequency
set_freq=128000 #kHz
#Convert to 4 bytes
freq_bytes=set_freq.to_bytes(4,'little')
freq_list=list(freq_bytes)
ser.write(tpipacket([0x08,0x09]+freq_list))
rx=tpirx(ser.read(7))
print(rx)

"""
#Read RF Output Level
ser.write(tpipacket([0x07,0x0A]))
rx=tpirx(ser.read(100))
"""

#Write inital RF Output Level
PdBm_init=-33 #dBm
ser.write(tpipacket([0x08,0x0A]+list(PdBm_init.to_bytes(1,'little',signed=True))))
rx=tpirx(ser.read(8))
print(rx)

#Set Aux Input Action to RF out on while active
ser.write(tpipacket([0x08,0x13,0x01]))
rx=tpirx(ser.read(7))
print(rx)

#Enable RF Output
ser.write(tpipacket([0x08,0x0B,0x01]))
rx=tpirx(ser.read(7))
print(rx)

#RF_Pulse_Duration
t_on= 0.0001

#Import AD2 control packages
if sys.platform.startswith("win"):
    dwf = cdll.dwf
elif sys.platform.startswith("darwin"):
    dwf = cdll.LoadLibrary("/Library/Frameworks/dwf.framework/dwf")
else:
    dwf = cdll.LoadLibrary("libdwf.so")
    
hdwf = c_int()
dwRead = c_uint32()
sts = c_byte()
IsEnabled = c_bool()
usbVoltage = c_double()
usbCurrent = c_double()
auxVoltage = c_double()
auxCurrent = c_double()

version = create_string_buffer(16)
dwf.FDwfGetVersion(version)
print("DWF Version: "+str(version.value))

print("Opening first device")
dwf.FDwfDeviceOpen(c_int(-1), byref(hdwf))

if hdwf.value == hdwfNone.value:
    print("failed to open device")
    szerr = create_string_buffer(512)
    dwf.FDwfGetLastErrorMsg(szerr)
    print(str(szerr.value))

#Disable attenuator
#Attenuator Control, set DIO 0 low to disable extra 16 dB att, or set high to enbale att
dwf.FDwfDigitalIOOutputEnableSet(hdwf, c_int(0x01)) #Mask indicating output will be set on DIO 0 (bit 1)
# set value on enabled IO pins
dwf.FDwfDigitalIOOutputSet(hdwf, c_int(0x00)) 
time.sleep(.1)


instr=['Connect open standard and press any key to measure',
       'Connect short standard and press any key to measure',
       'Connect 50 ohm load standard and press any key to measure']

gamma=np.zeros((2,3),dtype=complex)

#Calibrations
print('Power on AMT amplifier')
for i in range(0,3):
    input(instr[i])
    #Without attenuator
    dwf.FDwfDigitalIOOutputEnableSet(hdwf, c_int(0x01)) #Mask indicating output will be set on DIO 0 (bit 1)
    # set value on enabled IO pins
    dwf.FDwfDigitalIOOutputSet(hdwf, c_int(0x00)) 
    time.sleep(.1)
    #Generate a 100us pulse on AD2 DIO1 needed to:      
    dwf.FDwfDigitalOutRunSet(hdwf, c_double(t_on)) # 100 us pulse duration
    dwf.FDwfDigitalOutRepeatSet(hdwf, c_int(1)) # once
    dwf.FDwfDigitalOutIdleSet(hdwf, c_int(1), c_int(1)) # 1=DwfDigitalOutIdleLow, low when not running 
    dwf.FDwfDigitalOutCounterInitSet(hdwf, c_int(1), c_int(1), c_int(0)) # initialize high on start
    dwf.FDwfDigitalOutCounterSet(hdwf, c_int(1), c_int(0), c_int(0)) # low/high count zero, no toggle during run
    dwf.FDwfDigitalOutEnableSet(hdwf, c_int(1), c_int(1)) 
    dwf.FDwfDigitalOutWaitSet(hdwf,c_double(1)) #wait 1 second after generation is started
    
    #Ultraview Impedance Acquisition

    print("Generating 100us pulse")
    dwf.FDwfDigitalOutConfigure(hdwf, c_int(1)) #starts generation
    
   
    p = subprocess.run([cwd+"/acquire","1","-dcm","-dcs","01","-ttledge","-ttlinv"," -f "+datDirectory,cwd])
    data=uv.load2ch(datDirectory)
    gamma[0,i]=uv.computegamma(data)
    
    #With attenuator
    dwf.FDwfDigitalIOOutputEnableSet(hdwf, c_int(0x01)) #Mask indicating output will be set on DIO 0 (bit 1)
    # set value on enabled IO pins
    dwf.FDwfDigitalIOOutputSet(hdwf, c_int(0x01)) 
    time.sleep(.1)
    #Generate a 100us pulse on AD2 DIO1 needed to:
    dwf.FDwfDigitalOutRunSet(hdwf, c_double(t_on)) # 100 us pulse duration
    dwf.FDwfDigitalOutRepeatSet(hdwf, c_int(1)) # once
    dwf.FDwfDigitalOutIdleSet(hdwf, c_int(1), c_int(1)) # 1=DwfDigitalOutIdleLow, low when not running 
    dwf.FDwfDigitalOutCounterInitSet(hdwf, c_int(1), c_int(1), c_int(0)) # initialize high on start
    dwf.FDwfDigitalOutCounterSet(hdwf, c_int(1), c_int(0), c_int(0)) # low/high count zero, no toggle during run
    dwf.FDwfDigitalOutEnableSet(hdwf, c_int(1), c_int(1)) 
    dwf.FDwfDigitalOutWaitSet(hdwf,c_double(1)) #wait 1 second after generation is started
    
    #Ultraview Impedance Acquisition

    print("Generating 100us pulse")
    dwf.FDwfDigitalOutConfigure(hdwf, c_int(1)) #starts generation
    p = subprocess.run([cwd+"/acquire","1","-dcm","-dcs","01","-ttledge","-ttlinv"," -f "+datDirectory,cwd])
    data=uv.load2ch(datDirectory)
    gamma[1,i]=uv.computegamma(data)
    
#Disable RF Output
ser.write(tpipacket([0x08,0x0B,0x00]))
rx=tpirx(ser.read(7))
print(rx)

dwf.FDwfAnalogOutReset(hdwf, c_int(0))

dwf.FDwfDeviceClose(hdwf)
print("Closing AD2 device")

#Close serial connection with TPI
ser.close()  

E=np.zeros((3,2),dtype=complex)

#Compute calibration constants (without att)
A=([1,1*gamma[0,0],-1],[1,-1*gamma[0,1],1],[1,0*gamma[0,2],0])
B=([gamma[0,0]],[gamma[0,1]],[gamma[0,2]])
E[:,0]=np.matmul(np.linalg.inv(A),B)[:,0]

#Compute calibration constants (with att)
A=([1,1*gamma[1,0],-1],[1,-1*gamma[1,1],1],[1,0*gamma[1,2],0])
B=([gamma[1,0]],[gamma[1,1]],[gamma[1,2]])
E[:,1]=np.matmul(np.linalg.inv(A),B)[:,0]


np.save('cal.npy',E)



