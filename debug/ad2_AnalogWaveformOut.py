# -*- coding: utf-8 -*-
"""
Created on Tue Aug 23 15:15:19 2022

@author: jboss
"""

#Import packages
#Note that the Digilent Waveforms SDK must be installed
from ctypes import *
from dwfconstants import *
import time
import sys
import numpy as np
from matplotlib import pyplot as plt
import subprocess
import serial


#Desired Vout sweel
Vout=np.linspace(0,1,16)

#RF_Pulse_Duration
#t_on= 0.0001
t_on = 1

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

#Attenuator Control, set DIO 0 low to disable extra 16 dB att, or set high to enbale att
dwf.FDwfDigitalIOOutputEnableSet(hdwf, c_int(0x01)) #Mask indicating output will be set on DIO 0 (bit 1)
# set value on enabled IO pins
dwf.FDwfDigitalIOOutputSet(hdwf, c_int(0x00)) 
time.sleep(.1)

#Configure Analog Output Static Offset Initial Value 0V
DC_offset = 0 #V at device
DC = DC_offset/(-100) #Accounting for op amp gain 
dwf.FDwfAnalogOutNodeEnableSet(hdwf,c_int(0),AnalogOutNodeCarrier,c_int(1))
dwf.FDwfAnalogOutNodeOffsetSet(hdwf,c_int(0),AnalogOutNodeCarrier,c_double(DC))


#DC Output Loop
for n in range(0,Vout.size):
    #Configure Analog Output Static Offset
    DC_offset = Vout[n] #V at device
    DC = DC_offset/(-100) #Accounting for op amp gain 
    #dwf.FDwfAnalogOutNodeEnableSet(hdwf,c_int(0),AnalogOutNodeCarrier,c_int(1))
    dwf.FDwfAnalogOutNodeOffsetSet(hdwf,c_int(0),AnalogOutNodeCarrier,c_double(DC))
    time.sleep(0.1) #AO settling time
    
    #Generate a 100us pulse on AD2 DIO1 needed to:
    # 1) Enable the TPI frequency source (active while AUX IN is high)
    # 2) Unblank the AMT amplifier
    # 3) Trigger the Ultraview Card to begin acquisition
    # 4) Trigger AO1 pattern generation (input to high voltage op amp)
    
    dwf.FDwfDigitalOutRunSet(hdwf, c_double(t_on)) # 100 us pulse duration
    dwf.FDwfDigitalOutRepeatSet(hdwf, c_int(1)) # once
    dwf.FDwfDigitalOutIdleSet(hdwf, c_int(1), c_int(1)) # 1=DwfDigitalOutIdleLow, low when not running 
    dwf.FDwfDigitalOutCounterInitSet(hdwf, c_int(1), c_int(1), c_int(0)) # initialize high on start
    dwf.FDwfDigitalOutCounterSet(hdwf, c_int(1), c_int(0), c_int(0)) # low/high count zero, no toggle during run
    dwf.FDwfDigitalOutEnableSet(hdwf, c_int(1), c_int(1))
    
    dwf.FDwfDigitalOutWaitSet(hdwf,c_double(1)) #wait 1 second after generation is started

    time.sleep(1)
        
#Configure Analog Output Static Offset Final Value 0V
DC_offset = 0 #V at device
DC = DC_offset/(-100) #Accounting for op amp gain 

dwf.FDwfAnalogOutNodeOffsetSet(hdwf,c_int(0),AnalogOutNodeCarrier,c_double(DC))

dwf.FDwfAnalogOutReset(hdwf, c_int(0))

dwf.FDwfDeviceClose(hdwf)
print("Closing AD2 device")
