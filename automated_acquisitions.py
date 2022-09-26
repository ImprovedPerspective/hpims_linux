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
import uvfunc as uv
import subprocess
import serial

# Smith Chart Plotter
import plotly.graph_objects as go

import os 
cwd = os.getcwd()
datDirectory  = cwd+"/./uvdma.dat"
#p = subprocess.run([cwd+"/acquire"," 1 -ttledge -ttlinv -dcm -dcs 01 -f "+datDirectory])

from serial.tools import list_ports

port = list(serial.tools.list_ports.comports())
portName =""
for p in port:
	print(p.device)
	print(p.description)
	if str.__contains__(p.description,"FT230X"):
		portName = (p.device)
print(portName)
#Configure TPI
ser=serial.Serial(portName,3000000,timeout=1)

#Load Calibrations
E=np.load('cal.npy')


#Desired Power Sweep
PdBm=np.arange(-25,-19,2)

#Desired Vout sweel
Vout=np.linspace(0,40,1)

Zmatrix=np.zeros((PdBm.size,Vout.size),dtype=complex)

#plotting
fig,ax = plt.subplots(nrows=1,ncols=2,figsize=(12,5))


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

#Disable RF Output
ser.write(tpipacket([0x08,0x0B,0x00]))
rx=tpirx(ser.read(7))
print(rx)

#Write inital RF Output Level
PdBm_init=-65 #dBm
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
    
     
    #RF Power loop
    for i in range(0,PdBm.size):
            
        #Write RF Output Level
        ser.write(tpipacket([0x08,0x0A]+list(int(PdBm[i]).to_bytes(1,'little',signed=True))))
        rx=tpirx(ser.read(8))
        print(rx)
        
        #Enable attenuator if PdBm > -35 dBm, otherwise disable
        if PdBm[i]>=-35:
            dwf.FDwfDigitalIOOutputSet(hdwf, c_int(0x01)) 
            time.sleep(.1)
            Ea=E[:,0] #Use no att calibration
        else:
            dwf.FDwfDigitalIOOutputSet(hdwf, c_int(0x00)) 
            time.sleep(.1)    
            Ea=E[:,1] #Use att calibration 
        
        """
        #Configure AO 0 for pulsed output
        #Waveform shape
        cSamples = 100
        hzRate = 1e6
        
        rgdSamples = (c_double*cSamples)()
        # samples between -1 and +1
        for t in range(0,cSamples):
            rgdSamples[t] = 1.0*t/cSamples;
        
        Amp=1
        dwf.FDwfAnalogOutNodeFunctionSet(hdwf, c_int(0), AnalogOutNodeCarrier, c_int(31)) # funcPlay
        
        dwf.FDwfAnalogOutNodeDataSet(hdwf,c_int(0),AnalogOutNodeCarrier,rgdSamples,cSamples)
        dwf.FDwfAnalogOutNodeAmplitudeSet(hdwf,c_int(0),AnalogOutNodeCarrier,c_double(Amp))
        dwf.FDwfAnalogOutIdleSet(hdwf, c_int(0), c_int(1)) # DwfAnalogOutIdleOffset
        dwf.FDwfAnalogOutTriggerSourceSet(hdwf,c_int(0),trigsrcDigitalOut)
        dwf.FDwfAnalogOutRunSet(hdwf,c_int(0),c_double(t_on))
        #dwf.FDwfAnalogOutWaitSet(hdwf,c_int(0),c_double(10e-6))
        
        dwf.FDwfAnalogOutNodeFrequencySet(hdwf, c_int(0), AnalogOutNodeCarrier, c_double(hzRate)) 
        dwf.FDwfAnalogOutRunSet(hdwf, c_int(0), c_double(cSamples/hzRate)) # run for pattern duration
        dwf.FDwfAnalogOutRepeatSet(hdwf, c_int(0), c_int(1)) # repeat once
        
        dwf.FDwfAnalogOutConfigure(hdwf, c_int(0), c_bool(True))
            
        #dwf.FDwfAnalotOutRepeatTriggerSet(hdwf,c_int(0),c_int(1))
        
        """    
            
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
        
        #Ultraview Impedance Acquisition
        dwf.FDwfDigitalOutConfigure(hdwf, c_int(1)) #starts generation
        print("Generating 100us pulse")
            
        p = subprocess.run([cwd+"/acquire","1","-dcm","-dcs","01","-ttledge","-ttlinv"," -f "+datDirectory,cwd])
        data=uv.load2ch(datDirectory)
        spect=np.fft.fftshift(np.fft.fft(data,axis=0),axes=0)
        #ax[0].plot(data)
        #ax[1].plot(np.abs(spect))
        #print(data.shape)
        #plt.show(block=False)
        gamma=uv.computegamma(data)

        #Diagnostics
                




        #Correction
        gamma=(gamma-Ea[0])/(gamma*Ea[1]-Ea[2])
        
        gamma_mag=np.abs(gamma)
        gamma_ph=np.angle(gamma)
        gamma_ph_deg=gamma_ph*180/np.pi
        
        Zmatrix[i,n]=-50*(gamma+1)/(gamma-1)
        Zi = np.transpose(Zmatrix.imag/50).tolist()[0]
        Zr = np.transpose(Zmatrix.real/50).tolist()[0]
        fig = go.Figure(go.Scattersmith(imag=Zi, real=Zr))
        fig.show()
 
        
        
        


#Configure Analog Output Static Offset Final Value 0V
DC_offset = 0 #V at device
DC = DC_offset/(-100) #Accounting for op amp gain 

dwf.FDwfAnalogOutNodeOffsetSet(hdwf,c_int(0),AnalogOutNodeCarrier,c_double(DC))

#Disable RF Output
ser.write(tpipacket([0x08,0x0B,0x00]))
rx=tpirx(ser.read(7))
print(rx)

dwf.FDwfAnalogOutReset(hdwf, c_int(0))

dwf.FDwfDeviceClose(hdwf)
print("Closing AD2 device")

#Close serial connection with TPI
ser.close()

print(Zmatrix)
