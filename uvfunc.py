#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 17 12:48:40 2022

@author: jboss
"""
import numpy as np

def load2ch(filename):
    file=open(filename,"rb")
    dt=np.dtype('uint16')
    data=np.fromfile(file,dtype=dt)
    file.close()
    siz=data.shape
    data=np.reshape(data,(int(siz[0]/2),2))
    return data  

def load4ch(filename):
    file=open(filename,"rb")
    dt=np.dtype('uint16')
    data=np.fromfile(file,dtype=dt)
    file.close()
    siz=data.shape
    data=np.reshape(data,(int(siz[0]/4),4))
    return data  

def computegamma(data,pts=1024):
    #remove DC
    data=data-np.average(data,axis=0)
    #data=data[1000:(1000+pts)]
    siz=data.shape
    pkpt=int(np.round(3/250*siz[0]))
    
    """
    #Window
    win=np.hanning(int(siz[0]))
    for n in range(0,2):
        data[:,n]=data[:,n]*win
    """
    
    #FFT    
    spect=np.fft.fftshift(np.fft.fft(data,axis=0),axes=0)
    magspect=np.abs(spect)
    
    #Peak index
    idx=magspect[:,0].argmax()
    
    gamma=spect[pkpt,1]/spect[pkpt,0]
    return gamma