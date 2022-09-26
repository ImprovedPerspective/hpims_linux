# -*- coding: utf-8 -*-
"""
Created on Wed Aug 31 15:13:12 2022

@author: Katie
"""


import numpy as np
import matplotlib.pyplot as plt

Z=np.load('Z_83122.npy')

Zmag=np.abs(Z)

PdBm=np.arange(-31,-19,2)
Pin=np.power(10,(PdBm+65)/10)*.001

Vout=np.linspace(0,40,16)


plt.scatter(Pin,Vout,Zmag)
