import numpy as np

#Desired Power Sweep
PdBm=np.arange(-25,-19,2)

#Desired Vout sweel
Vout=np.linspace(0,40,2)
Zmatrix=np.zeros((PdBm.size,Vout.size),dtype=complex)

#data = (np.array([Vout,PdBm,Zmatrix],dtype=object))
with open('test.npy', 'wb') as f:
    np.save(f, Vout)
    np.save(f, PdBm)
    np.save(f, Zmatrix)

with open('test.npy', 'rb') as f:
    v = np.load(f)
    p = np.load(f)
    z = np.load(f)
print(v)
print(p)
print(z)
