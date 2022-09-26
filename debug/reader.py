import numpy as np
import uvfunc as uv
import matplotlib.pyplot as plt 

data=uv.load2ch('uvdma.dat')
print("hello word")
print(data.shape)
#fig=figure()
plt.plot(data[:,0])
plt.show()


