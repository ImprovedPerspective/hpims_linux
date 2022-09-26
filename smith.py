
#!/usr/bin/env python3
import matplotlib.pylab as pl 
import smithplot
import numpy as np

from smithplot import SmithAxes

fig = pl.figure() 
ax1 = fig.add_subplot(111,projection='smith') 
pl.plot(200 + 100j, datatype=SmithAxes.Z_PARAMETER)

pl.show(block=False)

pl.plot(100 + 100j, datatype=SmithAxes.Z_PARAMETER)
pl.show(block=False)

dat = np.array([0,50+0j,25,100j])

pl.plot(dat,equipoints=4,datatype=SmithAxes.Z_PARAMETER)
pl.show(block=False)

input("Wait for Final Input")
