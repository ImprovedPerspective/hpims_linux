
import subprocess
import sys
import os 
cwd = os.getcwd()
datDirectory  = cwd+"/./uvdma.dat"
p = subprocess.run([cwd+"/acquire"," 1 -ttledge -ttlinv -dcm -dcs 01 -f "+datDirectory])
data=uv.load2ch(datDirectory)
print(data)
