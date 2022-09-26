import numpy as np
import matplotlib.pyplot as plt

filename = "data/2022-09-26_13-43-54.npy" #input("Input filename Here")

with open(filename, 'rb') as f:
    v = np.load(f)
    p = np.load(f)
    z = np.load(f)

print(v)
print(p)
print(z)
