import numpy as np

with open('test.npy', 'rb') as f:
    x = np.load(f)
    y = np.load(f)
    d = np.load(f)
    yv = np.load(f)
    ya = np.load(f)

print(y)
print((y[1:] - y[:-1]) / 0.1)
# print(yv)