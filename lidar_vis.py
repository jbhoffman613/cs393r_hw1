import numpy as np
import matplotlib.pyplot as plt

f = open('temp.txt')
d = int(f.readline())
print(d)

x = []
y = []

for i in range(d):
    line = f.readline().split()
    x.append(float(line[2]))
    y.append(float(line[3]))

print(x)
print(y)

plt.scatter(x, y)
plt.show()