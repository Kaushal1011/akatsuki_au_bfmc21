import matplotlib.pyplot as plt
import numpy as np
import joblib

arr=joblib.load("area.z")
# print(arr)
x=[i for i in range(len(arr))]

plt.subplot(121)

arrnp=np.array([i for i in arr if i!=0],dtype=np.uint64)
print(arrnp)
plt.subplot(1, 2, 1)
plt.scatter(x,arr)
plt.subplot(1, 2, 2)
plt.hist(arrnp)

plt.show()

