import numpy as np

mat = np.array([[1,2],[2,3]])
ref = np.array([1,1])

delta = np.subtract(mat , ref)

print(delta)