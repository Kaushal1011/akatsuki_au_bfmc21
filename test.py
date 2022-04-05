import numpy as np
import SharedArray as sa

# Create an array in shared memory.
a = sa.create("shm://test", 10)

# Attach it as a different array. This can be done from another
# python interpreter as long as it runs on the same computer.
b = sa.attach("shm://test")

# See how they are actually sharing the same memory.
a[0] = 42
print(b[0])

# Destroying a does not affect b.
del a
print(b[0])

# See how "test" is still present in shared memory even though we
# destroyed the array a. This method only works on Linux.
sa.list()

# Now destroy the array "test" from memory.
sa.delete("test")

# The array b is still there, but once you destroy it then the
# data is gone for real.
print(b[0])