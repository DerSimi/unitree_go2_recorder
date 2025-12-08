import numpy as np
import matplotlib.pyplot as plt

from data_loader import DataLoader

# Load data
data_loader = DataLoader(name='storage')
data_loader.print_debug()

# Hint: See this time steps as a reference, but do not rely on them, as they are inaccurate.
# # Time management
dt_arr = data_loader.get_dt()
print("dt mean and var:", dt_arr.mean(), dt_arr.var())

assert np.all(dt_arr > 0)
dt = dt_arr.mean()

plt.figure(figsize=(8, 4))
plt.hist(dt_arr, bins=50, color='skyblue', edgecolor='black')
plt.title(f"Dt Histogram, Samples: {len(dt_arr)}, Mean: {dt_arr.mean():.5f}, Std: {dt_arr.std():.5f}")
plt.xlabel("dt value")
plt.ylabel("Frequency")
plt.tight_layout()
plt.show()