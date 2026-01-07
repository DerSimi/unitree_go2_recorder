import numpy as np
import matplotlib.pyplot as plt

from data_loader import DataLoader

# Load data
data_loader = DataLoader(name='test')
data_loader.print_debug()

# # Time management
dt_arr = data_loader.get_dt()
print("dt mean and var:", dt_arr.mean(), dt_arr.var())

# assert np.all(dt_arr > 0)
dt = dt_arr.mean()

dt_arr = dt_arr[100:-100]
mask = dt_arr <= 0.4
removed = (~mask).sum()
if removed:
    print(f"Removed {removed} dt samples > 0.4s")
dt_arr = dt_arr[mask]
linspace = np.linspace(0, data_loader.get_duration(), dt_arr.shape[0])

plt.figure(figsize=(8, 4))
plt.scatter(linspace, 1/dt_arr, color='skyblue', edgecolor='black')
plt.title(f"Recording Frequency over Time, Samples: {len(dt_arr)}, Mean: {dt_arr.mean():.5f}s, Std: {dt_arr.std():.5f}s")
plt.xlabel("Time (s)")
plt.ylabel("Frequency (Hz)")
plt.tight_layout()
plt.show()