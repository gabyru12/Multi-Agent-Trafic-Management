import matplotlib.pyplot as plt
import numpy as np

# Create some x values
x = np.linspace(0, 1, 200)

# Linear function starting at (0.0, 0.0)
y_linear = x   # change this if your function is different

plt.figure(figsize=(8, 5))

# Plot the linear function
plt.plot(x, y_linear, label="SpawnTimer", linewidth=2)

# Plot the striped horizontal line at y = 0.25
plt.axhline(
    y=0.25,
    color='black',
    linestyle=(0, (5, 5)),   # dashed / striped pattern
    linewidth=1.5,
    label="Lower Bound = 0.25"
)

# Labels and legend
plt.xlabel("X")
plt.ylabel("Time Interval")
plt.title("Spawn Timer Linear Curve with Lower Bound")
plt.grid(True)
plt.legend()

plt.show()
