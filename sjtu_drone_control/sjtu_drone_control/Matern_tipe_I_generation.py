import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.spatial import distance

# Parameters
area_size = 100  # Area size (100x100)
num_points = 30  # Desired number of users
min_distance = 1  # Minimum distance between users

# Generate candidate points with a Poisson process
lambda_poisson = num_points / (area_size ** 2)  # Expected density
num_candidates = int(1.5 * num_points)  # Generate more points than needed

# Generate random candidates in the area
candidates = np.random.uniform(0, area_size, (num_candidates, 2))

# Apply Matérn type-I filter (exclusion by minimum distance)
selected_points = []
for point in candidates:
    if all(distance.euclidean(point, p) >= min_distance for p in selected_points):
        selected_points.append(point)
    if len(selected_points) >= num_points:
        break

# Convert to numpy array
selected_points = np.array(selected_points)

# Plot the distribution
plt.figure(figsize=(6, 6))
plt.scatter(selected_points[:, 0], selected_points[:, 1], c='b', label='Users')
plt.xlim(0, area_size)
plt.ylim(0, area_size)
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('User Distribution (Matérn Type-I)')
plt.grid(True)
plt.legend()
plt.show()

# Create a DataFrame with the generated points
df = pd.DataFrame(selected_points, columns=["X", "Y"])
df.index = [f"P{i+1}" for i in range(len(df))]  # Point names

# Save to a CSV file
csv_filename = "matern_type1_distribution.csv"
df.to_csv(csv_filename, index_label="Point")

print(f"CSV file generated: {csv_filename}")
