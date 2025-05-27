# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import csv
import os

# File paths
waypoint_file = os.path.expanduser("~/catkin_ws/waypoints.csv")
trajectory_file = os.path.expanduser("~/catkin_ws/boat_trajectory.csv")
truth_csv = os.path.expanduser("~/catkin_ws/trash_ground_truth.csv")

# Load Assigned Waypoints
assigned_x, assigned_y = [], []
with open(waypoint_file, 'r') as f:
    reader = csv.reader(f)
    next(reader)  # Skip header
    for row in reader:
        assigned_x.append(float(row[0]))
        assigned_y.append(float(row[1]))

# Load Actual Boat Trajectory
actual_x, actual_y = [], []
with open(trajectory_file, 'r') as f:
    reader = csv.reader(f)
    next(reader)  # Skip header
    for row in reader:
        actual_x.append(float(row[0]))
        actual_y.append(float(row[1]))

# Load Trash Ground Truth
trash_truth_x, trash_truth_y = [], []
if os.path.exists(truth_csv):
    with open(truth_csv, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # Skip header
        for row in reader:
            trash_truth_x.append(float(row[0]))
            trash_truth_y.append(float(row[1]))

# Plot All
plt.figure(figsize=(10, 6))
plt.plot(assigned_x, assigned_y, 'bo--', label='Assigned Waypoints')
plt.plot(actual_x, actual_y, 'r.-', label='Boat Trajectory')
plt.scatter(trash_truth_x, trash_truth_y, c='black', marker='x', s=80, label='Ground Truth Trash')

plt.xlabel('X (meters)')
plt.ylabel('Y (meters)')
plt.title('Waypoints vs Trajectory vs Trash Locations')
plt.legend()
plt.grid(True)

output_path = os.path.expanduser("~/catkin_ws/plot_combined_groundtruth.png")
plt.savefig(output_path)
plt.show()

print("✅ Final plot saved as {}".format(output_path))

