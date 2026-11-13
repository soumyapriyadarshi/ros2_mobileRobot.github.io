
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys

if len(sys.argv) < 2:
    print("Usage: python3 plot_results.py results/run.csv")
    sys.exit(1)

df = pd.read_csv(sys.argv[1])

# 1. Robot trajectory
plt.figure()
plt.plot(df['x'], df['y'], label='Robot trajectory')
plt.scatter(df['x'].iloc[0], df['y'].iloc[0],
            marker='o', label='Start')
plt.scatter(8, 8, marker='*', s=150, label='Goal')
plt.xlabel('X position (m)')
plt.ylabel('Y position (m)')
plt.title('Robot Trajectory')
plt.axis('equal')
plt.grid()
plt.legend()
plt.savefig('trajectory.png', dpi=300)

# 2. Distance to goal
plt.figure()
plt.plot(df['time'], df['distance_to_goal'])
plt.xlabel('Time (seconds)')
plt.ylabel('Distance to goal (m)')
plt.title('Distance to Goal vs Time')
plt.grid()
plt.savefig('goal_distance.png', dpi=300)

# 3. Obstacle clearance
plt.figure()
plt.plot(df['time'], df['min_obstacle_distance'])
plt.xlabel('Time (seconds)')
plt.ylabel('Minimum LiDAR range (m)')
plt.title('Obstacle Clearance vs Time')
plt.grid()
plt.savefig('obstacle_clearance.png', dpi=300)

# 4. Velocity
plt.figure()
plt.plot(df['time'], df['linear_velocity'],
         label='Linear velocity')
plt.plot(df['time'], df['angular_velocity'],
         label='Angular velocity')
plt.xlabel('Time (seconds)')
plt.ylabel('Velocity')
plt.title('Robot Velocity vs Time')
plt.grid()
plt.legend()
plt.savefig('velocity.png', dpi=300)

# 5. Calculate path length
dx = np.diff(df['x'])
dy = np.diff(df['y'])

path_length = np.sum(np.sqrt(dx**2 + dy**2))

print(f"Path length: {path_length:.3f} m")
print(f"Final goal distance: {df['distance_to_goal'].iloc[-1]:.3f} m")
print(f"Minimum LiDAR range: {df['min_obstacle_distance'].min():.3f} m")
print(f"Simulation recording duration: {df['time'].iloc[-1]:.2f} s")

plt.show()
