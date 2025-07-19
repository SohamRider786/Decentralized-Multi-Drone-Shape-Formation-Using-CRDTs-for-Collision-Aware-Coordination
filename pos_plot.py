import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load drone positions
df = pd.read_csv("drone_positions.csv")

# Styling
plt.figure(figsize=(12, 8))
plt.title("Drone Transition: Inverted V ➝ Circle", fontsize=16)
plt.xlabel("X Coordinate")
plt.ylabel("Y Coordinate")
plt.grid(True, linestyle='--', alpha=0.5)

# Use consistent color palette
init_color = '#1f77b4'  # Blue
final_color = '#d62728' # Red
arrow_color = '#999999' # Gray

# Plot arrows from initial to final
for _, row in df.iterrows():
    x_init, y_init = row['x_init'], row['y_init']
    x_final, y_final = row['x_final'], row['y_final']
    drone_id = int(row['id'])

    # Draw arrow
    plt.arrow(x_init, y_init, x_final - x_init, y_final - y_init,
              head_width=1.5, head_length=3, fc=arrow_color, ec=arrow_color, alpha=0.6, length_includes_head=True)

    # Initial point
    plt.plot(x_init, y_init, 'o', color=init_color)
    plt.text(x_init + 1, y_init + 1, str(drone_id), fontsize=7, color=init_color)

    # Final point
    plt.plot(x_final, y_final, 'o', color=final_color)
    plt.text(x_final + 1, y_final + 1, str(drone_id), fontsize=7, color=final_color)

# Draw connected inverted V shape (sorted by x_init)
df_v = df.sort_values(by='x_init')
plt.plot(df_v['x_init'], df_v['y_init'], linestyle=':', color=init_color, linewidth=2, label='Initial Inverted V')

# Draw circle outline from final positions
x_final = df['x_final'].values
y_final = df['y_final'].values
center_x, center_y = np.mean(x_final), np.mean(y_final)
radii = np.sqrt((x_final - center_x)**2 + (y_final - center_y)**2)
avg_radius = np.mean(radii)

theta = np.linspace(0, 2 * np.pi, 300)
circle_x = center_x + avg_radius * np.cos(theta)
circle_y = center_y + avg_radius * np.sin(theta)
plt.plot(circle_x, circle_y, 'r--', linewidth=2, label='Final Circle')

# Final touches
plt.legend(fontsize=10, loc='upper right')
plt.axis('equal')
plt.tight_layout()
plt.savefig("enhanced_drone_plot.png", dpi=300)
plt.show()
