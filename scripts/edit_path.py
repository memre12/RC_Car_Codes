import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

CSV_PATH = "/home/xavier/race_car_ws/rc_pure_pursuit/big_hallway_adjusted.csv"
OUTPUT_PATH = "big_hallway_adjusted_with_speed.csv"
DEFAULT_SPEED = 2.0  # default speed for every point

# ============================
# Load CSV
# ============================
df = pd.read_csv(CSV_PATH)

# Check required columns
if not {"x", "y"}.issubset(df.columns):
    raise ValueError("CSV must contain columns 'x' and 'y'")

# Add speed column if missing
if "desired_speed" not in df.columns:
    df["desired_speed"] = DEFAULT_SPEED

points = df[["x", "y"]].values

# ============================
# Plot
# ============================
fig, ax = plt.subplots()
scatter = ax.scatter(
    df["x"], df["y"], 
    c=df["desired_speed"], cmap="coolwarm", s=15
)
plt.title("Click a point → enter speed in terminal")
plt.xlabel("x")
plt.ylabel("y")
plt.colorbar(scatter, label="desired_speed (m/s)")
plt.gca().invert_yaxis()  # optional for ROS-like coordinate systems

# ============================
# Helper: Find nearest point
# ============================
def nearest_point(x, y):
    distances = np.linalg.norm(points - np.array([x, y]), axis=1)
    return np.argmin(distances)

# ============================
# On-click event
# ============================
def click_event(event):
    if event.xdata is None or event.ydata is None:
        return

    idx = nearest_point(event.xdata, event.ydata)

    print(f"\nClicked point index = {idx}")
    print(f"Path point = ({df.loc[idx,'x']:.3f}, {df.loc[idx,'y']:.3f})")
    
    try:
        new_speed = float(input("Enter desired speed (m/s): "))
    except:
        print("Invalid input, skipping.")
        return

    df.loc[idx, "desired_speed"] = new_speed

    # Update colors
    scatter.set_array(df["desired_speed"])
    fig.canvas.draw_idle()

plt.connect("button_press_event", click_event)

print("\nInstructions:")
print(" 1) Click on a path point")
print(" 2) Enter speed in terminal")
print(" 3) Close plot window when finished\n")

plt.show()

# ============================
# Save updated CSV
# ============================
df.to_csv(OUTPUT_PATH, index=False)
print(f"\nSaved updated CSV → {OUTPUT_PATH}")
