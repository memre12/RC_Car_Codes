import numpy as np
import pandas as pd
from scipy.signal import savgol_filter

# Load CSV (skip header)
df = pd.read_csv("path_speed.csv", skiprows=1, header=None, names=["x", "y", "speed"])

x = df["x"].astype(float).values
y = df["y"].astype(float).values
speed = df["speed"].astype(float).values   # Buna dokunmuyoruz

# Close the loop (first point to the end)
x = np.append(x, x[0])
y = np.append(y, y[0])
speed = np.append(speed, speed[0])  # speed değişmeyecek ama loop için ekliyoruz

# --- Apply smooth filter (NO SLOLOM) ---
# window_length MUST be odd → adjust as needed
WINDOW = 31

x_s = savgol_filter(x, WINDOW, polyorder=3, mode="wrap")
y_s = savgol_filter(y, WINDOW, polyorder=3, mode="wrap")

# Save output (speed untouched)
smooth_df = pd.DataFrame({
    "x": x_s,
    "y": y_s,
    "desired_speed": speed   # speed aynı!
})

smooth_df.to_csv("smooth_loop_path_2.csv", index=False)

print("Saved smooth_loop_path.csv")
