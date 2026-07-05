# plot_raceline.py
import yaml
import pandas as pd
import matplotlib.pyplot as plt
from PIL import Image
from pathlib import Path

MAP_YAML = "plot_csv_on_map/IV_2026_SIM_clean.yaml"
CSV_FILE = "plot_csv_on_map/generated_waypoints.csv"

with open(MAP_YAML, "r") as f:
    map_info = yaml.safe_load(f)

map_img_path = Path(MAP_YAML).parent / map_info["image"]
resolution = map_info["resolution"]
origin_x, origin_y, _ = map_info["origin"]

img = Image.open(map_img_path)
width, height = img.size

df = pd.read_csv(CSV_FILE)

# assumes CSV columns are x,y in world/map coordinates
x_world = df["x"]
y_world = df["y"]

# world -> pixel
x_pix = (x_world - origin_x) / resolution
y_pix = height - (y_world - origin_y) / resolution

plt.figure(figsize=(12, 9))
plt.imshow(img, cmap="gray", origin="upper")

plt.plot(x_pix, y_pix, linewidth=2, label="raceline")
plt.scatter(x_pix.iloc[0], y_pix.iloc[0], s=80, marker="o", label="start")
plt.scatter(x_pix.iloc[-1], y_pix.iloc[-1], s=80, marker="x", label="end")

plt.axis("equal")
plt.legend()
plt.title("Raceline over map")
plt.show()