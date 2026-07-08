#!/usr/bin/env python3
"""
Raceline Visual Editor v2 for F1TENTH / ROS2 map YAML + raceline CSV.

Features:
- Load a ROS map YAML file and display the map image in world coordinates.
- Load a raceline CSV with at least x,y columns.
- Left-click / left-drag: select waypoint(s)
    - Click near a point selects one point.
    - Drag a rectangle selects multiple points.
- Right-click / right-drag: move selected waypoint(s)
    - If right-click starts near an unselected point, that point becomes selected first.
- Middle-click / middle-drag: pan view.
- Mouse wheel: zoom centered on cursor.
- E key: insert waypoint after nearest point at current mouse position.
- Delete/Backspace: delete selected waypoint(s).
- Up/Down keys: adjust velocity if a velocity/speed column exists.
- Recompute yaw and curvature columns if present.
- Save edited CSV.

Install:
    pip install matplotlib numpy pyyaml pillow pandas

Run:
    python3 raceline_editor_gui_v2.py
"""

from __future__ import annotations

import math
import tkinter as tk
from tkinter import filedialog, messagebox
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd
import yaml
from PIL import Image

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.patches import Rectangle


DEFAULT_POINT_SIZE = 16
SELECTED_POINT_SIZE = 58
CLICK_SELECT_RADIUS_PX = 12


class RacelineEditor:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("F1TENTH Raceline Visual Editor v2")

        self.map_yaml_path: Optional[Path] = None
        self.csv_path: Optional[Path] = None
        self.df: Optional[pd.DataFrame] = None

        self.x_col = "x"
        self.y_col = "y"
        self.yaw_col: Optional[str] = None
        self.curv_col: Optional[str] = None
        self.vel_col: Optional[str] = None

        self.selected_indices: set[int] = set()

        self.left_selecting = False
        self.right_moving = False
        self.middle_panning = False

        self.select_start_data: Optional[tuple[float, float]] = None
        self.select_start_px: Optional[tuple[float, float]] = None
        self.move_last_data: Optional[tuple[float, float]] = None
        self.pan_last_data: Optional[tuple[float, float]] = None
        self.last_mouse_data: Optional[tuple[float, float]] = None

        self.undo_stack: list[pd.DataFrame] = []

        self.fig, self.ax = plt.subplots(figsize=(9, 7))
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_title("Load map YAML and raceline CSV")
        self.ax.set_xlabel("world x [m]")
        self.ax.set_ylabel("world y [m]")

        self.map_artist = None
        self.line_artist = None
        self.point_artist = None
        self.selected_artist = None
        self.selection_rect_artist: Optional[Rectangle] = None

        self.status = tk.StringVar(value="Ready.")

        self._build_ui()
        self._connect_events()

    def _build_ui(self):
        top = tk.Frame(self.root)
        top.pack(side=tk.TOP, fill=tk.X)

        tk.Button(top, text="Load Map YAML", command=self.load_map_yaml).pack(side=tk.LEFT, padx=3, pady=3)
        tk.Button(top, text="Load CSV", command=self.load_csv).pack(side=tk.LEFT, padx=3, pady=3)
        tk.Button(top, text="Save CSV", command=self.save_csv).pack(side=tk.LEFT, padx=3, pady=3)
        tk.Button(top, text="Save As...", command=self.save_csv_as).pack(side=tk.LEFT, padx=3, pady=3)
        tk.Button(top, text="Recompute Yaw/Curv", command=self.recompute_yaw_curvature).pack(side=tk.LEFT, padx=3, pady=3)
        tk.Button(top, text="Undo", command=self.undo).pack(side=tk.LEFT, padx=3, pady=3)

        help_text = (
            "Left drag: select | Right drag: move selected | Middle drag: pan | "
            "Wheel: zoom | E: insert | Delete: remove | Up/Down: velocity"
        )
        tk.Label(top, text=help_text).pack(side=tk.LEFT, padx=10)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        toolbar = NavigationToolbar2Tk(self.canvas, self.root)
        toolbar.update()

        tk.Label(self.root, textvariable=self.status, anchor="w").pack(side=tk.BOTTOM, fill=tk.X)

    def _connect_events(self):
        self.canvas.mpl_connect("button_press_event", self.on_press)
        self.canvas.mpl_connect("button_release_event", self.on_release)
        self.canvas.mpl_connect("motion_notify_event", self.on_motion)
        self.canvas.mpl_connect("key_press_event", self.on_key)
        self.canvas.mpl_connect("scroll_event", self.on_scroll)

    def push_undo(self):
        if self.df is not None:
            self.undo_stack.append(self.df.copy(deep=True))
            if len(self.undo_stack) > 50:
                self.undo_stack.pop(0)

    def undo(self):
        if not self.undo_stack:
            self.status.set("Nothing to undo.")
            return
        self.df = self.undo_stack.pop()
        self.selected_indices.clear()
        self.redraw()
        self.status.set("Undo complete.")

    def load_map_yaml(self):
        path = filedialog.askopenfilename(
            title="Open map YAML",
            filetypes=[("YAML files", "*.yaml *.yml"), ("All files", "*.*")]
        )
        if not path:
            return

        self.map_yaml_path = Path(path)
        try:
            with open(self.map_yaml_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)

            image_path = Path(data["image"])
            if not image_path.is_absolute():
                image_path = self.map_yaml_path.parent / image_path

            resolution = float(data["resolution"])
            origin = data.get("origin", [0.0, 0.0, 0.0])
            origin_x, origin_y = float(origin[0]), float(origin[1])

            img = Image.open(image_path).convert("L")
            arr = np.array(img)

            height, width = arr.shape[:2]
            extent = [
                origin_x,
                origin_x + width * resolution,
                origin_y,
                origin_y + height * resolution,
            ]

            self.ax.clear()
            self.ax.set_aspect("equal", adjustable="box")
            self.ax.set_xlabel("world x [m]")
            self.ax.set_ylabel("world y [m]")
            self.ax.set_title(str(self.map_yaml_path.name))

            self.map_artist = self.ax.imshow(
                arr,
                cmap="gray",
                origin="upper",
                extent=extent,
                alpha=0.85,
            )

            self.status.set(f"Loaded map: {image_path.name}, resolution={resolution}, origin={origin[:2]}")
            self.redraw(keep_limits=True)

        except Exception as e:
            messagebox.showerror("Map load error", str(e))

    def detect_columns(self):
        assert self.df is not None
        cols_lower = {str(c).lower().strip(): c for c in self.df.columns}

        def pick(candidates):
            for c in candidates:
                if c in cols_lower:
                    return cols_lower[c]
            return None

        if self.df.shape[1] < 2:
            raise ValueError("CSV must have at least two columns for x and y.")

        self.x_col = pick(["x", "pos_x", "world_x"]) or self.df.columns[0]
        self.y_col = pick(["y", "pos_y", "world_y"]) or self.df.columns[1]

        self.yaw_col = pick(["yaw", "heading", "theta"])
        self.curv_col = pick(["curvature", "curv", "kappa"])
        self.vel_col = pick(["velocity", "vel", "speed", "v"])

    def load_csv(self):
        path = filedialog.askopenfilename(
            title="Open raceline CSV",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if not path:
            return

        try:
            self.csv_path = Path(path)

            df = pd.read_csv(self.csv_path)
            lower_cols = [str(c).lower().strip() for c in df.columns]
            if not any(c in lower_cols for c in ["x", "pos_x", "world_x"]) or len(df.columns) < 2:
                df = pd.read_csv(self.csv_path, header=None)
                names = ["x", "y", "yaw", "curvature", "velocity"]
                df.columns = names[:len(df.columns)]

            self.df = df
            self.detect_columns()
            self.selected_indices.clear()
            self.undo_stack.clear()

            self.redraw()
            self.status.set(
                f"Loaded CSV: {self.csv_path.name} | x={self.x_col}, y={self.y_col}, "
                f"yaw={self.yaw_col}, curv={self.curv_col}, vel={self.vel_col}"
            )

        except Exception as e:
            messagebox.showerror("CSV load error", str(e))

    def save_csv(self):
        if self.df is None:
            messagebox.showwarning("No CSV", "Load a raceline CSV first.")
            return
        if self.csv_path is None:
            self.save_csv_as()
            return

        try:
            self.df.to_csv(self.csv_path, index=False)
            self.status.set(f"Saved CSV: {self.csv_path}")
        except Exception as e:
            messagebox.showerror("CSV save error", str(e))

    def save_csv_as(self):
        if self.df is None:
            messagebox.showwarning("No CSV", "Load a raceline CSV first.")
            return

        path = filedialog.asksaveasfilename(
            title="Save raceline CSV as",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if not path:
            return
        self.csv_path = Path(path)
        self.save_csv()

    def xy_array(self):
        assert self.df is not None
        return self.df[[self.x_col, self.y_col]].to_numpy(dtype=float)

    def nearest_idx(self, x, y) -> Optional[int]:
        if self.df is None or x is None or y is None or len(self.df) == 0:
            return None
        pts = self.xy_array()
        d2 = (pts[:, 0] - x) ** 2 + (pts[:, 1] - y) ** 2
        return int(np.argmin(d2))

    def nearest_idx_with_pixel_radius(self, event, radius_px=CLICK_SELECT_RADIUS_PX) -> Optional[int]:
        if self.df is None or event.xdata is None or event.ydata is None:
            return None

        pts = self.xy_array()
        display_pts = self.ax.transData.transform(pts)
        mouse = np.array([event.x, event.y])
        d2 = np.sum((display_pts - mouse) ** 2, axis=1)
        idx = int(np.argmin(d2))
        if math.sqrt(float(d2[idx])) <= radius_px:
            return idx
        return None

    def on_press(self, event):
        if event.inaxes != self.ax:
            return

        if event.xdata is not None and event.ydata is not None:
            self.last_mouse_data = (float(event.xdata), float(event.ydata))

        if self.df is None:
            return

        if event.button == 1:
            self.left_selecting = True
            self.select_start_data = (float(event.xdata), float(event.ydata))
            self.select_start_px = (float(event.x), float(event.y))

            idx = self.nearest_idx_with_pixel_radius(event)
            if idx is not None:
                self.selected_indices = {idx}
                self.status.set(f"Selected point {idx}.")
                self.redraw(keep_limits=True)

        elif event.button == 3:
            idx = self.nearest_idx_with_pixel_radius(event)

            if idx is not None and idx not in self.selected_indices:
                self.selected_indices = {idx}

            if self.selected_indices:
                self.push_undo()
                self.right_moving = True
                self.move_last_data = (float(event.xdata), float(event.ydata))
                self.status.set(f"Moving {len(self.selected_indices)} selected point(s).")
                self.redraw(keep_limits=True)

        elif event.button == 2:
            self.middle_panning = True
            self.pan_last_data = (float(event.xdata), float(event.ydata))

    def on_motion(self, event):
        if event.inaxes != self.ax:
            return

        if event.xdata is not None and event.ydata is not None:
            self.last_mouse_data = (float(event.xdata), float(event.ydata))

        if self.df is None:
            return

        if self.middle_panning and self.pan_last_data is not None:
            if event.xdata is None or event.ydata is None:
                return

            last_x, last_y = self.pan_last_data
            dx = last_x - float(event.xdata)
            dy = last_y - float(event.ydata)

            x0, x1 = self.ax.get_xlim()
            y0, y1 = self.ax.get_ylim()

            self.ax.set_xlim(x0 + dx, x1 + dx)
            self.ax.set_ylim(y0 + dy, y1 + dy)
            self.canvas.draw_idle()
            return

        if self.right_moving and self.move_last_data is not None and self.selected_indices:
            if event.xdata is None or event.ydata is None:
                return

            last_x, last_y = self.move_last_data
            current_x, current_y = float(event.xdata), float(event.ydata)
            dx = current_x - last_x
            dy = current_y - last_y

            for idx in self.selected_indices:
                if 0 <= idx < len(self.df):
                    self.df.at[idx, self.x_col] = float(self.df.at[idx, self.x_col]) + dx
                    self.df.at[idx, self.y_col] = float(self.df.at[idx, self.y_col]) + dy

            self.move_last_data = (current_x, current_y)
            self.redraw(keep_limits=True, fast=True)
            self.status.set(f"Moved {len(self.selected_indices)} selected point(s): dx={dx:.3f}, dy={dy:.3f}")
            return

        if self.left_selecting and self.select_start_data is not None:
            if event.xdata is None or event.ydata is None:
                return

            x0, y0 = self.select_start_data
            x1, y1 = float(event.xdata), float(event.ydata)

            if self.select_start_px is not None:
                dx_px = abs(float(event.x) - self.select_start_px[0])
                dy_px = abs(float(event.y) - self.select_start_px[1])
                if dx_px < 4 and dy_px < 4:
                    return

            self.update_selection_rect(x0, y0, x1, y1)
            self.canvas.draw_idle()

    def on_release(self, event):
        if self.df is not None and self.left_selecting and self.select_start_data is not None:
            if event.inaxes == self.ax and event.xdata is not None and event.ydata is not None:
                dragged = False
                if self.select_start_px is not None:
                    dx_px = abs(float(event.x) - self.select_start_px[0])
                    dy_px = abs(float(event.y) - self.select_start_px[1])
                    dragged = dx_px >= 4 or dy_px >= 4

                if dragged:
                    x0, y0 = self.select_start_data
                    x1, y1 = float(event.xdata), float(event.ydata)
                    self.select_points_in_box(x0, y0, x1, y1)
                    self.status.set(f"Selected {len(self.selected_indices)} point(s).")

            self.remove_selection_rect()
            self.redraw(keep_limits=True)

        self.left_selecting = False
        self.right_moving = False
        self.middle_panning = False
        self.select_start_data = None
        self.select_start_px = None
        self.move_last_data = None
        self.pan_last_data = None

    def on_scroll(self, event):
        if event.inaxes != self.ax or event.xdata is None or event.ydata is None:
            return

        scale = 0.9 if event.button == "up" else 1.1

        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        xdata = float(event.xdata)
        ydata = float(event.ydata)

        old_width = xlim[1] - xlim[0]
        old_height = ylim[1] - ylim[0]
        new_width = old_width * scale
        new_height = old_height * scale

        relx = (xlim[1] - xdata) / old_width
        rely = (ylim[1] - ydata) / old_height

        self.ax.set_xlim(xdata - new_width * (1 - relx), xdata + new_width * relx)
        self.ax.set_ylim(ydata - new_height * (1 - rely), ydata + new_height * rely)
        self.canvas.draw_idle()

    def on_key(self, event):
        if self.df is None:
            return

        key = event.key.lower() if isinstance(event.key, str) else event.key

        if key in ["delete", "backspace"]:
            if self.selected_indices:
                self.push_undo()
                deleted = sorted(self.selected_indices)
                self.df = self.df.drop(index=self.df.index[deleted]).reset_index(drop=True)
                self.selected_indices.clear()
                self.redraw(keep_limits=True)
                self.status.set(f"Deleted {len(deleted)} point(s).")

        elif key in ["up", "down"] and self.selected_indices:
            if self.vel_col is None:
                self.status.set("No velocity/speed column found.")
                return
            self.push_undo()
            delta = 0.1 if key == "up" else -0.1

            for idx in self.selected_indices:
                old_v = float(self.df.at[idx, self.vel_col])
                self.df.at[idx, self.vel_col] = max(0.0, old_v + delta)

            self.redraw(keep_limits=True)
            self.status.set(f"Adjusted velocity for {len(self.selected_indices)} point(s) by {delta:+.1f}.")

        elif key == "e":
            self.insert_at_last_mouse()

        elif key == "escape":
            self.selected_indices.clear()
            self.remove_selection_rect()
            self.redraw(keep_limits=True)
            self.status.set("Selection cleared.")

    def insert_at_last_mouse(self):
        if self.df is None:
            return

        if self.last_mouse_data is None:
            self.status.set("Move mouse over the map first, then press E to insert.")
            return

        x, y = self.last_mouse_data
        idx = self.nearest_idx(x, y)
        if idx is None:
            return

        self.push_undo()
        self.insert_point_after(idx, x, y)
        self.selected_indices = {idx + 1}
        self.redraw(keep_limits=True)
        self.status.set(f"Inserted point {idx + 1} after nearest point {idx} at x={x:.3f}, y={y:.3f}.")

    def update_selection_rect(self, x0, y0, x1, y1):
        self.remove_selection_rect()
        xmin, xmax = sorted([x0, x1])
        ymin, ymax = sorted([y0, y1])
        self.selection_rect_artist = Rectangle(
            (xmin, ymin),
            xmax - xmin,
            ymax - ymin,
            fill=False,
            linestyle="--",
            linewidth=1.2,
        )
        self.ax.add_patch(self.selection_rect_artist)

    def remove_selection_rect(self):
        if self.selection_rect_artist is not None:
            try:
                self.selection_rect_artist.remove()
            except Exception:
                pass
            self.selection_rect_artist = None

    def select_points_in_box(self, x0, y0, x1, y1):
        assert self.df is not None
        pts = self.xy_array()
        xmin, xmax = sorted([x0, x1])
        ymin, ymax = sorted([y0, y1])

        mask = (
            (pts[:, 0] >= xmin) &
            (pts[:, 0] <= xmax) &
            (pts[:, 1] >= ymin) &
            (pts[:, 1] <= ymax)
        )
        self.selected_indices = set(np.nonzero(mask)[0].astype(int).tolist())

    def insert_point_after(self, idx: int, x: float, y: float):
        assert self.df is not None

        new_row = self.df.iloc[idx].copy()
        new_row[self.x_col] = float(x)
        new_row[self.y_col] = float(y)

        upper = self.df.iloc[:idx + 1]
        lower = self.df.iloc[idx + 1:]
        self.df = pd.concat([upper, pd.DataFrame([new_row]), lower], ignore_index=True)

    def recompute_yaw_curvature(self):
        if self.df is None:
            messagebox.showwarning("No CSV", "Load a raceline CSV first.")
            return

        if len(self.df) < 3:
            messagebox.showwarning("Too few points", "Need at least 3 points.")
            return

        self.push_undo()
        pts = self.xy_array()
        x = pts[:, 0]
        y = pts[:, 1]

        x_prev, x_next = np.roll(x, 1), np.roll(x, -1)
        y_prev, y_next = np.roll(y, 1), np.roll(y, -1)

        dx = x_next - x_prev
        dy = y_next - y_prev
        yaw = np.arctan2(dy, dx)

        curv = np.zeros(len(x), dtype=float)
        for i in range(len(x)):
            p0 = pts[(i - 1) % len(pts)]
            p1 = pts[i]
            p2 = pts[(i + 1) % len(pts)]

            a = np.linalg.norm(p1 - p0)
            b = np.linalg.norm(p2 - p1)
            c = np.linalg.norm(p2 - p0)

            area2 = abs(np.cross(p1 - p0, p2 - p0))
            denom = a * b * c
            if denom > 1e-9:
                curv[i] = 2.0 * area2 / denom
            else:
                curv[i] = 0.0

        if self.yaw_col is None:
            self.df["yaw"] = yaw
            self.yaw_col = "yaw"
        else:
            self.df[self.yaw_col] = yaw

        if self.curv_col is None:
            self.df["curvature"] = curv
            self.curv_col = "curvature"
        else:
            self.df[self.curv_col] = curv

        self.redraw(keep_limits=True)
        self.status.set("Recomputed yaw and curvature.")

    def redraw(self, keep_limits=False, fast=False):
        if keep_limits:
            xlim = self.ax.get_xlim()
            ylim = self.ax.get_ylim()
        else:
            xlim = ylim = None

        for artist in [self.line_artist, self.point_artist, self.selected_artist]:
            if artist is not None:
                try:
                    artist.remove()
                except Exception:
                    pass

        self.line_artist = None
        self.point_artist = None
        self.selected_artist = None

        if self.df is not None and len(self.df) > 0:
            pts = self.xy_array()
            x, y = pts[:, 0], pts[:, 1]

            self.line_artist, = self.ax.plot(x, y, "-", linewidth=1.2)
            self.point_artist = self.ax.scatter(x, y, s=DEFAULT_POINT_SIZE)

            valid_selected = sorted(i for i in self.selected_indices if 0 <= i < len(self.df))
            if valid_selected:
                sx = x[valid_selected]
                sy = y[valid_selected]
                self.selected_artist = self.ax.scatter(sx, sy, s=SELECTED_POINT_SIZE, marker="o")
                self.ax.set_title(f"Selected {len(valid_selected)} waypoint(s)")

            if not keep_limits:
                pad = 1.0
                self.ax.set_xlim(float(np.min(x)) - pad, float(np.max(x)) + pad)
                self.ax.set_ylim(float(np.min(y)) - pad, float(np.max(y)) + pad)

        if keep_limits and xlim is not None and ylim is not None:
            self.ax.set_xlim(xlim)
            self.ax.set_ylim(ylim)

        if fast:
            self.canvas.draw_idle()
        else:
            self.canvas.draw()


def main():
    root = tk.Tk()
    root.geometry("1200x850")
    RacelineEditor(root)
    root.mainloop()


if __name__ == "__main__":
    main()
