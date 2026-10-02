import math
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from pathlib import Path
from typing import Optional

import numpy as np
import math
import pandas as pd
import yaml
import json
from PIL import Image

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.patches import Rectangle

DEFAULT_REDRAW_RES = 0.05
SELECTOR_SIZE_PX = 7

MAX_UNDO = 25

class RacePath:
    def __init__(self):
        self.nodes = []
        self.segments = []

    def sample(self, resolution):
        # Return list of tuple (x,y)
        # INCLUDING START AND END POINTS
        if resolution <= 0:
            raise ValueError("Resolution must be positive!")

        # No nodes = no path
        if not self.nodes:
            return []

        # One node with no segments = just that node
        if not self.segments:
            return [self.nodes[0].to_tuple()]

        racepath = []
        for segment in self.segments:
            racepath.extend(segment.sample(resolution))
        racepath.append(self.segments[-1].end.to_tuple())

        return racepath

    def from_dict(self, data):
        self.nodes = []
        self.segments = []
        Node.current_id = 0

        nodes_by_id = {}
        for node_id, n in data["nodes"].items():
            node_id = int(node_id)
            nodes_by_id[node_id] = Node(n["X"], n["Y"], node_id)
            nodes_by_id[node_id].prop = n["prop"]

        for node_id in sorted(nodes_by_id):
            self.nodes.append(nodes_by_id[node_id])

        for _, s in data["segments"].items():
            if s["type"] == "line":
                start_id = s["start_id"]
                end_id = s["end_id"]
                self.segments.append(LineSegment(self.nodes[start_id], self.nodes[end_id]))
            elif s["type"] == "bezier":
                start_id = s["start_id"]
                end_id = s["end_id"]
                cpt1x = s["cpt1"]["X"]
                cpt1y = s["cpt1"]["Y"]
                cpt2x = s["cpt2"]["X"]
                cpt2y = s["cpt2"]["Y"]
                self.segments.append(BezierSegment(self.nodes[start_id], 
                                                        self.nodes[end_id], 
                                                        ControlPoint(cpt1x,cpt1y),
                                                        ControlPoint(cpt2x,cpt2y)
                                                                        ))
        
    def to_dict(self):
        data = {}
        data["nodes"] = {}
        for node in self.nodes:
            data["nodes"][str(node.id)] = {}
            data["nodes"][str(node.id)]["X"] = node.x
            data["nodes"][str(node.id)]["Y"] = node.y
            data["nodes"][str(node.id)]["prop"] = node.prop

        data["segments"] = {}
        for i, segment in enumerate(self.segments):
            data["segments"][str(i)] = {}
            if isinstance(segment, LineSegment):
                data["segments"][str(i)]["type"] = "line"
                data["segments"][str(i)]["start_id"] = segment.start.id
                data["segments"][str(i)]["end_id"] = segment.end.id
            elif isinstance(segment, BezierSegment):
                data["segments"][str(i)]["type"] = "bezier"
                data["segments"][str(i)]["start_id"] = segment.start.id
                data["segments"][str(i)]["end_id"] = segment.end.id
                data["segments"][str(i)]["cpt1"] = {}
                data["segments"][str(i)]["cpt1"]["X"] = segment.cpt1.x
                data["segments"][str(i)]["cpt1"]["Y"] = segment.cpt1.y
                data["segments"][str(i)]["cpt2"] = {}
                data["segments"][str(i)]["cpt2"]["X"] = segment.cpt2.x
                data["segments"][str(i)]["cpt2"]["Y"] = segment.cpt2.y

        return data

    def get_control_points(self):
        cpts = []
        for segment in self.segments:
            if isinstance(segment, BezierSegment):
                cpts.extend([segment.cpt1, segment.cpt2])
        return cpts

    def create_node(self, pos, connectLast = True, connectType = "bezier"):
        # pos is tuple of x,y
        newNode = Node(pos[0], pos[1])

        if connectLast and self.nodes:
            lastNode = self.nodes[-1]
            if connectType == "line":
                self.segments.append(LineSegment(lastNode, newNode))
            elif connectType == "bezier":
                # Create two middle control nodes
                cpt1 = ControlPoint(
                    (newNode.x - lastNode.x) * 1.0 / 3.0 + lastNode.x, 
                    (newNode.y - lastNode.y) * 1.0 / 3.0 + lastNode.y, 
                )
                cpt2 = ControlPoint(
                    (newNode.x - lastNode.x) * 2.0 / 3.0 + lastNode.x, 
                    (newNode.y - lastNode.y) * 2.0 / 3.0 + lastNode.y, 
                )

                self.segments.append(BezierSegment(lastNode, newNode, cpt1, cpt2))
            else:
                raise ValueError(f"Unknown segment type: {connectType}")
        self.nodes.append(newNode)
        return newNode

class ControlPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def to_tuple(self):
        return (self.x, self.y)

class Node:
    current_id = 0
    def __init__(self, x, y, id=None):
        if id is not None:
            self.id = id
            Node.current_id = max(Node.current_id, id + 1)
        else:
            self.id = Node.current_id
            Node.current_id += 1
        
        self.x = x
        self.y = y
        self.prop = {}

    def to_tuple(self):
        return (self.x, self.y)

class Segment:
    def __init__(self, start, end):
        self.start = start
        self.end = end

    def sample(self, resolution):
        # Return list of tuples (x,y)
        # DOES NOT INCLUDE ENDPOINT, race path sampling will add a point at the end for you
        if resolution <= 0:
            raise ValueError("Resolution must be positive!")
        
        dt = resolution / self.length()
        path = []
        for t in np.arange(0, 1, dt):
            path.append(self.position(t))
        return path

    def position(self, t):
        raise NotImplementedError

    def curvature(self, t):
        raise NotImplementedError

    def length(self):
        raise NotImplementedError

class LineSegment(Segment):
    def position(self, t):
        # t is a float that ranges from 0 to 1
        # return a tuple (x,y)
        x = (self.end.x - self.start.x) * t + self.start.x
        y = (self.end.y - self.start.y) * t + self.start.y
        return (x, y)
    def curvature(self, t):
        return 0.0
    def length(self):
        return math.dist(self.start.to_tuple(), self.end.to_tuple())

class BezierSegment(Segment):
    def __init__(self, start, end, ctrl_pt1, ctrl_pt2):
        super().__init__(start, end)

        # Control points (Control Points)
        self.cpt1 = ctrl_pt1
        self.cpt2 = ctrl_pt2

    def position(self, t):
        x = self.start.x * (1-t) ** 3 + \
            self.cpt1.x * 3 * t * (1-t) ** 2 + \
            self.cpt2.x * 3 * t ** 2 * (1-t) + \
            self.end.x * t ** 3
        y = self.start.y * (1-t) ** 3 + \
            self.cpt1.y * 3 * t * (1-t) ** 2 + \
            self.cpt2.y * 3 * t ** 2 * (1-t) + \
            self.end.y * t ** 3
        return (x,y)

    def curvature(self, t):
        P0 = np.array([self.start.x, self.start.y])
        P1 = np.array([self.cpt1.x, self.cpt1.y])
        P2 = np.array([self.cpt2.x, self.cpt2.y])
        P3 = np.array([self.end.x, self.end.y])

        dB = (3 * (1 - t)**2 * (P1 - P0) + \
            6 * (1 - t) * t * (P2 - P1) + \
            3 * t**2 * (P3 - P2))
        dx, dy = dB[0], dB[1]

        ddB = (6 * (1 - t) * (P2 - 2 * P1 + P0) + \
           6 * t * (P3 - 2 * P2 + P1))
        ddx, ddy = ddB[0], ddB[1]

        numerator = dx * ddy - dy * ddx
        denominator = (dx**2 + dy**2)**1.5

        if denominator > 0:
            kappa = numerator / denominator
        else:
            kappa = 0.0

        return kappa

    def length(self):
        # Currently use approximation
        ts = np.linspace(0, 1, 100)
        pts = [self.position(t) for t in ts]

        total = 0.0

        for i in range(len(pts) - 1):
            total += math.dist(pts[i], pts[i + 1])

        return total

class RacelineEditor:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Roboracer Raceline Patcher v0.2.1")

        # Map and Waypoint Paths
        self.map_yaml_path: Optional[Path] = None
        self.csv_path: Optional[Path] = None
        self.rpp_path: Optional[Path] = None

        # Old CSV Implementation
        '''
        # Waypoint DataFrame
        self.df: Optional[pd.DataFrame] = None
        # Column Names, Processed by detect_columns()
        self.x_col = "x" # Mandatory for Waypoint CSV
        self.y_col = "y" # Mandatory for Waypoint CSV
        self.yaw_col: Optional[str] = None
        self.curv_col: Optional[str] = None
        self.vel_col: Optional[str] = None
        '''

        # Initialize List of Nodes and Segments
        self.path = RacePath()


        # Setup fig for Display
        self.fig, self.ax = plt.subplots(figsize=(9, 7))
        self.ax.set_aspect(3/4, adjustable="box")
        self.ax.set_xlabel("world x [m]")
        self.ax.set_ylabel("world y [m]")

        # Visual Artists
        self.map_artist = None
        self.line_artist = None
        self.point_artist = None
        self.control_point_artist = None
        self.selected_artist = None
        self.selection_rect_artist: Optional[Rectangle] = None

        # Bottom Status Text
        self.status = tk.StringVar(value="Idle")

        # Editor Variables
        self.selected = []
        self.undo_stack = []
        self.redo_stack = []

        # Dragging 
        self.dragging = False
        self.drag_last_x = None
        self.drag_last_y = None

        # Panning
        self.panning = False
        self.pan_last_x = None
        self.pan_last_y = None

        self._build_ui()
        self._connect_events()

    def _build_ui(self):
        '''
        ------------------
        MENU BAR
        ------------------
        '''
        menu_font = ("Arial", 12)
        menu_bar = tk.Menu(self.root, font=menu_font)

        # FILE DROPDOWN
        file_menu = tk.Menu(menu_bar, tearoff=0, font=menu_font)

        file_menu.add_command(
            label = "Load Map",
            command=self.load_map_yaml
        )
        file_menu.add_separator()
        file_menu.add_command(
            label="Open Raceline Project",
            command=self.open_raceline_rpp
        )
        file_menu.add_command(
            label="Save Raceline Project",
            command=self.save_raceline_rpp
        )
        file_menu.add_command(
            label="Save Raceline Project As",
            command=self.save_raceline_rpp_as
        )
        file_menu.add_separator()
        file_menu.add_command(
            label="Save Raceline CSV",
            command=self.save_raceline_csv
        )
        file_menu.add_command(
            label="Save Raceline CSV As",
            command=self.save_raceline_csv_as
        )

        # EDIT DROPDOWN
        edit_menu = tk.Menu(menu_bar, tearoff=0, font=menu_font)

        # Attach main menu bars together
        menu_bar.add_cascade(
            label="File",
            menu=file_menu
        )
        menu_bar.add_cascade(
            label='Edit',
            menu=edit_menu
        )
        self.root.config(menu=menu_bar)

        '''
        ------------------
        STATUS BAR
        ------------------
        '''
        tk.Label(self.root, textvariable=self.status, anchor="w").pack(side=tk.BOTTOM, fill=tk.X)

        '''
        ------------------
        CONTENT
        ------------------
        '''
        content = tk.Frame(self.root)
        content.pack(
            fill=tk.BOTH,
            expand=True
        )

        '''
        ------------------
        PROPERTY PANEL
        ------------------
        '''
        property_panel = tk.Frame(content, width = 300)
        property_panel.pack(side = tk.RIGHT, fill = tk.Y)
        property_panel.pack_propagate(False)

        tk.Label(property_panel, text="Waypoint Property", anchor="n", font=("Arial", 16)).pack(side=tk.TOP, fill=tk.X, pady=3.0)

        self.property_table = ttk.Treeview(
            property_panel,
            columns=("property", "value"),
            show = "headings",
            height=10
        )

        self.property_table.heading("property", text="Property")
        self.property_table.heading("value", text="Value")

        self.property_table.column("property", width=140)
        self.property_table.column("value", width=140)

        self.property_table.pack(
            side=tk.TOP,
            fill=tk.X,
            padx=5,
            pady=5
        )

        '''
        ------------------
        CANVAS
        ------------------
        '''
        self.canvas = FigureCanvasTkAgg(self.fig, master=content)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        toolbar = NavigationToolbar2Tk(self.canvas, self.root)
        toolbar.update()

    def _connect_events(self):
        # Connect Tk Windows event to class functions
        self.canvas.mpl_connect("scroll_event", self.on_scroll)
        self.canvas.mpl_connect("key_press_event", self.on_key_press)
        self.canvas.mpl_connect("button_press_event", self.on_press)
        self.canvas.mpl_connect("motion_notify_event", self.on_motion)
        self.canvas.mpl_connect("button_release_event", self.on_release)
    '''
    ==============
       CONTROLS
    ==============
    '''
    def on_scroll(self, event):
        if event.inaxes != self.ax or event.xdata is None or event.ydata is None:
            # Check if there is a cursor on ax
            return

        # Control zooming
        scale = 0.85 if event.button == "up" else 1.15

        # Retrieve old "zoom"
        old_xlim = self.ax.get_xlim()
        old_ylim = self.ax.get_ylim()

        # Retrieve mouse positions with respect to ax
        xmouse = event.xdata
        ymouse = event.ydata

        # Apply zoom to ax
        x1 = xmouse - (xmouse - old_xlim[0]) * scale 
        x2 = xmouse + (old_xlim[1] - xmouse) * scale 
        y1 = ymouse - (ymouse - old_ylim[0]) * scale 
        y2 = ymouse + (old_ylim[1] - ymouse) * scale 

        self.ax.set_xlim(x1,x2)
        self.ax.set_ylim(y1,y2)

        # Redraw when scrolling is complete
        self.canvas.draw_idle()

    def on_key_press(self, event):
        if event.key == "ctrl+z":
            self.racepath_undo()
        elif event.key == "ctrl+y":
            self.racepath_redo()
        elif event.key == "a":
            self.push_racepath_undo_stack()
            if event.inaxes != self.ax:
                return
            
            self.path.create_node((event.xdata, event.ydata))
            self.redraw()

    def on_press(self, event):
        '''
        Button press control for the following
        1. Space + Left Click - For Panning
        2. Left Click - Select Object
        3. CTRL + Left Click - Select More Objects
        4. Left Click on Selected Object - Drag
        '''
        # Check if the mouse is in the editing area
        if event.inaxes != self.ax:
            return

        # Panning: Hold Space + LMB
        if event.button == 1 and event.key == " ":
            # Button == 1 - LMB down
            # key = "space" - space down
            # Initiate panning - save mouse positions
            self.push_racepath_undo_stack()
            self.panning = True
            self.pan_last_x = event.x
            self.pan_last_y = event.y

        # Selecting and Dragging: Left Click on Object:
        elif event.button == 1:
            mouse_obj = self.find_object_at(event)


            if event.key == "control" and mouse_obj is not None:
                # Hold CTRL = MultiSelect
                if mouse_obj not in self.selected:
                    self.selected.append(mouse_obj)
                    self.status.set(f"Selected {len(self.selected)} object(s).")
            elif mouse_obj is None:
                self.selected = []
                self.status.set("Idle")
            else:
                self.selected = [mouse_obj]
                self.status.set(f"Selected {1} object.")

            self.update_property_panel()
            self.redraw()

            if mouse_obj in self.selected:
                # Initiate Dragging
                self.push_racepath_undo_stack()
                self.dragging = True
                self.drag_last_x = event.x
                self.drag_last_y = event.y
                
    def on_motion(self, event):
        if event.inaxes != self.ax:
            return

        # Panning control
        if self.dragging:
            DRAGGING_SPEED = 1.15
            xlim = self.ax.get_xlim()
            ylim = self.ax.get_ylim()

            dx = event.x - self.drag_last_x
            dy = event.y - self.drag_last_y

            # Rescale dx and dy based on current zoom
            # Retrieve ax dimensions (pixels) on Tk Windows
            axes_width = self.ax.bbox.width
            axes_height = self.ax.bbox.height

            meters_per_pixel_x = (xlim[1] - xlim[0]) / axes_width
            meters_per_pixel_y = (ylim[1] - ylim[0]) / axes_height

            dx *= meters_per_pixel_x * DRAGGING_SPEED
            dy *= meters_per_pixel_y * DRAGGING_SPEED

            for obj in self.selected:
                obj.x += dx
                obj.y += dy

            self.drag_last_x = event.x
            self.drag_last_y = event.y

            self.update_property_panel()
            self.redraw()
            
        if self.panning:
            PANNING_SPEED = 1.15
            # Update ax xlim and ylim
            xlim = self.ax.get_xlim()
            ylim = self.ax.get_ylim()

            dx = event.x - self.pan_last_x
            dy = event.y - self.pan_last_y

            # Rescale dx and dy based on current zoom
            # Retrieve ax dimensions (pixels) on Tk Windows
            axes_width = self.ax.bbox.width
            axes_height = self.ax.bbox.height

            meters_per_pixel_x = (xlim[1] - xlim[0]) / axes_width
            meters_per_pixel_y = (ylim[1] - ylim[0]) / axes_height

            dx *= meters_per_pixel_x * PANNING_SPEED
            dy *= meters_per_pixel_y * PANNING_SPEED
            
            self.ax.set_xlim(xlim[0] - dx, xlim[1] - dx)
            self.ax.set_ylim(ylim[0] - dy, ylim[1] - dy)

            # Redraw when the motion is over
            self.canvas.draw_idle()

            self.pan_last_x = event.x
            self.pan_last_y = event.y

    def on_release(self, event):
        '''
        # Check button release to
        1. Deactivate panning mode
        2. Deactivate dragging mode
        '''

        # Deactivate panning (LMB released)
        if event.button == 1 and self.panning:
            self.panning = False    
            self.pan_last_x = None
            self.pan_last_y = None

        elif event.button == 1 and self.dragging:
            self.dragging = False    
            self.drag_last_x = None
            self.drag_last_y = None

    '''
    ====================
       TOOL UTILITIES
    ====================
    '''
    def racepath_undo(self):
        self.selected = []
        if self.undo_stack:
            racedict = self.undo_stack.pop()
            self.redo_stack.append(self.path.to_dict())
            self.path.from_dict(racedict)
        else:
            self.status.set(f"Nothing left to undo!")

        self.redraw()
        self.update_property_panel()

    def racepath_redo(self):
        self.selected = []
        if self.redo_stack:
            racedict = self.redo_stack.pop()
            self.undo_stack.append(self.path.to_dict())
            self.path.from_dict(racedict)
            
        else:
            self.status.set(f"Nothing to redo!")

        self.redraw()
        self.update_property_panel()

    def push_racepath_undo_stack(self):
        self.redo_stack = [] # Clear redo stack
        self.undo_stack.append(self.path.to_dict())

        if len(self.undo_stack) > MAX_UNDO:
            self.undo_stack.pop(0)

    def close_loop(self):
        # TODO Make this work properly when we can add points to loop
        pass

    '''
    ======================
       CANVAS UTILITIES
    ======================
    '''
    
    def update_property_panel(self):
        for item in self.property_table.get_children():
            self.property_table.delete(item)

        # Check selected object
        if len(self.selected) == 1 and isinstance(self.selected[0], (Node, ControlPoint)):
            # One Node/Control Point slection: Show X, Y
            # TODO Implement extra node properties 
            node = self.selected[0]
            self.property_table.insert(
                "",
                "end",
                values=("X", node.x)
            )
            self.property_table.insert(
                "",
                "end",
                values=("Y", node.y)
            )
        if len(self.selected) > 1:
            self.property_table.insert(
                "",
                "end",
                values=("X", "Multiselection")
            )
            self.property_table.insert(
                "",
                "end",
                values=("Y", "Multiselection")
            )

    def redraw(self, keep_limits=True, fast=False):
        '''
        Redraw all waypoints dots/lines and update matplotlib self.ax
        Then call canvas.draw() to represent change in Tk window.

        PARAMETERS
        1. keep_limits
            Whether to keep the camera angle (The xlim and ylim usually 
            change after a point is changed), this option maintain the lims
            Otherwise, find new lim using relim()
        '''
        if self.path is None or not self.path.nodes:
            return

        # Delete old artists if they are not empty
        if self.line_artist is not None:
            self.line_artist.remove()
            self.line_artist = None
        if self.point_artist is not None:
            self.point_artist.remove()
            self.point_artist = None
        if self.control_point_artist is not None:
            self.control_point_artist.remove()
            self.control_point_artist = None
        if self.selected_artist is not None:
            self.selected_artist.remove()
            self.selected_artist = None

        # List of Nodes
        x = np.array([node.x for node in self.path.nodes])
        y = np.array([node.y for node in self.path.nodes])

        # List of Sample to draw line
        line = self.path.sample(DEFAULT_REDRAW_RES)
        x_line = np.array([pt[0] for pt in line])
        y_line = np.array([pt[1] for pt in line])

        # List of Control Points
        cpts = self.path.get_control_points()
        
        x_cpt = np.array([pt.x for pt in cpts])
        y_cpt = np.array([pt.y for pt in cpts])

        # Save camera angle (xlim and ylim)
        if keep_limits:
            xlim = self.ax.get_xlim()
            ylim = self.ax.get_ylim()

        # Plot on ax and save artists
        self.line_artist, = self.ax.plot(x_line,y_line,"-",color="blue",zorder=2) # Connecting Lines
        self.point_artist = self.ax.scatter(x,y,s=24,color="blue",zorder=3) # Waypoint Dots
        self.control_point_artist = self.ax.scatter(x_cpt,y_cpt, facecolors='none',color="blue",s=18,zorder=3) # Control Points

        # Draw larger selected dots on top
        selected = self.selected

        if len(selected) > 0:
            selected_x = [sel.x for sel in selected]
            selected_y = [sel.y for sel in selected]
                
            self.selected_artist = self.ax.scatter(
                selected_x,
                selected_y,
                s=40,
                facecolors='none',
                linewidth=2,
                color="lime",
                zorder=4
            )
        
        if keep_limits:
            self.ax.set_xlim(xlim)
            self.ax.set_ylim(ylim)
        else:
            self.ax.relim()
            self.ax.update_datalim(np.column_stack((x, y)))
            self.ax.autoscale_view()

        self.canvas.draw_idle() # Update Tk Windows

    def find_object_at(self, event):
        if self.path is None or not self.path.nodes:
            return None
        
        selectable = self.path.nodes + self.path.get_control_points()

        retObj = None
        objDist = float("inf")
        
        for obj in selectable:
            screen_x, screen_y = self.ax.transData.transform((obj.x, obj.y))
            dist_to_obj = math.dist((event.x, event.y), (screen_x, screen_y))
            if dist_to_obj <= SELECTOR_SIZE_PX and dist_to_obj < objDist:
                objDist = dist_to_obj
                retObj = obj

        return retObj

    '''
    ===================
       FILE HANDLING
    ===================
    '''
    def load_map_yaml(self):
        # File Selector
        path = filedialog.askopenfilename(
            title="Open Map YAML",
            filetypes=[("YAML files", "*.yaml *.yml"), ("All files", "*.*")]
        )
        if not path:
            return
        
        self.map_yaml_path = Path(path)
        self.display_map_from_yaml()

    def display_map_from_yaml(self):
        try:
            with open(self.map_yaml_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
    
            image_path = Path(data["image"])
            if not image_path.is_absolute():
                image_path = self.map_yaml_path.parent / image_path

            resolution = float(data["resolution"])
            origin = data.get("origin", [0.0, 0.0, 0.0])
            origin_x, origin_y = float(origin[0]), float(origin[1])

            # Load map image
            img = Image.open(image_path).convert("L")
            arr = np.array(img)
            height, width = arr.shape[:2]

            # Convert dimensions from pixels to meters
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

            # Show map underneath on ax
            self.map_artist = self.ax.imshow(
                arr,
                cmap="gray",
                origin="upper",
                extent=extent,
                alpha=0.85,
                zorder=0
            )

            self.status.set(f"Loaded map: {image_path.name}, resolution={resolution}, origin={origin[:2]}")
            self.redraw(keep_limits=False)

        except Exception as e:
            messagebox.showerror("Error Loading Map YAML", str(e))

    def detect_columns(self):
        # Check if the loaded dataframe has the X and Y columns
        assert self.df is not None
        cols_lower = {str(c).lower().strip(): c for c in self.df.columns}

        # Function to help index differently named DataFrame columns
        def pick(candidates):
            for c in candidates:
                if c in cols_lower:
                    return cols_lower[c]
            return None

        if self.df.shape[1] < 2:
            raise ValueError("Waypoint DataFrame must have at least two columns!")

        self.x_col = pick(["x", "pos_x", "world_x"]) or self.df.columns[0]
        self.y_col = pick(["y", "pos_y", "world_y"]) or self.df.columns[1]

        self.yaw_col = pick(["yaw", "heading", "theta"])
        self.curv_col = pick(["curvature", "curv", "kappa"])
        self.vel_col = pick(["velocity", "vel", "speed", "v"])

    def open_raceline_rpp(self):
        # File Selector
        path = filedialog.askopenfilename(
            title="Open Raceline Project",
            filetypes=[("Patcher Project Files", "*.rpp"), ("All files", "*.*")]
        )
        if not path:
            return

        try:
            self.rpp_path = Path(path)

            with open(self.rpp_path, 'r') as f:
                data = json.load(f)
                
                self.map_yaml_path = Path(data["map_yaml_path"])
                self.display_map_from_yaml()

                self.path = RacePath()
                self.selected = []
                self.path.from_dict(data["racepath"])
                
            self.redraw(keep_limits=False)
            self.status.set(
                f"Loaded RPP: {self.rpp_path.name}, map_yaml = {self.map_yaml_path.name}"
            )

        except Exception as e:
            messagebox.showerror("RPP load error", str(e))

    def load_raceline_csv(self):
        # File Selector
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

            self.redraw(keep_limits=False)
            self.status.set(
                f"Loaded CSV: {self.csv_path.name} | x={self.x_col}, y={self.y_col}, "
                f"yaw={self.yaw_col}, curv={self.curv_col}, vel={self.vel_col}"
            )

        except Exception as e:
            messagebox.showerror("CSV load error", str(e))

    def save_raceline_csv(self):
        if self.path is None or not self.path.nodes:
            messagebox.showwarning("No Raceline to Save", "Create some nodes/segments first.")
            return
        if self.csv_path is None:
            self.save_raceline_csv_as()
            return
        try:
            # TODO Implement CSV Saving Windows
            CSV_RESOLUTION = 0.02

            sampled = self.path.sample(CSV_RESOLUTION)

            df = pd.DataFrame(sampled, columns=["X", "Y"])
            df.to_csv(self.csv_path, index=False)
            self.status.set(f"Saved CSV: {self.csv_path}")
        except Exception as e:
            messagebox.showerror("CSV save error", str(e))

    def save_raceline_csv_as(self):
        if self.path is None or not self.path.nodes:
            messagebox.showwarning("No Raceline to Save", "Create some nodes/segments first.")
            return

        path = filedialog.asksaveasfilename(
            title="Save Raceline Project CSV As",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if not path:
            return
        self.csv_path = Path(path)
        self.save_raceline_csv()    

    def save_raceline_rpp(self):
        if self.path is None or not self.path.nodes:
            messagebox.showwarning("No RPP", "Load a raceline project RPP first.")
            return
        if self.rpp_path is None:
            self.save_raceline_rpp_as()
            return
        try:
            data = {}
            data["racepath"] = self.path.to_dict()
            data["map_yaml_path"] = str(self.map_yaml_path.absolute())

            with open(self.rpp_path, 'w') as fp:
                json.dump(data, fp, indent=4)

            self.status.set(f"Saved RPP: {self.rpp_path}")
        except Exception as e:
            messagebox.showerror("RPP save error", str(e))

    def save_raceline_rpp_as(self):
        if self.path is None or not self.path.nodes:
            messagebox.showwarning("No RPP", "Load a raceline project RPP first.")
            return

        path = filedialog.asksaveasfilename(
            title="Save Raceline Project RPP as",
            defaultextension=".rpp",
            filetypes=[("RPP files", "*.rpp"), ("All files", "*.*")]
        )
        if not path:
            return
        self.rpp_path = Path(path)
        self.save_raceline_rpp()

def main():
    root = tk.Tk()
    root.geometry("1280x720")
    RacelineEditor(root)

    def on_close():
        plt.close("all")
        root.quit()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()

if __name__ == "__main__":
    main()
