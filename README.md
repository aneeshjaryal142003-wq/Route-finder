import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import math, heapq, random

try:
    from PIL import Image, ImageTk, ImageDraw, ImageFilter, ImageOps
    HAVE_PIL = True
except Exception:
    HAVE_PIL = False

# ---------- CONFIG ----------
CANVAS_WIDTH = 1000
CANVAS_HEIGHT = 650
DEFAULT_SAMPLING = 6
DEFAULT_THRESHOLD = 120
CAR_SPEED = 120.0  # pixels per second

# Colors (soft, realistic palette)
MARKER_START = "#00b894"
MARKER_END = "#d63031"
PATH_COLOR = "#ffeaa7"
VISITED_COLOR = "#74b9ff"
ROAD_PREVIEW = "#dfe6e9"
BG_COLOR = "#2d3436"

# ---------- Utility ----------
def brightness(rgb):
    r, g, b = rgb
    return int(0.2126 * r + 0.7152 * g + 0.0722 * b)

# ---------- A* Search ----------
def astar_grid(start, goal, walkable, rows, cols, diagonal=True):
    def heuristic(a, b):
        (r1, c1), (r2, c2) = a, b
        return math.hypot(r1 - r2, c1 - c2)

    open_heap = []
    heapq.heappush(open_heap, (heuristic(start, goal), 0, start))
    came_from = {start: None}
    gscore = {start: 0}
    visited_order = []

    directions = [(-1,0),(1,0),(0,-1),(0,1)]
    if diagonal:
        directions += [(-1,-1),(-1,1),(1,-1),(1,1)]

    while open_heap:
        f, g, current = heapq.heappop(open_heap)
        visited_order.append(current)
        if current == goal:
            break
        cr, cc = current
        for dr, dc in directions:
            nr, nc = cr + dr, cc + dc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            nb = (nr, nc)
            if nb not in walkable:
                continue
            cost = math.hypot(dr, dc)
            tentative_g = g + cost
            if nb not in gscore or tentative_g < gscore[nb]:
                gscore[nb] = tentative_g
                heapq.heappush(open_heap, (tentative_g + heuristic(nb, goal), tentative_g, nb))
                came_from[nb] = current

    if goal not in came_from:
        return None, visited_order

    path = []
    cur = goal
    while cur is not None:
        path.append(cur)
        cur = came_from[cur]
    path.reverse()
    return path, visited_order

# ---------- APP ----------
class RealisticMapApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("🗺 Realistic Offline Map Route Finder")
        self.configure(bg=BG_COLOR)
        self.geometry(f"{CANVAS_WIDTH+350}x{CANVAS_HEIGHT+20}")

        # State
        self.map_image = None
        self.map_tk = None
        self.sampling = tk.IntVar(value=DEFAULT_SAMPLING)
        self.threshold = tk.IntVar(value=DEFAULT_THRESHOLD)
        self.diagonal = tk.BooleanVar(value=True)
        self.walkable = set()
        self.start = None
        self.end = None
        self.path = None
        self.visited = []
        self.mode = tk.StringVar(value="start")

        # Animation
        self._car = None
        self._anim_after = None
        self._anim_index = 0
        self._anim_pos = None

        self.create_widgets()
        self.load_default_map()

    # ---------- GUI ----------
    def create_widgets(self):
        control_frame = ttk.Frame(self)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=8)

        ttk.Label(control_frame, text="Realistic Map Controls", font=("Segoe UI", 12, "bold")).pack(pady=4)
        ttk.Button(control_frame, text="📁 Load Map Image", command=self.load_map_image).pack(fill='x', pady=3)
        ttk.Button(control_frame, text="🧭 Use Demo Map", command=self.load_default_map).pack(fill='x', pady=3)

        ttk.Separator(control_frame).pack(fill='x', pady=8)
        ttk.Label(control_frame, text="Marker Mode:").pack(anchor='w')
        ttk.Radiobutton(control_frame, text="Set Start", variable=self.mode, value='start').pack(anchor='w')
        ttk.Radiobutton(control_frame, text="Set End", variable=self.mode, value='end').pack(anchor='w')

        ttk.Label(control_frame, text="\nGrid Sampling:").pack(anchor='w')
        ttk.Spinbox(control_frame, from_=2, to=24, textvariable=self.sampling).pack(fill='x')

        ttk.Label(control_frame, text="Road Brightness Threshold:").pack(anchor='w')
        ttk.Spinbox(control_frame, from_=0, to=255, textvariable=self.threshold).pack(fill='x')

        ttk.Checkbutton(control_frame, text="Allow Diagonal Moves", variable=self.diagonal).pack(anchor='w', pady=6)

        ttk.Button(control_frame, text="🔍 Build Grid & Preview", command=self.build_grid).pack(fill='x', pady=5)
        ttk.Button(control_frame, text="🚦 Find Route", command=self.run_search).pack(fill='x', pady=5)
        ttk.Button(control_frame, text="🚗 Animate Path", command=self.animate_along_path).pack(fill='x', pady=5)
        ttk.Button(control_frame, text="⏹ Stop Animation", command=self.stop_animation).pack(fill='x', pady=5)
        ttk.Button(control_frame, text="🧹 Clear Map", command=self.clear_all).pack(fill='x', pady=5)

        ttk.Separator(control_frame).pack(fill='x', pady=8)
        self.status = tk.StringVar(value="Load a map, build grid, set Start/End, then Run.")
        ttk.Label(control_frame, textvariable=self.status, wraplength=320).pack(anchor='w', pady=4)

        # Canvas
        self.canvas = tk.Canvas(self, width=CANVAS_WIDTH, height=CANVAS_HEIGHT, bg='black', highlightthickness=0)
        self.canvas.pack(side=tk.RIGHT, padx=10, pady=8)
        self.canvas.bind("<Button-1>", self.on_click)

    # ---------- Map ----------
    def load_default_map(self):
        if HAVE_PIL:
            img = Image.new("RGB", (CANVAS_WIDTH, CANVAS_HEIGHT), (65, 75, 80))
            draw = ImageDraw.Draw(img)
            for i in range(100, CANVAS_WIDTH-100, 180):
                draw.line([(i, 0), (i, CANVAS_HEIGHT)], fill=(230, 230, 220), width=random.randint(12, 20))
            for j in range(80, CANVAS_HEIGHT-80, 160):
                draw.line([(0, j), (CANVAS_WIDTH, j)], fill=(230, 230, 220), width=random.randint(10, 18))
            img = img.filter(ImageFilter.GaussianBlur(1))
            self.set_map_image(img)
            self.status.set("Demo map loaded successfully.")
        else:
            self.status.set("Install Pillow for demo map.")

    def set_map_image(self, pil_img):
        iw, ih = pil_img.size
        ratio = min(CANVAS_WIDTH/iw, CANVAS_HEIGHT/ih)
        new_size = (int(iw*ratio), int(ih*ratio))
        resized = pil_img.resize(new_size, Image.LANCZOS)
        self.map_image = resized
        self.map_tk = ImageTk.PhotoImage(resized)
        self.canvas.delete("all")
        self.canvas.create_image(CANVAS_WIDTH//2, CANVAS_HEIGHT//2, image=self.map_tk)

    def load_map_image(self):
        path = filedialog.askopenfilename(filetypes=[("Images", ".png;.jpg;.jpeg;.bmp")])
        if not path:
            return
        img = Image.open(path).convert("RGB")
        self.set_map_image(img)
        self.status.set(f"Loaded: {path.split('/')[-1]}")

    # ---------- Logic ----------
    def canvas_to_image_coords(self, x, y):
        img_x = (CANVAS_WIDTH - self.map_image.width) // 2
        img_y = (CANVAS_HEIGHT - self.map_image.height) // 2
        return x - img_x, y - img_y

    def image_to_grid(self, ix, iy):
        s = self.sampling.get()
        return int(iy // s), int(ix // s)

    def grid_to_image_center(self, cell):
        r, c = cell
        s = self.sampling.get()
        img_x = (CANVAS_WIDTH - self.map_image.width)//2
        img_y = (CANVAS_HEIGHT - self.map_image.height)//2
        return img_x + c*s + s/2, img_y + r*s + s/2

    def on_click(self, event):
        ix, iy = self.canvas_to_image_coords(event.x, event.y)
        if ix < 0 or iy < 0 or ix >= self.map_image.width or iy >= self.map_image.height:
            return
        cell = self.image_to_grid(ix, iy)
        if self.mode.get() == "start":
            self.start = cell
            self.path = None
            self.stop_animation()
            self.draw_overlay()
            self.status.set(f"Start set at {cell}")
        elif self.mode.get() == "end":
            self.end = cell
            self.path = None
            self.stop_animation()
            self.draw_overlay()
            self.status.set(f"End set at {cell}")

    def detect_roads(self):
        img = self.map_image
        s = self.sampling.get()
        iw, ih = img.size
        cols, rows = iw//s, ih//s
        self.rows, self.cols = rows, cols
        px = img.load()
        edged = img.convert("L").filter(ImageFilter.FIND_EDGES)
        ed_px = edged.load()
        thresh = self.threshold.get()
        walk = set()
        for r in range(rows):
            for c in range(cols):
                sx, sy = int((c + 0.5)*s), int((r + 0.5)*s)
                if sx >= iw or sy >= ih:
                    continue
                if brightness(px[sx, sy]) >= thresh or ed_px[sx, sy] > 40:
                    walk.add((r, c))
        return walk

    def build_grid(self):
        self.walkable = self.detect_roads()
        self.status.set(f"Grid built — cells: {len(self.walkable)} walkable")
        self.draw_overlay(preview=True)

    def run_search(self):
        if not self.walkable or not self.start or not self.end:
            messagebox.showinfo("Missing Info", "Build grid and set start/end first.")
            return
        path, visited = astar_grid(self.start, self.end, self.walkable, self.rows, self.cols, self.diagonal.get())
        if not path:
            self.status.set("No path found.")
            return
        self.path, self.visited = path, visited
        self.status.set(f"Path found — length {len(path)}")
        self.draw_overlay()

    def draw_overlay(self, preview=False):
        self.canvas.delete("all")
        self.canvas.create_image(CANVAS_WIDTH//2, CANVAS_HEIGHT//2, image=self.map_tk)
        if preview:
            s = self.sampling.get()
            img_x = (CANVAS_WIDTH - self.map_image.width)//2
            img_y = (CANVAS_HEIGHT - self.map_image.height)//2
            for (r, c) in self.walkable:
                x1, y1 = img_x + c*s, img_y + r*s
                x2, y2 = x1+s, y1+s
                self.canvas.create_rectangle(x1, y1, x2, y2, fill=ROAD_PREVIEW, outline="")
        if self.path:
            pts = [self.grid_to_image_center(p) for p in self.path]
            for i in range(len(pts)-1):
                self.canvas.create_line(*pts[i], *pts[i+1], fill=PATH_COLOR, width=4, capstyle=tk.ROUND)
        if self.start:
            x, y = self.grid_to_image_center(self.start)
            self.canvas.create_oval(x-8, y-8, x+8, y+8, fill=MARKER_START, outline='black')
        if self.end:
            x, y = self.grid_to_image_center(self.end)
            self.canvas.create_oval(x-8, y-8, x+8, y+8, fill=MARKER_END, outline='black')
        if self._anim_pos:
            x, y = self._anim_pos
            self.canvas.create_oval(x-6, y-6, x+6, y+6, fill="#2d3436", outline="white")

    # ---------- Animation ----------
    def animate_along_path(self):
        if not self.path:
            messagebox.showwarning("No Path", "Run pathfinding first.")
            return
        pts = [self.grid_to_image_center(p) for p in self.path]
        self._anim_pts = pts
        self._anim_index = 0
        self._anim_pos = pts[0]
        self._animate_step()

    def _animate_step(self):
        pts = self._anim_pts
        if self._anim_index >= len(pts)-1:
            return
        x1, y1 = pts[self._anim_index]
        x2, y2 = pts[self._anim_index+1]
        dist = math.hypot(x2-x1, y2-y1)
        steps = int(max(1, dist / 4))
        def step(i):
            t = i / steps
            self._anim_pos = (x1 + (x2-x1)*t, y1 + (y2-y1)*t)
            self.draw_overlay()
            if i < steps:
                self._anim_after = self.after(20, step, i+1)
            else:
                self._anim_index += 1
                self._animate_step()
        step(0)

    def stop_animation(self):
        if self._anim_after:
            self.after_cancel(self._anim_after)
            self._anim_after = None
        self._anim_pos = None
        self.draw_overlay()

    def clear_all(self):
        self.start = None
        self.end = None
        self.path = None
        self.visited.clear()
        self.stop_animation()
        self.walkable.clear()
        self.draw_overlay()
        self.status.set("Cleared all paths and markers.")

# ---------- MAIN ----------
if __name__ == "__main__":
    app = RealisticMapApp()
    app.mainloop()
