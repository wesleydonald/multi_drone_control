import matplotlib.pyplot as plt
import matplotlib.patches as patches
import random
import math

# --- CONFIGURATION ---
CANVAS_W = 1600
CANVAS_H = 1600
HERO_W = 600  
HERO_H = 300 
ITERATIONS = 10
MIN_AREA = 20000 
JITTER_AMOUNT = 3
EXPORT_FILENAME = "layout_data.txt"
# ---------------------

def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def get_point_on_segment(p1, p2, t):
    x = p1[0] + (p2[0] - p1[0]) * t
    y = p1[1] + (p2[1] - p1[1]) * t
    return (x, y)

class Quad:
    def __init__(self, p1, p2, p3, p4):
        self.points = [p1, p2, p3, p4]
    
    @property
    def width(self):
        top = distance(self.points[0], self.points[1])
        bottom = distance(self.points[3], self.points[2])
        return (top + bottom) / 2

    @property
    def height(self):
        left = distance(self.points[0], self.points[3])
        right = distance(self.points[1], self.points[2])
        return (left + right) / 2
    
    @property
    def area(self):
        x = [p[0] for p in self.points]
        y = [p[1] for p in self.points]
        return 0.5 * abs(sum(x[i]*y[i-1] - x[i-1]*y[i] for i in range(4)))

    @property
    def centroid(self):
        cx = sum(p[0] for p in self.points) / 4
        cy = sum(p[1] for p in self.points) / 4
        return (cx, cy)

def split_quad(quad):
    w = quad.width
    h = quad.height
    if quad.area < MIN_AREA:
        return [quad]

    split_vertical = False
    if h > w * 0.6: 
        split_vertical = False 
    elif w > h * 3.5:
        split_vertical = True 
    else:
        split_vertical = random.random() > 0.75

    p = quad.points
    t1 = random.uniform(0.3, 0.7)
    skew = random.uniform(-0.25, 0.25) 
    t2 = max(0.2, min(0.8, t1 + skew))

    if split_vertical:
        top_pt = get_point_on_segment(p[0], p[1], t1)
        btm_pt = get_point_on_segment(p[3], p[2], t2) 
        q1 = Quad(p[0], top_pt, btm_pt, p[3])
        q2 = Quad(top_pt, p[1], p[2], btm_pt)
    else: 
        left_pt = get_point_on_segment(p[0], p[3], t1)
        rght_pt = get_point_on_segment(p[1], p[2], t2)
        q1 = Quad(p[0], p[1], rght_pt, left_pt)
        q2 = Quad(left_pt, rght_pt, p[2], p[3])

    return [q1, q2]

def apply_jitter(quads, jitter_amount):
    vertex_map = {}
    
    # Simple clamp helper to keep values within [min_val, max_val]
    def clamp(val, min_val, max_val):
        return max(min_val, min(val, max_val))

    def to_key(pt):
        return (round(pt[0], 2), round(pt[1], 2))

    hx1 = (CANVAS_W - HERO_W) / 2
    hx2 = hx1 + HERO_W
    hy1 = (CANVAS_H - HERO_H) / 2
    hy2 = hy1 + HERO_H
    
    def is_boundary(pt):
        x, y = pt
        # Checking if point is on the canvas edge or hero box edge
        if x < 5 or x > CANVAS_W - 5 or y < 5 or y > CANVAS_H - 5: return True
        if (abs(x - hx1) < 5 or abs(x - hx2) < 5) and (hy1 - 5 < y < hy2 + 5): return True
        if (abs(y - hy1) < 5 or abs(y - hy2) < 5) and (hx1 - 5 < x < hx2 + 5): return True
        return False

    for q in quads:
        for pt in q.points:
            key = to_key(pt)
            if key not in vertex_map:
                if is_boundary(pt):
                    vertex_map[key] = pt 
                else:
                    dx = random.uniform(-jitter_amount, jitter_amount)
                    dy = random.uniform(-jitter_amount, jitter_amount)
                    
                    # Apply jitter and clamp to canvas boundaries
                    new_x = clamp(pt[0] + dx, 0, CANVAS_W)
                    new_y = clamp(pt[1] + dy, 0, CANVAS_H)
                    
                    vertex_map[key] = (new_x, new_y)

    new_quads = []
    for q in quads:
        new_points = [vertex_map[to_key(pt)] for pt in q.points]
        new_quads.append(Quad(*new_points))
    return new_quads

def generate_layout():
    fig, ax = plt.subplots(figsize=(12, 10))
    ax.set_facecolor('#050505')
    fig.patch.set_facecolor('#050505')

    hx1 = (CANVAS_W - HERO_W) / 2
    hx2 = hx1 + HERO_W
    hy1 = (CANVAS_H - HERO_H) / 2
    hy2 = hy1 + HERO_H

    # CREATE HERO BOX QUAD
    hero_quad = Quad((hx1, hy2), (hx2, hy2), (hx2, hy1), (hx1, hy1))

    # Define Surroundings
    r_top = Quad((0, CANVAS_H), (CANVAS_W, CANVAS_H), (CANVAS_W, hy2), (0, hy2))
    r_btm = Quad((0, hy1), (CANVAS_W, hy1), (CANVAS_W, 0), (0, 0))
    r_lft = Quad((0, hy2), (hx1, hy2), (hx1, hy1), (0, hy1))
    r_rgt = Quad((hx2, hy2), (CANVAS_W, hy2), (CANVAS_W, hy1), (hx2, hy1))

    quads = [r_top, r_btm, r_lft, r_rgt]

    for i in range(ITERATIONS):
        next_gen = []
        for q in quads:
            if i > 4 and random.random() > 0.85:
                next_gen.append(q)
            else:
                next_gen.extend(split_quad(q))
        quads = next_gen

    all_quads = [hero_quad] + quads
    all_quads = apply_jitter(all_quads, JITTER_AMOUNT)

    # --- UPDATED EXPORT ---
    with open(EXPORT_FILENAME, "w") as f:
        # Save canvas dimensions at the top for easy parsing
        f.write(f"CANVAS_SIZE: {CANVAS_W}x{CANVAS_H}\n")
        f.write(f"ID | Points (P1, P2, P3, P4)\n")
        f.write("-" * 50 + "\n")
        for i, q in enumerate(all_quads):
            pts = [f"({p[0]:.2f}, {p[1]:.2f})" for p in q.points]
            f.write(f"Cell {i}: {', '.join(pts)}\n")
    
    print(f"Success: Exported {len(all_quads)} cells for a {CANVAS_W}x{CANVAS_H} canvas.")

    # --- DRAWING ---
    for i, q in enumerate(all_quads):
        xs = [p[0] for p in q.points] + [q.points[0][0]]
        ys = [p[1] for p in q.points] + [q.points[0][1]]
        
        clr = '#ff3333' if i == 0 else '#ffee00'
        lw = 3 if i == 0 else 1.2
        
        ax.plot(xs, ys, color=clr, linewidth=lw, alpha=0.9)

    plt.xlim(0, CANVAS_W)
    plt.ylim(0, CANVAS_H)
    plt.axis('off')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    generate_layout()