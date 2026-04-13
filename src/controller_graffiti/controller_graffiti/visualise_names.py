import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import re
import random
from skimage import transform
from matplotlib.textpath import TextPath
from matplotlib.font_manager import FontProperties
from matplotlib.colors import to_rgba

# --- CONFIGURATION (Defaults) ---
INPUT_FILENAME = "layout_data.txt"
FONT_PATH = "Inlanders Demo.otf" 
# These will be overwritten by the file if CANVAS_SIZE is found

HERO_W = 400  
HERO_H = 200 
HERO_TEXT = "UNSW"

# Color Palette
COLORS = ['black', 'red', 'blue', 'green']
# ---------------------

def rasterize_text(text, font_path, color_name='black', size=(1000, 500)):
    # Create a high-res temp canvas
    fig_temp = plt.figure(figsize=(size[0]/100, size[1]/100), dpi=100)
    try:
        fp = FontProperties(fname=font_path)
    except:
        fp = FontProperties(weight='bold')

    t = TextPath((0, 0), text, prop=fp)
    bbox = t.get_extents()
    ax = fig_temp.add_axes([0, 0, 1, 1])
    ax.axis('off')
    
    # --- MODIFICATION HERE ---
    # We set facecolor to 'none' (hollow) and edgecolor to 'black' (the stroke).
    # linewidth controls the thickness of the outline.
    patch = patches.PathPatch(t, facecolor='none', edgecolor='black', linewidth=8)
    # -------------------------
    
    ax.add_patch(patch)
    ax.set_xlim(bbox.xmin, bbox.xmax)
    ax.set_ylim(bbox.ymin, bbox.ymax)
    
    fig_temp.canvas.draw()
    data = np.frombuffer(fig_temp.canvas.tostring_argb(), dtype=np.uint8)
    data = data.reshape(fig_temp.canvas.get_width_height()[::-1] + (4,))
    plt.close(fig_temp)
    
    # Reorder channels from ARGB to RGBA for processing
    img = np.copy(data)
    img[:, :, 0] = data[:, :, 1] 
    img[:, :, 1] = data[:, :, 2] 
    img[:, :, 2] = data[:, :, 3] 
    img[:, :, 3] = data[:, :, 0] 

    # Detect the stroke pixels (black lines)
    # Since background is white (255) and lines are black (0), < 200 catches the lines.
    is_text = (img[:,:,0] < 200)
    
    rgba = to_rgba(color_name)
    r, g, b = int(rgba[0]*255), int(rgba[1]*255), int(rgba[2]*255)
    
    final_img = np.zeros_like(img)
    # Apply the user's chosen color ONLY to the outline pixels
    final_img[is_text] = [r, g, b, 255] 
    
    return final_img

def shrink_quad(points, factor=0.95):
    pts = np.array(points)
    centroid = np.mean(pts, axis=0)
    vectors = pts - centroid
    new_pts = centroid + vectors * factor
    return new_pts

def get_warp_transform(img, dst_points):
    h, w, _ = img.shape
    
    # 1. Define Source Points: Top-Left, Top-Right, Bottom-Right, Bottom-Left
    # (In image processing, (0,0) is usually Top-Left)
    src = np.array([[0, 0], [w, 0], [w, h], [0, h]])
    
    pts = np.array(dst_points)

    # 2. Sort by X (horizontal) first to separate Left side from Right side
    # This ensures "Left points are on the left and right on the right"
    sorted_x = pts[pts[:, 0].argsort()]
    lefts = sorted_x[:2]   # The two points with the smallest X
    rights = sorted_x[2:]  # The two points with the largest X

    # 3. Sort the Lefts by Y (vertical)
    # In your plotting system (Matplotlib origin='lower'), Higher Y is Top.
    tl = lefts[lefts[:, 1].argsort()[1]]  # Top-Left (Larger Y)
    bl = lefts[lefts[:, 1].argsort()[0]]  # Bottom-Left (Smaller Y)

    # 4. Sort the Rights by Y
    tr = rights[rights[:, 1].argsort()[1]] # Top-Right (Larger Y)
    br = rights[rights[:, 1].argsort()[0]] # Bottom-Right (Smaller Y)

    # 5. Order Destination to match Source: TL, TR, BR, BL
    dst = np.array([tl, tr, br, bl])

    tform = transform.ProjectiveTransform()
    tform.estimate(src, dst)
    return tform

def dist_to_hero_box(cx, cy, current_w, current_h):
    hx1 = (current_w - HERO_W) / 2
    hx2 = hx1 + HERO_W
    hy1 = (current_h - HERO_H) / 2
    hy2 = hy1 + HERO_H
    dx = max(hx1 - cx, 0, cx - hx2)
    dy = max(hy1 - cy, 0, cy - hy2)
    return np.sqrt(dx*dx + dy*dy)

def interactive_session():
    global CANVAS_W, CANVAS_H
    cells = []
    
    # 1. LOAD DATA & PARSE CANVAS SIZE
    try:
        with open(INPUT_FILENAME, "r") as f:
            lines = f.readlines()  
        
        for line in lines:
            if "CANVAS_SIZE:" in line:
                size_match = re.search(r"(\d+)x(\d+)", line)
                if size_match:
                    CANVAS_W = int(size_match.group(1))
                    CANVAS_H = int(size_match.group(2))
                    print(f"Adjusting viewport to saved size: {CANVAS_W}x{CANVAS_H}")
                continue

            if "|" in line or "---" in line or not line.strip(): 
                continue

            numbers = re.findall(r"[-+]?\d*\.\d+|\d+", line)
            if len(numbers) >= 9:
                cell_id = int(numbers[0])
                coords = [float(x) for x in numbers[1:]]
                points = [(coords[i], coords[i+1]) for i in range(0, 8, 2)]
                
                cx = sum(p[0] for p in points)/4
                cy = sum(p[1] for p in points)/4
                dist = dist_to_hero_box(cx, cy, CANVAS_W, CANVAS_H)
                
                cells.append({'id': cell_id, 'points': points, 'dist': dist})
    except FileNotFoundError:
        print("Data file not found.")
        return

    if not cells:
        print("No cell data found in file.")
        return

    hero_cell = next((c for c in cells if c['id'] == 0), None)
    available_cells = [c for c in cells if c['id'] != 0]

    # 2. SETUP PLOT
    plt.ion() 
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_facecolor('white')
    fig.patch.set_facecolor('white')
    ax.set_xlim(0, CANVAS_W)
    ax.set_ylim(0, CANVAS_H)
    ax.axis('off')

    # 3. DRAW HERO BOX
    if hero_cell:
        print("Initializing Hero Box...")
        hero_pts_shrunk = shrink_quad(hero_cell['points'], 0.95)
        # Note: Hero text will also be outlined now
        hero_img = rasterize_text(HERO_TEXT, FONT_PATH, color_name='black')
        tform = get_warp_transform(hero_img, hero_pts_shrunk)
        warped_hero = transform.warp(hero_img, tform.inverse, output_shape=(CANVAS_H, CANVAS_W))
        ax.imshow(warped_hero, origin='lower', extent=[0, CANVAS_W, 0, CANVAS_H])
        plt.draw()
        plt.pause(0.01)

    # 4. INTERACTIVE LOOP
    print("\n" + "="*40)
    print(" Interactive Mode Started")
    print(" Type a name and press ENTER.")
    print("="*40 + "\n")
    
    while True:
        if not available_cells:
            print("All boxes filled!")
            break

        user_input = input(f"Next Name ({len(available_cells)} left) > ").strip()
        if user_input.lower() == 'exit':
            break
        if not user_input:
            continue

        available_cells.sort(key=lambda x: x['dist'])
        pool_size = min(10, len(available_cells))
        candidate_pool = available_cells[:pool_size]
        target_cell = random.choice(candidate_pool)
        available_cells.remove(target_cell)

        rnd_color = random.choice(COLORS)
        txt_img = rasterize_text(user_input, FONT_PATH, color_name=rnd_color)
        target_pts_shrunk = shrink_quad(target_cell['points'], 0.9)

        tform = get_warp_transform(txt_img, target_pts_shrunk)
        warped_txt = transform.warp(txt_img, tform.inverse, output_shape=(CANVAS_H, CANVAS_W))
        ax.imshow(warped_txt, origin='lower', extent=[0, CANVAS_W, 0, CANVAS_H])
        
        plt.draw()
        plt.pause(0.01)
        print(f"Placed '{user_input}' in Cell {target_cell['id']} ({rnd_color})")

    plt.ioff()
    plt.show()

if __name__ == "__main__":
    interactive_session()