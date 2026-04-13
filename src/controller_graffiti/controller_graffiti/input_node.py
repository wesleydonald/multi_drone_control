import rclpy
from rclpy.node import Node
import os, re, random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import TextBox
from matplotlib.textpath import TextPath
from matplotlib.font_manager import FontProperties
from matplotlib.colors import to_rgba
from matplotlib.backends.backend_agg import FigureCanvasAgg
from skimage import transform
from ament_index_python.packages import get_package_share_directory

# ROS2 Imports
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header, Float32MultiArray, MultiArrayDimension

class GraffitiNode(Node):
    def __init__(self):
        super().__init__('graffiti_node')
        
        # Publisher using Float32MultiArray to allow "Nested" data structures
        # Structure: [num_strokes, pts_in_s1, x1, y1, x2, y2..., pts_in_s2, ...]
        self.path_pub = self.create_publisher(Float32MultiArray, 'drone_graffiti_strokes', 10)
        
        try:
            self.package_share = get_package_share_directory('controller_graffiti')
        except Exception:
            self.get_logger().error("Package share not found!")
            return

        self.input_filename = os.path.join(self.package_share, 'layout_data.txt')
        self.font_path = os.path.join(self.package_share, 'Inlanders Demo.otf')
        
        self.canvas_w, self.canvas_h = 400, 200 
        self.hero_w, self.hero_h = 400, 200
        self.hero_text = "UNSW"
        self.colors = ['black', 'red', 'blue', 'green']
        self.cells, self.available_cells = [], []
        self.hero_drawn = False

        if self.load_data():
            self.setup_mural_gui()
            self.get_logger().info("Vector Publisher Ready. Differentiating strokes for MPC.")
        
    def load_data(self):
        if not os.path.exists(self.input_filename): return False
        with open(self.input_filename, "r") as f:
            lines = f.readlines()
        for line in lines:
            if "CANVAS_SIZE:" in line:
                m = re.search(r"(\d+)x(\d+)", line)
                if m: self.canvas_w, self.canvas_h = int(m.group(1)), int(m.group(2))
                continue
            num = re.findall(r"[-+]?\d*\.\d+|\d+", line)
            if len(num) >= 9:
                pts = [(float(num[i+1]), float(num[i+2])) for i in range(0, 8, 2)]
                cx, cy = sum(p[0] for p in pts)/4, sum(p[1] for p in pts)/4
                hx1, hy1 = (self.canvas_w-self.hero_w)/2, (self.canvas_h-self.hero_h)/2
                d = np.sqrt(max(hx1-cx, 0, cx-(hx1+self.hero_w))**2 + max(hy1-cy, 0, cy-(hy1+self.hero_h))**2)
                self.cells.append({'id': int(num[0]), 'points': pts, 'dist': d})
        self.available_cells = [c for c in self.cells if c['id'] != 0]
        return True

    def setup_mural_gui(self):
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        plt.subplots_adjust(bottom=0.2) 
        self.ax.set_facecolor('white')
        self.ax.set_xlim(0, self.canvas_w)
        self.ax.set_ylim(0, self.canvas_h)
        self.ax.axis('off')
        self.canvas_data = np.ones((self.canvas_h, self.canvas_w, 4))
        self.mural_artist = self.ax.imshow(self.canvas_data, origin='lower', extent=[0, self.canvas_w, 0, self.canvas_h])
        axbox = plt.axes([0.2, 0.05, 0.6, 0.075])
        self.text_box = TextBox(axbox, 'Input: ', initial="")
        self.text_box.on_submit(self.on_submit)

    def get_vector_paths(self, text):
        """Extracts strokes and differentiates them using 'MoveTo' commands."""
        try: fp = FontProperties(fname=self.font_path)
        except: fp = FontProperties(weight='bold')
        t = TextPath((0, 0), text, prop=fp)
        vertices, codes, bbox = t.vertices, t.codes, t.get_extents()
        paths, current_path = [], []
        
        for i in range(len(vertices)):
            if codes[i] == 1 and current_path: # New stroke detected
                paths.append(np.array(current_path))
                current_path = []
            
            vn = [(vertices[i][0] - bbox.xmin) / (bbox.xmax - bbox.xmin),
                  (vertices[i][1] - bbox.ymin) / (bbox.ymax - bbox.ymin)]
            current_path.append(vn)
            
            if codes[i] == 79: # End of a closed loop
                paths.append(np.array(current_path))
                current_path = []

        if current_path: paths.append(np.array(current_path))
        return paths

    def rdp_simplify(self, points, epsilon=0.001):
        if len(points) < 3: return points
        dmax, idx = 0, 0
        for i in range(1, len(points)-1):
            d = np.abs(np.cross(points[-1]-points[0], points[0]-points[i])) / np.linalg.norm(points[-1]-points[0])
            if d > dmax: idx, dmax = i, d
        if dmax > epsilon:
            res1 = self.rdp_simplify(points[:idx+1], epsilon)
            res2 = self.rdp_simplify(points[idx:], epsilon)
            return np.vstack((res1[:-1], res2))
        return np.vstack((points[0], points[-1]))

    def publish_strokes(self, vector_paths, tform):
        """Serializes grouped strokes for MPC."""
        flat_data = [float(len(vector_paths))] # First element is total strokes
        
        for path in vector_paths:
            warped_path = tform(path)
            final_path = self.rdp_simplify(warped_path, epsilon=0.0005)
            
            # Metadata for this stroke: number of points
            flat_data.append(float(len(final_path)))
            for p in final_path:
                flat_data.append(p[0] * 0.001) # X in meters
                flat_data.append(p[1] * 0.001) # Y in meters
        
        msg = Float32MultiArray()
        msg.data = flat_data
        self.path_pub.publish(msg)
        self.get_logger().info(f"Published {int(flat_data[0])} differentiated strokes.")

    def on_submit(self, text):
        text = text.strip()
        if not text: return
        
        # Target Selection Logic
        if text.lower() == 'hero':
            target_cell = next((c for c in self.cells if c['id'] == 0), None)
            draw_text, color, shrink = self.hero_text, 'black', 0.95
        else:
            self.available_cells.sort(key=lambda x: x['dist'])
            target_cell = self.available_cells.pop(random.randint(0, min(9, len(self.available_cells)-1)))
            draw_text, color, shrink = text, random.choice(self.colors), 0.9

        # 1. Generate Vectors and Transform
        vector_paths = self.get_vector_paths(draw_text)
        pts = np.array(target_cell['points'])
        centroid = np.mean(pts, axis=0)
        shrunk_pts = centroid + (pts - centroid) * shrink
        sorted_x = shrunk_pts[shrunk_pts[:, 0].argsort()]
        l, r = sorted_x[:2], sorted_x[2:]
        dst = np.array([l[l[:, 1].argsort()[1]], r[r[:, 1].argsort()[1]], r[r[:, 1].argsort()[0]], l[l[:, 1].argsort()[0]]])
        src = np.array([[0, 1], [1, 1], [1, 0], [0, 0]]) 
        tform = transform.ProjectiveTransform()
        tform.estimate(src, dst)

        # 2. Publish Serialized Stroke Data
        self.publish_strokes(vector_paths, tform)

        # 3. Visual Update
        self.render_mural(draw_text, color, dst)
        self.text_box.set_val("")

    def render_mural(self, text, color, dst):
        fig_t = FigureCanvasAgg(plt.figure(figsize=(10, 5), dpi=100))
        try: fp = FontProperties(fname=self.font_path)
        except: fp = FontProperties(weight='bold')
        t = TextPath((0, 0), text, prop=fp)
        ax = fig_t.figure.add_axes([0,0,1,1]); ax.axis('off')
        ax.add_patch(patches.PathPatch(t, facecolor='none', edgecolor='black', linewidth=8))
        bbox = t.get_extents()
        ax.set_xlim(bbox.xmin, bbox.xmax); ax.set_ylim(bbox.ymin, bbox.ymax)
        fig_t.draw()
        rgba = np.asarray(fig_t.buffer_rgba())
        h_img = np.zeros((rgba.shape[0], rgba.shape[1], 4))
        h_img[rgba[:,:,0] < 200] = list(to_rgba(color)[:3]) + [1.0]
        
        src_img = np.array([[0, 0], [rgba.shape[1], 0], [rgba.shape[1], rgba.shape[0]], [0, rgba.shape[0]]])
        tform_img = transform.ProjectiveTransform()
        tform_img.estimate(src_img, dst)
        
        warped = transform.warp(h_img, tform_img.inverse, output_shape=(self.canvas_h, self.canvas_w))
        mask = warped[:, :, 3] > 0
        self.canvas_data[mask] = warped[mask]
        self.mural_artist.set_data(self.canvas_data)
        self.fig.canvas.draw_idle()
        plt.close(fig_t.figure)

def main(args=None):
    rclpy.init(args=args)
    node = GraffitiNode()
    timer = node.create_timer(0.02, lambda: node.fig.canvas.flush_events())
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: plt.close('all'); node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()