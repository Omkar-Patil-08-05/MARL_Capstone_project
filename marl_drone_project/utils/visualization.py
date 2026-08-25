import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

class SARVisualizer:
    def __init__(self, env):
        self.env = env
        self.fig = None
        self.ax = None
        self.drone_trajectories = {i: [] for i in range(env.num_drones)}
        
        # Color mapping for grid cells
        self.color_map = {
            0: [1.0, 1.0, 1.0],      # white (unexplored)
            1: [0.8, 0.9, 1.0],      # light blue (explored)
            -1: [0.3, 0.3, 0.3]      # dark gray (building)
        }
        
    def init_render(self):
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_xlim(-0.5, self.env.x_size - 0.5)
        self.ax.set_ylim(-0.5, self.env.y_size - 0.5)
        self.ax.set_aspect('equal')
        self.ax.invert_yaxis()  # Match array coordinates
        
        # Grid image
        self.grid_img = self.ax.imshow(self._get_grid_colors(), origin='upper')
        
        self.drone_markers = []
        self.drone_texts = []
        self.drone_lines = []
        self.fov_patches = []
        
        colors = ['red', 'blue', 'green', 'orange', 'purple', 'cyan']
        
        for i in range(self.env.num_drones):
            color = colors[i % len(colors)]
            
            # Trajectory Line
            line, = self.ax.plot([], [], color=color, alpha=0.4, linewidth=2)
            self.drone_lines.append(line)
            
            # FOV Patch
            fov = patches.Rectangle((0, 0), 1, 1, fill=True, color=color, alpha=0.15)
            self.ax.add_patch(fov)
            self.fov_patches.append(fov)
            
            # Drone Marker
            marker, = self.ax.plot([], [], marker='^', markersize=10, color=color)
            self.drone_markers.append(marker)
            
            # Drone ID Label
            txt = self.ax.text(0, 0, f"D{i}", color='black', fontsize=9, fontweight='bold')
            self.drone_texts.append(txt)
            
        # Victims
        self.victim_markers = []
        for _ in range(self.env.num_victims):
            vm, = self.ax.plot([], [], marker='*', markersize=14, color='red', linestyle='None')
            self.victim_markers.append(vm)
            
        # Text Overlay
        self.title_text = self.ax.text(0.5, 1.02, "", transform=self.ax.transAxes, ha='center', fontsize=12, fontweight='bold')
        self.metrics_text = self.ax.text(1.02, 0.5, "", transform=self.ax.transAxes, ha='left', va='center', fontsize=10, bbox=dict(facecolor='white', alpha=0.8, edgecolor='none'))
        self.fig.tight_layout(rect=(0, 0, 0.85, 1))  # Leave room for metrics on the right
        
    def _get_grid_colors(self):
        c_grid = np.zeros((self.env.y_size, self.env.x_size, 3))
        for x in range(self.env.x_size):
            for y in range(self.env.y_size):
                val = self.env.grid[x, y]
                c_grid[y, x] = self.color_map[val]
        return c_grid
        
    def reset(self):
        for i in range(self.env.num_drones):
            self.drone_trajectories[i] = []
        if self.fig is None:
            self.init_render()
            
    def render(self, show_hidden_victims=True):
        if self.fig is None:
            self.init_render()
            
        if self.fig is None:
            return  # Fail gracefully if rendering cannot be initialized
            
        # Update grid background (Explored vs Unexplored vs Buildings)
        self.grid_img.set_data(self._get_grid_colors())
        
        for i in range(self.env.num_drones):
            x, y = self.env.drone_positions[i]
            
            # Only append if position changed (or it's the first step)
            if not self.drone_trajectories[i] or self.drone_trajectories[i][-1] != (x, y):
                self.drone_trajectories[i].append((x, y))
            
            # Trajectory
            tx, ty = zip(*self.drone_trajectories[i])
            self.drone_lines[i].set_data(tx, ty)
            
            # FOV Box
            radius = self.env.fov_radius
            self.fov_patches[i].set_bounds((x - radius - 0.5, y - radius - 0.5, radius * 2 + 1, radius * 2 + 1))
            
            # Drone Position
            self.drone_markers[i].set_data([x], [y])
            self.drone_texts[i].set_position((x + 0.3, y - 0.3))
            
        # Update Victims
        idx = 0
        for (vx, vy), status in self.env.victims.items():
            if status == 1:
                # Detected
                self.victim_markers[idx].set_data([vx], [vy])
                self.victim_markers[idx].set_color('lime')
                self.victim_markers[idx].set_markeredgecolor('black')
            elif show_hidden_victims:
                # Hidden but visible in debug
                self.victim_markers[idx].set_data([vx], [vy])
                self.victim_markers[idx].set_color('darkred')
                self.victim_markers[idx].set_markeredgecolor('none')
            else:
                self.victim_markers[idx].set_data([], [])
            idx += 1
            
        # Update Metrics
        valid_cells = np.sum(self.env.grid != -1)
        explored = np.sum(self.env.grid == 1)
        cov = explored / valid_cells if valid_cells > 0 else 0
        
        self.title_text.set_text(f"SAR Grid | Step: {self.env.current_step}")
        
        m = self.env.metrics
        info = f"Coverage: {cov*100:.1f}%\n\n"
        info += f"Victims: {m['victims_detected']} / {self.env.num_victims}\n\n"
        info += f"Steps: {self.env.current_step} / {self.env.max_steps}\n\n"
        info += f"Collisions: {m['collisions']}\n"
        info += f"Near-miss: {m['near_collisions']}\n"
        
        self.metrics_text.set_text(info)
        
        # Force GUI refresh
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.01)
        
    def close(self):
        if self.fig is not None:
            plt.ioff()
            plt.close(self.fig)
            self.fig = None
