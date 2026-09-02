import argparse
import os
import sys
import pickle
import imageio
import numpy as np
import matplotlib
matplotlib.use('Agg')

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils.visualization import SARVisualizer

class MockEnv:
    def __init__(self, traj_data):
        self.num_drones = len(traj_data['initial_drone_positions'])
        self.num_victims = len(traj_data['victim_positions'])
        self.x_size, self.y_size = traj_data['grid_shape']
        self.fov_radius = 2
        self.current_step = 0
        self.max_steps = 300

        # Grid: 0 = unexp, 1 = exp, -1 = obstacle (we don't have obstacles saved, so all 0)
        self.grid = np.zeros((self.x_size, self.y_size))

        self.drone_positions = [tuple(p) for p in traj_data['initial_drone_positions']]

        # Victims: string key '(x, y)' to status int
        self.victims = {}
        for k, v in traj_data['victim_positions'].items():
            # Parse '(x, y)' string back to tuple
            clean_k = k.strip('()')
            parts = clean_k.split(',')
            x, y = int(parts[0]), int(parts[1])
            self.victims[(x, y)] = v

        self.metrics = {
            'collisions': 0,
            'near_collisions': 0,
            'hover_count': 0,
            'victims_detected': 0
        }

    def update_from_step(self, step_data):
        self.current_step = step_data['step']
        self.drone_positions = [tuple(p) for p in step_data['drone_positions']]

        # Update exploration
        for px, py in self.drone_positions:
            for dx in range(-self.fov_radius, self.fov_radius + 1):
                for dy in range(-self.fov_radius, self.fov_radius + 1):
                    nx, ny = px + dx, py + dy
                    if 0 <= nx < self.x_size and 0 <= ny < self.y_size:
                        self.grid[nx, ny] = 1

        # Update metrics
        self.metrics['collisions'] = step_data['metrics'].get('collisions', 0)
        self.metrics['near_collisions'] = step_data['metrics'].get('near_collisions', 0)
        self.metrics['hover_count'] = step_data['metrics'].get('hover_count', 0)
        self.metrics['victims_detected'] = step_data['metrics'].get('victims_detected', 0)

        # Determine which victims were found based on drone positions
        for i, ((vx, vy), status) in enumerate(self.victims.items()):
            if status == 0:
                for px, py in self.drone_positions:
                    if abs(px - vx) <= self.fov_radius and abs(py - vy) <= self.fov_radius:
                        self.victims[(vx, vy)] = 1

def render_trajectory(trajectory_file, output_file, fps=10):
    print(f"Loading trajectory from {trajectory_file}...")
    with open(trajectory_file, 'rb') as f:
        traj_data = pickle.load(f)

    mock_env = MockEnv(traj_data)
    visualizer = SARVisualizer(mock_env)
    visualizer.reset()
    assert visualizer.fig is not None
    visualizer.fig.suptitle(f"Evaluation Replay (Ep {traj_data['episode']})", color='purple', weight='bold')

    frames = []
    steps = traj_data['steps']

    print(f"Rendering {len(steps)} frames...")
    for i, step_data in enumerate(steps):
        mock_env.update_from_step(step_data)
        visualizer.render(show_hidden_victims=True)

        frame_path = f"/tmp/eval_frame_{i:03d}.png"
        visualizer.fig.savefig(frame_path, dpi=100)
        frames.append(imageio.v2.imread(frame_path))
        os.remove(frame_path)

    duration = 1.0 / fps
    imageio.mimsave(output_file, frames, duration=duration)
    print(f"Saved GIF to {output_file}")
    visualizer.close()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--trajectory', type=str, required=True, help="Path to .pkl trajectory")
    parser.add_argument('--output', type=str, required=True, help="Path to output .gif")
    parser.add_argument('--fps', type=int, default=10, help="Frames per second")
    args = parser.parse_args()

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    render_trajectory(args.trajectory, args.output, args.fps)

if __name__ == '__main__':
    main()
