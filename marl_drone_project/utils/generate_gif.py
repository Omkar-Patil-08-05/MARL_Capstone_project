import os
import sys
import torch
import numpy as np
import imageio
import matplotlib.pyplot as plt

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from env.sar_env import SARGridEnv
from utils.visualization import SARVisualizer
from train.networks.qmix import RNNAgent

def create_gif(model_path, output_path, max_steps=150, seed=42):
    env = SARGridEnv(num_drones=2, seed=seed)
    env.reset()
    
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    obs_dim = len(env.get_agent_state(0))
    agent_net = RNNAgent(input_shape=obs_dim, rnn_hidden_dim=64, n_actions=env.action_space).to(device)
    
    ckpt = torch.load(model_path, map_location=device)
    agent_net.load_state_dict(ckpt['agent_net'])
    agent_net.eval()
    
    # Use Agg backend for headless plotting
    import matplotlib
    matplotlib.use('Agg')
    
    visualizer = SARVisualizer(env)
    visualizer.reset()
    visualizer.fig.suptitle("QMIX SAR Best Policy Replay", color='blue', weight='bold')
    
    hidden_state = agent_net.init_hidden().expand(env.num_drones, -1).to(device)
    
    frames = []
    
    for step in range(max_steps):
        local_obs = [env.get_agent_state(i) for i in range(env.num_drones)]
        obs_tensor = torch.FloatTensor(np.array(local_obs)).to(device)
        
        with torch.no_grad():
            q_vals, hidden_state = agent_net(obs_tensor, hidden_state)
            
        actions = [q_vals[i].argmax().item() for i in range(env.num_drones)]
        _, _, _, done, _ = env.step(actions)
        
        visualizer.render(show_hidden_victims=True)
        
        # Save frame
        frame_path = f"/tmp/frame_{step:03d}.png"
        visualizer.fig.savefig(frame_path, dpi=100)
        frames.append(imageio.imread(frame_path))
        os.remove(frame_path)
        
        if done:
            break
            
    # Save GIF
    imageio.mimsave(output_path, frames, duration=0.1)
    print(f"Saved GIF to {output_path}")
    visualizer.close()

if __name__ == '__main__':
    create_gif(
        model_path='models/qmix_sar_v3_best_best.pth',
        output_path='/home/capstone/.gemini/antigravity-ide/brain/7c816e4d-3701-40b7-b001-78f9ab4632d1/qmix_sar_final_replay.gif'
    )
