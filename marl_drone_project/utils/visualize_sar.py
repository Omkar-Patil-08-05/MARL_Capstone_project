import argparse
import time
import sys
import os
import torch
import numpy as np
import random
import matplotlib.pyplot as plt

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from env.sar_env import SARGridEnv
from utils.visualization import SARVisualizer
from train.networks.qmix import RNNAgent

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, default='../models/qmix_sar_v1.pth')
    parser.add_argument('--random', action='store_true', help="Use random policy instead of trained model")
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--steps', type=int, default=150)
    return parser.parse_args()

def main():
    args = get_args()
    
    np.random.seed(args.seed)
    random.seed(args.seed)
    torch.manual_seed(args.seed)
    
    # Adjust directory resolution if executed from root
    if not os.path.exists(args.model):
        alt_path = os.path.join('models', 'qmix_sar_v1.pth')
        if os.path.exists(alt_path):
            args.model = alt_path
            
    env = SARGridEnv(num_drones=2, seed=args.seed)
    env.reset()
    
    visualizer = SARVisualizer(env)
    
    agent_net = None
    if not args.random and os.path.exists(args.model):
        print(f"Loading smoke-policy checkpoint from {args.model}...")
        obs_dim = len(env.get_agent_state(0))
        agent_net = RNNAgent(input_shape=obs_dim, rnn_hidden_dim=64, n_actions=env.action_space)
        ckpt = torch.load(args.model, map_location='cpu')
        agent_net.load_state_dict(ckpt['agent_net'])
        agent_net.eval()
    else:
        print("Running RANDOM policy for demonstration.")
        args.random = True
        
    visualizer.reset()
    
    if not args.random:
        if visualizer.fig is None:
            visualizer.init_render()
        # Override figure title to clarify this is the final trained MARL policy
        if visualizer.fig is not None:
            visualizer.fig.suptitle("FINAL QMIX SAR POLICY REPLAY", color='green', weight='bold')

    hidden_state = None
    if agent_net is not None:
        hidden_state = agent_net.init_hidden().expand(env.num_drones, -1)
        
    print("Starting visualization. Ensure X11/display is active.")
    for step in range(args.steps):
        local_obs = [env.get_agent_state(i) for i in range(env.num_drones)]
        
        actions = []
        if args.random:
            actions = [np.random.randint(0, env.action_space) for _ in range(env.num_drones)]
        else:
            assert agent_net is not None, "agent_net must be initialized"
            obs_tensor = torch.FloatTensor(np.array(local_obs))
            with torch.no_grad():
                q_vals, hidden_state = agent_net(obs_tensor, hidden_state)
            actions = [q_vals[i].argmax().item() for i in range(env.num_drones)]
            
        _, _, _, done, info = env.step(actions)
        
        visualizer.render(show_hidden_victims=True)
        time.sleep(0.1) # slow down for visual clarity
        
        if done:
            print(f"Episode finished early at step {step}!")
            break
            
    print("Visualization complete. Close the matplotlib window to exit.")
    plt.show(block=True)
    visualizer.close()

if __name__ == '__main__':
    main()
