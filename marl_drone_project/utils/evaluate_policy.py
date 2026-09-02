import argparse
import os
import sys
import json
import time
import numpy as np
import pandas as pd  # type: ignore
import torch
import random
import pickle

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from marl_drone_project.env.sar_env import SARGridEnv  # type: ignore
from marl_drone_project.train.networks.qmix import RNNAgent  # type: ignore

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, required=True, help="Path to best checkpoint")
    parser.add_argument('--num_drones', type=int, default=6)
    parser.add_argument('--episodes', type=int, default=100)
    parser.add_argument('--max_steps', type=int, default=300)
    parser.add_argument('--output_dir', type=str, default='results/sar_qmix')
    return parser.parse_args()

def evaluate():
    args = get_args()

    # Setup directories
    traj_dir = os.path.join(args.output_dir, 'n6_evaluation_trajectories')
    render_dir = os.path.join(args.output_dir, 'n6_evaluation_renders')
    os.makedirs(traj_dir, exist_ok=True)
    os.makedirs(render_dir, exist_ok=True)

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Using device: {device}")

    # Initialize environment once to get dimensions
    env = SARGridEnv(num_drones=args.num_drones, max_steps=args.max_steps)
    env.reset(episode_num=2000) # Ensure curriculum stage 3

    obs_dim = len(env.get_agent_state(0))
    agent_net = RNNAgent(input_shape=obs_dim, rnn_hidden_dim=64, n_actions=env.action_space).to(device)

    print(f"Loading checkpoint from {args.model}...")
    ckpt = torch.load(args.model, map_location=device)
    agent_net.load_state_dict(ckpt['agent_net'])
    agent_net.eval()

    results = []

    print(f"Starting {args.episodes} Evaluation Episodes...")
    for ep in range(1, args.episodes + 1):
        # Generate random seed per episode
        ep_seed = np.random.randint(0, 1000000)
        np.random.seed(ep_seed)
        random.seed(ep_seed)

        env = SARGridEnv(num_drones=args.num_drones, max_steps=args.max_steps, seed=ep_seed)
        env.reset(episode_num=2000) # Curriculum 3

        trajectory_steps = []
        trajectory = {
            'episode': ep,
            'seed': ep_seed,
            'initial_drone_positions': [list(p) for p in env.drone_positions],
            'victim_positions': {str(k): v for k, v in env.victims.items()},
            'grid_shape': env.grid.shape,
            'steps': trajectory_steps
        }

        hidden_state = agent_net.init_hidden().expand(args.num_drones, -1).to(device)
        done = False
        info = {'coverage': 0.0, 'metrics': env.metrics}

        for step in range(args.max_steps):
            local_obs = [env.get_agent_state(i) for i in range(args.num_drones)]
            obs_tensor = torch.FloatTensor(np.array(local_obs)).to(device)

            with torch.no_grad():
                q_vals, hidden_state = agent_net(obs_tensor, hidden_state)

            actions = [q_vals[i].argmax().item() for i in range(args.num_drones)]
            _, _, step_rewards, done, info = env.step(actions)

            # Record trajectory step
            trajectory_steps.append({
                'step': step,
                'drone_positions': [list(p) for p in env.drone_positions],
                'actions': actions,
                'rewards': step_rewards,
                'coverage': info['coverage'],
                'metrics': info['metrics'].copy()
            })

            if done:
                break

        metrics = env.metrics
        coverage = info['coverage']
        reward = sum([sum(s['rewards']) for s in trajectory_steps])
        victims = metrics['victims_detected']
        collisions = metrics['collisions']
        hover = metrics['hover_count']
        steps = env.current_step
        sar_score = (coverage * 100) + (victims * 20) - (collisions * 5)

        # Save trajectory
        traj_path = os.path.join(traj_dir, f'episode_{ep:03d}.pkl')
        with open(traj_path, 'wb') as f:
            pickle.dump(trajectory, f)

        res = {
            'episode': ep,
            'seed': ep_seed,
            'reward': reward,
            'coverage': coverage,
            'victims_detected': victims,
            'mission_time': steps,
            'collisions': collisions,
            'hover_count': hover,
            'loss': 'N/A',
            'epsilon': 0.0,
            'sar_score': sar_score
        }
        results.append(res)

        print(f"Episode {ep}/{args.episodes} | Rew: {reward:.2f} | Cov: {coverage:.2f} | Vic: {victims} | Col: {collisions} | Hov: {hover} | Steps: {steps} | SAR: {sar_score:.2f}")

    # Aggregation
    df = pd.DataFrame(results)
    stats_path = os.path.join(args.output_dir, 'n6_evaluation_stats.csv')
    df.to_csv(stats_path, index=False)

    print("\n========================================")
    print("N=6 DETERMINISTIC EVALUATION COMPLETE")
    print("========================================")
    print(f"Episodes: {args.episodes}")
    print("Epsilon: 0")
    print(f"\nMean Coverage: {float(df['coverage'].mean()):.3f}")
    print(f"Mean Victims: {float(df['victims_detected'].mean()):.2f}")
    print(f"5/5 Victim Rate: {float((df['victims_detected'] == 5).mean())*100:.2f}%")
    print(f"Mean Collisions: {float(df['collisions'].mean()):.2f}")
    print(f"Mean Hover: {float(df['hover_count'].mean()):.2f}")
    print(f"Mean Reward: {float(df['reward'].mean()):.2f}")
    print(f"Mean SAR Score: {float(df['sar_score'].mean()):.2f}")

    print(f"\n>=80% Coverage: {float((df['coverage'] >= 0.8).mean())*100:.2f}%")
    print(f">=90% Coverage: {float((df['coverage'] >= 0.9).mean())*100:.2f}%")
    print(f">=95% Coverage: {float((df['coverage'] >= 0.95).mean())*100:.2f}%")

    print(f"\nPositive Reward Rate: {float((df['reward'] > 0).mean())*100:.2f}%")
    hq_success = float(((df['coverage'] >= 0.9) & (df['victims_detected'] == 5) & (df['collisions'] <= 20)).mean()) * 100
    print(f"High-Quality Search Success: {hq_success:.2f}%")

    best_idx = int(df['sar_score'].argmax())
    worst_idx = int(df['sar_score'].argmin())
    best_sar_ep = int(df['episode'].iloc[best_idx])
    worst_sar_ep = int(df['episode'].iloc[worst_idx])
    print(f"\nBest SAR Episode: {best_sar_ep}")
    print(f"Worst SAR Episode: {worst_sar_ep}")

    # Save Metadata
    meta = {
        'checkpoint_path': args.model,
        'checkpoint_filename': os.path.basename(args.model),
        'num_drones': args.num_drones,
        'episodes': args.episodes,
        'max_steps': args.max_steps,
        'epsilon': 0.0,
        'evaluation_timestamp': time.time(),
        'environment_class': 'SARGridEnv',
        'agent_class': 'RNNAgent',
        'randomization_mode': 'per-episode numpy random seed'
    }
    with open(os.path.join(args.output_dir, 'n6_evaluation_metadata.json'), 'w') as f:
        json.dump(meta, f, indent=4)

    print("\n[NOTE] Existing rendering utilities (generate_gif.py) expect to run the environment live and cannot directly consume the serialized trajectories. A follow-up adapter is required to render n6_eval_best_sar.gif without rerunning the environment step-by-step.")

if __name__ == '__main__':
    evaluate()
