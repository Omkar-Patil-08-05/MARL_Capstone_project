import os
import sys
import torch
import numpy as np
import random
import argparse

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from env.sar_env import SARGridEnv
from train.networks.qmix import RNNAgent

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, default='models/qmix_sar_v2.pth')
    parser.add_argument('--episodes', type=int, default=20)
    parser.add_argument('--max_seq_length', type=int, default=300)
    parser.add_argument('--seed', type=int, default=1000)
    parser.add_argument('--stage', type=int, default=3, choices=[1, 2, 3])
    return parser.parse_args()

def evaluate(env, policy, episodes, n_actions, agent_net=None, device=None):
    metrics = {
        'reward': [], 'coverage': [], 'victims_detected': [], 'collisions': [],
        'boundary_collisions': [], 'mission_time': [], 'redundant_steps': []
    }
    
    for _ in range(episodes):
        env.reset(episode_num=5000 if env.curriculum_stage == 3 else (1000 if env.curriculum_stage == 2 else 0))
        local_obs = [env.get_agent_state(i) for i in range(env.num_drones)]
        
        hidden_state = None
        if policy == 'qmix':
            hidden_state = agent_net.init_hidden().expand(env.num_drones, -1).to(device)
            
        done = False
        ep_reward = 0
        
        while not done:
            actions = []
            if policy == 'random':
                actions = [random.randint(0, n_actions - 1) for _ in range(env.num_drones)]
            elif policy == 'qmix':
                obs_tensor = torch.FloatTensor(np.array(local_obs)).to(device)
                with torch.no_grad():
                    q_vals, hidden_state = agent_net(obs_tensor, hidden_state)
                actions = [q_vals[i].argmax().item() for i in range(env.num_drones)]
                
            _, next_local_obs, rewards, done, info = env.step(actions)
            local_obs = next_local_obs
            ep_reward += sum(rewards)
            
        metrics['reward'].append(ep_reward)
        metrics['coverage'].append(info['coverage'])
        metrics['victims_detected'].append(info['metrics']['victims_detected'])
        metrics['collisions'].append(info['metrics']['collisions'])
        metrics['boundary_collisions'].append(info['metrics'].get('boundary_collisions', 0))
        metrics['mission_time'].append(info['metrics']['mission_time'])
        metrics['redundant_steps'].append(info['metrics']['redundant_steps'])
        
    return metrics

def print_metrics(metrics, name):
    cov = np.array(metrics['coverage'])
    print(f"\n--- {name} POLICY ---")
    print(f"Mean Reward:         {np.mean(metrics['reward']):.1f}")
    print(f"Mean Coverage:       {np.mean(cov)*100:.2f}% (Std: {np.std(cov)*100:.2f}%)")
    print(f"Max Coverage:        {np.max(cov)*100:.2f}%")
    print(f"% Reaching >= 99%:   {np.mean(cov >= 0.99)*100:.1f}%")
    print(f"% Reaching 100%:     {np.mean(cov == 1.0)*100:.1f}%")
    print(f"Victims Detected:    {np.mean(metrics['victims_detected']):.2f} / 5 (Rate: {np.mean(np.array(metrics['victims_detected'])==5)*100:.1f}%)")
    print(f"Collisions:          {np.mean(metrics['collisions']):.1f}")
    print(f"Boundary Collisions: {np.mean(metrics['boundary_collisions']):.1f}")
    print(f"Redundant Steps:     {np.mean(metrics['redundant_steps']):.1f}")
    print(f"Mission Time:        {np.mean(metrics['mission_time']):.1f}")

def main():
    args = get_args()
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    
    env = SARGridEnv(num_drones=2, max_steps=args.max_seq_length, seed=args.seed)
    env.curriculum_stage = args.stage
    env.reset(episode_num=5000 if args.stage == 3 else (1000 if args.stage == 2 else 0))
    obs_dim = len(env.get_agent_state(0))
    n_actions = env.action_space
    
    print(f"--- SAR Evaluation ({args.episodes} episodes) ---")
    
    # 1. Random Baseline
    # random_metrics = evaluate(env, 'random', args.episodes, n_actions)
    # print_metrics(random_metrics, "RANDOM")
    
    # 2. Heuristic Feasibility (Imported)
    # print("\n--- HEURISTIC POLICY ---")
    # sys.path.append(os.path.dirname(__file__))
    # try:
    #     from utils.feasibility_test import get_nearest_unexplored, get_action_towards
    #     heuristic_metrics = {
    #         'reward': [], 'coverage': [], 'victims_detected': [], 'collisions': [],
    #         'boundary_collisions': [], 'mission_time': [], 'redundant_steps': []
    #     }
    #     for _ in range(args.episodes):
    #         env.reset(episode_num=5000 if args.stage == 3 else (1000 if args.stage == 2 else 0))
    #         done = False
    #         ep_reward = 0
    #         while not done:
    #             actions = []
    #             target_mask = set()
    #             for d in range(env.num_drones):
    #                 pos = env.drone_positions[d]
    #                 target = get_nearest_unexplored(env, pos, target_mask)
    #                 if target:
    #                     for i in range(-env.fov_radius, env.fov_radius + 1):
    #                         for j in range(-env.fov_radius, env.fov_radius + 1):
    #                             target_mask.add((target[0]+i, target[1]+j))
    #                     a = get_action_towards(pos, target, env)
    #                     actions.append(a)
    #                 else:
    #                     actions.append(4)
    #             _, _, rewards, done, info = env.step(actions)
    #             ep_reward += sum(rewards)
    #             
    #         heuristic_metrics['reward'].append(ep_reward)
    #         heuristic_metrics['coverage'].append(info['coverage'])
    #         heuristic_metrics['victims_detected'].append(info['metrics']['victims_detected'])
    #         heuristic_metrics['collisions'].append(info['metrics']['collisions'])
    #         heuristic_metrics['boundary_collisions'].append(info['metrics'].get('boundary_collisions', 0))
    #         heuristic_metrics['mission_time'].append(info['metrics']['mission_time'])
    #         heuristic_metrics['redundant_steps'].append(info['metrics']['redundant_steps'])
    #     
    #     print_metrics(heuristic_metrics, "HEURISTIC FEASIBILITY")
    # except ImportError:
    #     print("Could not import feasibility_test heuristic.")
    
    # 3. QMIX Policy
    if os.path.exists(args.model):
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        agent_net = RNNAgent(input_shape=obs_dim, rnn_hidden_dim=64, n_actions=n_actions).to(device)
        ckpt = torch.load(args.model, map_location=device)
        agent_net.load_state_dict(ckpt['agent_net'])
        agent_net.eval()
        
        qmix_metrics = evaluate(env, 'qmix', args.episodes, n_actions, agent_net, device)
        print_metrics(qmix_metrics, "QMIX SAR")
    else:
        print(f"\nERROR: Model {args.model} not found.")

if __name__ == '__main__':
    main()
