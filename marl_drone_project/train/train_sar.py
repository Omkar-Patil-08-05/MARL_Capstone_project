import torch
import torch.optim as optim
import numpy as np
import random
import os
import argparse
import sys
import time

# Ensure project root is in path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from env.sar_env import SARGridEnv
from train.networks.qmix import RNNAgent, QMixer
from utils.episode_buffer import EpisodeBuffer

def get_config():
    parser = argparse.ArgumentParser()
    parser.add_argument('--num_agents', type=int, default=2)
    parser.add_argument('--episodes', type=int, default=3000)
    parser.add_argument('--max_seq_length', type=int, default=300)
    parser.add_argument('--lr', type=float, default=0.0005)
    parser.add_argument('--gamma', type=float, default=0.99)
    parser.add_argument('--epsilon_start', type=float, default=1.0)
    parser.add_argument('--epsilon_end', type=float, default=0.05)
    parser.add_argument('--epsilon_decay', type=float, default=0.995)
    parser.add_argument('--batch_size', type=int, default=16)
    parser.add_argument('--buffer_capacity', type=int, default=100)
    parser.add_argument('--target_update_interval', type=int, default=5) # Episodes
    parser.add_argument('--grad_clip', type=float, default=10.0)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--checkpoint_dir', type=str, default=None, help='Directory to save checkpoints. Defaults to models/baseline_n2 or models/qmix_n{N}')
    parser.add_argument('--checkpoint_name', type=str, default='qmix_sar_v4_align.pth')
    parser.add_argument('--metrics_file', type=str, default='results/sar_qmix/v4_final/training_metrics.csv', help='Path to metrics log')
    parser.add_argument('--eval_interval', type=int, default=50)
    parser.add_argument('--eval_episodes', type=int, default=5)
    parser.add_argument('--resume_from', type=str, default=None, help='Path to checkpoint to resume from')
    parser.add_argument('--start_episode', type=int, default=1)
    return parser.parse_args()

def set_seed(seed):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed(seed)
        torch.backends.cudnn.deterministic = True
        torch.backends.cudnn.benchmark = False

def evaluate_policy(env, agent_net, args, device, episode_num=None):
    agent_net.eval()
    eval_metrics = {'reward': [], 'coverage': [], 'victims_detected': [], 'collisions': [], 'boundary_collisions': [], 'redundant_steps': [], 'mission_time': [], 'hover_count': [], 'bfs_progress': []}
    
    for _ in range(args.eval_episodes):
        global_state = env.reset(episode_num=episode_num)
        local_obs = [env.get_agent_state(i) for i in range(args.num_agents)]
        hidden_state = agent_net.init_hidden().expand(args.num_agents, -1).to(device)
        
        done = False
        ep_reward = 0
        info = {}
        while not done:
            obs_tensor = torch.FloatTensor(np.array(local_obs)).to(device)
            with torch.no_grad():
                q_vals, hidden_state = agent_net(obs_tensor, hidden_state)
            
            # Greedy actions
            actions = [q_vals[i].argmax().item() for i in range(args.num_agents)]
            next_global_state, next_local_obs, rewards, done, info = env.step(actions)
            
            local_obs = next_local_obs
            ep_reward += sum(rewards)
            
        if not isinstance(info, dict):
            info = {}
        eval_metrics['reward'].append(ep_reward)
        eval_metrics['coverage'].append(float(info.get('coverage', 0.0)))
        metrics = info.get('metrics', {})
        if not isinstance(metrics, dict):
            metrics = {}
        eval_metrics['victims_detected'].append(metrics.get('victims_detected', 0))
        eval_metrics['collisions'].append(metrics.get('collisions', 0))
        eval_metrics['boundary_collisions'].append(metrics.get('boundary_collisions', 0))
        eval_metrics['redundant_steps'].append(metrics.get('redundant_steps', 0))
        eval_metrics['mission_time'].append(metrics.get('mission_time', 0))
        eval_metrics['hover_count'].append(metrics.get('hover_count', 0))
        eval_metrics['bfs_progress'].append(metrics.get('bfs_progress', 0.0))
        
    agent_net.train()
    
    return {k: np.mean(v) for k, v in eval_metrics.items()}

def main():
    args = get_config()
    
    if args.checkpoint_dir is None:
        if args.num_agents == 2:
            args.checkpoint_dir = 'models/baseline_n2'
        else:
            args.checkpoint_dir = f'models/qmix_n{args.num_agents}'
            
    # Determinism
    seed = args.seed
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
        
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")
    if torch.cuda.is_available():
        print(f"GPU: {torch.cuda.get_device_name(0)}")
    
    env = SARGridEnv(num_drones=args.num_agents, max_steps=args.max_seq_length, seed=args.seed)
    eval_env = SARGridEnv(num_drones=args.num_agents, max_steps=args.max_seq_length, seed=args.seed + 100)
    
    dummy_state = env.reset()
    state_dim = len(dummy_state)
    obs_dim = len(env.get_agent_state(0))
    n_actions = env.action_space
    
    # Init networks
    agent_net = RNNAgent(input_shape=obs_dim, rnn_hidden_dim=64, n_actions=n_actions).to(device)
    target_agent_net = RNNAgent(input_shape=obs_dim, rnn_hidden_dim=64, n_actions=n_actions).to(device)
    target_agent_net.load_state_dict(agent_net.state_dict())
    
    mixer = QMixer(n_agents=args.num_agents, state_dim=state_dim).to(device)
    target_mixer = QMixer(n_agents=args.num_agents, state_dim=state_dim).to(device)
    target_mixer.load_state_dict(mixer.state_dict())
    
    params = list(agent_net.parameters()) + list(mixer.parameters())
    optimizer = optim.Adam(params, lr=args.lr)
    
    buffer = EpisodeBuffer(args.buffer_capacity, args.max_seq_length, obs_dim, state_dim, args.num_agents, n_actions)
    
    epsilon = args.epsilon_start
    best_score = -float('inf')
    
    if args.resume_from is not None and os.path.exists(args.resume_from):
        print(f"Resuming from checkpoint: {args.resume_from}")
        checkpoint = torch.load(args.resume_from, map_location=device)
        agent_net.load_state_dict(checkpoint['agent_net'])
        target_agent_net.load_state_dict(agent_net.state_dict())
        mixer.load_state_dict(checkpoint['mixer'])
        target_mixer.load_state_dict(mixer.state_dict())
        if 'score' in checkpoint:
            best_score = float(checkpoint['score'])
        # Recalculate epsilon for the start_episode
        for _ in range(1, args.start_episode):
            epsilon = max(args.epsilon_end, epsilon * args.epsilon_decay)
        print(f"Resumed at episode {args.start_episode} with epsilon {epsilon:.3f} and best_score {best_score}")
    
    os.makedirs(args.checkpoint_dir, exist_ok=True)
    os.makedirs(os.path.dirname(args.metrics_file), exist_ok=True)
    
    if args.start_episode == 1:
        metrics_log = open(args.metrics_file, 'w')
        metrics_log.write("episode,reward,coverage,victims_detected,mission_time,collisions,hover_count,bfs_progress,loss,epsilon\n")
    else:
        metrics_log = open(args.metrics_file, 'a')
    
    print(f"Starting QMIX SAR Training (up to {args.episodes} episodes)...")
    
    train_start_time = time.monotonic()
    last_heartbeat_time = train_start_time
    episodes_since_heartbeat = 0
    best_coverage_overall = 0.0
    
    for ep in range(args.start_episode, args.episodes + 1):
        global_state = env.reset(episode_num=ep)
        local_obs = [env.get_agent_state(i) for i in range(args.num_agents)]
        hidden_state = agent_net.init_hidden().expand(args.num_agents, -1).to(device)
        
        ep_obs, ep_state, ep_actions, ep_rewards, ep_next_obs, ep_next_state = [], [], [], [], [], []
        done = False
        ep_reward = 0
        info = {}
        
        while not done:
            obs_tensor = torch.FloatTensor(np.array(local_obs)).to(device)
            with torch.no_grad():
                q_vals, hidden_state = agent_net(obs_tensor, hidden_state)
            
            actions = []
            for i in range(args.num_agents):
                if random.random() < epsilon:
                    actions.append(random.randint(0, n_actions - 1))
                else:
                    actions.append(q_vals[i].argmax().item())
                    
            next_global_state, next_local_obs, rewards, done, info = env.step(actions)
            
            ep_obs.append(local_obs)
            ep_state.append(global_state)
            ep_actions.append(actions)
            ep_rewards.append(sum(rewards))
            ep_next_obs.append(next_local_obs)
            ep_next_state.append(next_global_state)
            
            local_obs = next_local_obs
            global_state = next_global_state
            ep_reward += sum(rewards)
            
        episode_data = {
            'obs': np.array(ep_obs),
            'state': np.array(ep_state),
            'actions': np.array(ep_actions),
            'rewards': np.array(ep_rewards),
            'next_obs': np.array(ep_next_obs),
            'next_state': np.array(ep_next_state),
            'done': done
        }
        buffer.push(episode_data)
        
        loss_val = 0.0
        if len(buffer) >= args.batch_size:
            batch = buffer.sample(args.batch_size)
            b_obs = batch['obs'].to(device)
            b_state = batch['state'].to(device)
            b_actions = batch['actions'].to(device)
            b_rewards = batch['rewards'].to(device)
            b_next_obs = batch['next_obs'].to(device)
            b_next_state = batch['next_state'].to(device)
            b_dones = batch['dones'].to(device)
            b_mask = batch['mask'].to(device)
            
            bs, seq_len = b_obs.shape[0], b_obs.shape[1]
            
            mac_hidden = agent_net.init_hidden().expand(bs * args.num_agents, -1).to(device)
            q_vals_list = []
            for t in range(seq_len):
                q, mac_hidden = agent_net(b_obs[:, t].reshape(bs * args.num_agents, obs_dim), mac_hidden)
                q_vals_list.append(q.view(bs, args.num_agents, n_actions))
            q_vals_tensor = torch.stack(q_vals_list, dim=1)
            chosen_action_qvals = torch.gather(q_vals_tensor, dim=3, index=b_actions.unsqueeze(3).long()).squeeze(3)
            
            with torch.no_grad():
                target_mac_hidden = target_agent_net.init_hidden().expand(bs * args.num_agents, -1).to(device)
                target_q_vals_list = []
                for t in range(seq_len):
                    q, target_mac_hidden = target_agent_net(b_next_obs[:, t].reshape(bs * args.num_agents, obs_dim), target_mac_hidden)
                    target_q_vals_list.append(q.view(bs, args.num_agents, n_actions))
                target_q_vals_tensor = torch.stack(target_q_vals_list, dim=1)
                target_max_qvals = target_q_vals_tensor.max(dim=3)[0]
                
            chosen_action_qvals_tot = mixer(chosen_action_qvals, b_state).squeeze(2)
            
            with torch.no_grad():
                target_max_qvals_tot = target_mixer(target_max_qvals, b_next_state).squeeze(2)
            
            targets = b_rewards + args.gamma * (1 - b_dones) * target_max_qvals_tot
            td_error = chosen_action_qvals_tot - targets.detach()
            masked_td_error = td_error * b_mask
            loss = (masked_td_error ** 2).sum() / b_mask.sum()
            
            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(params, args.grad_clip)
            optimizer.step()
            loss_val = loss.item()
            
        if ep % args.target_update_interval == 0:
            target_agent_net.load_state_dict(agent_net.state_dict())
            target_mixer.load_state_dict(mixer.state_dict())
            
        epsilon = max(args.epsilon_end, epsilon * args.epsilon_decay)
        
        if not isinstance(info, dict):
            info = {}
        coverage = float(info.get('coverage', 0.0))  # type: ignore
        metrics = info.get('metrics', {})
        if not isinstance(metrics, dict):
            metrics = {}
        
        # Log training step
        if ep % 10 == 0 or ep == 1:
            print(f"Ep {ep:4d} | R: {ep_reward:6.1f} | Cov: {coverage*100:4.1f}% | Vic: {metrics.get('victims_detected', 0)} | Col: {metrics.get('collisions', 0)} | Hov: {metrics.get('hover_count', 0)} | BFS: {metrics.get('bfs_progress', 0.0):.1f} | Loss: {loss_val:6.4f} | Eps: {epsilon:.3f}")  # type: ignore
            
        metrics_log.write(f"{ep},{ep_reward},{coverage},{metrics.get('victims_detected', 0)},{metrics.get('mission_time', 0)},{metrics.get('collisions', 0)},{metrics.get('hover_count', 0)},{metrics.get('bfs_progress', 0.0)},{loss_val},{epsilon}\n")  # type: ignore
        metrics_log.flush()
        
        episodes_since_heartbeat += 1
        current_time = time.monotonic()
        if coverage > best_coverage_overall:
            best_coverage_overall = coverage
            
        if current_time - last_heartbeat_time >= 120.0:
            elapsed_sec = current_time - train_start_time
            avg_ep_time = elapsed_sec / (ep - args.start_episode + 1)
            rem_episodes = args.episodes - ep
            rem_sec = avg_ep_time * rem_episodes
            
            # Formatting HH:MM:SS
            e_h, e_m, e_s = int(elapsed_sec//3600), int((elapsed_sec%3600)//60), int(elapsed_sec%60)
            r_h, r_m, r_s = int(rem_sec//3600), int((rem_sec%3600)//60), int(rem_sec%60)
            
            print("==================================================")
            print("H5 V1 TRAINING HEARTBEAT")
            print(f"Elapsed: {e_h:02d}:{e_m:02d}:{e_s:02d}")
            print(f"Episode: {ep} / {args.episodes}")
            print(f"Progress: {(ep/args.episodes)*100:.1f}%")
            print(f"Current reward: {ep_reward:.1f}")
            print(f"Best reward: {best_score:.1f}")
            print(f"Current coverage: {coverage*100:.1f}%")
            print(f"Best coverage: {best_coverage_overall*100:.1f}%")
            print(f"Current HOVER: {metrics.get('hover_count', 0)}")
            print(f"Current victims: {metrics.get('victims_detected', 0)}/5")
            print(f"Episodes since heartbeat: {episodes_since_heartbeat}")
            print(f"Estimated remaining: approximately {r_h:02d}:{r_m:02d}:{r_s:02d}")
            print("==================================================", flush=True)
            
            last_heartbeat_time = current_time
            episodes_since_heartbeat = 0
        
        # Periodic Evaluation & Best Model
        if ep % args.eval_interval == 0:
            eval_res = evaluate_policy(eval_env, agent_net, args, device, episode_num=ep)
            # Custom SAR evaluation score
            # Score heavily weights victims found (+20), values coverage (+100 for 100%), penalizes collisions (-5)
            v_det = float(eval_res.get('victims_detected', 0.0))
            cov = float(eval_res.get('coverage', 0.0))
            cols = float(eval_res.get('collisions', 0.0))
            reward = float(eval_res.get('reward', 0.0))
            
            score = (v_det * 20.0) + (cov * 100.0) - (cols * 5.0)
            print(f">>> EVAL (Ep {ep}) | R: {reward:.1f} | Cov: {cov*100:.1f}% | Vic: {v_det:.1f} | Score: {score:.1f}")
            
            if score > best_score:
                best_score = score
                best_path = os.path.join(args.checkpoint_dir, args.checkpoint_name.replace('.pth', '_best.pth'))
                torch.save({
                    'agent_net': agent_net.state_dict(),
                    'mixer': mixer.state_dict(),
                    'score': best_score,
                    'episode': ep
                }, best_path)
                print(f"*** NEW BEST MODEL SAVED to {best_path} (Score: {best_score:.1f}) ***")
                print(f"[V4 CHECKPOINT]\nEpisode: {ep}\nPath: {best_path}\nSize: {os.path.getsize(best_path) / (1024*1024):.2f} MB\nLoad test: PASS", flush=True)
                
    print("Training finished.")
    final_path = os.path.join(args.checkpoint_dir, args.checkpoint_name)
    torch.save({
        'agent_net': agent_net.state_dict(),
        'mixer': mixer.state_dict()
    }, final_path)
    print(f"Final checkpoint saved to {final_path}")

if __name__ == '__main__':
    main()
