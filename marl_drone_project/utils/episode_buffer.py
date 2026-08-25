import numpy as np
import random
import torch

class EpisodeBuffer:
    def __init__(self, capacity, max_seq_length, obs_dim, global_state_dim, n_agents, n_actions):
        self.capacity = capacity
        self.max_seq_length = max_seq_length
        self.obs_dim = obs_dim
        self.global_state_dim = global_state_dim
        self.n_agents = n_agents
        self.n_actions = n_actions
        
        self.buffer = []
        self.position = 0
        
    def push(self, episode):
        # episode is a dict with numpy arrays of shape (seq_len, ...)
        if len(self.buffer) < self.capacity:
            self.buffer.append(None)
        self.buffer[self.position] = episode
        self.position = (self.position + 1) % self.capacity
        
    def sample(self, batch_size):
        batch = random.sample(self.buffer, batch_size)
        
        # Determine max sequence length in this batch for dynamic padding
        seq_lens = [ep['obs'].shape[0] for ep in batch]
        max_len = max(seq_lens)
        
        obs_batch = np.zeros((batch_size, max_len, self.n_agents, self.obs_dim))
        state_batch = np.zeros((batch_size, max_len, self.global_state_dim))
        actions_batch = np.zeros((batch_size, max_len, self.n_agents))
        rewards_batch = np.zeros((batch_size, max_len))
        
        next_obs_batch = np.zeros((batch_size, max_len, self.n_agents, self.obs_dim))
        next_state_batch = np.zeros((batch_size, max_len, self.global_state_dim))
        
        dones_batch = np.zeros((batch_size, max_len))
        mask_batch = np.zeros((batch_size, max_len))
        
        for i, ep in enumerate(batch):
            sl = seq_lens[i]
            obs_batch[i, :sl] = ep['obs']
            state_batch[i, :sl] = ep['state']
            actions_batch[i, :sl] = ep['actions']
            rewards_batch[i, :sl] = ep['rewards']
            
            next_obs_batch[i, :sl] = ep['next_obs']
            next_state_batch[i, :sl] = ep['next_state']
            
            # Mask indicates valid transitions that contribute to the loss
            mask_batch[i, :sl] = 1.0 
            if ep['done']:
                # The last transition doesn't have a valid next state if done
                # Standard practice is to mask out the termination step's TD error
                # Or set done=1 and let the TD target handle it: r + gamma * (1 - done) * Q
                dones_batch[i, sl-1] = 1.0
                
        return {
            'obs': torch.FloatTensor(obs_batch),
            'state': torch.FloatTensor(state_batch),
            'actions': torch.LongTensor(actions_batch),
            'rewards': torch.FloatTensor(rewards_batch),
            'next_obs': torch.FloatTensor(next_obs_batch),
            'next_state': torch.FloatTensor(next_state_batch),
            'dones': torch.FloatTensor(dones_batch),
            'mask': torch.FloatTensor(mask_batch)
        }
        
    def __len__(self):
        return len(self.buffer)
