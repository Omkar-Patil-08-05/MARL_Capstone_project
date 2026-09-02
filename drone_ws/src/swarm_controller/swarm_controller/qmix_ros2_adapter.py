import sys
import os
import torch
import numpy as np

# Ensure project root is in path so we can load RNNAgent
sys.path.append('/home/capstone/capstone_project_antigravity')
from marl_drone_project.train.networks.qmix import RNNAgent

class QMIXAdapter:
    def __init__(self, checkpoint_path, num_agents=2):
        self.num_agents = num_agents
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        # Calculate expected dimension (5N + 19)
        self.expected_dim = 5 * num_agents + 19

        # Load weights temporarily to inspect input shape
        checkpoint = torch.load(checkpoint_path, map_location=self.device)
        # Using a fallback to 29 if shape inspection is brittle, but better to check fc1 weight
        if 'agent_net' in checkpoint and 'fc1.weight' in checkpoint['agent_net']:
            self.input_shape = checkpoint['agent_net']['fc1.weight'].shape[1]
        elif 'agent_net' in checkpoint and 'rnn.weight_ih_l0' in checkpoint['agent_net']: # GRU
            self.input_shape = checkpoint['agent_net']['rnn.weight_ih_l0'].shape[1]
        else:
            # Fallback if we can't introspect
            self.input_shape = self.expected_dim

        if self.input_shape != self.expected_dim:
            raise RuntimeError(f"CRITICAL: QMIX checkpoint expects {self.input_shape}-dimensional observations but N={num_agents} environment produces {self.expected_dim}. Use a compatible checkpoint.")

        self.rnn_hidden_dim = 64
        self.n_actions = 5

        self.agent_net = RNNAgent(
            input_shape=self.input_shape,
            rnn_hidden_dim=self.rnn_hidden_dim,
            n_actions=self.n_actions
        ).to(self.device)

        # Load weights
        checkpoint = torch.load(checkpoint_path, map_location=self.device)
        self.agent_net.load_state_dict(checkpoint['agent_net'])
        self.agent_net.eval()

        # Hidden states per agent
        self.hidden_states = {}
        self.reset()

    def reset(self):
        """Resets the hidden state for all drones."""
        # Initialize hidden state on the same device as the model's weights
        for i in range(self.num_agents):
            self.hidden_states[i] = self.agent_net.init_hidden()

    def select_action(self, agent_id, obs_np):
        """
        Performs a deterministic greedy inference step for a single agent.
        Returns the discrete action integer.
        """
        # Convert observation to tensor
        obs_tensor = torch.FloatTensor(obs_np).unsqueeze(0).to(self.device)

        with torch.no_grad():
            q_vals, new_hidden = self.agent_net(obs_tensor, self.hidden_states[agent_id])

        # Update hidden state
        self.hidden_states[agent_id] = new_hidden

        # Greedy action selection (epsilon = 0)
        action = q_vals.argmax().item()
        return action
