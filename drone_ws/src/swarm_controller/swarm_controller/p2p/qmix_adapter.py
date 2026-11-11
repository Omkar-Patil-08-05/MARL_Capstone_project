"""
Decentralized QMIX Policy Adapter for P2P Drone Nodes
Protocol Version: 2.1-P2P
Interfaces independent drone controllers to the trained canonical QMIX checkpoint.
Guarantees independent policy evaluation, isolated recurrent hidden state,
and strict validation of the 29-dimensional observation space.
"""

import hashlib
import os
import sys
import numpy as np
import torch

# Ensure repo root is accessible for loading model architecture
candidate_roots = [
    os.path.expanduser('~/capstone_project_antigravity'),
    '/home/capstone/capstone_project_antigravity',
    os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../../../..')),
    os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../..')),
]
for root in candidate_roots:
    if os.path.isdir(os.path.join(root, 'marl_drone_project')) and root not in sys.path:
        sys.path.insert(0, root)

from marl_drone_project.train.networks.qmix import RNNAgent

CANONICAL_CHECKPOINT_SHA256 = "f87d87528969ff3165a0cdfb299f5d228ad99502b974176d05b81479242a272d"
CANONICAL_CHECKPOINT_SIZE = 158042
EXPECTED_OBS_DIM = 29
EXPECTED_HIDDEN_DIM = 64
EXPECTED_ACTIONS = 5


class QMIXAdapter:
    """
    Independent local QMIX policy adapter.
    Each DecentralizedDroneNode instantiates its own QMIXAdapter with private recurrent state.
    """
    def __init__(self, checkpoint_path: str, num_agents: int = 2, device: str = None):
        self.checkpoint_path = os.path.expanduser(checkpoint_path)
        self.num_agents = num_agents
        self.expected_dim = EXPECTED_OBS_DIM
        self.rnn_hidden_dim = EXPECTED_HIDDEN_DIM
        self.n_actions = EXPECTED_ACTIONS

        if device is not None:
            self.device = torch.device(device)
        else:
            self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        # Verify file existence
        if not os.path.isfile(self.checkpoint_path):
            raise FileNotFoundError(f"QMIX checkpoint not found at: {self.checkpoint_path}")

        # Compute file hash and size for verification
        self.file_size = os.path.getsize(self.checkpoint_path)
        with open(self.checkpoint_path, "rb") as f:
            self.file_sha256 = hashlib.sha256(f.read()).hexdigest()

        # Build network architecture matching canonical checkpoint
        self.agent_net = RNNAgent(
            input_shape=self.expected_dim,
            rnn_hidden_dim=self.rnn_hidden_dim,
            n_actions=self.n_actions
        ).to(self.device)

        # Load weights
        checkpoint = torch.load(self.checkpoint_path, map_location=self.device)
        if 'agent_net' in checkpoint:
            self.agent_net.load_state_dict(checkpoint['agent_net'])
        else:
            self.agent_net.load_state_dict(checkpoint)
        self.agent_net.eval()

        # Independent recurrent hidden states
        self.hidden_states = {}
        self.reset()

    def reset(self, agent_id: int = None):
        """Resets recurrent hidden state for a specific agent or all agents."""
        if agent_id is not None:
            self.hidden_states[agent_id] = self.agent_net.init_hidden()
        else:
            for i in range(max(self.num_agents, 2)):
                self.hidden_states[i] = self.agent_net.init_hidden()

    def validate_observation(self, obs_np: np.ndarray) -> bool:
        """Validates that observation has strictly 29 dimensions and all finite values."""
        if not isinstance(obs_np, np.ndarray):
            return False
        if obs_np.shape != (self.expected_dim,):
            return False
        if not np.all(np.isfinite(obs_np)):
            return False
        return True

    def select_action(self, agent_id: int = 0, obs_np: np.ndarray = None) -> int:
        """
        Performs a deterministic greedy inference step for the local agent.
        Validates 29-dimensional observation and updates internal recurrent state.
        Returns the discrete action integer (0: +X, 1: -X, 2: +Y, 3: -Y, 4: Hover).
        """
        if obs_np is None:
            raise ValueError("Observation vector obs_np cannot be None")

        if not self.validate_observation(obs_np):
            raise ValueError(
                f"Invalid observation vector! Shape={getattr(obs_np, 'shape', None)}, "
                f"finite={np.all(np.isfinite(obs_np)) if isinstance(obs_np, np.ndarray) else False}. "
                f"Expected strict ({self.expected_dim},) with all finite values."
            )

        if agent_id not in self.hidden_states:
            self.hidden_states[agent_id] = self.agent_net.init_hidden()

        obs_tensor = torch.as_tensor(obs_np, dtype=torch.float32, device=self.device).unsqueeze(0)

        with torch.no_grad():
            q_vals, new_hidden = self.agent_net(obs_tensor, self.hidden_states[agent_id])

        # Update local recurrent hidden state
        self.hidden_states[agent_id] = new_hidden

        # Deterministic greedy action selection (argmax)
        action = int(q_vals.argmax().item())
        return action

    def get_checkpoint_metadata(self) -> dict:
        return {
            "checkpoint_path": self.checkpoint_path,
            "sha256": self.file_sha256,
            "size_bytes": self.file_size,
            "is_canonical": (self.file_sha256 == CANONICAL_CHECKPOINT_SHA256 and self.file_size == CANONICAL_CHECKPOINT_SIZE),
            "input_dimension": self.expected_dim,
            "hidden_dimension": self.rnn_hidden_dim,
            "action_dimension": self.n_actions,
            "device": str(self.device)
        }
