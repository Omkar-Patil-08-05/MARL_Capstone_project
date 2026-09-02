"""
QMIX RNNAgent inference wrapper for ROS 2 deployment.

Loads the trained N=6 QMIX checkpoint and performs batched greedy
policy inference while maintaining persistent GRU hidden states
across timesteps within an episode.

The RNNAgent architecture is duplicated here (not imported) to keep
the ROS 2 package self-contained without a runtime dependency on the
training project's Python path.  The architecture is byte-identical
to marl_drone_project/train/networks/qmix.py and must stay in sync.
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np


# ── Exact copy of the trained RNNAgent architecture ──────────────
# Source: marl_drone_project/train/networks/qmix.py  (lines 6-29)
# DO NOT modify without also verifying the checkpoint compatibility.

class RNNAgent(nn.Module):
    """
    Shared parameter recurrent agent network.
    All agents use this single network and distinguish themselves via the one-hot
    agent_id in the input observation.
    """
    def __init__(self, input_shape, rnn_hidden_dim=64, n_actions=5):
        super(RNNAgent, self).__init__()
        self.rnn_hidden_dim = rnn_hidden_dim

        self.fc1 = nn.Linear(input_shape, rnn_hidden_dim)
        self.rnn = nn.GRUCell(rnn_hidden_dim, rnn_hidden_dim)
        self.fc2 = nn.Linear(rnn_hidden_dim, n_actions)

    def init_hidden(self):
        return self.fc1.weight.new(1, self.rnn_hidden_dim).zero_()

    def forward(self, inputs, hidden_state):
        x = F.relu(self.fc1(inputs))
        h_in = hidden_state.reshape(-1, self.rnn_hidden_dim)
        h = self.rnn(x, h_in)
        q = self.fc2(h)
        return q, h


# ── Inference wrapper ────────────────────────────────────────────

class QMIXInference:
    """
    Stateful greedy QMIX inference for N=6 deployment.

    Usage
    -----
    >>> qmix = QMIXInference(model_path, num_drones=6)
    >>> qmix.reset_episode()
    >>> actions = qmix.get_actions(observations)   # observations: (6, 49)
    """

    # Action semantics (matches SARGridEnv)
    ACTION_PLUS_X  = 0
    ACTION_MINUS_X = 1
    ACTION_PLUS_Y  = 2
    ACTION_MINUS_Y = 3
    ACTION_HOVER   = 4
    NUM_ACTIONS    = 5

    def __init__(
        self,
        model_path: str,
        num_drones: int = 6,
        obs_dim: int = 49,
        rnn_hidden_dim: int = 64,
        device: str = "auto",
    ):
        self.num_drones = num_drones
        self.obs_dim = obs_dim
        self.rnn_hidden_dim = rnn_hidden_dim

        # Resolve device
        if device == "auto":
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        else:
            self.device = torch.device(device)

        # Instantiate network with the exact training architecture
        self.agent_net = RNNAgent(
            input_shape=obs_dim,
            rnn_hidden_dim=rnn_hidden_dim,
            n_actions=self.NUM_ACTIONS,
        ).to(self.device)

        # Load checkpoint
        ckpt = torch.load(model_path, map_location=self.device, weights_only=False)
        self.agent_net.load_state_dict(ckpt["agent_net"])
        self.agent_net.eval()

        # Hidden state: (num_drones, rnn_hidden_dim)
        self.hidden_state = None
        self.reset_episode()

        print(f"✅ QMIXInference ready  |  device={self.device}  |  "
              f"obs_dim={obs_dim}  hidden=({num_drones},{rnn_hidden_dim})")

    # ── Episode lifecycle ────────────────────────────────────────

    def reset_episode(self):
        """Zero-initialize the GRU hidden state for all drones."""
        h0 = self.agent_net.init_hidden()                    # (1, 64)
        self.hidden_state = h0.expand(self.num_drones, -1).contiguous().to(self.device)
        # shape: (num_drones, rnn_hidden_dim)

    # ── Inference ────────────────────────────────────────────────

    def get_actions(self, observations) -> list[int]:
        """
        Batched greedy inference for all drones.

        Parameters
        ----------
        observations : array-like of shape (num_drones, obs_dim)
            Six 49-dimensional float32 observation vectors.

        Returns
        -------
        actions : list[int]
            Six discrete actions in {0,1,2,3,4}.
        """
        obs = np.asarray(observations, dtype=np.float32)
        assert obs.shape == (self.num_drones, self.obs_dim), (
            f"Expected observations shape ({self.num_drones}, {self.obs_dim}), "
            f"got {obs.shape}"
        )

        obs_tensor = torch.from_numpy(obs).to(self.device)  # (6, 49)

        with torch.no_grad():
            q_vals, self.hidden_state = self.agent_net(obs_tensor, self.hidden_state)
            # q_vals: (6, 5),  hidden_state: (6, 64)

        actions = q_vals.argmax(dim=1).cpu().tolist()         # greedy
        return actions