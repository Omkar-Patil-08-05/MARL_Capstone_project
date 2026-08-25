import torch
import torch.nn as nn
import torch.nn.functional as F


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
        # Initialize hidden state on the same device as the model's weights
        return self.fc1.weight.new(1, self.rnn_hidden_dim).zero_()

    def forward(self, inputs, hidden_state):
        x = F.relu(self.fc1(inputs))
        h_in = hidden_state.reshape(-1, self.rnn_hidden_dim)
        h = self.rnn(x, h_in)
        q = self.fc2(h)
        return q, h


class QMixer(nn.Module):
    """
    Proper Monotonic QMIX Mixer.
    Generates non-negative weights using hypernetworks conditioned on the true global state.
    """
    def __init__(self, n_agents, state_dim, mixing_embed_dim=32, hypernet_embed=64):
        super(QMixer, self).__init__()
        self.n_agents = n_agents
        self.state_dim = state_dim
        self.embed_dim = mixing_embed_dim

        # Hypernetwork 1: outputs weights for layer 1
        # Generates a weight matrix of shape (n_agents, embed_dim)
        self.hyper_w_1 = nn.Sequential(
            nn.Linear(self.state_dim, hypernet_embed),
            nn.ReLU(),
            nn.Linear(hypernet_embed, self.embed_dim * self.n_agents)
        )
        
        # Hypernetwork 2: outputs weights for layer 2
        # Generates a weight matrix of shape (embed_dim, 1)
        self.hyper_w_final = nn.Sequential(
            nn.Linear(self.state_dim, hypernet_embed),
            nn.ReLU(),
            nn.Linear(hypernet_embed, self.embed_dim)
        )

        # State dependent biases
        self.hyper_b_1 = nn.Linear(self.state_dim, self.embed_dim)
        self.hyper_b_final = nn.Sequential(
            nn.Linear(self.state_dim, self.embed_dim),
            nn.ReLU(),
            nn.Linear(self.embed_dim, 1)
        )

    def forward(self, agent_qs, states):
        """
        agent_qs: [batch_size, seq_len, n_agents]
        states: [batch_size, seq_len, state_dim]
        """
        bs = agent_qs.size(0)
        states = states.reshape(-1, self.state_dim)
        agent_qs = agent_qs.view(-1, 1, self.n_agents)
        
        # First layer (Monotonic: Absolute weights)
        w1 = torch.abs(self.hyper_w_1(states))
        b1 = self.hyper_b_1(states)
        w1 = w1.view(-1, self.n_agents, self.embed_dim)
        b1 = b1.view(-1, 1, self.embed_dim)
        
        hidden = F.elu(torch.bmm(agent_qs, w1) + b1)
        
        # Second layer (Monotonic: Absolute weights)
        w_final = torch.abs(self.hyper_w_final(states))
        w_final = w_final.view(-1, self.embed_dim, 1)
        b_final = self.hyper_b_final(states).view(-1, 1, 1)
        
        # Output calculation
        y = torch.bmm(hidden, w_final) + b_final
        
        # Reshape to [batch_size, seq_len, 1]
        q_tot = y.view(bs, -1, 1)
        return q_tot
