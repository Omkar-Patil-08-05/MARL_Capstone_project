import sys
import os
import torch
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from train.networks.qmix import RNNAgent

def main():
    print("Testing RNNAgent callable...")
    try:
        agent = RNNAgent(input_shape=10, rnn_hidden_dim=64, n_actions=5)
        obs = torch.randn(2, 10)
        h = agent.init_hidden().expand(2, -1)
        q, h = agent(obs, h)
        print("Success")
    except Exception as e:
        print(f"Exception: {repr(e)}")

if __name__ == '__main__':
    main()
