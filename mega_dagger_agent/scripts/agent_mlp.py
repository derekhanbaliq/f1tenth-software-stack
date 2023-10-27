#!/usr/bin/env python3

import torch
from torch import nn, optim


class AgentPolicyMLP(nn.Module):
    """
    The AgentPolicy consists of a two-layer MLP.
    """

    def __init__(self, observ_dim, hidden_dim, action_dim, lr, device):
        super().__init__()
        self.mlp = nn.Sequential(
            nn.Linear(observ_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim)
        )
        self.mlp.to(device)
        self.optimizer = optim.Adam(self.mlp.parameters(), lr)
        self.loss_function = nn.MSELoss()
        self.device = device

    def forward(self, observ_tensor: torch.FloatTensor):
        return self.mlp(observ_tensor)

    def train(self, observs, actions):
        """
        Trains the agent given a batch of observations and expert-labeled actions.
        
        Returns: the loss.
        """
        observ_tensor = torch.as_tensor(observs, dtype=torch.float32, device=self.device)
        action_tensor = torch.as_tensor(actions, dtype=torch.float32, device=self.device)
        self.optimizer.zero_grad()
        pred_action = self(observ_tensor)
        loss = self.loss_function(pred_action, action_tensor)
        loss.backward()
        self.optimizer.step()
        return loss.item()

    def get_action(self, observ):
        """
        Predicts an action given an observation.
        """
        observ_tensor = torch.as_tensor(observ, dtype=torch.float32, device=self.device)
        action_tensor = self(observ_tensor)
        return action_tensor.detach().cpu().numpy()
