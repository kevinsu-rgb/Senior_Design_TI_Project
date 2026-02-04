## Imports
from torch import nn


class NeuralNetwork(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv_stack = nn.Sequential(
            nn.Conv2d(1, 16, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Flatten(),
        )

        self.linear_relu_stack = nn.Sequential(
            nn.Linear(16 * 64 * 4, 512), nn.ReLU(), nn.Linear(512, 2)
        )

    def forward(self, x):
        x = self.conv_stack(x)
        logits = self.linear_relu_stack(x)
        return logits
