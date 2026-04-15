import torch.nn as nn
from torch.optim import SGD, Adam
class NeuralNetwork(nn.Module):
    def __init__(self, input_size, output_size):
        super(LinearModel, self).__init__()

        self.pc_branch = nn.Sequential(
            nn.BatchNorm1d(num_features=input_size),
            nn.Linear(input_size, 64),
            nn.ReLU(),
            nn.BatchNorm1d(64),
            nn.Linear(64, 32),
            nn.ReLU()
        )

        self.conv_layers = nn.Sequential(
            nn.Conv2d(1, 16, kernel_size=3, padding='same'),
            nn.ReLU(),
            nn.Conv2d(16, 16, kernel_size=3, padding='same'),
            nn.ReLU(),
            nn.AdaptiveAvgPool2d((4, 4)),  # fixes output to 16*4*4=256
            nn.Flatten()
        )

        self.fusion_head = nn.Sequential(
            nn.Linear(32 + 256, 64),
            nn.BatchNorm1d(64),
            nn.ReLU(),
            nn.Linear(64, output_size)
        )

    def forward(self, xp, xh):
        xp = xp.view(xp.size(0), -1)
        out_p = self.pc_branch(xp)

        batch_size, channels, window_size, h, w = xh.shape
        xh = xh.permute(0, 2, 1, 3, 4).contiguous()
        xh_reshaped = xh.view(batch_size * window_size, channels, h, w)
        out_h = self.conv_layers(xh_reshaped)
        out_h = out_h.view(batch_size, window_size, -1).mean(dim=1)

        combined = torch.cat((out_p, out_h), dim=1)
        return self.fusion_head(combined)
