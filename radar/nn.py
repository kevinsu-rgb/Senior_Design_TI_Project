from torch import nn
import torch

class NeuralNetwork(nn.Module):
    '''
    This is modified to have two branches...
    Point Cloud -> Linear -> Fusion -> Linear -> Out
    Heatmap -> Convlayer ->  ^
    '''
    def __init__(self, input_size, output_size):
        super(LinearModel, self).__init__()

        self.pc_branch = nn.Sequential(
            nn.BatchNorm1d(num_features=input_size),
            nn.Linear(in_features=input_size, out_features=64),
            nn.ReLU(),
            nn.BatchNorm1d(num_features=64),
            nn.Linear(in_features=64, out_features=32),
            nn.ReLU()
        )

        self.conv_layers = nn.Sequential(
            nn.Conv2d(1, 16, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(16, 32, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.AdaptiveAvgPool2d((4, 4)),
            nn.Flatten()
        )

        combined_size = 32 + 512

        self.fusion_head = nn.Sequential(
            nn.Linear(combined_size, 64),
            nn.BatchNorm1d(64),
            nn.ReLU(),
            nn.Linear(64, output_size)
        )

    def forward(self, xp, xh):
        out_p = self.pc_branch(xp) # Shape: [Batch, 32]

        # xh_shape: (BATCH_SIZE, 1, WINDOW_SIZE, 32, 32)
        # Reshape xh to treat each heatmap in the window as a separate item for Conv2d
        batch_size, channels, window_size, h, w = xh.shape
        xh_reshaped = xh.view(batch_size * window_size, channels, h, w) # Shape: (B*W, 1, 32, 32)

        out_h_per_frame = self.conv_layers(xh_reshaped) # Shape: (B*W, 512)

        # Aggregate the features from each frame in the window
        out_h = out_h_per_frame.view(batch_size, window_size, -1) # Shape: (B, W, 512)
        out_h = torch.mean(out_h, dim=1) # Aggregate by taking mean across window_size dimension. Shape: (B, 512)

        combined = torch.cat((out_p, out_h), dim=1)

        out = self.fusion_head(combined)

        return torch.softmax(out, dim=1)
