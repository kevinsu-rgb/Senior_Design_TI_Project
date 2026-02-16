from torch import nn


class LinearModel(nn.Module):
    def __init__(self, input_size: int, output_size: int):
        super(LinearModel, self).__init__()
        self.bn1 = nn.BatchNorm1d(num_features=input_size)
        self.fc1 = nn.Linear(in_features=input_size, out_features=64)
        self.bn2 = nn.BatchNorm1d(num_features=64)
        self.fc2 = nn.Linear(in_features=64, out_features=32)
        self.bn3 = nn.BatchNorm1d(num_features=32)
        self.fc3 = nn.Linear(in_features=32, out_features=16)
        self.bn4 = nn.BatchNorm1d(num_features=16)
        self.fc4 = nn.Linear(in_features=16, out_features=output_size)
        self.relu = nn.ReLU()

    def get_weights(self):
        return self.weight

    def forward(self, x):
        out = self.bn1(x)
        out = self.fc1(out)
        out = self.relu(out)
        out = self.bn2(out)
        out = self.fc2(out)
        out = self.relu(out)
        out = self.bn3(out)
        out = self.fc3(out)
        out = self.bn4(out)
        out = self.fc4(out)

        out = torch.softmax(out, dim=1)

        return out
