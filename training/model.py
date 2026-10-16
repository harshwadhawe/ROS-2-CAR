"""The steering network: ImageNet MobileNetV2 + a one-number head."""
import torch
import torch.nn as nn
import torchvision

MAX_STEER = 0.4                 # rad, matches actuator/teleop max_steering_angle


class Policy(nn.Module):
    """N x 3 x H x W RGB 0-255 -> steer rad. Normalisation is inside the model."""
    def __init__(self, pretrained=True):
        super().__init__()
        w = torchvision.models.MobileNet_V2_Weights.IMAGENET1K_V1 if pretrained else None
        self.features = torchvision.models.mobilenet_v2(weights=w).features
        self.head = nn.Sequential(nn.AdaptiveAvgPool2d(1), nn.Flatten(), nn.Dropout(0.3), nn.Linear(1280, 1))
        self.register_buffer('mean', torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1) * 255)
        self.register_buffer('std', torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1) * 255)

    def forward(self, x):
        return torch.tanh(self.head(self.features((x - self.mean) / self.std))).reshape(-1) * MAX_STEER


class Deployed(nn.Module):
    """What the Pi runs: the camera frame as-is (N x H x W x 3, RGB 0-255)."""
    def __init__(self, policy):
        super().__init__()
        self.policy = policy

    def forward(self, img_in):
        return self.policy(img_in.permute(0, 3, 1, 2))
