import numpy as np
import matplotlib.pyplot as plt
import torch
import torch.nn as nn
import torch.optim as optim
try:
    import torch
    import torch.nn as nn
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False
    print("Warning: PyTorch not available. VAE functionality will be disabled.")

class LidarVAE(nn.Module):
    def __init__(self, input_dim=640, hidden_dims=[512, 256, 128], latent_dim=32):
        """Variational Autoencoder for Lidar Scan Data"""
        super(LidarVAE, self).__init__()
        self.input_dim = input_dim
        self.latent_dim = latent_dim
        
        # Encoder
        encoder_layers = []
        prev_dim = input_dim
        for hidden_dim in hidden_dims:
            encoder_layers.extend([
                nn.Linear(prev_dim, hidden_dim),
                nn.ReLU(),
                nn.BatchNorm1d(hidden_dim)
            ])
            prev_dim = hidden_dim
        
        self.encoder = nn.Sequential(*encoder_layers)
        
        # Latent space layers
        self.fc_mu = nn.Linear(prev_dim, latent_dim)
        self.fc_logvar = nn.Linear(prev_dim, latent_dim)
        
        # Decoder
        decoder_layers = []
        prev_dim = latent_dim
        for hidden_dim in reversed(hidden_dims):
            decoder_layers.extend([
                nn.Linear(prev_dim, hidden_dim),
                nn.ReLU(),
                nn.BatchNorm1d(hidden_dim)
            ])
            prev_dim = hidden_dim
        
        decoder_layers.append(nn.Linear(prev_dim, input_dim))
        decoder_layers.append(nn.Sigmoid())
        self.decoder = nn.Sequential(*decoder_layers)
    
    def encode(self, x):
        h = self.encoder(x)
        mu = self.fc_mu(h)
        logvar = self.fc_logvar(h)
        return mu, logvar
    
    def reparameterize(self, mu, logvar):
        std = torch.exp(0.5 * logvar)
        eps = torch.randn_like(std)
        return mu + eps * std
    
    def decode(self, z):
        return self.decoder(z)
    
    def forward(self, x):
        mu, logvar = self.encode(x)
        z = self.reparameterize(mu, logvar)
        recon_x = self.decode(z)
        return recon_x, mu, logvar

def load_vae_model(model_path='/home/mehdi/NerualRateMaps/lidar_vae_model.pth', device=None):
    """
    Load a trained VAE model from checkpoint.
    
    Args:
        model_path: Path to the saved model checkpoint
        device: Device to load model on ('cuda', 'cpu', or None for auto)
    
    Returns:
        tuple: (model, data_min, data_max) for normalization
    """
    import os
    
    if not TORCH_AVAILABLE:
        raise ImportError("PyTorch is required to load VAE model")
    
    # Convert to absolute path
    model_path = os.path.abspath(model_path)
    
    # Check if file exists
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"VAE model file not found: {model_path}")
    
    # Check if file is empty or corrupted
    file_size = os.path.getsize(model_path)
    if file_size == 0:
        raise ValueError(f"VAE model file is empty: {model_path}")
    
    if file_size < 1000:  # Model files should be at least a few KB
        raise ValueError(f"VAE model file seems too small ({file_size} bytes): {model_path}. File may be corrupted.")
    
    if device is None:
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    else:
        device = torch.device(device)
    
    # Load checkpoint - try multiple methods for robustness
    import io
    import zipfile
    
    checkpoint = None

    # Explicitly set weights_only=False to suppress warning and ensure compatibility
    # Note: This is safe if you trust the model file source
    try:
        checkpoint = torch.load(model_path, map_location=device, weights_only=False)
    except RuntimeError as e:
        # If weights_only=False fails, try with pickle_module explicitly
        if "archive does not contain any files" in str(e):
            raise RuntimeError(
                f"VAE model file appears to be corrupted or empty: {model_path}. "
                f"Please check if the file was saved correctly. Error: {e}"
            ) from e
        # Try alternative loading method
        try:
            checkpoint = torch.load(model_path, map_location=device, pickle_module=None)
        except Exception as e2:
            raise RuntimeError(
                f"Failed to load VAE model from {model_path}. "
                f"Original error: {e}, Secondary error: {e2}"
            ) from e2

    # Re-create model with same architecture params
    model = LidarVAE(
        input_dim=checkpoint["input_dim"],
        latent_dim=checkpoint["latent_dim"],
        hidden_dims=checkpoint["hidden_dims"],
    ).to(device)

    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()

    
    # Get normalization parameters and convert to Python scalars
    data_min = checkpoint['data_min']
    data_max = checkpoint['data_max']
    
    # Convert torch tensors to Python scalars if needed
    if isinstance(data_min, torch.Tensor):
        data_min = data_min.item()
    if isinstance(data_max, torch.Tensor):
        data_max = data_max.item()
    
    print(f"VAE model loaded from {model_path}")
    print(f"  Device: {device}")
    print(f"  Input dim: {checkpoint['input_dim']}")
    print(f"  Latent dim: {checkpoint['latent_dim']}")
    print(f"  Data range: [{data_min:.4f}, {data_max:.4f}]")
    
    return model, data_min, data_max

if __name__ == "__main__":
    model, data_min, data_max = load_vae_model()
    print(model)
    print(data_min)
    print(data_max)
