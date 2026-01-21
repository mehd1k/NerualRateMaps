#!/usr/bin/env python3
"""
Script to check if VAE model file exists and is valid
"""

import os
import sys

def check_model_file(model_path='lidar_vae_model.pth'):
    """Check if the VAE model file exists and is valid"""
    model_path = os.path.abspath(model_path)
    
    print(f"Checking VAE model file: {model_path}")
    print(f"Absolute path: {os.path.abspath(model_path)}")
    print(f"Current working directory: {os.getcwd()}")
    print()
    
    # Check if file exists
    if not os.path.exists(model_path):
        print(f"❌ ERROR: Model file does not exist: {model_path}")
        print()
        print("To create the model file:")
        print("1. Open LidarVAE.ipynb")
        print("2. Run all cells to train the model")
        print("3. The model will be saved as 'lidar_vae_model.pth' in the current directory")
        return False
    
    # Check file size
    file_size = os.path.getsize(model_path)
    print(f"✓ File exists")
    print(f"  File size: {file_size:,} bytes ({file_size / 1024 / 1024:.2f} MB)")
    
    if file_size == 0:
        print(f"❌ ERROR: File is empty (0 bytes)")
        return False
    
    if file_size < 1000:
        print(f"⚠ WARNING: File seems too small ({file_size} bytes). Expected at least a few KB.")
        return False
    
    # Try to load it
    try:
        import torch
        print()
        print("Attempting to load model...")
        checkpoint = torch.load(model_path, map_location='cpu', weights_only=False)
        
        # Check required keys
        required_keys = ['input_dim', 'hidden_dims', 'latent_dim', 'model_state_dict', 'data_min', 'data_max']
        missing_keys = [key for key in required_keys if key not in checkpoint]
        
        if missing_keys:
            print(f"❌ ERROR: Checkpoint missing required keys: {missing_keys}")
            return False
        
        print(f"✓ Model loaded successfully")
        print(f"  Input dimension: {checkpoint['input_dim']}")
        print(f"  Latent dimension: {checkpoint['latent_dim']}")
        print(f"  Hidden dimensions: {checkpoint['hidden_dims']}")
        print(f"  Data range: [{checkpoint['data_min']:.4f}, {checkpoint['data_max']:.4f}]")
        print()
        print("✅ Model file is valid and ready to use!")
        return True
        
    except ImportError:
        print("❌ ERROR: PyTorch is not installed")
        return False
    except Exception as e:
        print(f"❌ ERROR: Failed to load model: {e}")
        print()
        print("The file may be corrupted. You may need to:")
        print("1. Delete the existing file")
        print("2. Re-train the model in LidarVAE.ipynb")
        return False

if __name__ == '__main__':
    model_path = sys.argv[1] if len(sys.argv) > 1 else 'lidar_vae_model.pth'
    success = check_model_file(model_path)
    sys.exit(0 if success else 1)

