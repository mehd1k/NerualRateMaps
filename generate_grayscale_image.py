import numpy as np
from PIL import Image

def generate_nonlinear_grayscale(width=600, height=900, n_shades=5, top_gray=220, gamma=2.0, output_file="grayscale.png"):
    """
    Generate a non-linear stepped grayscale image.
    
    Parameters:
        width (int): Image width in pixels.
        height (int): Image height in pixels.
        n_shades (int): Number of horizontal gray bands.
        top_gray (int): Maximum brightness (0–255) at the top.
        gamma (float): Nonlinearity factor (>1 darkens faster near bottom, <1 lightens).
        output_file (str): Output filename.
    """
    block_height = height // n_shades
    img = np.zeros((height, width), dtype=np.uint8)

    # Generate nonlinear brightness curve
    for i in range(n_shades):
        # Normalized position (0=top, 1=bottom)
        t = i / (n_shades - 1)
        # Apply gamma curve for nonlinear transition
        gray_value = int(top_gray * ((1 - t) ** gamma))
        img[i * block_height:(i + 1) * block_height, :] = gray_value

    Image.fromarray(img).save(output_file)
    print(f"Saved {output_file} with top_gray={top_gray}, gamma={gamma}, and {n_shades} shades.")

# Example usage
generate_nonlinear_grayscale(width=512, height=900, n_shades=50, top_gray=100, gamma=1.7, output_file="grayscale.png")
