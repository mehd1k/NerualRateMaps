# import subprocess
# import sys

# def install(package):
#     """Install a Python package using pip."""
#     try:
#         subprocess.check_call([sys.executable, "-m", "pip", "install", package])
#     except subprocess.CalledProcessError as e:
#         print(f"Error installing package {package}: {e}")
#         sys.exit(1)

# # List of packages to install
# packages = ['scipy', 'direct']

# for package in packages:
#     install(package)


