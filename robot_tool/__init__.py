import numpy as np
import sys,os
home_dir = os.path.expanduser("~")
conda_env_path = os.environ.get("CONDA_PREFIX",os.path.join(home_dir,"miniconda3","envs","humble"))
if conda_env_path:
    conda_site_packages = os.path.join(conda_env_path, "lib", f"python{sys.version_info.major}.{sys.version_info.minor}", "site-packages")
    print(f"conda path: {conda_site_packages}")
    if conda_site_packages not in sys.path:
        sys.path.append(conda_site_packages)
from . import *