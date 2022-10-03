import os
from pathlib import Path

from .solver import solve

modes = [Path(mode).stem for mode in os.listdir("configurations")]
