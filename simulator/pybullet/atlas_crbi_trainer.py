import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)

import pybullet as pb
import numpy as np

#main loop
if __name__ == "__main__":

    generate_data()
