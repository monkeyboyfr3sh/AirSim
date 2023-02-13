import get_custom_paths
import setup_path
import airsim

import numpy as np
import cv2

import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from skspatial.objects import Sphere
import time

import subprocess
from custom_util.airsim_data_utils import AirSimClientManager

def start_airsim(airsim_path: str):
    print(f"Starting AirSim using path '{airsim_path}'!")
    subprocess.call(airsim_path)

if __name__ == "__main__":
    airsim_bin_path = "C:\\Users\\david\\Documents\\AirSimBinaries\\AirSimNH\\WindowsNoEditor\\AirSimNH.exe"
    start_airsim(airsim_bin_path)