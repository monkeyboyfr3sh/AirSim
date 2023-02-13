import os
import sys
harness_folder = os.path.abspath(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
custom_folder = os.path.join(harness_folder,"../")
airsim_folder = os.path.join(custom_folder,"../")
# airsim_folder = os.path.join(airsim_folder,"../")
sys.path.append(harness_folder)
sys.path.append(custom_folder)
sys.path.append(airsim_folder)