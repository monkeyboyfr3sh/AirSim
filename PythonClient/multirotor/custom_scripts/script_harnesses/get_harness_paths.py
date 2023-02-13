import os
import sys
custom_folder = os.path.abspath(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
tasks_folder = os.path.join(custom_folder,"tasks")
multirotor_folder = os.path.join(custom_folder,"..")
python_client_folder = os.path.join(multirotor_folder,"..")
custom_util_folder = os.path.join(python_client_folder,"custom_util")

sys.path.append(custom_folder)
sys.path.append(tasks_folder)
sys.path.append(multirotor_folder)
sys.path.append(custom_util_folder)

# from custom_util.airsim_data_utils import *