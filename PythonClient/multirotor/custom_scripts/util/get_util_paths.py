import os
import sys
custom_folder = os.path.abspath(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
multirotor_folder = os.path.join(custom_folder,"..")
python_client_folder = os.path.join(multirotor_folder,"..")

sys.path.append(custom_folder)
sys.path.append(multirotor_folder)