import os
import sys
root_folder = os.path.abspath(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
parent_folder = os.path.join(root_folder,"../")
sys.path.append(root_folder)
sys.path.append(parent_folder)