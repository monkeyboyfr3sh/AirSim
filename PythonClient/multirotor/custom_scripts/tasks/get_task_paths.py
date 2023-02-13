import os
import sys
custom_folder = os.path.abspath(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
harnesses_folder = os.path.join(custom_folder,"harnesses")
multirotor_folder = os.path.join(custom_folder,"..")

sys.path.append(custom_folder)
sys.path.append(harnesses_folder)
sys.path.append(multirotor_folder)