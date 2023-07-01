import setup_path 
import airsim

import cv2
from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtWidgets import QMdiSubWindow,QMdiArea,QTextEdit

import nav_ui_impl as nav_ui
import sub_window_impl as sub_window_ui

def create_dialog(CustomDialogImpl: QtWidgets.QDialog, dialog_name = None, pos = (100,100)):

    customDialog = CustomDialogImpl()
    customDialog.setWindowTitle(dialog_name)
    customDialog.move(pos[0], pos[0])
    customDialog.setWindowFlags(customDialog.windowFlags() | 
                          QtCore.Qt.WindowType.WindowMinimizeButtonHint |
                          QtCore.Qt.WindowType.WindowMaximizeButtonHint |
                          QtCore.Qt.WindowType.WindowMinMaxButtonsHint )
    
    # Read the the size then set fixed window
    size = customDialog.size()
    customDialog.setFixedSize(size)
    return customDialog

if __name__ == "__main__":
    import sys

    app = QtWidgets.QApplication(sys.argv)

    dialog_list = []

    # Create the main dialog
    dialog_list.append( create_dialog(nav_ui.NavUIDialog,
                           pos=(100, 100),dialog_name="Main Window"))
    # # Create the sub dialog
    # dialog_list.append( create_dialog(sub_window_ui.SubWindowDialog,
    #                            pos=(300, 300),dialog_name="Subby Wubby"))


    for dialog in dialog_list:
        dialog.show()

    sys.exit(app.exec())