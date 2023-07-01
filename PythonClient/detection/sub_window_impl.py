import setup_path 
import airsim

import sub_window_base 

from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtWidgets import QMdiSubWindow,QMdiArea,QTextEdit

class SubWindowDialog(QtWidgets.QDialog, sub_window_base.Ui_Dialog):
    count = 0

    def __init__(self):
        super().__init__()
        self.setupUi(self)

        # Setup add window button
        self.addwindow_pushButton.clicked.connect(self.add_window)

        # self.navigate_button.pressed.connect(self.start_stream)
        # self.stop_navigate_button.pressed.connect(self.stop_stream)

    def add_window(self):
        SubWindowDialog.count = SubWindowDialog.count + 1
        # Create sub window
        sub = QMdiSubWindow()
        # Do stuff inside sub window
        sub.setWidget(QTextEdit())
        # Set the titlebar of sub window
        sub.setWindowTitle(f"Subby Window {SubWindowDialog.count}")
        # Add subwindow in mdi
        self.mdiArea.addSubWindow(sub)

        # Show the window
        sub.show()

        self.mdiArea.cascadeSubWindows()

#     def start_stream(self):
#         self.timer.start(10)  # Set the desired interval in milliseconds (30 fps = 33 ms)
#         self.capture = cv2.VideoCapture(0, cv2.CAP_DSHOW)
#         self.timer.timeout.connect(self.update_frame)

#     def stop_stream(self):
#         if self.capture is not None:
#             self.capture.release()
#         self.timer.stop()
#         self.graphicsView.setScene(None)
#         self.timer.timeout.disconnect()

#     def update_frame(self):
#         ret, frame = self.capture.read()
#         if ret:
#             # Convert the OpenCV frame to a QImage
#             height, width, channel = frame.shape
#             bytes_per_line = 3 * width
#             image = QtGui.QImage(frame.data, width, height, bytes_per_line)

#             # Swap red and blue channels
#             image = image.rgbSwapped()

#             pixmap = QtGui.QPixmap.fromImage(image)
#             # Display the frame in the graphics view
#             scene = QtWidgets.QGraphicsScene()
#             scene.addPixmap(pixmap)
#             self.graphicsView.setScene(scene)

#     def navigate_pressed(self):
#         print("Navigate button pressed")

#     def stop_navigate_pressed(self):
#         print("Stop Navigate button pressed")
# self.capture = cv2.VideoCapture(0, cv2.CAP_DSHOW)




