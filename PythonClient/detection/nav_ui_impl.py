import setup_path 
import airsim

import nav_gui_base
import sub_window_base 

import cv2
from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtWidgets import QMdiSubWindow,QMdiArea,QTextEdit

class VideoStreamWorker(QtCore.QThread):
    frameCaptured = QtCore.pyqtSignal(QtGui.QImage)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.cap = None

    def run(self):
        # Start the stream
        self.start_stream()
        # Now loop 
        while True:
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = frame.shape
                bytes_per_line = ch * w
                image = QtGui.QImage(frame.data, w, h, bytes_per_line, QtGui.QImage.Format.Format_RGB888)
                self.frameCaptured.emit(image)

            if not self.cap.isOpened():
                break

    def stop_stream(self):
        if self.cap:
            self.cap.release()
            self.cap = None

    def start_stream(self):
        if not self.cap:
            self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)


class VideoStreamDialog(QtWidgets.QDialog, nav_gui_base.Ui_Dialog):
    count = 0
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)

        self.worker = VideoStreamWorker()
        self.worker.frameCaptured.connect(self.update_stream)

        # Setup navigate command buttons
        self.navigate_button.clicked.connect(self.start_stream)
        self.stop_navigate_button.clicked.connect(self.stop_stream)

        # Setup add window button
        self.addwindow_pushButton.clicked.connect(self.add_window)
        
        # Setup for multiple displays
        # self.mdiArea.

    def start_stream(self):
        self.worker.start()

    def stop_stream(self):
        self.worker.stop_stream()
        self.worker.terminate()

    def add_window(self):
        VideoStreamDialog.count = VideoStreamDialog.count + 1
        # Create sub window
        sub = QMdiSubWindow()
        # Do stuff inside sub window
        sub.setWidget(QTextEdit())
        # Set the titlebar of sub window
        sub.setWindowTitle(f"Subby Window {VideoStreamDialog.count}")
        # Add subwindow in mdi
        self.mdiArea.addSubWindow(sub)

        # Show the window
        sub.show()

        self.mdiArea.cascadeSubWindows()

    @QtCore.pyqtSlot(QtGui.QImage)
    def update_stream(self, image):
        pixmap = QtGui.QPixmap.fromImage(image)
        pixmap = pixmap.scaled(self.graphicsView.size(), QtCore.Qt.AspectRatioMode.KeepAspectRatio)

        self.graphicsView.setScene(QtWidgets.QGraphicsScene())
        self.graphicsView.scene().addPixmap(pixmap)
        self.graphicsView.fitInView(self.graphicsView.scene().sceneRect(), QtCore.Qt.AspectRatioMode.KeepAspectRatio)

    def blank_graphics_view(self):
        self.graphicsView.setScene(QtWidgets.QGraphicsScene())
        self.graphicsView.scene().clear()

    def closeEvent(self, event):
        self.stop_stream()
        self.worker.wait()
        self.blank_graphics_view()

class SubWindowDialog(QtWidgets.QDialog, nav_gui_base.Ui_Dialog):

    def __init__(self):
        super().__init__()
        self.setupUi(self)

        # self.navigate_button.pressed.connect(self.start_stream)
        # self.stop_navigate_button.pressed.connect(self.stop_stream)

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

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)

    # dialog = VideoStreamDialog()
    # sub_dialog = SubWindowDialog()

    # dialog.show()
    # sub_dialog.show()




    # dialog = VideoStreamDialog()
    # sub_dialog = SubWindowDialog()

    # dialog.show()
    # sub_dialog.show()

    dialog = VideoStreamDialog()
    # dialog = QtWidgets.QDialog()
    dialog.setWindowTitle("Main Dialog")
    dialog.move(100, 100)
    # dialog.setGeometry(100, 100, 400, 300)

    sub_dialog = SubWindowDialog()
    # sub_dialog = QtWidgets.QDialog(dialog)
    sub_dialog.setWindowTitle("Sub Dialog")
    sub_dialog.move(300, 300)
    # sub_dialog.setGeometry(dialog.geometry().right() + 10, dialog.geometry().top(), 300, 200)

    dialog.show()
    sub_dialog.show()


    
    sys.exit(app.exec())

