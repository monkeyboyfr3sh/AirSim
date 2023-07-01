import setup_path 
import airsim

import nav_gui_base

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
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)

        self.worker = VideoStreamWorker()
        self.worker.frameCaptured.connect(self.update_stream)

        # Setup navigate command buttons
        self.navigate_button.clicked.connect(self.start_stream)
        self.stop_navigate_button.clicked.connect(self.stop_stream)

    def start_stream(self):
        self.worker.start()

    def stop_stream(self):
        self.worker.stop_stream()
        self.worker.terminate()

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

