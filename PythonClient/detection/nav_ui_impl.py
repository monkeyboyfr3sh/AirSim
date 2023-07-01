import setup_path 
import airsim

import nav_gui_base

import cv2
from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtWidgets import QMdiSubWindow,QMdiArea,QTextEdit

import detection_utils as dt_util
from lidar_plotter import LidarPlotter

from queue import Queue

from simulation_tasks import viewer_task

DRONE_HEIGHT = -10

class VideoStreamWorker(QtCore.QThread):
    frameCaptured = QtCore.pyqtSignal(QtGui.QImage)

    def __init__(self, png_queue, parent=None):
        super().__init__(parent)
        self.cap = None
        self.png_queue = png_queue

    def run(self):
        # Start the stream
        self.start_stream()
        # Now loop 
        while True:

            # Check if the queue has a new image
            if( not (self.png_queue.empty()) ):
                print("got")

                # Get the picture from queue
                frame = self.png_queue.get()

                h, w, ch = frame.shape
                bytes_per_line = ch * w
                image = QtGui.QImage(frame.data, w, h, bytes_per_line, QtGui.QImage.Format.Format_RGB888)
                self.frameCaptured.emit(image)

            # if not self.cap.isOpened():
            #     break

    def stop_stream(self):
        pass
        # if self.cap:
        #     self.cap.release()
        #     self.cap = None

    def start_stream(self):
        pass
        # if not self.cap:
        #     self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

class AirSimConnectWorker(QtCore.QThread):
    def __init__(self, png_queue, parent=None):
        super().__init__(parent)
        print("airsim_connect thread init")
        self.connct_command = True
        self.png_queue = png_queue

    def run(self):
        print("airsim_connect thread entering")

        # Connect to the AirSim simulator
        print("airsim_connect thread connecting")
        client = airsim.MultirotorClient()
        client.confirmConnection()

        count = 0
        while self.connct_command:
            # Decode raw image 
            png = dt_util.get_fpv_frame(client=client)
    
            # Now run detect process
            detect_objects = dt_util.get_detected_object(client)
            if(detect_objects!=None):
                for object in detect_objects:
                    dt_util.draw_object_detection(png,object)
                detect_list = detect_objects
            else:
                detect_list = []

            # Draw HUD
            dt_util.draw_HUD(png,client)

            # Push image into queue
            try:
                self.png_queue.put_nowait(png)
            except:
                pass

        # Cleanup
        print("airsim_connect thread cleaning up")
        # cv2.destroyAllWindows() 
        client.enableApiControl(False)
        client.armDisarm(False)

class NavUIDialog(QtWidgets.QDialog, nav_gui_base.Ui_Dialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)

        self.png_queue = None

        self.videoStreamWorker = None
        self.airsimConnectWorker = None

        # Setup navigate command buttons
        self.navigate_button.clicked.connect(self.start_stream)
        self.stop_navigate_button.clicked.connect(self.stop_stream)

        self.airsimConnect_button.clicked.connect(self.airsim_connect)
        self.airsimDisconnect_button.clicked.connect(self.airsim_disconnect)

    def start_stream(self):

        # Detect if we have worker
        if (not self.videoStreamWorker) and (not self.png_queue):
            # Create the worker
            self.videoStreamWorker = VideoStreamWorker(self.png_queue)
            self.videoStreamWorker.frameCaptured.connect(self.update_stream)

            # Now start the worker
            self.videoStreamWorker.start()

    def stop_stream(self):

        # Detect if we have worker
        if self.videoStreamWorker:
            # Stop the worker
            self.videoStreamWorker.stop_stream()
            self.videoStreamWorker.terminate()

            self.blank_graphics_view()

            # Clear the var
            self.videoStreamWorker = None


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

    def airsim_connect(self):
        # Detect if we have worker
        if not self.airsimConnectWorker:
            self.png_queue = Queue(maxsize=10) 
            self.airsimConnectWorker = AirSimConnectWorker(self.png_queue)
            self.airsimConnectWorker.connct_command = True
            self.airsimConnectWorker.start()

    def airsim_disconnect(self):
        # Detect if we have worker
        if self.airsimConnectWorker:
            self.airsimConnectWorker.connct_command = False
            self.airsimConnectWorker.wait()

            # Clear the var
            self.airsimConnectWorker = None
