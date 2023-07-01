import setup_path 
import airsim

import nav_gui_base

import numpy as np
import cv2
from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtWidgets import QMdiSubWindow,QMdiArea,QTextEdit

import detection_utils as dt_util
from lidar_plotter import LidarPlotter
import time

from queue import Queue

from simulation_tasks import viewer_task

DRONE_HEIGHT = -10

class AirSimConnectWorker(QtCore.QThread):
    frameCaptured = QtCore.pyqtSignal(QtGui.QImage)

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
                # Assuming your 4-channel image is stored as a cv2 Mat object named 'image'
                png= cv2.cvtColor(png, cv2.COLOR_BGRA2BGR)
                h, w, ch = png.shape
                bytes_per_line = ch * w
                image = QtGui.QImage(png.tobytes(), w, h, bytes_per_line, QtGui.QImage.Format.Format_BGR888)
                self.frameCaptured.emit(image)
            except:
                print("err")
                pass

        # Cleanup
        print("airsim_connect thread cleaning up")
        client.enableApiControl(False)
        client.armDisarm(False)

class NavUIDialog(QtWidgets.QDialog, nav_gui_base.Ui_Dialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)

        self.png_queue = None

        self.videoStreamWorker = None
        self.airsimConnectWorker = None

        # # Setup navigate command buttons
        # self.navigate_button.clicked.connect(self.start_stream)
        # self.stop_navigate_button.clicked.connect(self.stop_stream)

        self.airsimConnect_button.clicked.connect(self.airsim_connect)
        self.airsimDisconnect_button.clicked.connect(self.airsim_disconnect)

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
        pass

    def airsim_connect(self):
        # Detect if we have worker
        if not self.airsimConnectWorker:
            self.png_queue = Queue(maxsize=10) 
            self.airsimConnectWorker = AirSimConnectWorker(self.png_queue)
            self.airsimConnectWorker.frameCaptured.connect(self.update_stream)
            self.airsimConnectWorker.connct_command = True
            self.airsimConnectWorker.start()

    def airsim_disconnect(self):
        # Detect if we have worker
        if self.airsimConnectWorker:
            self.airsimConnectWorker.connct_command = False
            self.blank_graphics_view()

            # Clear the var
            self.airsimConnectWorker = None
