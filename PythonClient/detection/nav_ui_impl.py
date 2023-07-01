import setup_path 
import airsim
import nav_gui_base
import numpy as np
import cv2
from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtWidgets import QMdiSubWindow, QMdiArea, QTextEdit
import detection_utils as dt_util
from lidar_plotter import LidarPlotter
import threading
import simulation_tasks as sim_tasks 
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

DRONE_HEIGHT = -10  # Default height of the drone

class AirSimConnectWorker(QtCore.QThread):
    frameCaptured = QtCore.pyqtSignal(QtGui.QImage)  # Signal emitted when a frame is captured
    finished = QtCore.pyqtSignal(QtGui.QImage)  # Signal emitted when the worker is finished

    def __init__(self, parent=None):
        super().__init__(parent)
        logger.info("airsim_connect thread init")
        self.connect_command = True  # Flag to control the connection

    def run(self):
        logger.info("airsim_connect thread entering")
        # Connect to the AirSim simulator
        logger.info("airsim_connect thread connecting")
        client = airsim.MultirotorClient()
        client.confirmConnection()

        # Turn on object detection for specific filters
        detect_filter = "Monument*"
        dt_util.detection_filter_on_off(client, True, detect_filter)
        detect_filter = "Car*"
        dt_util.detection_filter_on_off(client, True, detect_filter)
        detect_filter = "Deer*"
        dt_util.detection_filter_on_off(client, True, detect_filter)

        while self.connect_command:
            # Capture frame from the simulator
            png = dt_util.get_fpv_frame(client=client)
    
            # Run object detection on the captured frame
            detect_objects = dt_util.get_detected_object(client)
            if detect_objects is not None:
                for obj in detect_objects:
                    dt_util.draw_object_detection(png, obj)
                detect_list = detect_objects
            else:
                detect_list = []

            # Draw HUD (Heads-Up Display) on the frame
            dt_util.draw_HUD(png, client)

            # Convert and emit the captured frame as QImage
            try:
                png = cv2.cvtColor(png, cv2.COLOR_BGRA2BGR)
                h, w, ch = png.shape
                bytes_per_line = ch * w
                image = QtGui.QImage(png.tobytes(), w, h, bytes_per_line, QtGui.QImage.Format.Format_BGR888)
                self.frameCaptured.emit(image)
            except:
                logger.error("An error occurred")
                pass

        # Cleanup after the connection is stopped
        logger.info("airsim_connect thread cleaning up")
        client.enableApiControl(False)
        client.armDisarm(False)
        logger.info("airsim_connect thread exiting")
        self.finished.emit(QtGui.QImage())  # Emit the 'finished' signal when the worker is done

class NavUIDialog(QtWidgets.QDialog, nav_gui_base.Ui_Dialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)

        self.airsimConnectWorker = None  # Worker for connecting to AirSim

        # Connect button signals to respective slots
        self.airsimConnect_button.clicked.connect(self.airsim_connect)
        self.airsimDisconnect_button.clicked.connect(self.airsim_disconnect)
        self.resetButton.clicked.connect(self.reset)
        self.navigate_button.clicked.connect(self.navigte)
        self.stop_navigate_button.clicked.connect(self.stop_navigate)

        self.target_name = "Monument_01_176"  # Default target name
        self.default_z = DRONE_HEIGHT  # Default z-coordinate
        self.task_thread = threading.Thread()  # Thread for running tasks

    def blank_graphics_view(self):
        """
        Clears the graphics view and sets a blank screen or image background.
        """
        # Blank the screen
        self.graphicsView.setScene(QtWidgets.QGraphicsScene())
        self.graphicsView.scene().clear()

        # Try setting an image as background
        try:
            # Load and display an image
            png = cv2.imread("./disconnect_meme.jpg")
            h, w, ch = png.shape
            bytes_per_line = ch * w
            image = QtGui.QImage(png.tobytes(), w, h, bytes_per_line, QtGui.QImage.Format.Format_BGR888)
            self.update_stream(image)
        except:
            logger.error("Error setting image")

    def closeEvent(self, event):
        """
        Overrides the closeEvent method to handle the closing of the dialog.
        """
        pass

    def airsim_connect(self):
        """
        Connects to AirSim by creating and starting the AirSimConnectWorker.
        """
        # Detect if we have a worker
        if not self.airsimConnectWorker:
            self.airsimConnectWorker = AirSimConnectWorker()
            self.airsimConnectWorker.frameCaptured.connect(self.update_stream)
            self.airsimConnectWorker.finished.connect(self.worker_finished)  # Connect the 'finished' signal to a slot
            self.airsimConnectWorker.connect_command = True
            self.airsimConnectWorker.start()

    def airsim_disconnect(self):
        """
        Disconnects from AirSim by setting the connect_command of the AirSimConnectWorker to False.
        """
        # Detect if we have a worker
        if self.airsimConnectWorker:
            self.airsimConnectWorker.connect_command = False

    def navigte(self):
        """
        Starts the navigation task by creating a task client and running the 'navigate_to_monument' task in a thread.
        """
        if self.task_thread.is_alive():
            logger.info("Task is already running!")
            return

        self.task_thread, task_client = sim_tasks.create_task_client(target=sim_tasks.navigate_to_monument,
                                                                     args=(self.default_z,), start_task=True)

    def stop_navigate(self):
        """
        Stops the navigation task by creating a task client and running the 'client_disarm' task in a thread.
        """
        self.task_thread, task_client = sim_tasks.create_task_client(target=sim_tasks.client_disarm,
                                                                     start_task=True)
        self.task_thread = threading.Thread()

    def reset(self):
        """
        Resets the simulation by creating a task client and running the 'sim_reset' task in a thread.
        """
        self.task_thread, task_client = sim_tasks.create_task_client(target=sim_tasks.sim_reset,
                                                                     start_task=True)
        self.task_thread = threading.Thread()

    @QtCore.pyqtSlot(QtGui.QImage)
    def update_stream(self, image):
        """
        Updates the graphics view with the provided image.
        """
        pixmap = QtGui.QPixmap.fromImage(image)
        pixmap = pixmap.scaled(self.graphicsView.size(), QtCore.Qt.AspectRatioMode.KeepAspectRatio)

        self.graphicsView.setScene(QtWidgets.QGraphicsScene())
        self.graphicsView.scene().addPixmap(pixmap)
        self.graphicsView.fitInView(self.graphicsView.scene().sceneRect(), QtCore.Qt.AspectRatioMode.KeepAspectRatio)

    @QtCore.pyqtSlot(QtGui.QImage)
    def worker_finished(self, image):
        """
        Handles the finishing of the AirSimConnectWorker by resetting the graphics view and worker variable.
        """
        logger.info("AirSimConnectWorker has finished")  # Do something to handle the worker finishing
        self.blank_graphics_view()
        self.airsimConnectWorker = None  # Reset the worker variable
