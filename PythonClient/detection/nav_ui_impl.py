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

DRONE_HEIGHT = -10

class AirSimConnectWorker(QtCore.QThread):
    frameCaptured = QtCore.pyqtSignal(QtGui.QImage)
    finished = QtCore.pyqtSignal(QtGui.QImage)

    def __init__(self, parent=None):
        super().__init__(parent)
        print("airsim_connect thread init")
        self.connect_command = True

    def run(self):
        print("airsim_connect thread entering")
        # Connect to the AirSim simulator
        print("airsim_connect thread connecting")
        client = airsim.MultirotorClient()
        client.confirmConnection()

        # Turn on detection
        detect_filter = "Monument*"
        dt_util.detection_filter_on_off(client, True, detect_filter)
        detect_filter = "Car*"
        dt_util.detection_filter_on_off(client, True, detect_filter)
        detect_filter = "Deer*"
        dt_util.detection_filter_on_off(client, True, detect_filter)
        # detect_filter = "Raccoon*"
        # dt_util.detection_filter_on_off(client, True, detect_filter)
        # detect_filter = "InstancedFoliageAct*"
        # detection_filter_on_off(client, True, detect_filter)

        while self.connect_command:
            # Decode raw image 
            png = dt_util.get_fpv_frame(client=client)
    
            # Now run detect process
            detect_objects = dt_util.get_detected_object(client)
            if detect_objects is not None:
                for obj in detect_objects:
                    dt_util.draw_object_detection(png, obj)
                detect_list = detect_objects
            else:
                detect_list = []

            # Draw HUD
            dt_util.draw_HUD(png, client)

            # Push image into queue
            try:
                png = cv2.cvtColor(png, cv2.COLOR_BGRA2BGR)
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
        print("airsim_connect thread exiting")
        self.finished.emit(QtGui.QImage())  # Emit the 'finished' signal when the worker is done

class NavUIDialog(QtWidgets.QDialog, nav_gui_base.Ui_Dialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)

        self.airsimConnectWorker = None

        self.airsimConnect_button.clicked.connect(self.airsim_connect)
        self.airsimDisconnect_button.clicked.connect(self.airsim_disconnect)

        self.resetButton.clicked.connect(self.reset)
        self.navigate_button.clicked.connect(self.navigte)
        self.stop_navigate_button.clicked.connect(self.stop_navigate)

        self.target_name = "Monument_01_176"
        # self.target_name = "Car_35"
        self.default_z = DRONE_HEIGHT
        self.task_thread = threading.Thread()

    def blank_graphics_view(self):
        self.graphicsView.setScene(QtWidgets.QGraphicsScene())
        self.graphicsView.scene().clear()

    def closeEvent(self, event):
        pass

    def airsim_connect(self):
        # Detect if we have worker
        if not self.airsimConnectWorker:
            self.airsimConnectWorker = AirSimConnectWorker()
            self.airsimConnectWorker.frameCaptured.connect(self.update_stream)
            self.airsimConnectWorker.finished.connect(self.worker_finished)  # Connect the 'finished' signal to a slot
            self.airsimConnectWorker.connect_command = True
            self.airsimConnectWorker.start()

    def airsim_disconnect(self):
        # Detect if we have worker
        if self.airsimConnectWorker:
            self.airsimConnectWorker.connect_command = False

    def navigte(self):
        if (self.task_thread.is_alive()):
            print("Task is already running!")
            return

        self.task_thread, task_client = sim_tasks.create_task_client(target=sim_tasks.navigate_to_monument,
                                                                     args=(self.default_z,),start_task=True)

    def stop_navigate(self):
        self.task_thread, task_client = sim_tasks.create_task_client(target=sim_tasks.client_disarm,start_task=True)
        self.task_thread = threading.Thread()

    def reset(self):
        self.task_thread, task_client = sim_tasks.create_task_client(target=sim_tasks.sim_reset,start_task=True)
        self.task_thread = threading.Thread()

    @QtCore.pyqtSlot(QtGui.QImage)
    def update_stream(self, image):
        pixmap = QtGui.QPixmap.fromImage(image)
        pixmap = pixmap.scaled(self.graphicsView.size(), QtCore.Qt.AspectRatioMode.KeepAspectRatio)

        self.graphicsView.setScene(QtWidgets.QGraphicsScene())
        self.graphicsView.scene().addPixmap(pixmap)
        self.graphicsView.fitInView(self.graphicsView.scene().sceneRect(), QtCore.Qt.AspectRatioMode.KeepAspectRatio)

    @QtCore.pyqtSlot(QtGui.QImage)
    def worker_finished(self, image):
        print(f"AirSimConnectWorker has finished")  # Do something to handle the worker finishing
        self.blank_graphics_view()
        self.airsimConnectWorker = None  # Reset the worker variable