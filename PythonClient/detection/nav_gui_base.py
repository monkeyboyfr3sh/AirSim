# Form implementation generated from reading ui file 'nav_gui.ui'
#
# Created by: PyQt6 UI code generator 6.4.2
#
# WARNING: Any manual changes made to this file will be lost when pyuic6 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt6 import QtCore, QtGui, QtWidgets


class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(516, 712)
        self.navigate_button = QtWidgets.QPushButton(parent=Dialog)
        self.navigate_button.setGeometry(QtCore.QRect(220, 360, 81, 31))
        self.navigate_button.setObjectName("navigate_button")
        self.stop_navigate_button = QtWidgets.QPushButton(parent=Dialog)
        self.stop_navigate_button.setGeometry(QtCore.QRect(210, 310, 101, 31))
        self.stop_navigate_button.setObjectName("stop_navigate_button")
        self.graphicsView = QtWidgets.QGraphicsView(parent=Dialog)
        self.graphicsView.setGeometry(QtCore.QRect(70, 40, 381, 261))
        self.graphicsView.setObjectName("graphicsView")

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.navigate_button.setText(_translate("Dialog", "Navigate"))
        self.stop_navigate_button.setText(_translate("Dialog", "Stop Navigate"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Dialog = QtWidgets.QDialog()
    ui = Ui_Dialog()
    ui.setupUi(Dialog)
    Dialog.show()
    sys.exit(app.exec())
