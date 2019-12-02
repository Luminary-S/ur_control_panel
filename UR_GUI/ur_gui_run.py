#!/usr/bin/python
# -*- coding: utf-8 -*-

from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QMainWindow, QMessageBox,QStatusBar
from URwindow import MainWindow
import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

if __name__ == '__main__':

    app = QApplication(sys.argv)
    ui = MainWindow()
    ui.show()

    sys.exit(app.exec_())
