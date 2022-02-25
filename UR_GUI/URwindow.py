#!/usr/bin/venv python
# -*- coding: utf-8 -*-

"""
Module implementing MainWindow.
"""
#project path
import re
from PROJECTPATH import *
import time
# pyqt5 class
from PyQt5.QtCore import pyqtSlot, Qt
from PyQt5.QtWidgets import QMainWindow, QMessageBox,QStatusBar
from PyQt5 import QtGui
from PyQt5.QtGui import QIntValidator,QDoubleValidator,QRegExpValidator,QPixmap,QImage
from Ui_URgui import Ui_MainWindow
from ur_gui_ros_node import guiNode

# define class
# from gui_ros_node import *
import cv2


class MainWindow(QMainWindow, Ui_MainWindow):
    """
    Class documentation goes here.
    """

    def __init__(self, parent=None):
        """
        Constructor

        @param parent reference to the parent widget
        @type QWidget
        """
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.setWindowTitle('UR Control Panel')
        # self.setFixedSize(1440,950)
        self.init_show()

        # gui node
        self.ur_node = guiNode()
        self.ur_node.init()
        # self.guiNode = guiNode()
        # self.guiNode.init()

        # cam frame
        # self.guiNode.baseCam.baseCamThread.camSignal.connect( self.set_baseCam_pic )
        # self.guiNode.sensorAr.sensorArThread.sensorArSignal.connect( self.update_sensor_data )

        # self.guiNode.spin()

    def init_show(self):      


                                # QMessageBox.Yes | QMessageBox.No)
        # pass
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(Project_Path +'img/ur.ico'), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.setWindowIcon(icon)
        # self.setWindowIcon(QtGui.QIcon(Project_Path + "img/icon.ico"))
        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)

    def set_label_pic(self, label, img):
        try:
            h, w = img.shape[:2]
            img = QImage(img,
                         w, h, QImage.Format_RGB888)
            img = QPixmap.fromImage(img)
            label.setPixmap(img)
            label.setScaledContents(True)
        except:
            print("no img for qtgui")

    @pyqtSlot()
    def showEvent(self, evt):
        msg = QMessageBox.about(self,
                    "INFO",
                    "Pls first set ur type and ur install type!")
        self.set_ur_eepos_btns_bool(True)

    
    def show_joints_angles(self, j_list):
        self.base_joint_lEdit.setText(str(j_list[0]))
        self.shoulder_joint_lEdit.setText(str(j_list[1]))
        self.elbow_joint_lEdit.setText(str(j_list[2]))
        self.wrist1_joint_lEdit.setText(str(j_list[3]))
        self.wrist2_joint_lEdit.setText(str(j_list[4]))
        self.wrist3_joint_lEdit.setText(str(j_list[5]))

    def show_ee_cartesian_pos(self, pos_list):
        self.x_ee_lEdit.setText(str(pos_list[0]))
        self.y_ee_lEdit.setText(str(pos_list[1]))
        self.z_ee_lEdit.setText(str(pos_list[2]))
        self.th_x_ee_lEdit.setText(str(pos_list[3]))
        self.th_y_ee_lEdit.setText(str(pos_list[4]))
        self.th_z_ee_lEdit.setText(str(pos_list[5]))
        p_list = self.ur_node.getpi(pos_list)
        self.th_x_rad_ee_lEdit.setText(str(p_list[3]))
        self.th_y_rad_ee_lEdit.setText(str(p_list[4]))
        self.th_z_rad_ee_lEdit.setText(str(p_list[5]))

    """
    real time info box
    """

    # region real time info
    def set_status_txt(self, status):
        self.info_textEdit.append(status)

    @pyqtSlot()
    def on_clear_ur_info_txt_btn_clicked(self):
        self.info_textEdit.clear()
    """
        ur move part
    """
    # region ur move
    @pyqtSlot()
    def on_get_joint_btn_clicked(self):
        print("================")
        q = self.ur_node.ur.get_q()
        strr = '[' + ','.join(map(str,[round(x,3) for x in q ])) + ']'
        q = self.ur_node.getDegree(q)
        stri = '[' + ','.join(map(str,q)) + ']'
        self.show_note_joint_lEdit.setText(stri)
        self.show_note_rad_joint_lEdit.setText(strr)

        pos = self.ur_node.ur.get_ee_pos_cartesian()

        note = self.get_note_lEdit.text()
        self.set_status_txt('UR now in ' + note + ":"  )
        self.set_status_txt(stri)
        self.show_joints_angles(q)
        self.show_ee_cartesian_pos(pos)

    @pyqtSlot()
    def on_clear_note_btn_clicked(self):
        self.get_note_lEdit.clear()

    ### input move
    @pyqtSlot()
    def on_move_joint_angle_btn_clicked(self):
        # q = self.ur_node.ur.get_q()
        str_q_d = self.input_joint_angle_lEdit.text()
        q_d = re.findall(r'[[](.*?)[]]',str_q_d)[0].split(',')
        q_d = map(float,q_d)
        self.ur_node.ur_move_to_point_degree(q_d)
        # stri = '[' + ','.join(map(str,q)) + ']'
        # self.show_note_joint_lEdit.setText(stri)
        # note = str_q_d
        self.set_status_txt('UR now moves to: ')
        self.set_status_txt(str_q_d)

    @pyqtSlot()
    def on_move_joint_radian_btn_clicked(self):
        # q = self.ur_node.ur.get_q()
        str_q_d = self.input_joint_radian_lEdit.text()
        q_d = re.findall(r'[[](.*?)[]]',str_q_d)[0].split(',')
        q_d = map(float, q_d)
        self.ur_node.ur_move_to_point_radian(q_d)
        self.set_status_txt('UR now moves to: ')
        self.set_status_txt(str_q_d)

    @pyqtSlot()
    def on_clear_angle_btn_clicked(self):
        self.input_joint_angle_lEdit.clear()

    @pyqtSlot()
    def on_clear_radian_btn_clicked(self):
        self.input_joint_radian_lEdit.clear()
    # def on_ur_route_ok_btn_clicked(self):
    #     len = self.ur_route_len_edit.text()
    #     width = self.ur_route_width_edit.text()
    #     len = int(len)
    #     width = int(width)
    #     self.guiNode.ur.set_route_move_rosparam(len,width)
    #     self.guiNode.ur.set_route_move_status(1)
    #     status = " open zigzag route move, set len: " + str(len) + ", width: " + str(width)
    #     self.set_ur_info_textEdit(status)

    @pyqtSlot()
    def on_ur3_ch_btn_clicked(self):
        # self.guiNode.ur.set_route_move_status(0)
        status = " set ur type as UR3"
        self.ur_node.set_ur_type('ur3')
        self.set_status_txt(status)

    @pyqtSlot()
    def on_ur5_ch_btn_clicked(self):
        # self.guiNode.ur.set_route_move_status(0)
        status = " set ur type as UR5"
        self.ur_node.set_ur_type('ur5')
        self.set_status_txt(status)

    @pyqtSlot()
    def on_normal_install_btn_clicked(self):
        # self.guiNode.ur.set_route_move_status(0)
        status = " set ur install type as: normal install"
        self.ur_node.set_ur_install_type('normal')
        self.set_status_txt(status)

    @pyqtSlot()
    def on_down_install_btn_clicked(self):
        # self.guiNode.ur.set_route_move_status(0)
        status = " set ur install type as: down install"
        self.ur_node.set_ur_install_type('down')
        self.set_status_txt(status)

    @pyqtSlot()
    def on_ur_stop_btn_clicked(self):
        # self.guiNode.ur.set_impedance_status(0)
        status = " stop UR !!!"
        self.set_status_txt(status)


    @pyqtSlot()
    def on_get_init_ur_pos_btn_clicked(self):
        """
        Slot documentation goes here.
        """
        # q_list = self.ur.set_init_q()
        q = self.ur_node.ur.get_q()
        self.ur_node.ur.set_init_q()
        q = self.ur_node.getDegree(q)
        stri =  '[' + ','.join(map(str,q))  +']'
        self.show_init_pos_tedit.setText(stri)
        self.set_status_txt("set INIT ur pos: " + stri)
        msg = QMessageBox.information(self,
                                "Confirm",
                                "Pls confirm your pos choosen!",
                                QMessageBox.Yes | QMessageBox.No)
        if (msg == QMessageBox.Yes):
            self.ur_node.ur.ur_init_ready = 1
            self.set_status_txt(" init pos is ok!.")
            # if self.ur.ur_final_ready == 1:
            #     self.ur.ur_ready = 1
        else:
            self.ur_node.ur.ur_init_ready = 0
            self.ur_node.ur.ur_ready = 0
        self.ur_node.ur.ur_ready = self.ur_node.ur.ur_init_ready * self.ur_node.ur.ur_final_ready
        # self.set_ur_related_btns_bool(self.ur_node.ur.ur_ready)


    @pyqtSlot()
    def on_get_final_ur_pos_btn_clicked(self):
        """
        Slot documentation goes here.
        """
        # q_list = self.ur_node.ur.set_final_q()
        q = self.ur_node.ur.get_q()
        self.ur_node.ur.set_final_q()
        q = self.ur_node.getDegree(q)
        stri =  '[' + ','.join(map(str, q)) + ']'
        self.show_final_pos_tedit.setText(stri)
        self.set_status_txt("set FINAL ur pos: " + stri)
        # self.set_status_txt(stri)
        msg = QMessageBox.information(self,
                                "Confirm",
                                "Pls confirm your pos choosen!",
                                QMessageBox.Yes | QMessageBox.No)
        if (msg == QMessageBox.Yes):
            self.ur_node.ur.ur_final_ready = 1
            self.set_status_txt(" final pos is ok!.")
            # if self.ur.ur_init_ready == 1:
            #     self.ur.ur_ready = 1
        else:
            self.ur_node.ur.ur_init_ready = 0
            self.ur_node.ur.ur_ready = 0
        self.ur_node.ur.ur_ready = self.ur_node.ur.ur_init_ready * self.ur_node.ur.ur_final_ready
        # self.set_ur_related_btns_bool(self.ur_node.ur.ur_ready)

    @pyqtSlot()
    def on_init2final_btn_clicked(self):
        if self.ur_node.ur.ur_final_ready == 0 :
            QMessageBox.information(self,
                                    "Confirm",
                                    "Pls choose final position!",
                                    QMessageBox.Yes | QMessageBox.No)
            return False
        else:
            self.ur_node.ur_move_to_point_radian(self.ur_node.ur.final_q)
            return True

    @pyqtSlot()
    def on_final2init_btn_clicked(self):
        if self.ur_node.ur.ur_init_ready == 0 :
            QMessageBox.information(self,
                                    "Confirm",
                                    "Pls choose init position !",
                                    QMessageBox.Yes | QMessageBox.No)
            return False
        else:
            self.ur_node.ur_move_to_point_radian(self.ur_node.ur.init_q)
            return True


    """UR EE Pos step movement"""
    def set_ur_eepos_btns_bool(self, bool):
        self.ur_forward_btn.setEnabled(bool)
        self.ur_back_btn.setEnabled(bool)
        self.ur_up_btn.setEnabled(bool)
        self.ur_down_btn.setEnabled(bool)
        self.ur_left_btn.setEnabled(bool)
        self.ur_right_btn.setEnabled(bool)

    @pyqtSlot()
    def on_ur_forward_btn_clicked(self):
        # q = self.ur.get_q()
        # pub = self.ur.pub
        self.ur_node.ur_step_move_forward()
    @pyqtSlot()
    def on_ur_back_btn_clicked(self):
        # q = self.ur.get_q()
        # pub = self.ur.pub
        self.ur_node.ur_step_move_backward()
    @pyqtSlot()
    def on_ur_up_btn_clicked(self):
        # q = self.ur.get_q()
        # pub = self.ur.pub
        self.ur_node.ur_step_move_up()
    @pyqtSlot()
    def on_ur_down_btn_clicked(self):
        # q = self.ur.get_q()
        # pub = self.ur.pub
        self.ur_node.ur_step_move_down()
    @pyqtSlot()
    def on_ur_left_btn_clicked(self):
        # q = self.ur.get_q()
        # pub = self.ur.pub
        self.ur_node.ur_step_move_left()
    @pyqtSlot()
    def on_ur_right_btn_clicked(self):
        # q = self.ur.get_q()
        # pub = self.ur.pub
        self.ur_node.ur_step_move_right()

    @pyqtSlot()
    def on_add_pos_to_list_btn_clicked(self):
        pos_tag = self.get_note_lEdit.text()
        self.pos_list_comboBox.addItems([pos_tag])
        self.ur_node.add_pos_to_list(pos_tag)

    @pyqtSlot()
    def on_del_pos_from_list_btn_clicked(self):
        pos_tag = self.pos_list_comboBox.currentText()
        cindex = self.pos_list_comboBox.currentIndex()
        self.ur_node.del_pos_from_list(pos_tag)
        self.pos_list_comboBox.removeItem(cindex)

    @pyqtSlot()
    def on_loop_move_btn_clicked(self):
        times = int(self.set_loop_times_lEdit.text())
        self.ur_node.loop_move(times)

    @pyqtSlot()
    def on_show_list_btn_clicked(self):
        # print("===")
        keys = self.ur_node.get_pos_dict_key()
        # print("===")
        str = "The path list is:" + '\n' + "->".join(keys)
        # print(keys)
        reply4 = QMessageBox.information(self, "path", str, QMessageBox.Yes)

    # endregion