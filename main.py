from PyQt5.uic import loadUi
from PyQt5 import QtGui, QtCore
from matplotlib.backends.backend_qt5agg import (NavigationToolbar2QT as NavigationToolbar)
from PyQt5.QtWidgets import QMessageBox
import re
import threading
import socket
import csv
import time
import pickle
from PyQt5.QtWidgets import *
from oglwidget import OglWidget
import os
import functools
import numpy as np
from numpy import sin, cos
from ahrsfilter import AHRS
from magnet import mag_cal


class PaintPicture(QDialog):
    def __init__(self):
        super(PaintPicture, self).__init__()
        self.setWindowTitle("Help")
        layout = QVBoxLayout()
        self.setLayout(layout)
        self.show()

    def showImage(self):
        filename = r'helpPage.jpg'
        image = QtGui.QImage(filename)
        self.imageLabel = QLabel()
        self.imageLabel.setPixmap(QtGui.QPixmap.fromImage(image))
        layout = self.layout()
        layout.addWidget(self.imageLabel)


class OnlinePage(QMainWindow):

    def __init__(self):
        QMainWindow.__init__(self)
        loadUi(r"ui_design.ui", self)
        self.plotScroll.setStyleSheet("background-color: #ffffff;")
        self.model = OglWidget()
        self.modelLayout.addStretch(0)
        self.modelLayout.addWidget(self.model)
        self.setWindowTitle("IMU")
        f = QtCore.QFile("theme.qss")
        f.open(QtCore.QFile.ReadOnly)
        style = (QtCore.QTextStream(f).readAll())
        f.close()
        self.setStyleSheet(style)
        self.rawPlots = [self.plot1, self.plot2, self.plot3]
        self.addToolBar(NavigationToolbar(self.eulerPlot.canvas, self))
        for p in self.rawPlots:
            self.addToolBar(NavigationToolbar(p.canvas, self))

        self.flags = [False, False, False]
        self.plotRange = self.plotSlider.value()
        self.selectedImu = int(self.imuCombo.currentIndex())
        self.axName = ["Acceleration (g)", "Gyroscope (deg/sec)", "Magnetometer (uT)"]
        self.savePath = ""
        self.sockets = [None, None, None]
        self.imuCount = 3
        self.reg = 5000
        self.mp = 10000
        self.yLim = [(-2, 2), (-450, 450), (-50, 50)]
        self.plotSkip = 3
        self.modelUpdateTime = 20  # ms
        self.calibDUration = 10  # s
        self.port = 8888
        self.ips = [self.ip1, self.ip2, self.ip3]
        self.imuIcons = [self.imu1icn, self.imu2icn, self.imu3icn]

        self.calibData = [[], [], [], []]
        self.magMtx = [np.array([0, 0, 0]), np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])]
        self.frameMtx = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

        self.data = []
        self.quaternion = [1, 1, 1, 1]
        self.filteredData = []
        self.angles = []
        for _ in range(self.imuCount):
            self.data.append([])
            self.filteredData.append([])
            self.angles.append([])

        self.settings = []
        try:
            with open('settings', 'rb') as f:
                self.settings = pickle.load(f)
                self.mac = self.settings[0]
                self.mgrf = float(self.settings[1][0])
                self.frq = int(self.settings[1][1])
        except Exception as e:
            # print(e)
            self.mac = ['', '', '']
            self.mgrf = 0.0486615
            self.frq = 100
            self.settings = [self.mac, [self.mgrf, self.frq]]

        self.mac1.setText(self.mac[0])
        self.mac2.setText(self.mac[1])
        self.mac3.setText(self.mac[2])
        self.magRef.setText(str(self.mgrf))
        self.freq.setText(str(self.frq))

        self.connectBtn.clicked.connect(self.connectClicked)
        self.saveBtn.clicked.connect(self.saveClicked)
        self.plotSlider.valueChanged[int].connect(self.setPlotRange)
        self.paramApply.clicked.connect(self.applyParam)
        self.imuCombo.currentIndexChanged.connect(self.imuComboChanged)
        self.calibBtn.clicked.connect(self.calibBtnClicked)
        self.mtxBtn.clicked.connect(self.mtxBtnClicked)
        self.resetCalibBtn.clicked.connect(self.resetCalib)
        self.importBtn.clicked.connect(self.importBtnClicked)
        self.exportBtn.clicked.connect(self.exportBtnClicked)
        self.helpBtn.clicked.connect(self.helpClicked)

    def helpClicked(self):
        self.helpWindow = PaintPicture()
        self.helpWindow.showImage()

    def popup(self, title, txt, inf, det, detflag=False):
        msg = QMessageBox()
        msg.setWindowTitle(title)
        msg.setText(txt)
        msg.setIcon(QMessageBox.Question)
        msg.setStandardButtons(QMessageBox.Ok)
        msg.setDefaultButton(QMessageBox.Retry)
        msg.setInformativeText(inf)
        if detflag:
            msg.setDetailedText(det)
        msg.exec_()

    def imuComboChanged(self):
        self.selectedImu = int(self.imuCombo.currentIndex())
        threading.Thread(target=self.drawPlot).start()
        self.draw3D()

    def importBtnClicked(self):
        pth = QFileDialog.getOpenFileName(self)[0]
        try:
            with open(pth, 'rb') as f:
                mtx = pickle.load(f)
                self.magMtx = mtx[0]
                self.frameMtx = mtx[1]
        except Exception as e:
            self.popup("Error", "Wrong File Format or Invalid Address", "", "")
            return
        self.calibBtn.setText("Calibration Done")
        self.calibBtn.setStyleSheet("background-color: #70bf73;")
        self.calibBtn.setEnabled(False)

    def exportBtnClicked(self):
        pth = QFileDialog.getExistingDirectory(self, options=QFileDialog.ShowDirsOnly)
        try:
            with open(pth + r'/Calibration Matrices', 'wb') as f:
                mtx = [self.magMtx, self.frameMtx]
                pickle.dump(mtx, f)
        except Exception as e:
            self.popup("Error", "Wrong File Format or Invalid Address", "", "")
            return

    def mtxBtnClicked(self):
        self.popup("Calibration", "", 'Click on \"Show Details\" to ' 'See CalibRation Matrices',
                   'Magnet Fit Coefficients:\n{}\nMagnet Rotation Vector:\n{}\n ' 'Frame Calibration:\n{}'
                   .format(self.magMtx[0], self.magMtx[1], self.frameMtx), detflag=True)
        # print(self.magMtx, self.frameMtx)

    def applyParam(self):
        if len(self.magRef.text()):
            self.mgrf = float(self.magRef.text())
        else:
            self.mgrf = 0.0486615
        if len(self.freq.text()):
            self.frq = int(self.freq.text())
        else:
            self.frq = 100
        with open('settings', 'wb') as f:
            self.settings[1] = [self.mgrf, self.frq]
            pickle.dump(self.settings, f)

    # find ip form arp table (by MAC)
    def findIP(self, mac):
        if mac == 'test':
            return '127.0.0.1'
        if not len(mac):
            mac = "x:x:x:x:x:x"
        mac = mac.replace(":", "-")
        f = os.popen('arp -a')
        while True:
            temp = f.readlines(1)
            if not len(temp):
                break
            if mac in temp[0]:
                ip = re.findall(r'[0-9]+(?:\.[0-9]+){3}', temp[0])
                if len(ip):
                    return ip[0]
                else:
                    return None

    def rotationMtx(self, state):
        ahrs = AHRS(self.frq, self.mgrf)
        ang = []
        for i in range(0, len(self.calibData[state][1][0])):
            ang.append([np.degrees(d) for d in ahrs.run([self.calibData[state][1][0][i],
                                                         self.calibData[state][1][1][i],
                                                         self.calibData[state][1][2][i]],
                                                        [self.calibData[state][2][0][i],
                                                         self.calibData[state][2][1][i],
                                                         self.calibData[state][2][2][i]],
                                                        [self.calibData[state][3][0][i],
                                                         self.calibData[state][3][1][i],
                                                         self.calibData[state][3][2][i]])[:3]])

        # test frame calibration data
        with open(r'calib' + r'.csv', "w+", newline='') as f:
            writer = csv.writer(f)
            temp = ["Time"]
            for p in self.axName:
                temp.append(p + "_X")
                temp.append(p + "_Y")
                temp.append(p + "_Z")
            writer.writerow(temp)
            for k in range(0, len(self.calibData[state][0])):
                temp = [self.calibData[state][0][k], self.calibData[state][1][0][k], self.calibData[state][1][1][k],
                        self.calibData[state][1][2][k], self.calibData[state][2][0][k], self.calibData[state][2][1][k],
                        self.calibData[state][2][2][k], self.calibData[state][3][0][k], self.calibData[state][3][1][k],
                        self.calibData[state][3][2][k]]
                writer.writerow(temp)

        alpha = np.mean([c[0] for c in ang])
        beta = np.mean([c[1] for c in ang])
        gamma = np.mean([c[2] for c in ang])

        r11 = cos(gamma) * cos(beta)
        r12 = cos(gamma) * sin(beta) * sin(alpha) - sin(gamma) * cos(alpha)
        r13 = cos(gamma) * sin(beta) * cos(alpha) + sin(gamma) * sin(alpha)
        r21 = sin(gamma) * cos(beta)
        r22 = sin(gamma) * sin(beta) * sin(alpha) + cos(gamma) * cos(alpha)
        r23 = sin(gamma) * sin(beta) * sin(alpha) - cos(gamma) * sin(alpha)
        r31 = -sin(beta)
        r32 = cos(beta) * sin(alpha)
        r33 = cos(beta) * cos(alpha)

        self.frameMtx = np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])

    def startCalib(self):
        self.connectBtn.setEnabled(False)
        self.saveBtn.setEnabled(False)
        self.calibBtn.setText("Recording For {} Seconds...".format(self.calibDUration))

    def finishCalib(self, state, res=False, txt=""):
        if not res:
            self.resetCalib()
        else:
            self.calibBtn.setText(txt)
            if txt == "Frame Calibration (Magnet Calibrated)":
                try:
                    self.magMtx = mag_cal([[[self.calibData[0][3][0], self.calibData[0][3][1], self.calibData[0][3][2]]
                                            for _ in range(len(self.calibData[0][3][0]))],
                                           [[self.calibData[1][3][0], self.calibData[1][3][1], self.calibData[1][3][2]]
                                            for _ in range(len(self.calibData[1][3][0]))],
                                           [[self.calibData[2][3][0], self.calibData[2][3][1], self.calibData[2][3][2]]
                                            for _ in range(len(self.calibData[2][3][0]))]])
                except Exception as e:
                    print(e)
                    self.resetCalib()
            elif txt == "Calibration Done":
                try:
                    self.rotationMtx(state)
                    self.calibBtn.setStyleSheet("background-color: #70bf73;")
                    self.calibBtn.setEnabled(False)
                except Exception as e:
                    self.resetCalib()

        self.connectBtn.setEnabled(True)
        self.saveBtn.setEnabled(True)

    def resetCalib(self):
        self.calibBtn.setStyleSheet("background-color: qlineargradient(spread:pad, x1:0.5, y1:0, x2:0.5, y2:1, "
                                    "stop:0 #fbfdfd, stop:0.5 #ffffff, stop:1 #fbfdfd); :disabled {background-color: "
                                    "qlineargradient(spread:pad, x1:0.5, y1:0, x2:0.5, y2:1, stop:0 #dce7eb, stop:0.5 "
                                    "#e0e8eb, stop:1 #dee7ec);")
        self.calibBtn.setEnabled(True)
        self.calibBtn.setText("Magnetometer Calibration (Z Position)")
        self.calibData = [[], [], [], []]
        self.magMtx = [np.array([0, 0, 0]), np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])]
        self.frameMtx = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    def recordForCalib(self, state, txt):
        ip = self.findIP(self.mac1.text())
        port = self.port
        try:
            sk = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sk.connect((ip, port))
        except Exception as e:
            self.finishCalib(state)
            sk.close()
            return

        self.calibData[state] = [[],
                                 [[], [], []],
                                 [[], [], []],
                                 [[], [], []],
                                 [[], [], []]]
        count = 0
        err = 0
        for i in range(0, self.calibDUration * self.frq):
            try:
                recvData = sk.recv(41)
            except Exception as e:
                self.finishCalib(state)
                sk.close()
                return

            try:
                self.calibData[state][0].append(int.from_bytes(recvData[0:4], 'big') / self.mp - self.reg)
                for i in range(1, 4):
                    self.calibData[state][1][i - 1].append(round(int.from_bytes(recvData[4 * i:4 + 4 * i], 'big') /
                                                                 self.mp - self.reg, 4))
                for i in range(4, 7):
                    self.calibData[state][2][i - 4].append(round(int.from_bytes(recvData[4 * i:4 + 4 * i], 'big') /
                                                                 self.mp - self.reg, 4))
                for i in range(7, 10):
                    self.calibData[state][3][i - 7].append(round(int.from_bytes(recvData[4 * i:4 + 4 * i], 'big') /
                                                                 self.mp - self.reg, 4))
                for i in range(0, 3):
                    self.calibData[state][4][i].append(bin(recvData[40])[- (i + 1)])

            # remove corrupted packets
            except Exception as e:
                # print(e)
                err += 1
                m = [len(self.calibData[state][0])]
                for t in range(1, 4):
                    for u in range(0, 3):
                        m.append(len(self.calibData[state][t][u]))
                m = min(m)
                if len(self.calibData[state][0]) > m:
                    del self.calibData[state][0][-1]
                for t in range(1, 5):
                    for u in range(0, 3):
                        if len(self.calibData[state][t][u]) > m:
                            del self.calibData[state][t][u][-1]
                if count % 100 == 0:
                    print("{p:.6f}%".format(p=100 * err / count))
                    # print(err, count)
                    continue

            if state == 3:
                fl = np.array([self.calibData[state][3][0][-1], self.calibData[state][3][1][-1],
                               self.calibData[state][3][2][-1]])
                fl = np.dot((fl - self.magMtx[0]), self.magMtx[1])
                self.calibData[state][3][0][-1] = fl[0]
                self.calibData[state][3][1][-1] = fl[1]
                self.calibData[state][3][2][-1] = fl[2]

            count += 1

        self.finishCalib(state, res=True, txt=txt)

    def calibBtnClicked(self):
        if self.calibBtn.text() == "Magnetometer Calibration (Z Position)":
            self.startCalib()
            threading.Thread(target=self.recordForCalib, args=[0, "Magnetometer Calibration (Y Position)"]).start()

        elif self.calibBtn.text() == "Magnetometer Calibration (Y Position)":
            self.startCalib()
            threading.Thread(target=self.recordForCalib, args=[1, "Magnetometer Calibration (X Position)"]).start()

        elif self.calibBtn.text() == "Magnetometer Calibration (X Position)":
            self.startCalib()
            threading.Thread(target=self.recordForCalib, args=[2, "Frame Calibration (Magnet Calibrated)"]).start()

        elif self.calibBtn.text() == "Frame Calibration (Magnet Calibrated)":
            self.startCalib()
            threading.Thread(target=self.recordForCalib, args=[3, "Calibration Done"]).start()

    def setPlotRange(self, value):
        self.plotRange = value

    def connectClicked(self):
        if self.connectBtn.text() == "Connect":
            self.mac = [self.mac1.text(), self.mac2.text(), self.mac3.text()]
            with open('settings', 'wb') as f:
                self.settings[0] = self.mac
                pickle.dump(self.settings, f)
            threading.Thread(target=self.run, args=[0, self.findIP(self.mac1.text()), self.port]).start()
            threading.Thread(target=self.run, args=[1, self.findIP(self.mac2.text()), self.port]).start()
            threading.Thread(target=self.run, args=[2, self.findIP(self.mac3.text()), self.port]).start()
            threading.Thread(target=self.drawPlot).start()
            self.draw3D()

        else:
            self.disconnectFromIMU([0, 1, 2])

    def disconnectFromIMU(self, imu):
        for i in imu:
            self.imuIcons[i].setPixmap(QtGui.QPixmap(r"d.png"))
            self.flags[i] = False
            try:
                self.sockets[i].close()
            except Exception as e:
                # print(e)
                pass

        if self.flags == [False, False, False]:
            self.connectBtn.setText("Connect")
            self.saveBtn.setEnabled(True)
            self.calibBtn.setEnabled(True)
            self.importBtn.setEnabled(True)
            self.resetCalibBtn.setEnabled(True)

    def connected(self, imu):
        self.calibBtn.setEnabled(False)
        self.importBtn.setEnabled(False)
        self.saveBtn.setEnabled(False)
        self.resetCalibBtn.setEnabled(False)
        self.connectBtn.setText("Disconnect")
        for i in imu:
            self.flags[i] = True
            self.imuIcons[i].setPixmap(QtGui.QPixmap(r"c.png"))
            self.data[i] = [[],
                            [[], [], []],
                            [[], [], []],
                            [[], [], []],
                            [[], [], []]]
            self.filteredData[i] = [[],
                                    [[], [], []],
                                    [[], [], []],
                                    [[], [], []]]

    def run(self, imu, ip, port):
        if ip is None:
            self.disconnectFromIMU([imu])
        self.ips[imu].setText(ip)

        try:
            self.sockets[imu] = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sockets[imu].connect((ip, port))
        except Exception as e:
            self.disconnectFromIMU([imu])

        self.connected([imu])
        err = 0
        count = 0
        ahrs = AHRS(self.frq, self.mgrf)

        while self.flags[imu]:
            count += 1
            try:
                recvData = self.sockets[imu].recv(41)
            except Exception as e:
                # print(e)
                self.disconnectFromIMU([imu])
                return

            # for i in range(0, 10):
            #     d = ''.join(format(x, '02x') for x in bytearray(recvData[0 + 4 * i:4 + 4 * i]))
            #     t = (int(d, 16) / self.mp) - self.reg
            #     print(str(round(t, 4)).zfill(8), end=', ')
            # print()

            try:
                self.data[imu][0].append(int.from_bytes(recvData[0:4], 'big') / self.mp - self.reg)
                for i in range(1, 4):
                    self.data[imu][1][i - 1].append(round(int.from_bytes(recvData[4 * i:4 + 4 * i], 'big') /
                                                          self.mp - self.reg, 4))
                for i in range(4, 7):
                    self.data[imu][2][i - 4].append(round(int.from_bytes(recvData[4 * i:4 + 4 * i], 'big') /
                                                          self.mp - self.reg, 4))
                for i in range(7, 10):
                    self.data[imu][3][i - 7].append(round(int.from_bytes(recvData[4 * i:4 + 4 * i], 'big') /
                                                          self.mp - self.reg, 4))
                for i in range(0, 3):
                    self.data[imu][4][i].append(bin(recvData[40])[- (i + 1)])

            # remove corrupted packets
            except Exception as e:
                # print(e)
                err += 1
                m = [len(self.data[imu][0])]
                for t in range(1, 4):
                    for u in range(0, 3):
                        m.append(len(self.data[imu][t][u]))
                m = min(m)
                if len(self.data[imu][0]) > m:
                    del self.data[imu][0][-1]
                for t in range(1, 5):
                    for u in range(0, 3):
                        if len(self.data[imu][t][u]) > m:
                            del self.data[imu][t][u][-1]
                if count % 100 == 0:
                    print("{p:.6f}%".format(p=100 * err / count))
                    # print(err, count)
                    continue

            self.filteredData[imu][0].append(self.data[imu][0][-1])

            fl = np.dot(self.frameMtx, np.array([self.data[imu][1][0][-1], self.data[imu][1][1][-1],
                                                 self.data[imu][1][2][-1]]).transpose())
            self.filteredData[imu][1][0].append(fl[0])
            self.filteredData[imu][1][1].append(fl[1])
            self.filteredData[imu][1][2].append(fl[2])

            fl = np.dot(self.frameMtx, np.array([self.data[imu][2][0][-1], self.data[imu][2][1][-1],
                                                 self.data[imu][2][2][-1]]).transpose())
            self.filteredData[imu][2][0].append(fl[0])
            self.filteredData[imu][2][1].append(fl[1])
            self.filteredData[imu][2][2].append(fl[2])

            fl = np.dot((np.array([self.data[imu][3][0][-1], self.data[imu][3][1][-1],
                                   self.data[imu][3][2][-1]]) - self.magMtx[0]), self.magMtx[1])
            fl = np.dot(self.frameMtx, fl.transpose())
            self.filteredData[imu][3][0].append(fl[0])
            self.filteredData[imu][3][1].append(fl[1])
            self.filteredData[imu][3][2].append(fl[2])

            euler = ahrs.run([self.filteredData[imu][1][0][-1],
                              self.filteredData[imu][1][1][-1],
                              self.filteredData[imu][1][2][-1]],
                             [self.filteredData[imu][2][0][-1],
                              self.filteredData[imu][2][1][-1],
                              self.filteredData[imu][2][2][-1]],
                             [self.filteredData[imu][3][0][-1],
                              self.filteredData[imu][3][1][-1],
                              self.filteredData[imu][3][2][-1]])
            self.quaternion = euler[3]
            self.angles[imu].append([np.degrees(d) for d in euler[:3]])

            # print(np.degrees(self.angles[self.selectedImu][-1][0]),
            #                            np.degrees(self.angles[self.selectedImu][-1][1]),
            #                            np.degrees(self.angles[self.selectedImu][-1][2]))

        self.disconnectFromIMU([imu])

    def drawPlot(self):
        time.sleep(0.5)
        self.plotRange = self.plotSlider.value()
        counter = 0
        while self.flags[self.selectedImu]:
            self.selectedImu = int(self.imuCombo.currentIndex())
            counter += 1
            if not len(self.filteredData[self.selectedImu]):
                continue
            if counter % self.plotSkip != 0:
                continue

            for ind, p in enumerate(self.rawPlots):
                p.canvas.axes.clear()
                if len(self.filteredData[self.selectedImu][0]) < self.plotRange:
                    m = min([len(self.filteredData[self.selectedImu][0]),
                             len(self.filteredData[self.selectedImu][ind + 1][0]),
                             len(self.filteredData[self.selectedImu][ind + 1][1]),
                             len(self.filteredData[self.selectedImu][ind + 1][2])])
                    p.canvas.axes.plot(self.filteredData[self.selectedImu][0][:m],
                                       self.filteredData[self.selectedImu][ind + 1][0][:m])
                    p.canvas.axes.plot(self.filteredData[self.selectedImu][0][:m],
                                       self.filteredData[self.selectedImu][ind + 1][1][:m])
                    p.canvas.axes.plot(self.filteredData[self.selectedImu][0][:m],
                                       self.filteredData[self.selectedImu][ind + 1][2][:m])

                else:
                    p.canvas.axes.plot(self.filteredData[self.selectedImu][0][-self.plotRange::1],
                                       self.filteredData[self.selectedImu][ind + 1][0][-self.plotRange::1])
                    p.canvas.axes.plot(self.filteredData[self.selectedImu][0][-self.plotRange::1],
                                       self.filteredData[self.selectedImu][ind + 1][1][-self.plotRange::1])
                    p.canvas.axes.plot(self.filteredData[self.selectedImu][0][-self.plotRange::1],
                                       self.filteredData[self.selectedImu][ind + 1][2][-self.plotRange::1])

                # p.canvas.axes.set_ylim(self.yLim[ind])
                p.canvas.axes.legend(('X', 'Y', 'Z'), loc='upper right')
                p.canvas.axes.set_title(self.axName[ind])
                p.canvas.axes.grid("True")
                p.canvas.draw()

            self.eulerPlot.canvas.axes.clear()
            if len(self.filteredData[self.selectedImu][0]) < self.plotRange:
                m = min([len(self.filteredData[self.selectedImu][0]), len(self.angles[self.selectedImu])])
                self.eulerPlot.canvas.axes.plot(self.filteredData[self.selectedImu][0][:m],
                                                [ag[0] for ag in self.angles[self.selectedImu][:m]])
                self.eulerPlot.canvas.axes.plot(self.filteredData[self.selectedImu][0][:m],
                                                [ag[1] for ag in self.angles[self.selectedImu][:m]])
                self.eulerPlot.canvas.axes.plot(self.filteredData[self.selectedImu][0][:m],
                                                [ag[2] for ag in self.angles[self.selectedImu][:m]])

            else:
                self.eulerPlot.canvas.axes.plot(self.filteredData[self.selectedImu][0][-self.plotRange::1],
                                                [ag[0] for ag in self.angles[self.selectedImu][-self.plotRange::1]])
                self.eulerPlot.canvas.axes.plot(self.filteredData[self.selectedImu][0][-self.plotRange::1],
                                                [ag[1] for ag in self.angles[self.selectedImu][-self.plotRange::1]])
                self.eulerPlot.canvas.axes.plot(self.filteredData[self.selectedImu][0][-self.plotRange::1],
                                                [ag[2] for ag in self.angles[self.selectedImu][-self.plotRange::1]])

            self.eulerPlot.canvas.axes.legend(('X', 'Y', 'Z'), loc='upper right')
            self.eulerPlot.canvas.axes.set_title("Euler Angles")
            self.eulerPlot.canvas.axes.grid("True")
            self.eulerPlot.canvas.draw()

    def rotate(self, timer):
        if self.flags[self.selectedImu]:
            # update with angles
            # self.model.updateWithAngle(self.angles[self.selectedImu][-1][0], self.angles[self.selectedImu][-1][1],
            #                            self.angles[self.selectedImu][-1][2])

            # update with Quaternion
            self.model.updateWithQuaternion(self.quaternion)
        else:
            timer.stop()

    def draw3D(self):
        time.sleep(0.5)
        timer = QtCore.QTimer(self)
        timerCallback = functools.partial(self.rotate, timer)
        timer.timeout.connect(timerCallback)
        timer.start(self.modelUpdateTime)

    def saveClicked(self):
        self.savePath = QFileDialog.getExistingDirectory(self, options=QFileDialog.ShowDirsOnly)
        threading.Thread(target=self.saveToCSV).start()

    def saveToCSV(self):
        if len(self.savePath):

            # Save Raw Data
            for i in range(self.imuCount):
                if not len(self.data[i][0]):
                    continue
                with open(self.savePath + r'/Raw Data_IMU_' + str(i + 1) + r'_' +
                          time.ctime().replace(':', '-') + r'.csv', "w+", newline='') as f:
                    writer = csv.writer(f)
                    temp = ["Time"]
                    for p in self.axName:
                        temp.append(p + "_X")
                        temp.append(p + "_Y")
                        temp.append(p + "_Z")
                    temp.append("Switch_1")
                    temp.append("Switch_2")
                    temp.append("Switch_3")
                    writer.writerow(temp)
                    for k in range(0, len(self.data[i][0])):
                        temp = [self.data[i][0][k], self.data[i][1][0][k], self.data[i][1][1][k], self.data[i][1][2][k],
                                self.data[i][2][0][k], self.data[i][2][1][k], self.data[i][2][2][k],
                                self.data[i][3][0][k], self.data[i][3][1][k], self.data[i][3][2][k],
                                self.data[i][4][0][k], self.data[i][4][1][k], self.data[i][4][2][k]]
                        writer.writerow(temp)

            # Save Filtered Data
            for i in range(self.imuCount):
                if not len(self.filteredData[i][0]):
                    continue
                with open(self.savePath + r'/Filtered Data_IMU_' + str(i + 1) + r'_' +
                          time.ctime().replace(':', '-') + r'.csv', "w+", newline='') as f:
                    writer = csv.writer(f)
                    temp = ["Time"]
                    for p in self.axName:
                        temp.append(p + "_X")
                        temp.append(p + "_Y")
                        temp.append(p + "_Z")
                    temp.append("Roll")
                    temp.append("Pitch")
                    temp.append("Yaw")
                    writer.writerow(temp)
                    for k in range(0, len(self.filteredData[i][0])):
                        temp = [self.filteredData[i][0][k], self.filteredData[i][1][0][k],
                                self.filteredData[i][1][1][k], self.filteredData[i][1][2][k],
                                self.filteredData[i][2][0][k], self.filteredData[i][2][1][k],
                                self.filteredData[i][2][2][k], self.filteredData[i][3][0][k],
                                self.filteredData[i][3][1][k], self.filteredData[i][3][2][k],
                                self.angles[i][k][0], self.angles[i][k][1], self.angles[i][k][2]]
                        writer.writerow(temp)


app = QApplication([])
window = OnlinePage()
window.show()
app.exec_()
