# -*- coding: utf-8 -*-
# PyQt的导入必须放在前面，否则release运行时会崩溃！
from PyQt5.QtWidgets import QDialog
from CommonDefine import *


class SettingDlg(QDialog):
    sigParamChanged = pyqtSignal()

    def init(self, stLidarParam, parent=None):
        if parent is not None:
            self.setParent(parent)

        self.stLidarParam = stLidarParam
        # self.stLidarParam.printSelf()
        # self.stCameraParam.printSelf()

        self.setObjectName("Settings")
        self.resize(50, 100)

        # 设置 正则表达式，显示输入0.0.0.0~255.255.255.255
        rx = QtCore.QRegExp("((2[0-4]\\d|25[0-5]|[01]?\\d\\d?)\\.){3}(2[0-4]\\d|25[0-5]|[01]?\\d\\d?)")
        ipValidator = QtGui.QRegExpValidator(rx)  # 实例化自定义校验器

        self.gridLayout = QtWidgets.QGridLayout(self)
        self.gridLayout.setContentsMargins(15, 15, 15, 15)
        self.gridLayout.setSpacing(10)
        self.gridLayout.setObjectName("gridLayout")
        ###------------------------###
        # self.labLIDAR1 = QtWidgets.QLabel(self)
        # self.labLIDAR1.setObjectName("labLIDAR1")
        # self.labLIDAR1.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        #
        # self.labSrcIp1 = QtWidgets.QLabel(self)
        # self.labSrcIp1.setObjectName("labSrcIp1")
        # self.labSrcIp1.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        # self.edtSrcIp1 = QtWidgets.QLineEdit(self)
        # self.edtSrcIp1.setObjectName("edtSrcIp1")
        # self.edtSrcIp1.setValidator(ipValidator)
        # # self.edtSrcIp1.setInputMask("000.000.000.000;")  # 只要加上;0保证有默认值即可使得正则和mask同时生效。edtForwardDstIp
        # self.edtSrcIp1.setText(self.stLidarParam.getLidarSrcIp(0))  # 8号门雷达IP
        #
        #
        # self.labLIDAR2 = QtWidgets.QLabel(self)
        # self.labLIDAR2.setObjectName("labLIDAR2")
        # self.labLIDAR2.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        #
        # self.labSrcIp2 = QtWidgets.QLabel(self)
        # self.labSrcIp2.setObjectName("labSrcIp2")
        # self.labSrcIp2.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        # self.edtSrcIp2 = QtWidgets.QLineEdit(self)
        # self.edtSrcIp2.setObjectName("edtSrcIp2")
        # self.edtSrcIp2.setValidator(ipValidator)
        # # self.edtSrcIp2.setInputMask("000.000.000.000;")  # 只要加上;0保证有默认值即可使得正则和mask同时生效。edtForwardDstIp
        # self.edtSrcIp2.setText(self.stLidarParam.getLidarSrcIp(1))  # 8号门雷达IP
        #
        #
        # self.labLIDAR3 = QtWidgets.QLabel(self)
        # self.labLIDAR3.setObjectName("labLIDAR3")
        # self.labLIDAR3.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        #
        # self.labSrcIp3 = QtWidgets.QLabel(self)
        # self.labSrcIp3.setObjectName("labSrcIp3")
        # self.labSrcIp3.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        # self.edtSrcIp3 = QtWidgets.QLineEdit(self)
        # self.edtSrcIp3.setObjectName("edtSrcIp3")
        # self.edtSrcIp3.setValidator(ipValidator)
        # # self.edtSrcIp3.setInputMask("000.000.000.000;")  # 只要加上;0保证有默认值即可使得正则和mask同时生效。edtForwardDstIp
        # self.edtSrcIp3.setText(self.stLidarParam.getLidarSrcIp(2))  # 8号门雷达IP
        ###------------------------###

        self.btnOK = QtWidgets.QPushButton(self)
        self.btnOK.setObjectName("btnOK")
        self.btnOK.setMinimumSize(50, 24)
        self.btnCacel = QtWidgets.QPushButton(self)
        self.btnCacel.setObjectName("btnCacel")
        self.btnCacel.setMinimumSize(50, 24)

        self.btnlayout = QtWidgets.QHBoxLayout()
        spacerItem = QtWidgets.QSpacerItem(0, 0, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.btnlayout.addItem(spacerItem)
        self.btnlayout.addWidget(self.btnCacel)
        self.btnlayout.addWidget(self.btnOK)

        ###------------------------###
        # self.lidarlayout1 = QtWidgets.QHBoxLayout(self)
        # self.lidarlayout1.addWidget(self.labLIDAR1)
        # self.lidarlayout1.addWidget(self.labSrcIp1)
        # self.lidarlayout1.addWidget(self.edtSrcIp1)
        #
        # self.lidarlayout2 = QtWidgets.QHBoxLayout(self)
        # self.lidarlayout2.addWidget(self.labLIDAR2)
        # self.lidarlayout2.addWidget(self.labSrcIp2)
        # self.lidarlayout2.addWidget(self.edtSrcIp2)
        #
        # self.lidarlayout3 = QtWidgets.QHBoxLayout(self)
        # self.lidarlayout3.addWidget(self.labLIDAR3)
        # self.lidarlayout3.addWidget(self.labSrcIp3)
        # self.lidarlayout3.addWidget(self.edtSrcIp3)
        ###------------------------###

        # self.gridLayout.addLayout(self.lidarlayout1, 0, 0, 1, 7)
        # self.gridLayout.addLayout(self.lidarlayout2, 1, 0, 1, 7)
        # self.gridLayout.addLayout(self.lidarlayout3, 2, 0, 1, 7)

        self.gridLayout.addLayout(self.btnlayout, 1, 0, 1, 7)

        self.retranslateUi()
        self.connectSlots()
        QtCore.QMetaObject.connectSlotsByName(self)

    def retranslateUi(self):
        _translate = QtCore.QCoreApplication.translate
        self.setWindowTitle(_translate("Dialog", "Settings"))
        # self.labSrcIp1.setText(_translate("Dialog", "src1"))
        # self.labLIDAR1.setText(_translate("Dialog", "LIDAR1"))
        # self.labSrcIp2.setText(_translate("Dialog", "src2"))
        # self.labLIDAR2.setText(_translate("Dialog", "LIDAR2"))
        # self.labSrcIp3.setText(_translate("Dialog", "src3"))
        # self.labLIDAR3.setText(_translate("Dialog", "LIDAR3"))
        self.btnOK.setText(_translate("Dialog", "OK"))
        self.btnCacel.setText(_translate("Dialog", "Cancel"))

    def connectSlots(self):
        self.btnOK.clicked.connect(self.slotAccept)
        self.btnCacel.clicked.connect(self.slotReject)

    def slotAccept(self):
        MyLog.writeLog("{}[:{}] - Rstart online...".
                 format(__file__.split('/')[len(__file__.split('/')) - 1],
                        sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)

        # self.stLidarParam.setStationSrcIp(self.edtSrcIp1.text(), 0)
        # self.stLidarParam.setStationSrcIp(self.edtSrcIp2.text(), 1)
        # self.stLidarParam.setStationSrcIp(self.edtSrcIp3.text(), 2)
        # LidarCount = 0
        # if not len(self.edtSrcIp1.text()) == 0:
        #     LidarCount += 1
        # if not len(self.edtSrcIp2.text()) == 0:
        #     LidarCount += 1
        # if not len(self.edtSrcIp3.text()) == 0:
        #     LidarCount += 1
        # self.stLidarParam.setLidarCount(LidarCount)
        # self.stLidarParam.setLidarCount(LidarCount)

        self.stLidarParam.setUdpSocket(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
        self.stLidarParam.setShowPc(True)
        self.stLidarParam.setOnline(True)
        # self.stLidarParam.printSelf()

        # serializationParam(self.stLidarParam.getParam(), bSave=True)
        self.sigParamChanged.emit()

        MyLog.writeLog("{}[:{}] - Start online".
                 format(__file__.split('/')[len(__file__.split('/')) - 1],
                        sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)

        self.accept()

    # def slotAccept(self):
    #     # c语言中的c,s,p类型对应Python3.0中 Python type都是bytes类型。
    #     # 所以在使用这些类型的时候，要将需要pack的字符串写成bytes型的,
    #     # encode函数可以将字符串转换成bytes类型
    #     LidarSrcIP = self.edtSrcIp.text().encode('utf-8')
    #     LidarDstIP = self.edtDstIp.text().encode('utf-8')
    #     CameraIP = self.edtCameraIp1.text()
    #     CameraChannel = self.edtCameraChannelCount.text()
    #     CameraUsr = self.edtCameraUsr1.text()
    #     CameraPwd = self.edtCameraPwd1.text()
    #
    #     strLidarPackType = str(len(LidarSrcIP)) + "s" + str(len(LidarDstIP)) + "s" + "2?H"
    #     self.bufLidarParam = struct.pack(strLidarPackType, LidarSrcIP, LidarDstIP, True, True, 100)  # LidarSrcIP, LidarDstIP, bShowPC, bOnline, nFPS
    #
    #     video_path = "rtsp://" + CameraUsr + ":" + CameraPwd + "@" + CameraIP \
    #                  + ":554/cam/realmonitor?channel=" + CameraChannel + "&subtype=0"
    #     strCameraPackType = str(len(video_path.encode('utf-8'))) + "s" + "2?H"
    #     self.bufCameraParam = struct.pack(strCameraPackType, video_path.encode('utf-8'), True, True, 100)  # video_path, bShowBox, bOnline, nFps
    #
    #     self.sigParamChanged.emit(strLidarPackType, self.bufLidarParam, strCameraPackType, self.bufCameraParam)
    #     self.accept()

    def slotReject(self):
        self.reject()
