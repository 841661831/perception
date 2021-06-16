# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ActionTip.ui'
#
# Created by: PyQt5 UI code generator 5.13.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QDialog
from CommonDefine import *


class ActionTipDlg(QDialog):
    def __init__(self, strMsg="", stLidarParam=None):
        QDialog.__init__(self)
        # self.setWindowFlags(QtCore.Qt.WindowCloseButtonHint)
        self.setObjectName("ActionTip")
        self.setFixedSize(500, 200)

        self.strMac = "None"
        self.stLidarParam = None
        self.stCameraParam = None
        if stLidarParam is not None:
            self.stLidarParam = stLidarParam
        self.timerCheckMac = QTimer()
        self.timerCheckMac.timeout.connect(self.slotGetCorrectMac)
        self.timerCheckMac.start(1000)

        # self.setFixedHeight(200)
        self.mainLayout = QtWidgets.QVBoxLayout(self)
        self.mainLayout.setContentsMargins(15, 15, 15, 0)
        self.mainLayout.setSpacing(15)

        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")

        self.labelTip = QtWidgets.QLabel(strMsg)
        self.labelTip.setObjectName("label")
        self.labelTip.setFixedHeight(24)
        self.labelTip.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        self.gridLayout.addWidget(self.labelTip, 0, 0, 1, 2)
        if strMsg == "":
            self.setFixedSize(500, 150)
            self.labelTip.setVisible(False)

        self.label = QtWidgets.QLabel()
        self.label.setObjectName("label")
        self.label.setFixedSize(80, 24)
        self.label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.gridLayout.addWidget(self.label, 1, 0, 1, 1)

        self.lineEdit = QtWidgets.QLineEdit()
        self.lineEdit.setObjectName("lineEdit")
        self.lineEdit.setFixedHeight(24)
        self.gridLayout.addWidget(self.lineEdit, 1, 1, 1, 1)

        self.label_2 = QtWidgets.QLabel()
        self.label_2.setObjectName("label_2")
        self.label_2.setFixedSize(80, 24)
        self.label_2.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.gridLayout.addWidget(self.label_2, 2, 0, 1, 1)

        self.labelId = QtWidgets.QLabel()
        self.labelId.setObjectName("labelId")
        self.labelId.setFixedHeight(24)
        self.labelId.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.gridLayout.addWidget(self.labelId, 2, 1, 1, 1)

        self.buttonBox = QtWidgets.QDialogButtonBox(self)
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")

        self.mainLayout.addLayout(self.gridLayout)
        self.mainLayout.addWidget(self.buttonBox)
        self._translate = QtCore.QCoreApplication.translate
        self.buttonBox.button(QtWidgets.QDialogButtonBox.Ok).setText(self._translate("WJ-DepthViewer", 'Ok'))
        self.buttonBox.button(QtWidgets.QDialogButtonBox.Cancel).setText(self._translate("WJ-DepthViewer", 'Cancel'))
        self.retranslateUi(self)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

        self.msgBox = QMessageBox(QMessageBox.Warning, self._translate("ActionTip", 'Warning'),
                                  self._translate("ActionTip", "Lidar or camera network is unavailable,"
                                                               " or IP configuration is wrong!Please repair the network configuration,"
                                                               " or disconnect all network links of this machine"),
                                  QMessageBox.Ok)
        self.msgBox.setModal(True)

        QtCore.QMetaObject.connectSlotsByName(self)

    def retranslateUi(self, ActionTip):
        ActionTip.setWindowTitle(self._translate("ActionTip", "Action tip"))
        self.label.setText(self._translate("ActionTip", "Action Code:"))
        self.label_2.setText(self._translate("ActionTip", "Local ID:"))
        self.labelId.setText(self._translate("ActionTip", "Loading..."))

    def accept(self):
        # 激活码是16进制字串，在激活函数内不转换成2进制，再解密，激活
        nRet = my_authorization(strCheckAuthorizationPath, strRecordPath, self.lineEdit.text())
        if nRet == 0:
            self.resetParam()
            QDialog.accept(self)
        else:
            self.lineEdit.setStyleSheet("color:red")
            if nRet == 1:
                self.lineEdit.setText(self._translate("ActionTip", "Activation code error!"))
            elif nRet == 2:
                self.lineEdit.setText(self._translate("ActionTip", "The activation code has expired!"))
            elif nRet == 3:
                self.lineEdit.setText(self._translate("ActionTip", "The computer cannot be activated!"))
            elif nRet == 4:
                self.lineEdit.setText(self._translate("ActionTip", "Activation code timeout!"))
            else:
                self.lineEdit.setText(self._translate("ActionTip", "Activation fails!"))

    def reject(self):
        if not self.labelTip.isVisible():
            self.resetParam()
            QDialog.reject(self)
            return
        msg_box = QMessageBox()
        msg_box.setWindowTitle(self._translate("WJ-DepthViewer", 'Warning'))
        msg_box.setText(self._translate("WJ-DepthViewer", "Do you want to exit the software?"))
        msg_box.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
        buttonY = msg_box.button(QMessageBox.Ok)
        buttonY.setText(self._translate("WJ-DepthViewer", 'Ok'))
        buttonN = msg_box.button(QMessageBox.Cancel)
        buttonN.setText(self._translate("WJ-DepthViewer", 'Cancel'))
        msg_box.exec_()
        if msg_box.clickedButton() == buttonY:
            self.resetParam()
            QDialog.reject(self)
        # msg_box.addButton(QPushButton( self._translate("WJ-DepthViewer", 'Warning')), QMessageBox.YesRole)
        # ret = QMessageBox.question(self, self._translate("WJ-DepthViewer", 'Warning'), self._translate("WJ-DepthViewer", "Do you want to exit the software?"), QMessageBox.Ok | QMessageBox.Cancel)
        # if ret == QMessageBox.Ok:
        #     self.resetParam()zh
        #     QDialog.reject(self)

    def resetParam(self):
        self.lineEdit.setStyleSheet("color:black")
        self.lineEdit.setText("")

    def getReachableIp(self):
        if not my_isExist(ipTruePath):
            return None

        self.listDevIp = []
        fTrue = open(ipTruePath, 'r')
        for ip in fTrue.readlines():
            ip = ip.replace('\n', '')  # 替换掉换行符
            self.listDevIp.append(ip)

        if len(self.listDevIp) == 0:
            return []

        return self.listDevIp

    def slotGetCorrectMac(self):
        if self.strMac != "None":
            self.labelId.setText(str(encodeComputerID(self.strMac), encoding="utf8"))
            self.timerCheckMac.stop()
            MyLog.writeLog("{}[:{}] - Get the correct identification code!"
                           .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno),
                           LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)
            return

        self.labelId.setText(self._translate("ActionTip", "Loading..."))

        # 1.判断是否有mac地址无对应IP，有这样的mac就绑定！
        listMac = get_no_connect_mac_address()
        if len(listMac) > 0:
            self.strMac = listMac[0]
            self.labelId.setText(str(encodeComputerID(self.strMac), encoding="utf8"))
            self.timerCheckMac.stop()
            MyLog.writeLog("{}[:{}] - Get the correct identification code!"
                           .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno),
                           LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)
            return

        # 2.否则，判断配置文件中的雷达的IP是否能ping通，能则绑定与雷达同网段的IP
        listReachIp = self.getReachableIp()
        while listReachIp is None:
            QApplication.processEvents(QtCore.QEventLoop.ExcludeUserInputEvents, 100)
            listReachIp = self.getReachableIp()

        t1 = time.time()
        while len(listReachIp) == 0:
            if time.time() - t1 > 3:
                MyLog.writeLog("{}[:{}] - ping time out!".format(__file__.split('/')[len(__file__.split('/')) - 1],
                    sys._getframe().f_lineno), LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)
                break

            QApplication.processEvents(QtCore.QEventLoop.ExcludeUserInputEvents, 100)
            listReachIp = self.getReachableIp()

        # 3.如果配置文件中的IP都ping不通，则提示异常
        if len(listReachIp) == 0:
            MyLog.writeLog("{}[:{}] - No reachable ip!"
                           .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno),
                           LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)
            if self.msgBox.isHidden():
                self.msgBox.show()
            return

        # 4.如果ping通了，则在本机中寻找与ping通IP在同网段的IP对应的mac地址
        listSrcIp = []
        for StationIp in self.stLidarParam.getlistAttachedIp():
            listSrcIp.append(StationIp)

        # 查找ping通的是配置文件中的哪个IP
        strIpTemp = ""
        bReachable = False
        for i in range(len(listSrcIp)):
            strIpTemp = listSrcIp[i]
            if strIpTemp in listReachIp:
                bReachable = True
                break

        # 寻找与ping通IP在同网段的IP对应的mac地址
        if bReachable:
            self.strMac = get_same_net_segment_mac_address(strIpTemp)
            if self.strMac == "None":
                MyLog.writeLog("{}[:{}] - Cannot find an IP in the same network segment as the reachable IP!"
                               .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno),
                               LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)
                if self.msgBox.isHidden():
                    self.msgBox.show()
            else:
                self.labelId.setText(str(encodeComputerID(self.strMac), encoding="utf8"))
                self.timerCheckMac.stop()
                MyLog.writeLog("{}[:{}] - Get the correct identification code!"
                               .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno),
                               LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)
                return
        else:  # 找不到与ping通IP在同网段的IP，则无法确定应该绑定哪个mac, 提示异常
            MyLog.writeLog("{}[:{}] - Cannot find an IP in the same network segment as the reachable IP!"
                           .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno),
                           LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)
            if self.msgBox.isHidden():
                self.msgBox.show()

