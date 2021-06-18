# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ActionTip.ui'
#
# Created by: PyQt5 UI code generator 5.13.1
#
# WARNING! All changes made in this file will be lost!

from WinCommon import *


class ActionTipDlg(QDialog):
    def __init__(self, strMsg="", stSysState=None, parent=None):
        QDialog.__init__(self)
        # self.setWindowFlags(QtCore.Qt.WindowCloseButtonHint)
        self.setObjectName("ActionTip")
        self.setFixedSize(500, 400)

        self.strMac = "None"
        self.stSysState = None
        if stSysState is not None:
            self.stSysState = stSysState
        if parent is not None:
            self.parent = parent

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
            self.setFixedSize(500, 350)
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

        self.labelQRcode = QtWidgets.QLabel()
        self.labelQRcode.setObjectName("labelQRcode")
        self.labelQRcode.setFixedHeight(200)
        self.labelQRcode.setFixedWidth(200)
        self.labelQRcode.setScaledContents(True)

        self.buttonBox = QtWidgets.QDialogButtonBox(self)
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")

        buttonY = self.buttonBox.button(QtWidgets.QDialogButtonBox.Ok)
        buttonY.setText('激活')
        buttonN = self.buttonBox.button(QtWidgets.QDialogButtonBox.Cancel)
        buttonN.setText('取消')

        self.mainLayout.addLayout(self.gridLayout)
        #add QRCode
        self.mainLayout.addWidget(self.labelQRcode, alignment=QtCore.Qt.AlignCenter)
        self.mainLayout.addWidget(self.buttonBox, alignment=QtCore.Qt.AlignVCenter)

        self.retranslateUi()
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

        QtCore.QMetaObject.connectSlotsByName(self)

    def retranslateUi(self):
        self.setWindowTitle("激活界面")
        self.label.setText("激活码:")
        self.label_2.setText("本地ID:")
        self.labelId.setText("加载中...")

    def accept(self):
        # 激活码是16进制字串，在激活函数内不转换成2进制，再解密，激活
        self.stSysState.set_act_code(self.lineEdit.text())
        self.parent.slotPublishWinState()
        time.sleep(1)
        self.slotGetCorrectMac()
        nRet = self.stSysState.get_active_res()
        if not self.stSysState.get_act_state() and nRet == 0:
            nRet = 5
        if nRet == 0:
            self.resetParam()
            QDialog.accept(self)
        else:
            if nRet == 1:
                self.lineEdit.setText("激活码错误！")
            elif nRet == 2:
                self.lineEdit.setText("激活码已过期！")
            elif nRet == 3:
                self.lineEdit.setText("该设备无法被激活！")
            elif nRet == 4:
                self.lineEdit.setText("激活超时！")
            else:
                self.lineEdit.setText("激活失败！")

    def reject(self):
        if not self.labelTip.isVisible():
            self.resetParam()
            QDialog.reject(self)
            return
        msg_box = QMessageBox()
        msg_box.setWindowTitle('警告')
        msg_box.setText("是否退出该软件")
        msg_box.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
        buttonY = msg_box.button(QMessageBox.Ok)
        buttonY.setText('确定')
        buttonN = msg_box.button(QMessageBox.Cancel)
        buttonN.setText('取消')
        msg_box.exec_()
        if msg_box.clickedButton() == buttonY:
            self.resetParam()
            QDialog.reject(self)

    def resetParam(self):
        self.lineEdit.setStyleSheet("color:black")
        self.lineEdit.setText("")

    def slotGetCorrectMac(self):
        if self.parent is not None:
            self.parent.slotUpdateSysState()
        self.labelId.setText("加载中...")
        s_localId = self.stSysState.get_localId()
        if len(s_localId) != 0:
            self.labelId.setText(s_localId)
            self.QRCodeImg = qrcode.make(s_localId)
            self.labelQRcode.setPixmap(ImageQt.toqpixmap(self.QRCodeImg))