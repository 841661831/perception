# -*- coding: utf-8 -*-
# PyQt的导入必须放在前面，否则release运行时会崩溃！
from WinCommon import *


class SettingDlg(QDialog):
    sigParamChanged = pyqtSignal()

    def init(self, stSysState, parent=None):
        if parent is not None:
            self.setParent(parent)

        self.stSysState = stSysState

        self.setObjectName("Settings")
        self.resize(50, 100)

        self.gridLayout = QtWidgets.QGridLayout(self)
        self.gridLayout.setContentsMargins(15, 15, 15, 15)
        self.gridLayout.setSpacing(10)
        self.gridLayout.setObjectName("gridLayout")

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

        self.gridLayout.addLayout(self.btnlayout, 1, 0, 1, 7)

        self.retranslateUi()
        self.connectSlots()
        QtCore.QMetaObject.connectSlotsByName(self)

    def retranslateUi(self):
        self.setWindowTitle("状态切换")
        self.btnOK.setText("确定")
        self.btnCacel.setText("取消")

    def connectSlots(self):
        self.btnOK.clicked.connect(self.slotAccept)
        self.btnCacel.clicked.connect(self.slotReject)

    def slotAccept(self):
        global_log.warning("Restart online!")
        self.stSysState.set_online(True)
        self.sigParamChanged.emit()
        global_log.warning( "Start online")
        self.accept()

    def slotReject(self):
        self.reject()
