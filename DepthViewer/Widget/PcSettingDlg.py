# -*- coding: utf-8 -*-
# PyQt的导入必须放在前面，否则release运行时会崩溃！
from PyQt5.QtWidgets import QDialog
from CommonDefine import *


class PcSettingDlg(QDialog):
    sigParamChanged = pyqtSignal()

    def init(self, stLidarParam, parent=None):
        if parent is not None:
            self.setParent(parent)

        self.setObjectName("PcSettingDlg")
        self.resize(300, 200)
        self.stLidarParam = stLidarParam

        self.gridLayout = QtWidgets.QGridLayout(self)
        self.gridLayout.setContentsMargins(15, 15, 15, 15)
        self.gridLayout.setSpacing(10)
        self.gridLayout.setObjectName("gridLayout")

        self.checkHighSpeed = QtWidgets.QCheckBox()
        self.checkHighSpeed.setObjectName("HighSpeedFlag")
        self.checkHighSpeed.setChecked(self.stLidarParam.getHighSpeed())
        self.nSelectBaseStation = self.stLidarParam.getBaseStationID()
        self.strIP = self.stLidarParam.getlistAttachedIp()[self.nSelectBaseStation]
        self.labBaseStationId = QtWidgets.QLabel(self)
        self.labBaseStationId.setAlignment(QtCore.Qt.AlignRight)
        self.spbBaseStationId = QDoubleSpinBox(self)
        self.spbBaseStationId.setRange(1, len(self.stLidarParam.getlistAttachedIp()))
        self.spbBaseStationId.setSingleStep(1)
        self.spbBaseStationId.setValue(1)
        self.spbBaseStationId.setDecimals(0)
        self.btnUpdate = QtWidgets.QPushButton(self)
        self.btnUpdate.setObjectName("btnOK")
        self.btnUpdate.setMinimumSize(50, 24)
        self.labBaseStationIp = QtWidgets.QLabel(self)
        self.labBaseStationIp.setAlignment(QtCore.Qt.AlignRight)

        self.labRotationParam = QtWidgets.QLabel(self)
        self.labRotationX = QtWidgets.QLabel(self)
        self.labRotationY = QtWidgets.QLabel(self)
        self.labRotationZ = QtWidgets.QLabel(self)
        self.labTranslationParam = QtWidgets.QLabel(self)
        self.labTranslationX = QtWidgets.QLabel(self)
        self.labTranslationY = QtWidgets.QLabel(self)
        self.labTranslationZ = QtWidgets.QLabel(self)

        self.labRotationParam.setAlignment(QtCore.Qt.AlignLeft)
        self.labRotationX.setAlignment(QtCore.Qt.AlignRight)
        self.labRotationY.setAlignment(QtCore.Qt.AlignRight)
        self.labRotationZ.setAlignment(QtCore.Qt.AlignRight)
        self.labTranslationParam.setAlignment(QtCore.Qt.AlignLeft)
        self.labTranslationX.setAlignment(QtCore.Qt.AlignRight)
        self.labTranslationY.setAlignment(QtCore.Qt.AlignRight)
        self.labTranslationZ.setAlignment(QtCore.Qt.AlignRight)

        self.spbRotationX = QDoubleSpinBox(self)
        self.spbRotationY = QDoubleSpinBox(self)
        self.spbRotationZ = QDoubleSpinBox(self)
        self.spbTranslationX = QDoubleSpinBox(self)
        self.spbTranslationY = QDoubleSpinBox(self)
        self.spbTranslationZ = QDoubleSpinBox(self)

        fRotationVector = self.stLidarParam.getRotationVector(self.nSelectBaseStation)
        fTranslationVector = self.stLidarParam.getTranslationVector(self.nSelectBaseStation)

        self.spbRotationX.setRange(-90, 90)
        self.spbRotationX.setSingleStep(0.1)
        self.spbRotationX.setValue(fRotationVector[0])

        self.spbRotationY.setRange(-90, 90)
        self.spbRotationY.setSingleStep(0.1)
        self.spbRotationY.setValue(fRotationVector[1])

        self.spbRotationZ.setRange(-360, 360)
        self.spbRotationZ.setSingleStep(0.1)
        self.spbRotationZ.setValue(fRotationVector[2])

        self.spbTranslationX.setRange(-600, 600)
        self.spbTranslationX.setSingleStep(0.1)
        self.spbTranslationX.setValue(fTranslationVector[0])

        self.spbTranslationY.setRange(-600, 600)
        self.spbTranslationY.setSingleStep(0.1)
        self.spbTranslationY.setValue(fTranslationVector[1])

        self.spbTranslationZ.setRange(-100, 100)
        self.spbTranslationZ.setSingleStep(0.1)
        self.spbTranslationZ.setValue(fTranslationVector[2])

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

        self.gridLayout.addWidget(self.checkHighSpeed, 0, 0, 1, 4)
        self.gridLayout.addWidget(self.labBaseStationId, 1, 0, 1, 2)
        self.gridLayout.addWidget(self.spbBaseStationId, 1, 3, 1, 1)
        self.gridLayout.addWidget(self.btnUpdate, 1, 5, 1, 1)
        self.gridLayout.addWidget(self.labBaseStationIp, 1, 6, 1, 2)
        self.gridLayout.addWidget(self.labRotationParam, 2, 0, 1, 2)
        self.gridLayout.addWidget(self.labRotationX, 2, 2, 1, 1)
        self.gridLayout.addWidget(self.spbRotationX, 2, 3, 1, 1)
        self.gridLayout.addWidget(self.labRotationY, 2, 4, 1, 1)
        self.gridLayout.addWidget(self.spbRotationY, 2, 5, 1, 1)
        self.gridLayout.addWidget(self.labRotationZ, 2, 6, 1, 1)
        self.gridLayout.addWidget(self.spbRotationZ, 2, 7, 1, 1)
        self.gridLayout.addWidget(self.labTranslationParam, 3, 0, 1, 2)
        self.gridLayout.addWidget(self.labTranslationX, 3, 2, 1, 1)
        self.gridLayout.addWidget(self.spbTranslationX, 3, 3, 1, 1)
        self.gridLayout.addWidget(self.labTranslationY, 3, 4, 1, 1)
        self.gridLayout.addWidget(self.spbTranslationY, 3, 5, 1, 1)
        self.gridLayout.addWidget(self.labTranslationZ, 3, 6, 1, 1)
        self.gridLayout.addWidget(self.spbTranslationZ, 3, 7, 1, 1)

        self.gridLayout.addLayout(self.btnlayout, 4, 0, 1, 8)

        self.retranslateUi()
        self.connectSlots()
        QtCore.QMetaObject.connectSlotsByName(self)

    def retranslateUi(self):
        _translate = QtCore.QCoreApplication.translate
        self.setWindowTitle(_translate("Dialog", "Settings"))
        self.checkHighSpeed.setText(_translate("Dialog", "HighSpeed"))
        # self.checkBoxBgdFrameFilter.setText(_translate("WJ-DepthViewer", "Bgd Frame Filter"))
        # self.checkBoxLiveUpdateBgdFrame.setText(_translate("WJ-DepthViewer", "Live Update Bgd Frame"))
        # self.checkBoxVoxelPoints.setText(_translate("WJ-DepthViewer", "Voxel Points"))
        #
        # self.checkBoxRotationTranslation.setText(_translate("WJ-DepthViewer", "Rotation & Translation"))
        self.labBaseStationId.setText(_translate("Dialog", "BaseStationID"))
        self.labRotationParam.setText(_translate("Dialog", "BaseStationR"))
        self.labBaseStationIp.setText(self.strIP)
        self.labRotationX.setText(_translate("Dialog", "X"))
        self.labRotationY.setText(_translate("Dialog", "Y"))
        self.labRotationZ.setText(_translate("Dialog", "Z"))

        self.labTranslationParam.setText(_translate("Dialog", "BaseStationT"))
        self.labTranslationX.setText(_translate("Dialog", "X"))
        self.labTranslationY.setText(_translate("Dialog", "Y"))
        self.labTranslationZ.setText(_translate("Dialog", "Z"))
        self.btnUpdate.setText(_translate("Dialog", "Update"))
        self.btnOK.setText(_translate("Dialog", "OK"))
        self.btnCacel.setText(_translate("Dialog", "Cancel"))

    def connectSlots(self):
        # self.checkBoxRotationTranslation.stateChanged.connect(self.slotRotationTranslationChanged)
        self.btnUpdate.clicked.connect(self.slotParamUpdata)
        self.btnOK.clicked.connect(self.slotAccept)
        self.btnCacel.clicked.connect(self.slotReject)

    def slotAccept(self):
        nState = self.checkHighSpeed.checkState()
        if nState == QtCore.Qt.Unchecked:
            self.stLidarParam.setHighSpeed(False)
        else:
            self.stLidarParam.setHighSpeed(True)
        # nState = self.checkBoxBgdFrameFilter.checkState()
        # if nState == QtCore.Qt.Unchecked:
        #     self.stLidarParam.setUseBgdFrameFilter(False)
        # else:
        #     self.stLidarParam.setUseBgdFrameFilter(True)
        #
        # nState = self.checkBoxLiveUpdateBgdFrame.checkState()
        # if nState == QtCore.Qt.Unchecked:
        #     self.stLidarParam.setLiveUpdateBgdFrame(False)
        # else:
        #     self.stLidarParam.setLiveUpdateBgdFrame(True)
        #
        # nState = self.checkBoxVoxelPoints.checkState()
        # if nState == QtCore.Qt.Unchecked:
        #     self.stLidarParam.setUseVoxelPoints(False)
        # else:
        #     self.stLidarParam.setUseVoxelPoints(True)
        #
        # nState = self.checkBoxRotationTranslation.checkState()
        # if nState == QtCore.Qt.Unchecked:
        #     self.stLidarParam.setRotationTranslation(False)
        # else:
        #     self.stLidarParam.setRotationTranslation(True)

        # self.stLidarParam.setLaneRotationVector(np.array([self.spbLaneRotationX1.value(), self.spbLaneRotationY1.value(), self.spbLaneRotationZ1.value()]))
        # self.stLidarParam.setLaneTranslationVector(np.array([self.spbLaneTranslationX1.value(), self.spbLaneTranslationY1.value(), self.spbLaneTranslationZ1.value()]))
        self.stLidarParam.setRotationVector(np.array([self.spbRotationX.value(), self.spbRotationY.value(), self.spbRotationZ.value()]), self.nSelectBaseStation)
        self.stLidarParam.setTranslationVector(np.array([self.spbTranslationX.value(), self.spbTranslationY.value(), self.spbTranslationZ.value()]), self.nSelectBaseStation)
        serializationParam(self.stLidarParam.getParam(), bSave=True)
        self.accept()

    def slotReject(self):
        self.reject()

    def slotRotationTranslationChanged(self, nStatus):
        if nStatus == QtCore.Qt.Unchecked:
            self.spbRotationX.setEnabled(False)
            self.spbRotationY.setEnabled(False)
            self.spbRotationZ.setEnabled(False)
            self.spbTranslationX.setEnabled(False)
            self.spbTranslationY.setEnabled(False)
            self.spbTranslationZ.setEnabled(False)
        else:
            self.spbRotationX.setEnabled(True)
            self.spbRotationY.setEnabled(True)
            self.spbRotationZ.setEnabled(True)
            self.spbTranslationX.setEnabled(True)
            self.spbTranslationY.setEnabled(True)
            self.spbTranslationZ.setEnabled(True)

    def slotParamUpdata(self):
        self.stLidarParam.setBaseStationID(int(self.spbBaseStationId.value()))
        self.nSelectBaseStation = self.stLidarParam.getBaseStationID()
        self.strIP = self.stLidarParam.getlistAttachedIp()[self.nSelectBaseStation - 1]
        self.labBaseStationIp.setText(self.strIP)
        self.spbBaseStationId.setValue(int(self.nSelectBaseStation))
        self.nSelectBaseStation -= 1
        fRotationVector = self.stLidarParam.getRotationVector(self.nSelectBaseStation)
        fTranslationVector = self.stLidarParam.getTranslationVector(self.nSelectBaseStation)
        self.spbRotationX.setValue(fRotationVector[0])
        self.spbRotationY.setValue(fRotationVector[1])
        self.spbRotationZ.setValue(fRotationVector[2])
        self.spbTranslationX.setValue(fTranslationVector[0])
        self.spbTranslationY.setValue(fTranslationVector[1])
        self.spbTranslationZ.setValue(fTranslationVector[2])