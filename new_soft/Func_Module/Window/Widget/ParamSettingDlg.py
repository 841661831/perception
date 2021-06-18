# -*- coding: utf-8 -*-
# PyQt的导入必须放在前面，否则release运行时会崩溃！
from WinCommon import *

class ParamLayoutPart:
    def __init__(self, parent, strEventName="默认", bLimitVal=False):
        self.bLimitVal = bLimitVal
        self.parent = parent
        self.labEventName = QtWidgets.QLabel(self.parent)
        self.labEventName.setAlignment(QtCore.Qt.AlignVCenter)

        self.checkEnable = QtWidgets.QCheckBox()
        self.checkEnable.setObjectName("Enable")
        self.checkEnable.setChecked(1)
        self.checkEnable.stateChanged.connect(self.checkEnableChanged)

        self.labResponseTime = QtWidgets.QLabel(self.parent)
        self.labResponseTime.setAlignment(QtCore.Qt.AlignVCenter)
        self.spbResponseTime = QDoubleSpinBox(self.parent)
        self.spbResponseTime.setRange(0, 100)
        self.spbResponseTime.setSingleStep(0.1)

        self.labLimitVal = QtWidgets.QLabel(self.parent)
        self.labLimitVal.setAlignment(QtCore.Qt.AlignVCenter)
        self.spbLimitVal = QDoubleSpinBox(self.parent)
        self.spbLimitVal.setRange(0, 120)
        self.spbLimitVal.setSingleStep(0.1)

        self.labEventName.setText(strEventName)
        self.labResponseTime.setText("响应时间")
        self.labLimitVal.setText("阈值")
        self.labLimitVal.setVisible(self.bLimitVal)
        self.spbLimitVal.setVisible(self.bLimitVal)

    def setPosition(self, row, col):
        self.parent.gridLayout.addWidget(self.labEventName, row, 4 * (col - 1) + 0, 1, 1)
        self.parent.gridLayout.addWidget(self.checkEnable, row, 4 * (col - 1) + 1, 1, 1)
        self.parent.gridLayout.addWidget(self.labResponseTime, row, 4 * (col - 1) + 2, 1, 1)
        self.parent.gridLayout.addWidget(self.spbResponseTime, row, 4 * (col - 1) + 3, 1, 1)
        if self.bLimitVal:
            self.parent.gridLayout.addWidget(self.labLimitVal, row, 4 * (col - 1) + 6, 1, 2)
            self.parent.gridLayout.addWidget(self.spbLimitVal, row, 4 * (col - 1) + 7, 1, 2)

    def setVal(self, listVal):
        self.checkEnable.setChecked(listVal[0])
        self.spbResponseTime.setValue(listVal[1])
        self.spbLimitVal.setValue(listVal[2])

    def getVal(self):
        if self.checkEnable.checkState() == QtCore.Qt.Unchecked:
            bCheckState = False
        else:
            bCheckState = True
        if self.bLimitVal:
            return bCheckState, float(self.spbResponseTime.value()), float(self.spbLimitVal.value())
        else:
            return bCheckState, float(self.spbResponseTime.value())

    def checkEnableChanged(self, nStatus):
        if nStatus == QtCore.Qt.Unchecked:
            self.spbResponseTime.setEnabled(False)
            self.spbLimitVal.setEnabled(False)
        else:
            self.spbResponseTime.setEnabled(True)
            self.spbLimitVal.setEnabled(True)


class ParamSettingDlg(QDialog):
    sigParamChanged = pyqtSignal()

    def init(self, stSysState, stStationParam, stEventParam, stTrackParam, parent=None):
        if parent is not None:
            self.setParent(parent)

        self.setObjectName("ParamSettingDlg")
        self.resize(300, 300)
        self.stSysState = stSysState
        self.stStationParam = stStationParam
        self.stEventParam = stEventParam
        self.stTrackParam = stTrackParam

        self.gridLayout = QtWidgets.QGridLayout(self)
        self.gridLayout.setContentsMargins(15, 15, 15, 15)
        self.gridLayout.setSpacing(10)
        self.gridLayout.setObjectName("gridLayout")

        self.labEventParam = QtWidgets.QLabel(self)
        self.gridLayout.addWidget(self.labEventParam, 0, 0, 1, 4)

        self.occupy_dedicated_lane = ParamLayoutPart(self, strEventName="占用专用车道", bLimitVal=False)
        self.occupy_dedicated_lane.setPosition(1, 1)
        self.occupy_dedicated_lane.setVal(self.stEventParam.occupy_dedicated_lane.getVal())
        self.people_occupy_motor_lane = ParamLayoutPart(self, strEventName="行人进入隧道", bLimitVal=False)
        self.people_occupy_motor_lane.setPosition(1, 2)
        self.people_occupy_motor_lane.setVal(self.stEventParam.people_occupy_motor_lane.getVal())
        self.retorgrade = ParamLayoutPart(self, strEventName="逆行", bLimitVal=False)
        self.retorgrade.setPosition(2, 1)
        self.retorgrade.setVal(self.stEventParam.retorgrade.getVal())
        self.cross_line = ParamLayoutPart(self, strEventName="压线", bLimitVal=False)
        self.cross_line.setPosition(2, 2)
        self.cross_line.setVal(self.stEventParam.cross_line.getVal())
        self.illegal_stop = ParamLayoutPart(self, strEventName="违停", bLimitVal=False)
        self.illegal_stop.setPosition(3, 1)
        self.illegal_stop.setVal(self.stEventParam.illegal_stop.getVal())
        self.cross_lane = ParamLayoutPart(self, strEventName="横穿马路", bLimitVal=False)
        self.cross_lane.setPosition(3, 2)
        self.cross_lane.setVal(self.stEventParam.cross_lane.getVal())
        self.run_the_red_light = ParamLayoutPart(self, strEventName="闯红灯", bLimitVal=False)
        self.run_the_red_light.setPosition(4, 1)
        self.run_the_red_light.setVal(self.stEventParam.run_the_red_light.getVal())
        self.occupy_bus_lane = ParamLayoutPart(self, strEventName="占用公交车道", bLimitVal=False)
        self.occupy_bus_lane.setPosition(4, 2)
        self.occupy_bus_lane.setVal(self.stEventParam.occupy_bus_lane.getVal())
        self.changes_lane = ParamLayoutPart(self, strEventName="变道", bLimitVal=False)
        self.changes_lane.setPosition(5, 1)
        self.changes_lane.setVal(self.stEventParam.changes_lane.getVal())
        self.spills_detect = ParamLayoutPart(self, strEventName="抛撒物", bLimitVal=False)
        self.spills_detect.setPosition(5, 2)
        self.spills_detect.setVal(self.stEventParam.spills_detect.getVal())
        self.accident_detect = ParamLayoutPart(self, strEventName="事故检测", bLimitVal=False)
        self.accident_detect.setPosition(6, 1)
        self.accident_detect.setVal(self.stEventParam.accident_detect.getVal())
        self.occupy_emergency_area = ParamLayoutPart(self, strEventName="占用应急车道", bLimitVal=False)
        self.occupy_emergency_area.setPosition(6, 2)
        self.occupy_emergency_area.setVal(self.stEventParam.occupy_emergency_area.getVal())
        self.speeding = ParamLayoutPart(self, strEventName="超速", bLimitVal=True)
        self.speeding.setPosition(7, 1)
        self.speeding.setVal(self.stEventParam.speeding.getVal())
        self.stroll = ParamLayoutPart(self, strEventName="慢行", bLimitVal=True)
        self.stroll.setPosition(8, 1)
        self.stroll.setVal(self.stEventParam.stroll.getVal())
        # 数据读取参数
        self.labReadParam = QtWidgets.QLabel(self)
        self.gridLayout.addWidget(self.labReadParam, 9, 0, 1, 4)

        self.checkHighspeed = QtWidgets.QCheckBox()
        self.checkHighspeed.setObjectName("Highspeed")
        self.checkHighspeed.setChecked(self.stStationParam.getHighSpeed())

        self.checkTimeMatch = QtWidgets.QCheckBox()
        self.checkTimeMatch.setObjectName("TimeMatch")
        self.checkTimeMatch.setChecked(self.stStationParam.getTimeMatch())

        self.gridLayout.addWidget(self.checkHighspeed, 10, 0, 1, 2)
        self.gridLayout.addWidget(self.checkTimeMatch, 10, 4, 1, 2)
        # 跟踪算法参数
        self.labTrackParam = QtWidgets.QLabel(self)
        self.gridLayout.addWidget(self.labTrackParam, 11, 0, 1, 4)

        self.labHorizon_threshoud = QtWidgets.QLabel(self)
        self.labHorizon_threshoud.setAlignment(QtCore.Qt.AlignVCenter)
        self.spbHorizon_threshoud = QDoubleSpinBox(self)
        self.spbHorizon_threshoud.setRange(0, 120)
        self.spbHorizon_threshoud.setSingleStep(0.1)
        self.spbHorizon_threshoud.setValue(self.stTrackParam.getHorizon_threshoud())
        self.gridLayout.addWidget(self.labHorizon_threshoud, 12, 0, 1, 3)
        self.gridLayout.addWidget(self.spbHorizon_threshoud, 12, 3, 1, 1)

        self.labVertical_threshoud = QtWidgets.QLabel(self)
        self.labVertical_threshoud.setAlignment(QtCore.Qt.AlignVCenter)
        self.spbVertical_threshoud = QDoubleSpinBox(self)
        self.spbVertical_threshoud.setRange(0, 120)
        self.spbVertical_threshoud.setSingleStep(0.1)
        self.spbVertical_threshoud.setValue(self.stTrackParam.getVertical_threshoud())
        self.gridLayout.addWidget(self.labVertical_threshoud, 12, 4, 1, 3)
        self.gridLayout.addWidget(self.spbVertical_threshoud, 12, 7, 1, 1)

        self.labDisplayFrames = QtWidgets.QLabel(self)
        self.labDisplayFrames.setAlignment(QtCore.Qt.AlignVCenter)
        self.spbDisplayFrames = QDoubleSpinBox(self)
        self.spbDisplayFrames.setDecimals(0)
        self.spbDisplayFrames.setRange(0, 100)
        self.spbDisplayFrames.setSingleStep(1)
        self.spbDisplayFrames.setValue(self.stTrackParam.getDisplayFrames())
        self.gridLayout.addWidget(self.labDisplayFrames, 13, 0, 1, 3)
        self.gridLayout.addWidget(self.spbDisplayFrames, 13, 3, 1, 1)

        self.labMaxLifetime = QtWidgets.QLabel(self)
        self.labMaxLifetime.setAlignment(QtCore.Qt.AlignVCenter)
        self.spbMaxLifetime = QDoubleSpinBox(self)
        self.spbMaxLifetime.setDecimals(0)
        self.spbMaxLifetime.setRange(0, 100)
        self.spbMaxLifetime.setSingleStep(1)
        self.spbMaxLifetime.setValue(self.stTrackParam.getMaxLifetime())
        self.gridLayout.addWidget(self.labMaxLifetime, 13, 4, 1, 3)
        self.gridLayout.addWidget(self.spbMaxLifetime, 13, 7, 1, 1)
        # 控制按钮
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

        self.gridLayout.addLayout(self.btnlayout, 14, 0, 1, 8)

        self.retranslateUi()
        self.connectSlots()
        QtCore.QMetaObject.connectSlotsByName(self)

    def retranslateUi(self):
        self.setWindowTitle("参数设置")
        self.labEventParam.setText("事件检测算法参数")
        self.labReadParam.setText("数据读取参数")
        self.checkHighspeed.setText("高速/隧道模式")
        self.checkTimeMatch.setText("时间匹配模式")
        self.labTrackParam.setText("跟踪算法参数")
        self.labDisplayFrames.setText("显示帧数")
        self.labMaxLifetime.setText("最大寿命")
        self.labHorizon_threshoud.setText("横向距离阈值")
        self.labVertical_threshoud.setText("纵向距离阈值")
        self.btnOK.setText("确定")
        self.btnCacel.setText("取消")

    def connectSlots(self):
        self.btnOK.clicked.connect(self.slotAccept)
        self.btnCacel.clicked.connect(self.slotReject)

    def slotAccept(self):
        self.slotEventAccept()
        self.slotStationAccept()
        self.slotTrackAccept()
        self.stSysState.set_update_param(True)
        time.sleep(0.3)
        self.stSysState.set_update_param(False)
        self.accept()

    def slotEventAccept(self):
        self.stEventParam.occupy_dedicated_lane.setVal(self.occupy_dedicated_lane.getVal())
        self.stEventParam.people_occupy_motor_lane.setVal(self.people_occupy_motor_lane.getVal())
        self.stEventParam.retorgrade.setVal(self.retorgrade.getVal())
        self.stEventParam.speeding.setVal(self.speeding.getVal())
        self.stEventParam.stroll.setVal(self.stroll.getVal())
        self.stEventParam.cross_line.setVal(self.cross_line.getVal())
        self.stEventParam.illegal_stop.setVal(self.illegal_stop.getVal())
        self.stEventParam.cross_lane.setVal(self.cross_lane.getVal())
        self.stEventParam.run_the_red_light.setVal(self.run_the_red_light.getVal())
        self.stEventParam.occupy_bus_lane.setVal(self.occupy_bus_lane.getVal())
        self.stEventParam.changes_lane.setVal(self.changes_lane.getVal())
        self.stEventParam.spills_detect.setVal(self.spills_detect.getVal())
        self.stEventParam.accident_detect.setVal(self.accident_detect.getVal())
        self.stEventParam.occupy_emergency_area.setVal(self.occupy_emergency_area.getVal())
        self.stEventParam.saveParam()

    def slotStationAccept(self):
        if self.checkHighspeed.checkState() == QtCore.Qt.Unchecked:
            self.stStationParam.setHighSpeed(False)
        else:
            self.stStationParam.setHighSpeed(True)
        if self.checkTimeMatch.checkState() == QtCore.Qt.Unchecked:
            self.stStationParam.setTimeMatch(False)
        else:
            self.stStationParam.setTimeMatch(True)
        self.stStationParam.saveParam()

    def slotTrackAccept(self):
        self.stTrackParam.setDisplayFrames(int(self.spbDisplayFrames.value()))
        self.stTrackParam.setHorizon_threshoud(float(self.spbHorizon_threshoud.value()))
        self.stTrackParam.setVertical_threshoud(float(self.spbVertical_threshoud.value()))
        self.stTrackParam.setMaxLifetime(int(self.spbMaxLifetime.value()))
        self.stTrackParam.saveParam()

    def slotReject(self):
        self.reject()