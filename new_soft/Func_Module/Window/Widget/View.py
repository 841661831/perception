# -*- coding: utf-8 -*-
# PyQt的导入必须放在前面，否则release运行时会崩溃！
from WinCommon import *
from Widget.PcPyGLWidget import PcPyGLWidget
from Widget.SettingDlg import SettingDlg
from Widget.ParamSettingDlg import ParamSettingDlg
from Widget.ActionTipDlg import ActionTipDlg
from Widget.ComboCheckBox import ComboCheckBox

TIME_INTERVAL = stUiParam.nTimeInterval  # 单位ms
PACK_VERSION_DATE = "v1.0_20210530"
PACK_VERSION_DATE_DEBUG = "v1.0_20210530"


class MainWindow(QMainWindow):
    sigQuit = pyqtSignal()
    sigSelectBox = pyqtSignal(int)

    def init(self, s_cfg_path):
        self.Mylog = initLog(s_cfg_path)
        self.setObjectName("WJ-GlobalTrackViewer")
        self.setContextMenuPolicy(QtCore.Qt.NoContextMenu)
        self.data_bus_server = DataBus(strHost)
        self.sub_winSysState = self.data_bus_server.get_subscriber(['winSys_state'])[0]
        self.list_channel = get_channel_param(s_cfg_path)
        self.list_subChannel = self.data_bus_server.get_subscriber(self.list_channel)
        self.resize(820, 656)
        self.bPause = True
        self.bAuto = True
        self.bOnline = False
        self.stDstData = None
        self.strSaveBoxInfoFilePath = ""
        self.listBoxInfo = []
        self.msgBox = None
        # 系统状态全局变量
        self.m_manager = BaseManager()
        self.m_manager.register('structSysState', structSysState)
        self.m_manager.start()
        self.stSysState = self.m_manager.structSysState()
        # 状态切换窗口
        self.settingDlg = SettingDlg()
        self.settingDlg.init(self.stSysState)

        # 参数设置窗口
        self.stStationParam = lidarParam(os.path.join(get_cfg_path(), 'modParam/read_Param.xml'))
        self.stStationParam.updateParam()
        self.stEventParam = structEventAlgParam(os.path.join(get_cfg_path(), 'modParam/event_Param.xml'))
        self.stEventParam.updateParam()
        self.stTrackParam = structTrackParam(os.path.join(get_cfg_path(), 'modParam/track_Param.xml'))
        self.stTrackParam.updateParam()
        self.paramSettingDlg = ParamSettingDlg()
        self.paramSettingDlg.init(self.stSysState, self.stStationParam, self.stEventParam, self.stTrackParam)
        # 计算帧率
        self.timeFps = QTime()
        self.bFirstTest = True
        self.nLastTime = 0
        self.nLastFrameId = 0
        self.fFps = 10
        # 计算帧率
        self.timerUpdate = QTimer()
        self.timerUpdate.setInterval(TIME_INTERVAL)
        self.timerCheckAuthor = QTimer()
        self.timerCheckAuthor.setInterval(60 * 1000)  # 300 * 1000
        self.timerCheckAuthor.start()
        self.timerSurvivalCheck = QTimer()
        self.timerSurvivalCheck.setInterval(10000)
        self.timerSurvivalCheck.start()
        self.updateSysState = QTimer()
        self.updateSysState.setInterval(1000)
        self.updateSysState.start()

        self.timeNoData = QTime()
        self.dataTrackQueue = Queue()
        self.dataEventQueue = Queue()
        self.bFirstNoData = True  # 用于控制在没有任何数据时，刷新一次显示区域，使显示区域变成黑屏
        self.nSelectBoxId = -1

        self.stUiParam = structUiParam()
        self.stUiParam.update(s_cfg_path)
        self.listVBO = get_lane_vbo(self.stStationParam, self.stUiParam)

        # Create QAction
        self.actionOpen = QtWidgets.QAction(self)
        self.actionOpen.setObjectName("actionOpen")
        self.actionSetting = QtWidgets.QAction(self)
        self.actionSetting.setObjectName("actionSetting")
        self.actionQuit = QtWidgets.QAction(self)
        self.actionQuit.setObjectName("actionQuit")

        self.actionModeSwitch = QtWidgets.QAction(self)
        self.actionModeSwitch.setObjectName("actionModeSwitch")

        self.actionPlayPause = QtWidgets.QAction(self)
        self.actionPlayPause.setObjectName("actionPlayPause")
        self.actionStop = QtWidgets.QAction(self)
        self.actionStop.setObjectName("actionStop")
        self.actionRestart = QtWidgets.QAction(self)
        self.actionRestart.setObjectName("actionRestart")
        self.actionPrevious = QtWidgets.QAction(self)
        self.actionPrevious.setObjectName("actionPrevious")
        self.actionNext = QtWidgets.QAction(self)
        self.actionNext.setObjectName("actionNext")

        stretch = QtWidgets.QLabel()
        stretch.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)
        # self.actionTimeLeft = QtWidgets.QAction(self)
        self.actionTimeLeft = QtWidgets.QPushButton(self)
        self.actionTimeLeft.setObjectName("actionTimeLeft")
        self.actionTimeLeft.setVisible(True)

        item = QtWidgets.QLabel()
        item.setFixedSize(5, 1)
        self.checkBoxLoop = QtWidgets.QCheckBox()
        self.checkBoxLoop.setObjectName("checkBoxLoop")

        item2 = QtWidgets.QLabel()
        item2.setFixedSize(5, 1)
        self.sliderFrameId = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.sliderFrameId.setObjectName("sliderFrameId")
        self.sliderFrameId.setMaximumWidth(100)
        self.sliderFrameId.setValue(0)
        self.spFrameId = QtWidgets.QSpinBox()
        self.spFrameId.setObjectName("spFrameId")
        self.spFrameId.setValue(0)
        self.sliderFrameId.setEnabled(True)
        self.spFrameId.setEnabled(True)

        # 旋转角度输入框
        self.labelRotationAngle = QtWidgets.QLabel()
        self.labelRotationAngle.setText("°")
        self.editRotationAngle = QtWidgets.QLineEdit()
        self.editRotationAngle.setObjectName("editRotationAngle")
        self.editRotationAngle.setMaximumWidth(70)
        self.editRotationAngle.setPlaceholderText("视角旋转")
        self.editRotationAngle.setValidator(QDoubleValidator(-180.0, 180.0, 2))

        # QToolBar
        self.mainToolBar = QtWidgets.QToolBar(self)
        self.mainToolBar.setObjectName("mainToolBar")
        self.mainToolBar.addAction(self.actionOpen)
        self.mainToolBar.addAction(self.actionPrevious)
        self.mainToolBar.addAction(self.actionPlayPause)
        self.mainToolBar.addAction(self.actionNext)
        self.mainToolBar.addAction(self.actionStop)
        self.mainToolBar.addAction(self.actionRestart)
        self.mainToolBar.addAction(self.actionModeSwitch)

        self.mainToolBar.addWidget(self.editRotationAngle)
        self.mainToolBar.addWidget(self.labelRotationAngle)
        self.mainToolBar.addWidget(item)
        self.mainToolBar.addWidget(self.checkBoxLoop)
        # self.mainToolBar.addWidget(self.sliderFrameId)
        self.mainToolBar.addWidget(self.spFrameId)
        # self.checkBoxLoop.setVisible(False)
        # self.sliderFrameId.setVisible(False)
        # self.spFrameId.setVisible(False)
        self.mainToolBar.addWidget(stretch)
        # self.mainToolBar.addAction(self.actionTimeLeft)
        self.mainToolBar.addWidget(self.actionTimeLeft)
        self.addToolBar(QtCore.Qt.TopToolBarArea, self.mainToolBar)

        # QStatusBar
        self.statusbar = QtWidgets.QStatusBar(self)
        self.statusbar.setObjectName("statusbar")
        # self.statusbar.setStyleSheet("QtWidgets.QStatusBar.item{border: 3px}")
        self.labVersion = QLabel()
        self.statusbar.addPermanentWidget(self.labVersion, stretch=0)
        self.setStatusBar(self.statusbar)

        # create centralwidget
        self.centralwidget = QtWidgets.QWidget(self)
        self.centralwidget.setObjectName("centralwidget")
        self.mainLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.mainLayout.setContentsMargins(15, 15, 15, 15)
        self.mainLayout.setSpacing(10)
        self.mainLayout.setObjectName("mainLayout")
        self.centralwidget.setLayout(self.mainLayout)

        # create centralwidget Layout
        self.mainSplitter = QtWidgets.QSplitter(self.centralwidget)
        self.mainSplitter.setOrientation(QtCore.Qt.Horizontal)
        self.subSplitterLeft = QtWidgets.QSplitter(self.mainSplitter)
        self.subSplitterLeft.setOrientation(QtCore.Qt.Vertical)
        self.subSplitterRight = QtWidgets.QSplitter(self.mainSplitter)
        self.subSplitterRight.setOrientation(QtCore.Qt.Vertical)
        self.mainLayout.addWidget(self.mainSplitter)

        # ============================ opengl widget ===========================
        # Title Label
        self.titlePc = QtWidgets.QLabel()
        # self.titlePc.setStyleSheet("QLabel{background-color:rgb(87,96,95);border:0px;}")
        self.titlePc.setFixedHeight(25)
        self.openGLWidgetTitle = QtWidgets.QLabel()
        self.openGLWidgetTitle.setStyleSheet("border:0px;")
        spacerItem = QtWidgets.QSpacerItem(0, 0, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.checkBoxShowPcBox = QtWidgets.QCheckBox()
        self.checkBoxShowPcBox.setObjectName("checkBoxShowPcBox")
        self.checkBoxShowPcBox.setChecked(True)
        self.checkBoxShowPcBox.setStyleSheet("border:0px;")
        self.checkBoxShowLane = QtWidgets.QCheckBox()
        self.checkBoxShowLane.setObjectName("checkBoxShowPcBox")
        self.checkBoxShowLane.setChecked(True)
        self.checkBoxShowLane.setStyleSheet("border:0px;")
        self.checkBoxShowTable = QtWidgets.QCheckBox()
        self.checkBoxShowTable.setObjectName("checkBoxShowRLFusion")
        self.checkBoxShowTable.setChecked(False)
        self.checkBoxShowTable.setStyleSheet("border:0px;")
        self.btnPcSetting = QtWidgets.QPushButton(self)
        self.btnPcSetting.setObjectName("btnPcSetting")
        self.btnPcSetting.setMinimumSize(21, 21)
        self.btnPcSetting.setStyleSheet("border:0px;")

        self.hBoxLayout = QtWidgets.QHBoxLayout(self.titlePc)
        self.hBoxLayout.addWidget(self.openGLWidgetTitle)
        self.hBoxLayout.addItem(spacerItem)
        self.hBoxLayout.addWidget(self.checkBoxShowPcBox)
        self.hBoxLayout.addWidget(self.checkBoxShowLane)
        self.hBoxLayout.addWidget(self.checkBoxShowTable)
        self.hBoxLayout.addWidget(self.btnPcSetting)
        self.hBoxLayout.setContentsMargins(5, 2, 5, 2)
        self.hBoxLayout.setSpacing(5)

        # GLWidget
        self.openGLWidget = PcPyGLWidget(self.Mylog, self)
        self.openGLWidget.setObjectName("openGLWidget")
        self.openGLWidget.setMinimumHeight(10)
        self.openGLWidget.setSyncParam(self.dataTrackQueue, self.dataEventQueue)

        # GlWidget Frame
        self.frame = QtWidgets.QFrame()
        self.frame.setFrameShape(QtWidgets.QFrame.Box)
        self.frame.setStyleSheet("border: 1px solid gray;")
        vBoxLayout = QtWidgets.QVBoxLayout(self.frame)
        vBoxLayout.addWidget(self.titlePc)
        vBoxLayout.addWidget(self.openGLWidget)
        vBoxLayout.setContentsMargins(0, 0, 0, 0)
        vBoxLayout.setSpacing(0)
        self.subSplitterLeft.addWidget(self.frame)
        # ============================ opengl widget ===========================

        self.dictTableWidgetShowType = {}

        # QTableWidget Box Info
        # QTableWidget Box Info Title
        self.titleTableBoxType = ComboCheckBox(
            ["FrameId", "BoxId", "Length(m)",
             "Width(m)", "Height(m)",
             "Speed(m/s)", "Angle(deg)",
             "Class"])

        if self.stUiParam.bDebug:
            self.titleTableBoxType = ComboCheckBox(
                ["FrameId", "BoxId", "Length(m)",
                 "Width(m)", "Height(m)",
                 "Speed(m/s)", "Angle(deg)",
                 "Class", "Longitude(deg)",
                 "Latitude(deg)", "Source"])
        self.titleTableBoxType.setObjectName("titleTableBoxType")
        if (self.stUiParam.SoftwareStyle == 0):
            self.titleTableBoxType.setStyleSheet(
                "width: 90px; height: 15px; font-size: 12px;background-color:#515151;")  # background-color:#000000;515151

        self.titleTableBoxInfo = QtWidgets.QLabel()
        self.titleTableBoxInfo.setFixedHeight(25)
        self.titleTableBoxInfo.setStyleSheet("border:0px;")
        self.tableBoxInfoTitle = QtWidgets.QLabel()
        self.tableBoxInfoTitle.setStyleSheet("border:0px;")

        self.checkBoxSaveBoxInfo = QtWidgets.QCheckBox()
        self.checkBoxSaveBoxInfo.setObjectName("checkBoxSaveBoxInfo")
        self.checkBoxSaveBoxInfo.setChecked(False)
        self.checkBoxSaveBoxInfo.setStyleSheet("border:0px;")

        self.tablehlayout = QtWidgets.QHBoxLayout(self.titleTableBoxInfo)
        self.tablehlayout.addWidget(self.tableBoxInfoTitle)
        self.tablehlayout.addWidget(self.titleTableBoxType)
        self.tablehlayout.setEnabled(False)

        spacerItem = QtWidgets.QSpacerItem(0, 0, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.tablehlayout.addItem(spacerItem)

        self.tablehlayout.addWidget(self.checkBoxSaveBoxInfo)
        self.tablehlayout.setContentsMargins(5, 2, 5, 2)
        self.tablehlayout.setSpacing(5)

        # QTableWidget Box Info
        self.tableWidget = QtWidgets.QTableWidget(self.centralwidget)
        # self.tableWidget.horizontalHeader().setStretchLastSection(True)
        # self.tableWidget.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.tableWidget.setObjectName("tableWidget")
        # self.tableWidget.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.tableWidget.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        # self.tableWidget.setVisible(False)

        # QTableWidget Box Info Frame
        self.tableBoxInfoFrame = QtWidgets.QFrame()
        self.tableBoxInfoFrame.setFrameShape(QtWidgets.QFrame.Box)
        self.tableBoxInfoFrame.setStyleSheet("border: 1px solid gray;")
        vBoxLayouttableBoxInfo = QtWidgets.QVBoxLayout(self.tableBoxInfoFrame)
        vBoxLayouttableBoxInfo.addWidget(self.titleTableBoxInfo)
        vBoxLayouttableBoxInfo.addWidget(self.tableWidget)
        vBoxLayouttableBoxInfo.setContentsMargins(0, 0, 0, 0)
        vBoxLayouttableBoxInfo.setSpacing(0)

        self.subSplitterRight.setVisible(False)
        self.mainSplitter.addWidget(self.subSplitterLeft)
        self.mainSplitter.addWidget(self.subSplitterRight)
        self.mainSplitter.setStretchFactor(0, 5)  # 设置分割界面的初始比例4/5
        self.mainSplitter.setStretchFactor(1, 1)  # 设置分割界面的初始比例1/5

        # Set UI Text
        self.retranslateUi()
        self.setIcon()
        self.setActionEnable(False)
        self.setCentralWidget(self.centralwidget)
        # connect slots
        self.connectSlots()

        QtCore.QMetaObject.connectSlotsByName(self)
        # self.openGLWidget.resize(QtCore.QSize(500, 500))
        self.stSysState.set_sys_state(4)
        self.slotPublishWinState()
        self.openGLWidget.setSysParam(self.stSysState, self.stStationParam)

    def __del__(self):
        self.slotStop()
        del self.openGLWidget

    # Set UI Text
    def retranslateUi(self):
        # self.setWindowTitle("WJ-ISFP")
        self.setWindowTitle("智慧基站-全域软件")

        # 设置水平方向的表头标签与垂直方向上的表头标签，注意必须在初始化行列之后进行，否则没有效果
        self.tableWidget.setColumnCount(8)
        if (self.stUiParam.SoftwareStyle == 0):
            self.tableWidget.verticalHeader().setHidden(True)
        self.tableWidget.setHorizontalHeaderLabels(
            ['FrameId', 'BoxId', 'Length(m)', 'Width(m)', 'Height(m)', 'Speed(m/s)', 'Angle(deg)', 'Class'])
        self.dictTableWidgetShowType = {'FrameId': 0, 'BoxId': 1, 'Length(m)': 2, 'Width(m)': 3, 'Height(m)': 4,
                                        'Speed(m/s)': 5, 'Angle(deg)': 6, 'Class': 7}
        if self.stUiParam.bDebug:
            self.tableWidget.setColumnCount(11)
            self.tableWidget.setHorizontalHeaderLabels(
                ['FrameId', 'BoxId', 'Length(m)', 'Width(m)', 'Height(m)', 'Speed(m/s)', 'Angle(deg)', 'Class',
                 'Longitude(deg)', 'Latitude(deg)', "Source"])
            self.dictTableWidgetShowType = {'FrameId': 0, 'BoxId': 1, 'Length(m)': 2, 'Width(m)': 3, 'Height(m)': 4,
                                            'Speed(m/s)': 5, 'Angle(deg)': 6, 'Class': 7, 'Longitude(deg)': 8,
                                            'Latitude(deg)': 9, "Source": 10}

        # self.tableWidget.horizontalHeader().setStretchLastSection(True)
        # self.tableWidget.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        self.tableWidget.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)

        self.actionOpen.setText("文件")
        self.actionSetting.setText("设置")
        self.actionQuit.setText("退出")
        self.actionModeSwitch.setText("在线切换")
        self.actionPlayPause.setText("运行")
        self.actionStop.setText("停止")
        self.actionRestart.setText("重新开始")
        self.actionPrevious.setText("上一帧")
        self.actionNext.setText("下一帧")
        if self.stUiParam.bDebug:
            strVersion = "{} Debug!".format(PACK_VERSION_DATE_DEBUG)
        else:
            strVersion = "{}".format(PACK_VERSION_DATE)

        self.labVersion.setText(strVersion)
        self.labVersion.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.checkBoxLoop.setText("循环")
        self.labelRotationAngle.setText("°")
        self.checkBoxShowPcBox.setText("显示目标")
        self.checkBoxShowLane.setText("显示车道")
        self.checkBoxShowTable.setText("显示表格")
        self.checkBoxSaveBoxInfo.setText("保存目标信息")
        self.titlePc.setText("全域数据")
        self.tableBoxInfoTitle.setText("目标信息")
        self.btnPcSetting.setToolTip("参数设置")

    # Set Icon
    def setIcon(self):
        style = QApplication.style()

        icon = style.standardIcon(QtWidgets.QStyle.SP_DirOpenIcon)
        self.actionOpen.setIcon(icon)

        img = QtGui.QImage(os.path.join(get_cfg_path(), "Icon/pause.png"))
        pixmap = QtGui.QPixmap(img)
        fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        icon = QIcon(fitPixmap)
        # icon = style.standardIcon(QtWidgets.QStyle.SP_MediaPlay)
        self.actionPlayPause.setIcon(icon)

        img = QtGui.QImage(os.path.join(get_cfg_path(), "Icon/stop.png"))
        pixmap = QtGui.QPixmap(img)
        fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        icon = QIcon(fitPixmap)
        # icon = style.standardIcon(QtWidgets.QStyle.SP_MediaStop)
        self.actionStop.setIcon(icon)

        icon = style.standardIcon(QtWidgets.QStyle.SP_BrowserReload)
        self.actionRestart.setIcon(icon)

        img = QtGui.QImage(os.path.join(get_cfg_path(), "Icon/pre.png"))
        pixmap = QtGui.QPixmap(img)
        fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        icon = QIcon(fitPixmap)
        self.actionPrevious.setIcon(icon)

        img = QtGui.QImage(os.path.join(get_cfg_path(), "Icon/next.png"))
        pixmap = QtGui.QPixmap(img)
        fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        icon = QIcon(fitPixmap)
        self.actionNext.setIcon(icon)

        icon = style.standardIcon(QtWidgets.QStyle.SP_DialogNoButton)
        self.actionModeSwitch.setIcon(icon)

        img = QtGui.QImage(os.path.join(get_cfg_path(), "Icon/pcSetting.png"))
        pixmap = QtGui.QPixmap(img)
        fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        icon = QIcon(fitPixmap)
        self.btnPcSetting.setIcon(icon)

    def setActionEnable(self, bEnabled):
        self.actionPrevious.setEnabled(bEnabled)
        self.actionPlayPause.setEnabled(bEnabled)
        self.actionNext.setEnabled(bEnabled)
        self.actionStop.setEnabled(bEnabled)
        self.actionRestart.setEnabled(bEnabled)

    def connectSlots(self):
        self.actionOpen.triggered.connect(self.slotOpenFile)
        self.actionSetting.triggered.connect(self.slotSetting)
        self.actionQuit.triggered.connect(self.slotQuit)

        self.actionModeSwitch.triggered.connect(self.slotModeSwitch)

        self.actionPlayPause.triggered.connect(self.slotPlayPause)
        self.actionStop.triggered.connect(self.slotStopPlay)
        self.actionRestart.triggered.connect(self.slotRestart)
        self.actionPrevious.triggered.connect(self.slotPrevious)
        self.actionNext.triggered.connect(self.slotNext)
        self.actionTimeLeft.clicked.connect(self.slotInitiativeAuthorization)
        self.checkBoxLoop.stateChanged.connect(self.slotLoopChanged)
        self.editRotationAngle.returnPressed.connect(self.slotEditChange)
        self.checkBoxShowPcBox.stateChanged.connect(self.slotShowPcBoxChanged)
        self.checkBoxShowLane.stateChanged.connect(self.slotShowLaneChanged)
        self.checkBoxShowTable.stateChanged.connect(self.slotShowTableChanged)
        self.checkBoxSaveBoxInfo.stateChanged.connect(self.slotSaveBoxInfoChanged)
        self.btnPcSetting.clicked.connect(self.slotPcSetting)

        self.sliderFrameId.sliderReleased.connect(self.slotSkip)
        self.spFrameId.valueChanged.connect(self.slotSkip)

        self.settingDlg.sigParamChanged.connect(self.slotSetParam)

        self.timerUpdate.timeout.connect(self.slotTimeOut)
        self.timerCheckAuthor.timeout.connect(self.slotCheckAuthorization)
        self.timerSurvivalCheck.timeout.connect(self.slotSurvivalCheck)
        self.updateSysState.timeout.connect(self.slotUpdateSysState)
        self.openGLWidget.sigSelectBox
        if self.stUiParam.bAutoConnect:
            self.timerAutoConnect = QTimer()
            self.timerAutoConnect.timeout.connect(self.slotAutoConnect)
            self.timerAutoConnect.setInterval(100)
            self.timerAutoConnect.start()

        self.openGLWidget.sigSelectBox.connect(self.slotSelectBox)

        self.tableWidget.cellClicked.connect(self.slotCellClicked)
        self.sigSelectBox.connect(self.slotSelectBox)

        self.timerCheckAuthor.singleShot(1000, self.slotCheckAuthorization)

    def slotModeSwitch(self):
        ret = self.slotSetting()
        if ret == QtWidgets.QDialog.Accepted:
            self.sliderFrameId.setEnabled(False)
            self.spFrameId.setEnabled(False)
            self.Mylog.warning("select online mode!")
            style = QApplication.style()
            icon = style.standardIcon(QtWidgets.QStyle.SP_DialogYesButton)
            self.actionModeSwitch.setIcon(icon)
            self.setCursor(QtCore.Qt.WaitCursor)
            self.statusbar.showMessage("加载数据中..., 请稍等!", 0)
            # 状态栏本身显示的信息 第二个参数是信息停留的时间，单位是毫秒，默认是0（0表示在下一个操作来临前一直显示）

    def slotAutoConnect(self):
        if not self.stSysState.get_act_state():
            return

        self.Mylog.warning("select online mode!")
        style = QApplication.style()
        icon = style.standardIcon(QtWidgets.QStyle.SP_DialogYesButton)
        self.actionModeSwitch.setIcon(icon)
        self.setCursor(QtCore.Qt.WaitCursor)

        # 状态栏本身显示的信息 第二个参数是信息停留的时间，单位是毫秒，默认是0（0表示在下一个操作来临前一直显示）
        self.statusbar.showMessage("加载数据中..., 请稍等!!", 0)
        self.settingDlg.slotAccept()
        self.timerAutoConnect.stop()

    def slotSetting(self):
        return self.settingDlg.exec()

    def slotSetParam(self):
        # self.slotStop()
        self.checkBoxLoop.setEnabled(False)
        self.titlePc.setText("全域数据")

        self.timeNoData.restart()
        self.bOnline = True
        self.bAuto = True
        self.slotStart()
        self.openGLWidget.setSysParam()
        self.timerUpdate.start()

    def slotOpenFile(self):
        strExePath = get_exe_path()
        filter = "txt or csv or json file(*.csv *.txt *.json)"
        FileDlg = QtWidgets.QFileDialog()
        FileDlg.setFileMode(QtWidgets.QFileDialog.ExistingFiles)
        strFileName, filetype = FileDlg.getOpenFileNames(None, "请选择文件", strExePath,
                                                         filter)  # 返回的QStringList, 被转换成了元组!

        # 判断选择的文件列表是否为空,即使不选择,FileList的len依然是2,如果选择了文件,则FileList中的[0]元素长度必然>0
        if len(strFileName) < 1:
            return

        self.sliderFrameId.setEnabled(False)
        self.spFrameId.setEnabled(False)
        self.Mylog.warning("open file:{} ...".format(strFileName))
        style = QApplication.style()
        icon = style.standardIcon(QtWidgets.QStyle.SP_DialogNoButton)
        self.actionModeSwitch.setIcon(icon)
        self.setCursor(QtCore.Qt.WaitCursor)
        self.titlePc.setText("全域数据")
        self.stSysState.set_sys_state(1)
        self.slotPublishWinState()

        self.openGLWidget.openFile(strFileName)
        self.bOnline = False
        self.timeNoData.restart()
        self.bAuto = True
        self.checkBoxLoop.setEnabled(True)
        self.timerUpdate.start()

    def slotPlayPause(self):
        self.timerUpdate.stop()
        self.bPause = not self.bPause
        if not self.bAuto:
            self.timeNoData.restart()

        style = QApplication.style()
        # icon = style.standardIcon(QtWidgets.QStyle.SP_MediaPlay)
        if self.bPause:
            self.Mylog.warning("pause...")

            self.actionPlayPause.setText("运行")
            img = QtGui.QImage(os.path.join(get_cfg_path(), "Icon/play.png"))
            pixmap = QtGui.QPixmap(img)
            fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                      QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
            icon = QIcon(fitPixmap)

            self.timerUpdate.stop()
            self.openGLWidget.pause()
        else:
            self.Mylog.warning("play...")
            self.bAuto = True
            self.actionPlayPause.setText("暂停")

            if self.stSysState.get_stop():
                self.setCursor(QtCore.Qt.WaitCursor)
                self.slotStart()
                self.slotStartPlay()
            self.openGLWidget.play()
            self.timerUpdate.start()
            # icon = style.standardIcon(QtWidgets.QStyle.SP_MediaPause)
            img = QtGui.QImage(os.path.join(get_cfg_path(), "Icon/pause.png"))
            pixmap = QtGui.QPixmap(img)
            fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                      QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
            icon = QIcon(fitPixmap)
        self.actionPlayPause.setIcon(icon)

    def slotStart(self):
        self.Mylog.warning("start..")
        self.statusbar.showMessage("加载数据中..., 请稍等!", 0)  # 状态栏本身显示的信息 第二个参数是信息停留的时间，单位是毫秒，默认是0（0表示在下一个操作来临前一直显示）
        self.stSysState.set_stop(False)
        self.stSysState.set_sys_state(1)
        self.slotPublishWinState()
        if not self.bOnline:
            return

    def slotStop(self, bQuit=False):
        self.Mylog.warning("stop...")
        self.statusbar.showMessage("停止显示!", 0)  # 状态栏本身显示的信息 第二个参数是信息停留的时间，单位是毫秒，默认是0（0表示在下一个操作来临前一直显示）
        self.bAuto = True
        self.stSysState.set_stop(True)
        if not bQuit:
            self.stSysState.set_sys_state(2)
            self.slotPublishWinState()
        self.timerUpdate.stop()

        # 关闭点云和视频显示
        self.openGLWidget.stop()

        # 修改界面相关标志位和文字
        self.bPause = True

        self.actionPlayPause.setText("运行")

        style = QApplication.style()
        img = QtGui.QImage(os.path.join(get_cfg_path(), "Icon/play.png"))
        pixmap = QtGui.QPixmap(img)
        fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        icon = QIcon(fitPixmap)
        # icon = style.standardIcon(QtWidgets.QStyle.SP_MediaPlay)
        self.actionPlayPause.setIcon(icon)
        self.setActionEnable(False)
        self.actionPlayPause.setEnabled(True)

    def slotStopPlay(self):
        self.Mylog.warning("stop play...")
        self.statusbar.showMessage("停止显示!", 0)  # 状态栏本身显示的信息 第二个参数是信息停留的时间，单位是毫秒，默认是0（0表示在下一个操作来临前一直显示）
        self.bAuto = True
        self.stSysState.set_stop(True)
        self.timerUpdate.stop()

        # 关闭点云和视频显示
        self.openGLWidget.stopPlay()

        # 修改界面相关标志位和文字
        self.bPause = True
        self.actionPlayPause.setText("运行")
        style = QApplication.style()
        # icon = style.standardIcon(QtWidgets.QStyle.SP_MediaPlay)
        img = QtGui.QImage(os.path.join(get_cfg_path(), "Icon/play.png"))
        pixmap = QtGui.QPixmap(img)
        fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        icon = QIcon(fitPixmap)
        self.actionPlayPause.setIcon(icon)
        self.setActionEnable(False)
        self.actionPlayPause.setEnabled(True)

    def slotStartPlay(self):
        self.Mylog.warning("start play...")
        self.statusbar.showMessage("加载数据中..., 请稍等!", 0)  # 状态栏本身显示的信息 第二个参数是信息停留的时间，单位是毫秒，默认是0（0表示在下一个操作来临前一直显示）
        self.stSysState.set_stop(False)
        self.timeNoData.restart()

        # start点云和视频显示
        self.openGLWidget.startPlay()

        # 修改界面相关标志位和文字
        self.bPause = False
        self.actionPlayPause.setText("暂停")
        img = QtGui.QImage(os.path.join(get_cfg_path(), "Icon/pause.png"))
        pixmap = QtGui.QPixmap(img)
        fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        icon = QIcon(fitPixmap)
        self.actionPlayPause.setIcon(icon)
        self.setActionEnable(False)
        self.actionPlayPause.setEnabled(True)
        self.timerUpdate.start()

    def slotRestart(self):
        self.Mylog.warning("restart...")
        self.setCursor(QtCore.Qt.WaitCursor)
        self.stSysState.set_noData(False)
        self.slotStopPlay()
        self.slotStartPlay()
        self.timerUpdate.start()

    def slotPrevious(self):
        # 关闭点云和视频显示
        self.bAuto = False
        self.timerUpdate.stop()
        self.actionPlayPause.setText("运行")
        # style = QApplication.style()
        # icon = style.standardIcon(QtWidgets.QStyle.SP_MediaPlay)
        img = QtGui.QImage(os.path.join(get_cfg_path(), "Icon/play.png"))
        pixmap = QtGui.QPixmap(img)
        fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        icon = QIcon(fitPixmap)
        self.actionPlayPause.setIcon(icon)

        self.openGLWidget.pause()
        self.openGLWidget.previous()
        self.timerUpdate.start(10)

        # 修改界面相关标志位和文字
        self.bPause = True
        self.Mylog.warning("slotPrevious!")

    def slotNext(self):
        self.bAuto = False
        self.actionPlayPause.setText("运行")
        img = QtGui.QImage(os.path.join(get_cfg_path(), "Icon/play.png"))
        pixmap = QtGui.QPixmap(img)
        fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        icon = QIcon(fitPixmap)
        self.actionPlayPause.setIcon(icon)

        # 关闭目标显示
        self.timerUpdate.stop()
        self.openGLWidget.pause()
        self.openGLWidget.next()
        self.timerUpdate.start(10)

        # 修改界面相关标志位和文字
        self.bPause = True
        self.Mylog.warning("slotPrevious!")

    def slotSurvivalCheck(self):
        try:
            mem, lisPidMemOut = getProMemByName()
            strMem = str(mem) + 'MB'
            self.Mylog.warning("Survival check! Mem={}, [pid, mem]:{}".format(strMem, lisPidMemOut))
            if self.stUiParam.bDebug:
                strVersion = "{} Debug! Mem:{}".format(PACK_VERSION_DATE_DEBUG, strMem)
            else:
                strVersion = "{} Mem:{}".format(PACK_VERSION_DATE_DEBUG, strMem)

            self.labVersion.setText(strVersion)

        except:
            self.Mylog.error("except! slotSurvivalCheck err!!")
            self.Mylog.error("except! Call stack:\n{}!".format(traceback.format_exc()))

    def slotUpdateSysState(self):
        """系统状态更新"""
        try:
            self.slotPublishWinState()
            list_msg = self.sub_winSysState.parse_response(False, 0.1)
            channel = str(list_msg[1], encoding="utf-8")
            json_state = json.loads(list_msg[2])
            if self.stSysState.get_sys_state() == 4 and json_state['sysState'] == 1:
                self.slotSetParam()
            self.stSysState.set_sys_state(json_state['sysState'])
            self.stSysState.set_act_state(bool(json_state['actState']))
            self.stSysState.set_online(bool(json_state['onlineState']))
            self.stSysState.set_localId(json_state['localId'])
            self.stSysState.set_active_res(json_state['activeState'])
            self.stSysState.set_timeLeft(json_state['actTimeLeft'])
            self.stSysState.set_noSpace(json_state['noSpace'])
        except:
            return

    def slotPublishWinState(self):
        """系统状态更新"""
        try:
            dictWinState = {'winState': self.stSysState.get_sys_state(),
                            'onlineState': int(self.stSysState.get_online()),
                            'paramUpdate': int(self.stSysState.get_update_param()),
                            'actCode': self.stSysState.get_act_code(),
                            'offlineFile': self.stSysState.get_offline_file()}
            json_state = json.dumps(dictWinState)
            self.data_bus_server.publish('win_state', json_state)
        except:
            pass

    def notEnoughSpace(self, fSpace=0, strMsg=''):
        self.Mylog.info("Free disk space {}mb is less than {}mb!".format(fSpace, MIN_SPACE_LIMIT))
        if self.msgBox is None:
            self.msgBox = QMessageBox(QMessageBox.Warning, '警告',
                                      "磁盘空间不足" + "{}mb, {}".format(MIN_SPACE_LIMIT, strMsg), QMessageBox.Ok)
            self.msgBox.setModal(True)

        if self.msgBox.isHidden():
            self.msgBox.show()

    def slotTimeOut(self):
        time1 = time.time()
        try:
            if self.bAuto and self.timerUpdate.interval() < TIME_INTERVAL:
                self.timerUpdate.start(TIME_INTERVAL)

            if self.stSysState.get_noData():
                if self.bFirstNoData:
                    self.stDstData = None
                    self.openGLWidget.slotTimerOut(self.stDstData)
                    self.slotUpdateTableWidget()
                    self.bFirstNoData = False

                if self.timeNoData.elapsed() < 5000:
                    return
                self.timerUpdate.stop()
                self.setActionEnable(False)
                self.actionPlayPause.setEnabled(True)
                self.setCursor(QtCore.Qt.WaitCursor)

                # 状态栏本身显示的信息 第二个参数是信息停留的时间，单位是毫秒，默认是0（0表示在下一个操作来临前一直显示）
                self.statusbar.showMessage("加载错误!", 0)

                if not self.bPause:
                    self.bPause = True
                    # style = QApplication.style()
                    # icon = style.standardIcon(QtWidgets.QStyle.SP_MediaPlay)
                    img = QtGui.QImage(os.path.join(get_cfg_path(), "Icon/play.png"))
                    pixmap = QtGui.QPixmap(img)
                    fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                              QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
                    icon = QIcon(fitPixmap)
                    self.actionPlayPause.setIcon(icon)

                self.timerUpdate.start()
                self.timeNoData.restart()
                self.stDstData = None
                return

            # 时间限制在算法上啦，显示耗时不高，主要是算法提供的慢，以至于显示刷新回等待队列有数据，才能刷新
            if self.dataTrackQueue.qsize() == 0:
                if self.timeNoData.elapsed() < 1500:
                    return

                if self.bAuto:
                    if not self.actionPrevious.isEnabled():
                        self.setActionEnable(False)
                        self.actionPlayPause.setEnabled(True)

                    if self.bOnline or (not self.stSysState.get_noData() and not self.openGLWidget.isPlayOver()):
                        if self.cursor().shape() != QtCore.Qt.WaitCursor:
                            self.setCursor(QtCore.Qt.WaitCursor)
                        self.statusbar.showMessage("加载数据中...",
                                                   0)  # 状态栏本身显示的信息 第二个参数是信息停留的时间，单位是毫秒，默认是0（0表示在下一个操作来临前一直显示）

                    if (not self.bOnline) and (self.openGLWidget.isPlayOver() or self.stSysState.get_noData()) \
                            and self.stSysState.get_state() in [5, 6]:
                        if self.cursor().shape() != QtCore.Qt.ArrowCursor:
                            self.setCursor(QtCore.Qt.ArrowCursor)
                        self.statusbar.showMessage("播放结束!", 0)  # 状态栏本身显示的信息 第二个参数是信息停留的时间，单位是毫秒，默认是0（0表示在下一个操作来临前一直显示）
                        if self.checkBoxLoop.checkState() == QtCore.Qt.Checked:
                            self.openGLWidget.setAutoLoopPlay(True)

                if not self.bPause:
                    self.bPause = True
                    img = QtGui.QImage(os.path.join(get_cfg_path(), "Icon/play.png"))
                    pixmap = QtGui.QPixmap(img)
                    fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                              QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
                    icon = QIcon(fitPixmap)
                    self.actionPlayPause.setIcon(icon)

                return

            self.timeNoData.restart()

            if self.bAuto:
                self.bPause = False
                img = QtGui.QImage(os.path.join(get_cfg_path(), "Icon/pause.png"))
                pixmap = QtGui.QPixmap(img)
                fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                          QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
                icon = QIcon(fitPixmap)
                self.actionPlayPause.setIcon(icon)
            self.setActionEnable(True)

            if self.cursor().shape() == QtCore.Qt.WaitCursor:
                self.setCursor(QtCore.Qt.ArrowCursor)
                self.statusbar.showMessage("加载完成!", 3000)  # 状态栏本身显示的信息 第二个参数是信息停留的时间，单位是毫秒，默认是0（0表示在下一个操作来临前一直显示）

            try:
                self.stDstData = self.dataTrackQueue.get_nowait()
            except:
                return

            if self.stSysState.get_noData():
                if not self.bOnline:
                    self.titlePc.setText("全域数据  [警告: 无全域数据!]")
            elif "无全域数据" in self.titlePc.text():
                self.titlePc.setText("全域数据")

            if self.stSysState.get_noData():
                self.stDstData = None

            nFrameCount = self.stSysState.get_frameCount()
            bOnline = self.stSysState.get_online()
            if (nFrameCount > 0) and (not bOnline):
                self.sliderFrameId.setEnabled(True)
                self.spFrameId.setEnabled(True)
                self.sliderFrameId.setRange(0, nFrameCount - 1)
                self.spFrameId.setRange(0, nFrameCount - 1)
            else:
                self.sliderFrameId.setEnabled(False)
                self.spFrameId.setEnabled(False)

            self.bFirstNoData = True  # 用于控制在没有任何数据时，刷新一次显示区域，使显示区域变成黑屏
            self.nFrameId = self.openGLWidget.slotTimerOut(self.stDstData)
            if self.stSysState.get_noData() and self.stDstData is not None:
                self.nFrameId = self.stDstData.nFrameId
            self.slotUpdateTableWidget()
            if not bOnline:
                self.sliderFrameId.sliderReleased.disconnect(self.slotSkip)
                self.spFrameId.valueChanged.disconnect(self.slotSkip)
                self.sliderFrameId.setValue(self.nFrameId)
                self.spFrameId.setValue(self.nFrameId)
                self.sliderFrameId.sliderReleased.connect(self.slotSkip)
                self.spFrameId.valueChanged.connect(self.slotSkip)
        except:
            self.Mylog.error("except! slotTimeOut err!")
            self.Mylog.error("except! Call stack:\n{}!".format(traceback.format_exc()))

    def slotInitiativeAuthorization(self):
        dlgAction = ActionTipDlg(stSysState=self.stSysState, parent=self)
        if dlgAction.exec() == QDialog.Accepted:
            self.stSysState.set_act_state(True)
            self.timerCheckAuthor.singleShot(1000, self.slotCheckAuthorization)

    def slotCheckAuthorization(self):
        try:
            if self.stSysState.get_noSpace():
                self.slotStopPlay()
                self.notEnoughSpace(fSpace, "请释放部分空间!")
            elif self.stSysState.get_stop():
                if self.msgBox is not None:
                    self.msgBox.hide()
                self.slotStartPlay()

            strMsg = "请输入激活码！"
            nDaysLeft = self.stSysState.get_timeLeft()
            if self.stSysState.get_sys_state() == 4:
                return
            b_act_state = self.stSysState.get_act_state()
            if nDaysLeft == "Activated":
                self.actionTimeLeft.setText("已激活!")
            else:
                nDaysLeft = max(0, nDaysLeft)
                self.actionTimeLeft.setText("时间剩余:{}天".format(nDaysLeft))
            self.slotStopPlay()
            if b_act_state:
                return
            self.timerCheckAuthor.stop()
            try:
                dlgAction = ActionTipDlg(strMsg, stSysState=self.stSysState, parent=self)
                if dlgAction.exec() == QDialog.Accepted:
                    self.timerCheckAuthor.singleShot(1000, self.slotCheckAuthorization)
                    self.timerCheckAuthor.start()
                    self.slotStartPlay()
                    return
                else:
                    self.slotQuit(True)
            except:
                self.timerCheckAuthor.start()
        except:
            self.Mylog.error("except! slotCheckAuthorization err!")
            self.Mylog.error(" - except! Call stack:\n{}!".format(traceback.format_exc()))

    def slotEditChange(self):
        axis = [0.0, 0.0, 1.0]
        self.openGLWidget.rotate(axis, float(self.editRotationAngle.text()))
        return

    def slotLoopChanged(self, nStatus):
        if nStatus == QtCore.Qt.Unchecked:
            self.openGLWidget.setAutoLoopPlay(False)
            self.Mylog.warning("setAutoLoopPlay(False)!")
        else:
            self.openGLWidget.setAutoLoopPlay(True)
            self.Mylog.warning("setAutoLoopPlay(True)!")

    def slotShowPcBoxChanged(self, nStatus):
        if nStatus == QtCore.Qt.Unchecked:
            self.openGLWidget.setShowBox(False)
        else:
            self.openGLWidget.setShowBox(True)

    def slotShowLaneChanged(self, nStatus):
        if nStatus == QtCore.Qt.Unchecked:
            self.openGLWidget.setShowLane(False)
        else:
            self.openGLWidget.setShowLane(True)

    def slotShowTableChanged(self, nStatus):
        if nStatus == QtCore.Qt.Unchecked:
            self.subSplitterRight.setVisible(False)
        else:
            self.subSplitterRight.setVisible(True)

    def slotSaveBoxInfoChanged(self, nStatus):
        if nStatus == QtCore.Qt.Unchecked:
            self.strSaveBoxInfoFilePath = ""
        else:
            # save box info
            strSaveDir = time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
            self.strSaveBoxInfoFilePath = '{}{}'.format(csvPath, strSaveDir)
            cre_dir(self.strSaveBoxInfoFilePath)
            if self.stSysState.get_noSpace:
                del_dir(self.strSaveBoxInfoFilePath)
                self.checkBoxSaveBoxInfo.setCheckState(QtCore.Qt.Unchecked)
                self.notEnoughSpace(fSpace, "保存数据已停止!")
                return

    def slotPcSetting(self):
        self.paramSettingDlg.exec()

    def slotSkip(self):
        self.sliderFrameId.sliderReleased.disconnect(self.slotSkip)
        self.spFrameId.valueChanged.disconnect(self.slotSkip)
        strObjName = self.sender().objectName()
        if "sliderFrameId" in strObjName:
            self.spFrameId.setValue(self.sliderFrameId.value())
        elif "spFrameId" in strObjName:
            self.sliderFrameId.setValue(self.spFrameId.value())
        else:
            return

        nSkipToFrameId = self.spFrameId.value()
        print('nSkipToFrameId', nSkipToFrameId)
        self.bAuto = False
        self.timerUpdate.stop()

        self.openGLWidget.pause()
        self.openGLWidget.skipTo(nSkipToFrameId)
        self.timerUpdate.start(self.stUiParam.nTimeInterval)

        # 修改界面相关标志位和文字
        self.bPause = True

        self.actionPlayPause.setText("运行")

        img = QtGui.QImage(os.path.join(get_cfg_path(), "Icon/play.png"))
        pixmap = QtGui.QPixmap(img)
        fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        icon = QIcon(fitPixmap)
        self.actionPlayPause.setIcon(icon)
        self.sliderFrameId.sliderReleased.connect(self.slotSkip)
        self.spFrameId.valueChanged.connect(self.slotSkip)

    def slotCellClicked(self, nRow, nCol):
        self.tableWidget.selectRow(nRow)
        itemBoxId = self.tableWidget.item(nRow, 1)
        self.nSelectBoxId = int(itemBoxId.text())
        self.slotSelectBox(self.nSelectBoxId)
        print("slotCellClicked box {}".format(self.nSelectBoxId))

    def slotSelectBox(self, nSelectBoxId):
        self.nSelectBoxId = nSelectBoxId
        strObjName = self.sender().objectName()
        if "openGLWidget" in strObjName:

            self.openGLWidget.setSelectBoxId(nSelectBoxId)
            self.sigSelectBox.disconnect(self.slotSelectBox)
            self.slotUpdateTableWidget()
            self.sigSelectBox.connect(self.slotSelectBox)
        else:
            self.openGLWidget.setSelectBoxId(nSelectBoxId)
            self.slotUpdateTableWidget()

    def slotUpdateTableWidget(self):
        timeTemp = QTime()
        timeTemp.start()
        if self.stDstData is None:
            return

        if self.bFirstTest:
            self.timeFps.start()
            self.bFirstTest = False

        listBoxInfo = copy.deepcopy(self.stDstData.listBoxInfo)
        nPcBoxCount = len(listBoxInfo)

        listBoxInfoDeduplication = []
        for boxInfo in listBoxInfo:
            bIsRepeat = False
            for i in range(len(listBoxInfoDeduplication)):
                if int(boxInfo.boxId) == int(listBoxInfoDeduplication[i].boxId):
                    bIsRepeat = True
                    break

            if not bIsRepeat:
                listBoxInfoDeduplication.append(boxInfo)

        listBoxInfo = listBoxInfoDeduplication
        nDelCount = 0
        nRowCount = len(listBoxInfo)
        nEsapsed = self.timeFps.elapsed() - self.nLastTime
        if nEsapsed > 1000:
            nFrameCount = self.nFrameId - self.nLastFrameId
            if self.nFrameId < self.nLastFrameId:
                nTemp = MAX_FRAME_ID - self.nLastFrameId
                nFrameCount = self.nFrameId + nTemp

            self.fFps = round(nFrameCount / nEsapsed * 1000, 3)
            if not self.bAuto:
                self.fFps = 1

            if self.fFps < 0 or self.fFps > 15:
                self.fFps = 10
                self.Mylog.warning("View: self.fFps ERROR, self.fFps={}, nFrameIdCur={}, nFrameIdLast={}, fEsapsed={}!"
                                   .format(self.fFps, self.nFrameId, self.nLastFrameId, nEsapsed / 1000))
            self.timeFps.restart()
            self.nLastFrameId = self.nFrameId

        self.statusbar.showMessage("目标:{}, 帧率:{}".format(nRowCount, self.fFps), 0)

        nCarCount = 0
        lisCarType = ["car", "truck", "bus"]
        nBicycleCount = 0
        lisBicycleType = ["bicycle", "motorbike"]
        nPersonCount = 0
        self.tableWidget.setSortingEnabled(False)
        self.tableWidget.setRowCount(nRowCount)
        self.tableWidget.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tableWidget.clearContents()
        lSelectRow = 0
        for i in range(nRowCount):
            boxInfo = listBoxInfo[i]
            if boxInfo.strClass in lisCarType:
                nCarCount += 1
            elif boxInfo.strClass in lisBicycleType:
                nBicycleCount += 1
            elif boxInfo.strClass == "person":
                nPersonCount += 1

            if self.strSaveBoxInfoFilePath != "" and boxInfo.boxSpeed is not None:
                # tCsv = time.time()
                strAllPcBoxInfoFilePath = '{}{}'.format(self.strSaveBoxInfoFilePath, '/AllPcBoxInfo.csv')
                if not my_isExist(strAllPcBoxInfoFilePath):
                    boxInfoHead = ['Timestamp', 'FrameIdPcap', 'FrameId', 'boxId', 'boxCenter[0]', 'boxCenter[1]',
                                   'boxCenter[2]',
                                   'boxSize[0]', 'boxSize[1]', 'boxSize[2]', 'boxSpeed', 'boxAngle', 'Class',
                                   'Color', 'boxLongitude', 'boxLatitude', 'Source', 'nConfidence']
                    my_save2csv(strAllPcBoxInfoFilePath, boxInfoHead)

                # 保存识别结果，目的是与转发出去的数据作对比，测试转发是否正常
                angle_north_t = self.stLidarParam.getAngleNorthT()
                boxAngle = (180 + angle_north_t + boxInfo.boxAngle / PI_rads) % 360  # 弧度转角度
                boxInfo2csv = [self.stDstData.tTimeStamp, int(self.nFrameId), int(boxInfo.boxId),
                               boxInfo.boxCenter[0], boxInfo.boxCenter[1], boxInfo.boxCenter[2],
                               boxInfo.boxSize[0], boxInfo.boxSize[1], boxInfo.boxSize[2],
                               boxInfo.boxSpeed, boxAngle, getSendEnumFromStr(boxInfo.strClass),
                               getSendEnumFromColorStr(boxInfo.strColor), boxInfo.boxLongitude, boxInfo.boxLatitude,
                               boxInfo.strSource, boxInfo.nConfidence]

                if self.stSysState.get_noSpace:
                    self.checkBoxSaveBoxInfo.setCheckState(QtCore.Qt.Unchecked)
                    self.notEnoughSpace(fSpace, "保存数据已停止!")
                else:
                    my_save2csv(strAllPcBoxInfoFilePath, boxInfo2csv)

            item = QtWidgets.QTableWidgetItem()
            item.setText(str(int(self.nFrameId)))
            # item.setData(QtCore.Qt.DisplayRole, boxInfo.frameId)
            self.tableWidget.setItem(lSelectRow, 0, item)

            item = QtWidgets.QTableWidgetItem()
            item.setText(str(int(boxInfo.boxId)))
            # item.setData(QtCore.Qt.DisplayRole, boxInfo.boxId)
            self.tableWidget.setItem(lSelectRow, 1, item)

            strLen = str(boxInfo.boxSize[1])
            strWidth = str(boxInfo.boxSize[0])
            strHeight = str(boxInfo.boxSize[2])
            retType = type(boxInfo.boxSize[0])

            item = QtWidgets.QTableWidgetItem()
            item.setText(strLen)
            # item.setData(QtCore.Qt.DisplayRole, boxInfo.boxSize[0])
            self.tableWidget.setItem(lSelectRow, 2, item)

            item = QtWidgets.QTableWidgetItem()
            item.setText(strWidth)
            # item.setData(QtCore.Qt.DisplayRole, boxInfo.boxSize[1])
            self.tableWidget.setItem(lSelectRow, 3, item)

            item = QtWidgets.QTableWidgetItem()
            item.setText(strHeight)
            # item.setData(QtCore.Qt.DisplayRole, boxInfo.boxSize[2])
            self.tableWidget.setItem(lSelectRow, 4, item)

            item = QtWidgets.QTableWidgetItem()
            strSpeed = str(boxInfo.boxSpeed)
            retType = type(boxInfo.boxSpeed)
            # if retType == float:
            if strSpeed != 'None':
                # print("strSpeed round... ", i)
                strSpeed = str(round(boxInfo.boxSpeed, 3))
            item.setText(strSpeed)
            # item.setTextAlignment(QtCore.Qt.AlignLeft)
            # item.setText("--")
            # item.setTextAlignment(QtCore.Qt.AlignCenter)
            # item.setData(QtCore.Qt.DisplayRole, boxInfo.boxSpeed)
            self.tableWidget.setItem(lSelectRow, 5, item)

            item = QtWidgets.QTableWidgetItem()
            strAngle = str(boxInfo.boxAngle)
            retType = type(boxInfo.boxAngle)
            # if retType == float:
            if strAngle != 'None':
                # print("strLen round... ", i)
                fAngle = boxInfo.boxAngle / math.pi * 180
                if fAngle > 360:
                    fAngle = fAngle - 360
                strAngle = str(round(fAngle, 3))
            item.setText(strAngle)
            # item.setData(QtCore.Qt.DisplayRole, boxInfo.boxAngle)
            self.tableWidget.setItem(lSelectRow, 6, item)

            item = QtWidgets.QTableWidgetItem(boxInfo.strClass)
            self.tableWidget.setItem(lSelectRow, 7, item)
            # self.tableWidget.setColumnHidden(1, True)
            if self.stUiParam.bDebug:
                item = QtWidgets.QTableWidgetItem()
                strLongitude = str(boxInfo.boxLongitude)
                item.setText(strLongitude)
                # item.setData(QtCore.Qt.DisplayRole, boxInfo.boxSpeed)
                self.tableWidget.setItem(lSelectRow, 8, item)

                item = QtWidgets.QTableWidgetItem()
                strLatitude = str(boxInfo.boxLatitude)
                item.setText(strLatitude)
                # item.setData(QtCore.Qt.DisplayRole, boxInfo.boxSpeed)
                self.tableWidget.setItem(lSelectRow, 9, item)

                item = QtWidgets.QTableWidgetItem(boxInfo.strSource)
                self.tableWidget.setItem(lSelectRow, 10, item)
            for i in range(self.tableWidget.columnCount()):
                self.tableWidget.setColumnHidden(i, True)

            listTableWidgetType = self.titleTableBoxType.get_selected()
            for i in range(len(listTableWidgetType)):
                self.tableWidget.setColumnHidden(self.dictTableWidgetShowType[listTableWidgetType[i]], False)
            if self.nSelectBoxId == int(boxInfo.boxId):
                self.tableWidget.selectRow(lSelectRow)

            lSelectRow += 1
        self.tableWidget.setRowCount(nRowCount - nDelCount)
        self.tableWidget.setSortingEnabled(True)

    def slotQuit(self, bSure=False):
        if not bSure:
            msg_box = QMessageBox()
            msg_box.setWindowTitle('提示')
            msg_box.setText("是否关闭软件")
            msg_box.setStandardButtons(QMessageBox.Ok | QMessageBox.Close | QMessageBox.Cancel)
            buttonY = msg_box.button(QMessageBox.Ok)
            buttonY.setText('确定')
            buttonN = msg_box.button(QMessageBox.Cancel)
            buttonN.setText('取消')
            buttonC = msg_box.button(QMessageBox.Close)
            buttonC.setText('关闭界面')
            msg_box.exec_()
            if msg_box.clickedButton() == buttonY:
                self.Mylog.warning("Quit..")
                self.setEnabled(False)
                self.timerUpdate.stop()
                self.slotStop()
                self.sigQuit.emit()
                self.stSysState.set_sys_state(3)
                self.slotPublishWinState()
                return True
            elif msg_box.clickedButton() == buttonC:
                self.Mylog.warning("Close MainWindow..")
                self.setEnabled(False)
                self.timerUpdate.stop()
                self.slotStop(True)
                self.sigQuit.emit()
                return True
            return False
        else:
            self.Mylog.warning("Quit..")
            self.setEnabled(False)
            self.timerUpdate.stop()
            self.slotStop()
            self.sigQuit.emit()
            self.stSysState.set_sys_state(3)
            self.slotPublishWinState()
            return True

    def closeEvent(self, event):
        bQuit = self.slotQuit()
        if bQuit:
            killSelfSamePro()
            QMainWindow.closeEvent(self, event)
        else:
            event.ignore()

    def close_window(self, event):
        bQuit = self.slotQuit()
        if bQuit:
            killSelfSamePro()
            QMainWindow.closeEvent(self, event)
        else:
            event.ignore()
