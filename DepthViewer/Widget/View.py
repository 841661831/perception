# -*- coding: utf-8 -*-
# PyQt的导入必须放在前面，否则release运行时会崩溃！
from .PcPyGLWidget import *
from .SettingDlg import *
from .PcSettingDlg import *
from .ActionTipDlg import *
from .ComboCheckBox import ComboCheckBox
from multiprocessing.managers import BaseManager
from multiprocessing import Lock
import open3d as o3d

TIME_INTERVAL = stUiParam.nTimeInterval  # 单位ms
PACK_VERSION_DATE = "v2.11_20210609"
PACK_VERSION_DATE_DEBUG = "v2.11_20210609"

g_bIpCheckRunning = True
# g_bIpCheckOnline = True


def ipCheckThread(strDevIp, listReachable, lockCheck):
    MyLog.writeLog("{}[:{}] - ipCheckThread start, pid={}, tid={}"
                   .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, os.getpid(), threading.currentThread().name)
                   , LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)

    global g_bIpCheckRunning

    while g_bIpCheckRunning:
        # if g_bIpCheckOnline:
        ret = isIpReachable(strDevIp)
        lockCheck.acquire()
        listReachable.clear()
        listReachable.append(ret)
        lockCheck.release()
        time.sleep(0.1)



def ipCheckProcess(bRunning, stLidarParam):
    while bRunning.value == 1:
        MyLog.writeLog("{}[:{}] - ipCheckProcess start, pid={}"
                       .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, os.getpid())
                       , LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)
        global g_bIpCheckRunning
        g_bIpCheckRunning = True
        lockCheck = threading.Lock()

        listAllIp = []
        listForwardDstDevIp = []
        for strLidarSrcIp in stLidarParam.getlistAttachedIp():
            if strLidarSrcIp is not None and len(strLidarSrcIp) > 0:
                listAllIp.append(strLidarSrcIp)
        listForwardDstDev = stLidarParam.getListForwardDstDev()
        listForwardEventDstDev = stLidarParam.getListForwardEventDstDev()
        listForwardDstDev.extend(listForwardEventDstDev)
        for strDev in listForwardDstDev:
            nameList = strDev[0].split('-')  # 分割字符串
            strDevIp = nameList[0]
            if strDevIp in listAllIp or strDevIp is None:
                continue

            listForwardDstDevIp.append(strDevIp)
            listAllIp.append(strDevIp)

        listAllReachable = []
        listThread = []
        for strDevIp in listAllIp:
            listReachable = [0]
            th = threading.Thread(target=ipCheckThread, args=(strDevIp, listReachable, lockCheck,))
            th.start()
            listThread.append(th)
            listAllReachable.append(listReachable)

        # #创建检测线程
        # memorycheck_thread = threading.Thread(target=detectMemSize)
        # # memorycheck_thread.daemon = True
        # memorycheck_thread.start()

        tTemp = time.time()
        while bRunning.value == 1:

            listAllIpTemp = []
            for strLidarSrcIp in stLidarParam.getlistAttachedIp():
                if strLidarSrcIp is not None and len(strLidarSrcIp) > 0:
                    listAllIpTemp.append(strLidarSrcIp)
            listForwardDstDev = stLidarParam.getListForwardDstDev()
            listForwardEventDstDev = stLidarParam.getListForwardEventDstDev()
            listForwardDstDev.extend(listForwardEventDstDev)
            for strDev in listForwardDstDev:
                nameList = strDev[0].split('-')  # 分割字符串
                strDevIp = nameList[0]
                if strDevIp in listAllIpTemp or strDevIp is None:
                    continue

                listAllIpTemp.append(strDevIp)

            if listAllIpTemp != listAllIp:
                print("ipCheckProcess:ip changed! Cur:", listAllIpTemp)
                break

            t1 = time.time()
            listReachableIp = []
            strIpReachable = ""
            nForwardCount = len(listForwardDstDevIp)
            # print("len:{}-{}".format(len(listAllIp), len(listAllReachable)))
            lockCheck.acquire()
            for i in range(len(listAllReachable)):
                if i == 0:
                    strIpReachable += "Lidar:{}, ".format(int(listAllReachable[i][0]))
                elif 0 < i <= nForwardCount:
                    strIpReachable += "V2X{}:{}, ".format(i, int(listAllReachable[i][0]))
                else:
                    strIpReachable += "Camera{}:{}, ".format(i - nForwardCount, int(listAllReachable[i][0]))

                # t1 = time.time()
                if len(listAllReachable[i]) > 0 and listAllReachable[i][0]:
                    listReachableIp.append(listAllIp[i])
            # print("ip time:%.2fms" % ((time.time() - t1) * 1000))
            lockCheck.release()

            # print("listReachableIp:", listReachableIp)
            with open(ipTruePathTemp, 'w') as fTrue:
                for strDevIp in listReachableIp:
                    fTrue.write(strDevIp + "\n")
            my_renameFile2(ipTruePathTemp, ipTruePath, True)

            if time.time() - tTemp > 10:
                MyLog.writeLog("{}[:{}] - {}".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, strIpReachable), LOG_WARNING, ENABLE_WRITE_LOG, True)
                tTemp = time.time()

            time.sleep(1)

        g_bIpCheckRunning = False
        for th in listThread:
            if th.is_alive():
                th.join()
        # if memorycheck_thread.is_alive():
        #     memorycheck_thread.join()

        MyLog.writeLog("{}[:{}] - exit ipCheckProcess..".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)


class MainWindow(QMainWindow):
    sigQuit = pyqtSignal()
    sigSelectBox = pyqtSignal(int)

    def init(self, tStart):
        self.tStart = tStart
        self.setObjectName("WJ-DepthViewer")
        self.setContextMenuPolicy(QtCore.Qt.NoContextMenu)
        self.resize(820, 656)
        self.bPause = True
        self.bStop = False
        self.bAuto = True
        self.bOnline = False
        self.bSaveAll = False
        self.bPcReadyToPlay = False
        self.bInit = True
        self.bUseBgdFrameFilter = False
        self.bActivated = False
        self.stDstData = None
        self.strSaveBoxInfoFilePath = ""
        self.strSaveSrcDataDir = ""
        self.listBoxInfo = []
        self.msgBox = None
        self.msgBoxCreatBmp = None

        self.timeTemp = QTime()
        self.timeTemp.start()
        self._translate = QtCore.QCoreApplication.translate
        self.manager = BaseManager()
        self.manager.start()
        self.stLidarParam = self.manager.structLidarParam()
        stLidarParam = structLidarParam()
        self.stUiParam = structUiParam()
        self.stPcAlgParam = structPcAlgParam()
        serializationParam(stLidarParam = stLidarParam, stPcAlgParam=self.stPcAlgParam, stUiParam=self.stUiParam)
        self.stLidarParam.setParam(stLidarParam)

        self.mp_bRuning = multiprocessing.Value("H", 1)
        self.ipCheckPro = Process(target=ipCheckProcess, args=(self.mp_bRuning, self.stLidarParam,))
        self.ipCheckPro.start()

        self.settingDlg = SettingDlg()
        self.settingDlg.init(self.stLidarParam)
        self.pcSettingDlg = PcSettingDlg()
        self.pcSettingDlg.init(self.stLidarParam)
        self.timeTest = QTime()
        self.bFirstTest = True
        self.nLastTime = 0
        self.nLastFrameId = 0
        self.fFps = 10

        self.timerUpdate = QTimer()
        self.timerUpdate.setInterval(TIME_INTERVAL)
        self.timerCheck = QTimer()
        self.timerCheck.setInterval(60 * 1000)  # 300 * 1000
        self.timerCheck.start()
        self.timeCheckIP = QTimer()
        self.timeCheckIP.setInterval(1000)
        self.timeCheckIP.start()
        self.timerSurvivalCheck = QTimer()
        self.timerSurvivalCheck.setInterval(10000)
        self.timerSurvivalCheck.start()

        self.timeNoData = QTime()
        self.strSaveFilePath = ""
        self.stSrcPcResultQueue = Queue()
        self.bFusion = True
        self.bFirstNoData = True  # 用于控制在没有任何数据时，刷新一次显示区域，使显示区域变成黑屏
        self.nSelectBoxId = -1
        self.mp_bRuning = multiprocessing.Value("H", 1)
        self.mp_bOnline = multiprocessing.Value("H", 0)
        self.mp_bSave = multiprocessing.Value("H", 0)
        # self.savePro.start()

        self.bStationRegistration = False
        self.bSelectCube = False
        self.nAddClickCount = 0
        self.nAldCubeCount = 0
        self.listSelectData = [[],[]]


        # self.backgroundata = readVectorGraph()
        self.listVBO = getVectorGraphVBO(self.stLidarParam)


        self.m_manager = BaseManager()
        self.m_manager.register('structData', structData)
        self.m_manager.start()
        self.m_obj = self.m_manager.structData()
        self.m_lock = Lock()

        self.comboType = ComboCheckBox([self._translate("WJ-DepthViewer","car"), self._translate("WJ-DepthViewer","bicycle") , self._translate("WJ-DepthViewer","bus"), self._translate("WJ-DepthViewer", "motorbike"), self._translate("WJ-DepthViewer","person"), self._translate("WJ-DepthViewer","cone"), self._translate("WJ-DepthViewer","truck"), self._translate("WJ-DepthViewer","Minibus")])
        self.comboType.setObjectName("comboType")
        # add
        self.bPcInfoReady = False
        self.listPcInfo = []

        # QMenuBar
        self.menubar = QtWidgets.QMenuBar(self)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 820, 39))
        self.menubar.setObjectName("menubar")

        # Add menu at menubar
        self.menuFile = QtWidgets.QMenu(self.menubar)
        self.menuFile.setObjectName("menuFile")
        self.menuSubSave = QtWidgets.QMenu(self.menuFile)
        self.menuSubSave.setObjectName("menuSubSave")

        self.menuOperator = QtWidgets.QMenu(self.menubar)
        self.menuOperator.setObjectName("menuOperator")
        # self.setMenuBar(self.menubar)

        # Create QAction
        self.actionOpen = QtWidgets.QAction(self)
        self.actionOpen.setObjectName("actionOpen")
        self.actionOpenSingle = QtWidgets.QAction(self)
        self.actionOpenSingle.setObjectName("actionOpenSingle")
        self.actionOpenSingle.setVisible(False)
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

        self.actionStationRegistration = QtWidgets.QAction(self)
        self.actionStationRegistration.setObjectName("actionLidarRegistration")
        self.actionStationRegistration.setCheckable(True)
        self.actionStationRegistration.setVisible(True)

        stretch = QtWidgets.QLabel()
        stretch.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)
        #self.actionTimeLeft = QtWidgets.QAction(self)
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

        #旋转角度输入框
        self.labelRotationAngle = QtWidgets.QLabel()
        self.labelRotationAngle.setText(self._translate("WJ-DepthViewer","°"))
        self.editRotationAngle = QtWidgets.QLineEdit()
        self.editRotationAngle.setObjectName("editRotationAngle")
        self.editRotationAngle.setMaximumWidth(70)
        self.editRotationAngle.setPlaceholderText(self._translate("WJ-DepthViewer", "Ratation Angle"))
        self.editRotationAngle.setValidator(QDoubleValidator(-180.0, 180.0, 2))
        # doubleValidator = QDoubleValidator(self.editRotationAngle)
        # doubleValidator.setRange(-180, 180)
        # doubleValidator.setNotation(QDoubleValidator.StandardNotation)
        # doubleValidator.setDecimals(2)

        # Add QAction for menu
        # self.menuSubSave.addAction(self.actionSubSavePC)

        self.menuFile.addAction(self.actionOpen)
        self.menuFile.addAction(self.actionOpenSingle)
        self.menuFile.addAction(self.menuSubSave.menuAction())
        # self.menuFile.addAction(self.actionSetting)
        self.menuFile.addAction(self.actionQuit)

        self.menuOperator.addAction(self.actionModeSwitch)

        self.menubar.addAction(self.menuFile.menuAction())
        self.menubar.addAction(self.menuOperator.menuAction())

        # QToolBar
        self.mainToolBar = QtWidgets.QToolBar(self)
        self.mainToolBar.setObjectName("mainToolBar")
        self.mainToolBar.addAction(self.actionOpen)
        self.mainToolBar.addAction(self.actionOpenSingle)
        self.mainToolBar.addAction(self.actionPrevious)
        self.mainToolBar.addAction(self.actionPlayPause)
        self.mainToolBar.addAction(self.actionNext)
        self.mainToolBar.addAction(self.actionStop)
        self.mainToolBar.addAction(self.actionRestart)
        self.mainToolBar.addAction(self.actionModeSwitch)
        self.mainToolBar.addAction(self.actionStationRegistration)

        self.mainToolBar.addWidget(self.comboType)
        self.mainToolBar.addWidget(self.editRotationAngle)
        self.mainToolBar.addWidget(self.labelRotationAngle)
        self.mainToolBar.addWidget(item)
        # self.mainToolBar.addWidget(self.checkBoxLoop)
        # self.mainToolBar.addWidget(self.sliderFrameId)
        self.mainToolBar.addWidget(self.spFrameId)
        # self.checkBoxLoop.setVisible(False)
        # self.sliderFrameId.setVisible(False)
        # self.spFrameId.setVisible(False)
        self.mainToolBar.addWidget(stretch)
        #self.mainToolBar.addAction(self.actionTimeLeft)
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
        self.checkBoxCoordinateFlag = QtWidgets.QCheckBox()
        self.checkBoxCoordinateFlag.setObjectName("checkBoxCoordinateFlag")
        self.checkBoxCoordinateFlag.setChecked(True)
        self.checkBoxCoordinateFlag.setStyleSheet("border:0px;")
        self.checkBoxShowLane = QtWidgets.QCheckBox()
        self.checkBoxShowLane.setObjectName("checkBoxShowPcBox")
        self.checkBoxShowLane.setChecked(True)
        self.checkBoxShowLane.setStyleSheet("border:0px;")
        self.checkBoxShowStationFusion = QtWidgets.QCheckBox()
        self.checkBoxShowStationFusion.setObjectName("checkBoxShowRLFusion")
        self.checkBoxShowStationFusion.setChecked(True)
        self.checkBoxShowStationFusion.setStyleSheet("border:0px;")
        self.checkBoxShowTable = QtWidgets.QCheckBox()
        self.checkBoxShowTable.setObjectName("checkBoxShowRLFusion")
        self.checkBoxShowTable.setChecked(False)
        self.checkBoxShowTable.setStyleSheet("border:0px;")
        self.btnPcSetting = QtWidgets.QPushButton(self)
        self.btnPcSetting.setObjectName("btnPcSetting")
        self.btnPcSetting.setMinimumSize(21, 21)
        self.btnPcSetting.setStyleSheet("border:0px;")

        self.selectStation = QtWidgets.QComboBox()
        list_tmp = []
        for i in range(len(self.stLidarParam.getlistAttachedIp())):
            list_tmp.append("{}号基站".format(i+1))
        self.selectStation.setObjectName("selectStation")
        self.selectStation.addItems(list_tmp)
        self.selectStation.addItem("全景")
        self.selectStation.setStyleSheet("background-color:#515151;border-radius: 5px; color: rgb(255, 255, 255);border:0px")
        self.selectStation.setVisible(True)

        # self.LidarCount = 3
        # self.RadarCount = 1
        # self.titleShowLidarBoxIp = ComboCheckBox(
        #     [self._translate("WJ-DepthViewer", "Lidar1"), self._translate("WJ-DepthViewer", "Lidar2"),
        #      self._translate("WJ-DepthViewer", "Lidar3"), self._translate("WJ-DepthViewer", "Radar")])
        # self.titleShowLidarBoxIp.setObjectName("titleShowLidarBoxIp")
        # self.titleShowLidarBoxIp.setStyleSheet("width: 90px; height: 15px; font-size: 12px;background-color:#515151;")

        self.hBoxLayout = QtWidgets.QHBoxLayout(self.titlePc)
        self.hBoxLayout.addWidget(self.openGLWidgetTitle)
        self.hBoxLayout.addItem(spacerItem)
        self.hBoxLayout.addWidget(self.selectStation)
        self.hBoxLayout.addWidget(self.checkBoxShowPcBox)
        self.hBoxLayout.addWidget(self.checkBoxCoordinateFlag)
        self.hBoxLayout.addWidget(self.checkBoxShowLane)
        self.hBoxLayout.addWidget(self.checkBoxShowStationFusion)
        self.hBoxLayout.addWidget(self.checkBoxShowTable)
        # self.hBoxLayout.addWidget(self.titleShowLidarBoxIp)
        self.hBoxLayout.addWidget(self.btnPcSetting)
        self.hBoxLayout.setContentsMargins(5, 2, 5, 2)
        self.hBoxLayout.setSpacing(5)
        # self.subSplitterLeft.addWidget(self.openGLWidgetTitle)

        # GLWidget
        # self.openGLWidget = QtWidgets.QOpenGLWidget(self.frame)
        self.openGLWidget = PcPyGLWidget(self)
        self.openGLWidget.setObjectName("openGLWidget")
        self.openGLWidget.setMinimumHeight(10)
        self.openGLWidget.setSyncParam(self.stSrcPcResultQueue)
        # self.subSplitterLeft.addWidget(self.openGLWidget)

        # GlWidget Frame
        self.frame = QtWidgets.QFrame()
        self.frame.setFrameShape(QtWidgets.QFrame.Box)
        self.frame.setStyleSheet("border: 1px solid gray;")
        vBoxLayout = QtWidgets.QVBoxLayout(self.frame)
        vBoxLayout.addWidget(self.titlePc)
        vBoxLayout.addWidget(self.openGLWidget)
        vBoxLayout.setContentsMargins(0,0,0,0)
        vBoxLayout.setSpacing(0)
        self.subSplitterLeft.addWidget(self.frame)

        # TabPc corner
        # self.checkBoxShowPcBox = QtWidgets.QCheckBox()
        # self.checkBoxShowPcBox.setObjectName("checkBoxShowPcBox")
        # self.checkBoxShowPcBox.setChecked(True)
        # cornerWidgetPc = QtWidgets.QWidget()
        # hLayoutPc = QtWidgets.QHBoxLayout()
        # hLayoutPc.addWidget(self.checkBoxShowPcBox)
        # hLayoutPc.setContentsMargins(0, 0, 0, 0)
        # cornerWidgetPc.setLayout(hLayoutPc)
        #
        # # QTabPc
        # self.tabPc = QtWidgets.QTabWidget(self.centralwidget)
        # self.tabPc.setObjectName("tabWidget")
        # self.tabPc.setVisible(True)
        # self.tabPc.addTab(self.openGLWidget, "")
        # self.tabPc.setCornerWidget(cornerWidgetPc, QtCore.Qt.TopRightCorner)
        # self.subSplitterLeft.addWidget(self.tabPc)
        ''
        # ============================ opengl widget ===========================
        ''

        self.dictTableWidgetShowType = {}

        # QTableWidget Box Info
        # QTableWidget Box Info Title
        self.titleTableBoxType = ComboCheckBox(
            [self._translate("WJ-DepthViewer", "FrameId"), self._translate("WJ-DepthViewer", "BoxId"), self._translate("WJ-DepthViewer", "Length(m)"),
             self._translate("WJ-DepthViewer", "Width(m)"), self._translate("WJ-DepthViewer", "Height(m)"),
             self._translate("WJ-DepthViewer", "Speed(m/s)"), self._translate("WJ-DepthViewer", "Angle(deg)"),
             self._translate("WJ-DepthViewer", "Class")])

        if self.stUiParam.bDebug:
            self.titleTableBoxType = ComboCheckBox(
            [self._translate("WJ-DepthViewer", "FrameId"), self._translate("WJ-DepthViewer", "BoxId"), self._translate("WJ-DepthViewer", "Length(m)"),
             self._translate("WJ-DepthViewer", "Width(m)"), self._translate("WJ-DepthViewer", "Height(m)"),
             self._translate("WJ-DepthViewer", "Speed(m/s)"), self._translate("WJ-DepthViewer", "Angle(deg)"),
             self._translate("WJ-DepthViewer", "Class"), self._translate("WJ-DepthViewer", "Longitude(deg)"),
             self._translate("WJ-DepthViewer", "Latitude(deg)") , self._translate("WJ-DepthViewer", "Source")])
        self.titleTableBoxType.setObjectName("titleTableBoxType")
        if (stUiParam.SoftwareStyle == 0):
            self.titleTableBoxType.setStyleSheet("width: 90px; height: 15px; font-size: 12px;background-color:#515151;")#background-color:#000000;515151

        # self.btnStartSelectLidarPC = QtWidgets.QPushButton(self)
        # self.btnStartSelectLidarPC.setObjectName("btnStartSelectPC")
        # self.btnStartSelectLidarPC.setMinimumSize(21, 21)
        # self.btnStartSelectLidarPC.setStyleSheet("border:0px;background-color:#515151")
        # self.btnStartSelectLidarPC.setVisible(False)

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

        spacerItemLidar = QtWidgets.QSpacerItem(0, 0, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)

        # self.tablehlayout.addItem(spacerItemLidar)
        # self.tablehlayout.addWidget(self.btnStartSelectLidarPC)


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

        # QTableWidget Select Area Point Info
        self.tableWidgetPointInfo = QtWidgets.QTableWidget(self.centralwidget)
        # self.tableWidgetPointInfo.horizontalHeader().setStretchLastSection(True)
        self.tableWidgetPointInfo.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.tableWidgetPointInfo.setObjectName("tableWidgetPointInfo")
        # self.tableWidgetPointInfo.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.tableWidgetPointInfo.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.tableWidgetPointInfo.setVisible(False)
        if self.stUiParam.bDebug:
            self.subSplitterRight.addWidget(self.tableBoxInfoFrame)
        else:
            self.subSplitterRight.addWidget(self.tableWidget)
        self.subSplitterRight.addWidget(self.tableWidgetPointInfo)


        ###################Station registration table####################

        self.titleStation = QtWidgets.QLabel()
        # self.titleVideo.setStyleSheet("QLabel{background-color:rgb(87,96,95);border:0px;}")
        self.titleStation.setFixedHeight(25)
        self.titleStation.setVisible(False)
        self.StationParamTitle = QtWidgets.QLabel()
        self.StationParamTitle.setStyleSheet("border:0px;")
        self.StationParamTitle.setText("Station Param")

        spacerItemRadar = QtWidgets.QSpacerItem(0, 0, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)

        self.btnAdd = QtWidgets.QPushButton(self)
        self.btnAdd.setObjectName("btnAdd")
        self.btnAdd.setMinimumSize(21, 21)
        self.btnAdd.setStyleSheet("border:0px;background-color:#515151")

        self.btnSave = QtWidgets.QPushButton(self)
        self.btnSave.setObjectName("btnSave")
        self.btnSave.setMinimumSize(21, 21)
        self.btnSave.setStyleSheet("border:0px;background-color:#515151")

        self.btnStartSelectCube1 = QtWidgets.QPushButton(self)
        self.btnStartSelectCube1.setObjectName("btnStartSelectPC")
        self.btnStartSelectCube1.setMinimumSize(21, 21)
        self.btnStartSelectCube1.setStyleSheet("border:0px;background-color:#515151")
        self.btnStartSelectCube1.setVisible(False)

        self.btnStartSelectCube2 = QtWidgets.QPushButton(self)
        self.btnStartSelectCube2.setObjectName("btnStartSelectRadar")
        self.btnStartSelectCube2.setMinimumSize(21, 21)
        self.btnStartSelectCube2.setStyleSheet("border:0px;background-color:#515151")
        self.btnStartSelectCube2.setVisible(False)

        self.StationParamhlayout = QtWidgets.QHBoxLayout(self.titleStation)
        self.StationParamhlayout.addWidget(self.StationParamTitle)
        self.StationParamhlayout.addWidget(self.btnAdd)
        self.StationParamhlayout.addWidget(self.btnSave)
        self.StationParamhlayout.addItem(spacerItemRadar)
        self.StationParamhlayout.addWidget(self.btnStartSelectCube1)
        self.StationParamhlayout.addWidget(self.btnStartSelectCube2)

        self.StationParamhlayout.setContentsMargins(5, 2, 5, 2)
        self.StationParamhlayout.setSpacing(5)

        self.tableWidgetSelectCube = QtWidgets.QTableWidget(self.centralwidget)
        self.tableWidgetSelectCube.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.tableWidgetSelectCube.setObjectName("tableWidgetPointInfo")
        self.tableWidgetSelectCube.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectItems)
        self.tableWidgetSelectCube.setVisible(False)

        self.tableRadarFrame = QtWidgets.QFrame()
        self.tableRadarFrame.setFrameShape(QtWidgets.QFrame.Box)
        self.tableRadarFrame.setStyleSheet("border: 1px solid gray;")
        vBoxLayouttableRadar = QtWidgets.QVBoxLayout(self.tableRadarFrame)
        vBoxLayouttableRadar.addWidget(self.titleStation)
        vBoxLayouttableRadar.addWidget(self.tableWidgetSelectCube)
        vBoxLayouttableRadar.setContentsMargins(0, 0, 0, 0)
        vBoxLayouttableRadar.setSpacing(0)

        self.subSplitterRight.addWidget(self.tableRadarFrame)
        self.subSplitterRight.addWidget(self.tableWidgetSelectCube)
        self.subSplitterRight.setVisible(False)
        self.mainSplitter.addWidget(self.subSplitterLeft)
        self.mainSplitter.addWidget(self.subSplitterRight)
        self.mainSplitter.setStretchFactor(0, 5)  # 设置分割界面的初始比例4/5
        self.mainSplitter.setStretchFactor(1, 1)  # 设置分割界面的初始比例1/5

        # Set UI Text
        self.retranslateUi()
        self.setIcon()
        self.setActionEnable(False)

        # self.tabWidget.setCurrentIndex(0)

        # Set the component in the center of the main window
        self.setCentralWidget(self.centralwidget)

        # connect slots
        self.connectSlots()
        my_delFile(ipTruePath)

        QtCore.QMetaObject.connectSlotsByName(self)
        # self.openGLWidget.resize(QtCore.QSize(500, 500))

        self.openGLWidget.setLIDARParam(self.stLidarParam)

    def __del__(self):
        self.slotStop()
        del self.openGLWidget

    # Set UI Text
    def retranslateUi(self):
        self.setWindowTitle(self._translate("WJ-DepthViewer", "WJ-ISFP"))

        # 设置水平方向的表头标签与垂直方向上的表头标签，注意必须在初始化行列之后进行，否则没有效果
        self.tableWidget.setColumnCount(8)
        if (stUiParam.SoftwareStyle == 0):
            self.tableWidget.verticalHeader().setHidden(True)
        self.tableWidget.setHorizontalHeaderLabels(
            ['FrameId', 'BoxId', 'Length(m)', 'Width(m)', 'Height(m)', 'Speed(m/s)', 'Angle(deg)', 'Class'])
        self.dictTableWidgetShowType = {'FrameId': 0, 'BoxId': 1, 'Length(m)': 2, 'Width(m)': 3, 'Height(m)': 4, 'Speed(m/s)': 5, 'Angle(deg)': 6, 'Class': 7}
        if self.stUiParam.bDebug:
            self.tableWidget.setColumnCount(11)
            self.tableWidget.setHorizontalHeaderLabels(
                ['FrameId', 'BoxId', 'Length(m)', 'Width(m)', 'Height(m)', 'Speed(m/s)', 'Angle(deg)', 'Class', 'Longitude(deg)', 'Latitude(deg)', "Source"])
            self.dictTableWidgetShowType = {'FrameId': 0, 'BoxId': 1, 'Length(m)': 2, 'Width(m)': 3, 'Height(m)': 4,
                                            'Speed(m/s)': 5, 'Angle(deg)': 6, 'Class': 7, 'Longitude(deg)': 8, 'Latitude(deg)': 9, "Source": 10}


        # self.tableWidget.horizontalHeader().setStretchLastSection(True)
        # self.tableWidget.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        self.tableWidget.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)

        self.tableWidgetPointInfo.setColumnCount(6)
        self.tableWidgetPointInfo.setHorizontalHeaderLabels(['FrameId', 'PointId', 'x(m)', 'y(m)', 'z(m)', 'intensity'])

        self.tableWidgetSelectCube.setColumnCount(2)
        self.tableWidgetSelectCube.setHorizontalHeaderLabels(['Major-Station', 'Minor-Station'])

        self.menuFile.setTitle(self._translate("WJ-DepthViewer", "File"))
        self.actionOpen.setText(self._translate("WJ-DepthViewer", "Open"))
        self.actionOpenSingle.setText(self._translate("WJ-DepthViewer", "OpenSingle"))

        self.menuSubSave.setTitle(self._translate("WJ-DepthViewer", "Save"))

        self.actionSetting.setText(self._translate("WJ-DepthViewer", "Setting"))
        self.actionQuit.setText(self._translate("WJ-DepthViewer", "Quit"))

        self.menuOperator.setTitle(self._translate("WJ-DepthViewer", "Operator"))
        self.actionModeSwitch.setText(self._translate("WJ-DepthViewer", "OnLine"))

        self.actionPlayPause.setText(self._translate("WJ-DepthViewer", "Play"))
        self.actionStop.setText(self._translate("WJ-DepthViewer", "Stop"))
        self.actionRestart.setText(self._translate("WJ-DepthViewer", "Restart"))
        self.actionPrevious.setText(self._translate("WJ-DepthViewer", "Previous"))
        self.actionNext.setText(self._translate("WJ-DepthViewer", "Next"))
        self.actionStationRegistration.setText(self._translate("WJ-DepthViewer", "LidarRegistration"))
        if stUiParam.bDebug:
            strVersion = self._translate("WJ-DepthViewer", "{} Debug!".format(PACK_VERSION_DATE_DEBUG))
        else:
            strVersion = self._translate("WJ-DepthViewer", "{}".format(PACK_VERSION_DATE))

        self.labVersion.setText(strVersion)
        self.labVersion.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.checkBoxLoop.setText(self._translate("WJ-DepthViewer", "Loop"))
        # self.labelRotationAngle.setText(self._translate("WJ-DepthViewer", "Rota Angle"))
        self.checkBoxShowPcBox.setText(self._translate("WJ-DepthViewer", "show box"))
        self.checkBoxShowLane.setText(self._translate("WJ-DepthViewer", "show lane"))
        self.checkBoxCoordinateFlag.setText(self._translate("WJ-DepthViewer", "Coordinate"))
        self.checkBoxShowStationFusion.setText(self._translate("WJ-DepthViewer", "show Fusion"))
        self.checkBoxShowTable.setText(self._translate("WJ-DepthViewer", "show Table"))
        self.checkBoxSaveBoxInfo.setText(self._translate("WJ-DepthViewer", "Save Box Info"))
        self.titlePc.setText(self._translate("WJ-DepthViewer", "Point cloud"))
        self.tableBoxInfoTitle.setText(self._translate("WJ-DepthViewer", "Box Info"))
        self.btnPcSetting.setToolTip(self._translate("WJ-DepthViewer", "pc setting"))

        self.btnAdd.setToolTip(self._translate("WJ-DepthViewer", "AddStationTableRow"))
        self.btnSave.setToolTip(self._translate("WJ-DepthViewer", "SaveStationParam"))
        self.btnStartSelectCube1.setToolTip(self._translate("WJ-DepthViewer", "SelectCube1"))
        self.btnStartSelectCube2.setToolTip(self._translate("WJ-DepthViewer", "SelectCube2"))



    # Set Icon
    def setIcon(self):
        style = QApplication.style()

        icon = style.standardIcon(QtWidgets.QStyle.SP_DirOpenIcon)
        self.actionOpen.setIcon(icon)

        img = QtGui.QImage(strExePath + "/Configs/Icon/pause.png")
        pixmap = QtGui.QPixmap(img)
        fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        icon = QIcon(fitPixmap)
        #icon = style.standardIcon(QtWidgets.QStyle.SP_MediaPlay)
        self.actionPlayPause.setIcon(icon)

        img = QtGui.QImage(strExePath + "/Configs/Icon/stop.png")
        pixmap = QtGui.QPixmap(img)
        fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        icon = QIcon(fitPixmap)
        #icon = style.standardIcon(QtWidgets.QStyle.SP_MediaStop)
        self.actionStop.setIcon(icon)


        icon = style.standardIcon(QtWidgets.QStyle.SP_BrowserReload)
        self.actionRestart.setIcon(icon)

        img = QtGui.QImage(strExePath + "/Configs/Icon/pre.png")
        pixmap = QtGui.QPixmap(img)
        fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        icon = QIcon(fitPixmap)
        #icon = style.standardIcon(QtWidgets.QStyle.SP_MediaSeekBackward)
        self.actionPrevious.setIcon(icon)

        img = QtGui.QImage(strExePath + "/Configs/Icon/next.png")
        pixmap = QtGui.QPixmap(img)
        fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        icon = QIcon(fitPixmap)
        #icon = style.standardIcon(QtWidgets.QStyle.SP_MediaSeekForward)
        self.actionNext.setIcon(icon)

        icon = style.standardIcon(QtWidgets.QStyle.SP_DialogNoButton)
        self.actionModeSwitch.setIcon(icon)

        img = QtGui.QImage(strExePath + "/Configs/Icon/pcSetting.png")
        pixmap = QtGui.QPixmap(img)
        fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio, QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        icon = QIcon(fitPixmap)
        self.btnPcSetting.setIcon(icon)


        # img = QtGui.QImage(strExePath + "/Configs/Icon/OpenSingle.jpeg")
        # pixmap = QtGui.QPixmap(img)
        # fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio, QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        # icon = QIcon(fitPixmap)
        # self.actionOpenSingle.setIcon(icon)

        img = QtGui.QImage(strExePath + "/Configs/Icon/calculate.png")
        pixmap = QtGui.QPixmap(img)
        fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        icon = QIcon(fitPixmap)
        self.actionStationRegistration.setIcon(icon)

        img = QtGui.QImage(strExePath + "/Configs/Icon/add.png")
        pixmap = QtGui.QPixmap(img)
        fitPixmap = pixmap.scaled(21, 21, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        icon = QIcon(fitPixmap)
        self.btnAdd.setIcon(icon)

        img = QtGui.QImage(strExePath + "/Configs/Icon/ROI.jpeg")
        pixmap = QtGui.QPixmap(img)
        fitPixmap = pixmap.scaled(21, 21, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        icon = QIcon(fitPixmap)
        self.btnStartSelectCube1.setIcon(icon)
        self.btnStartSelectCube2.setIcon(icon)

        img = QtGui.QImage(strExePath + "/Configs/Icon/calculate.png")
        pixmap = QtGui.QPixmap(img)
        fitPixmap = pixmap.scaled(21, 21, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        icon = QIcon(fitPixmap)
        self.btnSave.setIcon(icon)

    def setActionEnable(self, bEnabled):
        self.actionPrevious.setEnabled(bEnabled)
        self.actionPlayPause.setEnabled(bEnabled)
        self.actionNext.setEnabled(bEnabled)
        self.actionStop.setEnabled(bEnabled)
        self.actionRestart.setEnabled(bEnabled)
        self.actionStationRegistration.setEnabled(bEnabled)

    def connectSlots(self):
        self.actionOpen.triggered.connect(self.slotOpenFile)
        self.actionOpenSingle.triggered.connect(self.slotOpenSingleFile)
        self.actionSetting.triggered.connect(self.slotSetting)
        self.actionQuit.triggered.connect(self.slotQuit)

        self.actionModeSwitch.triggered.connect(self.slotModeSwitch)

        self.selectStation.activated.connect(self.slotSelectionChange)


        self.actionPlayPause.triggered.connect(self.slotPlayPause)
        self.actionStop.triggered.connect(self.slotStopPlay)
        self.actionRestart.triggered.connect(self.slotRestart)
        self.actionPrevious.triggered.connect(self.slotPrevious)
        self.actionNext.triggered.connect(self.slotNext)
        #self.actionTimeLeft.triggered.connect(self.slotInitiativeAuthorization)
        self.actionTimeLeft.clicked.connect(self.slotInitiativeAuthorization)
        self.checkBoxLoop.stateChanged.connect(self.slotLoopChanged)
        self.editRotationAngle.returnPressed.connect(self.slotEditChange)
        self.checkBoxShowPcBox.stateChanged.connect(self.slotShowPcBoxChanged)
        self.checkBoxShowLane.stateChanged.connect(self.slotShowLaneChanged)
        self.checkBoxCoordinateFlag.stateChanged.connect(self.slotShowCoordinateFlagChanged)
        self.checkBoxShowStationFusion.stateChanged.connect(self.slotShowStationFusionChanged)
        self.checkBoxShowTable.stateChanged.connect(self.slotShowTableChanged)
        self.checkBoxSaveBoxInfo.stateChanged.connect(self.slotSaveBoxInfoChanged)
        self.btnPcSetting.clicked.connect(self.slotPcSetting)

        self.sliderFrameId.sliderReleased.connect(self.slotSkip)
        self.spFrameId.valueChanged.connect(self.slotSkip)

        self.settingDlg.sigParamChanged.connect(self.slotSetParam)

        self.timerUpdate.timeout.connect(self.slotTimeOut)
        self.timerCheck.timeout.connect(self.slotCheckAuthorization)
        self.timeCheckIP.timeout.connect(self.slotNetStatusChanged)
        self.timerSurvivalCheck.timeout.connect(self.slotSurvivalCheck)
        if stUiParam.bAutoConnect:
            self.timerAutoConnect = QTimer()
            self.timerAutoConnect.timeout.connect(self.slotAutoConnect)
            self.timerAutoConnect.setInterval(100)
            self.timerAutoConnect.start()

        self.openGLWidget.sigSelectBox.connect(self.slotSelectBox)
        self.openGLWidget.sigSelectStationBox.connect(self.slotSelectStationBox)

        self.tableWidget.cellClicked.connect(self.slotCellClicked)
        self.tableWidgetSelectCube.cellClicked.connect(self.slotSelectCellClicked)
        self.sigSelectBox.connect(self.slotSelectBox)

        self.timerCheck.singleShot(1000, self.slotCheckAuthorization)

        self.actionStationRegistration.triggered.connect(self.slotStationRegistration)
        self.btnAdd.clicked.connect(self.slotAddaInfo)
        self.btnSave.clicked.connect(self.slotSaveSelect)
        self.btnStartSelectCube1.clicked.connect(self.slotSelectCube1)
        self.btnStartSelectCube2.clicked.connect(self.slotSelectCube2)




    # 装饰器@QtCore.pyqtSlot()不是必需的！ 但如果非要加上，且信号要传递参数，
    # 则必须在装饰器@QtCore.pyqtSlot()的括号里填写要传递的参数类型
    @QtCore.pyqtSlot(bool)
    def slotTest2(self, b):
        print(b)

    def slotSelectionChange(self):

        station_id = self.selectStation.currentText()
        self.openGLWidget.SelectionChange(station_id)


    def slotModeSwitch(self):
        ret = self.slotSetting()
        if ret == QtWidgets.QDialog.Accepted:
            self.sliderFrameId.setEnabled(False)
            self.spFrameId.setEnabled(False)
            MyLog.writeLog("{}[:{}] - select online mode!".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
            style = QApplication.style()
            icon = style.standardIcon(QtWidgets.QStyle.SP_DialogYesButton)
            self.actionModeSwitch.setIcon(icon)
            self.setCursor(QtCore.Qt.WaitCursor)
            self.statusbar.showMessage(self._translate("WJ-DepthViewer", "Loading data..., it is expected to wait a few seconds!"), 0)  # 状态栏本身显示的信息 第二个参数是信息停留的时间，单位是毫秒，默认是0（0表示在下一个操作来临前一直显示）
            # QMessageBox.information(self, self._translate("WJ-DepthViewer", 'Info'), self._translate("WJ-DepthViewer", "Data loading, it is expected to wait a few seconds!"), QMessageBox.Ok)
            # self.mp_bOnline.value = 1
            # self.bOnline = True
            # self.bAuto = True
            # self.timeNoData.restart()
            # self.timerUpdate.start()

    def slotAutoConnect(self):
        if not self.bActivated:
            return

        MyLog.writeLog("{}[:{}] - select online mode!".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                        sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
        style = QApplication.style()
        icon = style.standardIcon(QtWidgets.QStyle.SP_DialogYesButton)
        self.actionModeSwitch.setIcon(icon)
        self.setCursor(QtCore.Qt.WaitCursor)

        # 状态栏本身显示的信息 第二个参数是信息停留的时间，单位是毫秒，默认是0（0表示在下一个操作来临前一直显示）
        self.statusbar.showMessage(self._translate("WJ-DepthViewer", "Loading data..., it is expected to wait a few seconds!"), 0)
        self.settingDlg.slotAccept()
        self.timerAutoConnect.stop()

    def slotSetting(self):
        return self.settingDlg.exec()

    def slotSetParam(self):
        # self.slotStop()
        self.checkBoxLoop.setEnabled(False)
        self.titlePc.setText(self._translate("WJ-DepthViewer", "Point cloud"))

        self.timeNoData.restart()
        self.mp_bOnline.value = 1
        self.bOnline = True
        self.bAuto = True
        self.slotStart()
        self.openGLWidget.setLIDARParam()

        self.mp_bRuning.value = 0
        if self.ipCheckPro is not None and self.ipCheckPro.is_alive():
            self.ipCheckPro.terminate()

        self.mp_bRuning.value = 1
        self.ipCheckPro = Process(target=ipCheckProcess, args=(self.mp_bRuning, self.stLidarParam,))
        self.ipCheckPro.start()
        self.timerUpdate.start()



    # 文件多选
    # def slotOpenFile(self):
    #     strExePath = getExcutePath()
    #     # filter = "Ply (*.ply); Binary (*.bin)"
    #     filter = "pc or video(*.pcap *.avi)"
    #     FileDlg = QtWidgets.QFileDialog()
    #     FileDlg.setFileMode(QtWidgets.QFileDialog.ExistingFiles)
    #     FileList, filetype = FileDlg.getOpenFileNames(None, self._translate("WJ-DepthViewer", "Select files for LiDAR"), strExePath, filter)  # 返回的QStringList, 被转换成了元组!
    #
    #     # 判断选择的文件列表是否为空,即使不选择,FileList的len依然是2,如果选择了文件,则FileList中的[0]元素长度必然>0
    #     if len(FileList) < 1:
    #         return
    #
    #     listSelectPath = []
    #     for strFile in FileList:
    #         # nameList = path.split('/')
    #         # strFile = nameList[len(nameList) - 1]
    #         strFilePath = strFile[:strFile.rfind('/') + 1]
    #         strFile = strFile[strFile.rfind('/') + 1:]
    #         strFileName = strFile[:strFile.find('.')]
    #         if "Channel" in strFile:
    #             strFileName = strFile[:strFile.find('_')]
    #
    #         strFileName = strFilePath + strFileName
    #         print(strFile, strFileName)
    #         if strFileName in listSelectPath:
    #             continue
    #
    #         listSelectPath.append(strFileName)
    #
    #     print(listSelectPath)
    #     MyLog.writeLog("{}[:{}] - open file:{} ...".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, FileList), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
    #     style = QApplication.style()
    #     icon = style.standardIcon(QtWidgets.QStyle.SP_DialogNoButton)
    #     self.actionModeSwitch.setIcon(icon)
    #     self.setCursor(QtCore.Qt.WaitCursor)
    #
    #     self.mp_bOnline.value = 0
    #     if self.bSaveAll:
    #         self.slotSaveAll()
    #
    #     self.openGLWidget.openFile(listSelectPath)
    #     self.bOnline = False
    #     self.timeNoData.restart()
    #     self.bAuto = True
    #     self.checkBoxLoop.setEnabled(True)
    #     self.timerUpdate.start()

    def slotOpenFile(self):
        strExePath = getExcutePath()
        # filter = "Ply (*.ply); Binary (*.bin)"
        # filter = "pc or video(*.pcap *.avi *.csv)"
        filter = "txt or csv or json file(*.csv *.txt *.json)"
        FileDlg = QtWidgets.QFileDialog()
        FileDlg.setFileMode(QtWidgets.QFileDialog.ExistingFiles)
        strFileName, filetype = FileDlg.getOpenFileNames(None, self._translate("WJ-DepthViewer", "Select files for LiDAR"), strExePath, filter)  # 返回的QStringList, 被转换成了元组!

        # 判断选择的文件列表是否为空,即使不选择,FileList的len依然是2,如果选择了文件,则FileList中的[0]元素长度必然>0
        if len(strFileName) < 1:
            return
        if isinstance(strFileName,str):
            FileList = [strFileName]
        else:
            FileList = strFileName
        listSelectPath = []
        for strFile in FileList:
            # nameList = path.split('/')
            # strFile = nameList[len(nameList) - 1]
            strFilePath = strFile[:strFile.rfind('/') + 1]
            strFileName = strFile[strFile.rfind('/') + 1:]

            strFileName = strFilePath + strFileName
            if strFileName in listSelectPath:
                continue

            listSelectPath.append(strFileName)

        self.sliderFrameId.setEnabled(False)
        self.spFrameId.setEnabled(False)

        MyLog.writeLog("{}[:{}] - open file:{} ...".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, FileList), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
        style = QApplication.style()
        icon = style.standardIcon(QtWidgets.QStyle.SP_DialogNoButton)
        self.actionModeSwitch.setIcon(icon)
        self.setCursor(QtCore.Qt.WaitCursor)
        self.titlePc.setText(self._translate("WJ-DepthViewer", "Point cloud"))

        self.mp_bOnline.value = 0
        if self.bSaveAll:
            self.slotSaveAll()

        self.openGLWidget.openFile(listSelectPath)
        self.bOnline = False
        self.timeNoData.restart()
        self.bAuto = True
        self.checkBoxLoop.setEnabled(True)
        self.timerUpdate.start()

    # 读取单帧
    def slotOpenSingleFile(self):
        strExePath = getExcutePath()
        FileDlg = QtWidgets.QFileDialog()
        FileDlg.setFileMode(QtWidgets.QFileDialog.ExistingFile)
        strDirName = FileDlg.getExistingDirectory(None, self._translate("WJ-DepthViewer", "Select dir"), strExePath, QtWidgets.QFileDialog.ShowDirsOnly)  # 返回的QStringList, 被转换成了元组!

        # 判断选择的文件列表是否为空,即使不选择,FileList的len依然是2,如果选择了文件,则FileList中的[0]元素长度必然>0
        if len(strDirName) < 1:
            return

        FileList = [strDirName]
        listSelectPath = []
        for strDirName in FileList:
            listSelectPath.append(strDirName)

        print(listSelectPath)
        MyLog.writeLog("{}[:{}] - open file:{} ...".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, FileList), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
        style = QApplication.style()
        icon = style.standardIcon(QtWidgets.QStyle.SP_DialogNoButton)
        self.actionModeSwitch.setIcon(icon)
        self.setCursor(QtCore.Qt.WaitCursor)
        self.titlePc.setText(self._translate("WJ-DepthViewer", "Point cloud"))

        self.mp_bOnline.value = 0
        if self.bSaveAll:
            self.slotSaveAll()

        self.stLidarParam.setSingleFrame(True)
        self.openGLWidget.openFile(listSelectPath)
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
        #icon = style.standardIcon(QtWidgets.QStyle.SP_MediaPlay)
        if self.bPause:
            MyLog.writeLog("{}[:{}] - pause...".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)

            self.actionPlayPause.setText(self._translate("WJ-DepthViewer", "Play"))
            img = QtGui.QImage(strExePath + "/Configs/Icon/play.png")
            pixmap = QtGui.QPixmap(img)
            fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                      QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
            icon = QIcon(fitPixmap)

            self.timerUpdate.stop()
            if self.bSaveAll:
                self.slotSaveAll()
            self.openGLWidget.pause()
        else:
            MyLog.writeLog("{}[:{}] - play...".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
            self.bAuto = True
            self.actionPlayPause.setText(self._translate("WJ-DepthViewer", "Pause"))

            if self.bStop:
                self.setCursor(QtCore.Qt.WaitCursor)
                self.slotStart()
                self.slotStartPlay()
            self.openGLWidget.play()
            self.timerUpdate.start()
            #icon = style.standardIcon(QtWidgets.QStyle.SP_MediaPause)
            img = QtGui.QImage(strExePath + "/Configs/Icon/pause.png")
            pixmap = QtGui.QPixmap(img)
            fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                      QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
            icon = QIcon(fitPixmap)
        self.actionPlayPause.setIcon(icon)

    def slotStart(self):
        MyLog.writeLog("{}[:{}] - start..".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
        self.statusbar.showMessage(self._translate("WJ-DepthViewer", "Loading data..., it is expected to wait a few seconds!"), 0)  # 状态栏本身显示的信息 第二个参数是信息停留的时间，单位是毫秒，默认是0（0表示在下一个操作来临前一直显示）
        self.bStop = False
        if not self.bOnline:
            return

        self.mp_bRuning.value = 1
        self.stLidarParam.setParamChanged(True)

        if self.ipCheckPro is None or (not self.ipCheckPro.is_alive()):
            self.ipCheckPro = Process(target=ipCheckProcess, args=(self.mp_bRuning, self.stLidarParam,))
            self.ipCheckPro.start()

    def slotStop(self):
        MyLog.writeLog("{}[:{}] - stop...".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
        self.statusbar.showMessage(self._translate("WJ-DepthViewer", "Stop play!"), 0)  # 状态栏本身显示的信息 第二个参数是信息停留的时间，单位是毫秒，默认是0（0表示在下一个操作来临前一直显示）
        self.bAuto = True
        self.bStop = True
        self.timerUpdate.stop()

        self.mp_bRuning.value = 0

        if self.bSaveAll:
            self.slotSaveAll()

        # 关闭点云和视频显示
        self.openGLWidget.stop()

        self.stSrcPcResultQueue = Queue()
        #self.openGLWidget.setSyncParam(self.stSrcPcResultQueue, self.pcSaveQueue, self.pcBaseStationQueue1, self.pcBaseStationQueue2)
        self.openGLWidget.setSyncParam(self.stSrcPcResultQueue)

        if self.ipCheckPro is not None and self.ipCheckPro.is_alive():
            # print("self.savePro.join() start..")
            self.ipCheckPro.terminate()
        # 修改界面相关标志位和文字
        self.bPause = True
        # print("self.bPause = True 2")

        self.actionPlayPause.setText(self._translate("WJ-DepthViewer", "Play"))

        style = QApplication.style()
        img = QtGui.QImage(strExePath + "/Configs/Icon/play.png")
        pixmap = QtGui.QPixmap(img)
        fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        icon = QIcon(fitPixmap)
        #icon = style.standardIcon(QtWidgets.QStyle.SP_MediaPlay)
        self.actionPlayPause.setIcon(icon)
        self.setActionEnable(False)
        self.actionPlayPause.setEnabled(True)
        my_delDir(CachePath)
        my_delDir(SavePath)

    def slotStopPlay(self):
        MyLog.writeLog("{}[:{}] - stop play...".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
        self.statusbar.showMessage(self._translate("WJ-DepthViewer", "Stop play!"), 0)  # 状态栏本身显示的信息 第二个参数是信息停留的时间，单位是毫秒，默认是0（0表示在下一个操作来临前一直显示）
        self.bAuto = True
        self.bStop = True
        self.timerUpdate.stop()

        self.mp_bOnline.value = 0
        if self.bSaveAll:
            self.slotSaveAll()

        # 关闭点云和视频显示
        self.openGLWidget.stopPlay()

        # 修改界面相关标志位和文字
        self.bPause = True
        self.actionPlayPause.setText(self._translate("WJ-DepthViewer", "Play"))
        style = QApplication.style()
        #icon = style.standardIcon(QtWidgets.QStyle.SP_MediaPlay)
        img = QtGui.QImage(strExePath + "/Configs/Icon/play.png")
        pixmap = QtGui.QPixmap(img)
        fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        icon = QIcon(fitPixmap)
        self.actionPlayPause.setIcon(icon)
        self.setActionEnable(False)
        self.actionPlayPause.setEnabled(True)
        my_delDir(CachePath)

    def slotStartPlay(self):
        MyLog.writeLog("{}[:{}] - start play...".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
        self.statusbar.showMessage(self._translate("WJ-DepthViewer", "Loading data..., it is expected to wait a few seconds!"), 0)  # 状态栏本身显示的信息 第二个参数是信息停留的时间，单位是毫秒，默认是0（0表示在下一个操作来临前一直显示）
        self.bStop = False
        self.mp_bOnline.value = 1
        self.timeNoData.restart()

        # start点云和视频显示
        self.openGLWidget.startPlay()

        # 修改界面相关标志位和文字
        self.bPause = False
        self.actionPlayPause.setText(self._translate("WJ-DepthViewer", "Pause"))
        #style = QApplication.style()
        #icon = style.standardIcon(QtWidgets.QStyle.SP_MediaPause)
        img = QtGui.QImage(strExePath + "/Configs/Icon/pause.png")
        pixmap = QtGui.QPixmap(img)
        fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        icon = QIcon(fitPixmap)
        self.actionPlayPause.setIcon(icon)
        self.setActionEnable(False)
        self.actionPlayPause.setEnabled(True)
        self.timerUpdate.start()

    def slotRestart(self):
        MyLog.writeLog("{}[:{}] - restart...".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
        self.setCursor(QtCore.Qt.WaitCursor)
        self.stLidarParam.setNoPcData(False)
        self.slotStopPlay()
        self.slotStartPlay()
        self.timerUpdate.start()

    def slotPrevious(self):
        # print("slotPrevious() start..")
        # 关闭点云和视频显示
        self.bAuto = False
        self.timerUpdate.stop()
        self.actionPlayPause.setText(self._translate("WJ-DepthViewer", "Play"))
        #style = QApplication.style()
        #icon = style.standardIcon(QtWidgets.QStyle.SP_MediaPlay)
        img = QtGui.QImage(strExePath + "/Configs/Icon/play.png")
        pixmap = QtGui.QPixmap(img)
        fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        icon = QIcon(fitPixmap)
        self.actionPlayPause.setIcon(icon)

        if self.bSaveAll:
            self.slotSaveAll()
        self.openGLWidget.pause()
        self.openGLWidget.previous()
        self.timerUpdate.start(10)

        # 修改界面相关标志位和文字
        self.bPause = True
        # print("self.bPause = True 3")


        MyLog.writeLog("{}[:{}] - slotPrevious!".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                        sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)

        # print("slotPrevious() end..")

    def slotNext(self):
        # print("slotNext() end..")
        self.bAuto = False
        self.actionPlayPause.setText(self._translate("WJ-DepthViewer", "Play"))
        # style = QApplication.style()
        # icon = style.standardIcon(QtWidgets.QStyle.SP_MediaPlay)
        img = QtGui.QImage(strExePath + "/Configs/Icon/play.png")
        pixmap = QtGui.QPixmap(img)
        fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        icon = QIcon(fitPixmap)
        self.actionPlayPause.setIcon(icon)

        # 关闭点云和视频显示
        self.timerUpdate.stop()
        if self.bSaveAll:
            self.slotSaveAll()
        self.openGLWidget.pause()
        self.openGLWidget.next()
        self.timerUpdate.start(10)

        # 修改界面相关标志位和文字
        self.bPause = True
        # print("self.bPause = True 4")

        MyLog.writeLog("{}[:{}] - slotPrevious!".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                        sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)

        # print("slotNext() end..")

    def slotCheckIP(self):
        if not my_isExist(ipTruePath):
            return

        self.listDevIp = []
        fTrue = open(ipTruePath, 'r')
        for ip in fTrue.readlines():
            ip = ip.replace('\n', '')  # 替换掉换行符
            self.listDevIp.append(ip)

        if len(self.listDevIp) == 0:
            return

        bChanged = False
        listForwardDstDevTemp = self.stLidarParam.getListForwardDstDev()
        listForwardEventDstDevTemp = self.stLidarParam.getListForwardEventDstDev()
        for strDev in listForwardDstDevTemp:
            nameList = strDev[0].split('-')  # 分割字符串
            strDevIp = nameList[0]
            if strDevIp in self.listDevIp:
                if not strDev[1]:
                    strDev[1] = True
                    bChanged = True
            else:
                if strDev[1]:
                    strDev[1] = False
                    bChanged = True

        for strDev in listForwardEventDstDevTemp:
            nameList = strDev[0].split('-')  # 分割字符串
            strDevIp = nameList[0]
            if strDevIp in self.listDevIp:
                if not strDev[1]:
                    strDev[1] = True
                    bChanged = True
            else:
                if strDev[1]:
                    strDev[1] = False
                    bChanged = True

        if bChanged:
            MyLog.writeLog("{}[:{}] - ip reachable changed! setParamChanged(True)!".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                      sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
            self.stLidarParam.setPause(True)
            self.stLidarParam.setListForwardDstDev(listForwardDstDevTemp)
            self.stLidarParam.setListForwardEventDstDev(listForwardEventDstDevTemp)
            self.stLidarParam.setParamChanged(True)
            self.stLidarParam.setPause(False)

    def slotNetStatusChanged(self):
        try:
            if not self.bOnline or not my_isExist(ipTruePath):
                self.slotCheckIP()
                return

            # listHostIp = get_all_ip_address()
            self.slotCheckIP()
            if len(self.listDevIp) > 0:
                if self.stLidarParam.getStationSrcIp(0) not in self.listDevIp:
                    self.titlePc.setText(self._translate("WJ-DepthViewer", "Point cloud  [Warning:Lidar network anomaly!]"))
                else:
                    if self.stLidarParam.getNoPcData():
                        self.titlePc.setText(self._translate("WJ-DepthViewer", "Point cloud  [Warning:Please check if the lidar1 mac binding or IP filling is correct!]"))
                    else:
                        self.titlePc.setText(self._translate("WJ-DepthViewer", "Point cloud"))
            else:
                if self.stLidarParam.getNoPcData():
                    self.titlePc.setText(self._translate("WJ-DepthViewer", "Point cloud  [Warning:Lidar1 network anomaly!]"))
                else:
                    self.titlePc.setText(self._translate("WJ-DepthViewer", "Point cloud"))
        except:
            MyLog.writeLog("{}[:{}] - except! slotNetStatusChanged err!"
                           .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno)
                            ,LOG_ERROR, ENABLE_WRITE_LOG, ENABLE_PRINT)

            MyLog.writeLog("{}[:{}] - except! Call stack:\n{}!"
                           .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno,
                                   traceback.format_exc()), LOG_ERROR, ENABLE_WRITE_LOG, ENABLE_PRINT)

    def slotSurvivalCheck(self):
        try:
            startWatchDog()
            mem, lisPidMemOut = getProMemByName()
            strMem = str(mem) + 'MB'
            # strMem = str(int(psutil.virtual_memory().used / 1024 / 1024)) + 'MB'  # 总内存占用
            MyLog.writeLog("{}[:{}] - Survival check! Mem={}, [pid, mem]:{}".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                            sys._getframe().f_lineno, strMem, lisPidMemOut), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
            # MyLog.writeLog("{}[:{}] - Survival check!".format(__file__.split('/')[len(__file__.split('/')) - 1],
            #                                                 sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)

            if stUiParam.bDebug:
                strVersion = self._translate("WJ-DepthViewer", "{} Debug! Mem:{}".format(PACK_VERSION_DATE_DEBUG, strMem))
            else:
                strVersion = self._translate("WJ-DepthViewer", "{} Mem:{}".format(PACK_VERSION_DATE_DEBUG, strMem))

            self.labVersion.setText(strVersion)

            if self.ipCheckPro is None or (not self.ipCheckPro.is_alive()):
                self.ipCheckPro = Process(target=ipCheckProcess, args=(self.mp_bRuning, self.stLidarParam,))
                self.ipCheckPro.start()
            # self.actionTimeLeft.setText(strMem)
            # 当重启开关打开，且帧率低于指定值，且内存占用或显存占用超过指定值时，需要重启软件
            bRestart = isNeedRestartApp(self.fFps)
            if bRestart:
                self.slotQuit(True)
        except:
            MyLog.writeLog("{}[:{}] - except! slotSurvivalCheck err!!"
                           .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno)
                            ,LOG_ERROR, ENABLE_WRITE_LOG, ENABLE_PRINT)

            MyLog.writeLog("{}[:{}] - except! Call stack:\n{}!"
                           .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno,
                                   traceback.format_exc()), LOG_ERROR, ENABLE_WRITE_LOG, ENABLE_PRINT)

    def notEnoughSpace(self, fSpace=0, strMsg=''):
        MyLog.writeLog("{}[:{}] - Free disk space {}mb is less than {}mb!"
                       .format(__file__.split('/')[len(__file__.split('/')) - 1],
                               sys._getframe().f_lineno, fSpace, MIN_SPACE_LIMIT), LOG_INFO, ENABLE_WRITE_LOG, True)
        if self.msgBox is None:
            self.msgBox = QMessageBox(QMessageBox.Warning, self._translate("WJ-DepthViewer", 'Warning'), self._translate("WJ-DepthViewer",
                                "The disk space is less than")+ "{}mb, {}".format(MIN_SPACE_LIMIT, strMsg), QMessageBox.Ok)
            self.msgBox.setModal(True)

        if self.msgBox.isHidden():
            self.msgBox.show()

    # def setLidarList(self, newLidarCount, newRadarCount):
    #     if self.LidarCount == newLidarCount and self.RadarCount == newRadarCount:
    #         return
    #     else:
    #         self.hBoxLayout.removeWidget(self.btnPcSetting)
    #         self.btnPcSetting.setParent(None)
    #         self.hBoxLayout.removeWidget(self.titleShowLidarBoxIp)
    #         self.titleShowLidarBoxIp.setParent(None)
    #         LidarLabel = []
    #         LidarTitle = []
    #         for i in range(newLidarCount):
    #             LidarLabel.append(self._translate("WJ-DepthViewer", "Lidar"+ str(i+1)))
    #         for i in range(newRadarCount):
    #             LidarLabel.append(self._translate("WJ-DepthViewer", "Radar"))
    #
    #     self.titleShowLidarBoxIp = ComboCheckBox(LidarLabel)
    #     self.titleShowLidarBoxIp.setObjectName("titleShowLidarBoxIp")
    #     self.titleShowLidarBoxIp.setStyleSheet("width: 90px; height: 15px; font-size: 12px;background-color:#515151;")
    #     self.hBoxLayout.addWidget(self.titleShowLidarBoxIp)
    #     self.hBoxLayout.addWidget(self.btnPcSetting)
    #     self.LidarCount = newLidarCount
    #     self.RadarCount = newRadarCount


    def slotTimeOut(self):
        time1 = time.time()
        try:
            if self.bAuto and self.timerUpdate.interval() < TIME_INTERVAL:
                self.timerUpdate.start(TIME_INTERVAL)

            nTime = self.timeNoData.elapsed()
            bNoPc = self.stLidarParam.getNoPcData()
            # self.onNetStatusChanged()  # 网络状态发生改变时，要做出响应
            if self.stLidarParam.getNoPcData():
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
                self.statusbar.showMessage(self._translate("WJ-DepthViewer", "Loading error!"), 0)

                if not self.bPause:
                    self.bPause = True
                    # print("self.bPause = True 5")
                    # style = QApplication.style()
                    # icon = style.standardIcon(QtWidgets.QStyle.SP_MediaPlay)
                    img = QtGui.QImage(strExePath + "/Configs/Icon/play.png")
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
            if self.stSrcPcResultQueue.qsize() == 0:
                # print("self.stDstAllResultQueue.qsize() == 0:...")
                if self.timeNoData.elapsed() < 1500:
                    return

                if self.bAuto:
                    if not self.actionPrevious.isEnabled():
                        self.setActionEnable(False)
                        self.actionPlayPause.setEnabled(True)

                    if self.bOnline or (not self.stLidarParam.getNoPcData() and not self.openGLWidget.isPlayOver()):
                        if self.cursor().shape() != QtCore.Qt.WaitCursor:
                            self.setCursor(QtCore.Qt.WaitCursor)
                        self.statusbar.showMessage(self._translate("WJ-DepthViewer", "Loading data..."), 0)  # 状态栏本身显示的信息 第二个参数是信息停留的时间，单位是毫秒，默认是0（0表示在下一个操作来临前一直显示）
                        # MyLog.writeLog("{}[:{}] - bOnline:{}, pcOver:{}, vidOver:{}, NoPc:{}, NoVid:{},"
                        #                " PcResultSize:{}, AllResultSize:{}, PcStatus:{}, VidStatus:{}!"
                        #                .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno,
                        #                        self.bOnline, self.openGLWidget.isPlayOver(), self.videoWidget.isPlayOver(),
                        #                        self.stLidarParam.getNoPcData(), self.stCameraParam.getNoVideoData(),
                        #                        self.stSrcPcResultQueue.qsize(), self.stDstAllResultQueue.qsize(),
                        #                        self.stLidarParam.getStatus(), self.stCameraParam.getStatus()),
                        #                LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)

                    if (not self.bOnline) and (self.openGLWidget.isPlayOver() or self.stLidarParam.getNoPcData())\
                            and self.stLidarParam.getStatus() in [5, 6]:
                        if self.cursor().shape() != QtCore.Qt.ArrowCursor:
                            self.setCursor(QtCore.Qt.ArrowCursor)
                        self.statusbar.showMessage(self._translate("WJ-DepthViewer", "Play ended!"), 0)  # 状态栏本身显示的信息 第二个参数是信息停留的时间，单位是毫秒，默认是0（0表示在下一个操作来临前一直显示）
                        if self.checkBoxLoop.checkState() == QtCore.Qt.Checked:
                            print("View: start Loop...")
                            # self.slotStopPlay()
                            self.openGLWidget.setAutoLoopPlay(True)

                if not self.bPause:
                    self.bPause = True
                    # print("self.bPause = True 5")

                    # style = QApplication.style()
                    # icon = style.standardIcon(QtWidgets.QStyle.SP_MediaPlay)
                    img = QtGui.QImage(strExePath + "/Configs/Icon/play.png")
                    pixmap = QtGui.QPixmap(img)
                    fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                              QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
                    icon = QIcon(fitPixmap)
                    self.actionPlayPause.setIcon(icon)

                # self.timeNoData.restart()
                return

            self.timeNoData.restart()
            # print("self.bPause = False 1")

            if self.bAuto:
                self.bPause = False
                # style = QApplication.style()
                # icon = style.standardIcon(QtWidgets.QStyle.SP_MediaPause)
                img = QtGui.QImage(strExePath + "/Configs/Icon/pause.png")
                pixmap = QtGui.QPixmap(img)
                fitPixmap = pixmap.scaled(24, 24, QtCore.Qt.IgnoreAspectRatio,
                                          QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
                icon = QIcon(fitPixmap)
                self.actionPlayPause.setIcon(icon)
            self.setActionEnable(True)

            self.actionStationRegistration.setEnabled(not self.stLidarParam.getShowRLFusion() and not self.stLidarParam.getCoordinateFlag())

            if self.cursor().shape() == QtCore.Qt.WaitCursor:
                self.setCursor(QtCore.Qt.ArrowCursor)
                self.statusbar.showMessage(self._translate("WJ-DepthViewer", "Loading is complete!"), 3000)  # 状态栏本身显示的信息 第二个参数是信息停留的时间，单位是毫秒，默认是0（0表示在下一个操作来临前一直显示）

            # print("dstData size = ", self.stDstAllResultQueue.qsize())
            # print("view get() start")
            try:
                self.stDstData = self.stSrcPcResultQueue.get_nowait()
            except:
                return


                # usStatusTemp = self.stLidarParam.getStatus()
                # if usStatusTemp == 0 or 3 <= usStatusTemp <= 5:
                #     self.stLidarParam.setFrameId(self.stLidarParam.getFrameId() + 1)
            if self.stLidarParam.getNoPcData():
                if not self.bOnline:
                    listPcOver = np.full(len(self.stLidarParam.getListOfflineFiles()), True, bool)
                    self.stLidarParam.setPcOver(listPcOver)
                    self.titlePc.setText(self._translate("WJ-DepthViewer", "Point cloud  [Warning: No point cloud file or the file is damaged!]"))
            elif "No point cloud file" in self.titlePc.text():
                self.titlePc.setText(self._translate("WJ-DepthViewer", "Point cloud"))

            if self.stLidarParam.getNoPcData():
                self.stDstData = None

            if self.tStart is not None:
                print("elapsed = %.3fms" % ((time.time() - self.tStart) * 1000))
                self.tStart = None

            nFrameCount = self.stLidarParam.getLidarFrameCount()
            bOnline = self.stLidarParam.getOnline()
            if (nFrameCount > 0) and (not bOnline):
                self.sliderFrameId.setEnabled(True)
                self.spFrameId.setEnabled(True)
                self.sliderFrameId.setRange(0, nFrameCount - 1)
                self.spFrameId.setRange(0, nFrameCount - 1)
            else:
                self.sliderFrameId.setEnabled(False)
                self.spFrameId.setEnabled(False)


            # # test
            # gc.collect()
            # print('================== 1 ===================')
            # objgraph.show_growth()
            # tUpdate = time.time()
            self.bFirstNoData = True  # 用于控制在没有任何数据时，刷新一次显示区域，使显示区域变成黑屏
            if self.stDstData is not None:
                self.stDstData.filterList = self.comboType.get_selected()
            self.nFrameId= self.openGLWidget.slotTimerOut(self.stDstData)
            if self.stLidarParam.getNoPcData() and self.stDstData is not None:
                self.nFrameId = self.stDstData.nFrameId
            self.slotUpdateTableWidget()
            if (not bOnline):
                self.sliderFrameId.sliderReleased.disconnect(self.slotSkip)
                self.spFrameId.valueChanged.disconnect(self.slotSkip)
                self.sliderFrameId.setValue(self.nFrameId)
                self.spFrameId.setValue(self.nFrameId)
                self.sliderFrameId.sliderReleased.connect(self.slotSkip)
                self.spFrameId.valueChanged.connect(self.slotSkip)
            # else:
            #     self.sliderFrameId.setValue(0)
            #     self.spFrameId.setValue(0)
            # listIdUsed = [self.stDstData.pcData[len(self.stDstData.pcData) - 1]]
            # my_save2csv("test.csv", listIdUsed)

            # print("Update time={}, stDstAllResultQueue.qsize()={}".format(round((time.time()-tUpdate)*1000, 3), self.stDstAllResultQueue.qsize()))
            # gc.collect()
            # print('================== 2 ===================')
            # objgraph.show_growth()

            # # # test
            # if self.nFrameId > 100:
            #     self.slotQuit(True)
        except:
            MyLog.writeLog("{}[:{}] - except! slotTimeOut err!"
                           .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno)
                            ,LOG_ERROR, ENABLE_WRITE_LOG, ENABLE_PRINT)

            MyLog.writeLog("{}[:{}] - except! Call stack:\n{}!"
                           .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno,
                                   traceback.format_exc()), LOG_ERROR, ENABLE_WRITE_LOG, ENABLE_PRINT)
        # print("update time = %fms" % ((time.time() - time1) * 1000))
        # print("timeout = %d ms" % self.timeTemp.elapsed())
        self.timeTemp.restart()

    def slotInitiativeAuthorization(self):
        dlgAction = ActionTipDlg(stLidarParam=self.stLidarParam)
        if dlgAction.exec() == QDialog.Accepted:
            self.bActivated = True
            self.timerCheck.singleShot(1000, self.slotCheckAuthorization)

    def slotCheckAuthorization(self):
        return
        try:
            fSpace = getHdRemainingSpace("/")
            if fSpace < MIN_SPACE_LIMIT:
                self.slotStopPlay()
                self.notEnoughSpace(fSpace, "please free up some space!")
            elif self.bStop:
                self.msgBox.hide()
                self.slotStartPlay()

            strMsg = self._translate("WJ-DepthViewer", "Trial time has ended, please enter the activation code!")
            bOk, bFirst, nDaysLeft = my_encode_check(strCheckAuthorizationPath)
            if nDaysLeft == "Activated":
                self.actionTimeLeft.setText(self._translate("WJ-DepthViewer", "Activated!"))
            else:
                nDaysLeft = max(0, nDaysLeft)
                #self.actionTimeLeft.setText(self._translate("WJ-DepthViewer", "Time left"))
                #self.actionTimeLeft.setText(self._translate("WJ-DepthViewer", "Time left:"+"{}".format(nDaysLeft)+"days"))
                self.actionTimeLeft.setText(
                    self._translate("WJ-DepthViewer", "Time left:") + "{}".format(nDaysLeft) + self._translate("WJ-DepthViewer","days"))
            if bOk and not bFirst:
                self.bActivated = True
                return

            self.bActivated = False
            if bFirst:
                strMsg = self._translate("WJ-DepthViewer", "Please enter the activation code!")

            self.slotStopPlay()
            self.timerCheck.stop()
            try:
                dlgAction = ActionTipDlg(strMsg, stLidarParam=self.stLidarParam)
                if dlgAction.exec() == QDialog.Accepted:
                    self.bActivated = True
                    self.timerCheck.singleShot(1000, self.slotCheckAuthorization)
                    self.timerCheck.start()
                    self.slotStartPlay()
                    return
                else:
                    self.slotQuit(True)
            except:
                self.timerCheck.start()
        except:
            MyLog.writeLog("{}[:{}] - except! slotCheckAuthorization err!"
                           .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno)
                            ,LOG_ERROR, ENABLE_WRITE_LOG, ENABLE_PRINT)

            MyLog.writeLog("{}[:{}] - except! Call stack:\n{}!"
                           .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno,
                                   traceback.format_exc()), LOG_ERROR, ENABLE_WRITE_LOG, ENABLE_PRINT)

    def slotEditChange(self):
        axis = [0.0, 0.0, 1.0]
        self.openGLWidget.rotate(axis, float(self.editRotationAngle.text()))
        return

    def slotLoopChanged(self, nStatus):
        if nStatus == QtCore.Qt.Unchecked:
            self.openGLWidget.setAutoLoopPlay(False)
            MyLog.writeLog("{}[:{}] - setAutoLoopPlay(False)!".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                      sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
        else:
            self.openGLWidget.setAutoLoopPlay(True)
            MyLog.writeLog("{}[:{}] - setAutoLoopPlay(True)!".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                      sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)

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

    def slotShowCoordinateFlagChanged(self, nStatus):
        if nStatus == QtCore.Qt.Unchecked:
            self.stLidarParam.setCoordinateFlag(False)
        else:
            self.stLidarParam.setCoordinateFlag(True)

    def slotShowStationFusionChanged(self, nStatus):
        if nStatus == QtCore.Qt.Unchecked:
            self.openGLWidget.setShowRLFusion(False)
            self.stLidarParam.setShowRLFusion(False)
        else:
            self.openGLWidget.setShowRLFusion(True)
            self.stLidarParam.setShowRLFusion(True)

    def slotShowTableChanged(self, nStatus):
        if nStatus == QtCore.Qt.Unchecked:
            self.subSplitterRight.setVisible(False)
        else:
            self.subSplitterRight.setVisible(True)

    def slotSaveBoxInfoChanged(self, nStatus):
        if nStatus == QtCore.Qt.Unchecked:
            # cancel save box info
            self.strSaveBoxInfoFilePath = ""
        else:
            # save box info
            strSaveDir = time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
            self.strSaveBoxInfoFilePath = '{}{}'.format(csvPath, strSaveDir)
            my_mkdir(self.strSaveBoxInfoFilePath)
            fSpace = getHdRemainingSpace(self.strSaveBoxInfoFilePath)
            if fSpace < MIN_SPACE_LIMIT:
                my_delDir(self.strSaveBoxInfoFilePath)
                self.checkBoxSaveBoxInfo.setCheckState(QtCore.Qt.Unchecked)
                self.notEnoughSpace(fSpace, "the saving of data has been stopped!")
                return

    def slotPcSetting(self):
        self.pcSettingDlg.exec()

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
        print('nSkipToFrameId',nSkipToFrameId)
        self.bAuto = False
        self.timerUpdate.stop()
        if self.bSaveAll:
            self.slotSaveAll()

        # self.stLidarParam.setPcResultReady(False)
        # self.stCameraParam.setPcResultReady(False)

        self.openGLWidget.pause()
        self.openGLWidget.skipTo(nSkipToFrameId)
        self.timerUpdate.start(stUiParam.nTimeInterval)

        # 修改界面相关标志位和文字
        self.bPause = True
        # print("self.bPause = True 3")

        self.actionPlayPause.setText(self._translate("WJ-DepthViewer", "Play"))

        #style = QApplication.style()
        #icon = style.standardIcon(QtWidgets.QStyle.SP_MediaPlay)
        img = QtGui.QImage(strExePath + "/Configs/Icon/play.png")
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
        # self.sigSelectBox.emit(self.nSelectBoxId)
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

    def slotAddaInfo(self):
        if self.bStationRegistration:
            if self.nAddClickCount == 0:
                self.btnStartSelectCube1.setVisible(True)
                self.btnStartSelectCube2.setVisible(True)
                self.oldPcCount = 0
            self.nAddClickCount += 1
            self.listSelectData[0].append([])
            self.listSelectData[1].append([])

            self.bSelectCube1 = False
            self.bSelectCube2 = False
            self.openGLWidget.setSelectCube(False)

            self.tableWidgetSelectCube.setRowCount(self.nAddClickCount)
            print("tableWidgetSelectPoints add one raw")

    def slotShowSelect(self):
        try:
            if self.bStationRegistration:
                listMajorCubeCenter_temp = []
                listMinorCubeCenter_temp = []
                for i in range(self.nAddClickCount):
                    if self.tableWidgetSelectCube.item(i, 0) is None:
                        listMajorCubeCenter_temp.append(None)
                    else:
                        oldCubeCenter_temp = self.tableWidgetSelectCube.item(i, 0).text()
                        listMajorCubeCenter_temp.append([float(x) for x in oldCubeCenter_temp.split(',')])

                    if self.tableWidgetSelectCube.item(i, 1) is None:
                        listMinorCubeCenter_temp.append(None)
                    else:
                        newCubeCenter_temp = self.tableWidgetSelectCube.item(i, 1).text()
                        listMinorCubeCenter_temp.append([float(x) for x in newCubeCenter_temp.split(',')])

                pop_num = 0
                for i in range(len(listMajorCubeCenter_temp)):
                    i -= pop_num
                    if listMajorCubeCenter_temp[i] is None or listMinorCubeCenter_temp[i] is None:
                        listMajorCubeCenter_temp.pop(i)
                        listMinorCubeCenter_temp.pop(i)

                self.openGLWidget.showSelect([listMajorCubeCenter_temp,listMinorCubeCenter_temp],self.listSelectData)
        except:
            print(traceback.print_exc())
            MyLog.writeLog("{}[:{}] - Radar and lidar selected show Error".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                                  sys._getframe().f_lineno), LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)

    def slotSaveSelect(self):
        try:
            if self.bStationRegistration:
                listMajorCubeCenter_temp = []
                listMinorCubeCenter_temp = []
                for i in range(self.nAddClickCount):
                    if self.tableWidgetSelectCube.item(i, 0) is None:
                        listMajorCubeCenter_temp.append(None)
                    else:
                        oldCubeCenter_temp = self.tableWidgetSelectCube.item(i, 0).text()
                        listMajorCubeCenter_temp.append([float(x) for x in oldCubeCenter_temp.split(',')])

                    if self.tableWidgetSelectCube.item(i, 1) is None:
                        listMinorCubeCenter_temp.append(None)
                    else:
                        newCubeCenter_temp = self.tableWidgetSelectCube.item(i, 1).text()
                        listMinorCubeCenter_temp.append([float(x) for x in newCubeCenter_temp.split(',')])

                pop_num = 0
                for i in range(len(listMajorCubeCenter_temp)):
                    i -= pop_num
                    if listMajorCubeCenter_temp[i] is None or listMinorCubeCenter_temp[i] is None:
                        listMajorCubeCenter_temp.pop(i)
                        listMinorCubeCenter_temp.pop(i)
                        self.tableWidgetSelectCube.removeRow(i)
                        self.nAddClickCount -= 1
                        self.nAldCubeCount -= 1
                        self.noldPcCount -= 1
                        pop_num += 1
                if len(listMajorCubeCenter_temp) == 0 or len(listMinorCubeCenter_temp) == 0:
                    print("Seleted Point is None")
                    self.nAddClickCount = 0
                    self.nAldCubeCount = 0
                    self.noldPcCount = 0
                    self.tableWidgetSelectCube.setRowCount(self.nAddClickCount)
                    return

                npMajorCubeCenter = np.array(listMajorCubeCenter_temp)
                npMinorCubeCenter = np.array(listMinorCubeCenter_temp)

                fRotationVector = self.stLidarParam.getRotationVector(1)
                fTranslationVector = self.stLidarParam.getTranslationVector(1)
                npCenter = npMinorCubeCenter - np.array([fTranslationVector[0], fTranslationVector[1], fTranslationVector[2]]).reshape(1, 3)
                rotation = np.array([-fRotationVector[0] * PI_rads, -fRotationVector[1] * PI_rads, -fRotationVector[2] * PI_rads])
                #########rotate point ##################
                rotation_pcd = o3d.geometry.PointCloud()
                rotation_pcd.points = o3d.utility.Vector3dVector(npCenter)
                rotation_pcd.rotate(rotation=rotation,center=False)
                npMinorCubeCenter = np.asarray(rotation_pcd.points)

                # Major_R = np.array([0.99563347,-0.09334878,0.0],
                #                    [0.09334878,-0.99563347,0.0],
                #                    [0.0,0.0,1.0])
                # Major_T = np.array([-25.9413007,-11.1392702,-3.9])
                # Minor_R = np.array([],
                #                    [],
                #                    [])
                # Minor_T = np.array([])
                #
                # npMajorCubeCenter = np.multiply((npMajorCubeCenter - Major_T), np.linalg.inv(Major_R.T))

                file = open(getExcutePath() + "/Configs/Dev/StationCenter.txt", 'w')
                strRoadData = file.write(str([npMajorCubeCenter,npMinorCubeCenter]))
                file.close()

                # self.stLidarParam.setRadarRotationVector(Rotation, 0)
                # self.stLidarParam.setRadarTranslationVector(Translation, 0)
                # serializationParam(self.stLidarParam.getParam(), bSave=True)
        except:
            print(traceback.print_exc())
            MyLog.writeLog("{}[:{}] - Selected save Error".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                                  sys._getframe().f_lineno), LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)


    def slotSelectStationBox(self, listSelectStationBox):
        bit_num = 5
        [SelectBoxCenter, SelectBoxVertex] = listSelectStationBox

        x_temp = SelectBoxCenter[0]
        y_temp = SelectBoxCenter[1]
        z_temp = SelectBoxCenter[2]
        item = QtWidgets.QTableWidgetItem()
        item.setText(str(round(x_temp,bit_num)) + "," + str(round(y_temp,bit_num)) + "," + str(round(z_temp,bit_num)))
        if self.bSelectCube1:
            self.listSelectData[0][self.nAldCubeCount - 1] = [SelectBoxCenter, SelectBoxVertex]
            self.tableWidgetSelectCube.setItem(self.nAddClickCount - 1, 0, item)
        elif self.bSelectCube2:
            self.listSelectData[1][self.nAldCubeCount - 1] = [SelectBoxCenter, SelectBoxVertex]
            self.tableWidgetSelectCube.setItem(self.nAddClickCount - 1, 1, item)
        self.slotSelectCube()
        self.slotShowSelect()


    def slotSelectCellClicked(self, nRow, nCol):
        self.openGLWidget.setSelectedPoint(nRow, nCol)
        self.slotShowSelect()

    def slotSelectCube1(self):
        self.bSelectCube1 = True
        self.slotSelectCube()

    def slotSelectCube2(self):
        self.bSelectCube2 = True
        self.slotSelectCube()


    def slotSelectCube(self):
        self.bSelectCube = not self.bSelectCube
        self.openGLWidget.setSelectCube(self.bSelectCube)
        if self.bSelectCube1:
            self.bSelectCube1 = self.bSelectCube
            self.btnStartSelectCube1.setChecked(self.bSelectCube)
            self.btnStartSelectCube2.setEnabled(not self.bSelectCube)
            if self.bSelectCube:
                self.btnStartSelectCube1.setStyleSheet("border:0px;background-color:#FFFFFF")
            else:
                self.btnStartSelectCube1.setStyleSheet("border:0px;background-color:#515151")
        elif self.bSelectCube2:
            self.bSelectCube2 = self.bSelectCube
            self.btnStartSelectCube2.setChecked(self.bSelectCube)
            self.btnStartSelectCube1.setEnabled(not self.bSelectCube)
            if self.bSelectCube:
                self.btnStartSelectCube2.setStyleSheet("border:0px;background-color:#FFFFFF")
            else:
                self.btnStartSelectCube2.setStyleSheet("border:0px;background-color:#515151")

    def slotStationRegistration(self):
        self.bStationRegistration = not self.bStationRegistration
        self.actionStationRegistration.setChecked(self.bStationRegistration)
        self.openGLWidget.setRegistration(self.bStationRegistration)
        self.titleStation.setVisible(self.bStationRegistration)
        self.tableWidgetSelectCube.setVisible(self.bStationRegistration)
        if self.bStationRegistration:
            self.listSelectData = [[], []]
        else:
            self.nAddClickCount = 0
            self.tableWidgetSelectCube.setRowCount(self.nAddClickCount)
            self.btnStartSelectCube1.setVisible(False)
            self.btnStartSelectCube2.setVisible(False)

            self.bSelectCube1 = False
            self.bSelectCube2 = False
            self.openGLWidget.setSelectCube(False)
            self.openGLWidget.setshowSelectCube(False)
        self.slotPlayPause()

    def slotUpdateTableWidget(self):
        timeTemp = QTime()
        timeTemp.start()
        if self.stDstData is None:
            return

        if self.bFirstTest:
            self.timeTest.start()
            self.bFirstTest = False

        box_points = []
        box_id = []
        # if self.strSaveBoxInfoFilePath != "":
        #     # 传出目标框内的点
        #     # tr = time.time()
        #     indices_points = box_np_ops.points_in_rbbox(self.stDstData.pcData[0][:, :3], self.stDstData.pcTrackersCov)
        #     if len(indices_points.shape) > 1:
        #         for m in range(indices_points.shape[1]):
        #             # if in_points[indices_points[:, m], :].shape[0] > 0:
        #             # box_temp=in_points[indices_points[:, m], :]
        #             box_points.append(self.stDstData.pcData[0][indices_points[:, m], :])
        #             box_id.append(self.stDstData.pcTrackersCov[m, -1])
        #     # print("out elapsed = %.2fms" % ((time.time() - tr) * 1000))

        if stUiParam.bRotatingLane:
            listBoxInfo = copy.deepcopy(self.stDstData.listBoxInfo)
        else:
            listBoxInfo = copy.deepcopy(self.stDstData.listBaseBoxInfo)

        nPcBoxCount = len(listBoxInfo)
                # for nIndex in range(len(listVideoBoxInfoOneFrame)):
                #     boxInfo = listVideoBoxInfoOneFrame[nIndex]
                #     if self.strSaveBoxInfoFilePath != "" and boxInfo.strSource == "fusion" and boxInfo.strClass == "car":
                #         strSaveBoxInfoFilePath = self.strSaveBoxInfoFilePath + "/BoxId_{}/Channel_{}".format(int(boxInfo.boxId), i+1)
                #         my_mkdir(strSaveBoxInfoFilePath)
                #         boxVertex = boxInfo.boxVertex
                #         imgOriSize = [videoResultFrame.shape[1], videoResultFrame.shape[0]]
                #         if imgOriSize[0] != 1920 or imgOriSize[1] != 1080:
                #             size = np.array([1920, 1080])
                #             xOri1 = boxVertex[0] * size[0] / imgOriSize[0]
                #             yOri1 = boxVertex[1] * size[1] / imgOriSize[1]
                #             xOri2 = boxVertex[2] * size[0] / imgOriSize[0]
                #             yOri2 = boxVertex[3] * size[1] / imgOriSize[1]
                #             boxVertex = [xOri1, yOri1, xOri2, yOri2]
                #
                #         boxInfo2csv = [int(self.nFrameId), int(boxInfo.boxId), boxVertex]
                #         filePath = strSaveBoxInfoFilePath + '/BoxId_{}.csv'.format(int(boxInfo.boxId))
                #         # strSaveBoxInfoFilePath = '{}{}{}'.format(strSaveBoxInfoFilePath, strSaveFileName, '.csv')
                #         my_save2csv(filePath, boxInfo2csv)
                #         # cropped = self.stDstData.listVideoData[i][0][int(round(boxVertex[1])):int(round(boxVertex[3])),
                #         #           int(round(boxVertex[0])):int(round(boxVertex[2]))]  # 裁剪坐标为[y0:y1, x0:x1]
                #         # filePath = strSaveBoxInfoFilePath + '/img_Frame{}.jpg'.format(int(self.nFrameId))
                #         # cv2.imwrite(filePath, cropped)
                # print("save box time:%.2fms" % ((time.time() - tsave) * 1000))

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
        # self.nFrameId = self.nFrameId % MAX_FRAME_ID
        nEsapsed = self.timeTest.elapsed() - self.nLastTime
        if nEsapsed > 1000:
            nFrameCount = self.nFrameId - self.nLastFrameId
            if self.nFrameId < self.nLastFrameId:
                nTemp = MAX_FRAME_ID - self.nLastFrameId
                nFrameCount = self.nFrameId + nTemp
                # print("View: nFrameCount={}, nFrameIdCur={}, nFrameIdLast={}, fEsapsed={}, self.fFps={}".format(nFrameCount,
                #                                                                                    self.nFrameId,
                #                                                                                    self.nLastFrameId,
                #                                                                                    nEsapsed, self.fFps))

            self.fFps = round(nFrameCount / nEsapsed * 1000, 3)

            MyLog.writeLog("{}[:{}] - Fps:{}".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                     sys._getframe().f_lineno,self.fFps), LOG_INFO, ENABLE_WRITE_LOG,ENABLE_PRINT)

            if not self.bAuto:
                self.fFps = 1

            if self.fFps < 0 or self.fFps > 15:
                MyLog.writeLog("{}[:{}] - View: self.fFps ERROR, self.fFps={}, nFrameIdCur={}, nFrameIdLast={}, fEsapsed={}!"
                               .format(__file__.split('/')[len(__file__.split('/')) - 1],
                                       sys._getframe().f_lineno, self.fFps, self.nFrameId, self.nLastFrameId, nEsapsed / 1000),
                               LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
            self.timeTest.restart()
            self.nLastFrameId = self.nFrameId

        strFusionScaleByPc = ""

        nFusionCount = len(self.stDstData.listFusionId)
        fFusionScaleByPc = 0
        if nPcBoxCount > 0:
            fFusionScaleByPc = round(len(self.stDstData.listFusionId) / nPcBoxCount, 3)

        if stUiParam.bDebug:
            self.statusbar.showMessage(self._translate("WJ-DepthViewer", "Box:{}, Fps:{}"
                                                  .format(nRowCount, self.fFps)), 0)
        else:
            self.statusbar.showMessage(self._translate("WJ-DepthViewer", "Box:{}".format(nRowCount)), 0)

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
            # if boxInfo.strClass in self.stDstData.filterList:

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
                    boxInfoHead = ['Timestamp', 'FrameIdPcap', 'FrameId', 'boxId', 'boxCenter[0]', 'boxCenter[1]', 'boxCenter[2]',
                                   'boxSize[0]', 'boxSize[1]', 'boxSize[2]', 'boxSpeed', 'boxAngle', 'Class',
                                   'Color', 'boxLongitude', 'boxLatitude', 'Source', 'nConfidence']
                    my_save2csv(strAllPcBoxInfoFilePath, boxInfoHead)

                # 保存识别结果，目的是与转发出去的数据作对比，测试转发是否正常
                nFrameIdPcap = -1
                if self.stDstData.headData is not None:
                    nFrameIdPcap = int(self.stDstData.headData[3] * 256 + self.stDstData.headData[2])
                # boxInfo2csv = [int(self.nFrameId), int(boxInfo.boxId), boxInfo.boxSpeed, boxInfo.boxAngle, boxInfo.boxLongitude, boxInfo.boxLatitude]
                angle_north_t = self.stLidarParam.getAngleNorthT()
                boxAngle = (180 + angle_north_t + boxInfo.boxAngle / PI_rads) % 360  # 弧度转角度
                boxInfo2csv = [self.stDstData.pcData[2], nFrameIdPcap, int(self.nFrameId), int(boxInfo.boxId), boxInfo.boxCenter[0], boxInfo.boxCenter[1],
                               boxInfo.boxCenter[2], boxInfo.boxSize[0], boxInfo.boxSize[1], boxInfo.boxSize[2],
                               boxInfo.boxSpeed, boxAngle, getSendEnumFromStr(boxInfo.strClass),
                               getSendEnumFromColorStr(boxInfo.strColor), boxInfo.boxLongitude, boxInfo.boxLatitude,
                               boxInfo.strSource, boxInfo.nConfidence]

                fSpace = getHdRemainingSpace(self.strSaveBoxInfoFilePath)
                if fSpace < MIN_SPACE_LIMIT:
                    self.checkBoxSaveBoxInfo.setCheckState(QtCore.Qt.Unchecked)
                    self.notEnoughSpace(fSpace, "the saving of data has been stopped!")
                else:
                    my_save2csv(strAllPcBoxInfoFilePath, boxInfo2csv)

                # print("my_save2csv time = %.2fms" % ((time.time() - tCsv) * 1000))

                # if boxInfo.strSource == "fusion" and boxInfo.strClass == "car"
                # idTemp = -1
                # try:
                #     idTemp = box_id.index(boxInfo.boxId)
                # except:
                #     print("can't find the box! boxInfo.boxId={}, box_ids={}, listFusionId={}".format(boxInfo.boxId, box_id, self.stDstData.listFusionId))
                #     idTemp = -1
                #
                # if (idTemp >= 0) and (idTemp < len(box_points)):
                #     strSaveBoxInfoFilePath = self.strSaveBoxInfoFilePath + "/BoxId_{}/".format(int(boxInfo.boxId))
                #     my_mkdir(strSaveBoxInfoFilePath)
                #     filePath = strSaveBoxInfoFilePath + '/PC_BoxId_{}.csv'.format(int(boxInfo.boxId))
                #     npPoints = np.array(box_points[idTemp])
                #     boxInfo2csv = [int(self.nFrameId), int(boxInfo.boxId), npPoints]
                #     my_save2csv(filePath, boxInfo2csv)
                #     # print("my_save2csv time = %.2fms" % ((time.time() - tCsv) * 1000))
                #     # if boxInfo.strSource == "fusion":
                #     #     strSaveBoxInfoFilePath = self.strSaveBoxInfoFilePath + "_BoxId_{}".format(int(boxInfo.boxId))
                #     #     my_mkdir(strSaveBoxInfoFilePath)
                #     #     # for video in self.stDstData.listVideoData:
                #     #     #     if video is None:
                #     #     #         continue
                #     #
                #     #     boxVertex = boxInfo.boxVertex
                #     #     # print(boxVertex)
                #     #     cropped = self.stDstData.listVideoData[0][0][int(round(boxVertex[1])):int(round(boxVertex[3])), int(round(boxVertex[0])):int(round(boxVertex[2]))]  # 裁剪坐标为[y0:y1, x0:x1]
                #     #     filePath = strSaveBoxInfoFilePath + '/img_Frame{}.jpg'.format(int(self.nFrameId))
                #     #     cv2.imwrite(filePath, cropped)

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
            # if retType == float:
            if strLen != 'None' and strLen != 'N':
                # print("strLen round... ", i)
                # strLen = str(round(boxInfo.boxSize[1], 3))
                # strWidth = str(round(boxInfo.boxSize[0], 3))
                # strHeight = str(round(boxInfo.boxSize[2], 3))


                npCenter = np.array(boxInfo.boxCenter).reshape(1,3)
                if boxInfo.strSource == "Complement":
                    fRotationVector = self.stLidarParam.getRotationVector(1)
                    fTranslationVector = self.stLidarParam.getTranslationVector(1)

                    npCenter = npCenter - np.array([fTranslationVector[0], fTranslationVector[1], fTranslationVector[2]]).reshape(1,3)

                    rotation = np.array([-fRotationVector[0] * PI_rads, -fRotationVector[1] * PI_rads, -fRotationVector[2] * PI_rads])
                    #########rotate point ##################
                    rotation_pcd = o3d.geometry.PointCloud()
                    rotation_pcd.points = o3d.utility.Vector3dVector(npCenter)
                    rotation_pcd.rotate(rotation=rotation,center=False)
                    npCenter = np.asarray(rotation_pcd.points)
                strLen = str(round(npCenter[0, 0], 3))
                strWidth = str(round(npCenter[0, 1], 3))
                strHeight = str(round(npCenter[0, 2], 3))
                # print(strLen, strWidth, strHeight)

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
            #self.tableWidget.setColumnHidden(1, True)
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
            # else:
            #     nDelCount += 1
        self.tableWidget.setRowCount(nRowCount - nDelCount)
        self.tableWidget.setSortingEnabled(True)
        if nEsapsed > 1000:
            listSaveInfo = [datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"), nRowCount, nCarCount, nBicycleCount,
                            nPersonCount, self.fFps]


    def slotQuit(self, bSure=False):
        if not bSure:
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
                MyLog.writeLog("{}[:{}] - Quit..".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                         sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG,
                               ENABLE_PRINT)
                self.setEnabled(False)
                self.timerUpdate.stop()
                self.slotStop()
                self.sigQuit.emit()
                return True
            # ret = QMessageBox.question(self, self._translate("WJ-DepthViewer", 'Warning'), self._translate("WJ-DepthViewer", "Do you want to exit the software?"), QMessageBox.Ok | QMessageBox.Cancel)
            # if ret == QMessageBox.Ok:
            #     MyLog.writeLog("{}[:{}] - Quit..".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
            #     self.setEnabled(False)
            #     self.timerUpdate.stop()
            #     self.slotStop()
            #     self.sigQuit.emit()
            #     return True

            return False
        else:
            MyLog.writeLog("{}[:{}] - Quit..".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno),
                LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
            self.setEnabled(False)
            self.timerUpdate.stop()
            self.slotStop()
            startWatchDog()
            self.sigQuit.emit()
            return True

    def closeEvent(self, event):
        bQuit = self.slotQuit()
        if bQuit:
            my_killSelfSamePro()
            QMainWindow.closeEvent(self, event)
        else:
            event.ignore()


    def close_window(self, event):
        bQuit = self.slotQuit()
        if bQuit:
            my_killSelfSamePro()
            QMainWindow.closeEvent(self, event)
        else:
            event.ignore()