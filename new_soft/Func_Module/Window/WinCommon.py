# -*-coding:utf-8-*-
from __future__ import print_function

# PyQt的导入必须放在前面，否则release运行时会崩溃！
from PyQt5 import QtGui, QtCore, QtWidgets, QtOpenGL
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtCore import QTime, QPoint, QPointF, QRectF, QTranslator
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QMessageBox, QAbstractItemView, QDoubleSpinBox, \
    QSpacerItem, QSizePolicy, QWidget, QDialog
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen, QColor, QIcon, QBrush, QFont, QFontMetrics, QFontDatabase, \
    QDoubleValidator
from PyQt5.QtCore import pyqtSignal, QObject
import OpenGL

OpenGL.ERROR_CHECKING = True
from OpenGL.GL import *
from OpenGL.GLU import *
import open3d as o3d
import numpy.linalg as linalg
import qrcode
import re
from PIL import ImageQt
from multiprocessing.managers import BaseManager
import OpenGL.arrays.vbo as glvbo
from cureUi import cure

from Common.CommonDefine import *
from Common.CommonParam import *
from Common.CommonInterface import initLog, DataBus, DataBase

win_param_path = os.path.join(get_cfg_path(), 'modParam/win_Param.xml')
global_log = initLog(win_param_path)
strExePath = get_exe_path()


class structUiParam:
    def __init__(self):
        self.bDebug = False
        self.bAutoConnect = False
        self.nTimeInterval = 100
        self.fMemPercentMax = 90
        self.fGpuPercentMax = 90
        self.SoftwareStyle = 0
        self.nOfflineFrame = 5000
        self.nSaveResult = False
        self.s_laneFile_name = ''

    def update(self, s_cfg_path):
        """获取界面配置"""
        try:
            if not judge_exist(s_cfg_path):
                return
            tree = ET.parse(s_cfg_path)
            root = tree.getroot()
            self.bDebug = bool(int(root[2][0].text))
            self.bAutoConnect = bool(int(root[2][1].text))
            self.nTimeInterval = int(root[2][2].text)
            self.fMemPercentMax = float(root[2][3].text)
            self.fGpuPercentMax = float(root[2][4].text)
            self.SoftwareStyle = int(root[2][5].text)
            self.nOfflineFrame = int(root[2][6].text)
            self.nSaveResult = int(root[2][7].text)
            self.s_laneFile_name = root[2][8].text
        except:
            global_log.error("getUiParam Failed!\n{}\n".format(traceback.format_exc()))


stUiParam = structUiParam()
stUiParam.update('./Configs/win_Param.xml')


class structSysState(object):
    """系统状态结构体，用于整个界面的传输"""

    def __init__(self):
        self.n_sys_state = 0
        self.b_act_state = False
        self.b_online = False
        self.b_update_param = False
        self.s_offline_file = ''
        self.s_localId = ''
        self.s_act_code = ''
        self.n_active_res = 0
        self.s_timeLeft = 0
        self.b_noSpace = False

        self.b_pause = False
        self.b_running = False
        self.b_showBox = True
        self.b_showLane = True
        self.b_loop = False
        self.b_replay = False
        self.n_frameId = 0
        self.b_noData = False
        self.n_state = 0
        self.b_stop = False
        self.b_stip = False
        self.n_frameCount = 0

    def set_stip(self, b_stip):
        self.b_stip = b_stip

    def get_stip(self):
        return self.b_stip

    def set_frameCount(self, n_frameCount):
        self.n_frameCount = n_frameCount

    def get_frameCount(self):
        return self.n_frameCount

    def set_timeLeft(self, s_timeLeft):
        self.s_timeLeft = s_timeLeft

    def get_timeLeft(self):
        return self.s_timeLeft

    def set_stop(self, b_stop):
        self.b_stop = b_stop

    def get_stop(self):
        return self.b_stop

    def set_noSpace(self, b_noSpace):
        self.b_noSpace = b_noSpace

    def get_noSpace(self):
        return self.b_noSpace

    def set_sys_state(self, n_sys_state):
        self.n_sys_state = n_sys_state

    def get_sys_state(self):
        return self.n_sys_state

    def set_act_state(self, b_act_state):
        self.b_act_state = b_act_state

    def get_act_state(self):
        return self.b_act_state

    def set_online(self, b_online):
        self.b_online = b_online

    def get_online(self):
        return self.b_online

    def set_update_param(self, b_update_param):
        self.b_update_param = b_update_param

    def get_update_param(self):
        return self.b_update_param

    def set_offline_file(self, s_offline_file):
        self.s_offline_file = s_offline_file

    def get_offline_file(self):
        return self.s_offline_file

    def set_localId(self, s_localId):
        self.s_localId = s_localId

    def get_localId(self):
        return self.s_localId

    def set_act_code(self, s_act_code):
        self.s_act_code = s_act_code

    def get_act_code(self):
        return self.s_act_code

    def set_active_res(self, n_active_res):
        self.n_active_res = n_active_res

    def get_active_res(self):
        return self.n_active_res

    def set_pause(self, b_pause):
        self.b_pause = b_pause

    def get_pause(self):
        return self.b_pause

    def set_loop(self, b_loop):
        self.b_loop = b_loop

    def get_loop(self):
        return self.b_loop

    def set_running(self, b_running):
        self.b_running = b_running

    def get_running(self):
        return self.b_running

    def set_showBox(self, b_showBox):
        self.b_showBox = b_showBox

    def get_showBox(self):
        return self.b_showBox

    def set_showLane(self, b_showLane):
        self.b_showLane = b_showLane

    def get_showLane(self):
        return self.b_showLane

    def set_replay(self, b_replay):
        self.b_replay = b_replay

    def get_replay(self):
        return self.b_replay

    def set_frameId(self, n_frameId):
        self.n_frameId = n_frameId

    def get_frameId(self):
        return self.n_frameId

    def set_noData(self, b_noPcData):
        self.b_noData = b_noPcData

    def get_noData(self):
        return self.b_noData

    def set_state(self, n_state):
        self.n_state = n_state

    def get_state(self):
        return self.n_state


class structBoxInfo:
    """目标信息结构体"""

    def __init__(self):
        self.frameId = -1
        self.boxId = 0
        self.boxVertex = []
        self.boxSize = []
        self.boxCenter = []
        self.boxSpeed = "None"
        self.boxAngle = "None"
        self.boxLongitude = "None"
        self.boxLatitude = "None"
        self.strClass = ""
        self.strSource = ""
        self.nConfidence = 0
        self.nBaseStationId = 0
        self.nBaseStationBoxId = 0
        self.fColor = -10
        self.nHits = 0
        self.nBoxCamId = 0
        self.nLane = 0
        self.nPreFrameCnt = 0


class structData:
    def __init__(self):
        self.nFrameId = 0
        self.nBoxNum = 0
        self.listBoxInfo = []
        self.tTimeStamp = 0


##————————————界面功能函数————————————##
def load_vbo(pc_data, clr_data):
    """获取顶点和颜色信息"""
    if pc_data.size == 0:
        return
    try:
        vtx_Point = glvbo.VBO(pc_data[:, :3].astype(np.float32))  # 使用 顶点_强度_颜色 数组的前三列，创建顶点数据VBO
    except:
        global_log.error("shape:{}\nexcept:\n{}".format(pc_data.shape, traceback.format_exc()))
    vtx_Color = glvbo.VBO(clr_data.astype(np.float32) / 255.0)  # 使用 顶点_强度_颜色 数组的后三列，创建颜色数据VBO
    vtx_count = len(pc_data)  # 记录一帧数据的顶点个数
    return vtx_Point, vtx_Color, vtx_count


def get_lane_vbo(stStationParam, stUiParam):
    """获取车道矢量线的顶点信息"""
    s_laneFile_path = os.path.join(get_cfg_path(), 'Lane/' + stUiParam.s_laneFile_name) + '.json'
    list_vbo = []
    if judge_exist(s_laneFile_path):
        with open(s_laneFile_path, 'r', encoding='utf8')as fp:
            json_data = json.load(fp)
            features = json_data['features']
            stStationDev = stStationParam.getStationDev(0)
            stTransform = Lon_Lat_Meter_Transform(stStationDev.fLongitude, stStationDev.fLatitude,
                                                  stStationDev.fAngleNorth)
            for dic in features:
                xyz = []
                for j in range(len(dic["geometry"]["coordinates"][0])):
                    oldx, lody = dic["geometry"]["coordinates"][0][j][0], dic["geometry"]["coordinates"][0][j][1]
                    xyz.append([oldx, lody, -4.2])
                BKG_data = np.array(xyz)
                BKG_data[:, 0:2] = stTransform.lonlat_to_xy_of_draw(BKG_data[:, 0:2])
                clrs = np.ones((BKG_data.shape[0], 3)) * 150
                list_vbo.append(load_vbo(BKG_data, clrs))
        return list_vbo
    else:
        return list_vbo


def getPidByName(strProName):
    p = psutil.process_iter()
    listPid = []
    try:
        for r in p:
            aa = str(r)
            f = re.compile(strProName, re.I)
            if f.search(aa):
                listPid.append(int(aa.split('pid=')[1].split(',')[0]))
    except psutil.NoSuchProcess:
        print('the process does not exist')
    return listPid


def getProMemByName():
    currentId = os.getpid()
    proCur = psutil.Process(currentId)
    strProName = proCur.name()
    listPid = getPidByName(strProName)
    fAllMem = 0
    lisPidMemOut = []
    for pid in listPid:
        p = psutil.Process(pid)
        try:
            # strName = p.name()
            fMemory = p.memory_info().rss / 1024 / 1024
            fAllMem += fMemory
            lisPidMemOut.append([pid, int(fMemory)])
        except IOError as e:
            print(e)

    return int(fAllMem), lisPidMemOut


def getBoxInfo(npTargetMtx):
    listBoxInfo = []
    if npTargetMtx is not None:
        trackers_cov = np.zeros((npTargetMtx.shape[0], 20))
        pcBox = npTargetMtx[:, 20:].reshape(-1, 8, 3)
        for i in range(trackers_cov.shape[0]):
            stBoxInfo = structBoxInfo()
            stBoxInfo.boxVertexPc = pcBox[i]
            stBoxInfo.boxCenter = trackers_cov[i, 0:3]
            stBoxInfo.boxSize = trackers_cov[i, 3:6]
            stBoxInfo.boxAngle = trackers_cov[i, 6]
            nType = trackers_cov[i, 7]
            stBoxInfo.strClass = switch_class[nType]
            stBoxInfo.boxSpeed = trackers_cov[i, 8]
            stBoxInfo.boxId = int(trackers_cov[i, 9])
            stBoxInfo.nConfidence = int(trackers_cov[i, 10] * 100)
            stBoxInfo.strSource = "PointCloud"
            if int(trackers_cov[i, 11]) == 1:
                stBoxInfo.strSource = "Camera"
            elif int(trackers_cov[i, 11]) == 2:
                stBoxInfo.strSource = "Fusion"
            stBoxInfo.nBaseStationId = int(trackers_cov[i, 12])
            stBoxInfo.nBaseStationBoxId = int(trackers_cov[i, 13])
            stBoxInfo.nColor = int(trackers_cov[i, 14])
            stBoxInfo.boxLongitude = trackers_cov[i, 15]
            stBoxInfo.boxLatitude = trackers_cov[i, 16]
            stBoxInfo.nBoxCamId = int(trackers_cov[i, 17])
            stBoxInfo.nLane = int(trackers_cov[i, 18])
            stBoxInfo.nPreFrameCnt = int(trackers_cov[i, 19])
            listBoxInfo.append(stBoxInfo)  # 添加到链表
    return listBoxInfo


def getPcEnumFromStr(strClass):
    for i in range(len(switch_pc)):
        if strClass == switch_pc[i]:
            return i

    return -1


def getSendEnumFromColorStr(strColor):
    for i in range(len(switch_color)):
        if strColor == switch_color[i]:
            return i

    return -1
