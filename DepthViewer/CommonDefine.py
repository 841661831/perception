from __future__ import print_function

# PyQt的导入必须放在前面，否则release运行时会崩溃！
from PyQt5 import QtGui, QtCore, QtWidgets, QtOpenGL
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtCore import QTime, QPoint, QPointF, QRectF, QTranslator
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QMessageBox, QAbstractItemView, QDoubleSpinBox, \
    QSpacerItem, QSizePolicy, QWidget
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen, QColor, QIcon, QBrush, QFont, QFontMetrics, QFontDatabase, \
    QDoubleValidator
from PyQt5.QtCore import pyqtSignal, QObject

import ast
import pcap
import open3d as o3d
import multiprocessing
from multiprocessing import Process, Manager, Queue, Lock
from multiprocessing.managers import BaseManager
import torch.multiprocessing as mp
from numba import jit
import dpkt
import numpy as np
import math
import torch
import datetime
import os
import sys
import time
import socket
import copy
import subprocess
import threading
import psutil
import signal
import shlex
from subprocess import Popen, PIPE
import psutil
import GPUtil
import re
import traceback
import json
from kafka import KafkaProducer
import pandas as pd
from OpenGL.GL import *
import OpenGL.arrays.vbo as glvbo
import cv2
import struct
import shelve
import xml.etree.ElementTree as ET
from second.long_lat import XYZ_To_BLH
from copy import deepcopy
import func_timeout
import os
import shutil
import csv
import pytz
# 时间获取函数所需
from datetime import timezone
from datetime import timedelta

SECOND_1900_TO_1970 = 2208988800

# sys.path.append('/data/WJ-DepthViewer/')
from lib_my_numba import *
from pcap_offline.proc.pcap import *
import io
from cureUi import cure
import binascii

tz = pytz.timezone('Asia/Shanghai')

ENABLE_TEST = False

Rads_cov = 180 / math.pi
PI_rad = math.pi / 180

SavePathFile = "SaveFilePath.txt"
SavePcapPathFile = "SavePcapFilePath.txt"
CachePathFile = "CacheFilePath.txt"

VectorGraphPath = "/Configs/Lane/pavement-1.json"

PcCacheFile = "Frame"
VideoCacheFile = "Image"

listAllIndexOffset = []  # 用于存放多个pacp文件的帧位置
listCacheFileName = []  # 存放所有缓存文件的名字

switch_send_json = {0: "None", 1: "motor", 2: "non_motor", 3: "person", 4: "rsu", 5: "roadwork", 6: "accident",
                    7: "car", 8: "truck", 9: "bus", 10: "bicycle", 11: "motorbike"}
switch_send = {0: "None", 1: "car", 2: "large_truck", 3: "large_bus", 4: "person", 5: "bicycle", 6: "motorbike",
               7: "middle_bus", 8: "dangerous", 9: "relics", 10: "little_truck", 11: "middle_truck"}
switch_pc = {0: "car", 1: "bicycle", 2: "large_bus", 3: "motorbike", 4: "person", 5: "large_truck", 6: "middle_truck",
             7: "None", 8: "middle_bus", 9: "little_truck"}
switch_fusion = {0: "person", 1: "bicycle", 2: "car", 3: "motorbike", 4: "None", 5: "bus", 6: "None", 7: "truck",
                 8: "Minibus"}
switch_color = {0: "None", 1: "white", 2: "black", 3: "red", 4: "Silver", 5: "yellow", 6: "blue", 7: "multicolor",
                8: "brown", 9: "gray"}

PI_rads = math.pi / 180

MAX_FRAME_ID = 1e10  # 帧号最大值，超过该值，则重新计数
MIN_SPACE_LIMIT = 100  # 磁盘最小空间（MB）, 小于该值，则不允许保存

# 4路视频一起抓, 再经算法处理，然后显示, 800x600已是最大极限了，再往上，就会低于10fps
VIDEO_WIDGET_COUNT = 4

# ============ Play Mode start =============
MODE_AUTO_PLAY = 0
MODE_MANUAL_PLAY_SEARCH = 1
MODE_MANUAL_PLAY_START = 2
MODE_AUTO_PLAY_RE_SEARCH = 3
MODE_AUTO_PLAY_RE_START = 4
MODE_READ_OVER = 5


# =========== Play Mode end ===========


# def log_except_hook(*exc_info):
#     print("log_except_hook")
#     text = "".join(traceback.format_exception(*exc_info))
#     # logging.error("Unhandled exception: %s", text)
#     MyLog.writeLog("{}[:{}] - Unhandled exception!\n{}"
#                    .format(__file__.split('/')[len(__file__.split('/')) - 1],
#                            sys._getframe().f_lineno, text),
#                    LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
#
#
# sys.excepthook = log_except_hook


# =========== struct =================
class Lon_Lat_Meter_Transform(object):
    '''经纬度与m坐标系相互转换'''

    def __init__(self, lon0_bmp, lat0_bmp, angle0_bmp):  # 与全域基站经纬度转m转换关系一致
        '''初始化
        @param:
            lon0_bmp,lat0_bmp,angle0_bmp:用于绘图时的粗标的经纬度及夹角
        '''
        self.R_a = 6378137.00  # 地球长半轴
        self.R_b = 6356752.3142  # 地球短半轴
        self.lon0_bmp = lon0_bmp
        self.lat0_bmp = lat0_bmp
        self.angle0_bmp = angle0_bmp
        self.R_bmp, _ = cv2.Rodrigues(np.array([0, 0, self.angle0_bmp * np.pi / 180]))
        self.R_bmp_inv = np.linalg.inv(self.R_bmp)

    def lonlat_to_xy_of_draw(self, lonlat):
        '''经纬度转到画车道图的激光雷达m坐标系下
        @param:
            lon_lat:(-1,2) np.array
        @return:
            xy:(-1,2) np.array
        '''
        x = (lonlat[:, 0] - self.lon0_bmp) * np.pi / 180 * self.R_a * np.cos(self.lat0_bmp * np.pi / 180)
        y = (lonlat[:, 1] - self.lat0_bmp) * np.pi / 180 * self.R_b
        xyz = np.hstack((x.reshape(-1, 1), y.reshape(-1, 1), np.ones((len(x), 1))))  # 旋转
        xyz = self.R_bmp.dot(xyz.T).T
        return xyz[:, :2]

    def xy_of_draw_to_lonlat(self, xy):
        '''画车道图时的激光雷达m坐标系转换到经纬度
        @param:
            xy:(-1,2)  np.array
        @return:
            lonlat:(-1,2) np.array
        '''
        xyz = np.hstack((xy, np.ones((len(xy), 1))))
        origin_xyz = self.R_bmp_inv.dot(xyz.T).T  # 反旋转
        lon = origin_xyz[:, 0] / (self.R_a * np.cos(self.lat0_bmp * np.pi / 180)) * 180 / np.pi + self.lon0_bmp
        lat = origin_xyz[:, 1] / (self.R_b * np.pi / 180) + self.lat0_bmp
        lonlat = np.hstack((lon.reshape(-1, 1), lat.reshape(-1, 1)))
        return lonlat


class structBoxInfo:
    def __init__(self):
        self.frameId = -1
        self.boxId = 0
        self.nLane = -1  # 车道号
        self.boxVertexPc = []
        self.boxVertex = []  # for video box
        self.boxSize = []
        self.boxCenter = []
        self.boxSpeed = "None"
        self.boxAngle = "None"
        self.boxLongitude = "None"
        self.boxLatitude = "None"
        self.nPoints = "None"
        self.strClass = ""
        self.strSource = ""
        self.npoldboxCenter = None
        self.nConfidence = 0
        self.nEqupId = -1
        self.nBaseStationId = 0
        self.nBaseStationBoxId = 0
        self.fcolor = -10
        self.fcolor_score = -10
        self.nHits = 0
        self.nBoxCamId = 0
        self.nPreFrameCnt = 0
        self.nsingleFranme = 0


# B0车道信息
class B0ChannelData:  # B0
    def __init__(self):
        self.usChannelId = 0
        self.usChannelNum = 0
        self.usChannelDirection = 0
        self.ucChannelLineDir = 0
        self.ucChannelType = 0
        self.usChannelSpeedLimit = 0
        self.usChannelReservel1 = 0
        self.uiQueueLength = 0
        self.uiSpcaeOccupancy = 0
        self.ucCarNum = 0
        self.uc3ChannelReservel2 = (0, 0, 0)  # 3 unsigned char
        self.uiCarHeadDis = []
        self.usChannelReservel3 = 0


# B1帧车道信息
class B1ChannelData:  # B1
    def __init__(self):
        self.usChannelId = 0
        self.usChannelNum = 0
        self.usChannelDirection = 0
        self.ucChannelLineDir = 0
        self.ucChannelType = 0
        self.usChannelSpeedLimit = 0
        self.ucCongestionStatus = 0
        self.usTraffic = 0
        self.usTimeShare = 0
        self.usCar = 0
        self.usTruck = 0
        self.usBus = 0
        self.usMotorbike = 0
        self.usChannelReservel1 = 0


# B2违停信息
class B2ViolationData:  # B2
    def __init__(self):
        self.usViolationId = 0
        self.uiCameraNumbers = 0
        self.uiCameraIp = []
        self.uiViolationBeginTime1 = 0
        self.uiViolationBeginTime2 = 0
        self.uiViolationTime = 0
        self.usReservel1 = 0


# B2 通用目标信息
class B2GeneralTargetMsg:
    def __init__(self):
        self.usRetrogradeId = 0
        self.uiCameraNumbers = 0
        self.uiCameraIp = []
        self.uiRetrogradeBeginTime1 = 0
        self.uiRetrogradeBeginTime2 = 0
        self.usReservel1 = 0


# B2通用信息
class B2GeneralMsg:  # B2
    def __init__(self):
        self.usTargetNumber = 0
        self.usReservel1 = 0
        self.ulTargetInfo = []
        self.usReservel2 = 0


# B2 通用异常事件信息
class B2ExceptEventMsg:
    def __init__(self):
        self.usExceptId = 0
        self.usExceptType = 0
        self.uiCameraNumbers = 0
        self.uiCameraIp = []
        self.uiExceptBeginTime1 = 0
        self.uiExceptBeginTime2 = 0
        self.u2iExceptViolationTime = 0
        self.usReservel1 = 0


# B3车道信息
class B3ChannelData:  # B3
    def __init__(self):
        self.ucChannelId = 0
        self.ucChannelType = 0
        self.usChannelDirection = 0
        self.ucChannelSpeedLimit = 0
        self.ucMeanSpeed = 0
        self.uiQueueLength = 0
        self.usChannelInterval = 0
        self.ucSpcaeOccupancy = 0
        self.ucReservel1 = 0
        self.usMotorVehicles = 0
        self.usCarNum = 0
        self.usTruckFreight = 0
        self.usBigBus = 0
        self.usMediumBus = 0
        self.usTankCar = 0
        self.usPeople = 0
        self.usNoMotorVehicles = 0
        self.uiReservel2 = 0


# B4基站信息
class B4StationData:
    def __init__(self):
        self.ucStationId = 0
        self.ucCongestion = 0
        self.usReservel = 0


# B4帧车道信息
class B4ChannelData:  # B4
    def __init__(self):
        self.ucChannelId = 0
        self.ucChannelType = 0
        self.usChannelDirection = 0
        self.ucChannelSpeedLimit = 0
        self.ucStationNums = 0
        self.usReservel1 = 0
        self.tStationData = []
        self.ucCarfollowpercen = 0
        self.ucTimeOccupancy = 0
        self.usCarHeadDis = 0
        self.usCarFlow = 0
        self.usTruckFreightFlow = 0
        self.usBigBusFlow = 0
        self.usMediumBusFlow = 0
        self.usTankCarFlow = 0
        self.ucCarAverSpeed = 0
        self.ucTruckFreightAverSpeed = 0
        self.ucBigBusAverSpeed = 0
        self.ucMediumBusAverSpeed = 0
        self.ucTankCarAverSpeed = 0
        self.ucReservel2 = 0


# B5 通用异常事件信息
class B5ExceptTargetData:
    def __init__(self):
        self.usTargetId = 0
        self.usReservel = 0


# B5 通用异常事件信息
class B5ExceptEventMsg:
    def __init__(self):
        self.uiExceptId = 0
        self.ucExceptType = 0
        self.ucExceptOfStation = 0
        self.ucExceptOfChannel = 0
        self.ucExceptTargetNums = 0
        self.ExceptTarget = []
        self.uiExceptLong = 0
        self.uiExceptLat = 0
        self.uiExceptBeginTimeS = 0
        self.uiExceptBeginTimeUs = 0
        self.uiReservel = 0
        self.ucExceptCameraId = 0


# B0帧消息内容
class B0MsgContent:  # B0
    def __init__(self):  # len =
        self.usEquiqId = 0
        self.usReserve1 = 0
        self.uiTimeStamp1 = 0
        self.uiTimeStamp2 = 0
        self.usCarChannelNum = 0
        self.usReserve2 = 0
        self.tChannelData = []
        self.uiReserve3 = 0


# B1帧消息内容
class B1MsgContent:  # B1
    def __init__(self):
        self.usEquiqId = 0
        self.usReserve1 = 0
        self.uiStartTimeStamp1 = 0
        self.uiStartTimeStamp2 = 0
        self.uiEndTimeStamp1 = 0
        self.uiEndTimeStamp2 = 0
        self.usCarChannelNum = 0
        self.usReserve2 = 0
        self.tB1ChannelData = []


# B2帧消息内容
class B2MsgContent:  # B1
    def __init__(self):
        self.usEquiqId = 0
        self.usReserve1 = 0
        self.uiStartTimeStamp1 = 0
        self.uiStartTimeStamp2 = 0
        self.ExceptEentNumbers = 0
        self.usReserve2 = 0
        self.tTargetMsg = []
        self.usReserve3 = 0


# B3消息内容
class B3MsgContent:  # B3
    def __init__(self):  # len =
        self.usEquiqId = 0
        self.usReserve1 = 0
        self.uiTimeStampS = 0
        self.uiTimeStampUs = 0
        self.uiFrameId = 0
        self.ucCarChannelNums = 0
        self.u3cReserve2 = (0, 0, 0)
        self.tInfoData = []
        self.uiReserve3 = 0


# B4帧消息内容
class B4MsgContent:  # B4
    def __init__(self):
        self.usEquiqId = 0
        self.usReserve1 = 0
        self.uiStartTimeStampS = 0
        self.uiStartTimeStampUs = 0
        self.uiEndTimeStampS = 0
        self.uiEndTimeStampUs = 0
        self.uiFrameId = 0
        self.ucCarChannelNums = 0
        self.u3cReserve2 = (0, 0, 0)
        self.tInfoData = []


# B5帧消息内容
class B5MsgContent:  # B5
    def __init__(self):
        self.usEquiqId = 0
        self.usReserve1 = 0
        self.uiStartTimeStampS = 0
        self.uiStartTimeStampUs = 0
        self.ucExceptEentNums = 0
        self.usReserve2 = 0
        self.tInfoData = []
        self.uiReserve3 = 0


# B0帧瞬时事件
class B0TransientEvent:  # B0
    def __init__(self):  # len = 10 + n * MsgContent
        self.usBeginBit = 0xffff
        self.ucSerialNum = 2
        self.ucCommandNum = 0xb0
        self.ucSubComNum = 8
        self.ucStatusBit = 0
        self.usMsgLen = 2
        self.MsgContent = None
        self.ucCheck = 3
        self.ucCutoffBit = 0xff

    def setParam(self, sParam):
        self.usBeginBit = sParam.usBeginBit
        self.ucSerialNum = sParam.ucSerialNum
        self.ucCommandNum = sParam.ucCommandNum
        self.ucSubComNum = sParam.ucSubComNum
        self.ucStatusBit = sParam.ucStatusBit
        self.usMsgLen = sParam.usMsgLen
        self.MsgContent = sParam.MsgContent
        self.ucCheck = sParam.ucCheck
        self.ucCutoffBit = sParam.ucCutoffBit


# B1帧统计事件
class B1TransientEvent:  # B1
    def __init__(self):  # len = 10 + n * MsgContent
        self.usBeginBit = 0xffff
        self.ucSerialNum = 23
        self.ucCommandNum = 0xb1
        self.ucSubComNum = 123
        self.ucStatusBit = 13
        self.usMsgLen = 2
        self.MsgContent = None
        self.ucCheck = 1
        self.ucCutoffBit = 0xff


# B2交通异常事件
class B2TrafficIncident:  # B2
    def __init__(self):  # len = 10 + n * MsgContent
        self.usBeginBit = 0xffff
        self.ucSerialNum = 23
        self.ucCommandNum = 0xb2
        self.ucSubComNum = 123
        self.ucStatusBit = 13
        self.usMsgLen = 2
        self.MsgContent = None
        self.ucCheck = 1
        self.ucCutoffBit = 0xff


# B3帧瞬时事件
class B3TransientEvent:  # B3
    def __init__(self):  # len = 10 + n * MsgContent
        self.usBeginBit = 0xffff
        self.ucSerialNum = 2  # ??
        self.ucCommandNum = 0xb3
        self.ucSubComNum = 8  # ??
        self.ucStatusBit = 0  # ??
        self.usMsgLen = 2  # ??
        self.MsgContent = None
        self.ucCheck = 3  # ??
        self.ucCutoffBit = 0xff


# B4帧统计事件
class B4TransientEvent:  # B4
    def __init__(self):  # len = 10 + n * MsgContent
        self.usBeginBit = 0xffff
        self.ucSerialNum = 23
        self.ucCommandNum = 0xb4
        self.ucSubComNum = 123
        self.ucStatusBit = 13
        self.usMsgLen = 2
        self.MsgContent = None
        self.ucCheck = 1
        self.ucCutoffBit = 0xff


# B5交通异常事件
class B5TransientEvent:  # B5
    def __init__(self):  # len = 10 + n * MsgContent
        self.usBeginBit = 0xffff
        self.ucSerialNum = 23
        self.ucCommandNum = 0xb5
        self.ucSubComNum = 123
        self.ucStatusBit = 13
        self.usMsgLen = 2
        self.MsgContent = None
        self.ucCheck = 1
        self.ucCutoffBit = 0xff


class structData:
    def __init__(self):
        self.nFrameId = 0

        ###### add by siyu######
        self.online_num = -1
        self.nSingleFrame = {}
        self.nSingleStamp = {}
        self.nSingleEqupId = {}
        self.recvTime = {}
        self.n0TimeStamp = -1
        self.n0Frame = -1
        self.dictMessage = {}
        self.stationState = {}
        ###### add by siyu######

        # pc data
        self.headData = None
        self.pcData = None
        self.pcBox = None
        self.radarData = None
        self.radarBox = None
        self.pcTrackersCov = None
        self.radarTrackersCov = None
        self.nPcBoxIdMax = 0
        self.nradarBoxIdMax = 0
        # self.listSendData = []

        # video data
        self.pcTrackersCovToVideo = None
        self.radarTrackersCovToVideo = None
        self.pcBaseTrackersCovToVideo = None
        self.listVideoData = []
        self.fBox = None
        self.fClass = None
        self.fCenter = None
        self.fScore = None
        self.bFusion = True

        self.listBoxInfo = []
        self.listradarBoxInfo = []
        self.listBaseBoxInfo = []
        self.listVideoBoxInfo = []
        self.listFusionId = []
        self.dictTrafficFlowInfo = {}

        # B0帧瞬时事件
        self.B0TransientEvent = None
        # B1帧统计事件
        self.B1TransientEvent = None
        # 过滤列表，只显示该列表中的类型选项
        self.filterList = []
        self.trunAlg = 0
        self.nEqupId = []
        self.tTimeStamp = 0


class structCameraDev:
    def __init__(self):
        self.strCameraIp = ""
        self.nCameraChannel = 1
        self.in_parameter = np.mat([[1142.46, 0, 983.11], [0, 1161.61, 557.58], [0, 0, 1.0]])
        self.rotate_matrix = np.mat([1.57066659, -1.5350592, 0.96182733])
        self.translation_matrix = np.mat([100, -47.7, -85.6])
        self.dist_matrix = np.mat([-0.4093, 0.1742, -0.0063, -0.0006, 0])
        self.bUsrAndPwdRight = True


class structStationDev:
    def __init__(self):
        self.strStationSrcIp = ""
        self.fStationLongitude = 116.2869907
        self.fStationLatitude = 40.0516922
        self.fAngleNorthT = 0
        self.fRotationVector = np.zeros(3, float)
        self.fTranslationVector = np.zeros(3, float)
        self.listCameraDev = []


class structRadarDev:
    def __init__(self):
        self.strRadarSrcIp = ""
        self.strRadarDstIp = ""
        self.fRadarSrcPort = 55555
        self.fRadarLongitude = 116.2869907
        self.fRadarLatitude = 40.0516922
        self.fAngleNorthT = 0
        self.fRotationVector = np.zeros([3, 3], float)
        self.fTranslationVector = np.zeros(3, float)
        self.fOffset = 0.0


class structLidarParam(object):
    def __init__(self):
        self.nStationCount = 1
        self.nRadarCount = 1
        self.listStationDev = []
        self.listRadarDev = []
        self.listForwardDstDev = []
        self.listForwardEventDstDev = []  # 发送车流量的目的ip-port
        self.nNetworkPort = 0
        self.bUseCropRoi = False  # 是否使用裁剪的ROI区域
        self.bUseBgdFrameFilter = False  # 是否使用背景帧过滤地面点
        self.bLiveUpdateBgdFrame = False  # 是否实时更新背景帧
        self.bUseVoxelPoints = False  # 是否使用体素模式
        self.bRotationTranslation = True  # 是否进行旋转平移
        self.nUdpServicePort = 5555
        self.LidarSwitch = [True, True, True, True]
        self.listLidarCount = 0
        self.listRadarCount = 0

        self.bFusion = True
        self.bShowRLFusion = True

        self.udpSocket = None
        self.bRunning = True
        self.bShowPc = True
        self.bShowRLFusion = True
        self.bOnline = False
        self.bPause = False
        self.bSave = False
        self.listPcOver = []
        self.bLoop = False
        self.bReplay = False
        self.usStatus = 0
        self.nFrameId = 0
        self.nLidarFrameCount = 0  # 记录点云文件总帧数
        self.listOfflineFiles = []
        self.bReadChanged = False
        self.bAlgChanged = False
        self.bParamChanged = False
        self.bNoPcData = False
        self.bSaveBmp = False  # 是否旋转
        self.bSingleFrame = False  # 是否单帧模式
        self.nEventFeedBackPort = 5554  # 是否发送点云和识别结果
        self.bSendResult = False  # 是否发送点云和识别结果

        self.bGetingData = False

        self.fLaneRotationVector = np.zeros(3, float)
        self.fLaneTranslationVector = np.zeros(3, float)

        self.listAttachedIp = []
        self.nBaseStationID = 0
        self.bHighSpeed = False
        self.bCoordinateFlag = True
        self.dictBoxinLane = {}

    def setDictBoxinLane(self, dictBoxinLane):
        self.dictBoxinLane = dictBoxinLane

    def getDictBoxinLane(self):
        return self.dictBoxinLane

    def setUdpServicePort(self, nUdpServicePort):
        self.nUdpServicePort = nUdpServicePort

    # lzq添加，获取点云文件总帧数
    def getLidarFrameCount(self):
        return self.nLidarFrameCount

    def setLidarFrameCount(self, frameCount):
        self.nLidarFrameCount = frameCount

    def getUdpServicePort(self):
        return self.nUdpServicePort

    def setEventFeedBackPort(self, nEventFeedBackPort):
        self.nEventFeedBackPort = nEventFeedBackPort

    def getEventFeedBackPort(self):
        return self.nEventFeedBackPort

    def setSendResult(self, bSendResult):
        self.bSendResult = bSendResult

    def getSendResult(self):
        return self.bSendResult

    def setSingleFrame(self, bSingleFrame):
        self.bSingleFrame = bSingleFrame

    def getSingleFrame(self):
        return self.bSingleFrame

    def setSaveBmp(self, bSaveBmp):
        self.bSaveBmp = bSaveBmp

    def getSaveBmp(self):
        return self.bSaveBmp

    def setUseCropRoi(self, bUseCropRoi):
        self.bUseCropRoi = bUseCropRoi

    def getUseCropRoi(self):
        return self.bUseCropRoi

    def setUseBgdFrameFilter(self, bUseBgdFrameFilter):
        self.bUseBgdFrameFilter = bUseBgdFrameFilter

    def getUseBgdFrameFilter(self):
        return self.bUseBgdFrameFilter

    def setLiveUpdateBgdFrame(self, bLiveUpdateBgdFrame):
        self.bLiveUpdateBgdFrame = bLiveUpdateBgdFrame

    def getLiveUpdateBgdFrame(self):
        return self.bLiveUpdateBgdFrame

    def setUseVoxelPoints(self, bUseVoxelPoints):
        self.bUseVoxelPoints = bUseVoxelPoints

    def getUseVoxelPoints(self):
        return self.bUseVoxelPoints

    def setRotationTranslation(self, bRotationTranslation):
        self.bRotationTranslation = bRotationTranslation

    def getRotationTranslation(self):
        return self.bRotationTranslation

    def setRotationVector(self, fRotationVector, index):
        self.listStationDev[index].fRotationVector = fRotationVector

    def getRotationVector(self, index):
        return self.listStationDev[index].fRotationVector

    def setTranslationVector(self, fTranslationVector, index):
        self.listStationDev[index].fTranslationVector = fTranslationVector

    def getTranslationVector(self, index):
        return self.listStationDev[index].fTranslationVector

    def setLidarLongitude(self, fStationLongitude, index):
        self.listStationDev[index].fStationLongitude = fStationLongitude

    def getLidarLongitude(self, index):
        return self.listStationDev[index].fStationLongitude

    def setLidarLatitude(self, fStationLatitude, index):
        self.listStationDev[index].fStationLatitude = fStationLatitude

    def getLidarLatitude(self, index):
        return self.listStationDev[index].fStationLatitude

    def setAngleNorthT(self, fAngleNorthT, index):
        self.listStationDev[index].fAngleNorthT = fAngleNorthT

    def getAngleNorthT(self, index):
        return self.listStationDev[index].fAngleNorthT

    def setParamChanged(self, bParamChanged):
        self.bParamChanged = bParamChanged

    def getParamChanged(self):
        return self.bParamChanged

    def setLidarSwitch(self, LidarSwitch):
        self.LidarSwitch = LidarSwitch

    def getLidarSwitch(self):
        return self.LidarSwitch

    def setListLidarCount(self, ListLidarCount):
        self.listLidarCount = ListLidarCount

    def getListLidarCount(self):
        return self.listLidarCount

    def setListRadarCount(self, ListRadarCount):
        self.listRadarCount = ListRadarCount

    def getListRadarCount(self):
        return self.listRadarCount

    def setRadarSrcIp(self, RadarSrcIp, index):
        self.listRadarDev[index].strRadarSrcIp = RadarSrcIp

    def getRadarSrcIp(self, index):
        return self.listRadarDev[index].strRadarSrcIp

    def setRadarDstIp(self, RadarDstIp, index):
        self.listRadarDev[index].strRadarDstIp = RadarDstIp

    def getRadarDstIp(self, index):
        return self.listRadarDev[index].strRadarDstIp

    def setRadarSrcPort(self, RadarSrcPort, index):
        self.listRadarDev[index].fRadarSrcPort = RadarSrcPort

    def getRadarSrcPort(self, index):
        return self.listRadarDev[index].fRadarSrcPort

    def setRadarLongitude(self, RadarLongitude, index):
        self.listRadarDev[index].fRadarLongitude = RadarLongitude

    def getRadarLongitude(self, index):
        return self.listRadarDev[index].fRadarLongitude

    def setRadarLatitude(self, RadarLatitude, index):
        self.listRadarDev[index].fRadarLatitude = RadarLatitude

    def getRadarLatitude(self, index):
        return self.listRadarDev[index].RadarLatitude

    def setRadarAngleNorthT(self, AngleNorthT, index):
        self.listRadarDev[index].fAngleNorthT = AngleNorthT

    def getRadarAngleNorthT(self, index):
        return self.listRadarDev[index].AngleNorthT

    def setRadarRotationVector(self, RotationVector, index):
        self.listRadarDev[index].fRotationVector = RotationVector

    def getRadarRotationVector(self, index):
        return self.listRadarDev[index].fRotationVector

    def setRadarTranslationVector(self, TranslationVector, index):
        self.listRadarDev[index].fTranslationVector = TranslationVector

    def getRadarTranslationVector(self, index):
        return self.listRadarDev[index].fTranslationVector

    def setRadarOffset(self, fOffset, index):
        self.listRadarDev[index].fOffset = fOffset

    def getRadarOffset(self, index):
        return self.listRadarDev[index].fOffset

    def setShowRLFusion(self, bShowRLFusion):
        self.bShowRLFusion = bShowRLFusion

    def getShowRLFusion(self):
        return self.bShowRLFusion

    def setNoneLidarRegistration(self):
        self.listLidarOriginal = [[], [], []]

    def addLidarOriginal(self, element, index):
        self.listLidarOriginal[index - 1].append(element)

    def popLidarOriginal(self, index):
        self.listLidarOriginal[index - 1].pop(-1)

    def getLidarOriginal(self):
        return self.listLidarOriginal

    def setLaneRotationVector(self, fLaneRotationVector):
        self.fLaneRotationVector = fLaneRotationVector

    def getLaneRotationVector(self):
        return self.fLaneRotationVector

    def setLaneTranslationVector(self, fLaneTranslationVector):
        self.fLaneTranslationVector = fLaneTranslationVector

    def getLaneTranslationVector(self):
        return self.fLaneTranslationVector

    def setlistAttachedIp(self, listAttachedIp):
        self.listAttachedIp = listAttachedIp

    def getlistAttachedIp(self):
        return self.listAttachedIp

    def setBaseStationID(self, nBaseStationID):
        self.nBaseStationID = nBaseStationID

    def getBaseStationID(self):
        return self.nBaseStationID

    def setHighSpeed(self, bHighSpeed):
        self.bHighSpeed = bHighSpeed

    def getHighSpeed(self):
        return self.bHighSpeed

    def setCoordinateFlag(self, bCoordinateFlag):
        self.bCoordinateFlag = bCoordinateFlag

    def getCoordinateFlag(self):
        return self.bCoordinateFlag

    def setStationCameraDev(self, listCameraDev, index):
        self.listStationDev[index].listCameraDev = listCameraDev

    def getStationCameraDev(self, index):
        return self.listStationDev[index].listCameraDev

    def printSelf(self):
        print("============lidar start==============")
        for dev in self.listStationDev:
            print("strStationSrcIp:", dev.strStationSrcIp)

        print("listForwardDstDev:\n", self.listForwardDstDev)
        print("nNetworkPort:", self.nNetworkPort)
        print("bRunning:", self.bRunning)
        print("bShowPc:", self.bShowPc)
        print("bOnline:", self.bOnline)
        print("bPause:", self.bPause)
        print("bSave:", self.bSave)
        print("bPcOver:", self.listPcOver)
        print("bVideoOver:", self.bVideoOver)
        print("usStatus:", self.usStatus)
        print("nFrameId:", self.nFrameId)
        print("listOfflineFiles:\n", self.listOfflineFiles)
        print("============lidar end==============")

    def setParam(self, stParam):
        self.listStationDev = stParam.listStationDev
        self.listForwardDstDev = stParam.listForwardDstDev
        self.listForwardEventDstDev = stParam.listForwardEventDstDev
        self.nRadarCount = stParam.nRadarCount
        self.listRadarDev = stParam.listRadarDev
        self.nNetworkPort = stParam.nNetworkPort
        self.bUseCropRoi = stParam.bUseCropRoi
        self.bUseBgdFrameFilter = stParam.bUseBgdFrameFilter
        self.bLiveUpdateBgdFrame = stParam.bLiveUpdateBgdFrame
        self.bUseVoxelPoints = stParam.bUseVoxelPoints
        self.bRotationTranslation = stParam.bRotationTranslation
        self.nUdpServicePort = stParam.nUdpServicePort
        self.LidarSwitch = stParam.LidarSwitch
        self.bFusion = stParam.bFusion
        self.bShowRLFusion = stParam.bShowRLFusion
        self.udpSocket = stParam.udpSocket
        self.bRunning = stParam.bRunning
        self.bShowPc = stParam.bShowPc
        self.bOnline = stParam.bOnline
        self.bPause = stParam.bPause
        self.bSave = stParam.bSave
        self.bLoop = stParam.bLoop
        self.bReplay = stParam.bReplay
        self.bSingleFrame = stParam.bSingleFrame
        self.nEventFeedBackPort = stParam.nEventFeedBackPort
        self.bSendResult = stParam.bSendResult
        self.listPcOver = stParam.listPcOver
        self.usStatus = stParam.usStatus
        self.nFrameId = stParam.nFrameId
        self.listOfflineFiles = stParam.listOfflineFiles
        self.bReadChanged = stParam.bReadChanged
        self.bAlgChanged = stParam.bAlgChanged
        self.bParamChanged = stParam.bParamChanged
        self.bNoPcData = stParam.bNoPcData
        self.bSaveBmp = stParam.bSaveBmp
        self.nLidarFrameCount = stParam.nLidarFrameCount
        self.bGetingData = stParam.bGetingData
        self.listAttachedIp = stParam.listAttachedIp
        self.nBaseStationID = stParam.nBaseStationID
        self.bHighSpeed = stParam.bHighSpeed

    def setFusion(self, bFusion):
        self.bFusion = bFusion

    def getFusion(self):
        return self.bFusion

    def getParam(self):
        stParam = copy.copy(self)
        return stParam

    def setLidarCount(self, nStationCount):
        self.nStationCount = nStationCount

    def getLidarCount(self):
        return self.nStationCount

    def setRadarCount(self, nRadarCount):
        self.nRadarCount = nRadarCount

    def getRadarCount(self):
        return self.nRadarCount

    def setStationSrcIp(self, srcIp, index):
        self.listStationDev[index].strStationSrcIp = srcIp

    def getStationSrcIp(self, index):
        return self.listStationDev[index].strStationSrcIp

    def setListForwardDstDev(self, listForwardDstDev):
        self.listForwardDstDev = listForwardDstDev

    def getListForwardDstDev(self):
        return self.listForwardDstDev

    def setListForwardEventDstDev(self, listForwardEventDstDev):
        self.listForwardEventDstDev = listForwardEventDstDev

    def getListForwardEventDstDev(self):
        return self.listForwardEventDstDev

    def setNetworkPort(self, nNetworkPort):
        self.nNetworkPort = nNetworkPort

    def getNetworkPort(self):
        return self.nNetworkPort

    def setUdpSocket(self, udpSocket):
        self.udpSocket = udpSocket

    def getUdpSocket(self):
        return self.udpSocket

    def setRunning(self, bRunning):
        self.bRunning = bRunning

    def getRunning(self):
        return self.bRunning

    def setShowPc(self, bShowPc):
        self.bShowPc = bShowPc

    def getShowPc(self, ):
        return self.bShowPc

    def setGetingData(self, bGetingData):
        self.bGetingData = bGetingData

    def setOnline(self, bOnline):
        self.bOnline = bOnline

    def getOnline(self):
        return self.bOnline

    def setPause(self, bPause):
        self.bPause = bPause

    def getPause(self):
        return self.bPause

    def setSave(self, bSave):
        if bSave != self.bSave:
            self.bSave = bSave
            self.setParamChanged(True)
            print("setSave:", bSave)

    def getSave(self):
        return self.bSave

    def setPcOver(self, listPcOver):
        self.listPcOver = listPcOver

    def getPcOver(self):
        return self.listPcOver

    def setStatus(self, usStatus):
        self.usStatus = usStatus

    def getStatus(self):
        return self.usStatus

    def setFrameId(self, nFrameId):
        self.nFrameId = int(nFrameId)

    def getFrameId(self):
        return int(self.nFrameId)

    def setListOfflineFiles(self, listOfflineFiles):
        self.listOfflineFiles = listOfflineFiles

    def getListOfflineFiles(self):
        return self.listOfflineFiles

    def setReadChanged(self, bReadChanged):
        self.bReadChanged = bReadChanged

    def getReadChanged(self):
        return self.bReadChanged

    def setAlgChanged(self, bAlgChanged):
        self.bAlgChanged = bAlgChanged

    def getAlgChanged(self):
        return self.bAlgChanged

    def setAutoLoopPlay(self, bLoop):
        if bLoop != self.bLoop:
            self.bLoop = bLoop
            print("pc: setAutoLoopPlay:", bLoop)

    def getAutoLoopPlay(self):
        return self.bLoop

    def setManualReplay(self, bReplay):
        if bReplay != self.bReplay:
            self.bReplay = bReplay
            print("pc: setManualReplay:", bReplay)

    def getManualReplay(self):
        return self.bReplay

    def setNoPcData(self, bNoPcData):
        self.bNoPcData = bNoPcData

    def getNoPcData(self):
        return self.bNoPcData


class structPcAlgParam:
    def __init__(self):
        self.img = [None, None, None]
        # self.img = None
        self.device = None
        self.selected_mode = None
        self.anchor_cache = None
        self.voxel_generator = None
        self.anchor_area_threshold = None
        self.net = None
        self.filter_mode = None
        self.DATA_NAME = None
        self.mot_tracker = None
        # self.background_img = []
        self.background_img = [None, None, None]

        self.nSyncIdLimitPcOnline = 12
        self.nSyncIdLimitPcOffline = 0
        self.fCropBackSize = 0.5
        self.nBmpSize = 2000

        self.fRadius = 2.3
        self.nLastCount = 0
        self.fDis = 70
        self.fMinZ = -4.5
        self.fMaxZ = 0

        self.fCarThreshold = 0.25
        self.fConeThreshold = 0.9
        self.nMaxAge = 4
        self.nMinHits = 2

        self.line_k = None
        self.line_b = None
        self.line_limit = None
        self.line_angle = None


class structVideoAlgParam:
    def __init__(self):
        self.IMAGE_H = None
        self.IMAGE_W = None
        self.output_tensors = None
        self.input_tensor = None
        self.num_classes = None
        self.isess = None
        self.use_deepsort = []
        self.fusion_algorithm_list = []
        # self.listCooridinate_map = []
        self.traffic_flow = None
        self.filter_imgs = []

        self.nResizeVideoWidth = 800
        self.nResizeVideoHeight = 800

    # def __del__(self):
    #     if self.isess is not None:
    #         self.isess.__del__()
    #     if self.input_tensor is not None:
    #         self.input_tensor = None
    #     if self.output_tensors is not None:
    #         self.output_tensors = None


class structFusionAlgParam:
    def __init__(self):
        self.num_classes = None
        self.use_deepsort = []
        self.fusion_algorithm_list = []
        self.traffic_flow = None


class structUiParam:
    def __init__(self):
        self.bDebug = False
        self.bWriteLog = True
        self.bPrintLog = False
        self.bAutoConnect = True
        self.nTimeInterval = 100
        self.nCacheFileCountLimit = 100
        self.nSaveFileSizeLimit = 1073741824  # 1073741824 == 1024 * 1024 * 1024 byte, 就是1G  # 104857600 == 100m
        self.bAbnormalRestart = False
        self.fMemPercentMax = 90
        self.fGpuPercentMax = 90
        self.fFpsMinLimit = 8
        self.strTerminalId = "None"
        self.strKafkaNet = "None"
        self.strKafkaTopic = "None"
        self.bRotatingLane = False
        self.fRoughLanRotationVector = np.zeros(3, float)
        self.fRoughLanTranslationVector = np.zeros(3, float)
        self.fFineLanRotationVector = np.zeros(3, float)
        self.fFineLanTranslationVector = np.zeros(3, float)
        self.fBoxRotationAngle = 0
        self.listAttachedIp = []
        self.SoftwareStyle = 0
        # 车道统计相关参数
        self.alone_to_together_x = 0
        self.alone_to_together_y = 0

        self.nOfflineFrame = 5000
        self.bSaveE1 = False
        self.bSaveEvent = False

        self.nPlatformB3Frequency = 1
        self.nPlatformB4Frequency = 60


BaseManager.register('structLidarParam', structLidarParam)


# =========== struct =================


# ===================== 功能函数 =====================
def get_size(obj, seen=None):
    """Recursively finds size of objects"""
    size = sys.getsizeof(obj)
    if seen is None:
        seen = set()
    obj_id = id(obj)
    if obj_id in seen:
        return 0
    # Important mark as seen *before* entering recursion to gracefully handle
    # self-referential objects
    seen.add(obj_id)
    if isinstance(obj, dict):
        size += sum([get_size(v, seen) for v in obj.values()])
        size += sum([get_size(k, seen) for k in obj.keys()])
    elif hasattr(obj, '__dict__'):
        size += get_size(obj.__dict__, seen)
    elif hasattr(obj, '__iter__') and not isinstance(obj, (str, bytes, bytearray)):
        size += sum([get_size(i, seen) for i in obj])
    return size


def getExcutePath():
    if getattr(sys, 'frozen', False):  # 查找 sys 中有没 frozen 属性，如果没有返回Fasle。
        # sys中的 'frozen' 属性 是打包成 一个EXE文件特有的属性。
        # print("case 1")
        bundle_dir = sys._MEIPASS
    else:
        # print("case 2")
        # 这是在没打包成一个EXE文件的情况下，文件的当前路径。
        bundle_dir = os.path.dirname(os.path.abspath(__file__))

    return bundle_dir


strExePath = getExcutePath()
ipTruePathTemp = strExePath + "/Configs/Dev/ipTrueTemp.txt"
ipTruePath = strExePath + "/Configs/Dev/ipTrue.txt"

PERFORMANCE_INFO_PATH = strExePath + "/PerformanceInfo/"
csvPath = strExePath + "/BoxInfo/"
TempPath = strExePath + "/tmp/"
SavePath = strExePath + "/Save/"
CachePath = strExePath + "/Cache/"
PcCachePath = strExePath + "/Cache/PC_Cache/"
VideoCachePath = strExePath + "/Cache/Video_Cache/"
CacheDataKey = "CacheData"
CacheFileSuffix = ""
BoxCachePath = strExePath + "/Cache/Box_Cache/"
BoxCacheFile = "BoxFrame"
BoxInfoCachePath = strExePath + "/Cache/BoxInfoCache/"
BoxInfoCacheFile = "BoxInfoFrame"


def my_isExist(path):
    # 去除首位空格
    path = path.strip()

    # 去除尾部 \ 符号
    path = path.rstrip("\\")

    # 判断路径是否存在
    # 存在     True
    # 不存在   False
    isExists = os.path.exists(path)
    return isExists


def my_mkdir(path):
    isExists = my_isExist(path)

    # 判断结果
    if not isExists:
        try:
            # 如果不存在则创建目录
            os.makedirs(path)
            # print(path + ' create successful!')
        except:
            MyLog.writeLog("{}[:{}] - except! makedirs except!"
                           .format(__file__.split('/')[len(__file__.split('/')) - 1],
                                   sys._getframe().f_lineno), LOG_ERROR, True, True)

        isExists = my_isExist(path)
        if not isExists:
            MyLog.writeLog("{}[:{}] - except! my_mkdir failed!"
                           .format(__file__.split('/')[len(__file__.split('/')) - 1],
                                   sys._getframe().f_lineno), LOG_ERROR, True, True)
            return False

        return True

    # 如果目录存在则不创建，并提示目录已存在
    # print(path + ' is already exist!')
    return True


def my_delDir(path):
    try:
        isExists = my_isExist(path)
        if isExists:
            if os.path.isdir(path):
                shutil.rmtree(path, ignore_errors=True)
                return True
            else:
                return False
    except:
        print("{}[:{}] - except! my_delDir err!")
        print("{}[:{}] - except! Call stack:\n{}"
              .format(__file__.split('/')[len(__file__.split('/')) - 1],
                      sys._getframe().f_lineno, traceback.format_exc()))
        return False


# ======================== log ============================
# LOG_FORMAT = "%(asctime)s - %(levelname)s - %(message)s"
# # LOG_PATH = strExePath + '/Log/'
# LOG_PATH = '/var/log/WJ-ISFP-Log/'
# my_mkdir(LOG_PATH)
#
# # logging 初始化工作
# logger = logging.getLogger("syq")
# logger.setLevel(LOG_INFO)
#
# # 添加TimedRotatingFileHandler, 定义一个1秒换一次log文件的handler,保留3个旧log文件
# rf_handler = logging.handlers.TimedRotatingFileHandler(filename=LOG_PATH + "/log", when='D', interval=1, backupCount=3)
# rf_handler.setFormatter(logging.Formatter(LOG_FORMAT))
# rf_handler.setLevel(LOG_INFO)
# logger.addHandler(rf_handler)

LOG_PATH = '/var/log/WJ-ISFP-Log/'
LOG_INFO = 0
LOG_WARNING = 1
LOG_ERROR = 2


class MyLog:
    lockLog = Lock()
    strLastData = datetime.datetime.now().strftime("%Y-%m-%d")
    strLastDataPerformance = datetime.datetime.now().strftime("%Y-%m-%d")
    strLogFile = LOG_PATH + "log"
    strInfoFile = PERFORMANCE_INFO_PATH + "PerformanceInfo.csv"
    if my_isExist(strLogFile):
        with open(strLogFile, 'r') as f:
            strLastData = f.readline()[:10]

    if my_isExist(strInfoFile):
        with open(strInfoFile, 'r') as f:
            strLastDataPerformance = f.readline()[:10]

    @staticmethod
    def writeLog(msg, level, bWrite, bPrint):
        # t1 = time.time()
        strDataTime = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        strMsg = strDataTime + " - INFO - " + msg
        if level == LOG_WARNING:
            strMsg = strDataTime + " - WARNING - " + msg
        if level == LOG_ERROR:
            strMsg = strDataTime + " - ERROR - " + msg

        if bWrite:
            my_mkdir(LOG_PATH)
            strData = datetime.datetime.now().strftime("%Y-%m-%d")
            if strData != MyLog.strLastData:
                MyLog.lockLog.acquire()
                strLogFileBak = LOG_PATH + "log.{}".format(MyLog.strLastData)
                my_renameFile(MyLog.strLogFile, strLogFileBak)
                MyLog.strLastData = strData
                MyLog.lockLog.release()

            with open(MyLog.strLogFile, 'a') as f:
                f.write(strMsg + "\n")

            listLogFile = []
            dir_list = os.listdir(LOG_PATH)
            if len(dir_list) > 8:
                for file in dir_list:
                    if os.path.isfile(os.path.join(LOG_PATH, file)) and "log" in file:
                        listLogFile.append(file)
            if len(listLogFile) > 8:
                # 注意，这里使用lambda表达式，将文件按照最后修改时间顺序升序排列
                # os.path.getmtime() 函数是获取文件最后修改时间
                # os.path.getctime() 函数是获取文件最后创建时间
                listLogFile = sorted(listLogFile, key=lambda x: os.path.getmtime(os.path.join(LOG_PATH, x)))
                my_delFile(os.path.join(LOG_PATH, listLogFile[0]))

        if bPrint:
            print(strMsg)
        # print("writeLog time:%.2fms" % ((time.time() - t1) * 1000))

    @staticmethod
    def savePerformanceInfo(listInfo):
        my_mkdir(PERFORMANCE_INFO_PATH)
        strData = datetime.datetime.now().strftime("%Y-%m-%d")
        if strData != MyLog.strLastDataPerformance:
            strInfoFileBak = PERFORMANCE_INFO_PATH + "PerformanceInfo.csv.{}".format(MyLog.strLastDataPerformance)
            my_renameFile(MyLog.strInfoFile, strInfoFileBak)
            MyLog.strLastDataPerformance = strData

        # t1 = time.time()
        my_save2csv(MyLog.strInfoFile, listInfo)

        listInfoFile = []
        dir_list = os.listdir(PERFORMANCE_INFO_PATH)
        if len(dir_list) > 8:
            for file in dir_list:
                if os.path.isfile(os.path.join(PERFORMANCE_INFO_PATH, file)) and "PerformanceInfo" in file:
                    listInfoFile.append(file)
        if len(listInfoFile) > 8:
            # 注意，这里使用lambda表达式，将文件按照最后修改时间顺序升序排列
            # os.path.getmtime() 函数是获取文件最后修改时间
            # os.path.getctime() 函数是获取文件最后创建时间
            listInfoFile = sorted(listInfoFile, key=lambda x: os.path.getmtime(os.path.join(PERFORMANCE_INFO_PATH, x)))
            my_delFile(os.path.join(PERFORMANCE_INFO_PATH, listInfoFile[0]))


# ======================== log ============================


def getLanAngleFromCsv(strFilePath, mapLanAngel):
    listLanAngel = []
    with open(strFilePath) as csvfile:
        bSkip = True
        csv_reader = csv.reader(csvfile)  # 使用csv.reader读取csvfile中的文件
        for row in csv_reader:  # 将csv 文件中的数据保存到birth_data中
            try:
                ll = [int(row[0]), float(row[1])]
            except:
                print("[{}] Cannot convert to float!".format(row))
                continue
            # listLanAngel.append(ll)
            mapLanAngel[ll[0]] = ll[1] * PI_rad

    # listLanAngel = [[float(x) for x in row] for row in listLanAngel]  # 将数据从string形式转换为float形式
    return mapLanAngel


# 解析设备参数配置文件
def serializationParam(stLidarParam=None, stCameraParam=None, stPcAlgParam=None, stVideoAlgParam=None, stUiParam=None,
                       g_mapLanAngle=None, bSave=False):
    strExePath = getExcutePath()
    strFilePath = strExePath + '/Configs/Dev/DevParam.xml'
    if not my_isExist(strFilePath):
        MyLog.writeLog("{}[:{}] - DevParam.xml is not exist!.".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                      sys._getframe().f_lineno), LOG_ERROR, True, True)
        return

    tree = ET.parse(strFilePath)
    root = tree.getroot()

    if not bSave:
        # lidarParam
        if stLidarParam is not None:
            stLidarParam.nStationCount = int(root[0][0].text)
            listAttachedIp = []
            for tag in root[0][1]:
                stStationDev = structStationDev()
                stStationDev.strStationSrcIp = tag[0].text
                if stStationDev.strStationSrcIp is not None:
                    listAttachedIp.append(stStationDev.strStationSrcIp)
                else:
                    continue
                stStationDev.fStationLatitude = float(tag[1].text)
                stStationDev.fStationLongitude = float(tag[2].text)
                stStationDev.fAngleNorthT = float(tag[3].text)
                stStationDev.fRotationVector[0] = float(tag[4].text)
                stStationDev.fRotationVector[1] = float(tag[5].text)
                stStationDev.fRotationVector[2] = float(tag[6].text)
                stStationDev.fTranslationVector[0] = float(tag[7].text)
                stStationDev.fTranslationVector[1] = float(tag[8].text)
                stStationDev.fTranslationVector[2] = float(tag[9].text)
                for tag_tag in tag[10]:
                    stCameraDev = structCameraDev()
                    stCameraDev.strCameraIp = tag_tag[0].text
                    stCameraDev.nCameraChannel = int(tag_tag[1].text)
                    listInParameter = tag_tag[2].text.split(',')
                    listRotateMatrix = tag_tag[3].text.split(',')
                    listTranslationMatrix = tag_tag[4].text.split(',')
                    listDistMatrix = tag_tag[5].text.split(',')
                    stCameraDev.in_parameter = np.mat(
                        [[float(listInParameter[0]), float(listInParameter[1]), float(listInParameter[2])],
                         [float(listInParameter[3]), float(listInParameter[4]), float(listInParameter[5])],
                         [float(listInParameter[6]), float(listInParameter[7]), float(listInParameter[8])]])
                    stCameraDev.rotate_matrix = np.mat(
                        [float(listRotateMatrix[0]), float(listRotateMatrix[1]), float(listRotateMatrix[2])])
                    stCameraDev.translation_matrix = np.mat(
                        [float(listTranslationMatrix[0]), float(listTranslationMatrix[1]),
                         float(listTranslationMatrix[2])])
                    stCameraDev.dist_matrix = np.mat(
                        [float(listDistMatrix[0]), float(listDistMatrix[1]), float(listDistMatrix[2]),
                         float(listDistMatrix[3]), float(listDistMatrix[4])])
                    stStationDev.listCameraDev.append(stCameraDev)
                stLidarParam.listStationDev.append(stStationDev)
            stLidarParam.listAttachedIp = listAttachedIp

            for tag in root[0][2]:
                stLidarParam.listForwardDstDev.append([tag.text, False])
            for tag in root[0][3]:
                stLidarParam.listForwardEventDstDev.append([tag.text, False])
            stLidarParam.nNetworkPort = max(int(root[0][4].text), 0)

            stLidarParam.bUseCropRoi = bool(int(root[0][5].text))
            stLidarParam.bUseBgdFrameFilter = bool(int(root[0][6].text))
            stLidarParam.bLiveUpdateBgdFrame = bool(int(root[0][7].text))
            stLidarParam.bUseVoxelPoints = bool(int(root[0][8].text))
            stLidarParam.bRotationTranslation = bool(int(root[0][9].text))

            stLidarParam.nUdpServicePort = int(root[0][10].text)
            stLidarParam.nEventFeedBackPort = int(root[0][11].text)
            stLidarParam.bSendResult = bool(int(root[0][12].text))
            stLidarParam.bHighSpeed = bool(int(root[0][13].text))

            nCount = 0
            for tag in root[5][1]:
                stRadarDev = structRadarDev()
                stRadarDev.strRadarSrcIp = tag[0].text
                stRadarDev.strRadarDstIp = tag[1].text
                stRadarDev.fRadarSrcPort = int(tag[2].text)
                stRadarDev.fRadarLatitude = float(tag[3].text)
                stRadarDev.fRadarLongitude = float(tag[4].text)
                stRadarDev.fAngleNorthT = float(tag[5].text)

                RotationVector1 = tag[6].text.split(',')
                RotationVector2 = tag[7].text.split(',')
                RotationVector3 = tag[8].text.split(',')
                TranslationVector = tag[9].text.split(',')
                stRadarDev.fOffset = float(tag[10].text)

                stRadarDev.fRotationVector = np.mat(
                    [[float(RotationVector1[0]), float(RotationVector1[1]), float(RotationVector1[2])],
                     [float(RotationVector2[0]), float(RotationVector2[1]), float(RotationVector2[2])],
                     [float(RotationVector3[0]), float(RotationVector3[1]), float(RotationVector3[2])]])

                stRadarDev.fTranslationVector = np.mat(
                    [float(TranslationVector[0]), float(TranslationVector[1]), float(TranslationVector[2])])
                stLidarParam.listRadarDev.append(stRadarDev)
                nCount += 1
            stLidarParam.nRadarCount = nCount
            # MyLog.writeLog("{}[:{}] - serializationParam-stLidarParam:nNetworkPort={}, fLidarLongitude={},"
            #          " fLidarLatitude={}, fAngleNorthT={}".format(__file__.split('/')[len(__file__.split('/')) - 1],
            #                                                         sys._getframe().f_lineno,stLidarParam.nNetworkPort,
            #                                                         stLidarParam.fLidarLongitude,
            #                                                         stLidarParam.fLidarLatitude,
            #                                                         stLidarParam.fAngleNorthT),
            #                                                         LOG_INFO, True, True)

        # cameraParam
        if stCameraParam is not None:
            stCameraParam.nCameraChannelCount = max(int(root[1][0].text), 1)
            for tag in root[1][1]:
                stCameraDev = structCameraDev()
                stCameraDev.strCameraIp = tag[0].text
                stCameraDev.nCameraChannel = int(tag[1].text)
                stCameraDev.strCameraUsr = tag[2].text
                stCameraDev.strCameraPwd = tag[3].text
                listInParameter = tag[4].text.split(',')
                listRotateMatrix = tag[5].text.split(',')
                listTranslationMatrix = tag[6].text.split(',')
                listDistMatrix = tag[7].text.split(',')
                stCameraDev.in_parameter = np.mat(
                    [[float(listInParameter[0]), float(listInParameter[1]), float(listInParameter[2])],
                     [float(listInParameter[3]), float(listInParameter[4]), float(listInParameter[5])],
                     [float(listInParameter[6]), float(listInParameter[7]), float(listInParameter[8])]])
                stCameraDev.rotate_matrix = np.mat(
                    [float(listRotateMatrix[0]), float(listRotateMatrix[1]), float(listRotateMatrix[2])])
                stCameraDev.translation_matrix = np.mat(
                    [float(listTranslationMatrix[0]), float(listTranslationMatrix[1]), float(listTranslationMatrix[2])])
                stCameraDev.dist_matrix = np.mat(
                    [float(listDistMatrix[0]), float(listDistMatrix[1]), float(listDistMatrix[2]),
                     float(listDistMatrix[3]), float(listDistMatrix[4])])
                stCameraParam.listCameraDev.append(stCameraDev)
            # MyLog.writeLog("{}[:{}] - serializationParam-stCameraParam:nCameraChannelCount={}"
            #          .format(__file__.split('/')[len(__file__.split('/')) - 1],
            #                  sys._getframe().f_lineno,
            #                  stCameraParam.nCameraChannelCount), LOG_INFO, True, True)

        # pcAlgParam
        if stPcAlgParam is not None:
            stPcAlgParam.nSyncIdLimitPcOnline = max(int(root[2][0].text), 0)
            stPcAlgParam.nSyncIdLimitPcOffline = max(int(root[2][1].text), 0)
            stPcAlgParam.fCropBackSize = max(float(root[2][2].text), 0)
            stPcAlgParam.nBmpSize = max(int(root[2][3].text), 0)
            stPcAlgParam.fRadius = max(float(root[2][4].text), 0)
            stPcAlgParam.nLastCount = max(int(root[2][5].text), 0)
            stPcAlgParam.fDis = max(float(root[2][6].text), 0)
            stPcAlgParam.fMinZ = float(root[2][7].text)
            stPcAlgParam.fMaxZ = float(root[2][8].text)
            stPcAlgParam.fCarThreshold = float(root[2][9].text)
            stPcAlgParam.fConeThreshold = float(root[2][10].text)
            stPcAlgParam.nMaxAge = int(root[2][11].text)
            stPcAlgParam.nMinHits = int(root[2][12].text)

            # MyLog.writeLog("{}[:{}] - serializationParam-stPcAlgParam:nSyncIdLimitPcOnline={}, nSyncIdLimitPcOffline={}"
            #          .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno,
            #                  stPcAlgParam.nSyncIdLimitPcOnline, stPcAlgParam.nSyncIdLimitPcOffline), LOG_INFO, True, True)

        # videoAlgParam
        if stVideoAlgParam is not None:
            stVideoAlgParam.nSyncIdLimitVidOnline = max(int(root[3][0].text), 0)
            stVideoAlgParam.nSyncIdLimitVidOffline = max(int(root[3][1].text), 0)
            stVideoAlgParam.nResizeVideoWidth = max(int(root[3][2].text), 416)
            stVideoAlgParam.nResizeVideoHeight = max(int(root[3][3].text), 416)
            # MyLog.writeLog("{}[:{}] - serializationParam-stVideoAlgParam:nSyncIdLimitVidOnline={}, "
            #          "nSyncIdLimitVidOffline={}, nResizeVideoWidth={}, nResizeVideoHeight={}"
            #          .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno,
            #                  stVideoAlgParam.nSyncIdLimitVidOnline,
            #                  stVideoAlgParam.nSyncIdLimitVidOffline,
            #                  stVideoAlgParam.nResizeVideoWidth,
            #                  stVideoAlgParam.nResizeVideoHeight),
            #                  LOG_INFO, True, True)

        # UiParam
        if stUiParam is not None:
            stUiParam.bDebug = bool(int(root[4][0].text))
            stUiParam.bWriteLog = bool(int(root[4][1].text))
            stUiParam.bPrintLog = bool(int(root[4][2].text))
            stUiParam.bAutoConnect = bool(int(root[4][3].text))
            stUiParam.nTimeInterval = max(int(root[4][4].text), 1)
            stUiParam.nCacheFileCountLimit = max(int(root[4][5].text), 0)
            stUiParam.nSaveFileSizeLimit = max(int(root[4][6].text), 104857600)
            stUiParam.bAbnormalRestart = bool(int(root[4][7].text))
            stUiParam.fMemPercentMax = float(root[4][8].text)
            stUiParam.fGpuPercentMax = float(root[4][9].text)
            stUiParam.fFpsMinLimit = float(root[4][10].text)
            stUiParam.strTerminalId = root[4][11].text
            stUiParam.strKafkaNet = root[4][12].text
            stUiParam.strKafkaTopic = root[4][13].text
            stUiParam.bRotatingLane = bool(int(root[4][14].text))
            stUiParam.fRoughLanRotationVector[0] = float(root[4][15].text)
            stUiParam.fRoughLanRotationVector[1] = float(root[4][16].text)
            stUiParam.fRoughLanRotationVector[2] = float(root[4][17].text)
            stUiParam.fRoughLanTranslationVector[0] = float(root[4][18].text)
            stUiParam.fRoughLanTranslationVector[1] = float(root[4][19].text)
            stUiParam.fRoughLanTranslationVector[2] = float(root[4][20].text)

            stUiParam.fFineLanRotationVector[0] = float(root[4][21].text)
            stUiParam.fFineLanRotationVector[1] = float(root[4][22].text)
            stUiParam.fFineLanRotationVector[2] = float(root[4][23].text)
            stUiParam.fFineLanTranslationVector[0] = float(root[4][24].text)
            stUiParam.fFineLanTranslationVector[1] = float(root[4][25].text)
            stUiParam.fFineLanTranslationVector[2] = float(root[4][26].text)
            stUiParam.fBoxRotationAngle = float(root[4][27].text)
            for tag in root[4][28]:
                stUiParam.listAttachedIp.append(tag.text)
            stUiParam.alone_to_together_x = float(root[4][29].text)
            stUiParam.alone_to_together_y = float(root[4][30].text)
            stUiParam.nOfflineFrame = int(root[4][31].text)
            stUiParam.SoftwareStyle = int(root[4][32].text)
            stUiParam.bSaveE1 = int(root[4][33].text)
            stUiParam.bSaveEvent = int(root[4][34].text)
            stUiParam.nPlatformB3Frequency = int(root[4][35].text)
            stUiParam.nPlatformB4Frequency = int(root[4][36].text)
            # MyLog.writeLog("{}[:{}] - serializationParam-stUiParam:"
            #          "nTimeInterval={}, nCacheFileCountLimit={}, nSaveFileSizeLimit={}"
            #          .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno,
            #                  stUiParam.nTimeInterval, stUiParam.nCacheFileCountLimit, stUiParam.nSaveFileSizeLimit),
            #                  LOG_INFO, True, True)

        if g_mapLanAngle is not None:
            strLanAngleFilePath = strExePath + '/Configs/Video/chedao/traffic_lane_id_angle.csv'
            getLanAngleFromCsv(strLanAngleFilePath, g_mapLanAngle)
    else:
        # lidarParam
        if stLidarParam is not None:
            root[0][0].text = str(stLidarParam.nStationCount)
            i = 0
            listAttachedIp = []
            for tag in root[0][1]:
                if tag[0].text is None:
                    i += 1
                    continue
                stStationDev = stLidarParam.listStationDev[i]

                tag[0].text = stStationDev.strStationSrcIp
                if stStationDev.strStationSrcIp != '':
                    listAttachedIp.append(stStationDev.strStationSrcIp)
                tag[4].text = str(stStationDev.fRotationVector[0])
                tag[5].text = str(stStationDev.fRotationVector[1])
                tag[6].text = str(stStationDev.fRotationVector[2])
                tag[7].text = str(stStationDev.fTranslationVector[0])
                tag[8].text = str(stStationDev.fTranslationVector[1])
                tag[9].text = str(stStationDev.fTranslationVector[2])
                i += 1
            stLidarParam.listAttachedIp = listAttachedIp

            root[0][4].text = str(stLidarParam.nNetworkPort)
            root[0][5].text = str(int(stLidarParam.bUseCropRoi))
            root[0][6].text = str(int(stLidarParam.bUseBgdFrameFilter))
            root[0][7].text = str(int(stLidarParam.bLiveUpdateBgdFrame))
            root[0][8].text = str(int(stLidarParam.bUseVoxelPoints))
            root[0][9].text = str(int(stLidarParam.bRotationTranslation))
            root[0][10].text = str(stLidarParam.nUdpServicePort)
            root[0][11].text = str(int(stLidarParam.nEventFeedBackPort))
            root[0][12].text = str(int(stLidarParam.bSendResult))
            root[0][13].text = str(int(stLidarParam.bHighSpeed))

            i = 0
            for tag in root[5][1]:
                stDadarDev = stLidarParam.listRadarDev[i]
                tag[0].text = stDadarDev.strRadarSrcIp
                tag[1].text = stDadarDev.strRadarDstIp
                tag[2].text = str(int(stDadarDev.fRadarSrcPort))
                tag[3].text = str(stDadarDev.fRadarLatitude)
                tag[4].text = str(stDadarDev.fRadarLongitude)
                tag[5].text = str(stDadarDev.fAngleNorthT)

                listRotation = stDadarDev.fRotationVector.tolist()
                listTranslation = stDadarDev.fTranslationVector.tolist()

                tag[6].text = str(listRotation[0][0]) + ',' + str(listRotation[0][1]) + ',' + str(listRotation[0][2])
                tag[7].text = str(listRotation[1][0]) + ',' + str(listRotation[1][1]) + ',' + str(listRotation[1][2])
                tag[8].text = str(listRotation[2][0]) + ',' + str(listRotation[2][1]) + ',' + str(listRotation[2][2])
                tag[9].text = str(listTranslation[0][0]) + ',' + str(listTranslation[0][1]) + ',' + str(
                    listTranslation[0][2])
                i += 1
            root[5][0].text = str(i)
        # cameraParam
        if stCameraParam is not None:
            root[1][0].text = str(stCameraParam.nCameraChannelCount)
            i = 0
            for tag in root[1][1]:
                listCameraDev = stCameraParam.getListCameraDev()
                stCameraDev = listCameraDev[i]
                tag[0].text = stCameraDev.strCameraIp
                tag[1].text = str(stCameraDev.nCameraChannel)
                tag[2].text = stCameraDev.strCameraUsr
                tag[3].text = stCameraDev.strCameraPwd
                i += 1

        # pcAlgParam
        if stPcAlgParam is not None:
            root[2][0].text = str(stPcAlgParam.nSyncIdLimitPcOnline)
            root[2][1].text = str(stPcAlgParam.nSyncIdLimitPcOffline)
            root[2][2].text = str(stPcAlgParam.fCropBackSize)
            root[2][3].text = str(int(stPcAlgParam.nBmpSize))
            root[2][4].text = str(stPcAlgParam.fRadius)
            root[2][5].text = str(stPcAlgParam.nLastCount)
            root[2][6].text = str(stPcAlgParam.fDis)
            root[2][7].text = str(stPcAlgParam.fMinZ)
            root[2][8].text = str(stPcAlgParam.fMaxZ)
            root[2][9].text = str(stPcAlgParam.fCarThreshold)
            root[2][10].text = str(stPcAlgParam.fConeThreshold)
            root[2][11].text = str(stPcAlgParam.nMaxAge)
            root[2][12].text = str(stPcAlgParam.nMinHits)

        # videoAlgParam
        if stVideoAlgParam is not None:
            root[3][0].text = stVideoAlgParam.nSyncIdLimitVidOnline
            root[3][1].text = stVideoAlgParam.nSyncIdLimitVidOffline
            root[3][2].text = stVideoAlgParam.nResizeVideoWidth
            root[3][3].text = stVideoAlgParam.nResizeVideoHeight

        # UiParam
        if stUiParam is not None:
            root[4][0].text = str(int(stUiParam.bDebug))
            root[4][1].text = str(int(stUiParam.bWriteLog))
            root[4][2].text = str(int(stUiParam.bPrintLog))
            root[4][3].text = str(int(stUiParam.bAutoConnect))
            root[4][4].text = str(stUiParam.nTimeInterval)
            root[4][5].text = str(stUiParam.nCacheFileCountLimit)
            root[4][6].text = str(stUiParam.nSaveFileSizeLimit)
            root[4][7].text = str(int(stUiParam.bAbnormalRestart))
            root[4][8].text = str(stUiParam.fMemPercentMax)
            root[4][9].text = str(stUiParam.fGpuPercentMax)
            root[4][10].text = str(stUiParam.fFpsMinLimit)
            root[4][11].text = str(stUiParam.strTerminalId)
            root[4][12].text = str(stUiParam.strKafkaNet)
            root[4][13].text = str(stUiParam.strKafkaTopic)
            root[4][14].text = str(int(stUiParam.bRotatingLane))
            root[4][15].text = str(stUiParam.fRoughLanRotationVector[0])
            root[4][16].text = str(stUiParam.fRoughLanRotationVector[1])
            root[4][17].text = str(stUiParam.fRoughLanRotationVector[2])
            root[4][18].text = str(stUiParam.fRoughLanTranslationVector[0])
            root[4][19].text = str(stUiParam.fRoughLanTranslationVector[1])
            root[4][20].text = str(stUiParam.fRoughLanTranslationVector[2])
            root[4][21].text = str(stUiParam.fFineLanRotationVector[0])
            root[4][22].text = str(stUiParam.fFineLanRotationVector[1])
            root[4][23].text = str(stUiParam.fFineLanRotationVector[2])
            root[4][24].text = str(stUiParam.fFineLanTranslationVector[0])
            root[4][25].text = str(stUiParam.fFineLanTranslationVector[1])
            root[4][26].text = str(stUiParam.fFineLanTranslationVector[2])
            root[4][27].text = str(stUiParam.fBoxRotationAngle)
            root[4][29].text = str(stUiParam.alone_to_together_x)
            root[4][30].text = str(stUiParam.alone_to_together_y)
            root[4][31].text = str(stUiParam.nOfflineFrame)
            root[4][32].text = str(int(stUiParam.SoftwareStyle))
            root[4][33].text = str(int(stUiParam.bSaveE1))
            root[4][34].text = str(stUiParam.bSaveEvent)
            root[4][35].text = str(stUiParam.nPlatformB3Frequency)
            root[4][36].text = str(stUiParam.nPlatformB4Frequency)
        tree.write(strFilePath)
    return


stUiParam = structUiParam()
stLidarParamTemp = structLidarParam()
g_mapLanAngle = {}
serializationParam(stLidarParam=stLidarParamTemp, stUiParam=stUiParam, g_mapLanAngle=g_mapLanAngle)
ENABLE_WRITE_LOG = stUiParam.bWriteLog
ENABLE_PRINT = stUiParam.bPrintLog


# 判断IP是否可达
def isIpReachable(ip):
    # if os.system('ping -n 2 -w 1 %s' % ip):#只发送两个ECHO_REQUEST包
    # 检测IP是否可达，后面的参数用于关闭命令行输出
    if subprocess.call('ping -c 2 -w 3 %s' % ip, shell=True, stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE) == 0:  # 只发送两个ECHO_REQUEST包
        # MyLog.writeLog("{}[:{}] - {} is reachable.".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, ip), LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)
        return True
    else:
        # MyLog.writeLog("{}[:{}] - {} is unreachable.".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, ip), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
        return False


# 从类型名value获取类型的key
def getFusionEnumFromStr(strClass):
    for i in range(len(switch_fusion)):
        if strClass == switch_fusion[i]:
            return i

    return -1


# 从类型名value获取类型的key
def getPcEnumFromStr(strClass):
    for i in range(len(switch_pc)):
        if strClass == switch_pc[i]:
            return i

    return -1


# 从类型名value获取类型的key
def getSendEnumFromStr(strClass):
    for i in range(len(switch_send)):
        if strClass == switch_send[i]:
            return i

    return -1


# 从类型名value获取类型的key
def getSendEnumFromPcEnum(nPcClass):
    for i in range(len(switch_send)):
        if switch_pc[nPcClass] == switch_send[i]:
            return i

    return -1


# 从颜色名value获取颜色的key
def getSendEnumFromColorStr(strColor):
    for i in range(len(switch_color)):
        if strColor == switch_color[i]:
            return i

    return -1


# 从类型名value获取类型的key
def getSendJsonEnumFromStr(strClass):
    for i in range(len(switch_send_json)):
        if strClass == switch_send_json[i]:
            return i

    print("Conversion type err!", strClass)
    return -1


# ===================== 融合结果打包======================
def getHeadInfo(listBoxInfo):
    m = len(listBoxInfo)
    # m = min(100, m)

    start = struct.pack('>BB', 255, 255)
    serial_num = struct.pack('B', 0)
    main_command = struct.pack('B', 225)
    sub_command = struct.pack('B', 2)
    state = struct.pack('B', 0)
    length = m * 36 + 36
    information_lenth = struct.pack('>H', int(length))
    information = start + serial_num + main_command + sub_command + state + information_lenth
    return information


def getBoxNums(listBoxInfo):
    m = len(listBoxInfo)
    # m = min(100, m)

    trans_num = struct.pack('>H', m)
    object_reserve2 = struct.pack('BBBB', 0, 0, 0, 0)
    trans = trans_num + object_reserve2
    return trans


# 打包识别结果的目标框信息
def packetBoxInfo(listBoxInfo, logti_t, lat_t, angle_north_t, SingleFrame):
    m = len(listBoxInfo)
    trans_pack = []
    for index in range(m):
        boxInfo = listBoxInfo[index]
        if boxInfo.strSource == "camera":  # or (boxInfo.strSource == "fusion"):
            continue  # 融合结果打包，如果出现camera，则表明数据异常，需丢弃！
        object_id = struct.pack('>H', int(boxInfo.boxId % 65000))
        nClass = getSendEnumFromStr(boxInfo.strClass)
        if nClass == -1:
            nClass = 0
        object_type = struct.pack('B', nClass)
        if boxInfo.nConfidence > 255 or boxInfo.nConfidence < 0:
            boxInfo.nConfidence = int(boxInfo.nConfidence / 100)
        object_confidence = struct.pack('B', boxInfo.nConfidence)
        if boxInfo.fcolor < 0:
            object_color = struct.pack('B', 0)
        else:
            object_color = struct.pack('B', int(round(boxInfo.fcolor)))
        if boxInfo.strSource == "Radar":
            object_source = struct.pack('B', 3)
        elif boxInfo.strSource == "PcRfusion":
            object_source = struct.pack('B', 4)
        else:
            object_source = struct.pack('B', 0)
        object_laneid = struct.pack('B', int(boxInfo.nLane))
        if boxInfo.nBaseStationId > 0 and boxInfo.nBaseStationId < 256:
            object_BaseStationId = struct.pack('B', int(boxInfo.nBaseStationId))
        else:
            object_BaseStationId = struct.pack('B', 0)

        # 跑在线算法有单基站帧号，跑离线org没有保存单基站帧号
        try:
            boxInfo.nsingleFranme = SingleFrame["{}".format(int(boxInfo.nBaseStationId))]
            object_nsingleFranme = struct.pack('>H', boxInfo.nsingleFranme)
        except:
            object_nsingleFranme = struct.pack('>H', 0)

        logti, lat = XYZ_To_BLH(logti_t, lat_t, boxInfo.boxCenter[0], boxInfo.boxCenter[1], angle_north_t)
        object_log = struct.pack('>i', int(logti * 1e7))
        object_lat = struct.pack('>i', int(lat * 1e7))
        object_alti = struct.pack('>h', 0)
        speed = boxInfo.boxSpeed * 100
        object_speed = struct.pack('>H', int(round(speed)))
        pitch = (180 + angle_north_t + boxInfo.boxAngle / PI_rads) % 360
        object_pitch = struct.pack('>H', int(round(pitch)))
        length = boxInfo.boxSize[1] * 100
        object_length = struct.pack('>H', int(round(length)))
        wide = boxInfo.boxSize[0] * 100
        object_wide = struct.pack('>H', int(round(wide)))
        height = boxInfo.boxSize[2] * 100
        object_height = struct.pack('>H', int(round(height)))
        center_x = boxInfo.boxCenter[0] * 100
        # bsmybol = 0
        # if center_x < 0:
        #     center_x = -center_x
        # else:
        #     bsmybol += 1
        # if center_x > 65535:
        #     center_x = 65535
        object_center_x = struct.pack('>H', 0)
        center_y = boxInfo.boxCenter[1] * 100
        # if center_y < 0:
        #     center_y = -center_y
        # else:
        #     bsmybol += 2
        # if center_y > 65535:
        #     center_y = 65535
        object_center_y = struct.pack('>H', 0)
        center_z = boxInfo.boxCenter[2] * 100
        object_center_z = struct.pack('>h', int(round(center_z)))
        # object_symbol = struct.pack('B', bsmybol)
        object_BaseStationBoxId = struct.pack('>H', int(boxInfo.nBaseStationBoxId % 65000))
        sigle_object = object_id + object_type + object_confidence + object_color + object_source + object_laneid + \
                       object_BaseStationId + object_log + object_lat + object_alti + object_speed + object_pitch + object_length + \
                       object_wide + object_height + object_center_x + object_center_y + object_center_z + object_BaseStationBoxId + \
                       object_nsingleFranme
        trans_pack.append(sigle_object)
    return trans_pack


''


# ===================== 融合结果打包======================


# 读取指定帧
def readOneFrame(strCachePath, nFrameId):
    strfileName = '{}{}{}{}'.format(strCachePath, PcCacheFile, nFrameId, ".npy")
    if not os.path.exists(strfileName):
        return False, np.array([])

    frame = np.load(strfileName)  # np.load需添加异常处理，防止文件出错!
    return True, frame


def savePosAsCSVFromPcap(listSelectFilePath):
    for selectFilePath in listSelectFilePath:
        listIndexOffset = []  # 存放当前pcap文件每帧第一包的文件位置
        strPcapFile = selectFilePath + ".pcap"

        strIndexPath = selectFilePath + "_index.csv"  # 存放pcap每帧文件位置的csv文件路径
        if my_isExist(strIndexPath):
            continue
        else:
            with open(strPcapFile, 'rb') as f:
                fileLength = int(f.seek(0, io.SEEK_END))  # 获取文件大小
                f.seek(0)
                posOffset = 24
                __head = None
                _packet = None
                ctx = None
                buffSize = 1392  # buffSize不包含包头，即每个pcap数据包大小是16字节包头+1392字节数据包，共1408字节
                nFrameSeqTemp = 0
                isOneFrame = False
                # 读取24字节pacp包头
                ctx = f.read(posOffset)
                if len(ctx) != posOffset:
                    print("pcap invalid\n")
                    return
                posPackStart = f.tell()
                listIndexOffset.append(posPackStart)
                nFrameSeqTemp = int((nFrameSeqTemp + 1) % MAX_FRAME_ID)

                while posOffset < fileLength:
                    # 记录一包数据开始的位置
                    posPackStart = f.tell()
                    # 分析包头(包头占16字节)
                    ctx = f.read(16)
                    size = len(ctx)
                    if size == 16:
                        posOffset += 16
                        if _packet is None:
                            _packet = Packet()
                        ctx, size = _packet.parse(ctx)
                        packLen = _packet.head.incl  # 得到包长度
                        if packLen == buffSize:
                            ctx = f.read(packLen)
                            posTemp = f.tell()
                            _packet.parse(ctx)
                            if _packet.finish():
                                _mac = _packet.data
                                _net, _strType = _mac.data
                                if "ip" not in _strType:
                                    print("no IP packet!")
                                    posOffset += packLen
                                    _packet = None
                                    continue
                                _trans = _net.data
                                if _trans.__class__.__name__ != "UDP":
                                    print("no UDP packet!")
                                    posOffset += packLen
                                    _packet = None
                                    continue
                                pl_data = _trans.data
                                if len(pl_data) == 1350:
                                    # 输入一包数据和这一包开始的文件位置，如果这一包中的角度比之前的角度小，则认为这一包是下一帧的开始，记录文件位置
                                    t1 = time.time()
                                    isOneFrame = getOneFrameStartPos(pl_data)

                                    if isOneFrame:
                                        nFrameSeqTemp = int((nFrameSeqTemp + 1) % MAX_FRAME_ID)
                                        posOffset += packLen
                                        _packet = None

                                        if posPackStart not in listIndexOffset:
                                            listIndexOffset.append(posPackStart)
                                        nFrameSeqTemp = int((nFrameSeqTemp + 1) % MAX_FRAME_ID)
                                    # print("----------loadPerPacpCostTime---------", time.time() - t1)
                                    posOffset += packLen
                                    _packet = None
                        else:
                            f.read(packLen)  # 跳过长度不是1392字节的包
                            posOffset += packLen
                    else:
                        print("pcap invalid\n")
                        return

                nFrameCountPerFile = len(listIndexOffset)

                for i in range(len(listIndexOffset)):
                    my_save2csv(selectFilePath + "_index.csv", [listIndexOffset[i]])
                # continue


def readOneFrameFromPcap(nFrameId, listIndexOffset, ver_hori_angle, f):
    _packet = None
    frame = None
    now_angle_np = np.ones(1)
    head_data = np.ones(30)
    cacheSize = len(listIndexOffset)
    if cacheSize == 0 or cacheSize < nFrameId + 1:
        return False, np.array([])
    fileSize = int(f.seek(0, io.SEEK_END))
    posStart = int(listIndexOffset[nFrameId])
    last_angle = -1
    f.seek(posStart, 0)

    while posStart < fileSize:
        ctx = f.read(16)
        posStart += 16
        size = len(ctx)

        if size == 16:
            if _packet is None:
                _packet = Packet()
            ctx, size = _packet.parse(ctx)
            packLen = _packet.head.incl
            if packLen == 1392:
                posStart += packLen
                f.seek(42, io.SEEK_CUR)
                pl_data = f.read(1350)
                packet_data = np.array([int(i) for i in pl_data])
                if len(packet_data) == 1350:
                    bag_data = get_bag_data(packet_data, ver_hori_angle, now_angle_np, head_data)
                    now_angle = now_angle_np[0]
                    if now_angle > last_angle:
                        if last_angle == -1:
                            frame = bag_data
                            last_angle = now_angle
                        else:
                            frame = np.concatenate((frame, bag_data), axis=0)
                            last_angle = now_angle
                    else:
                        break
                else:
                    print("Error packet!")
                _packet = None
            else:
                f.read(packLen)
                _packet = None
                posStart += packLen

    return True, frame


# 读取指定帧from pcd file
def readOneFrameFromPcd(strPath, nFrameId):
    strfileName = '{}{}{}{}'.format(strPath, '000', nFrameId, ".pcd")
    if not os.path.exists(strfileName):
        return False, np.array([])

    points = []
    # 读取pcd文件,从pcd的第12行开始是三维点
    with open(strfileName) as f:
        for line in f.readlines()[11:len(f.readlines()) - 1]:
            strs = line.split(' ')
            points.append([float(strs[0]), float(strs[1]), float(strs[2].strip())])

    frame = np.array(points)
    return True, frame


def readOneFrameFromPcd2(strFilePath):
    if not os.path.exists(strFilePath):
        return False, np.array([])

    points = []
    # 读取pcd文件,从pcd的第12行开始是三维点
    with open(strFilePath) as f:
        for line in f.readlines()[11:len(f.readlines()) - 1]:
            strs = line.split(' ')
            points.append([float(strs[0]), float(strs[1]), float(strs[2]), float(strs[3])])

    frame = np.array(points)
    return True, frame


def getOneFrameStartPos(packet_data):
    last_angle = -1
    for i in range(0, 1320, 132):
        h_temp = int((packet_data[2 + i] + packet_data[3 + i] * 256) / 10)  #####4m的为-270
        now_angle = (h_temp / 10) % 360
        # print("now,last", now_angle, last_angle)
        if now_angle < last_angle or now_angle == 0.0:
            last_angle = 0
            return True
        last_angle = now_angle
    return False


# 读取指定帧
def readOneCache(strCachePath, nFrameId):
    strfileName = '{}{}{}'.format(strCachePath, PcCacheFile, nFrameId)
    if not os.path.exists(strfileName + ".dat"):
        # print("readOneCache failed!")
        return False, None

    fileCache = shelve.open(strfileName, "r")
    dataTemp = fileCache[CacheDataKey]
    # print("readOneCache successfully!")
    return True, dataTemp


# 读取指定帧
def saveOneCache(strCachePath, dstData):
    my_mkdir(strCachePath)
    strfileName = '{}{}{}'.format(strCachePath, PcCacheFile, int(dstData.nFrameId))
    fileCache = shelve.open(strfileName)
    fileCache[CacheDataKey] = dstData
    fileCache.close()


# 读取指定帧
def deleteOneCache(strFileName):
    timeT = QTime()
    timeT.start()
    strFileNameTemp = '{}{}'.format(strFileName, ".bak")
    my_delFile(strFileNameTemp)

    strFileNameTemp = '{}{}'.format(strFileName, ".dat")
    my_delFile(strFileNameTemp)

    strFileNameTemp = '{}{}'.format(strFileName, ".dir")
    my_delFile(strFileNameTemp)

    # print("deleteOneCache %d" % timeT.elapsed())
    timeT.restart()


def readTempPcap():
    tempPcapPath = TempPath + "Temp.pcap"
    with open(tempPcapPath, 'rb') as f:
        pcap = dpkt.pcap.Reader(f)
        for timestamp, raw_buf in pcap:  # timestamp时间戳，raw_buf包中的原始数据
            eth = dpkt.ethernet.Ethernet(raw_buf)
            return eth


def my_mkPcapByTime(strPath=""):
    strPathTemp = strPath
    if len(strPathTemp) == 0:
        strPathTemp = SavePath

    if strPathTemp[len(strPathTemp) - 1] != '/':
        strPathTemp = strPathTemp + '/'

    strSaveFileName = time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
    strSaveFilePath = '{}{}{}'.format(strPathTemp, strSaveFileName, '.pcap')
    my_mkdir(strPathTemp)
    # f = open(strSaveFilePath, 'wb')
    # f.close()
    return strSaveFilePath


def my_mkVideoByTime(strPath=""):
    strPathTemp = strPath
    if len(strPathTemp) == 0:
        strPathTemp = SavePath

    if strPathTemp[len(strPathTemp) - 1] != '/':
        strPathTemp = strPathTemp + '/'

    strSaveFileName = time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
    strSaveFilePath = '{}{}{}'.format(strPathTemp, strSaveFileName, '.avi')
    my_mkdir(strPathTemp)
    # f = open(strSaveFilePath, 'wb')
    # f.close()
    return strSaveFilePath


def my_delFile(filePath):
    try:
        isExists = my_isExist(filePath)
        if isExists:
            if os.path.isfile(filePath):
                os.remove(filePath)
                return True
            else:
                return False
        else:
            return True
    except:
        print("{}[:{}] - except! my_delFile err!")
        print("{}[:{}] - except! Call stack:\n{}"
              .format(__file__.split('/')[len(__file__.split('/')) - 1],
                      sys._getframe().f_lineno, traceback.format_exc()))
        return False


def my_renameFile(oldFilePath, newFilePath):
    isOldExists = my_isExist(oldFilePath)
    isNewExists = my_isExist(newFilePath)
    if isOldExists and (not isNewExists):
        # print("rename old:{}, new:{}".format(oldFilePath, newFilePath))
        os.rename(oldFilePath, newFilePath)


def my_renameFile2(oldFilePath, newFilePath, bOverWrite=False):
    isOldExists = my_isExist(oldFilePath)
    isNewExists = my_isExist(newFilePath)
    if isOldExists:
        if bOverWrite or (not isNewExists):
            # print("rename old:{}, new:{}".format(oldFilePath, newFilePath))
            os.rename(oldFilePath, newFilePath)


def my_csv2npy(filePath):
    my_delDir(BoxCachePath)
    my_mkdir(BoxCachePath)

    birth_data = []
    with open(filePath) as csvfile:
        csv_reader = csv.reader(csvfile)  # 使用csv.reader读取csvfile中的文件
        for row in csv_reader:  # 将csv 文件中的数据保存到birth_data中
            birth_data.append(row)

    birth_data = [[float(x) for x in row] for row in birth_data]  # 将数据从string形式转换为float形式
    birth_data = np.array(birth_data)  # 将list数组转化成array数组便于查看数据结构
    boxData = birth_data[:, 10:35]
    frame = np.array([])
    nBoxCount = 0
    nFrameCount = 0
    for i in range(0, boxData.shape[0]):
        data = boxData[i]
        if data[24] == nFrameCount:
            # frame = np.insert(frame, nBoxCount, values=data[0:24], axis=0)
            frame = np.concatenate((frame, data[0:24]), axis=0)
            nBoxCount += 1
        else:
            strFileTemp = '{}{}{}'.format(BoxCachePath, BoxCacheFile, nFrameCount)
            np.save(strFileTemp, frame)  # 缓存
            nFrameCount += 1

            nBoxCount = 0
            frame = np.array([])
            frame = np.insert(frame, nBoxCount, values=data[0:24], axis=0)
            nBoxCount += 1


def my_csv2npArrSorted(filePath, shape):
    if not my_isExist(filePath):
        return None

    birth_data = []
    with open(filePath) as csvfile:
        csv_reader = csv.reader(csvfile)  # 使用csv.reader读取csvfile中的文件
        for row in csv_reader:  # 将csv 文件中的数据保存到birth_data中
            birth_data.append(row)

    birth_data = [[float(x) for x in row] for row in birth_data]  # 将数据从string形式转换为float形式
    birth_data = np.array(birth_data)  # 将list数组转化成array数组便于查看数据结构
    boxData = birth_data  # birth_data[:, 10:35]
    frame = np.array([])
    for i in range(0, boxData.shape[0]):
        data = boxData[i]
        if i < boxData.shape[0] - 1:
            dataNext = boxData[i + 1]
            if data[0] > dataNext[0]:
                continue
        frame = np.concatenate((frame, data), axis=0)

    try:
        frame = frame.reshape(shape)
    except:
        MyLog.writeLog("{}[:{}] - shape is error!"
                       .format(__file__.split('/')[len(__file__.split('/')) - 1],
                               sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
        return None

    return frame


def my_csv2npArr(filePath, shape):
    if not my_isExist(filePath):
        return None

    birth_data = []
    with open(filePath) as csvfile:
        csv_reader = csv.reader(csvfile)  # 使用csv.reader读取csvfile中的文件
        for row in csv_reader:  # 将csv 文件中的数据保存到birth_data中
            birth_data.append(row)

    birth_data = [[float(x) for x in row] for row in birth_data]  # 将数据从string形式转换为float形式
    birth_data = np.array(birth_data)  # 将list数组转化成array数组便于查看数据结构
    boxData = birth_data  # birth_data[:, 10:35]
    frame = np.array([])
    for i in range(0, boxData.shape[0]):
        data = boxData[i]
        frame = np.concatenate((frame, data), axis=0)

    try:
        frame = frame.reshape(shape)
    except:
        MyLog.writeLog("{}[:{}] - shape is error!"
                       .format(__file__.split('/')[len(__file__.split('/')) - 1],
                               sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
        return None

    return frame


def my_save2csv(strSaveFilePath, npBoxInfo):
    with open(strSaveFilePath, "a") as csvfile:
        csv_writer = csv.writer(csvfile)  # 使用csv.reader读取csvfile中的文件
        csv_writer.writerow(npBoxInfo)


def my_saveRows2csv(strSaveFilePath, npBoxInfo):
    with open(strSaveFilePath, "a") as csvfile:
        csv_writer = csv.writer(csvfile)  # 使用csv.reader读取csvfile中的文件
        csv_writer.writerows(npBoxInfo)


def my_save2txt(strSaveFilePath, strSaveInfo):
    with open(strSaveFilePath, "a") as f:
        f.write(strSaveInfo)


# 获取ps执行结果
def get_ps():
    cmd = 'ps ax -o pid,ppid,cmd'
    p = Popen(shlex.split(cmd), stdout=PIPE)
    return p.stdout.readlines()[1:]


def split(s):
    s = s.split()
    return str(s[0], encoding="utf-8"), str(s[1], encoding="utf-8"), str(s[2], encoding="utf-8")


# 解析ps执行结果, 取出pid、ppid等信息
def parser_ps(data):
    procs = []
    for l in data:
        pid, ppid, cmd = [i.strip() for i in split(l)]
        procs.append({'pid': int(pid), 'ppid': int(ppid), 'cmd': cmd})
    return procs


# 根据pid递归查找所有关联进程的pid
def getPidTreeByPid(pid, procs, pidTree):
    root = [p for p in procs if p['pid'] == pid][0]
    if root['pid'] not in pidTree:
        pidTree.append(root['pid'])
    if root['ppid'] not in pidTree:
        pidTree.append(root['ppid'])

    childs = [proc for proc in procs if proc['ppid'] == pid]
    if childs:
        for c in childs:
            getPidTreeByPid(c['pid'], procs, pidTree)


# 杀掉除自己以外的所有同名进程
def my_killSelfSamePro():
    currentId = os.getpid()

    # 更具当前进程的id，获取当前进程对象
    proc = psutil.Process(pid=currentId)

    # 查找所有与当前进程同名的进程id
    allPyIds = [p.pid for p in psutil.process_iter() if proc.name() in str(p.name)]
    allPyIds.remove(currentId)

    # 去除自己以及自己的关联进程后，才是真正需要杀掉的进程
    if len(allPyIds) > 0:
        MyLog.writeLog("{}[:{}] - curPro:{}, kill:{}".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                             sys._getframe().f_lineno, currentId, allPyIds),
                       LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
        for PyId in allPyIds:
            os.kill(PyId, signal.SIGTERM)

    MyLog.writeLog(
        "{}[:{}] - The environment has been cleaned!".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                             sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG,
        ENABLE_PRINT)
    # os.kill(currentId, signal.SIGTERM)


# 杀掉除自己进程树以外的所有同名进程
def my_killProExceptSelf():
    # with open("./check.txt", "w") as f:
    #     f.write(datetime.datetime.now().strftime("%Y%m%d%H%M%S"))

    # 查找自己的关联进程
    pidTree = []
    data = get_ps()
    procs = parser_ps(data)
    currentId = os.getpid()
    getPidTreeByPid(currentId, procs, pidTree)
    MyLog.writeLog("{}[:{}] - pro tree:{}".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                  sys._getframe().f_lineno, pidTree), LOG_INFO, ENABLE_WRITE_LOG,
                   ENABLE_PRINT)

    # 更具当前进程的id，获取当前进程对象
    proc = psutil.Process(pid=currentId)

    # 查找所有与当前进程同名的进程id
    allPyIds = [p.pid for p in psutil.process_iter() if proc.name() in str(p.name)]

    # 去除自己以及自己的关联进程后，才是真正需要杀掉的进程
    PyIdsToKill = [x for x in allPyIds if x not in pidTree]
    if len(PyIdsToKill) > 0:
        MyLog.writeLog("{}[:{}] - curPro:{}, kill:{}".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                             sys._getframe().f_lineno, currentId, PyIdsToKill),
                       LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
        for PyId in PyIdsToKill:
            os.kill(PyId, signal.SIGTERM)

    startWatchDog()
    MyLog.writeLog(
        "{}[:{}] - The environment has been cleaned!".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                             sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG,
        ENABLE_PRINT)


# def checkKill():
#     bOk = my_isExist("./check.txt")
#     if not bOk:
#         my_killProExceptSelf()
#     else:
#         try:
#             with open("./check.txt", "r") as f:
#                 strTime = f.read()
#         except:
#             my_killProExceptSelf()
#             return
#
#         dataLast = datetime.datetime.strptime(strTime, '%Y%m%d%H%M%S')
#         dataNow = datetime.datetime.now()
#         delta = abs(dataNow - dataLast)
#         if delta.days > 0:
#             my_killProExceptSelf()
#             print("{}[:{}] - delta.days > 0! {} {}".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, dataNow.strftime("%Y-%m-%d %H:%M:%S"), dataLast.strftime("%Y-%m-%d %H:%M:%S")))
#         if delta.seconds > 10:  # 10s
#             my_killProExceptSelf()
#             print("{}[:{}] - delta.seconds > 10! {} {}".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, dataNow.strftime("%Y-%m-%d %H:%M:%S"), dataLast.strftime("%Y-%m-%d %H:%M:%S")))


def getBoxFromNpy(nFrameId):
    strfileName = '{}{}{}{}'.format(BoxCachePath, BoxCacheFile, nFrameId, ".npy")
    if not os.path.exists(strfileName):
        return False, np.array([])

    frame = np.load(strfileName)
    boxFrame = frame.reshape(-1, 8, 3)
    return True, boxFrame


def get_latest_pcap_name(file_path):
    if not my_isExist(file_path):
        return ""

    dir_list = os.listdir(file_path)
    if not dir_list:
        return ""
    else:
        # 注意，这里使用lambda表达式，将文件按照最后修改时间顺序升序排列
        # os.path.getmtime() 函数是获取文件最后修改时间
        # os.path.getctime() 函数是获取文件最后创建时间
        dir_list = sorted(dir_list, key=lambda x: os.path.getmtime(os.path.join(file_path, x)))
        # print(dir_list)

        nLen = len(dir_list)
        i = nLen - 1
        while i >= 0:
            filename, extension = os.path.splitext(dir_list[i])
            if extension == ".pcap":
                return file_path + "/" + filename
            i -= 1

        return ""


# ========================= 时间校验 开始 ===========================
# 密钥key 长度必须为16（AES-128）、24（AES-192）、或32（AES-256）Bytes 长度.目前AES-128足够用
from Crypto.Cipher import AES
from Crypto import Random
from binascii import b2a_hex, a2b_hex
import uuid
import psutil
from base64 import b64encode, b64decode

key = b'yaho! biubiubiu~'
usAuthorized = 0  # 0：未激活  1：永久激活  2：重置三个月试用期 3：重置一年试用期
bAddUsageTime = False
strMsgCheck1 = "WJ_ISFP_Release"
strMsgCheck2 = "Mac:"
nTimeLen = 14
strCheckAuthorizationPath = "/usr/lib/libgcgCloseVM7dd.so"
strRecordPath = "/usr/lib/libgcgCloseVM7dd_rec.so"


# 多网卡 mac
def get_all_mac_address():
    listMac = []
    dic = psutil.net_if_addrs()
    for adapter in dic:
        if 'docker' in adapter:  # docker的mac地址一直变，所以要过滤掉
            continue

        snicList = dic[adapter]
        mac = 'None'
        for snic in snicList:
            if snic.family.name in {'AF_LINK', 'AF_PACKET'}:
                mac = snic.address
                mac = mac.replace(':', '')  # 替换掉换行符
                if mac == "000000000000":
                    continue
                listMac.append(mac)

    return listMac


# 获取没有连接网线的网卡的mac地址
def get_no_connect_mac_address():
    listMac = []
    dic = psutil.net_if_addrs()
    for adapter in dic:
        if 'docker' in adapter:  # docker的mac地址一直变，所以要过滤掉
            continue

        bNoConnect = True
        snicList = dic[adapter]
        mac = 'None'
        for snic in snicList:
            if snic.family.name in {'AF_INET', 'AF_INET6'}:
                bNoConnect = False
                break

            if snic.family.name in {'AF_LINK', 'AF_PACKET'}:
                mac = snic.address
                mac = mac.replace(':', '')  # 替换掉换行符
                if mac == "000000000000":
                    bNoConnect = False
                    continue

        if bNoConnect:
            listMac.append(mac)

    return listMac


# 获取同网段IP对应的网卡的mac地址
def get_same_net_segment_mac_address(strIp):
    if strIp == "":
        return "None"

    listStr = strIp.split('.')
    if len(listStr) != 4:
        return "None"

    mac = 'None'
    strSegment = listStr[2]
    dic = psutil.net_if_addrs()
    for adapter in dic:
        if 'docker' in adapter:  # docker的mac地址一直变，所以要过滤掉
            continue

        mac = 'None'
        bSameSegment = False
        snicList = dic[adapter]
        for snic in snicList:
            if snic.family.name == 'AF_INET':
                bSameSegment = True
                strIpTemp = snic.address
                listStrTemp = strIpTemp.split('.')
                if len(listStrTemp) != 4 or listStrTemp[2] != strSegment:
                    bSameSegment = False
                    break
                elif mac != "None":
                    break

            if snic.family.name in {'AF_LINK', 'AF_PACKET'}:
                mac = snic.address
                mac = mac.replace(':', '')  # 替换掉换行符
                if mac == "000000000000":
                    bSameSegment = False
                    break

                if bSameSegment:
                    break

        if bSameSegment:
            return mac

    return "None"


def get_all_ip_address():
    listIp = []
    dic = psutil.net_if_addrs()
    for adapter in dic:
        if 'docker' in adapter:  # docker的mac地址一直变，所以要过滤掉
            continue

        snicList = dic[adapter]
        strIp = 'None'
        for snic in snicList:
            if snic.family.name in {'AF_LINK', 'AF_INET'}:
                strIp = snic.address
                listIp.append(strIp)

    return listIp


def get_mac_address():
    node = uuid.getnode()
    mac = uuid.UUID(int=node).hex[-12:]
    return mac


def getHdSerialNo():
    strDev = ""
    strDevSpare = ""
    p = subprocess.Popen('df -hl', shell=True, stdout=subprocess.PIPE, universal_newlines=True)
    out, err = p.communicate()
    for line in out.splitlines():
        listStr = line.split()
        if "/home" == listStr[len(listStr) - 1] and "/dev" in listStr[0]:
            strDev = listStr[0]
            # print(strDev)
        if strDevSpare == "" and "/dev" in listStr[0]:
            strDevSpare = listStr[0]

    if strDev == "":
        strDev = strDevSpare

    strSerialNo = ""
    pHdparm = subprocess.Popen("echo wanji|sudo -S hdparm -i {}".format(strDev), shell=True, stdout=subprocess.PIPE,
                               universal_newlines=True)
    out1, err1 = pHdparm.communicate()
    for line in out1.splitlines():
        if "SerialNo" in line:
            strSerialNo = line[line.index("SerialNo=") + len("SerialNo="):]
            # print(strSerialNo)

    if strSerialNo == "":
        return "S3YLNG0M704378T"

    return strSerialNo


def getHdRemainingSpace(strPath):
    if strPath == "":
        MyLog.writeLog("{}[:{}] - getHdRemainingSpace Error!! strPath is empty!"
                       .format(__file__.split('/')[len(__file__.split('/')) - 1],
                               sys._getframe().f_lineno), LOG_ERROR, ENABLE_WRITE_LOG, True)
        return -1

    try:
        # 获取目录的磁盘信息
        info = os.statvfs(strPath)
        fSpace = round(info.f_bsize * info.f_bavail / 1024 / 1024, 2)
    except:
        MyLog.writeLog("{}[:{}] - getHdRemainingSpace Error!! fSpace Error!!"
                       .format(__file__.split('/')[len(__file__.split('/')) - 1],
                               sys._getframe().f_lineno), LOG_ERROR, ENABLE_WRITE_LOG, True)
        return -1

    return fSpace


# def getHdRemainingSpace(strPath):
#     if strPath == "":
#         MyLog.writeLog("{}[:{}] - getHdRemainingSpace Error!! strPath is empty!"
#                        .format(__file__.split('/')[len(__file__.split('/')) - 1],
#                                sys._getframe().f_lineno), LOG_ERROR, ENABLE_WRITE_LOG, True)
#         return -1
#
#     try:
#         p = subprocess.Popen('df {}'.format(strPath), shell=True, stdout=subprocess.PIPE, universal_newlines=True)
#         out, err = p.communicate()
#         listLines = out.splitlines()
#         if len(listLines) < 2:
#             MyLog.writeLog("{}[:{}] - getHdRemainingSpace Error!! len(listLines) < 2!"
#                            .format(__file__.split('/')[len(__file__.split('/')) - 1],
#                                    sys._getframe().f_lineno), LOG_ERROR, ENABLE_WRITE_LOG, True)
#             return -1
#
#         fSpace = round(float(listLines[1].split()[3]) / 1024 / 1024, 3)
#     except:
#         MyLog.writeLog("{}[:{}] - getHdRemainingSpace Error!! fSpace Error!!"
#                        .format(__file__.split('/')[len(__file__.split('/')) - 1],
#                                sys._getframe().f_lineno), LOG_ERROR, ENABLE_WRITE_LOG, True)
#         return -1
#
#     return fSpace


# 对机器的识别码做简单加密
def encodeComputerID(strMac=""):
    if strMac == "":
        listMac = get_all_mac_address()
        strMac = listMac[0]
    num = getHdSerialNo()
    strId = strMac + num
    bytesId = bytes(strId, encoding="utf8")
    return b64encode(bytesId)


def decodeComputerID(encodeId):
    bytesDecodeId = b64decode(encodeId)
    strDecodeId = str(bytesDecodeId, encoding="utf8")
    strMac = strDecodeId[:12]
    strNum = strDecodeId[12:]
    return strMac, strNum


def my_encode_check(strPath):
    global bAddUsageTime
    strText = ""
    if os.path.exists(strPath):
        f = open(strPath, 'rb')
        strText = f.read()
        f.close()

    # 要加密的明文
    listMac = get_all_mac_address()
    if len(listMac) == 0:
        MyLog.writeLog(
            "{}[:{}] - my_encode_check: listMac empty!!".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                sys._getframe().f_lineno), LOG_ERROR, ENABLE_WRITE_LOG,
            ENABLE_PRINT)
        return False, False, -1

    strSerialNo = getHdSerialNo()
    if strSerialNo == "":
        return False, False, -1

    bFirst = False
    nDaysLeft = -1
    strMsgCheck3 = listMac[0]
    strMsgCheck4 = strSerialNo
    data = strMsgCheck1 + str(0) + str(usAuthorized) + strMsgCheck2 + strMsgCheck3 + strMsgCheck4
    if strText != "":
        # 解密的话要用key和iv生成新的AES对象
        mydecrypt = AES.new(key, AES.MODE_CFB, strText[:16])
        # 使用新生成的AES对象，将加密的密文解密
        decrypttext = mydecrypt.decrypt(strText[16:])
        strText = decrypttext.decode()

        str1 = strText[0:len(strMsgCheck1)]
        if str1 != strMsgCheck1:
            MyLog.writeLog(
                "{}[:{}] - my_encode_check: str1 Error!! {}".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                    sys._getframe().f_lineno, str1), LOG_ERROR,
                ENABLE_WRITE_LOG, ENABLE_PRINT)
            return False, False, -1

        pos = strText.find(strMsgCheck2)
        if pos < 0:
            MyLog.writeLog(
                "{}[:{}] - my_encode_check: pos Error!! ".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                 sys._getframe().f_lineno), LOG_ERROR, ENABLE_WRITE_LOG,
                ENABLE_PRINT)
            return False, False, -1

        str3 = strText[pos + len(strMsgCheck2):pos + len(strMsgCheck2) + len(strMsgCheck3)]
        # if str3 != strMsgCheck3:
        if str3 not in listMac:
            MyLog.writeLog(
                "{}[:{}] - my_encode_check: str3 Error!! {}".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                    sys._getframe().f_lineno, str3), LOG_ERROR,
                ENABLE_WRITE_LOG, ENABLE_PRINT)
            return False, False, -1
        strMsgCheck3 = str3
        # print("my_encode_check-get mac:", strMsgCheck3)

        str4 = strText[len(strText) - len(strMsgCheck4):len(strText)]
        if str4 != strMsgCheck4:
            MyLog.writeLog(
                "{}[:{}] - my_encode_check: str4 Error!! {} ".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                     sys._getframe().f_lineno, str4), LOG_ERROR,
                ENABLE_WRITE_LOG, ENABLE_PRINT)
            return False, False, -1

        # usAuthorizedTemp: 0：未激活  1：已永久激活  2：三个月试用期 3：一年试用期
        usAuthorizedTemp = int(strText[pos - 1:pos])
        if usAuthorizedTemp == 1:
            MyLog.writeLog("{}[:{}] - my_encode_check: already Activated! ".format(
                __file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_ERROR,
                           ENABLE_WRITE_LOG, ENABLE_PRINT)
            return True, False, "Activated"

        # usageTimeLimitTemp = usageTimeLimit  # 三个月试用期
        # if usAuthorizedTemp == 3:
        #     usageTimeLimitTemp = usageTimeLimit2  # 一年试用期

        nMinutesLeft = int(strText[len(strMsgCheck1):pos - 1])
        nDaysLeft = round((nMinutesLeft / 1440), 1)  # （总时间-已使用的时间）/ 一天的时间（单位都是分钟）= 剩余总天数
        if nDaysLeft <= 7:
            MyLog.writeLog("{}[:{}] - my_encode_check: Expiring! DaysLeft={}".format(
                __file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, nDaysLeft), LOG_ERROR,
                           ENABLE_WRITE_LOG, ENABLE_PRINT)

        if nMinutesLeft - 1 <= 0:
            MyLog.writeLog("{}[:{}] - my_encode_check: The trial period has ended! ".format(
                __file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_ERROR,
                           ENABLE_WRITE_LOG, ENABLE_PRINT)
            return False, False, -1

        strUsageTime = str(nMinutesLeft)
        if bAddUsageTime:
            strUsageTime = str(nMinutesLeft - 1)

        data = strMsgCheck1 + strUsageTime + str(usAuthorizedTemp) + strMsgCheck2 + strMsgCheck3 + strMsgCheck4
    else:
        bFirst = True

    bAddUsageTime = True

    # 生成长度等于AES块大小的不可重复的密钥向量
    iv = Random.new().read(AES.block_size)

    # 使用key和iv初始化AES对象, 使用MODE_CFB模式
    mycipher = AES.new(key, AES.MODE_CFB, iv)
    # 加密的明文长度必须为16的倍数，如果长度不为16的倍数，则需要补足为16的倍数
    # 将iv（密钥向量）加到加密的密文开头，一起传输
    ciphertext = iv + mycipher.encrypt(data.encode())

    # print('密钥k为：', key)
    # print('iv为：', b2a_hex(ciphertext)[:16])
    # print('加密后数据为：', b2a_hex(ciphertext)[16:])

    try:
        strPathTemp = strPath + ".tmp"
        f = open(strPathTemp, 'wb')
        f.write(ciphertext)
        f.close()
        my_renameFile2(strPathTemp, strPath, True)
    except:
        MyLog.writeLog("{}[:{}] - my_encode_check err! Failed to write authorization file! "
                       .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno)
                       , LOG_ERROR, ENABLE_WRITE_LOG, ENABLE_PRINT)

    print("nDaysLeft = ", nDaysLeft)
    return True, bFirst, nDaysLeft


def my_authorization(strPath, strRecordPath, strActivationCode):
    # return value: 0:successful, 1:ActivationCode error, 2:The activation code has expired, 3:can't be activated, 4:time out
    if strActivationCode == "":
        MyLog.writeLog("{}[:{}] - my_authorization: ActivationCode Error! ".format(
            __file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_ERROR, ENABLE_WRITE_LOG,
                       ENABLE_PRINT)
        return 1
    try:
        # 查找所有使用过的激活码，避免重复使用
        listRecord = []
        if my_isExist(strRecordPath):
            f = open(strRecordPath, 'r')
            listRecord = f.readlines()
            f.close()
            for i in range(len(listRecord)):
                listRecord[i] = listRecord[i].strip()

        if strActivationCode in listRecord:
            MyLog.writeLog("{}[:{}] - my_authorization: The activation code has expired! "
                           .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno),
                           LOG_ERROR, ENABLE_WRITE_LOG, ENABLE_PRINT)
            return 2

        strActivationCodeTemp = a2b_hex(strActivationCode)
        # 解密的话要用key和iv生成新的AES对象
        mydecrypt = AES.new(key, AES.MODE_CFB, strActivationCodeTemp[:16])
        # 使用新生成的AES对象，将加密的密文解密
        decrypttext = mydecrypt.decrypt(strActivationCodeTemp[16:])
        strText = decrypttext.decode()

        str1 = strText[0:len(strMsgCheck1)]
        if str1 != strMsgCheck1:
            MyLog.writeLog(
                "{}[:{}] - my_authorization: str1 Error!! ".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                   sys._getframe().f_lineno, str1), LOG_ERROR,
                ENABLE_WRITE_LOG, ENABLE_PRINT)
            return 1

        pos = strText.find(strMsgCheck2)
        if pos < 0:
            MyLog.writeLog(
                "{}[:{}] - my_authorization: pos Error! ".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                 sys._getframe().f_lineno), LOG_ERROR, ENABLE_WRITE_LOG,
                ENABLE_PRINT)
            return 1

        listMac = get_all_mac_address()
        if len(listMac) == 0:
            MyLog.writeLog(
                "{}[:{}] - my_authorization: listMac empty! ".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                     sys._getframe().f_lineno), LOG_ERROR,
                ENABLE_WRITE_LOG, ENABLE_PRINT)
            return 3

        strMsgCheck3 = listMac[0]
        str3 = strText[pos + len(strMsgCheck2):pos + len(strMsgCheck2 + strMsgCheck3)]
        # if str3 != strMsgCheck3:
        if str3 not in listMac:
            MyLog.writeLog(
                "{}[:{}] - my_authorization: str3 Error! ".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                  sys._getframe().f_lineno, str3), LOG_ERROR,
                ENABLE_WRITE_LOG, ENABLE_PRINT)
            return 1
        strMsgCheck3 = str3
        # print("my_authorization-bind mac:", strMsgCheck3)

        strActivationCodeTime = strText[pos + len(strMsgCheck2 + strMsgCheck3):pos + len(
            strMsgCheck2 + strMsgCheck3) + nTimeLen]
        if not isActivationCodeValid(strActivationCodeTime):
            MyLog.writeLog(
                "{}[:{}] - my_authorization: time out! ".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                sys._getframe().f_lineno), LOG_ERROR, ENABLE_WRITE_LOG,
                ENABLE_PRINT)
            return 4

        strSerialNo = getHdSerialNo()
        if strSerialNo == "":
            MyLog.writeLog("{}[:{}] - my_authorization: number Error:{}! ".format(
                __file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, strSerialNo), LOG_ERROR,
                           ENABLE_WRITE_LOG, ENABLE_PRINT)
            return 3

        strMsgCheck4 = strSerialNo
        str4 = strText[pos + len(strMsgCheck2 + strMsgCheck3) + nTimeLen:len(strText)]
        if str4 != strMsgCheck4:
            MyLog.writeLog(
                "{}[:{}] - my_encode_check: str4 Error:{}!! ".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                     sys._getframe().f_lineno, str4), LOG_ERROR,
                ENABLE_WRITE_LOG, ENABLE_PRINT)
            return 1

        # usAuthorizedTemp: 0：未激活  1：永久激活  2：重置三个月试用期 3：重置一年试用期
        usAuthorizedTemp = int(strText[pos - 1:pos])
        if usAuthorizedTemp not in [0, 1]:
            MyLog.writeLog("{}[:{}] - my_authorization: usAuthorizedTemp Error! ".format(
                __file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_ERROR,
                           ENABLE_WRITE_LOG, ENABLE_PRINT)
            return 1

        nMinutesLeft = int(strText[len(strMsgCheck1):pos - 1])

        data = strMsgCheck1 + str(nMinutesLeft) + str(
            usAuthorizedTemp) + strMsgCheck2 + strMsgCheck3 + strMsgCheck4  # 加密的明文长度必须为16的倍数，如果长度不为16的倍数，则需要补足为16的倍数
        iv = Random.new().read(AES.block_size)  # 生成长度等于AES块大小的不可重复的密钥向量
        mycipher = AES.new(key, AES.MODE_CFB, iv)  # 使用key和iv初始化AES对象, 使用MODE_CFB模式
        ciphertext = iv + mycipher.encrypt(data.encode())  # 将iv（密钥向量）加到加密的密文开头，一起传输
        f = open(strPath, 'wb')
        f.write(ciphertext)
        f.close()

        # 将使用过的激活码记录下来，避免重复使用
        f = open(strRecordPath, 'a')
        f.write(strActivationCode + "\n")
        f.close()
    except:
        MyLog.writeLog(
            "{}[:{}] - my_authorization: except!!! ".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                            sys._getframe().f_lineno), LOG_ERROR, ENABLE_WRITE_LOG,
            ENABLE_PRINT)
        return 1

    return 0


# 判断激活码时间是否有效
def isActivationCodeValid(strActivationCodeTime):
    # docker中获取多时间比实际时间晚8小时，在实际机器中获取的时间正常
    # d1 = datetime.datetime.now() + datetime.timedelta(hours=8)
    d1 = datetime.datetime.now()
    d2 = datetime.datetime.strptime(strActivationCodeTime, '%Y%m%d%H%M%S')
    delta = abs(d1 - d2)
    if delta.days > 0:
        MyLog.writeLog("{}[:{}] - isActivationCodeValid: delta.days > 0! {} {}".format(
            __file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno,
            d1.strftime("%Y-%m-%d %H:%M:%S"), d2.strftime("%Y-%m-%d %H:%M:%S")), LOG_ERROR, ENABLE_WRITE_LOG,
                       ENABLE_PRINT)
        return False
    # if delta.seconds > 600:  # 10分钟内有效
    #     MyLog.writeLog("{}[:{}] - isActivationCodeValid: delta.seconds > 600! {} {}".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, d1.strftime("%Y-%m-%d %H:%M:%S"), d2.strftime("%Y-%m-%d %H:%M:%S")), LOG_ERROR, ENABLE_WRITE_LOG, ENABLE_PRINT)
    #     return False

    return True


# ========================= 时间校验 结束 ===========================

def cacheThread(cacheQueue, stParam, lockVid):
    MyLog.writeLog("{}[:{}] - cacheProcess start, pid={}"
                   .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, os.getpid())
                   , LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)
    bChanged = True
    strCacheFilePath = ""
    nFrameIdTemp = 0
    while stParam.getRunning():
        while not stParam.getOnline() or stParam.getPause() or (1 <= stParam.getStatus() <= 2):
            time.sleep(1)
            # print("cacheProcess sleep...")

        if bChanged:
            # 创建缓存目录
            strCacheFilePath = CachePath + time.strftime('%Y%m%d%H%M%S', time.localtime(time.time())) + "/"
            my_mkdir(strCacheFilePath)
            my_mkdir(SavePath)
            f = open(SavePath + CachePathFile, "w")
            f.write(strCacheFilePath)
            f.close()
            bChanged = False
            MyLog.writeLog(
                "{}[:{}] - create strCacheFilePath: {}".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                               sys._getframe().f_lineno, strCacheFilePath), LOG_WARNING,
                ENABLE_WRITE_LOG, ENABLE_PRINT)

        # =================================== 缓存 ===================================
        t3 = time.time()
        dstData = None
        while (dstData is None) and stParam.getRunning() and stParam.getOnline():
            try:
                dstData = cacheQueue.get(True, 0.1)  # 设置超时时间，超过指定时间，则认为异常，走异常处理流程
            except:
                continue

        if dstData is None:
            continue

        # 缓存
        saveOneCache(strCacheFilePath, dstData)
        nFrameIdTemp += 1

        # 超出缓存限定数量，则从头删除缓存文件，使缓存保持在限定范围内
        # print("cache: cur:{}, Limit:{}".format(nFrameIdTemp, stUiParam.nCacheFileCountLimit))
        if nFrameIdTemp >= stUiParam.nCacheFileCountLimit:
            strFileTemp = '{}{}{}'.format(strCacheFilePath, PcCacheFile,
                                          nFrameIdTemp - stUiParam.nCacheFileCountLimit + 1)
            try:
                # print("cache: delete {}!".format(strFileTemp))
                deleteOneCache(strFileTemp)
            except:
                print("cache: delete error!")

        # print("saveOneCache elapsed =  %fms" % ((time.time() - t3) * 1000))
        # =================================== 缓存 ===================================

    MyLog.writeLog("{}[:{}] - exit cacheProcess...".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                           sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG,
                   ENABLE_PRINT)


# QImage to numpy
def qimage2numpy(qimage, dtype='array'):
    result_shape = (qimage.height(), qimage.width())
    temp_shape = (qimage.height(), int(qimage.bytesPerLine() * 8 / qimage.depth()))
    if qimage.format() in (QImage.Format_ARGB32_Premultiplied, QImage.Format_ARGB32, QImage.Format_RGB32):
        if dtype == 'array':
            dtype = np.uint8
            result_shape += (4,)
            temp_shape += (4,)
    elif qimage.format() == QImage.Format_Indexed8:
        dtype = np.uint8
    else:
        MyLog.writeLog("{}[:{}] - qimage2numpy only supports 32bit and 8bit images".format(
            __file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_ERROR, ENABLE_WRITE_LOG,
                       ENABLE_PRINT)
        raise ValueError("qimage2numpy only supports 32bit and 8bit images")

    # FIXME: raise error if alignment does not match
    buf = qimage.bits().asstring(qimage.bytesPerLine() * qimage.height())
    result = np.frombuffer(buf, dtype).reshape(temp_shape)
    if result_shape != temp_shape:
        result = result[:, :result_shape[1]]
    if qimage.format() == QImage.Format_RGB32 and dtype == np.uint8:
        result = result[..., :3]

    return result


def getIdinList(boxid, fusionID, index=0):
    if fusionID == []:
        return False
    for ID in fusionID:
        if boxid == ID[index]:
            return True
    return False


def getPcBoxInfo_old(pcBoxToVideo, stParam=None):
    nPcBoxIdMax = 0
    listBoxInfo = []
    if pcBoxToVideo is not None:
        trackersToEvent = np.zeros((pcBoxToVideo.shape[0], 46))
        trackers_cov = np.zeros((pcBoxToVideo.shape[0], 19))
        trackers_cov[:, 0:15] = pcBoxToVideo[:, 0:15]
        trackers_cov[:, 18] = pcBoxToVideo[:, 18]
        # print("trackers_cov.shape[0]",trackers_cov.shape[0],listSource.shape)
        pcBox = pcBoxToVideo[:, 19:].reshape(-1, 8, 3)
        lLidarLongitude = stParam.getLidarLongitude(0)
        lLidarLatitude = stParam.getLidarLatitude(0)
        lAngleNorthT = stParam.getAngleNorthT(0)
        stTransform = Lon_Lat_Meter_Transform(lLidarLongitude, lLidarLatitude, lAngleNorthT)
        npLonLat = stTransform.xy_of_draw_to_lonlat(trackers_cov[:, 0:2])
        trackers_cov[:, 15:17] = npLonLat[:, :]
        for i in range(trackers_cov.shape[0]):
            stBoxInfo = structBoxInfo()
            # 获取目标框数据，用于填充表格
            stBoxInfo.boxCenter = trackers_cov[i, 0:3]
            stBoxInfo.boxSize = trackers_cov[i, 3:6]
            stBoxInfo.boxAngle = trackers_cov[i, 6]
            nType = trackers_cov[i, 7]
            stBoxInfo.boxSpeed = trackers_cov[i, 8]
            stBoxInfo.boxId = int(trackers_cov[i][9])
            # stBoxInfo.boxId = trackers_cov[i, 12]
            stBoxInfo.boxVertexPc = pcBox[i]
            stBoxInfo.nPoints = 0
            stBoxInfo.strClass = switch_pc[nType]
            stBoxInfo.strSource = "PointCloud"
            if trackers_cov[i, 11] == 3:
                stBoxInfo.strSource = "Radar"
            elif trackers_cov[i, 11] == 4:
                stBoxInfo.strSource = "PcRfusion"
            elif trackers_cov[i, 11] == 10:
                stBoxInfo.strSource = "Complement"
            elif trackers_cov[i, 11] == 11:
                stBoxInfo.strSource = "RRfusion"

            if stParam is not None:
                stBoxInfo.boxLongitude = trackers_cov[i, 15]
                stBoxInfo.boxLatitude = trackers_cov[i, 16]
            stBoxInfo.nConfidence = int(trackers_cov[i, 10] * 100)
            stBoxInfo.nBaseStationId = trackers_cov[i, 12]
            stBoxInfo.nBaseStationBoxId = trackers_cov[i, 13]
            stBoxInfo.fcolor = trackers_cov[i, 14]
            listBoxInfo.append(stBoxInfo)  # 添加到链表
            nPcBoxIdMax = max(nPcBoxIdMax, stBoxInfo.boxId)
            stBoxInfo.nBoxCamId = trackers_cov[i, 17]
            stBoxInfo.nPreFrameCnt = trackers_cov[i, 18]

        trackersToEvent[:, 0: 12] = trackers_cov[:, 0: 12]
        trackersToEvent[:, 12] = trackers_cov[:, 13]
        trackersToEvent[:, 13] = trackers_cov[:, 12]
        trackersToEvent[:, 14:18] = trackers_cov[:, 14:18]
        trackersToEvent[:, 18:44] = pcBoxToVideo[:, 17:]
        trackersToEvent[:, 44] = time.time()

        return nPcBoxIdMax, listBoxInfo, trackersToEvent

    return -1, listBoxInfo, None


def getPcBoxInfo(pcBoxToVideo, stParam=None):
    nPcBoxIdMax = 0
    listBoxInfo = []
    tTimeStamp = time.time()
    if pcBoxToVideo is not None:
        trackersToEvent = np.zeros((pcBoxToVideo.shape[0], 46))
        trackers_cov = np.zeros((pcBoxToVideo.shape[0], 22))
        trackers_cov[:, 0:20] = pcBoxToVideo[:, 0:20]
        # print("trackers_cov.shape[0]",trackers_cov.shape[0],listSource.shape)
        pcBox = pcBoxToVideo[:, 20:].reshape(-1, 8, 3)
        lLidarLongitude = stParam.getLidarLongitude(0)
        lLidarLatitude = stParam.getLidarLatitude(0)
        lAngleNorthT = stParam.getAngleNorthT(0)
        stTransform = Lon_Lat_Meter_Transform(lLidarLongitude, lLidarLatitude, lAngleNorthT)
        npLonLat = stTransform.xy_of_draw_to_lonlat(trackers_cov[:, 0:2])
        trackers_cov[:, 20:22] = npLonLat[:, :]
        for i in range(trackers_cov.shape[0]):
            stBoxInfo = structBoxInfo()
            # 获取目标框数据，用于填充表格
            stBoxInfo.boxCenter = trackers_cov[i, 0:3]
            stBoxInfo.boxSize = trackers_cov[i, 3:6]
            stBoxInfo.boxAngle = trackers_cov[i, 6]
            nType = trackers_cov[i, 7]
            stBoxInfo.boxSpeed = trackers_cov[i, 8]
            stBoxInfo.boxId = int(trackers_cov[i][9])
            # stBoxInfo.boxId = trackers_cov[i, 12]
            stBoxInfo.boxVertexPc = pcBox[i]
            stBoxInfo.nPoints = 0
            stBoxInfo.strClass = switch_pc[nType]
            stBoxInfo.strSource = "PointCloud"
            if trackers_cov[i, 11] == 3:
                stBoxInfo.strSource = "Radar"
            elif trackers_cov[i, 11] == 4:
                stBoxInfo.strSource = "PcRfusion"
            elif trackers_cov[i, 11] == 10:
                stBoxInfo.strSource = "Complement"
            elif trackers_cov[i, 11] == 11:
                stBoxInfo.strSource = "RRfusion"

            if stParam is not None:
                stBoxInfo.boxLongitude = trackers_cov[i, 20]
                stBoxInfo.boxLatitude = trackers_cov[i, 21]
            stBoxInfo.nConfidence = int(trackers_cov[i, 10] * 100)
            stBoxInfo.nBaseStationId = trackers_cov[i, 12]
            stBoxInfo.nBaseStationBoxId = trackers_cov[i, 13]
            stBoxInfo.fcolor = trackers_cov[i, 14]
            nPcBoxIdMax = max(nPcBoxIdMax, stBoxInfo.boxId)
            stBoxInfo.nBoxCamId = trackers_cov[i, 17]
            stBoxInfo.nLane = trackers_cov[i, 18]
            stBoxInfo.nPreFrameCnt = trackers_cov[i, 19]

            listBoxInfo.append(stBoxInfo)  # 添加到链表

        trackersToEvent[:, 0: 12] = trackers_cov[:, 0: 12]
        trackersToEvent[:, 12] = trackers_cov[:, 13]
        trackersToEvent[:, 13] = trackers_cov[:, 12]
        trackersToEvent[:, [14, 15, 16, 17, 18, 19]] = trackers_cov[:, [14, 20, 21, 15, 16, 17]]
        trackersToEvent[:, 20:44] = pcBoxToVideo[:, 20:]
        trackersToEvent[:, 44] = tTimeStamp
        trackersToEvent[:, 45] = trackers_cov[:, 18]

        return nPcBoxIdMax, listBoxInfo, trackersToEvent, tTimeStamp

    return -1, listBoxInfo, None


def getRadarBoxInfo(pcBoxToVideo, RadarId, npoldData=None, strSource="", stParam=None):
    nPcBoxIdMax = 0
    listBoxInfo = []
    if pcBoxToVideo is not None:
        trackers_cov = pcBoxToVideo[:, 0:7]
        radarBox = pcBoxToVideo[:, 7:].reshape(-1, 8, 3)
        lLidarLongitude = stParam.getLidarLongitude(0)
        lLidarLatitude = stParam.getLidarLatitude(0)
        lAngleNorthT = stParam.getAngleNorthT(0)

        for i in range(trackers_cov.shape[0]):
            stBoxInfo = structBoxInfo()

            # 获取目标框数据，用于填充表格
            stBoxInfo.boxCenter = trackers_cov[i, 0:3]
            stBoxInfo.boxSize = trackers_cov[i, 3:6]
            stBoxInfo.boxAngle = trackers_cov[i, 6]
            stBoxInfo.boxVertexPc = radarBox[i]
            X_speed = RadarId[i][1]
            Y_speed = RadarId[i][2]
            # print("speed :", X_speed, Y_speed)
            stBoxInfo.boxId = int(RadarId[i][0])
            # stBoxInfo.boxSpeed = np.sqrt(math.pow(X_speed, 2) + math.pow(Y_speed, 2))
            stBoxInfo.boxSpeed = X_speed
            stBoxInfo.boxSize = [0.0, RadarId[i][3], 0.0]
            stBoxInfo.nPoints = 0
            stBoxInfo.strSource = "Radar"
            if npoldData is not None:
                stBoxInfo.npoldboxCenter = npoldData[i]
                print("npoldData[i]", npoldData[i])
            if strSource != "":
                stBoxInfo.strSource = strSource
            if stParam is not None:
                stBoxInfo.boxLongitude, stBoxInfo.boxLatitude = XYZ_To_BLH(lLidarLongitude, lLidarLatitude,
                                                                           stBoxInfo.boxCenter[0],
                                                                           stBoxInfo.boxCenter[1], lAngleNorthT)
            listBoxInfo.append(stBoxInfo)  # 添加到链表
    return listBoxInfo


def isNeedRestartApp(fFps):
    # 异常重启开关
    if not stUiParam.bAbnormalRestart:
        return False

    try:
        # 内存使用率超过指定值，且输出帧率低于7FPS, 则认为软件异常，需要重启软件
        fMemPercent = psutil.virtual_memory().percent
        if fMemPercent > stUiParam.fMemPercentMax and fFps < stUiParam.fFpsMinLimit:
            MyLog.writeLog("{}[:{}] - fMemPercent={}% > {}% and fFps < {}! Need Restart App!"
                           .format(__file__.split('/')[len(__file__.split('/')) - 1],
                                   sys._getframe().f_lineno, fMemPercent, stUiParam.fMemPercentMax,
                                   stUiParam.fFpsMinLimit),
                           LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
            return True

        # 显存使用率超过指定最大值，或小于指定最小值, 且输出帧率低于7FPS, 则认为软件异常，需要重启软件
        fGpuPercent = 0
        Gpus = GPUtil.getGPUs()
        for gpu in Gpus:
            fGpuPercent = round(max(fGpuPercent, gpu.memoryUtil) * 100, 2)

        if fGpuPercent > stUiParam.fGpuPercentMax and fFps < stUiParam.fFpsMinLimit:
            MyLog.writeLog("{}[:{}] - fGpuPercent={}%, > {}% and fFps < {}! Need Restart App!"
                           .format(__file__.split('/')[len(__file__.split('/')) - 1],
                                   sys._getframe().f_lineno, fGpuPercent, stUiParam.fGpuPercentMax,
                                   stUiParam.fFpsMinLimit),
                           LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
            return True
    except:
        MyLog.writeLog("{}[:{}] - isNeedRestartApp Error!"
                       .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno),
                       LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
    return False


# # 此方式不适用与当前多个进程的程序重启
# def restart_program():
#     print("restart...")
#     python = sys.executable
#     os.execl(python, python, * sys.argv)


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


def isWatchDogRun():
    p = subprocess.Popen('pgrep -f watchdog.sh', shell=True, stdout=subprocess.PIPE, universal_newlines=True)
    out, err = p.communicate()
    listPid = out.splitlines()
    bRun = False
    for pid in listPid:
        try:
            proc = psutil.Process(pid=int(pid))
            bRun = True
        except:
            continue

    if not bRun:
        MyLog.writeLog(
            "{}[:{}] - no such pro:".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno,
                                            listPid), LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)

    return bRun


# 使用启动一个监控脚本的方式，来监控程序进程是否存活，不存活则重启该程序
def startWatchDog():
    if not stUiParam.bAbnormalRestart:
        MyLog.writeLog("{}[:{}] - Watchdog is not enabled!".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                   sys._getframe().f_lineno), LOG_INFO,
                       ENABLE_WRITE_LOG, ENABLE_PRINT)
        return

    t1 = time.time()
    if not isWatchDogRun():
        MyLog.writeLog("{}[:{}] - startWatchDog...".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                           sys._getframe().f_lineno), LOG_INFO, ENABLE_WRITE_LOG,
                       ENABLE_PRINT)
        currentId = os.getpid()
        proc = psutil.Process(pid=currentId)
        strWatchDogSh = strExePath + "/Configs/watchdog.sh"
        strAutoStartSh = strExePath + "/Configs/AutoStart.sh"
        os.system('sh {} {} {}&'.format(strWatchDogSh, strAutoStartSh, proc.name()))
        MyLog.writeLog("{}[:{}] - startWatchDog elapsed = {}"
                       .format(__file__.split('/')[len(__file__.split('/')) - 1],
                               sys._getframe().f_lineno, round((time.time() - t1) * 1000, 3)),
                       LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)


# 获取由能抓到IP过滤器指定IP的数据包的网卡创建的嗅探器
def getSnifferOfAvailableNetCard(nDstPort, listAttachedIp, nNetworkPort=None):
    sniffer = None
    ipFilter = ''
    for strIp in listAttachedIp:
        if listAttachedIp.index(strIp) != 0:
            ipFilter = ipFilter + ' or '
        ipFilter = ipFilter + 'src host {}'.format(strIp)
    devs = pcap.findalldevs()
    if nNetworkPort is not None and 0 <= nNetworkPort < len(devs):
        try:
            # 嗅探器，可以没有参数，sniffer一直在抓包并把数据源源不断存入内存
            # promisc=True开启混杂模式，则将可以接收通过该网卡的所有数据包，不再需要wireshark
            # timeout_ms=100设置读取数据包的超时时间100ms，超时则退出readpkts函数，避免阻塞导致readPcOnline无法退出的问题
            # immediate=False禁用立即模式，则启用数据包缓存，该模式开启关闭都没有影响
            sniffer = pcap.pcap(name=devs[nNetworkPort], promisc=True, timeout_ms=100, immediate=True)
            # sniffer.setfilter('src host {} and dst host {} and src port {} and dst port {} and udp'.format(strStationSrcIp, strLidarDstIp, 3333, 3001))
            # sniffer.setfilter('dst port {}'.format(nDstPort))  # 设置嗅探器的过滤器，可提速
            # sniffer.setfilter('src host {}'.format('192.168.3.182'))  # 设置嗅探器的过滤器，可提速
            sniffer.setfilter(ipFilter)  # 设置嗅探器的过滤器，可提速
            listTest = sniffer.readpkts()
            if len(listTest) > 0:
                MyLog.writeLog("{}[:{}] - Find available network card:{}".format(
                    __file__.split('/')[len(__file__.split('/')) - 1],
                    sys._getframe().f_lineno, devs[nNetworkPort]), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                return sniffer, nNetworkPort
        except:
            MyLog.writeLog("{}[:{}] - The initial network card is not available!"
                           .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno),
                           LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)

    for nId in range(len(devs)):
        if nId == nNetworkPort:
            continue

        try:
            sniffer = pcap.pcap(name=devs[nId], promisc=True, timeout_ms=100, immediate=True)
            # sniffer.setfilter('src host {} and dst host {} and udp'.format(strSrcIpFilter, strDstIpFilter))  # 设置嗅探器的过滤器，可提速
            # sniffer.setfilter('src host {}'.format('192.168.3.182'))  # 设置嗅探器的过滤器，可提速
            # sniffer.setfilter('dst port {}'.format(nDstPort))  # 设置嗅探器的过滤器，可提速
            sniffer.setfilter(ipFilter)  # 设置嗅探器的过滤器，可提速
            listTest = sniffer.readpkts()
            if len(listTest) > 0:
                MyLog.writeLog("{}[:{}] - Find available network card:{}".format(
                    __file__.split('/')[len(__file__.split('/')) - 1],
                    sys._getframe().f_lineno, devs[nId]), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                break
            else:
                sniffer = None
        except:
            continue

    return sniffer, nId


# ===================== 功能函数 =====================
def getBccCheckVal(recvData):
    checkVal = 0x00
    for i in range(len(recvData)):
        checkVal ^= recvData[i]
    return checkVal


# def parse_GPS_time():#transfer para:pl_data
#     ###解析一包数据的GPS时间
#     #data = pl_data
#     # syq add
#     dataTimeNow = datetime.datetime.now()
#     # print('now',dataTimeNow)
#     # print('mocro',dataTimeNow.microsecond)
#     # dataTimePackage = dataTimeNow
#     strDataTimePackage = dataTimeNow.astimezone(timezone(timedelta(hours=-0))).strftime("%Y-%m-%d %H:%M:%S")
#     # print("strDataTimePackage",strDataTimePackage)
#     # 转换成时间数组
#     timeArray = time.strptime(strDataTimePackage, "%Y-%m-%d %H:%M:%S")
#     # 转换成时间戳
#     unix_t = time.mktime(timeArray)
#     unix_t = str(unix_t).split('.')[0]
#     unix_t = int(unix_t)
#     # print(unix_t,dataTimeNow.microsecond/1000000)
#     return unix_t,dataTimeNow.microsecond
#     # print("(unix_t + 28800) - (unix_t + 28800) % 60",(unix_t + 28800) - (unix_t + 28800) % 60)
#     # return (unix_t + 28800) - (unix_t + 28800) % 60,0

def parse_GPS_time():
    timeStamp = time.time()
    timeStampS, timeStampMS = math.floor(timeStamp), int((timeStamp - math.floor(timeStamp)) * 1e3)
    return timeStampS, timeStampMS


def getJsonBoxInfo(timeStamp, listBoxInfo, listBoxInLanReal):
    strTerminalId = stUiParam.strTerminalId
    timeStampTemp = int(timeStamp * 1000)
    dictMsg = {
        "time": timeStampTemp,
        "id": strTerminalId,
    }

    participants = []
    for boxInfo in listBoxInfo:
        for obj in listBoxInLanReal:
            if boxInfo.boxId in obj[1]:
                boxInfo.nLane = obj[0]
                break

        dictBoxInfo = {}
        dictBoxInfo["id"] = boxInfo.boxId
        dictPosition = {}
        dictPosition["longitude"] = round(boxInfo.boxLongitude, 7)
        dictPosition["latitude"] = round(boxInfo.boxLatitude, 7)
        # dictPosition["centralX"] = int(boxInfo.boxCenter[0]*100)
        # dictPosition["centralY"] = int(boxInfo.boxCenter[1]*100)
        # dictPosition["centralZ"] = int(boxInfo.boxCenter[2]*100)
        dictBoxInfo["location"] = dictPosition
        dictBoxInfo["type"] = getSendJsonEnumFromStr(boxInfo.strClass)
        dictBoxInfo["speed"] = round(boxInfo.boxSpeed, 2)
        pitch = (180 + stLidarParamTemp.fAngleNorthT + boxInfo.boxAngle / PI_rads) % 360
        dictBoxInfo["orientation"] = round(pitch, 2)
        dictSize = {}
        dictSize["length"] = int(boxInfo.boxSize[1] * 100)
        dictSize["width"] = int(boxInfo.boxSize[0] * 100)
        dictSize["height"] = int(boxInfo.boxSize[2] * 100 / 5)
        dictBoxInfo["size"] = dictSize
        dictBoxInfo["lane"] = boxInfo.nLane
        participants.append(dictBoxInfo)

    dictMsg["participants"] = participants
    jsonMsg = json.dumps(dictMsg).encode()
    return jsonMsg


def getJsonBoxInfo_Pics(listBoxInfo, angle_north_T):
    participants = []
    for boxInfo in listBoxInfo:
        dictBoxInfo = {}
        dictBoxInfo["id"] = int(boxInfo.boxId)
        type_tmp = getSendEnumFromStr(boxInfo.strClass)
        if type_tmp == -1:
            type_tmp = 0

        dictBoxInfo["type"] = type_tmp

        dictBoxInfo["confidence"] = int(boxInfo.nConfidence)
        if boxInfo.fcolor < 0:
            dictBoxInfo["color"] = 0
        else:
            dictBoxInfo["color"] = int(round(boxInfo.fcolor))

        if boxInfo.strSource == "PointCloud":
            strSource = 0
        elif boxInfo.strSource == "camera":
            strSource = 1
        elif boxInfo.strSource == "fusion":
            strSource = 2
        else:
            strSource = 0

        dictBoxInfo["source"] = strSource

        detecSource_tmp = int(boxInfo.nPreFrameCnt)
        if detecSource_tmp != 0:
            detecSource_tmp = 1
        dictBoxInfo["detecSource"] = detecSource_tmp

        dictBoxInfo["laneNum"] = int(boxInfo.nLane)

        if 0 < boxInfo.nBaseStationId < 256:
            dictBoxInfo["baseStationSource"] = int(boxInfo.nBaseStationId)
        else:
            dictBoxInfo["baseStationSource"] = 0

        dictBoxInfo["longitude"] = round(boxInfo.boxLongitude, 7)
        dictBoxInfo["latitude"] = round(boxInfo.boxLatitude, 7)
        dictBoxInfo["altitude"] = 0
        dictBoxInfo["speed"] = int(round(boxInfo.boxSpeed * 100))

        pitch = (180.0 + angle_north_T + boxInfo.boxAngle / PI_rads) % 360

        dictBoxInfo["courseAngle"] = int(round(pitch * 10, 2))
        dictBoxInfo["length"] = int(round(boxInfo.boxSize[1] * 100))
        dictBoxInfo["width"] = int(round(boxInfo.boxSize[0] * 100))
        dictBoxInfo["height"] = int(round(boxInfo.boxSize[2] * 100))

        dictBoxInfo["XCoordinate"] = int(boxInfo.boxCenter[0] * 100)
        dictBoxInfo["YCoordinate"] = int(boxInfo.boxCenter[1] * 100)
        dictBoxInfo["ZCoordinate"] = int(boxInfo.boxCenter[2] * 100)
        dictBoxInfo["sourceId"] = int(boxInfo.nBaseStationBoxId)

        participants.append(dictBoxInfo)

    return participants


def sendHttpMessage(stParam, stDstData):
    BaseStationInfoList = []
    try:
        if stParam.getOnline():
            for i in stDstData.nSingleFrame.keys():
                listtmp = {}
                listtmp["baseStationSource"] = stDstData.nSingleEqupId["{}".format(i)]
                listtmp["state"] = stDstData.stationState["{}".format(i)]
                listtmp["sourceframeNo"] = stDstData.nSingleFrame["{}".format(i)]
                listtmp["sourcetimeStamp"] = getTimeStamp(stDstData.nSingleStamp["{}".format(i)])
                listtmp["sourceRecievetimeStamp"] = getTimeStamp(stDstData.recvTime["{}".format(i)])  # srcdatalist[0]
                listtmp["sourcelongitude"] = round(stParam.getLidarLongitude(int(i) - 1), 7)
                listtmp["sourcelatitude"] = round(stParam.getLidarLatitude(int(i) - 1), 7)
                listtmp["sourceangle"] = int(stParam.getAngleNorthT(int(i) - 1))
                BaseStationInfoList.append(listtmp)
        else:
            for i in range(len(stParam.getlistAttachedIp())):
                listtmp = {}
                listtmp["soueceId"] = stParam.getBaseStationID()
                listtmp["sourceframeNo"] = stParam.getFrameId()
                listtmp["sourcetimeStamp"] = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S:%f')[:-3]
                listtmp["sourcelongitude"] = round(stParam.getLidarLongitude(i), 7)
                listtmp["sourcelatitude"] = round(stParam.getLidarLatitude(i), 7)
                listtmp["sourceangle"] = int(stParam.getAngleNorthT(i))
                BaseStationInfoList.append(listtmp)
    except:
        print(traceback.format_exc())
        MyLog.writeLog("{}[:{}] BaseStationInfoList failed!".
                       format(__file__.split('/')[len(__file__.split('/')) - 1],
                              sys._getframe().f_lineno),
                       LOG_ERROR, ENABLE_WRITE_LOG, ENABLE_PRINT)

    try:
        dictMsg = {}
        listAllBaseBoxInfo = stDstData.listBaseBoxInfo
        angle_north_T = stParam.getAngleNorthT(0)
        participants = getJsonBoxInfo_Pics(listAllBaseBoxInfo, angle_north_T)

        dictMsg["deviceId"] = stDstData.nEqupId[1]
        dictMsg["algorithmDuration"] = stDstData.trunAlg
        dictMsg["globalFrameNo"] = stDstData.nFrameId
        dictMsg["globalTimeStamp"] = getTimeStamp(stDstData.tTimeStamp)
        dictMsg["frameNo"] = stDstData.n0Frame  # 入口点云数据帧号   0号基站帧号
        dictMsg["timeStamp"] = getTimeStamp(stDstData.n0TimeStamp)  # 入口识别结果时间戳  0号基站时间戳
        dictMsg["longitude"] = round(stParam.getLidarLongitude(0), 7)
        dictMsg["latitude"] = round(stParam.getLidarLatitude(0), 7)
        dictMsg["angle"] = int(angle_north_T)
        dictMsg["participantNum"] = len(listAllBaseBoxInfo)
        dictMsg["e1FrameParticipant"] = participants
        dictMsg["stationNum"] = stDstData.nSingleFrame.__len__()
        dictMsg["sourceInfoList"] = BaseStationInfoList
        return dictMsg

    except:
        print(traceback.format_exc())
        MyLog.writeLog("{}[:{}] dictMsg failed!".
                       format(__file__.split('/')[len(__file__.split('/')) - 1],
                              sys._getframe().f_lineno),
                       LOG_ERROR, ENABLE_WRITE_LOG, ENABLE_PRINT)


def parsingHeadData(recvData):
    data = struct.unpack(">H4BH", recvData[0:8])
    return data


def parsingData(recvData):
    data = struct.unpack(">H4BH4B2H4B", recvData)
    checkVal = getBccCheckVal(recvData[:len(recvData) - 2])
    if checkVal != data[14]:
        return None

    return data


def parseBaseStationData(recvData):
    time_Stamp2 = time.time()
    len_recvdata = len(recvData)
    unpack_information = struct.unpack(">6BH", recvData[0:8])

    len_info = len(unpack_information)
    len2 = struct.calcsize(">6BH")
    Equiq_id = struct.unpack(">2B", recvData[8:10])
    data_reserve1 = struct.unpack(">H", recvData[10:12])
    frame_id = struct.unpack("4B", recvData[12:16])

    parse_data = []
    time_Stamp_ = struct.unpack(">2I", recvData[16:24])

    time_Stamp = time_Stamp_[0] + time_Stamp_[1] / 1000000.0

    # print(time_Stamp,"--------",time_Stamp2)
    logti = struct.unpack(">I", recvData[24:28])
    lat = struct.unpack(">I", recvData[28:32])
    angle_north = struct.unpack(">H", recvData[32:34])

    trans_num = struct.unpack("B", recvData[34:35])
    trans = struct.unpack("5B", recvData[35:40])
    # m, n = trackers_cov.shape
    sigle_object = ()
    try:
        if trans_num[0] > 0:
            for i in range(trans_num[0]):
                struct_box_info = structBoxInfo()
                struct_box_info.nEqupId = Equiq_id
                struct_box_info.frameId = frame_id
                object_id = struct.unpack('>H', recvData[40 + i * 36:42 + i * 36])
                object_type = struct.unpack('B', recvData[42 + i * 36:43 + i * 36])
                object_confidence = struct.unpack('B', recvData[43 + i * 36:44 + i * 36])
                object_color = struct.unpack('B', recvData[44 + i * 36:45 + i * 36])
                object_source = struct.unpack('B', recvData[45 + i * 36:46 + i * 36])
                object_symbol = struct.unpack('B', recvData[46 + i * 36:47 + i * 36])
                object_reserve1 = struct.unpack('B', recvData[47 + i * 36:48 + i * 36])
                object_log = struct.unpack('>i', recvData[48 + i * 36:52 + i * 36])
                object_lat = struct.unpack('>i', recvData[52 + i * 36:56 + i * 36])
                if 8192 < 58 + i * 36:
                    print("56+i*36:58+i*36", 58 + i * 36)
                object_alti = struct.unpack('>h', recvData[56 + i * 36:58 + i * 36])
                object_speed = struct.unpack('>H', recvData[58 + i * 36:60 + i * 36])
                object_pitch = struct.unpack('>H', recvData[60 + i * 36:62 + i * 36])
                object_length = struct.unpack('>H', recvData[62 + i * 36:64 + i * 36])
                object_wide = struct.unpack('>H', recvData[64 + i * 36:66 + i * 36])
                object_height = struct.unpack('>H', recvData[66 + i * 36:68 + i * 36])
                object_center_x = struct.unpack('>H', recvData[68 + i * 36:70 + i * 36])
                object_center_y = struct.unpack('>H', recvData[70 + i * 36:72 + i * 36])
                object_center_z = struct.unpack('>h', recvData[72 + i * 36:74 + i * 36])
                object_hits = struct.unpack('>H', recvData[74 + i * 36:76 + i * 36])
                sigle_object = sigle_object + object_id + object_type + object_confidence + object_color + object_source + \
                               object_reserve1 + object_log + object_lat + object_alti + object_speed + object_pitch + \
                               object_length + object_wide + object_height + object_center_x + object_center_y + \
                               object_center_z + object_hits
                struct_box_info.boxId = object_id[0]
                struct_box_info.boxSize.append(object_wide[0])
                struct_box_info.boxSize.append(object_length[0])
                struct_box_info.boxSize.append(object_height[0])
                if object_symbol[0] == 1:
                    struct_box_info.boxCenter.append(object_center_x[0])
                    struct_box_info.boxCenter.append(-object_center_y[0])
                elif object_symbol[0] == 2:
                    struct_box_info.boxCenter.append(-object_center_x[0])
                    struct_box_info.boxCenter.append(object_center_y[0])
                elif object_symbol[0] == 3:
                    struct_box_info.boxCenter.append(object_center_x[0])
                    struct_box_info.boxCenter.append(object_center_y[0])
                else:
                    struct_box_info.boxCenter.append(-object_center_x[0])
                    struct_box_info.boxCenter.append(-object_center_y[0])
                struct_box_info.boxCenter.append(object_center_z[0])
                struct_box_info.boxSpeed = object_speed[0]
                struct_box_info.boxAngle = object_pitch[0]
                struct_box_info.boxLongitude = object_log[0] / 1e7
                struct_box_info.boxLatitude = object_lat[0] / 1e7
                struct_box_info.strClass = switch_send[object_type[0]]
                struct_box_info.strSource = object_source[0]
                # print("struct_box_info.strSource",struct_box_info.strSource)
                # struct_box_info.strColor = switch_color[object_color[0]]
                struct_box_info.fcolor = object_color[0]
                struct_box_info.nConfidence = object_confidence[0]
                struct_box_info.nHits = object_hits[0000]

                parse_data.append(struct_box_info)
                # struct_box_info.clear()

        if (trans_num[0] == 0):
            object_reserve3 = struct.unpack('>I', recvData[40:44])
            # object_reserve4 = struct.unpack('BB', recvData[48:50])
            ucCheck = struct.unpack('B', recvData[44:45])
            ucCutoffBit = struct.unpack('B', recvData[45:46])
        else:
            object_reserve3 = struct.unpack('>I', recvData[76 + (trans_num[0] - 1) * 36:80 + (trans_num[0] - 1) * 36])
            # object_reserve4 = struct.unpack('BB', recvData[84 + (trans_num - 1) * 36:86 + (trans_num - 1) * 36])
            ucCheck = struct.unpack('B', recvData[80 + (trans_num[0] - 1) * 36:81 + (trans_num[0] - 1) * 36])
            ucCutoffBit = struct.unpack('B', recvData[81 + (trans_num[0] - 1) * 36:82 + (trans_num[0] - 1) * 36])

        # parse_data = unpack_information + unpack_head_pack + trans_num + trans + sigle_object + object_reserve3 + ucCheck + ucCutoffBit

        # checkVal = getBccCheckVal(recvData[:len(recvData) - 2])
        # if checkVal != parse_data[len(parse_data) - 2]:
        #     return None
        time_Stamp3 = time.time()
        # print("parse time:************************* ", time_Stamp3 - time_Stamp2)
        return parse_data, [time_Stamp, Equiq_id, frame_id]
    except:
        print("trans_num:", trans_num[0], "    recvData:", len(recvData), "   except: ", traceback.format_exc())


def parseResultData(recvData):
    len_recvdata = len(recvData)
    unpack_information = struct.unpack(">6BH", recvData[0:8])
    len_info = len(unpack_information)
    len2 = struct.calcsize(">6BH")
    Equiq_id = struct.unpack(">H", recvData[8:10])
    alg_time = struct.unpack(">H", recvData[10:12])
    frame_id = struct.unpack(">H", recvData[14:16])
    time_Stamp_ = struct.unpack(">2I", recvData[16:24])
    time_Stamp = time_Stamp_[0] + time_Stamp_[1] / 1000000.0
    logti = struct.unpack(">I", recvData[24:28])
    lat = struct.unpack(">I", recvData[28:32])
    angle_north = struct.unpack(">H", recvData[32:34])
    trans_num = struct.unpack(">H", recvData[34:36])
    trans = struct.unpack("4B", recvData[36:40])
    sigle_object = ()
    if trans_num[0] > 0:
        npBoxInfo = np.zeros((trans_num[0], 35))
        for i in range(trans_num[0]):
            object_id = struct.unpack('>H', recvData[40 + i * 36:42 + i * 36])
            object_type = struct.unpack('B', recvData[42 + i * 36:43 + i * 36])
            object_confidence = struct.unpack('B', recvData[43 + i * 36:44 + i * 36])
            object_color = struct.unpack('B', recvData[44 + i * 36:45 + i * 36])
            object_source = struct.unpack('B', recvData[45 + i * 36:46 + i * 36])
            object_laneid = struct.unpack('B', recvData[46 + i * 36:47 + i * 36])
            object_basestationId = struct.unpack('B', recvData[47 + i * 36:48 + i * 36])
            object_log = struct.unpack('>i', recvData[48 + i * 36:52 + i * 36])
            object_lat = struct.unpack('>i', recvData[52 + i * 36:56 + i * 36])
            object_alti = struct.unpack('>h', recvData[56 + i * 36:58 + i * 36])
            object_speed = struct.unpack('>H', recvData[58 + i * 36:60 + i * 36])
            object_pitch = struct.unpack('>H', recvData[60 + i * 36:62 + i * 36])
            object_length = struct.unpack('>H', recvData[62 + i * 36:64 + i * 36])
            object_wide = struct.unpack('>H', recvData[64 + i * 36:66 + i * 36])
            object_height = struct.unpack('>H', recvData[66 + i * 36:68 + i * 36])
            object_center_x = struct.unpack('>H', recvData[68 + i * 36:70 + i * 36])
            object_center_y = struct.unpack('>H', recvData[70 + i * 36:72 + i * 36])
            object_center_z = struct.unpack('>h', recvData[72 + i * 36:74 + i * 36])
            object_orgId = struct.unpack('>H', recvData[74 + i * 36:76 + i * 36])
            sigle_object = sigle_object + object_id + object_type + object_confidence + object_color + object_source + \
                           object_laneid + object_basestationId + object_log + object_lat + object_alti + object_speed + object_pitch + \
                           object_length + object_wide + object_height + object_center_x + object_center_y + \
                           object_center_z + object_orgId
            '''
            deviceID	frameId	timeStamp	lidarLongitude	lidarLatitude	lidarAngle	participantNum
            participants.ID	            participants.type	            participants.confidence	        participants.vehicleColor
            participants.source	        participants.globalLaneNum	    participants.longitude	        participants.latitude
            participants.altitude	    participants.speed	            participants.courseAngle	    participants.length
            participants.width	        participants.height	            participants.xCoordinate	    participants.yCoordinate
            participants.zCoordinate    participants.frameId	        participants.baseStationSource	participants.picLicense
            participants.laneNum	    participants.picId	            participants.stationPicId	    participants.pointsDataId
            participants.licenseColor   participants.isMatchStateFirst	participants.plateFlag	        participants.stationFlag
            '''
            npBoxInfo[i, :] = np.array(
                [Equiq_id[0], frame_id[0], time_Stamp, logti[0], lat[0], angle_north[0], trans_num[0],
                 object_id[0], getPcEnumFromStr(switch_send[object_type[0]]), object_confidence[0] / 100,
                 object_color[0],
                 object_source[0], object_laneid[0], object_log[0] / 1e7, object_lat[0] / 1e7,
                 object_alti[0] / 100, object_speed[0], object_pitch[0], object_length[0],
                 object_wide[0], object_height[0], object_center_x[0] / 100, object_center_y[0] / 100,
                 object_center_z[0] / 100, frame_id[0], object_basestationId[0], 0,
                 object_laneid[0], 0, 0, 0,
                 0, 0, 0, 0])

        return npBoxInfo, [time_Stamp, Equiq_id[0], frame_id[0]]
    else:
        return None, []


def parseResultData_new(recvData):
    len_recvdata = len(recvData)
    unpack_information = struct.unpack(">6BH", recvData[0:8])
    len_info = len(unpack_information)
    len2 = struct.calcsize(">6BH")
    Equiq_id = struct.unpack(">H", recvData[8:10])
    alg_time = struct.unpack(">H", recvData[10:12])
    frame_id = struct.unpack(">H", recvData[14:16])
    time_Stamp_ = struct.unpack(">2I", recvData[16:24])
    time_Stamp = time_Stamp_[0] + time_Stamp_[1] / 1000000.0
    logti = struct.unpack(">I", recvData[24:28])
    lat = struct.unpack(">I", recvData[28:32])
    angle_north = struct.unpack(">H", recvData[32:34])
    trans_num = struct.unpack(">H", recvData[34:36])
    trans = struct.unpack("4B", recvData[38:42])
    sigle_object = ()
    if trans_num[0] > 0:
        npBoxInfo = np.zeros((trans_num[0], 35))
        for i in range(trans_num[0]):
            object_id = struct.unpack('>H', recvData[40 + i * 38:42 + i * 38])
            object_type = struct.unpack('B', recvData[42 + i * 38:43 + i * 38])
            object_confidence = struct.unpack('B', recvData[43 + i * 38:44 + i * 38])
            object_color = struct.unpack('B', recvData[44 + i * 38:45 + i * 38])
            object_source = struct.unpack('B', recvData[45 + i * 38:46 + i * 38])
            object_laneid = struct.unpack('B', recvData[46 + i * 38:47 + i * 38])
            object_basestationId = struct.unpack('B', recvData[47 + i * 38:48 + i * 38])
            object_log = struct.unpack('>i', recvData[48 + i * 38:52 + i * 38])
            object_lat = struct.unpack('>i', recvData[52 + i * 38:56 + i * 38])
            object_alti = struct.unpack('>h', recvData[56 + i * 38:58 + i * 38])
            object_speed = struct.unpack('>H', recvData[58 + i * 38:60 + i * 38])
            object_pitch = struct.unpack('>H', recvData[60 + i * 38:62 + i * 38])
            object_length = struct.unpack('>H', recvData[62 + i * 38:64 + i * 38])
            object_wide = struct.unpack('>H', recvData[64 + i * 38:66 + i * 38])
            object_height = struct.unpack('>H', recvData[66 + i * 38:68 + i * 38])
            object_center_x = struct.unpack('>H', recvData[68 + i * 38:70 + i * 38])
            object_center_y = struct.unpack('>H', recvData[70 + i * 38:72 + i * 38])
            object_center_z = struct.unpack('>h', recvData[72 + i * 38:74 + i * 38])
            object_orgId = struct.unpack('>H', recvData[74 + i * 38:76 + i * 38])
            object_singleFrame = struct.unpack('>H', recvData[76 + i * 38:78 + i * 38])
            sigle_object = sigle_object + object_id + object_type + object_confidence + object_color + object_source + \
                           object_laneid + object_basestationId + object_log + object_lat + object_alti + object_speed + object_pitch + \
                           object_length + object_wide + object_height + object_center_x + object_center_y + \
                           object_center_z + object_orgId + object_singleFrame
            '''
            deviceID	frameId	timeStamp	lidarLongitude	lidarLatitude	lidarAngle	participantNum
            participants.ID	            participants.type	            participants.confidence	        participants.vehicleColor
            participants.source	        participants.globalLaneNum	    participants.longitude	        participants.latitude
            participants.altitude	    participants.speed	            participants.courseAngle	    participants.length
            participants.width	        participants.height	            participants.xCoordinate	    participants.yCoordinate
            participants.zCoordinate    participants.frameId	        participants.baseStationSource	participants.picLicense
            participants.laneNum	    participants.picId	            participants.stationPicId	    participants.pointsDataId
            participants.licenseColor   participants.isMatchStateFirst	participants.plateFlag	        participants.stationFlag
            '''
            npBoxInfo[i, :] = np.array(
                [Equiq_id[0], frame_id[0], time_Stamp, logti[0], lat[0], angle_north[0], trans_num[0],
                 object_id[0], getPcEnumFromStr(switch_send[object_type[0]]), object_confidence[0] / 100,
                 object_color[0],
                 object_source[0], object_laneid[0], object_log[0] / 1e7, object_lat[0] / 1e7,
                 object_alti[0] / 100, object_speed[0], object_pitch[0], object_length[0],
                 object_wide[0], object_height[0], object_center_x[0] / 100, object_center_y[0] / 100,
                 object_center_z[0] / 100, frame_id[0], object_basestationId[0], object_singleFrame[0],
                 object_laneid[0], object_orgId[0], 0, 0,
                 0, 0, 0, 0])

        return npBoxInfo, [time_Stamp, Equiq_id[0], frame_id[0]]
    else:
        return None, []


def parseBaseStationData_np(recvData):
    time_Stamp2 = time.time()
    len_recvdata = len(recvData)
    unpack_information = struct.unpack(">6BH", recvData[0:8])
    len_info = len(unpack_information)
    len2 = struct.calcsize(">6BH")
    Equiq_id = struct.unpack(">2B", recvData[8:10])
    # Equiq_id = struct.unpack(">H", recvData[8:10])
    alg_time = struct.unpack(">H", recvData[10:12])
    frame_id = struct.unpack("4B", recvData[12:16])
    # frame_id = struct.unpack(">H", recvData[14:16])
    time_Stamp_ = struct.unpack(">2I", recvData[16:24])
    time_Stamp = time_Stamp_[0] - 2208988800 + time_Stamp_[1] / 1000000.0
    logti = struct.unpack(">I", recvData[24:28])
    lat = struct.unpack(">I", recvData[28:32])
    angle_north = struct.unpack(">H", recvData[32:34])

    trans_num = struct.unpack("B", recvData[34:35])
    trans = struct.unpack("5B", recvData[35:40])
    # m, n = trackers_cov.shape
    sigle_object = ()
    npBoxInfo = None
    try:
        if trans_num[0] > 0:
            npBoxInfo = np.zeros((trans_num[0], 21))
            for i in range(trans_num[0]):
                object_id = struct.unpack('>H', recvData[40 + i * 36:42 + i * 36])
                object_type = struct.unpack('B', recvData[42 + i * 36:43 + i * 36])
                object_confidence = struct.unpack('B', recvData[43 + i * 36:44 + i * 36])
                object_color = struct.unpack('B', recvData[44 + i * 36:45 + i * 36])
                object_source = struct.unpack('B', recvData[45 + i * 36:46 + i * 36])
                object_symbol = struct.unpack('B', recvData[46 + i * 36:47 + i * 36])
                object_nBoxCamId = struct.unpack('B', recvData[47 + i * 36:48 + i * 36])
                object_log = struct.unpack('>i', recvData[48 + i * 36:52 + i * 36])
                object_lat = struct.unpack('>i', recvData[52 + i * 36:56 + i * 36])
                object_alti = struct.unpack('>h', recvData[56 + i * 36:58 + i * 36])
                object_speed = struct.unpack('>H', recvData[58 + i * 36:60 + i * 36])
                object_pitch = struct.unpack('>H', recvData[60 + i * 36:62 + i * 36])
                object_length = struct.unpack('>H', recvData[62 + i * 36:64 + i * 36])
                object_wide = struct.unpack('>H', recvData[64 + i * 36:66 + i * 36])
                object_height = struct.unpack('>H', recvData[66 + i * 36:68 + i * 36])
                object_center_x = struct.unpack('>H', recvData[68 + i * 36:70 + i * 36])
                object_center_y = struct.unpack('>H', recvData[70 + i * 36:72 + i * 36])
                object_center_z = struct.unpack('>h', recvData[72 + i * 36:74 + i * 36])
                object_hits = struct.unpack('>H', recvData[74 + i * 36:76 + i * 36])
                sigle_object = sigle_object + object_id + object_type + object_confidence + object_color + object_source + \
                               object_nBoxCamId + object_log + object_lat + object_alti + object_speed + object_pitch + \
                               object_length + object_wide + object_height + object_center_x + object_center_y + \
                               object_center_z + object_hits
                listCenter = []
                listBoxSize = []
                if object_symbol[0] == 1:
                    listCenter.append(object_center_x[0])
                    listCenter.append(-object_center_y[0])
                elif object_symbol[0] == 2:
                    listCenter.append(-object_center_x[0])
                    listCenter.append(object_center_y[0])
                elif object_symbol[0] == 3:
                    listCenter.append(object_center_x[0])
                    listCenter.append(object_center_y[0])
                else:
                    listCenter.append(-object_center_x[0])
                    listCenter.append(-object_center_y[0])
                listCenter.append(object_center_z[0])
                listCenter = [xyz / 100 for xyz in listCenter]
                listBoxSize.append(object_wide[0])
                listBoxSize.append(object_length[0])
                listBoxSize.append(object_height[0])
                listBoxSize = [wlh / 100 for wlh in listBoxSize]
                npBoxInfo[i, :] = np.array([listCenter[0], listCenter[1], listCenter[2],
                                            listBoxSize[0], listBoxSize[1], listBoxSize[2],
                                            object_pitch[0], getPcEnumFromStr(switch_send[object_type[0]]),
                                            object_confidence[0] / 100,
                                            object_source[0], object_id[0], object_speed[0],
                                            0, object_id[0], object_color[0], listCenter[0],
                                            listCenter[1], object_hits[0], object_nBoxCamId[0],
                                            object_log[0] / 1e7, object_lat[0] / 1e7])
        if (trans_num[0] == 0):
            object_reserve3 = struct.unpack('>I', recvData[40:44])
            # object_reserve4 = struct.unpack('BB', recvData[48:50])
            ucCheck = struct.unpack('B', recvData[44:45])
            ucCutoffBit = struct.unpack('B', recvData[45:46])
        else:
            object_reserve3 = struct.unpack('>I', recvData[76 + (trans_num[0] - 1) * 36:80 + (trans_num[0] - 1) * 36])
            # object_reserve4 = struct.unpack('BB', recvData[84 + (trans_num - 1) * 36:86 + (trans_num - 1) * 36])
            ucCheck = struct.unpack('B', recvData[80 + (trans_num[0] - 1) * 36:81 + (trans_num[0] - 1) * 36])
            ucCutoffBit = struct.unpack('B', recvData[81 + (trans_num[0] - 1) * 36:82 + (trans_num[0] - 1) * 36])

        time_Stamp3 = time.time()
        return npBoxInfo, [time_Stamp, Equiq_id, frame_id]
    except:
        print("trans_num:", trans_num[0], "recvData:", len(recvData), "   except: ", traceback.format_exc())
        return None, []


def load_vbo(pc_data, clr_data):
    if pc_data.size == 0:
        return
    try:
        vtx_Point = glvbo.VBO(pc_data[:, :3].astype(np.float32))  # 使用 顶点_强度_颜色 数组的前三列，创建顶点数据VBO
    except:
        print("-------shape:", pc_data.shape, "   except: ", traceback.format_exc())
    vtx_Color = glvbo.VBO(clr_data.astype(np.float32) / 255.0)  # 使用 顶点_强度_颜色 数组的后三列，创建颜色数据VBO
    vtx_count = len(pc_data)  # 记录一帧数据的顶点个数
    return vtx_Point, vtx_Color, vtx_count


def getVectorGraphVBO(stLidarParam):
    if my_isExist('./Configs/Lane/pavement-1.json'):
        with open(r'./Configs/Lane/pavement-1.json', 'r', encoding='utf8')as fp:
            json_data = json.load(fp)
            features = json_data['features']
            listVBO = []
            fLongitudeT = stLidarParam.getLidarLongitude(0)
            fLatitudeT = stLidarParam.getLidarLatitude(0)
            fAngleNorthT = stLidarParam.getAngleNorthT(0)
            stTransform = Lon_Lat_Meter_Transform(fLongitudeT, fLatitudeT, fAngleNorthT)
            for dic in features:
                xyz = []
                for j in range(len(dic["geometry"]["coordinates"][0])):
                    oldx, lody = dic["geometry"]["coordinates"][0][j][0], dic["geometry"]["coordinates"][0][j][1]
                    xyz.append([oldx, lody, -4.2])
                    # if j == 0 or j == len(dic["geometry"]["coordinates"][0]) - 1:
                    #     xyz.append([oldx,lody,-4.2])
                BKG_data = np.array(xyz)
                BKG_data[:, 0:2] = stTransform.lonlat_to_xy_of_draw(BKG_data[:, 0:2])
                clrs = np.ones((BKG_data.shape[0], 3)) * 150
                listVBO.append(load_vbo(BKG_data, clrs))
        return listVBO
    else:
        return []


def num2str0(num, cnt=2):
    if cnt == 3:
        if num < 10:
            return '00' + str(num)
        elif num < 100:
            return '0' + str(num)
        else:
            return str(num)
    else:
        if num < 10:
            return '0' + str(num)
        else:
            return str(num)


def str2num(strinput):
    if strinput == 'F':
        return 15
    elif strinput == 'E':
        return 14
    elif strinput == 'D':
        return 13
    elif strinput == 'C':
        return 12
    elif strinput == 'B':
        return 11
    elif strinput == 'A':
        return 10
    else:
        return int(strinput)


def getTimeStamp(timeStamp):
    time_stramp_temp1, time_stramp_temp2 = math.floor(timeStamp), int((timeStamp - math.floor(timeStamp)) * 1e3)
    localtime = time.localtime(time_stramp_temp1)
    strTime = '{}-{}-{} {}:{}:{}:{}'.format(str(localtime.tm_year), num2str0(localtime.tm_mon),
                                            num2str0(localtime.tm_mday),
                                            num2str0(localtime.tm_hour), num2str0(localtime.tm_min),
                                            num2str0(localtime.tm_sec),
                                            num2str0(time_stramp_temp2, cnt=3))
    return strTime


def getEscapeData(bytedata):
    replace_old1 = struct.pack('B', 255)
    replace_old2 = struct.pack('B', 254)
    replace_new1 = struct.pack('2B', 254, 1)
    replace_new2 = struct.pack('2B', 254, 0)

    head_data = bytedata[0:2]
    tail_data = bytedata[-1]
    pack_data = bytedata[2:-1]

    new_pack_data = bytes()
    for byteData in pack_data:
        if struct.pack('B', byteData) == replace_old1:
            new_pack_data = new_pack_data + replace_new1
        elif struct.pack('B', byteData) == replace_old2:
            new_pack_data = new_pack_data + replace_new2
        else:
            new_pack_data = new_pack_data + struct.pack('B', byteData)
    pack_data = head_data + new_pack_data + struct.pack('B', tail_data)
    return pack_data


def get_dict_from_csv(strFilePath):
    '''
    Args:
        strFilePath:在线保存的发送结果文件路径，为csv文件
    Returns:
        dictFrame:以帧号为值的字典，值为np数组，格式见解析代码
        nStartFrameId:起始帧号
        nEndFrameId:结束帧号
    '''
    if my_isExist(strFilePath):
        npfile = np.loadtxt(strFilePath, delimiter=',', dtype=str)
        try:
            npFrame = None
            nStartFrameId = None
            dictFrame = {}
            listFrameId = []
            for i in range(1, npfile.shape[0]):
                # nNowFrameId = int(npfile[i,1])
                nNowFrameId = int(float(npfile[i, 24]))
                if nNowFrameId not in listFrameId:
                    listFrameId.append(nNowFrameId)
                    dictFrame[nNowFrameId] = npfile[i, :].reshape((1, npfile.shape[1]))
                dictFrame[nNowFrameId] = np.concatenate(
                    (dictFrame[nNowFrameId], npfile[i, :].reshape((1, npfile.shape[1]))), axis=0)
                if nStartFrameId is None:
                    nStartFrameId = nNowFrameId
            nEndFrameId = max(listFrameId)
            return dictFrame, nStartFrameId, nEndFrameId
        except:
            return {}, 0, 0
    else:
        return {}, 0, 0


def get_dict_from_dir(strFilePath):
    '''
    Args:
        strFilePath:在线保存的发送结果文件路径，为csv文件
    Returns:
        dictFrame:以帧号为值的字典，值为np数组，格式见解析代码
        nStartFrameId:起始帧号
        nEndFrameId:结束帧号
    '''
    if my_isExist(strFilePath):
        try:
            strDataDir = os.path.dirname(strFilePath)
            dir_list = os.listdir(strDataDir)
            listDataFile = []
            if len(dir_list) > 1:
                for file in dir_list:
                    nFrameIdTemp = int(file[7:file.index('csv') - 1])
                    path_file = os.path.join(strDataDir, file)
                    if os.path.isfile(path_file) and "orgData" in file:
                        listDataFile.append(nFrameIdTemp)
            if len(listDataFile) > 1:
                listDataFile = sorted(listDataFile)
            dictFrame = {}
            nStartFrameId = listDataFile[0]
            nEndFrameId = listDataFile[-1]
            for nFrameIdTemp in listDataFile:
                strfilename = os.path.join(strDataDir, 'orgData{}.csv'.format(nFrameIdTemp))
                npStationdata0 = np.loadtxt(strfilename, delimiter=',', dtype=float)
                dictFrame[nFrameIdTemp] = npStationdata0

            return dictFrame, nStartFrameId, nEndFrameId
        except:
            return {}, 0, 0
    else:
        return {}, 0, 0


def get_dict_from_txt(strFilePath: str):
    '''
    Args:
        strFilePath:在线保存的发送结果文件路径，为txt文件
    Returns:
        dictFrame:以帧号为值的字典，值为np数组，格式见解析代码
        nStartFrameId:起始帧号
        nEndFrameId:结束帧号
    '''
    if my_isExist(strFilePath):
        try:
            with open(strFilePath, 'r') as f:
                listFrame = f.readlines()
                nStartFrameId = None
                nFrameIndex = 0
                dictFrame = {}
                for Frame in listFrame:
                    try:
                        # if nFrameIndex > stUiParam.nOfflineFrame:
                        #     break
                        listOneFrame = Frame.split(':')
                        strResult = listOneFrame[-1]
                        strResult = strResult.replace(' ', '')

                        byteResult = bytes()
                        for i in range(math.floor(len(strResult) / 2)):
                            strTemp = strResult[2 * i:+2 * i + 2]
                            num = str2num(strTemp[0]) * 16 + str2num(strTemp[1])
                            byteResult = byteResult + struct.pack('B', num)
                        npBoxInfo, listDevInfo = parseResultData_new(byteResult)
                        if nStartFrameId is None:
                            nStartFrameId = listDevInfo[2]
                        nNowFrameId = nStartFrameId + nFrameIndex
                        # nNowFrameId = listDevInfo[2]
                        dictFrame[nNowFrameId] = npBoxInfo
                        nFrameIndex += 1
                    except:
                        continue
                nEndFrameId = nNowFrameId
                return dictFrame, nStartFrameId, nEndFrameId
        except:
            return {}, 0, 0
    else:
        return {}, 0, 0


def get_dict_from_json(strFilePath: str):
    '''
    Args:
        strFilePath:在线保存的发送结果文件路径，为json文件
    Returns:
        dictFrame:以帧号为值的字典，值为np数组，格式见解析代码
        nStartFrameId:起始帧号
        nEndFrameId:结束帧号
    '''
    if my_isExist(strFilePath):
        try:
            with open(strFilePath, 'r') as jsonFile:
                listJsonData = jsonFile.readlines()
                dictFrame = {}
                nStartFrameId = 0
                nFrameCount = 0
                for jsonData in listJsonData:
                    if nFrameCount > stUiParam.nOfflineFrame:
                        break
                    dictOneFrame = json.loads(jsonData)
                    if nStartFrameId == 0:
                        nStartFrameId = dictOneFrame['frameID']
                    npOneFrame = np.zeros((dictOneFrame['participantNum'], 35))
                    npOneFrame[:, 0:7] = np.array([dictOneFrame['deviceID'],
                                                   dictOneFrame['frameID'],
                                                   0,
                                                   dictOneFrame['lidarLongitude'],
                                                   dictOneFrame['lidarLatitude'],
                                                   dictOneFrame['lidarAngle'],
                                                   dictOneFrame['participantNum'],
                                                   ])
                    listOneFrame = dictOneFrame['participants']
                    for i in range(len(listOneFrame)):
                        dictOneTarget = listOneFrame[i]
                        npOneFrame[i, 7:35] = np.array(
                            [dictOneTarget['ID'], dictOneTarget['type'], dictOneTarget['confidence'],
                             dictOneTarget['vehicleColor'],
                             dictOneTarget['source'], dictOneTarget['globalLaneNum'], dictOneTarget['longitude'],
                             dictOneTarget['latitude'],
                             dictOneTarget['altitude'], dictOneTarget['speed'], dictOneTarget['courseAngle'],
                             dictOneTarget['length'],
                             dictOneTarget['width'], dictOneTarget['height'], dictOneTarget['xCoordinate'],
                             dictOneTarget['yCoordinate'],
                             dictOneTarget['zCoordinate'], dictOneFrame['frameID'], dictOneTarget['baseStationSource'],
                             0,
                             dictOneTarget['laneNum'], 0, 0, 0,
                             dictOneTarget['licenseColor'], 0, 0, 0])
                    dictFrame[nStartFrameId + nFrameCount] = npOneFrame
                    nFrameCount += 1
                nEndFrameId = nStartFrameId + nFrameCount - 1
            return dictFrame, nStartFrameId, nEndFrameId
        except:
            return {}, 0, 0
    else:
        return {}, 0, 0


def getHttpB3Data(tMsgContent, dstData):
    """B3帧http协议组包"""
    eventList = []
    for i in range(tMsgContent[5]):
        dict_lane_info = {'laneNo': int(tMsgContent[7][i][0] % 255),
                          'laneType': int(tMsgContent[7][i][1]),
                          'laneDirection': int(math.floor(tMsgContent[7][i][2])),
                          'laneSpeedLimit': int(math.floor(tMsgContent[7][i][3])),
                          'laneSpeedAvg': int(math.floor(tMsgContent[7][i][4])),
                          'queueLength': int(math.floor(tMsgContent[7][i][5])),
                          'laneHeadSpace': int(math.floor(tMsgContent[7][i][6])),
                          'spaceOccupancy': int(math.floor(tMsgContent[7][i][7])),
                          'vehicleNum': int(math.floor(tMsgContent[7][i][9])),
                          'carNum': int(math.floor(tMsgContent[7][i][10])),
                          'truckNum': int(math.floor(tMsgContent[7][i][11])),
                          'busNum': int(math.floor(tMsgContent[7][i][12])),
                          'mediumBusNum': int(math.floor(tMsgContent[7][i][13])),
                          'dangerCarNum': int(math.floor(tMsgContent[7][i][14])),
                          'pedestrianNum': int(math.floor(tMsgContent[7][i][15])),
                          'nonVehicleNum': int(math.floor(tMsgContent[7][i][16]))}
        # ucReservel1 = tMsgContent[7][i][8]
        # uiReservel2 = tMsgContent[7][i][17]
        eventList.append(dict_lane_info)

    dict_B3 = {'deviceId': dstData.nEqupId[1],
               'timeStamp': getTimeStamp(time.time()),
               'eventFrameId': int(math.floor(tMsgContent[4])),
               'laneNum': int(math.floor(tMsgContent[5])),
               'eventList': eventList}
    json_B3 = json.dumps(dict_B3)
    return json_B3


def getHttpB4Data(tMsgContent, dstData, startTimeStamp):
    """B4帧http协议组包"""
    eventList = []
    for i in range(tMsgContent[7]):
        stationCongestionList = []
        for j in range(tMsgContent[9][i][4]):
            dictStationCongestion = {'stationId': int(math.floor(tMsgContent[9][i][6][j][0])),
                                     'congestionState': int(math.floor(tMsgContent[9][i][6][j][1]))}
            stationCongestionList.append(dictStationCongestion)
        dict_lane_info = {'laneNo': int(math.floor(tMsgContent[9][i][0] % 255)),
                          'laneType': int(math.floor(tMsgContent[9][i][1])),
                          'laneDirection': int(math.floor(tMsgContent[9][i][2])),
                          'laneSpeedLimit': int(math.floor(tMsgContent[9][i][3])),
                          'stationNum': int(math.floor(tMsgContent[9][i][4])),
                          'stationCongestionList': stationCongestionList,
                          'followPercent': int(math.floor(tMsgContent[9][i][7])),
                          'timeOccupancy': int(math.floor(tMsgContent[9][i][8])),
                          'headSpaceAvg': int(math.floor(tMsgContent[9][i][9])),
                          'carFlow': int(math.floor(tMsgContent[9][i][10])),
                          'truckFlow': int(math.floor(tMsgContent[9][i][11])),
                          'busFLow': int(math.floor(tMsgContent[9][i][12])),
                          'mediumBusFlow': int(math.floor(tMsgContent[9][i][13])),
                          'dangerCarFlow': int(math.floor(tMsgContent[9][i][14])),
                          'carSpeedAvg': int(math.floor(tMsgContent[9][i][15])),
                          'truckSpeedAvg': int(math.floor(tMsgContent[9][i][16])),
                          'busSpeedAvg': int(math.floor(tMsgContent[9][i][17])),
                          'mediumBusSpeedAvg': int(math.floor(tMsgContent[9][i][18])),
                          'dangerCarSpeedAvg': int(math.floor(tMsgContent[9][i][19]))}
        # ucReservel1 = tMsgContent[9][i][5]
        # uiReservel2 = tMsgContent[9][i][20]
        eventList.append(dict_lane_info)
    thisTime = time.time()
    thisTimeStamp = getTimeStamp(thisTime)
    dict_B4 = {'deviceId': dstData.nEqupId[1],
               'startTimeStamp': startTimeStamp,
               'endTimeStamp': thisTimeStamp,
               'eventFrameId': int(math.floor(tMsgContent[6])),
               'laneNum': int(math.floor(tMsgContent[7])),
               'eventList': eventList}
    json_B4 = json.dumps(dict_B4)
    return json_B4, thisTimeStamp


def getHttpB5Data(tMsgContent, dstData):
    """B5帧http协议组包"""
    b5eventList = []
    for i in range(tMsgContent[4]):
        targetList = []
        for j in range(tMsgContent[6][i][4]):
            dictTarget = {'targetId': int(math.floor(tMsgContent[6][i][5][j][0])),
                          'eventConfidence': int(math.floor(tMsgContent[6][i][5][j][1] * 100))}
            targetList.append(dictTarget)
        dict_lane_info = {'eventId': int(math.floor(tMsgContent[6][i][0])),
                          'eventType': int(math.floor(tMsgContent[6][i][1])),
                          'stationId': int(math.floor(tMsgContent[6][i][2])),
                          'laneNo': int(math.floor(tMsgContent[6][i][3])),
                          'targetNum': int(math.floor(tMsgContent[6][i][4])),
                          'targetList': targetList,
                          'eventLongitude': round(tMsgContent[6][i][6], 7),
                          'eventLatitude': round(tMsgContent[6][i][7], 7),
                          'eventTimeStamp': getTimeStamp(tMsgContent[6][i][8] +
                                                         tMsgContent[6][i][9] / 1e6)}
        b5eventList.append(dict_lane_info)
    dict_B5 = {'deviceId': dstData.nEqupId[1],
               'timeStamp': getTimeStamp(dstData.tTimeStamp),
               'eventNum': int(math.floor(tMsgContent[4])),
               'b5eventList': b5eventList}
    json_B5 = json.dumps(dict_B5)
    return json_B5


del_file_path = './savedata/org_data/'
def get_disk_space(path='/'):
    usage = psutil.disk_usage(path)
    # usage.total /1024/1024, usage.used/1024/1024 ,usage.free/1024/1024
    MyLog.writeLog("{}[:{}] - disk_space:total:{} G ,used:{} G , free:{} G , percent:{}% ".format(
        __file__.split('/')[len(__file__.split('/')) - 1],
        sys._getframe().f_lineno, round(usage.total / 1024 / 1024 / 1024, 2), round(usage.used / 1024 / 1024 / 1024, 2),
        round(usage.free / 1024 / 1024 / 1024, 2), usage.percent), LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)
    return usage.percent

# 文件按最后修改时间排序
def get_file_list(file_path):
    dir_list = os.listdir(file_path)
    if not dir_list:
        return
    else:
        dir_list = sorted(dir_list, key=lambda x: os.path.getmtime(os.path.join(file_path, x)))
        # print(dir_list)
        return dir_list

# 检测线程，每个60秒检测一次
def detectMemSize():
    while True:
        print('=====================detect======================')
        use_percent = get_disk_space()
        if use_percent > 90:
            MyLog.writeLog("{}[:{}] - Memory use over 90% , need to delete 1 savedata file".
                           format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno),
                           LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
            fileList = get_file_list(del_file_path)
            if len(fileList) > 3:
                shutil.rmtree(del_file_path + fileList[0])  # 删除非空文件夹
                MyLog.writeLog("{}[:{}] - del {}".
                               format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, fileList[0]),
                               LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
            else:
                MyLog.writeLog("{}[:{}] - savadata  num is less then 3 , cannot delete files".
                               format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno),
                               LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
        time.sleep(60)
