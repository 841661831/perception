# -*-coding:utf-8-*
from Widget.PyGLWidget import PyGLWidget
from PointCloud.PointCloud import *
from second.udp_send import packet_head, object_packet_data, object_nums, information_head
from OpenGL.GLU import gluProject
from second.dead_zone import get_object
import requests

BACKGROUND_BLACK = True

class PcPyGLWidget(PyGLWidget):
    sigPlayOver = pyqtSignal(bool)
    sigSelectBox = pyqtSignal(int)
    sigSelectStationBox = pyqtSignal(list)
    sigDrawRoiOver = pyqtSignal()

    def __init__(self, parent=None):
        PyGLWidget.__init__(self, parent)
        self.parent = parent

        self.stParam = structLidarParam()
        self.stDstData = None
        self.udpSocket = None
        self.lockPc = Lock()
        self.strForwardDevIp = ""
        self.nForwardDevPort = ""

        self.nDelay = 0
        self.nFrameCount = 0
        self.timeTest = QTime()

        self.modelview = np.zeros((4, 4))
        self.projection = np.zeros((4, 4))
        self.viewport = [0, 0, self.fWidth, self.fHeight]
        self.listRect = []
        self.listBoxInfo = []
        self.nSelectBoxId = -1
        self.listSelectBoxId = []
        self.dictHisTrack = {}
        self.selectBoxInfo = None

        self.firstPoint = None
        self.secondPoint = None


        self.rectSelectPoint = None

        self.rectInfo = None

        self.painter = QPainter()

        self.bShowAround = False
        self.npDeadId = np.array([])
        self.npOccluId = np.array([])

        self.bProCreated = False
        self.second = None
        self.read = None
        self.detect = None

        self.bFusion = True
        self.bShowBox = True
        self.bShowLane = True
        self.bShowRLFusion = True
        self.bFirstNoData = True

        self.GridWidth = 200
        self.GridHeight = 200
        self.GridStep = 5
        self.resize(450, 260)  # 重置一下大小，防止显示异常
        self.timeB1 = QTime()
        self.uiStartTimeStamp1 = 0
        self.uiStartTimeStamp2 = 0
        self.thisTimeStamp = None


        self.stUiParam = structUiParam()
        serializationParam(stUiParam=self.stUiParam)
        self.pcBaseQueueList = []

        self.bshowSelectCube = False
        self.listSelectCubeCenter = []
        self.listSelectVertexPc = []
        self.nSelectedTableRow = -1
        self.nSelectedTableCol = -1
        self.srcEventDataQueue = Queue()
        self.dstEventResultQueue = Queue()
        self.listBaseStationId = []
        # self.timeTemp = QTime()
        # self.timeTemp.start()


    def __del__(self):
        # print('__del__ start...')
        if self.second is not None and self.second.is_alive():
            self.second.terminate()

        if self.read is not None and self.read.is_alive():
            self.read.terminate()

        if self.detect is not None and self.detect.is_alive():
            self.detect.terminate()
        # print('__del__ end!')

    def setLIDARParam(self, stParam=None):
        # 从结构体bufParam中解析出相关的变量
        # self.stop()
        self.stParam.setPause(True)

        while self.srcEventDataQueue.qsize() > 0:
            try:
                toBeDel = self.srcEventDataQueue.get_nowait()
            except:
                continue

        while self.dstEventResultQueue.qsize() > 0:
            try:
                toBeDel = self.dstEventResultQueue.get_nowait()
            except:
                continue

        while self.stSrcPcResultQueue.qsize() > 0:
            try:
                toBeDel = self.stSrcPcResultQueue.get_nowait()
            except:
                continue

        if stParam is not None:
            self.stParam = stParam

        self.udpSocket = self.stParam.getUdpSocket()
        self.stParam.setStatus(0)
        listPcOver = np.full(len(self.stParam.getListOfflineFiles()), False, bool)
        self.stParam.setPcOver(listPcOver)
        self.stParam.setNoPcData(False)
        self.stParam.setFrameId(0)
        # self.stParam.setOnline(True)
        self.stParam.setReadChanged(True)
        self.stParam.setAlgChanged(True)
        self.stParam.setParamChanged(True)
        self.stParam.setPause(False)
        listNorthAngleT = []
        for i in range(len(self.stParam.getlistAttachedIp())):
            listNorthAngleT.append(self.stParam.getAngleNorthT(i))
            self.listBaseStationId.append(0)
        listSortNorthAngleT = sorted(listNorthAngleT)
        for i in range(len(self.stParam.getlistAttachedIp())):
            if self.stParam.getHighSpeed():
                self.listBaseStationId[i] = i + 1
            else:
                self.listBaseStationId[i] = listSortNorthAngleT.index(listNorthAngleT[i]) + 1

        self.start()
        return

    def setSelectBoxId(self, nSelectBoxId):
        self.nSelectBoxId = nSelectBoxId

        if self.bShowAround:
            boxCenter = None
            for boxInfo in self.listBoxInfo:
                if boxInfo.boxId == nSelectBoxId:
                    boxCenter = boxInfo.boxCenter

            if boxCenter is not None:
                mask, self.npDeadId, self.npOccluId = get_object(boxCenter, self.stDstData.pcTrackersCov, self.stDstData.pcBox, 20, 10)
            else:
                self.npDeadId, self.npOccluId = np.array([]), np.array([])
        self.updateGL()

    def setAutoLoopPlay(self, bLoop):
        self.stParam.setAutoLoopPlay(bLoop)

    def setShowBox(self, bShowBox):
        self.bShowBox = bShowBox
        self.update()

    def setShowLane(self, bShowLane):
        self.bShowLane = bShowLane
        self.update()

    def setShowRLFusion(self, bShowRLFusion):
        self.bShowRLFusion = bShowRLFusion
        self.update()

    def setManualReplay(self, bReplay):
        self.stParam.setManualReplay(bReplay)

    def isPlayOver(self):
        bOver = True
        listOver = self.stParam.getPcOver()
        for i in range(len(listOver)):
            bOver &= listOver[i]
        return bOver


    def setSyncParam(self, q):
        self.stSrcPcResultQueue = q

    # B0帧数据打包
    def B0_pcak(self, data: B0TransientEvent) -> B0TransientEvent:
        head_data = struct.pack('>HBBBB', data.usBeginBit, data.ucSerialNum, data.ucCommandNum, data.ucSubComNum,
                                data.ucStatusBit)
        msg_len = struct.pack('>H', data.usMsgLen)
        msg_con_data = bytes()
        channel_data = bytes()

        #tempEquiqId = self.stDstData.headData[1]
        msg_con_data = msg_con_data + struct.pack('BB', int(self.stDstData.headData[1]),int(self.stDstData.headData[0])) + \
                       struct.pack('>H', int(data.MsgContent.usReserve1)) + \
                       struct.pack('>I', int(data.MsgContent.uiTimeStamp1)) + \
                       struct.pack('>I', int(data.MsgContent.uiTimeStamp2 / 4.295)) + \
                       struct.pack('>H', int(data.MsgContent.usCarChannelNum))+ \
                       struct.pack('>H', int(data.MsgContent.usReserve2))
        for j in range(len(data.MsgContent.tChannelData)):
            channel_data = channel_data + struct.pack('>H', int(data.MsgContent.tChannelData[j].usChannelId))+ \
                           struct.pack('>H', int(data.MsgContent.tChannelData[j].usChannelNum)) + \
                           struct.pack('>H', int(data.MsgContent.tChannelData[j].usChannelDirection)) + \
                           struct.pack('B', int(data.MsgContent.tChannelData[j].ucChannelLineDir) )+ \
                           struct.pack('B', int(data.MsgContent.tChannelData[j].ucChannelType) )+ \
                           struct.pack('>H', int(data.MsgContent.tChannelData[j].usChannelSpeedLimit)) + \
                           struct.pack('>H', int (data.MsgContent.tChannelData[j].usChannelReservel1)) + \
                           struct.pack('>I', int(data.MsgContent.tChannelData[j].uiQueueLength* 100)) + \
                           struct.pack('>I', int(data.MsgContent.tChannelData[j].uiSpcaeOccupancy* 100)) + \
                           struct.pack('B', int(data.MsgContent.tChannelData[j].ucCarNum)) + \
                           struct.pack('B', int(data.MsgContent.tChannelData[j].uc3ChannelReservel2[0])) + \
                           struct.pack('B', int(data.MsgContent.tChannelData[j].uc3ChannelReservel2[1])) + \
                           struct.pack('B', int(data.MsgContent.tChannelData[j].uc3ChannelReservel2[2]))

            car_head_dis = bytes()
            if data.MsgContent.tChannelData[j].ucCarNum < 2:
                car_head_dis = car_head_dis + bytes()
            else:
                for k in range(len(data.MsgContent.tChannelData[j].uiCarHeadDis)):
                    car_head_dis = car_head_dis + struct.pack('>H', int(data.MsgContent.tChannelData[j].uiCarHeadDis[k]* 100))

            channel_data = channel_data + car_head_dis + struct.pack('>H', int(data.MsgContent.tChannelData[j].usChannelReservel3))

        msg_con_data = msg_con_data + channel_data + struct.pack('>I', int(data.MsgContent.uiReserve3))
        datalen = len(msg_con_data)
        check_data = head_data + struct.pack('>H', datalen) + msg_con_data
        check_bit = getBccCheckVal(check_data)
        pack_data = check_data + struct.pack('B', check_bit) + struct.pack('B', data.ucCutoffBit)
        # print("13232", pack_data)
        return pack_data

    #B1帧数据打包
    def B1_pcak(self, data: B1TransientEvent) -> B1TransientEvent:
        head_data = struct.pack('>HBBBB', data.usBeginBit, data.ucSerialNum, data.ucCommandNum, data.ucSubComNum,
                                data.ucStatusBit)
        msg_len = struct.pack('>H', data.usMsgLen)
        msg_con_data = bytes()
        channel_data = bytes()
        msg_con_data = msg_con_data + struct.pack('BB', int(self.stDstData.headData[1]),int(self.stDstData.headData[0])) + \
                       struct.pack('>H', int(data.MsgContent.usReserve1)) + \
                       struct.pack('>I', int(data.MsgContent.uiStartTimeStamp1)) + \
                       struct.pack('>I', int(data.MsgContent.uiStartTimeStamp2 / 4.295)) + \
                       struct.pack('>I', int(data.MsgContent.uiEndTimeStamp1)) + \
                       struct.pack('>I', int(data.MsgContent.uiEndTimeStamp2 / 4.295)) + \
                       struct.pack('>H', int(data.MsgContent.usCarChannelNum)) + \
                       struct.pack('>H', int(data.MsgContent.usReserve2))
        for j in range(len(data.MsgContent.tB1ChannelData)):
            # usTimeShare* 100  不知道是否要乘100,单位现在不确定
            channel_data = channel_data + struct.pack('>H', int(data.MsgContent.tB1ChannelData[j].usChannelId)) + \
                           struct.pack('>H', int(data.MsgContent.tB1ChannelData[j].usChannelNum)) + \
                           struct.pack('>H', int(data.MsgContent.tB1ChannelData[j].usChannelDirection)) + \
                           struct.pack('B', int(data.MsgContent.tB1ChannelData[j].ucChannelLineDir)) + \
                           struct.pack('B', int(data.MsgContent.tB1ChannelData[j].ucChannelType)) + \
                           struct.pack('>H', int(data.MsgContent.tB1ChannelData[j].usChannelSpeedLimit)) + \
                           struct.pack('B', int(data.MsgContent.tB1ChannelData[j].ucCongestionStatus)) + \
                           struct.pack('>H', int(data.MsgContent.tB1ChannelData[j].usTraffic)) + \
                           struct.pack('>H', int(data.MsgContent.tB1ChannelData[j].usTimeShare * 100)) + \
                           struct.pack('>H', int(data.MsgContent.tB1ChannelData[j].usCar)) + \
                           struct.pack('>H', int(data.MsgContent.tB1ChannelData[j].usTruck)) + \
                           struct.pack('>H', int(data.MsgContent.tB1ChannelData[j].usBus)) + \
                           struct.pack('>H', int(data.MsgContent.tB1ChannelData[j].usMotorbike)) + \
                           struct.pack('>H', int(data.MsgContent.tB1ChannelData[j].usChannelReservel1))
        temp_msg_len = len(msg_con_data) + len(channel_data)

        check_data = head_data + struct.pack('>H', temp_msg_len) + msg_con_data + channel_data
        check_bit = getBccCheckVal(check_data)
        pack_data = check_data + struct.pack('B', check_bit) + struct.pack('B', data.ucCutoffBit)
        return pack_data

    # B2帧数据打包
    def B2_pcak(self, data: B2TrafficIncident) -> B2TrafficIncident:
        head_data = struct.pack('>HBBBB', data.usBeginBit, data.ucSerialNum, data.ucCommandNum, data.ucSubComNum,
                                data.ucStatusBit)
        msg_len = struct.pack('>H', data.usMsgLen)
        msg_con_data = bytes()
        msg_except_data = bytes()
        msg_ip_data = bytes()
        msg_other_data = bytes()
        violation_data = bytes()
        middle_data = bytes()
        retrograde_data = bytes()
        msg_con_data = msg_con_data + struct.pack('BB', int(self.stDstData.headData[1]), int(self.stDstData.headData[0])) + \
                       struct.pack('>H', int(data.MsgContent.usReserve1)) + \
                       struct.pack('>I', int(data.MsgContent.uiStartTimeStamp1)) + \
                       struct.pack('>I', int(data.MsgContent.uiStartTimeStamp2 / 4.295)) + \
                       struct.pack('>H', int(data.MsgContent.ExceptEentNumbers )) + \
                       struct.pack('>H', int(data.MsgContent.usReserve2))



        #if
        for i in range(data.MsgContent.ExceptEentNumbers):
            msg_except_data =  struct.pack('>H', int(data.MsgContent.tTargetMsg[i].usExceptId)) + \
                               struct.pack('>H', int(data.MsgContent.tTargetMsg[i].usExceptType)) + \
                               struct.pack('>H', int(data.MsgContent.tTargetMsg[i].uiCameraNumbers))
            for j in range(data.MsgContent.tTargetMsg[i].uiCameraNumbers):
                listCameraDev = self.stParam.getStationCameraDev(data.MsgContent.tTargetMsg[i].uiCameraIp[j][0])
                strCameraIp = listCameraDev[data.MsgContent.tTargetMsg[i].uiCameraIp[j][1]].strCameraIp
                msg_ip_data = msg_ip_data + struct.pack('B', int(strCameraIp.split('.')[-1]))
            msg_except_data = msg_except_data + msg_ip_data  + \
                              struct.pack('>I', int(data.MsgContent.tTargetMsg[i].uiExceptBeginTime1))+ \
                              struct.pack('>I', int(data.MsgContent.tTargetMsg[i].uiExceptBeginTime2)) + \
                              struct.pack('>I', int(data.MsgContent.tTargetMsg[i].u2iExceptViolationTime)) + \
                              struct.pack('>H', int(data.MsgContent.tTargetMsg[i].usReservel1))
            msg_other_data = msg_other_data + msg_except_data

        msg_other_data = msg_other_data + struct.pack('>H', int(data.MsgContent.usReserve3))

        temp_msg_len = len(msg_con_data) + len(msg_other_data)

        check_data = head_data + struct.pack('>H', temp_msg_len) + msg_con_data + msg_other_data
        check_bit = getBccCheckVal(check_data)
        pack_data = check_data + struct.pack('B', check_bit) + struct.pack('B', data.ucCutoffBit)
        return pack_data

    # B3帧数据打包
    def B3_pcak(self, data: B3TransientEvent) -> B3TransientEvent:
        head_data = struct.pack('>HBBBB', data.usBeginBit, data.ucSerialNum, data.ucCommandNum, data.ucSubComNum, data.ucStatusBit)
        msg_len = struct.pack('>H', data.usMsgLen)
        msg_con_data = bytes()
        channel_data = bytes()
        len_channel_data = 0
        msg_con_data = msg_con_data + struct.pack('BB', int(self.stDstData.headData[1]), int(self.stDstData.headData[0])) + \
                       struct.pack('>H', int(data.MsgContent.usReserve1)) + \
                       struct.pack('>I', int(data.MsgContent.uiTimeStampS)) + \
                       struct.pack('>I', int(data.MsgContent.uiTimeStampUs / 4.295)) + \
                       struct.pack('>I', int(data.MsgContent.uiFrameId)) + \
                       struct.pack('B', int(data.MsgContent.ucCarChannelNums)) + \
                       struct.pack('B', int(data.MsgContent.u3cReserve2[0])) + \
                       struct.pack('B', int(data.MsgContent.u3cReserve2[1])) + \
                       struct.pack('B', int(data.MsgContent.u3cReserve2[2]))

        for j in range(len(data.MsgContent.tInfoData)):
            channel_data = channel_data + struct.pack('B', int(data.MsgContent.tInfoData[j].ucChannelId)) + \
                           struct.pack('B', int(data.MsgContent.tInfoData[j].ucChannelType)) + \
                           struct.pack('>H', int(data.MsgContent.tInfoData[j].usChannelDirection)) + \
                           struct.pack('B', int(data.MsgContent.tInfoData[j].ucChannelSpeedLimit)) + \
                           struct.pack('B', int(data.MsgContent.tInfoData[j].ucMeanSpeed)) + \
                           struct.pack('>I', int(data.MsgContent.tInfoData[j].uiQueueLength * 100)) + \
                           struct.pack('>H', int(data.MsgContent.tInfoData[j].usChannelInterval)) + \
                           struct.pack('B', int(data.MsgContent.tInfoData[j].ucSpcaeOccupancy * 100)) + \
                           struct.pack('B', int(data.MsgContent.tInfoData[j].ucReservel1)) + \
                           struct.pack('>H', int(data.MsgContent.tInfoData[j].usMotorVehicles)) + \
                           struct.pack('>H', int(data.MsgContent.tInfoData[j].usCarNum)) + \
                           struct.pack('>H', int(data.MsgContent.tInfoData[j].usTruckFreight)) + \
                           struct.pack('>H', int(data.MsgContent.tInfoData[j].usBigBus)) + \
                           struct.pack('>H', int(data.MsgContent.tInfoData[j].usMediumBus)) + \
                           struct.pack('>H', int(data.MsgContent.tInfoData[j].usTankCar)) + \
                           struct.pack('>H', int(data.MsgContent.tInfoData[j].usPeople)) + \
                           struct.pack('>H', int(data.MsgContent.tInfoData[j].usNoMotorVehicles)) + \
                           struct.pack('>H', int(data.MsgContent.tInfoData[j].uiReservel2))
            len_channel_data = len(channel_data)

        msg_reserve3 = struct.pack('>I', int(data.MsgContent.uiReserve3))
        temp_msg_len = len(msg_con_data) + len_channel_data + len(msg_reserve3)

        check_data = head_data + struct.pack('>H', temp_msg_len) + msg_con_data + channel_data + msg_reserve3
        check_bit = getBccCheckVal(check_data)
        pack_data = check_data + struct.pack('B', check_bit) + struct.pack('B', data.ucCutoffBit)
        pack_data = getEscapeData(pack_data)
        return pack_data

        # B4帧数据打包

    def B4_pcak(self, data: B4TransientEvent) -> B4TransientEvent:
        head_data = struct.pack('>HBBBB', data.usBeginBit, data.ucSerialNum, data.ucCommandNum, data.ucSubComNum, data.ucStatusBit)
        msg_len = struct.pack('>H', data.usMsgLen)
        msg_con_data = bytes()
        channel_data = bytes()
        msg_con_data = msg_con_data + struct.pack('BB', int(self.stDstData.headData[1]), int(self.stDstData.headData[0])) + \
                        struct.pack('>H', int(data.MsgContent.usReserve1)) + \
                        struct.pack('>I', int(data.MsgContent.uiStartTimeStampS)) + \
                        struct.pack('>I', int(data.MsgContent.uiStartTimeStampUs / 4.295)) + \
                        struct.pack('>I', int(data.MsgContent.uiEndTimeStampS)) + \
                        struct.pack('>I', int(data.MsgContent.uiEndTimeStampUs / 4.295)) + \
                        struct.pack('>I', int(data.MsgContent.uiFrameId)) + \
                        struct.pack('B', int(data.MsgContent.ucCarChannelNums)) + \
                        struct.pack('B', int(data.MsgContent.u3cReserve2[0])) + \
                        struct.pack('B', int(data.MsgContent.u3cReserve2[1])) + \
                        struct.pack('B', int(data.MsgContent.u3cReserve2[2]))

        for i in range(len(data.MsgContent.tInfoData)):
        # usTimeShare* 100  不知道是否要乘100,单位现在不确定
            channel_data = channel_data + struct.pack('B', int(data.MsgContent.tInfoData[i].ucChannelId)) + \
                            struct.pack('B', int(data.MsgContent.tInfoData[i].ucChannelType)) + \
                            struct.pack('>H', int(data.MsgContent.tInfoData[i].usChannelDirection)) + \
                            struct.pack('B', int(data.MsgContent.tInfoData[i].ucChannelSpeedLimit)) + \
                            struct.pack('B', int(data.MsgContent.tInfoData[i].ucStationNums)) + \
                            struct.pack('>H', int(data.MsgContent.tInfoData[i].usReservel1))

            for j in range(len(data.MsgContent.tInfoData[i].tStationData)):
                channel_data = channel_data + struct.pack('B', int(data.MsgContent.tInfoData[i].tStationData[j].ucStationId)) + \
                                struct.pack('B', int(data.MsgContent.tInfoData[i].tStationData[j].ucCongestion)) + \
                                struct.pack('>H', int(data.MsgContent.tInfoData[i].tStationData[j].usReservel))
            channel_data = channel_data + struct.pack('B', int(data.MsgContent.tInfoData[i].ucCarfollowpercen * 100)) + \
                            struct.pack('B', int(data.MsgContent.tInfoData[i].ucTimeOccupancy * 100)) + \
                            struct.pack('>H', int(data.MsgContent.tInfoData[i].usCarHeadDis)) + \
                            struct.pack('>H', int(data.MsgContent.tInfoData[i].usCarFlow)) + \
                            struct.pack('>H', int(data.MsgContent.tInfoData[i].usTruckFreightFlow)) + \
                            struct.pack('>H', int(data.MsgContent.tInfoData[i].usBigBusFlow)) + \
                            struct.pack('>H', int(data.MsgContent.tInfoData[i].usMediumBusFlow)) + \
                            struct.pack('>H', int(data.MsgContent.tInfoData[i].usTankCarFlow)) + \
                            struct.pack('B', int(data.MsgContent.tInfoData[i].ucCarAverSpeed)) + \
                            struct.pack('B', int(data.MsgContent.tInfoData[i].ucTruckFreightAverSpeed)) + \
                            struct.pack('B', int(data.MsgContent.tInfoData[i].ucBigBusAverSpeed)) + \
                            struct.pack('B', int(data.MsgContent.tInfoData[i].ucMediumBusAverSpeed)) + \
                            struct.pack('B', int(data.MsgContent.tInfoData[i].ucTankCarAverSpeed)) + \
                            struct.pack('B', int(data.MsgContent.tInfoData[i].ucReservel2))

        temp_msg_len = len(msg_con_data) + len(channel_data)

        check_data = head_data + struct.pack('>H', temp_msg_len) + msg_con_data + channel_data
        check_bit = getBccCheckVal(check_data)
        pack_data = check_data + struct.pack('B', check_bit) + struct.pack('B', data.ucCutoffBit)
        pack_data = getEscapeData(pack_data)
        return pack_data

    # B5帧数据打包
    def B5_pcak(self, data: B5TransientEvent) -> B5TransientEvent:
            head_data = struct.pack('>HBBBB', data.usBeginBit, data.ucSerialNum, data.ucCommandNum, data.ucSubComNum, data.ucStatusBit)
            msg_len = struct.pack('>H', data.usMsgLen)
            msg_con_data = bytes()
            msg_except_data = bytes()
            msg_end_data = bytes()

            msg_con_data = msg_con_data + struct.pack('BB', int(self.stDstData.headData[1]), int(self.stDstData.headData[0])) + \
                           struct.pack('>H', int(data.MsgContent.usReserve1)) + \
                           struct.pack('>I', int(data.MsgContent.uiStartTimeStampS)) + \
                           struct.pack('>I', int(data.MsgContent.uiStartTimeStampUs / 4.295)) + \
                           struct.pack('>H', int(data.MsgContent.ucExceptEentNums)) + \
                           struct.pack('>H', int(data.MsgContent.usReserve2))

            for i in range(data.MsgContent.ucExceptEentNums):
                msg_except_data = msg_except_data + struct.pack('>I', int(data.MsgContent.tInfoData[i].uiExceptId)) + \
                                                    struct.pack('B', int(data.MsgContent.tInfoData[i].ucExceptType)) + \
                                                    struct.pack('B', int(data.MsgContent.tInfoData[i].ucExceptOfStation)) + \
                                                    struct.pack('B', int(data.MsgContent.tInfoData[i].ucExceptOfChannel)) + \
                                                    struct.pack('B', int(data.MsgContent.tInfoData[i].ucExceptTargetNums))
                for j in range(data.MsgContent.tInfoData[i].ucExceptTargetNums):
                    if int(data.MsgContent.tInfoData[i].ExceptTarget[j].usTargetId) < 0:
                        data.MsgContent.tInfoData[i].ExceptTarget[j].usTargetId = 0
                    msg_except_data = msg_except_data + struct.pack('>H', int(data.MsgContent.tInfoData[i].ExceptTarget[j].usTargetId % 65000)) +\
                                      struct.pack('>H', int(data.MsgContent.tInfoData[i].ExceptTarget[j].usReservel))
                msg_except_data = msg_except_data + struct.pack('>i', int(data.MsgContent.tInfoData[i].uiExceptLong*1e7)) + \
                                                    struct.pack('>i', int(data.MsgContent.tInfoData[i].uiExceptLat*1e7)) + \
                                                    struct.pack('>I', int(data.MsgContent.tInfoData[i].uiExceptBeginTimeS)) + \
                                                    struct.pack('>I', int(data.MsgContent.tInfoData[i].uiExceptBeginTimeUs / 4.295)) + \
                                                    struct.pack('>I', int(data.MsgContent.tInfoData[i].uiReservel))
            msg_end_data = struct.pack('>I', int(data.MsgContent.uiReserve3))

            temp_msg_len = len(msg_con_data) + len(msg_except_data) + len(msg_end_data)
            check_data = head_data + struct.pack('>H', temp_msg_len) + msg_con_data + msg_except_data + msg_end_data
            check_bit = getBccCheckVal(check_data)
            pack_data = check_data + struct.pack('B', check_bit) + struct.pack('B', data.ucCutoffBit)
            pack_data = getEscapeData(pack_data)
            return pack_data

    def getPost2BS(self, data: B5TransientEvent) -> B5TransientEvent:
        head_data = struct.pack('>HBBBB', data.usBeginBit, data.ucSerialNum, 171, data.ucSubComNum, data.ucStatusBit)
        dictPost2BS = {}
        for i in range(data.MsgContent.ucExceptEentNums):
            if data.MsgContent.tInfoData[i].ucExceptCameraId == 0:
                continue
            msg_data = struct.pack('>I', int(data.MsgContent.tInfoData[i].uiExceptId)) + \
                       struct.pack('B', int(data.MsgContent.tInfoData[i].ucExceptCameraId)) + \
                       struct.pack('B', int(data.MsgContent.tInfoData[i].ucExceptOfStation))
            temp_msg_len = len(msg_data)
            check_data = head_data + struct.pack('>H', temp_msg_len) + msg_data
            check_bit = getBccCheckVal(check_data)
            pack_data = check_data + struct.pack('B', check_bit) + struct.pack('B', data.ucCutoffBit)
            dictPost2BS[data.MsgContent.tInfoData[i].ucExceptOfStation] = pack_data
        return dictPost2BS

    # dstData转换为B0帧数据内容
    def ConversionB0MsgContent(self, tMsgContent: list, usMsgLen: int, nFrameId: int):
        # test_file = open("B0data.txt", 'a')
        # test_file.write("=======================FramedId: " + str(nFrameId) + "=======================\n")
        lB0TransientData = B0TransientEvent()
        lB0TransientData.usBeginBit = 0xffff
        lB0TransientData.ucSerialNum = 0x01
        lB0TransientData.ucCommandNum = 0xb0
        lB0TransientData.ucSubComNum = 0
        lB0TransientData.ucStatusBit = 0
        lB0TransientData.usMsgLen = usMsgLen
        lB0TransientData.MsgContent = None
        lB0TransientData.ucCheck = 0
        lB0TransientData.ucCutoffBit = 0xff

        lMsgData1 = B0MsgContent()
        lMsgData1.usEquiqId = tMsgContent[0]
        lMsgData1.usReserve1 = tMsgContent[1]
        time_stramp_temp1, time_stramp_temp2 = parse_GPS_time()
        lMsgData1.uiTimeStamp1 = time_stramp_temp1
        lMsgData1.uiTimeStamp2 = time_stramp_temp2
        lMsgData1.usCarChannelNum = tMsgContent[3]
        lMsgData1.usReserve2 = tMsgContent[4]
        lMsgData1.usReserve3 = tMsgContent[6]

        lChDataList = []
        for k in range(lMsgData1.usCarChannelNum):
            lChDataList.append(B0ChannelData())
        for i in  range(lMsgData1.usCarChannelNum):
            lChDataList[i].usChannelId = tMsgContent[5][i][0]
            lChDataList[i].usChannelNum = tMsgContent[5][i][1]
            lChDataList[i].usChannelDirection = tMsgContent[5][i][2]
            lChDataList[i].ucChannelLineDir = tMsgContent[5][i][3]
            lChDataList[i].ucChannelType = tMsgContent[5][i][4]
            lChDataList[i].usChannelSpeedLimit = tMsgContent[5][i][5]
            lChDataList[i].usChannelReservel1 = tMsgContent[5][i][6]
            lChDataList[i].uiQueueLength = tMsgContent[5][i][7]
            lChDataList[i].uiSpcaeOccupancy = tMsgContent[5][i][8]
            lChDataList[i].ucCarNum = tMsgContent[5][i][9]
            lChDataList[i].uc3ChannelReservel2 = (0,0,0)
            if lChDataList[i].ucCarNum < 2:
                lChDataList[i].uiCarHeadDis = []
                # test_file.write(
                #     "CarChannel[" + str(i) + "].uiCarHeadDis: " + str(lChDataList[i].uiCarHeadDis) + '\n')
            else:
                for j in range(lChDataList[i].ucCarNum - 1):
                    if tMsgContent[5][i][11][j] < 0:
                        lChDataList[i].uiCarHeadDis.append(0)
                    else:
                        lChDataList[i].uiCarHeadDis.append(tMsgContent[5][i][11][j])

            lChDataList[i].usChannelReservel3 = tMsgContent[5][i][12]
            lMsgData1.tChannelData.append(lChDataList[i])

        lB0TransientData.MsgContent = lMsgData1
        #test_file.close()
        return lB0TransientData

    #dstData转换为B1帧数据内容
    def ConversionB1MsgContent(self, tMsgContent: list, usMsgLen: int, nFrameId: int):
        lB1TransientData = B1TransientEvent()
        #test_file = open("B1data.txt", "a")
        #test_file.write("=======================FramedId: " + str(nFrameId) + "=======================\n")
        lB1TransientData.usBeginBit = 0xffff
        lB1TransientData.ucSerialNum = 0x01
        lB1TransientData.ucCommandNum = 0xb1
        lB1TransientData.ucSubComNum = 0
        lB1TransientData.usMsgLen = usMsgLen
        lB1TransientData.MsgContent = None
        lB1TransientData.ucCheck = 0
        lB1TransientData.ucCutoffBit = 0xff

        lB1MsgData = B1MsgContent()
        lB1MsgData.usEquiqId = tMsgContent[0]
        lB1MsgData.usReserve1 = tMsgContent[1]

        lB1MsgData.uiStartTimeStamp1 = self.uiStartTimeStamp1
        lB1MsgData.uiStartTimeStamp2 = 0
        self.uiStartTimeStamp1, self.uiStartTimeStamp2 = parse_GPS_time()
        self.uiStartTimeStamp1 = self.uiStartTimeStamp1 - (self.uiStartTimeStamp1 % 60)
        lB1MsgData.uiEndTimeStamp1 = self.uiStartTimeStamp1
        lB1MsgData.uiEndTimeStamp2 = 0
        lB1MsgData.usCarChannelNum = tMsgContent[4]
        lB1MsgData.usReserve2 = tMsgContent[5]

        lChDataList = []
        for k in range(lB1MsgData.usCarChannelNum):
            lChDataList.append(B1ChannelData())
        #lChDataList = [B1ChannelData()] * lB1MsgData.usCarChannelNum
        for i in range(lB1MsgData.usCarChannelNum):
            lChDataList[i].usChannelId = tMsgContent[6][i][0]
            lChDataList[i].usChannelNum = tMsgContent[6][i][1]
            lChDataList[i].usChannelDirection = tMsgContent[6][i][2]
            lChDataList[i].ucChannelLineDir = tMsgContent[6][i][3]
            lChDataList[i].ucChannelType = tMsgContent[6][i][4]
            lChDataList[i].usChannelSpeedLimit = tMsgContent[6][i][5]
            lChDataList[i].ucCongestionStatus = tMsgContent[6][i][6]
            lChDataList[i].usTraffic = tMsgContent[6][i][7]
            lChDataList[i].usTimeShare = tMsgContent[6][i][8]
            lChDataList[i].usCar = tMsgContent[6][i][9]
            lChDataList[i].usTruck = tMsgContent[6][i][10]
            lChDataList[i].usBus = tMsgContent[6][i][11]
            lChDataList[i].usMotorbike = tMsgContent[6][i][12]
            lChDataList[i].usChannelReservel1 = tMsgContent[6][i][13]
            lB1MsgData.tB1ChannelData.append(lChDataList[i])
        #test_file.close()
        lB1TransientData.MsgContent = lB1MsgData
        return lB1TransientData

        # dstData转换为B2帧数据内容

    # dstData转换为B2帧数据内容
    def ConversionB2MsgContent(self, tMsgContent: list, usMsgLen: int, nFrameId: int):

        lB2TransientData = B2TrafficIncident()
        # test_file = open("B1data.txt", "a")
        # test_file.write("=======================FramedId: " + str(nFrameId) + "=======================\n")
        lB2TransientData.usBeginBit = 0xffff
        lB2TransientData.ucSerialNum = 0x01
        lB2TransientData.ucCommandNum = 0xb2
        lB2TransientData.ucSubComNum = 0
        lB2TransientData.usMsgLen = usMsgLen
        lB2TransientData.MsgContent = None
        lB2TransientData.ucCheck = 0
        lB2TransientData.ucCutoffBit = 0xff


        lB2MsgData = B2MsgContent()
        lB2MsgData.usEquiqId = tMsgContent[0]
        lB2MsgData.usReserve1 = tMsgContent[1]
        time1, time2 = parse_GPS_time()
        lB2MsgData.uiStartTimeStamp1 = time1
        lB2MsgData.uiStartTimeStamp2 = time2
        lB2MsgData.ExceptEentNumbers = tMsgContent[3]
        lB2MsgData.usReserve2 = tMsgContent[4]
        for i in range(lB2MsgData.ExceptEentNumbers):
            lB2MsgData.tTargetMsg.append(B2ExceptEventMsg())
            lB2MsgData.tTargetMsg[i].usExceptId = tMsgContent[5][i][0] % 65534
            lB2MsgData.tTargetMsg[i].usExceptType = tMsgContent[5][i][1]
            lB2MsgData.tTargetMsg[i].uiCameraNumbers = tMsgContent[5][i][2]
            for j in range(lB2MsgData.tTargetMsg[i].uiCameraNumbers):
                lB2MsgData.tTargetMsg[i].uiCameraIp.append(tMsgContent[5][i][3][j])
            lB2MsgData.tTargetMsg[i].uiExceptBeginTime1 = tMsgContent[5][i][4][0]
            lB2MsgData.tTargetMsg[i].uiExceptBeginTime2 = tMsgContent[5][i][4][1]
            lB2MsgData.tTargetMsg[i].u2iExceptViolationTime = tMsgContent[5][i][5]
            lB2MsgData.tTargetMsg[i].usReservel1 = tMsgContent[5][i][6]

        '''
        index = 0
        offset = 0
        for i in range(16):
            if i==0:
                lB2MsgData.tTargetMsg.append(B2GeneralMsg())
                lB2MsgData.tTargetMsg[i].usTargetNumber = tMsgContent[3 + index]
                lB2MsgData.tTargetMsg[i].usReservel1 = tMsgContent[4 + index]
                if lB2MsgData.tTargetMsg[i].usTargetNumber > 0:
                    for j in range(lB2MsgData.tTargetMsg[i].usTargetNumber):
                        lB2MsgData.tTargetMsg[i].ulTargetInfo.append(B2ViolationData())
                        lB2MsgData.tTargetMsg[i].ulTargetInfo[j].usViolationId = tMsgContent[5 + index][j][0]
                        lB2MsgData.tTargetMsg[i].ulTargetInfo[j].uiCameraNumbers = tMsgContent[5 + index][j][1]
                        for k in range((lB2MsgData.tTargetMsg[i].ulTargetInfo[j].uiCameraNumbers)):
                            lB2MsgData.tTargetMsg[i].ulTargetInfo[j].uiCameraIp.append(tMsgContent[5 + index][j][2][k])
                        lStartTimeStamp1, lStartTimeStamp2 = parse_GPS_time()
                        lB2MsgData.tTargetMsg[i].ulTargetInfo[j].uiRetrogradeBeginTime1 = tMsgContent[5 + index][j][3][0]
                        lB2MsgData.tTargetMsg[i].ulTargetInfo[j].uiRetrogradeBeginTime2 = tMsgContent[5 + index][j][3][1]
                        lB2MsgData.tTargetMsg[i].ulTargetInfo[j].uiViolationTime = int(tMsgContent[5 + index][j][4] *100)
                        lB2MsgData.tTargetMsg[i].ulTargetInfo[j].usReservel1 = tMsgContent[5 + index][j][5]

                        index = index + 4
                else:
                    index = index + 4
                lB2MsgData.tTargetMsg[i].usReservel2 = tMsgContent[6 + index - 4]
            else:
                lB2MsgData.tTargetMsg.append(B2GeneralMsg())
                lB2MsgData.tTargetMsg[i].usTargetNumber = tMsgContent[3 + index ]
                lB2MsgData.tTargetMsg[i].usReservel1 = tMsgContent[4 + index ]
                if lB2MsgData.tTargetMsg[i].usTargetNumber > 0:
                    for j in range(lB2MsgData.tTargetMsg[i].usTargetNumber):
                        lB2MsgData.tTargetMsg[i].ulTargetInfo.append(B2GeneralTargetMsg())
                        lB2MsgData.tTargetMsg[i].ulTargetInfo[j].usRetrogradeId = tMsgContent[5 + index ][j][0]
                        lB2MsgData.tTargetMsg[i].ulTargetInfo[j].uiCameraNumbers = tMsgContent[5 + index ][j][1]
                        for k in range((lB2MsgData.tTargetMsg[i].ulTargetInfo[j].uiCameraNumbers)):
                            lB2MsgData.tTargetMsg[i].ulTargetInfo[j].uiCameraIp.append(tMsgContent[5 + index][j][2][k])
                        lStartTimeStamp1, lStartTimeStamp2 = parse_GPS_time()
                        lB2MsgData.tTargetMsg[i].ulTargetInfo[j].uiRetrogradeBeginTime1 = tMsgContent[5 + index][j][3][0]
                        lB2MsgData.tTargetMsg[i].ulTargetInfo[j].uiRetrogradeBeginTime2 = tMsgContent[5 + index][j][3][1]
                        lB2MsgData.tTargetMsg[i].ulTargetInfo[j].usReservel1 = tMsgContent[5 + index][j][4]

                    index = index + 4
                else:
                    index = index +4
                lB2MsgData.tTargetMsg[i].usReservel2 = tMsgContent[6 + index - 4]
        '''
        '''
        if tMsgContent[3] == -1:
            lB2MsgData.usCarViolationsNumber = 0
        else:
            lB2MsgData.usCarViolationsNumber = tMsgContent[3]

        if tMsgContent[4] == -1:
            lB2MsgData.usReserve2 = 0
        else:
            lB2MsgData.usReserve2 = tMsgContent[4]

        lViDataList = []
        for k in range(lB2MsgData.usCarViolationsNumber):
            lViDataList.append(B2ViolationData())
        # lChDataList = [B1ChannelData()] * lB1MsgData.usCarChannelNum
        for i in range(lB2MsgData.usCarViolationsNumber):
            if tMsgContent[5][i][0] == -1:
                lViDataList[i].usViolationId = i + 1
            else:
                lViDataList[i].usViolationId = tMsgContent[5][i][0] % 65534
            if tMsgContent[5][i][1] == -1:
                lViDataList[i].uiCameraIp = [0, 0, 0, 0]
            else:
                lViDataList[i].uiCameraIp = self.listCameraIp[tMsgContent[5][i][1]].split('.')

            time1, time2 = parse_GPS_time()
            if tMsgContent[5][i][2]:
                lViDataList[i].uiViolationBeginTime1 = tMsgContent[5][i][2][0]
                lViDataList[i].uiViolationBeginTime2 = tMsgContent[5][i][2][1]
            else:
                lViDataList[i].uiViolationBeginTime1 = time1
                lViDataList[i].uiViolationBeginTime2 = time2
            if tMsgContent[5][i][3]:
                lViDataList[i].uiViolationTime1 = tMsgContent[5][i][3]
                lViDataList[i].uiViolationTime2 = tMsgContent[5][i][3] - math.floor(tMsgContent[5][i][3])
            else:
                lViDataList[i].uiViolationTime1 = time1
                lViDataList[i].uiViolationTime2 = time2
            if tMsgContent[5][i][4] > 0:
                lViDataList[i].usReservel1 = tMsgContent[5][i][4]
            else:
                lViDataList[i].usReservel1 = 0
            lB2MsgData.tB2ViolationData.append(lViDataList[i])

        if tMsgContent[6] == -1:
            lB2MsgData.usReserve3 = 0
        else:
            lB2MsgData.usReserve3 = tMsgContent[6]
        if tMsgContent[7] != -1:
            lB2MsgData.usNoCarRetrogradeNumber = tMsgContent[7]
        else:
            lB2MsgData.usNoCarRetrogradeNumber = []
        if tMsgContent[8] == -1:
            lB2MsgData.usReserve4 = 0
        else:
            lB2MsgData.usReserve4 = tMsgContent[8]

        lReDataList = []
        for k in range(lB2MsgData.usNoCarRetrogradeNumber):
            lReDataList.append(B2RetrogradeData())
        for j in range(lB2MsgData.usNoCarRetrogradeNumber):
            if tMsgContent[9][j][0] < 0 :
                lReDataList[j].usRetrogradeId = j + 1
            else:
                lReDataList[j].usRetrogradeId = tMsgContent[9][j][0] % 65534
                if tMsgContent[9][j][0] > 65500:
                    print("lReDataList[j].usRetrogradeId: ",lReDataList[j].usRetrogradeId)
            if tMsgContent[9][j][1] == -1:
                lReDataList[j].uiCameraIp = [0, 0, 0, 0]
            else:
                lReDataList[j].uiCameraIp = self.listCameraIp[tMsgContent[9][j][1]].split('.')
            if tMsgContent[9][j][2]:
                lReDataList[j].uiRetrogradeBeginTime1 = tMsgContent[9][j][2][0]
                lReDataList[j].uiRetrogradeBeginTime2 = tMsgContent[9][j][2][1]
            else:
                lReDataList[j].uiRetrogradeBeginTime1 = 0
                lReDataList[j].uiRetrogradeBeginTime2 = 0
            if tMsgContent[9][j][3] != -1:
                lReDataList[j].usReservel1 = tMsgContent[9][j][3]
            else:
                lReDataList[j].usReservel1 = 0
            lB2MsgData.tB2RetrogradeData.append(lReDataList[j])
        if tMsgContent[10] == -1:
            lB2MsgData.usReserve5 = 0
        else:
            lB2MsgData.usReserve5 = tMsgContent[10]
        # test_file.close()
        '''
        lB2TransientData.MsgContent = lB2MsgData
        return lB2TransientData

    # dstData转换为B3帧数据内容
    def ConversionB3MsgContent(self, tMsgContent: list, usMsgLen: int, nFrameId: int):
        # test_file = open("B3data.txt", 'a')
        # test_file.write("=======================FramedId: " + str(nFrameId) + "=======================\n")
        lB3TransientData = B3TransientEvent()
        lB3TransientData.usBeginBit = 0xffff
        lB3TransientData.ucSerialNum = 0x01
        lB3TransientData.ucCommandNum = 0xb3
        lB3TransientData.ucSubComNum = 0
        lB3TransientData.ucStatusBit = 0
        lB3TransientData.usMsgLen = usMsgLen
        lB3TransientData.MsgContent = None
        lB3TransientData.ucCheck = 0
        lB3TransientData.ucCutoffBit = 0xff

        lMsgData3 = B3MsgContent()
        lMsgData3.usEquiqId = tMsgContent[0]
        lMsgData3.usReserve1 = tMsgContent[1]
        time_stramp_temp1, time_stramp_temp2 = parse_GPS_time()
        lMsgData3.uiTimeStampS = time_stramp_temp1
        lMsgData3.uiTimeStampUs = time_stramp_temp2
        lMsgData3.uiFrameId = tMsgContent[4]
        lMsgData3.ucCarChannelNums = tMsgContent[5]
        lMsgData3.u3cReserve2 = (0,0,0)
        lMsgData3.uiReserve3 = tMsgContent[8]

        lChDataList = []
        for i in range(lMsgData3.ucCarChannelNums):
            lChDataList.append(B3ChannelData())
            lChDataList[i].ucChannelId = tMsgContent[7][i][0] % 255
            lChDataList[i].ucChannelType = tMsgContent[7][i][1]
            lChDataList[i].usChannelDirection = tMsgContent[7][i][2]
            lChDataList[i].ucChannelSpeedLimit = tMsgContent[7][i][3]
            lChDataList[i].ucMeanSpeed = tMsgContent[7][i][4]
            lChDataList[i].uiQueueLength = tMsgContent[7][i][5]
            if tMsgContent[7][i][6] < 0:
                lChDataList[i].usChannelInterval = 0
            else:
                lChDataList[i].usChannelInterval = tMsgContent[7][i][6]
            lChDataList[i].ucSpcaeOccupancy = tMsgContent[7][i][7]
            lChDataList[i].ucReservel1 = tMsgContent[7][i][8]
            lChDataList[i].usMotorVehicles = tMsgContent[7][i][9]
            lChDataList[i].usCarNum = tMsgContent[7][i][10]
            lChDataList[i].usTruckFreight = tMsgContent[7][i][11]
            lChDataList[i].usBigBus = tMsgContent[7][i][12]
            lChDataList[i].usMediumBus = tMsgContent[7][i][13]
            lChDataList[i].usTankCar = tMsgContent[7][i][14]
            lChDataList[i].usPeople = tMsgContent[7][i][15]
            lChDataList[i].usNoMotorVehicles = tMsgContent[7][i][16]
            lChDataList[i].uiReservel2 = tMsgContent[7][i][17]

        lMsgData3.tInfoData = lChDataList

        lB3TransientData.MsgContent = lMsgData3
        # test_file.close()
        return lB3TransientData

    # dstData转换为B4帧数据内容
    def ConversionB4MsgContent(self, tMsgContent: list, usMsgLen: int, nFrameId: int):
        lB4TransientData = B4TransientEvent()
        # test_file = open("B4data.txt", "a")
        # test_file.write("=======================FramedId: " + str(nFrameId) + "=======================\n")
        lB4TransientData.usBeginBit = 0xffff
        lB4TransientData.ucSerialNum = 0x01
        lB4TransientData.ucCommandNum = 0xb4
        lB4TransientData.ucSubComNum = 0
        lB4TransientData.ucStatusBit = 0
        lB4TransientData.usMsgLen = usMsgLen
        lB4TransientData.MsgContent = None
        lB4TransientData.ucCheck = 0
        lB4TransientData.ucCutoffBit = 0xff

        lB4MsgData = B4MsgContent()
        lB4MsgData.usEquiqId = tMsgContent[0]
        lB4MsgData.usReserve1 = tMsgContent[1]

        lB4MsgData.uiStartTimeStampS = self.uiStartTimeStamp1
        lB4MsgData.uiStartTimeStampUs = 0
        self.uiStartTimeStamp1, self.uiStartTimeStamp2 = parse_GPS_time()
        self.uiStartTimeStamp1 = self.uiStartTimeStamp1 - (self.uiStartTimeStamp1 % 60)
        lB4MsgData.uiEndTimeStampS = self.uiStartTimeStamp1
        lB4MsgData.uiEndTimeStampUs = 0
        lB4MsgData.uiFrameId = tMsgContent[6]
        lB4MsgData.ucCarChannelNums = tMsgContent[7]
        lB4MsgData.u3cReserve2 = (0,0,0)

        lChDataList = []
        for i in range(lB4MsgData.ucCarChannelNums):
            lChDataList.append(B4ChannelData())
        # lChDataList = [B4ChannelData()] * lB4MsgData.usCarChannelNum
        for i in range(lB4MsgData.ucCarChannelNums):
            if tMsgContent[9][i][0] < 0 or tMsgContent[9][i][0] > 255:
                lChDataList[i].ucChannelId = 0
            else:
                lChDataList[i].ucChannelId = tMsgContent[9][i][0]
            lChDataList[i].ucChannelType = tMsgContent[9][i][1]
            lChDataList[i].usChannelDirection = tMsgContent[9][i][2]
            lChDataList[i].ucChannelSpeedLimit = tMsgContent[9][i][3]
            lChDataList[i].ucStationNums = tMsgContent[9][i][4]
            lChDataList[i].usReservel1 = tMsgContent[9][i][5]

            lStationList = []
            for j in range(lChDataList[i].ucStationNums):
                lStationList.append(B4StationData())
            for j in range(lChDataList[i].ucStationNums):
                lStationList[j].ucStationId = tMsgContent[9][i][6][j][0]
                lStationList[j].ucCongestion = tMsgContent[9][i][6][j][1]
                lStationList[j].usReservel = tMsgContent[9][i][6][j][2]
                lChDataList[i].tStationData.append(lStationList[j])

            lChDataList[i].ucCarfollowpercen = tMsgContent[9][i][7]
            lChDataList[i].ucTimeOccupancy = tMsgContent[9][i][8]
            if tMsgContent[9][i][9] < 0:
                lChDataList[i].usCarHeadDis = 0
            else:
                lChDataList[i].usCarHeadDis = tMsgContent[9][i][9]
            lChDataList[i].usCarFlow = tMsgContent[9][i][10]
            lChDataList[i].usTruckFreightFlow = tMsgContent[9][i][11]
            lChDataList[i].usBigBusFlow = tMsgContent[9][i][12]
            lChDataList[i].usMediumBusFlow = tMsgContent[9][i][13]
            lChDataList[i].usTankCarFlow = tMsgContent[9][i][14]
            lChDataList[i].ucCarAverSpeed = tMsgContent[9][i][15]
            lChDataList[i].ucTruckFreightAverSpeed = tMsgContent[9][i][16]
            lChDataList[i].ucBigBusAverSpeed = tMsgContent[9][i][17]
            lChDataList[i].ucMediumBusAverSpeed = tMsgContent[9][i][18]
            lChDataList[i].ucTankCarAverSpeed = tMsgContent[9][i][19]
            lChDataList[i].ucReservel2 = tMsgContent[9][i][20]

            lB4MsgData.tInfoData.append(lChDataList[i])
        # test_file.close()
        lB4TransientData.MsgContent = lB4MsgData
        return lB4TransientData

    # dstData转换为B5帧数据内容
    def ConversionB5MsgContent(self, tMsgContent: list, usMsgLen: int, nFrameId: int):

        lB5TransientData = B5TransientEvent()
        # test_file = open("B1data.txt", "a")
        # test_file.write("=======================FramedId: " + str(nFrameId) + "=======================\n")
        lB5TransientData.usBeginBit = 0xffff
        lB5TransientData.ucSerialNum = 0x01
        lB5TransientData.ucCommandNum = 0xb5
        lB5TransientData.ucSubComNum = 0
        lB5TransientData.usMsgLen = usMsgLen
        lB5TransientData.MsgContent = None
        lB5TransientData.ucCheck = 0
        lB5TransientData.ucCutoffBit = 0xff

        lB5MsgData = B5MsgContent()
        lB5MsgData.usEquiqId = tMsgContent[0]
        lB5MsgData.usReserve1 = tMsgContent[1]

        time1, time2 = parse_GPS_time()
        lB5MsgData.uiStartTimeStampS = time1
        lB5MsgData.uiStartTimeStampUs = time2
        lB5MsgData.ucExceptEentNums = tMsgContent[4]
        lB5MsgData.usReserve2 = tMsgContent[5]
        lB5MsgData.uiReserve3 = tMsgContent[7]
        for i in range(lB5MsgData.ucExceptEentNums):
            lB5MsgData.tInfoData.append(B5ExceptEventMsg())
            lB5MsgData.tInfoData[i].uiExceptId = tMsgContent[6][i][0]
            lB5MsgData.tInfoData[i].ucExceptType = tMsgContent[6][i][1]
            if tMsgContent[6][i][2] < 0 or tMsgContent[6][i][2] > 255:
                lB5MsgData.tInfoData[i].ucExceptOfStation = 0
            else:
                lB5MsgData.tInfoData[i].ucExceptOfStation = tMsgContent[6][i][2]
            if tMsgContent[6][i][3] < 0 or tMsgContent[6][i][3] > 255:
                lB5MsgData.tInfoData[i].ucExceptOfChannel = 0
            else:
                lB5MsgData.tInfoData[i].ucExceptOfChannel = tMsgContent[6][i][3]
            lB5MsgData.tInfoData[i].ucExceptTargetNums = tMsgContent[6][i][4]
            for j in range(lB5MsgData.tInfoData[i].ucExceptTargetNums):
                lB5MsgData.tInfoData[i].ExceptTarget.append(B5ExceptTargetData())
                lB5MsgData.tInfoData[i].ExceptTarget[j].usTargetId = tMsgContent[6][i][5][j][0]
                lB5MsgData.tInfoData[i].ExceptTarget[j].usReservel = tMsgContent[6][i][5][j][1]
            lB5MsgData.tInfoData[i].uiExceptLong = tMsgContent[6][i][6]
            lB5MsgData.tInfoData[i].uiExceptLat = tMsgContent[6][i][7]
            # time_stramp_temp1, time_stramp_temp2 = parse_GPS_time()
            lB5MsgData.tInfoData[i].uiExceptBeginTimeS = tMsgContent[6][i][8]
            lB5MsgData.tInfoData[i].uiExceptBeginTimeUs = tMsgContent[6][i][9]
            lB5MsgData.tInfoData[i].usReservel = tMsgContent[6][i][10]
            lB5MsgData.tInfoData[i].ucExceptCameraId = tMsgContent[6][i][-1]

        lB5TransientData.MsgContent = lB5MsgData
        return lB5TransientData


    def slotTimerOut_old(self, stDstData):
        try:
            if self.timeB1.elapsed() < 1:
                self.timeB1.start()
                self.uiStartTimeStamp1, self.uiStartTimeStamp2 = parse_GPS_time()
                self.uiStartTimeStamp1 = self.uiStartTimeStamp1 - (self.uiStartTimeStamp1 % 60) + 28800
            # if self.stParam.bPause == 1:
            #     self.sigFileReadOver.emit()
            self.stDstData = stDstData
            if stDstData is None:
                MyLog.writeLog("{}[:{}] - Data processing error!...".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                   sys._getframe().f_lineno), LOG_ERROR, ENABLE_WRITE_LOG, ENABLE_PRINT)
                return 0, 0  # 表示数据有误

            self.bFirstNoData = True
            if stUiParam.bRotatingLane:
                self.listBoxInfo = copy.deepcopy(self.stDstData.listBoxInfo)
            else:
                self.listBoxInfo = copy.deepcopy(self.stDstData.listBaseBoxInfo)

            # tt = time.time()
            for boxInfoTemp in self.listBoxInfo:
                if boxInfoTemp.boxLongitude == "None":
                    boxInfoTemp.boxLongitude, boxInfoTemp.boxLatitude = XYZ_To_BLH(self.stParam.getLidarLongitude(0),
                                                                                    self.stParam.getLidarLatitude(0),
                                                                                    self.stBoxInfo.boxCenter[0],
                                                                                    self.stBoxInfo.boxCenter[1],
                                                                                    self.stParam.getAngleNorthT(0))

            # t1 = time.time()
            self.stParam.setFrameId(self.stDstData.nFrameId)
            try:
                # udp转发识别结果
                bSendResult = self.stParam.getSendResult()
                listForwardDstDevTemp = self.stParam.getListForwardDstDev()
                byteSaveResult = None
                if bSendResult and (len(listForwardDstDevTemp) > 0) and self.stDstData.pcTrackersCov.shape[0] > 0 and len(self.stDstData.pcTrackersCov.shape) == 2:
                    listAllBaseBoxInfo = self.stDstData.listBaseBoxInfo
                    information = getHeadInfo(listAllBaseBoxInfo)
                    trans_nums = getBoxNums(listAllBaseBoxInfo)
                    logti_t = self.stParam.getLidarLongitude(0)
                    lanti_t = self.stParam.getLidarLatitude(0)
                    angle_north_t = self.stParam.getAngleNorthT(0)
                    object_pack_ll = packetBoxInfo(listAllBaseBoxInfo, logti_t, lanti_t, angle_north_t)

                    objects_all = bytes()
                    if len(object_pack_ll) > 0:
                        for i in range(len(object_pack_ll)):
                            objects_all += object_pack_ll[i]
                    time_stamp = self.stDstData.tTimeStamp
                    time_stamp = time.time()
                    time_stamp_temp1, time_stamp_temp2 = math.floor(time_stamp), int((time_stamp - math.floor(time_stamp)) * 1e6)
                    time_pack = struct.pack('>I', time_stamp_temp1) + struct.pack('>I', time_stamp_temp2)  # 从1900年开始的秒数+微秒数
                    head_pack = struct.pack('BB', self.stDstData.nEqupId[0],self.stDstData.nEqupId[1]) + \
                                struct.pack('>H', int(self.stDstData.trunAlg)) + \
                                struct.pack('>2BH',0,0,int(self.stDstData.nFrameId % 65000)) + \
                                time_pack + \
                                struct.pack('>I', int(logti_t * 1e7)) + \
                                struct.pack('>I', int(lanti_t * 1e7)) + \
                                struct.pack('>H', int(round(angle_north_t)))
                    send_data = information + head_pack + trans_nums + objects_all + struct.pack('>I', 0) + struct.pack('B', 0) + struct.pack('B', 255)
                    #----------------------test----------------------#
                    if stUiParam.bSaveE1 == 1 and my_mkdir('./savedata/'):
                        if byteSaveResult is None:
                            byteSaveResult = send_data
                            with open(r'./savedata/ResultData.txt','a') as log_file:
                                tSend = time.time()
                                strTimeStamp = getTimeStamp(tSend)
                                hexData = binascii.b2a_hex(byteSaveResult).decode('utf-8')
                                t = hexData.upper()
                                hexData = ' '.join([t[2*i:2*(i+1)] for i in range(int(len(t)/2))])
                                log_file.write(strTimeStamp + ": " + str(hexData) + "\n")
                    #----------------------test----------------------#
                    # 转发识别结果
                    for i in range(len(listForwardDstDevTemp)):
                        strDev = listForwardDstDevTemp[i]
                        if not strDev[1]:
                            continue

                        nameList = strDev[0].split('-')  # 分割字符串
                        strDevIp = nameList[0]
                        nDevPort = int(nameList[1])
                        try:
                            self.udpSocket.sendto(send_data, (strDevIp, nDevPort))
                            # print("send result to:", strDevIp, " ", nDevPort)
                        except:
                            print(traceback.format_exc())
                            MyLog.writeLog("{}[:{}] - {} is unreachable!".
                                           format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                  sys._getframe().f_lineno, strDevIp),
                                           LOG_ERROR, ENABLE_WRITE_LOG, ENABLE_PRINT)
                # MyLog.writeLog("{}[:{}] - Forward Event Start!\n".format(__file__.split('/')[len(__file__.split('/')) - 1],
                #                                                      sys._getframe().f_lineno), LOG_ERROR, ENABLE_WRITE_LOG, True)
                # 转发车流量瞬时事件和统计事件信息
                listReachableIpPort = []
                EventData = ([], [], [], [], [], [])
                nEventFeedBackPort = self.stParam.getEventFeedBackPort()
                if self.dstEventResultQueue.qsize() > 0:
                    try:
                        EventData = self.dstEventResultQueue.get(True,0.01)
                    except:
                        pass
                listForwardEventDstDevTemp = self.stParam.getListForwardEventDstDev()
                for i in range(len(listForwardEventDstDevTemp)):
                    strDev = listForwardEventDstDevTemp[i]
                    if not strDev[1]:
                        continue

                    nameList = strDev[0].split('-')  # 分割字符串
                    strDevIp = nameList[0]
                    nDevPort = int(nameList[1])
                    listReachableIpPort.append([strDevIp, nDevPort])

                    if len(EventData) > 0:
                        if not self.stParam.getHighSpeed():
                            if EventData[0]:
                                B0SendData = self.B0_pcak(
                                    self.ConversionB0MsgContent(EventData[0], len(EventData[0]), stDstData.nFrameId))
                                if B0SendData != None:
                                    self.udpSocket.sendto(B0SendData, (strDevIp, nDevPort))
                            if EventData[1]:
                                B1SendData = self.B1_pcak(
                                    self.ConversionB1MsgContent(EventData[1], len(EventData[1]), stDstData.nFrameId))
                                if B1SendData != None:
                                    self.udpSocket.sendto(B1SendData, (strDevIp, nDevPort))
                            if EventData[2]:
                                B2SendData = self.B2_pcak(
                                    self.ConversionB2MsgContent(EventData[2], len(EventData[2]), stDstData.nFrameId))
                                if B2SendData != None:
                                    self.udpSocket.sendto(B2SendData, (strDevIp, nDevPort))
                        else:
                            if EventData[3]:
                                B3SendData = self.B3_pcak(
                                    self.ConversionB3MsgContent(EventData[3], len(EventData[3]), stDstData.nFrameId))
                                if B3SendData != None:
                                    self.udpSocket.sendto(B3SendData, (strDevIp, nDevPort))
                                    if stUiParam.bSaveEvent == 1 and my_mkdir('./savedata/Eventlog/'):
                                        with open(r'./savedata/Eventlog/B3send.log','a') as log_file:
                                            tEvent = time.time()
                                            strTimeStamp = getTimeStamp(tEvent)
                                            log_file.write(strTimeStamp + " - " + str(len(B3SendData)) + "\n")
                            if EventData[4]:
                                B4SendData = self.B4_pcak(
                                    self.ConversionB4MsgContent(EventData[4], len(EventData[4]), stDstData.nFrameId))
                                if B4SendData != None:
                                    self.udpSocket.sendto(B4SendData, (strDevIp, nDevPort))
                                    if stUiParam.bSaveEvent == 1 and my_mkdir('./savedata/Eventlog/'):
                                        with open(r'./savedata/Eventlog/B4send.log','a') as log_file:
                                            tEvent = time.time()
                                            strTimeStamp = getTimeStamp(tEvent)
                                            log_file.write(strTimeStamp + " - " + str(len(B4SendData)) + "\n")
                            if len(EventData[5]) != 0 and EventData[5][4] != 0:
                                stB5Msg = self.ConversionB5MsgContent(EventData[5], len(EventData[5]), stDstData.nFrameId)
                                dictPost2BS = self.getPost2BS(stB5Msg)
                                B5SendData = self.B5_pcak(stB5Msg)
                                if B5SendData != None:
                                    self.udpSocket.sendto(B5SendData, (strDevIp, nDevPort))
                                    if stUiParam.bSaveEvent == 1 and my_mkdir('./savedata/Eventlog/'):
                                        with open(r'./savedata/Eventlog/B5send.log','a') as log_file:
                                            tEvent = time.time()
                                            strTimeStamp = getTimeStamp(tEvent)
                                            log_file.write(strTimeStamp + " - " + str(len(B5SendData)) + "\n")
                                for nBSId in dictPost2BS:
                                    self.udpSocket.sendto(dictPost2BS[nBSId], (self.stParam.getStationSrcIp(self.listBaseStationId.index(nBSId)), nEventFeedBackPort))
                            MyLog.writeLog("{}[:{}] - Forward Event Sucessful!\n".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                                 sys._getframe().f_lineno), LOG_INFO, ENABLE_WRITE_LOG, True)
            except:
                print(traceback.format_exc())
                MyLog.writeLog("{}[:{}] - Forward error!\n{}".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                     sys._getframe().f_lineno,traceback.format_exc()), LOG_ERROR, ENABLE_WRITE_LOG, True)
            self.updateGL()

            # print("timeout = %d ms" % self.timeTemp.elapsed())
            # self.timeTemp.restart()
        except:
            print(traceback.format_exc())
        return self.stDstData.nFrameId

    def slotTimerOut(self, stDstData):
        try:
            if self.thisTimeStamp is None:
                self.timeB1.start()
                self.uiStartTimeStamp1, self.uiStartTimeStamp2 = parse_GPS_time()
                self.uiStartTimeStamp1 = self.uiStartTimeStamp1 - (self.uiStartTimeStamp1 % 60)
                thisTime = math.floor(time.time())
                thisTime = thisTime - (thisTime % 60)
                self.thisTimeStamp = getTimeStamp(thisTime)
            # if self.stParam.bPause == 1:
            #     self.sigFileReadOver.emit()
            self.stDstData = stDstData
            if stDstData is None:
                MyLog.writeLog("{}[:{}] - Data processing error!...".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                            sys._getframe().f_lineno), LOG_ERROR, ENABLE_WRITE_LOG, ENABLE_PRINT)
                return 0, 0  # 表示数据有误

            self.bFirstNoData = True
            if stUiParam.bRotatingLane:
                self.listBoxInfo = copy.deepcopy(self.stDstData.listBoxInfo)
            else:
                self.listBoxInfo = copy.deepcopy(self.stDstData.listBaseBoxInfo)

            # tt = time.time()
            for boxInfoTemp in self.listBoxInfo:
                if boxInfoTemp.boxLongitude == "None":
                    boxInfoTemp.boxLongitude, boxInfoTemp.boxLatitude = XYZ_To_BLH(self.stParam.getLidarLongitude(0),
                                                                                   self.stParam.getLidarLatitude(0),
                                                                                   self.stBoxInfo.boxCenter[0],
                                                                                   self.stBoxInfo.boxCenter[1],
                                                                                   self.stParam.getAngleNorthT(0))

            # t1 = time.time()
            self.stParam.setFrameId(self.stDstData.nFrameId)


            try:
                # http转发识别结果
                t1 = time.time()
                bSendResult = self.stParam.getSendResult()
                if bSendResult:


                    dictMsg = sendHttpMessage(self.stParam, self.stDstData)
                    req = json.dumps(dictMsg)
                    time1 = time.time()

                    listAllBaseBoxInfo = self.stDstData.listBaseBoxInfo
                    information = getHeadInfo(listAllBaseBoxInfo)
                    trans_nums = getBoxNums(listAllBaseBoxInfo)
                    logti_t = self.stParam.getLidarLongitude(0)
                    lanti_t = self.stParam.getLidarLatitude(0)
                    angle_north_t = self.stParam.getAngleNorthT(0)
                    nSingleFrame = self.stDstData.nSingleFrame
                    object_pack_ll = packetBoxInfo(listAllBaseBoxInfo, logti_t, lanti_t, angle_north_t, nSingleFrame)
                    objects_all = bytes()
                    if len(object_pack_ll) > 0:
                        for i in range(len(object_pack_ll)):
                            objects_all += object_pack_ll[i]
                    time_stamp = time.time()
                    time_stamp_temp1, time_stamp_temp2 = math.floor(time_stamp), int((time_stamp - math.floor(time_stamp)) * 1e6)
                    time_pack = struct.pack('>I', time_stamp_temp1) + struct.pack('>I', time_stamp_temp2)  # 从1900年开始的秒数+微秒数
                    head_pack = struct.pack('BB', self.stDstData.nEqupId[0],self.stDstData.nEqupId[1]) + \
                                struct.pack('>H', int(self.stDstData.trunAlg)) + \
                                struct.pack('>2BH',0,0,int(self.stDstData.nFrameId % 65000)) + \
                                time_pack + \
                                struct.pack('>I', int(logti_t * 1e7)) + \
                                struct.pack('>I', int(lanti_t * 1e7)) + \
                                struct.pack('>H', int(round(angle_north_t)))
                    send_data = information + head_pack + trans_nums + objects_all + struct.pack('>I', 0) + struct.pack('B', 0) + struct.pack('B', 255)

                    byteSaveResult = None
                    #----------------------test----------------------#
                    if stUiParam.bSaveE1 == 1 and my_mkdir('./savedata/'):
                        if byteSaveResult is None:
                            byteSaveResult = send_data
                            with open(r'./savedata/ResultData_0601.txt','a') as log_file:
                                tSend = time.time()
                                strTimeStamp = getTimeStamp(tSend)
                                hexData = binascii.b2a_hex(byteSaveResult).decode('utf-8')
                                t = hexData.upper()
                                hexData = ' '.join([t[2*i:2*(i+1)] for i in range(int(len(t)/2))])
                                log_file.write(strTimeStamp + ": " + str(hexData) + "\n")

                    # ----------------------test----------------------#
                    # 转发识别结果
                    listForwardDstDevTemp = self.stParam.getListForwardDstDev()
                    for i in range(len(listForwardDstDevTemp)):
                        strDev = listForwardDstDevTemp[i]
                        if not strDev[1]:
                            continue

                        nameList = strDev[0].split('-')  # 分割字符串
                        strDevIp = nameList[0]
                        nDevPort = int(nameList[1])
                        try:
                            res = requests.post('http://{}:{}/station/submitE1Frame'.format(strDevIp, nDevPort), data=req, timeout=0.03)
                            MyLog.writeLog("{}[:{}] - http send E1Frame success".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                                    sys._getframe().f_lineno), LOG_INFO, ENABLE_WRITE_LOG,ENABLE_PRINT)
                        except:
                            print(traceback.format_exc())
                            MyLog.writeLog("{}[:{}] - {} is unreachable!  http send E1Frame failed".
                                           format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                  sys._getframe().f_lineno, strDevIp),
                                           LOG_ERROR, ENABLE_WRITE_LOG, ENABLE_PRINT)

                MyLog.writeLog("{}[:{}] - Forward Event Start!\n".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                         sys._getframe().f_lineno), LOG_ERROR, ENABLE_WRITE_LOG, True)

                # 转发车流量瞬时事件和统计事件信息
                listReachableIpPort = []
                EventData = ([], [], [], [], [], [])
                nEventFeedBackPort = self.stParam.getEventFeedBackPort()
                if self.dstEventResultQueue.qsize() > 0:
                    try:
                        EventData = self.dstEventResultQueue.get(True,0.01)
                    except:
                        pass
                listForwardEventDstDevTemp = self.stParam.getListForwardEventDstDev()
                for i in range(len(listForwardEventDstDevTemp)):
                    strDev = listForwardEventDstDevTemp[i]
                    if not strDev[1]:
                        continue

                    nameList = strDev[0].split('-')  # 分割字符串
                    strDevIp = nameList[0]
                    nDevPort = int(nameList[1])
                    listReachableIpPort.append([strDevIp, nDevPort])

                    if len(EventData) > 0:
                        if not self.stParam.getHighSpeed():
                            if EventData[0]:
                                B0SendData = self.B0_pcak(
                                    self.ConversionB0MsgContent(EventData[0], len(EventData[0]), stDstData.nFrameId))
                                if B0SendData != None:
                                    self.udpSocket.sendto(B0SendData, (strDevIp, nDevPort))
                            if EventData[1]:
                                B1SendData = self.B1_pcak(
                                    self.ConversionB1MsgContent(EventData[1], len(EventData[1]), stDstData.nFrameId))
                                if B1SendData != None:
                                    self.udpSocket.sendto(B1SendData, (strDevIp, nDevPort))
                            if EventData[2]:
                                B2SendData = self.B2_pcak(
                                    self.ConversionB2MsgContent(EventData[2], len(EventData[2]), stDstData.nFrameId))
                                if B2SendData != None:
                                    self.udpSocket.sendto(B2SendData, (strDevIp, nDevPort))
                        else:
                            if EventData[3]:
                                try:
                                    json_B3 = getHttpB3Data(EventData[3], self.stDstData)
                                    strRes = requests.post('http://{}:{}/station/submitB3Frame'.format(strDevIp, nDevPort), data=json_B3, timeout=0.015)
                                    dictRes = strRes.json()
                                    if stUiParam.bSaveEvent == 1 and my_mkdir('./savedata/Eventlog/'):
                                        with open(r'./savedata/Eventlog/B3send.log', 'a') as log_file:
                                            tEvent = time.time()
                                            strTimeStamp = getTimeStamp(tEvent)
                                            log_file.write(strTimeStamp + " - " + str(dictRes) + "\n")
                                except:
                                    pass
                            if EventData[4]:
                                try:
                                    json_B4, self.thisTimeStamp = getHttpB4Data(EventData[4], self.stDstData, self.thisTimeStamp)
                                    strRes = requests.post('http://{}:{}/station/submitB4Frame'.format(strDevIp, nDevPort), data=json_B4, timeout=0.015)
                                    dictRes = strRes.json()
                                    if stUiParam.bSaveEvent == 1 and my_mkdir('./savedata/Eventlog/'):
                                        with open(r'./savedata/Eventlog/B4send.log', 'a') as log_file:
                                            tEvent = time.time()
                                            strTimeStamp = getTimeStamp(tEvent)
                                            log_file.write(strTimeStamp + " - " + str(dictRes) + "\n")
                                except:
                                    pass
                            if len(EventData[5]) != 0 and EventData[5][4] != 0:
                                json_B5 = getHttpB5Data(EventData[5], self.stDstData)
                                try:
                                    strRes = requests.post('http://{}:{}/station/submitB5Frame'.format(strDevIp, nDevPort), data=json_B5, timeout=0.015)
                                    dictRes = strRes.json()
                                    if stUiParam.bSaveEvent == 1 and my_mkdir('./savedata/Eventlog/'):
                                        with open(r'./savedata/Eventlog/B5send.log','a') as log_file:
                                            tEvent = time.time()
                                            strTimeStamp = getTimeStamp(tEvent)
                                            log_file.write(strTimeStamp + " - " + str(dictRes) + "\n")
                                except:
                                    pass
                                stB5Msg = self.ConversionB5MsgContent(EventData[5], len(EventData[5]), stDstData.nFrameId)
                                dictPost2BS = self.getPost2BS(stB5Msg)
                                for nBSId in dictPost2BS:
                                    self.udpSocket.sendto(dictPost2BS[nBSId], (self.stParam.getStationSrcIp(self.listBaseStationId.index(nBSId)), nEventFeedBackPort))
                            MyLog.writeLog("{}[:{}] - Forward Event Sucessful!\n".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                                         sys._getframe().f_lineno), LOG_INFO, ENABLE_WRITE_LOG, True)
            except:
                print(traceback.format_exc())
                MyLog.writeLog("{}[:{}] - Forward error!\n{}".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                     sys._getframe().f_lineno,traceback.format_exc()), LOG_ERROR, ENABLE_WRITE_LOG, True)
            self.updateGL()

            # print("timeout = %d ms" % self.timeTemp.elapsed())
            # self.timeTemp.restart()
        except:
            print(traceback.format_exc())
        return self.stDstData.nFrameId


    def SelectionChange(self, select_text):
        fLongitude = self.stParam.getLidarLongitude(0)
        fLatitude = self.stParam.getLidarLatitude(0)
        fAngleNorthT = self.stParam.getAngleNorthT(0)
        listlistAttachedIp = self.stParam.getlistAttachedIp()
        stTransform = Lon_Lat_Meter_Transform(fLongitude, fLatitude, fAngleNorthT)

        str_num = list(filter(str.isdigit, select_text))

        if str_num != []:
            i = int(''.join(str_num)) - 1
            x,y = stTransform.lonlat_to_xy_of_draw(np.array([self.stParam.getLidarLongitude(i),self.stParam.getLidarLatitude(i)]).reshape((1,2))).tolist()[0]
            self.resetView_selectStation(x ,y)
        else:
            i = len(listlistAttachedIp)//2
            x,y = stTransform.lonlat_to_xy_of_draw(np.array([self.stParam.getLidarLongitude(i),self.stParam.getLidarLatitude(i)]).reshape((1,2))).tolist()[0]
            self.resetView_selectStation(x ,y,1000)

    def openFile(self, FileList):
        # 判断选择的文件列表是否为空,即使不选择,FileList的len依然是2,如果选择了文件,则FileList中的[0]元素长度必然>0
        if len(FileList) < 1:
            return

        self.stParam.setPause(True)
        self.stParam.setListOfflineFiles(FileList)

        while self.srcEventDataQueue.qsize() > 0:
            try:
                toBeDel = self.srcEventDataQueue.get_nowait()
            except:
                continue

        while self.dstEventResultQueue.qsize() > 0:
            try:
                toBeDel = self.dstEventResultQueue.get_nowait()
            except:
                continue

        while self.stSrcPcResultQueue.qsize() > 0:
            try:
                toBeDel = self.stSrcPcResultQueue.get_nowait()
            except:
                continue

        self.stParam.setUdpSocket(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
        self.udpSocket = self.stParam.getUdpSocket()
        self.stParam.setStatus(0)
        listPcOver = np.full(len(self.stParam.getListOfflineFiles()), False, bool)
        self.stParam.setPcOver(listPcOver)
        self.stParam.setFrameId(0)
        self.stParam.setOnline(False)
        self.stParam.setReadChanged(True)
        self.stParam.setAlgChanged(True)
        self.stParam.setParamChanged(True)
        self.stParam.setPause(False)

        # self.stop()
        self.start()

    def stopPlay(self):
        self.stop()
        self.stParam.setPause(True)

        while self.srcEventDataQueue.qsize() > 0:
            try:
                toBeDel = self.srcEventDataQueue.get_nowait()
            except:
                continue

        while self.dstEventResultQueue.qsize() > 0:
            try:
                toBeDel = self.dstEventResultQueue.get_nowait()
            except:
                continue

        while self.stSrcPcResultQueue.qsize() > 0:
            try:
                toBeDel = self.stSrcPcResultQueue.get_nowait()
            except:
                continue
        # self.stParam.setOnline(False)
        self.listRect = []
        self.listBoxInfo = []
        self.nSelectBoxId = -1
        self.listSelectBoxId = []
        self.selectBoxInfo = None
        # self.stParam.setRunning(False)

    def startPlay(self):
        self.stParam.setStatus(0)
        listPcOver = np.full(len(self.stParam.getListOfflineFiles()), False, bool)
        self.stParam.setPcOver(listPcOver)
        self.stParam.setFrameId(0)
        self.stParam.setReadChanged(True)
        self.stParam.setAlgChanged(True)
        self.stParam.setParamChanged(True)
        self.stParam.setPause(False)

    def stop(self):
        # time = QTime()
        # time.start()
        # print("stopping...")
        # 关闭进程
        self.stParam.setRunning(False)
        self.stParam.setPause(False)
        self.stParam.setSave(False)
        self.stParam.setParamChanged(True)

        while self.second is not None and self.second.is_alive():
            self.second.terminate()

        while self.read is not None and self.read.is_alive():
            self.read.terminate()

        while self.detect is not None and self.detect.is_alive():
            self.detect.terminate()

        # 停止显示
        self.stParam.setPause(True)

        # 清空数据
        self.stParam.setFrameId(0)
        my_delDir(PcCachePath)

        # print("stop elapsed: %d" % time.elapsed())

    def start(self):
        if self.read is None or (not self.read.is_alive()):
            listPcOver = np.full(len(self.stParam.getListOfflineFiles()), False, bool)
            self.stParam.setRunning(True)
            self.stParam.setPause(False)
            self.stParam.setStatus(0)
            self.stParam.setPcOver(listPcOver)
            my_delDir(PcCachePath)
            listBaseQueue = []
            for i in range(len(self.stParam.getlistAttachedIp())):
                listBaseQueue.append(Queue())
            self.pcBaseQueueList = listBaseQueue
            # 判断选择的文件列表是否为空,即使不选择,FileList的len依然是2,如果选择了文件,则FileList中的[0]元素长度必然>0
            listOfflineFiles = self.stParam.getListOfflineFiles()
            if (not self.stParam.getOnline()) and (len(listOfflineFiles) < 1 or len(listOfflineFiles[0]) < 1):
                return

            self.second = Process(target=proPcAlg, args=(self.stSrcPcResultQueue, self.srcEventDataQueue, self.pcBaseQueueList, self.stParam, self.lockPc))
            self.read = Process(target=readData, args=(self.stSrcPcResultQueue,self.pcBaseQueueList, self.stParam, self.lockPc))
            self.detect = Process(target=proEventDetect, args=(self.srcEventDataQueue, self.dstEventResultQueue, self.stParam, self.lockPc))

            self.second.start()
            self.read.start()
            self.detect.start()

    def pause(self):
        # self.stParam.printSelf()
        self.stParam.setPause(True)
        # self.stParam.printSelf()

    def play(self):
        if 1 <= self.stParam.getStatus() <= 2:
            self.stParam.setStatus(3)
        if self.isPlayOver():
            self.stParam.setStatus(0)

        self.stParam.setPause(False)
        self.start()

    def restart(self):
        # stop
        self.stop()

        # start
        self.start()

    def previous(self):
        usStatusTemp = self.stParam.getStatus()
        self.stParam.setFrameId(max(0, self.stParam.getFrameId() - 1))
        # if usStatusTemp != 1 and usStatusTemp != 2:
        #     self.stParam.setFrameId(max(0, self.stParam.getFrameId() - 1))
        #     # self.stParam.nFrameId = max(0, self.stParam.nFrameId - 1)

        self.stParam.setStatus(1)
        self.stParam.setPause(False)
        MyLog.writeLog("{}[:{}] - PcPyGLWidget need:{}".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, self.stParam.getFrameId()), LOG_INFO, ENABLE_WRITE_LOG, True)

    def next(self):
        usStatusTemp = self.stParam.getStatus()
        self.stParam.setFrameId(self.stParam.getFrameId() + 1)
        # if usStatusTemp != 1 and usStatusTemp != 2:
        #     self.stParam.setFrameId(max(0, self.stParam.getFrameId() - 1))
        #     # self.stParam.nFrameId = max(0, self.stParam.nFrameId - 1)

        self.stParam.setStatus(1)
        self.stParam.setPause(False)
        MyLog.writeLog("{}[:{}] - PcPyGLWidget need:{}".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, self.stParam.getFrameId()), LOG_INFO, ENABLE_WRITE_LOG, True)

    def skipTo(self, nFrameId):
        self.stParam.setFrameId(max(0, nFrameId))
        self.stParam.setStatus(1)
        self.stParam.setPause(False)
        self.stParam.setParamChanged(True)
        MyLog.writeLog("{}[:{}] - PcPyGLWidget skipTo:{}".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                         sys._getframe().f_lineno, self.stParam.getFrameId()), LOG_INFO, ENABLE_WRITE_LOG, True)

    def onLine(self):
        self.stParam.setOnline(True)
        self.start()
        # self.restart()

    def offLine(self):
        self.stParam.setOnline(False)
        # self.restart()

    def setDefaultModelViewMatrix(self):
        self.modelview_matrix_ = np.array([[2.16917381e-01, 3.58195961e-01, -9.08098578e-01, 0.00000000e+00],
                                           [-9.74002719e-01, 1.71728823e-02, -2.25885659e-01, 0.00000000e+00],
                                           [-6.53166994e-02, 9.33488667e-01, 3.52608502e-01, 0.00000000e+00],
                                           [2.95688105e+00, 6.45137596e+00, -4.45000000e+01, 1.00000000e+00]])


    def drawAxis(self):
        # Local Space
        s_Scale = 10
        font = QtGui.QFont()
        font.setPointSize(12)

        glPushAttrib(GL_LINE_BIT)
        glLineWidth(1.0)
        glBegin(GL_LINES)

        # X轴
        glColor3ub(255, 0, 0)
        glVertex3i(0, 0, 0)
        glVertex3i(s_Scale, 0, 0)

        # Y轴
        glColor3ub(0, 255, 0)
        glVertex3i(0, 0, 0)
        glVertex3i(0, s_Scale, 0)

        # Z轴
        glColor3ub(0, 0, 255)
        glVertex3i(0, 0, 0)
        glVertex3i(0, 0, s_Scale)

        glEnd()
        glPopAttrib()
        glColor3ub(255, 0, 0)
        self.renderText(s_Scale, 0, 0, 'x', font)
        glColor3ub(0, 255, 0)
        self.renderText(0, s_Scale, 0, 'y', font)
        glColor3ub(0, 0, 255)
        self.renderText(0, 0, s_Scale, 'z', font)
        glColor3ub(255, 255, 255)

    def drawGrid(self, nWidth, nHeight, GridStep):
        glColor3f(0.5, 0.5, 0.5)
        glBegin(GL_LINES)
        z = -5
        # 画竖线
        step = 1
        for i in range(-int(nWidth), int(nWidth + 1), GridStep):
            glVertex3f(i, -nWidth * step, z)
            glVertex3f(i, nWidth * step, z)
        # # 划横线
        for j in range(-int(nHeight), int(nHeight + 1), GridStep):
            glVertex3f(-nHeight * step, j, z)
            glVertex3f(nHeight * step, j, z)
        glEnd()

    def GridRefresh(self, x, y, step):
        self.GridWidth = x
        self.GridHeight = y
        self.GridStep = step
        self.updateGL()

    def drawSelectCube(self):
        t1 = time.time()
        # 将要使用的顶点的序号保存到一个数组里面
        # 目前八个顶点是按照如下顺序排列的：上顶面：1,2,6,5（逆时针排序），下底面：0,3,7,4（逆时针排序）
        index_list = np.array([[1, 2], [2, 6], [6, 5], [5, 1],
                               [0, 3], [3, 7], [7, 4], [4, 0],
                               [0, 1], [3, 2], [7, 6], [4, 5]])
        try:
            nCol = 0
            for i in range(2):
                if len(self.listSelectCubeCenter[i]) == 0:
                    continue
                nRow = 0
                for listboxCeter in self.listSelectCubeCenter[i]:
                    if len(listboxCeter) == 0:
                        continue
                    glPointSize(5)
                    if i == 1:
                        glColor3f(0.0, 1.0, 1.0)
                    else:
                        glColor3f(1.0, 0.0, 0.0)
                    if self.nSelectedTableRow == nRow and self.nSelectedTableCol == nCol:
                        glColor3f(0.0, 1.0, 0.0)
                    glBegin(GL_POINTS)
                    glVertex3f(listboxCeter[0], listboxCeter[1], listboxCeter[2])
                    glEnd()
                    glPointSize(1)
                    nRow +=1
                nCol +=1
                for listboxVertex in self.listSelectVertexPc[i]:
                    if listboxVertex[1].shape[0] == 0:
                        continue
                    glLineWidth(3)
                    glBegin(GL_LINES)
                    for j in range(12):  # 12条线段
                        for k in range(2):  # 每条线段 2个顶点
                            # n行, 每一行是8个点, 每个点是(x,y,z)三坐标值, 所以box_data[i][j][k]中i对应“行”, j对应8个顶点中的“点序号”, k对应x，y，z
                            if i == 1:
                                glColor3f(0.0, 1.0, 1.0)
                            else:
                                glColor3f(1.0, 0.0, 0.0)
                            glVertex3fv(listboxVertex[1][index_list[j][k]])
                    glEnd()
                    glColor3f(1.0, 1.0, 1.0)
                    glLineWidth(1)
        except:
            print(traceback.format_exc())
            MyLog.writeLog("{}[:{}] - Except!! drawnewCube ERROR at Frame-{}!"
                           .format(__file__.split('/')[len(__file__.split('/')) - 1],
                                   sys._getframe().f_lineno, self.stParam.getFrameId()), LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)

    def drawBaseStation(self,nWide):
        fLongitude = self.stParam.getLidarLongitude(0)
        fLatitude = self.stParam.getLidarLatitude(0)
        fAngleNorthT = self.stParam.getAngleNorthT(0)
        listlistAttachedIp = self.stParam.getlistAttachedIp()
        stTransform = Lon_Lat_Meter_Transform(fLongitude, fLatitude, fAngleNorthT)
        for i in range(len(listlistAttachedIp)):
            x,y = stTransform.lonlat_to_xy_of_draw(np.array([self.stParam.getLidarLongitude(i),self.stParam.getLidarLatitude(i)]).reshape((1,2))).tolist()[0]
            z = 0
            # glBegin(GL_LINES)
            # glColor3f(1.0, 1.0, 0)
            # glVertex3fv(np.array([x-nWide,y-nWide,z]))
            # glVertex3fv(np.array([x-nWide,y+nWide,z]))
            # glVertex3fv(np.array([x-nWide,y+nWide,z]))
            # glVertex3fv(np.array([x+nWide,y+nWide,z]))
            # glVertex3fv(np.array([x+nWide,y+nWide,z]))
            # glVertex3fv(np.array([x+nWide,y-nWide,z]))
            # glVertex3fv(np.array([x+nWide,y-nWide,z]))
            # glVertex3fv(np.array([x-nWide,y-nWide,z]))
            # glColor3f(1.0, 1.0, 1.0)
            # glEnd()
            font = QtGui.QFont()
            font.setPointSize(15)
            self.renderText(x, y, z, '%d' % (i + 1), font)

    def drawCube(self):
        t1 = time.time()
        # 将要使用的顶点的序号保存到一个数组里面
        # 目前八个顶点是按照如下顺序排列的：上顶面：1,2,6,5（逆时针排序），下底面：0,3,7,4（逆时针排序）
        index_list = np.array([[1, 2], [2, 6], [6, 5], [5, 1],
                               [0, 3], [3, 7], [7, 4], [4, 0],
                               [0, 1], [3, 2], [7, 6], [4, 5]])

        # glColor3f(1.0, 0.0, 0.0)
        nCount = len(self.listBoxInfo)
        if nCount > 0:
            self.listRect = []
            # print("Box count: %d" % nCount)
            try:
                for i in range(nCount):
                    # if self.listBoxInfo[i].strClass in self.stDstData.filterList:
                    boxInfo = self.listBoxInfo[i]
                    boxVertexPc = boxInfo.boxVertexPc
                    if len(boxVertexPc) == 0:
                        continue

                    maxX = 1e-10
                    minX = 1e10
                    maxY = 1e-10
                    minY = 1e10
                    for n in range(8):
                        p = boxVertexPc[n]
                        # winPoint = np.zeros(2)
                        x, y, z = gluProject(p[0], p[1], p[2], self.modelview, self.projection, self.viewport)
                        y = self.fHeight - y
                        maxX = max(maxX, x)
                        minX = min(minX, x)
                        maxY = max(maxY, y)
                        minY = min(minY, y)

                    rect = QRectF(QPointF(minX, minY), QPointF(maxX, maxY))
                    self.listRect.append([rect, i])
                    if not self.stParam.getPause() and self.stParam.getStatus() in [0,3]:
                        if self.listBoxInfo[i].boxId not in self.dictHisTrack:
                            self.dictHisTrack[self.listBoxInfo[i].boxId] = [time.time(), self.listBoxInfo[i].boxCenter]
                        else:
                            if time.time() - self.dictHisTrack[self.listBoxInfo[i].boxId][0] < 60:
                                while len(self.dictHisTrack[self.listBoxInfo[i].boxId]) > 61:
                                    self.dictHisTrack[self.listBoxInfo[i].boxId].pop(1)
                                self.dictHisTrack[self.listBoxInfo[i].boxId].append(self.listBoxInfo[i].boxCenter)
                                self.dictHisTrack[self.listBoxInfo[i].boxId][0] = time.time()
                            else:
                                self.dictHisTrack[self.listBoxInfo[i].boxId] = [time.time(), self.listBoxInfo[i].boxCenter]
                    if self.listBoxInfo[i].boxId in self.listSelectBoxId \
                            or self.npDeadId.__contains__(boxInfo.boxId) \
                            or self.npOccluId.__contains__(boxInfo.boxId):
                        listTrack = self.dictHisTrack[self.listBoxInfo[i].boxId]
                        glLineWidth(1)
                        glBegin(GL_LINES)
                        glColor3f(0, 1, 0)
                        for i in range(1, len(listTrack) - 1):
                            glVertex3fv(np.array([listTrack[i]]))
                            glVertex3fv(np.array([listTrack[i+1]]))
                        glEnd()
                        glLineWidth(5)
                        # print("glLineWidth(10)...")
                    else:
                        glLineWidth(1)

                    glBegin(GL_LINES)
                    for j in range(12):  # 12条线段
                        for k in range(2):  # 每条线段 2个顶点
                            # n行, 每一行是8个点, 每个点是(x,y,z)三坐标值, 所以box_data[i][j][k]中i对应“行”, j对应8个顶点中的“点序号”, k对应x，y，z
                            if j in [3, 7, 8, 11]:
                                glColor3f(1.0, 1.0, 0.0)
                            else:
                                glColor3f(1.0, 1.0, 1.0)
                                if boxInfo.strSource == "Complement":
                                    glColor3f(0, 1, 1.0)
                                elif boxInfo.strSource == "PcRfusion":
                                    glColor3f(1.0, 0.0, 0.0)
                                elif boxInfo.strSource == "Radar":
                                    glColor3f(1, 0.0, 0.0)
                                elif boxInfo.strSource == "RRfusion":
                                    glColor3f(0.0, 1.0, 0.0)
                                if self.npDeadId.__contains__(boxInfo.boxId):
                                    glColor3f(0, 0, 1.0)
                                elif self.npOccluId.__contains__(boxInfo.boxId):
                                    glColor3f(0, 0.55, 0.55)
                            glVertex3fv(boxVertexPc[index_list[j][k]])

                    glColor3f(1.0, 1.0, 1.0)
                    glEnd()
            except:
                MyLog.writeLog("{}[:{}] - Except!! drawCube ERROR at Frame-{}!"
                               .format(__file__.split('/')[len(__file__.split('/')) - 1],
                                sys._getframe().f_lineno, self.stParam.getFrameId()), LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)
                print(traceback.format_exc())

        # print("drawCube elapsed = %.3fms" % ((time.time() - t1) * 1000))


    def drawCubeInfo(self):

        bIsExistSelectId = False
        nCount = len(self.listBoxInfo)
        if nCount > 0:
            boxInfoList = []
            try:
                for i in range(nCount):
                    boxInfo = self.listBoxInfo[i]
                    # if boxInfo.strClass in self.stDstData.filterList:
                    boxVertexPc = boxInfo.boxVertexPc
                    if len(boxVertexPc) == 0:
                        continue

                    # print(boxVertexPc)
                    if boxInfo.boxId == self.nSelectBoxId and self.stDstData is not None and i < len(self.listBoxInfo):
                        # 计算选中目标绘制信息的矩形框, 在二维绘制
                        pBottomLeft = boxVertexPc[0]
                        x, y, z = gluProject(pBottomLeft[0], pBottomLeft[1], pBottomLeft[2], self.modelview, self.projection, self.viewport)
                        y = self.fHeight - y
                        self.rectInfo = QRectF(QPointF(x, y - 86), QPointF(x + 70, y))
                        self.selectBoxInfo = self.listBoxInfo[i]
                        bIsExistSelectId = True
                    else:
                        # n行, 每一行是8个点, 每个点是(x,y,z)三坐标值, 所以box_data[i][j][k]中i对应“行”, j对应8个顶点中的“点序号”, k对应x，y，z
                        fx = boxVertexPc[1][0]
                        fy = boxVertexPc[1][1]
                        fz = boxVertexPc[1][2]

                        font = QtGui.QFont()
                        font.setPointSize(12)
                        self.renderText(fx, fy, fz, '{}-{}-{}-{}'.format(boxInfo.boxId,
                                                                         int(boxInfo.nBaseStationId),
                                                                         int(boxInfo.nPreFrameCnt),
                                                                         int(boxInfo.nBaseStationBoxId)), font)
                                                                         # int(self.stDstData.nFrameId)), font)


            except:
                print(traceback.format_exc())
                MyLog.writeLog("{}[:{}] - Except!! drawCubeInfo ERROR at Frame-{}!"
                               .format(__file__.split('/')[len(__file__.split('/')) - 1],
                                       sys._getframe().f_lineno, self.stParam.getFrameId()), LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)

        if not bIsExistSelectId:
            self.rectInfo = None

    def drawCubeInfo_1(self):

        strSelectFilePath = self.stParam.getListOfflineFiles()

        #  离线状态下,判断路径中是否含有exception，如果含有，则表示播放名为exception的文件夹中以含有车辆id号命名的异常车辆数据csv文件，
        #  从路径中拿出数字（车辆id），作为判断条件,否则播放正常数据
        #  如果车辆的boxId与路径中拿出来的车辆id匹配，则将字体放大，便于用户找到异常车辆情况
        if not self.stParam.getOnline():
            if 'exception' in strSelectFilePath[0]:
                str_num = list(filter(str.isdigit, strSelectFilePath[0]))
                num_in_filename = int(''.join(str_num))
                filename = num_in_filename
            else:
                filename = None
        else:
            filename = None

        bIsExistSelectId = False
        nCount = len(self.listBoxInfo)
        if nCount > 0:
            boxInfoList = []
            try:
                for i in range(nCount):
                    boxInfo = self.listBoxInfo[i]
                    # if boxInfo.strClass in self.stDstData.filterList:
                    boxVertexPc = boxInfo.boxVertexPc
                    if len(boxVertexPc) == 0:
                        continue

                    # print(boxVertexPc)
                    if boxInfo.boxId == self.nSelectBoxId and self.stDstData is not None and i < len(self.listBoxInfo):
                        # 计算选中目标绘制信息的矩形框, 在二维绘制
                        pBottomLeft = boxVertexPc[0]
                        x, y, z = gluProject(pBottomLeft[0], pBottomLeft[1], pBottomLeft[2], self.modelview, self.projection, self.viewport)
                        y = self.fHeight - y
                        self.rectInfo = QRectF(QPointF(x, y - 86), QPointF(x + 70, y))
                        self.selectBoxInfo = self.listBoxInfo[i]
                        bIsExistSelectId = True
                    else:
                        # n行, 每一行是8个点, 每个点是(x,y,z)三坐标值, 所以box_data[i][j][k]中i对应“行”, j对应8个顶点中的“点序号”, k对应x，y，z
                        fx = boxVertexPc[1][0]
                        fy = boxVertexPc[1][1]
                        fz = boxVertexPc[1][2]

                        font = QtGui.QFont()
                        font.setPointSize(12)
                        if filename != None:
                            if boxInfo.boxId == int(filename):
                                if int(boxInfo.boxId) > 0:
                                    font.setPointSize(24)
                                    self.renderText(fx, fy, fz, '{}-{}-{}-{}'.format(boxInfo.boxId,
                                                                                     int(boxInfo.nBaseStationId),
                                                                                     int(boxInfo.nPreFrameCnt),
                                                                                     int(boxInfo.nBaseStationBoxId)), font)

                            else:
                                if int(boxInfo.boxId) > 0:
                                    font.setPointSize(12)
                                    self.renderText(fx, fy, fz, '{}-{}-{}-{}'.format(boxInfo.boxId,
                                                                                     int(boxInfo.nBaseStationId),
                                                                                     int(boxInfo.nPreFrameCnt),
                                                                                     int(boxInfo.nBaseStationBoxId)), font)
                        else:
                            if int(boxInfo.boxId) > 0:
                                font.setPointSize(12)
                                self.renderText(fx, fy, fz, '{}-{}-{}-{}'.format(boxInfo.boxId,
                                                                                 int(boxInfo.nBaseStationId),
                                                                                 int(boxInfo.nPreFrameCnt),
                                                                                 int(boxInfo.nBaseStationBoxId)), font)
                            # int(self.stDstData.nFrameId)), font)


            except:
                print(traceback.format_exc())
                MyLog.writeLog("{}[:{}] - Except!! drawCubeInfo ERROR at Frame-{}!"
                               .format(__file__.split('/')[len(__file__.split('/')) - 1],
                                       sys._getframe().f_lineno, self.stParam.getFrameId()), LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)

        if not bIsExistSelectId:
            self.rectInfo = None

    def paintGL(self):
        # gltime = QTime()
        # gltime.start()
        try:
            maxHeight = None
            if self.selectBoxInfo is not None and not self.bRegistration:
                if self.selectBoxInfo.strSource == "PcRfusion":
                    strId = "PcRF_ID:{}".format(self.nSelectBoxId)
                else:
                    strId = "ID:{}".format(self.nSelectBoxId)
                strLen = "Length:{}".format(self.selectBoxInfo.boxSize[1])
                strWidth = "Width:{}".format(self.selectBoxInfo.boxSize[0])
                strHeight = "Height:{}".format(self.selectBoxInfo.boxSize[2])
                if strLen != 'None' and strLen != 'N':
                    strLen = "Length:{}".format(round(self.selectBoxInfo.boxSize[1], 3))
                    strWidth = "Width:{}".format(round(self.selectBoxInfo.boxSize[0], 3))
                    strHeight = "Height:{}".format(round(self.selectBoxInfo.boxSize[2], 3))

                strSpeed = "Speed:{}".format(self.selectBoxInfo.boxSpeed)
                if strSpeed != 'None':
                    strSpeed = "Speed:{}".format(round(self.selectBoxInfo.boxSpeed, 3))

                strAngle = "Angle:{}".format(self.selectBoxInfo.boxAngle)
                if strAngle != 'None':
                    fAngle = self.selectBoxInfo.boxAngle / math.pi * 180
                    if fAngle > 360:
                        fAngle = fAngle - 360
                    strAngle = "Angle:{}".format(round(fAngle, 3))

                strClass = "Class:{}".format(self.selectBoxInfo.strClass)

                fm = QFontMetrics(QFont('WenQuanYi Micro Hei'))
                maxWidth = max(fm.width(strLen), fm.width(strWidth))
                maxWidth = max(maxWidth, fm.width(strHeight))
                maxWidth = max(maxWidth, fm.width(strSpeed))
                maxWidth = max(maxWidth, fm.width(strAngle))
                maxWidth = max(maxWidth, fm.width(strClass))
                maxHeight = fm.height()
        except:
            MyLog.writeLog("{}[:{}] - Except!! paintGL ERROR at Frame-{}!"
                           .format(__file__.split('/')[len(__file__.split('/')) - 1],
                                   sys._getframe().f_lineno, self.stParam.getFrameId()), LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)
            PyGLWidget.paintGL(self)
            return

        PyGLWidget.paintGL(self)
        if not self.stParam.getShowPc():
            return

        glPushMatrix()  # 把当前矩阵入栈保存
        self.modelview = np.zeros((4, 4))
        self.projection = np.zeros((4, 4))
        self.viewport = [0, 0, self.fWidth, self.fHeight]

        # glGetIntegerv(GL_VIEWPORT, viewport)
        glGetDoublev(GL_MODELVIEW_MATRIX, self.modelview)
        glGetDoublev(GL_PROJECTION_MATRIX, self.projection)

        self.drawAxis()

        if self.bShowBox and self.stDstData is not None:
            self.drawCube()  # 绘制目标框
            self.drawCubeInfo()
        if self.bshowSelectCube:
            self.drawSelectCube()
        time1 = QTime()
        time1.start()
        nFrameId = self.stParam.getFrameId()
        if self.isPlayOver() or self.stParam.getPause():
            nFrameId = max(nFrameId - 1, 0)
        if self.stDstData is not None:
            self.renderText(0, 0, 0, 'Frame: %d' % self.stDstData.nFrameId)
        elif self.stParam.getRunning():
            self.renderText(0, 0, 0, 'Frame: %d' % nFrameId)
        else:
            self.renderText(0, 0, 0, 'Frame: %d' % 0)
        # time1 = QTime()
        # time1.start()
        if self.bShowLane and self.stParam.getRunning():
        # if self.bShowLane:
            self.drawBaseStation(2)
            glLineWidth(1)
            glPointSize(1)
            glEnableClientState(GL_VERTEX_ARRAY)
            glEnableClientState(GL_COLOR_ARRAY)
            for VBO in self.parent.listVBO:
                vtx_disp, clr_disp, cnt_disp = VBO
                vtx_disp.bind()
                glVertexPointer(3, GL_FLOAT, 0, vtx_disp)
                vtx_disp.unbind()
                clr_disp.bind()
                glColorPointer(3, GL_FLOAT, 0, clr_disp)
                clr_disp.unbind()
                glDrawArrays(GL_LINES, 0, cnt_disp)

            glDisableClientState(GL_VERTEX_ARRAY)
            glDisableClientState(GL_COLOR_ARRAY)
        # print("time1.start()",(time1.elapsed()))
        glPopMatrix()  # 恢复之前保存的矩阵

        # 注意：self.painter在创建时，一定不能传入self,否则会与paintGl冲突，导致崩溃。可在paintGL接口内部调用painter的begin（self）接口，指定绘制设备。
        if self.rectInfo is not None:
            pen = QPen(QColor("yellow"))
            pen.setWidth(2)
            self.painter.begin(self)
            self.painter.setPen(pen)
            if self.rectSelectPoint is not None:
                self.painter.drawRect(self.rectSelectPoint)

            if self.bShowBox and self.rectInfo is not None and maxHeight is not None and not self.bRegistration:
                self.rectInfo = QRectF(self.rectInfo.x(), self.rectInfo.y() + self.rectInfo.height() - maxHeight * 7, maxWidth, maxHeight * 7)
                brush = QBrush(QColor(0, 191, 255, 200))
                self.painter.setBrush(brush)
                self.painter.drawRect(self.rectInfo)

                pen.setColor(QColor("red"))
                self.painter.setFont(QFont('WenQuanYi Micro Hei'))
                self.painter.setPen(pen)
                pInfoRectTopLeft = self.rectInfo.topLeft()
                self.painter.drawText(QPoint(pInfoRectTopLeft.x() + 2, pInfoRectTopLeft.y() + maxHeight), strId)
                self.painter.drawText(QPoint(pInfoRectTopLeft.x() + 2, pInfoRectTopLeft.y() + maxHeight * 2), strLen)
                self.painter.drawText(QPoint(pInfoRectTopLeft.x() + 2, pInfoRectTopLeft.y() + maxHeight * 3), strWidth)
                self.painter.drawText(QPoint(pInfoRectTopLeft.x() + 2, pInfoRectTopLeft.y() + maxHeight * 4), strHeight)
                self.painter.drawText(QPoint(pInfoRectTopLeft.x() + 2, pInfoRectTopLeft.y() + maxHeight * 5), strSpeed)
                self.painter.drawText(QPoint(pInfoRectTopLeft.x() + 2, pInfoRectTopLeft.y() + maxHeight * 6), strAngle)
                self.painter.drawText(QPoint(pInfoRectTopLeft.x() + 2, pInfoRectTopLeft.y() + maxHeight * 7), strClass)

            self.painter.end()

        # print("paintGL() elapsed: %d ms" % (gltime.elapsed()))

    def initializeGL(self):
        # OpenGL state
        if BACKGROUND_BLACK:
            glClearColor(0.0, 0.0, 0.0, 0.0)
        else:
            glClearColor(1.0, 1.0, 1.0, 1.0)

        glEnable(GL_DEPTH_TEST)
        self.resetView()

    def mousePressEvent(self, _event):
        self.bIsLeftButton = False
        if _event.buttons() & QtCore.Qt.LeftButton:
            self.bIsLeftButton = True
                # print("self.firstPoint = ", self.firstPoint)
            # 处理选中目标框
            if self.stDstData is not None:
                curPoint = _event.pos()
                for i in range(len(self.listRect)):
                    rect = self.listRect[i][0]
                    if i >= len(self.listBoxInfo):
                        break
                    # print("Point:", curPoint.x(), curPoint.y(), "Rect:", rect.x(), rect.y(), rect.x() + rect.width(), rect.y() + rect.height(), "BoxID:", self.listBoxInfo[i].boxId)
                    if self.listRect[i][0].contains(curPoint):
                        self.nSelectBoxId = int(self.listBoxInfo[self.listRect[i][1]].boxId)
                        if self.nSelectBoxId not in self.listSelectBoxId:
                            self.listSelectBoxId.append(self.nSelectBoxId)
                        self.selectBoxInfo = self.listBoxInfo[self.listRect[i][1]]
                        self.npSelectBoxCenter = self.listBoxInfo[self.listRect[i][1]].boxCenter
                        self.npSelectBoxVertex = self.listBoxInfo[self.listRect[i][1]].boxVertexPc
                        if self.bSelectCube:
                            if self.listBoxInfo[self.listRect[i][1]].strSource == "Radar" or \
                                self.listBoxInfo[self.listRect[i][1]].strSource == "PcRfusion" or \
                                self.listBoxInfo[self.listRect[i][1]].strSource == "Complement":
                                self.sigSelectStationBox.emit([self.npSelectBoxCenter, self.npSelectBoxVertex])
                        else:
                            if not self.bRegistration:
                                self.sigSelectBox.emit(self.nSelectBoxId)
                        # print("Select:", self.listBoxInfo[i].boxId)
                        break

        elif _event.buttons() & QtCore.Qt.RightButton:
            self.nSelectBoxId = -1
            self.listSelectBoxId = []
            self.sigSelectBox.emit(self.nSelectBoxId)

        self.updateGL()
        PyGLWidget.mousePressEvent(self, _event)

    def mouseMoveEvent(self, _event):
        # print("mouseMoveEvent")
        PyGLWidget.mouseMoveEvent(self, _event)

    def mouseReleaseEvent(self, _event):

        PyGLWidget.mouseReleaseEvent(self, _event)

    def setRegistration(self, bStationRegistration):
        PyGLWidget.setRegistration(self, bStationRegistration)

    def setSelectCube(self, bSelectCube):
        PyGLWidget.setSelectCube(self, bSelectCube)


    def showSelect(self, listSelectCenter, listSelectData):

        self.listSelectCubeCenter = listSelectCenter
        self.listSelectVertexPc = listSelectData
        self.bshowSelectCube = True

        self.updateGL()
        self.update()

    def setshowSelectCube(self, bshowSelectCube):
        self.bshowSelectCube = bshowSelectCube

    def setSelectedPoint(self,nRow, nCol):
        self.nSelectedLidatTableRow = nRow
        self.nSelectedLidatTableCol = nCol
# ================================== PcPyGLWidget end =============================================
