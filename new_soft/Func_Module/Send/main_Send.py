from Common.interface import Interface, UsrSysState
from Common.CommonDefine import *
from Common.Protocol import parseNdarrayofByte, getByteofNdarray
switch_pc = {0: "car", 1: "bicycle", 2: "bus", 3: "motorbike", 4: "person", 5: "cone", 6: "truck", 7: "None", 8: "Minibus"}
switch_send = {0: "None", 1: "car", 2: "truck", 3: "bus", 4: "person", 5: "bicycle", 6: "motorbike", 7: "cone", 8: "Minibus"}
# 从类型名value获取类型的key
def getSendEnumFromStr(strClass):
    for i in range(len(switch_send)):
        if strClass == switch_send[i]:
            return i

    return -1

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

def getBoxInfo(trackers_cov):
    listBoxInfo = []
    for i in range(trackers_cov.shape[0]):
        stBoxInfo = structBoxInfo()
        # 获取目标框数据，用于填充表格
        stBoxInfo.boxCenter = trackers_cov[i, 0:3]
        stBoxInfo.boxSize = trackers_cov[i, 3:6]
        stBoxInfo.boxAngle = trackers_cov[i, 6]
        nType = trackers_cov[i, 7]
        stBoxInfo.boxSpeed = trackers_cov[i, 8]
        stBoxInfo.boxId = int(trackers_cov[i][9])
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

        stBoxInfo.nConfidence = int(trackers_cov[i, 10] * 100)
        stBoxInfo.nBaseStationId = trackers_cov[i, 12]
        stBoxInfo.nBaseStationBoxId = trackers_cov[i, 13]
        stBoxInfo.fcolor = trackers_cov[i, 14]
        stBoxInfo.nBoxCamId = trackers_cov[i, 17]
        stBoxInfo.nLane = trackers_cov[i, 18]
        stBoxInfo.nPreFrameCnt = trackers_cov[i, 19]

        listBoxInfo.append(stBoxInfo)  # 添加到链表

    return listBoxInfo

def getJsonBoxInfo_Pics(listBoxInfo):


    participants = []
    for boxInfo in listBoxInfo:
        dictBoxInfo = {}
        dictBoxInfo["id"] =int(boxInfo.boxId)
        dictBoxInfo["type"] = getSendEnumFromStr(boxInfo.strClass)

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
        dictBoxInfo["detecSource"] = int(boxInfo.nPreFrameCnt)
        dictBoxInfo["laneNum"] = int(boxInfo.nLane)

        if 0 < boxInfo.nBaseStationId < 256:
            dictBoxInfo["baseStationSource"] = int(boxInfo.nBaseStationId)
        else:
            dictBoxInfo["baseStationSource"] = 0

        # dictBoxInfo["longitude"] = round(boxInfo.boxLongitude, 7)
        # dictBoxInfo["latitude"] = round(boxInfo.boxLatitude, 7)
        dictBoxInfo["altitude"] = 0
        dictBoxInfo["speed"] = int(round(boxInfo.boxSpeed *100))

        # pitch=(180.0+angle_north_T+boxInfo.boxAngle / PI_rads) % 360
        #
        # dictBoxInfo["courseAngle"] = int(round(pitch *10, 2))
        dictBoxInfo["length"] = int(round(boxInfo.boxSize[1]*100))
        dictBoxInfo["width"] = int(round(boxInfo.boxSize[0]*100))
        dictBoxInfo["height"] = int(round(boxInfo.boxSize[2] * 100))

        dictBoxInfo["XCoordinate"] = int(boxInfo.boxCenter[0]*100)
        dictBoxInfo["YCoordinate"] = int(boxInfo.boxCenter[1]*100)
        dictBoxInfo["ZCoordinate"] = int(boxInfo.boxCenter[2]*100)
        dictBoxInfo["sourceId"] = int(boxInfo.nBaseStationBoxId)

        participants.append(dictBoxInfo)

    return participants

def E1Data_parse(data):
    recv_data = json.loads(data)

    participant = parseNdarrayofByte(recv_data["e1FrameParticipant"]).reshape((-1,44))[ : ,0:20]
    participants = getBoxInfo(participant)
    res = getJsonBoxInfo_Pics(participants)
    dictMsg = {}
    dictMsg["deviceId"] = recv_data["deviceId"]
    dictMsg["algorithmDuration"] = recv_data["algorithmDur"]
    dictMsg["globalFrameNo"] = recv_data["globalFrameNo"]
    dictMsg["globalTimeStamp"] = recv_data["globalTimeStamp"]
    dictMsg["frameNo"] = recv_data["sourceInfoList"][0]["sourceframeNo"]        # 入口点云数据帧号   0号基站帧号
    dictMsg["timeStamp"] = recv_data["sourceInfoList"][0]["sourcetimeStamp"]    #  入口识别结果时间戳  0号基站时间戳
    # dictMsg["longitude"] = round(stParam.getLidarLongitude(0), 7)
    # dictMsg["latitude"] = round(stParam.getLidarLatitude(0), 7)
    # dictMsg["angle"] = int(angle_north_T)
    dictMsg["participantNum"] = recv_data["participantNum"]
    dictMsg["e1FrameParticipant"] = res
    dictMsg["stationNum"] = recv_data["stationNum"]
    dictMsg["sourceInfoList"] = recv_data["sourceInfoList"]

    return dictMsg


class TestMod(Interface):
    def __init__(self):
        self.s_Parampath = './Configs/send_Param.xml'
        Interface.__init__(self, self.s_Parampath)

    def usr_process(self, data_bus_server):
        """重写方法"""
        # 功能接口状态类，用于接收接口类中的系统状态，同时上报功能进程的相关信息
        sys_state = UsrSysState(self.q_sys_state, self.q_model_state, self.log)
        # 获取用户订阅的subscriber列表，每个频道对应一个消息，为列表中的一个
        list_sub = data_bus_server.get_subscriber(self.list_channel)
        while sys_state.get_sys_state():
            try:
                list_msg = list_sub[0].parse_response(False, 0.01)
                channel = str(list_msg[1], encoding="utf-8")
                print(channel)
                if sys_state.get_sys_online():
                    req = E1Data_parse(list_msg[2])
                    print(req)
                    time.sleep(0.09)
                else:
                    self.log.info('there is no need to send offline')
            except:
                pass

        self.log.error('usr_process {} exit...'.format(self.log.name))


if __name__ == '__main__':
    testMod = TestMod()
    testMod.join()
    testMod.log.error('main exit...')
    sys.exit(0)
