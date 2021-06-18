from Common.CommonDefine import *


def getHttpB3Data(tMsgContent, nDeviceId):
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

    dict_B3 = {'deviceId': nDeviceId,
               'timeStamp': getTimeStamp(time.time()),
               'eventFrameId': int(math.floor(tMsgContent[4])),
               'laneNum': int(math.floor(tMsgContent[5])),
               'eventList': eventList}
    json_B3 = json.dumps(dict_B3)
    return json_B3


def getHttpB4Data(tMsgContent, nDeviceId, startTimeStamp):
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
    thisTime = math.floor(time.time())
    thisTime = thisTime - (thisTime % 60)
    thisTimeStamp = getTimeStamp(thisTime)
    dict_B4 = {'deviceId': nDeviceId,
               'startTimeStamp': startTimeStamp,
               'endTimeStamp': thisTimeStamp,
               'eventFrameId': int(math.floor(tMsgContent[6])),
               'laneNum': int(math.floor(tMsgContent[7])),
               'eventList': eventList}
    json_B4 = json.dumps(dict_B4)
    return json_B4, thisTimeStamp


def getHttpB5Data(tMsgContent, nDeviceId, strTimeStamp):
    """B5帧http协议组包"""
    b5eventList = []
    for i in range(tMsgContent[7]):
        targetList = []
        for j in range(tMsgContent[6][i][4]):
            dictTarget = {'targetId': int(math.floor(tMsgContent[6][i][5][j][0]))}
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
    dict_B5 = {'deviceId': nDeviceId,
               'timeStamp': strTimeStamp,
               'eventNum': int(math.floor(tMsgContent[4])),
               'b5eventList': b5eventList}
    json_B5 = json.dumps(dict_B5)
    return json_B5


def getByteofNdarray(npData):
    """获取字节流"""
    byteArray = npData.astype(np.float64).tobytes()
    base64Array = base64.b64encode(byteArray)
    strArray = str(base64Array)[2:-1]
    return strArray


def parseNdarrayofByte(byteData):
    """解析字节流"""
    base64Array = byteData.encode('ascii')
    byteArray = base64.b64decode(base64Array)
    npArray = np.frombuffer(byteArray, np.float64)
    return npArray
