# -*-coding:utf-8-*-
from Common.Interface.DataBus import DataBus
from WinCommon import getBoxInfo, structData, strHost, QTime
from Common.Protocol import parseNdarrayofByte


class offlineData(object):
    def __init__(self, Mylog, dataTrackQueue, stSysState):
        self.dictOfflineData = {}
        self.nEndFrameId = 0
        self.Mylog = Mylog
        self.dataTrackQueue = dataTrackQueue
        self.stSysState = stSysState
        self.th_offline = None

    def start(self):
        self.th_offline = threading.Thread(target=th_offlineData)
        self.th_offline.daemon = True
        self.th_offline.start()

    def th_offlineData(self):
        timeWait = QTime()
        timeWait.start()
        while self.stSysState.get_running() and (not self.stSysState.get_online()):
            while self.stSysState.get_pause() and (self.stSysState.get_state() in [0, 3]):
                while self.dataTrackQueue.qsize() > 0:
                    try:
                        self.dataTrackQueue.get_nowait()
                    except:
                        pass
                if self.stSysState.get_online():
                    Mylog.warning("exit readOffline 1...")
                    return
                time.sleep(0.01)
            if self.stSysState.get_stip():
                nFrameIdTemp = self.stSysState.get_frameId()
                self.stSysState.set_stip(False)
            bReadOver = False
            if timeWait.elapsed() < 100:
                continue
            timeWait.restart()
            if self.stSysState.get_state() in [0, 3]:  # 开始自动播放
                try:
                    stData = dictFrame[nFrameIdTemp]
                except:
                    nFrameIdTemp = int((nFrameIdTemp + 1) % MAX_FRAME_ID)
                    continue
                self.dataTrackQueue.put([stData, nFrameIdTemp])
                nFrameIdTemp = int((nFrameIdTemp + 1) % MAX_FRAME_ID)
                if nFrameIdTemp > self.nEndFrameId:
                    bReadOver = True
                    break
                while (self.dataTrackQueue.qsize() > 2) and (self.stSysState.get_state() not in [1, 2]):
                    if self.stSysState.get_online():
                        Mylog.warning("exit readOffline 2...")
                        return
                    time.sleep(nTimeOut)  # 防止队列堆积！
                continue

            while (0 < self.stSysState.get_state() <= 2) and (not self.stSysState.get_online()):

                while (self.stSysState.get_state() == 2) and self.stSysState.get_running():
                    if self.stSysState.get_online():
                        Mylog.warning("exit readOffline 3...")
                        return
                    time.sleep(0.01)
                if self.stSysState.get_state() != 1:
                    break
                while self.dataTrackQueue.qsize() > 0:
                    try:
                        toBeDel = self.dataTrackQueue.get_nowait()
                    except:
                        continue
                self.stSysState.set_state(2)
                nFrameIdTemp = self.stSysState.get_frameId()
                if nFrameIdTemp > self.nEndFrameId:
                    bReadOver = True
                try:
                    stData = dictFrame[nFrameIdTemp]
                except:
                    nFrameIdTemp = int((nFrameIdTemp + 1) % MAX_FRAME_ID)
                    continue
                self.dataTrackQueue.put(stData)
                time.sleep(nTimeOut)  # 添加延时，使算法线程获取锁，从而能从队列中get()！
                continue
            if bReadOver:
                break

    def addData(self, stData):
        self.dictOfflineData[stData.nFrameId] = stData
        self.nEndFrameId = stData.nFrameId
        self.stSysState.set_frameCount(self.nEndFrameId)


def offlineSubData(dataTrackQueue, dataEventQueue, stSysState, listSub, nTimeDelay, Mylog):
    offData = offlineData(Mylog, dataTrackQueue, stSysState)
    while stSysState.get_running() and stSysState.get_online():
        for dataSub in listSub:
            list_msg = dataSub.parse_response(False, nTimeDelay)
            if list_msg is None:
                continue
            channel = str(list_msg[1], encoding="utf-8")
            json_data = json.loads(list_msg[2])
            if channel == 'track_data':
                stData = structData()
                byteParticipant = json_data['e1FrameParticipant']
                npParticipant = parseNdarrayofByte(byteParticipant)
                stData.listBoxInfo = getBoxInfo(npParticipant)
                stData.nFrameId = json_data['globalFrameNo']
                stData.nBoxNum = json_data['globalFrameNo']
                stData.tTimeStamp = json_data['globalTimeStamp']
                offData.addData(stData)
            elif channel == 'event_data':
                dataEventQueue.put(json_data)


def onlineSubData(dataTrackQueue, dataEventQueue, stSysState, listSub, nTimeDelay):
    while stSysState.get_running() and stSysState.get_online():
        for dataSub in listSub:
            list_msg = dataSub.parse_response(False, nTimeDelay)
            if list_msg is None:
                continue
            channel = str(list_msg[1], encoding="utf-8")
            json_data = json.loads(list_msg[2])
            if channel == 'track_data':
                stData = structData()
                byteParticipant = json_data['e1FrameParticipant']
                npParticipant = parseNdarrayofByte(byteParticipant)
                stData.listBoxInfo = getBoxInfo(npParticipant)
                stData.nFrameId = json_data['globalFrameNo']
                stData.nBoxNum = json_data['globalFrameNo']
                stData.tTimeStamp = json_data['globalTimeStamp']
                dataTrackQueue.put(stData)
            elif channel == 'event_data':
                dataEventQueue.put(json_data)


def proSubData(dataTrackQueue, dataEventQueue, stSysState, list_subChannel, Mylog):
    nChannelNum = len(list_subChannel)
    nTimeDelay = round(100 / nChannelNum) / 1000

    while stSysState.get_running():
        if stSysState.get_online():
            onlineSubData(dataTrackQueue, dataEventQueue, stSysState, list_subChannel, nTimeDelay)
        else:
            offlineSubData(dataTrackQueue, dataEventQueue, stSysState, list_subChannel, nTimeDelay, Mylog)
