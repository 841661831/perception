from EventCommon import *
from EventAlg.traffic_flow_matrix import Traffic_Flow


class EventMod(Interface):
    def __init__(self):
        self.s_Parampath = os.path.join(get_cfg_path(), 'modParam/event_Param.xml')
        Interface.__init__(self, self.s_Parampath)

    def usr_process(self, data_bus_server):
        """重写方法"""
        # 功能接口状态类，用于接收接口类中的系统状态，同时上报功能进程的相关信息
        sys_state = UsrSysState(self.q_sys_state, self.q_model_state, self.log)
        # 获取用户订阅的subscriber列表，每个频道对应一个消息，为列表中的一个
        list_sub = data_bus_server.get_subscriber(self.list_channel)
        self.log.info("proEventDetect start")
        stStationParam = lidarParam(os.path.join(get_cfg_path(), 'modParam/read_Param.xml'))
        stStationParam.updateParam()
        stEventParam = structEventAlgParam(os.path.join(get_cfg_path(), 'modParam/event_Param.xml'))
        stEventParam.updateParam()
        stEventModParam = structEventModParam(os.path.join(get_cfg_path(), 'modParam/event_Param.xml'))
        stEventModParam.updateParam()
        traffic_flow = None
        try:
            traffic_flow = Traffic_Flow(os.path.join(get_cfg_path(), 'virtual_config/'),
                                        stStationParam.getStationCount(), stEventParam)
            self.log.info("Traffic_Flow init Successful!")
        except:
            traffic_flow = None
            self.log.error("Traffic_Flow init Failed!\nexcept:\n{}".format(traceback.format_exc()))
        listEventTime = []
        for i in range(6):
            listEventTime.append(MyTime())
            listEventTime[i].start()
        tTimeStamp = 0
        nDeviceId = 0
        thisTime = math.floor(time.time())
        thisTime = thisTime - (thisTime % 60)
        thisTimeStamp = getTimeStamp(thisTime)
        while sys_state.get_sys_state() and traffic_flow is not None:
            if sys_state.get_param_update():
                stEventParam.updateParam()
                print('stEventParam.updateParam()')
                Traffic_Flow.updateParam(stEventParam)
            try:
                list_msg = list_sub[0].parse_response(False, 0.1)
                channel = str(list_msg[1], encoding="utf-8")
                dictData = json.loads(list_msg[2])
                npParticipant = parseNdarrayofByte(dictData['e1FrameParticipant']).reshape((-1, 44))
                trackersToEvent = np.zeros((npParticipant.shape[0],46))
                trackersToEvent[:, 0:44] = npParticipant[:, 0:44]
                strTimeStamp = dictData['globalTimeStamp']
                listDate = strTimeStamp[0].split(' ')
                strYear, strMon, strDay = listDate[0].split('-')
                strHours, strMin, strSec, strMSec = listDate[0].split(':')
                ftime = datetime.datetime(int(strYear), int(strMon), int(strDay),
                                          int(strHours), int(strMin), int(strSec), int(strMSec))
                tTimeStamp = time.mktime(ftime.timetuple()) + int(strMSec) / 1000
                trackersToEvent[:, 44] = tTimeStamp
                nDeviceId = dictData['deviceId']
            except:
                continue
            B0, B1, B2, B3, B4, B5= [], [], [], [], [], []
            try:
                # 进行统计统计量  B1与B4应该同频率
                if math.floor(time.time()) % stEventModParam.B4 == 0 and listEventTime[4].getTime() > 1:  # 1/60Hz
                    traffic_flow.use(trackersToEvent, True)
                else:
                    traffic_flow.use(trackersToEvent, False)
                if not stStationParam.getHighSpeed():
                    if listEventTime[0].getTime() >= stEventModParam.B0 * 1000:  # 10Hz
                        B0 = traffic_flow.get_B0()
                        listEventTime[0].restart()

                    if listEventTime[1].getTime() >= stEventModParam.B1 * 1000:  # 1/60Hz
                        B1 = traffic_flow.get_B1()
                        listEventTime[1].restart()

                    # if listEventTime[2].getTime() >= stEventModParam.B2 * 1000:  # 10Hz
                    #     B2 = traffic_flow.get_B2()
                    #     listEventTime[2].restart()
                else:
                    if listEventTime[3].getTime() >= stEventModParam.B3 * 1000:  # 1Hz
                        B3 = traffic_flow.get_B3()
                        listEventTime[3].restart()

                    if stEventModParam.B4 != 0 and \
                            math.floor(time.time()) % stEventModParam.B4 == 0 and \
                            listEventTime[4].getTime() > 1:  # 1/60Hz
                        B4 = traffic_flow.get_B4()
                        listEventTime[4].restart()

                    B5 = traffic_flow.get_B5()
            except:
                self.log.error("Event detect alg Error !")
                self.log.error("except! Call stack:\n{}".format(traceback.format_exc()))
            json_B3, json_B4, json_B5 = {}, {}, {}
            if len(B3) != 0:
                json_B3 = getHttpB3Data(B3, nDeviceId)
            if len(B4) != 0:
                json_B4, thisTimeStamp = getHttpB4Data(B4, nDeviceId, thisTimeStamp)
            if len(B5) != 0 and B5[4] != 0:
                json_B5 = getHttpB5Data(B5, nDeviceId, strTimeStamp)

            dictEventData = {'b3Data': json_B3,
                             'b4Data': json_B4,
                             'b5Data': json_B5}
            jsonEventData = json.dumps(dictEventData)
            pubData = {'channel': 'event_data', 'data': jsonEventData}
            self.q_pub_data.put(pubData)
        self.log.error('usr_process {} exit...'.format(self.log.name))


if __name__ == '__main__':
    eventMod = EventMod()
    eventMod.join()
    eventMod.log.error('event exit...')
    sys.exit(0)
