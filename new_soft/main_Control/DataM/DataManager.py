from Common.CommonDefine import *
from Common.CommonInterface import StructSysState, parse_sys_state, DataBase


class DataManager(object):
    """数据管理"""

    def __init__(self, data_bus_server, log, list_channel, expire_time):
        """
        :param data_bus_server: redis服务
        :param log: 日志模块
        :param list_channel: 订阅频道列表
        :param expire_time: 数据过期时间
        """
        self.b_running = True
        self.__list_channel = list_channel
        self.__log = log
        self.__mod_name = log.name
        self.__data_bus_server = data_bus_server
        # 订阅数据频道
        self.__list_sub = self.__data_bus_server.get_subscriber(self.__list_channel)
        # 订阅线程列表
        self.list_sub_th = []
        # 订阅线程列表
        self.dataBase = DataBase(strHost, expire_time)
        # 订阅系统状态
        self.__sys_state = StructSysState()
        self.__sys_state_channel = 'sys_state'
        self.__sub_sys_state = self.__data_bus_server.get_subscriber([self.__sys_state_channel])
        self.__th_sub_state = threading.Thread(target=self.__th_sub_sys_state)
        self.__th_sub_state.daemon = True
        self.__th_sub_state.start()
        self.__log.warning('DataManager Started!')

    def __start_state_th(self):
        """启动状态服务功能"""
        for i in range(len(self.__list_sub)):
            th_tmp = threading.Thread(target=self.th_sub_data, args=(i,))
            th_tmp.daemon = True
            th_tmp.start()
            self.list_sub_th.append(th_tmp)

    def th_sub_data(self, index):
        while self.__sys_state.b_mod_running:
            list_msg = self.__list_sub[index].parse_response()
            channel = str(list_msg[1], encoding="utf-8")
            dict_data = json.loads(list_msg[2])
            self.save2db(channel, dict_data)

    def save2db(self, channel, dict_data):
        """
        存数据库
        :param channel: 频道
        :param dict_data: 对应的数据
        """
        date = datetime.datetime.now().strftime("%Y%m%d")
        if channel == 'track_data':
            n_frameId = dict_data['globalFrameNo']
            byteParticipant = dict_data['e1FrameParticipant']
            npParticipant = np.frombuffer(byteParticipant, dtype=np.float64).reshape((-1, 44))[:, 0:14]
            for i in range(dict_data['participantNum']):
                targetId = int(npParticipant[i, 9])
                byte_info = npParticipant[i, :].tobytes()
                self.dataBase.write(date, 'E1', n_frameId, byte_info, targetId=targetId)
        elif channel == 'event_data':
            b4_data = dict_data['b4Data']
            if len(b4_data) != 0:
                b4_eventId = b4_data['eventFrameId']
                self.dataBase.write(date, 'B4', b4_eventId, json.dumps(b4_data))
            b5_data = dict_data['b5Data']
            if len(b5_data) != 0:
                b5_eventList = b5_data['b5eventList']
                for i in range(b5_data['eventNum']):
                    b5_eventId = b5_eventList[i]['eventId']
                    self.dataBase.write(date, 'B5', b5_eventId, json.dumps(b5_eventList[i]))

    def __th_sub_sys_state(self):  # 定义发布方法
        """系统状态订阅线程"""
        while self.__sys_state.b_mod_running:
            list_msg = self.__sub_sys_state[0].parse_response()
            bytes_msg = list_msg[2]
            list_state = parse_sys_state(bytes_msg)
            self.__sys_state.n_sys_state = list_state[0]
            if self.__sys_state.n_sys_state == 3:
                self.__sys_state.b_mod_running = False
            self.__sys_state.b_act_state = list_state[1]
            self.__sys_state.b_sys_online = list_state[2]
            self.__sys_state.b_update_param = list_state[3]
            time.sleep(0.1)
        self.__log.warning('state-server th_sub exit...')

    def join(self):
        self.__th_sub_state.join()
        self.__log.warning('Data Manager exit...')
