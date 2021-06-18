# -*- coding:utf-8 -*-
from Common.CommonDefine import *
from Common.Interface.Mylog import initLog
from Common.Interface.DataBus import DataBus
from Common.Interface.DataBase import DataBase
from Common.Interface.StateServer import *


class Interface(object):
    def __init__(self, s_ParamPath):
        """
        :param s_log_ParamPath: 配置文件路径
        """
        self.log = initLog(s_ParamPath)
        self.list_channel = get_channel_param(s_ParamPath)
        # REDIS服务
        self.data_bus_server = DataBus(strHost)
        # 初始化状态服务
        self.__st_sys_state = None
        self.q_sys_state = Queue()
        self.q_model_state = Queue()
        self.status_server = StateServer(self.data_bus_server, self.log, self.q_sys_state, self.q_model_state, self)
        # 初始化用户进程
        # self.dict_q_sub = {}
        # for s_channel in self.list_channel:
        #     self.dict_q_sub[s_channel] = Queue()
        self.q_pub_data = Queue()
        self.list_th_sub_data = []
        self.th_pub_data = None
        self.pro_usr = None
        self.log.info('mod init successful!')

    def update_state(self):
        sys_state = self.status_server.get_sys_state()
        if self.__st_sys_state is None:
            self.__st_sys_state = copy.deepcopy(sys_state)
            if self.__st_sys_state.n_sys_state == 1:
                self.start_usr_process()
            elif self.__st_sys_state.n_sys_state == 3:
                self.exit_cur_mod()
        else:
            if self.__st_sys_state.n_sys_state == 0 and sys_state.n_sys_state == 1:
                self.start_usr_process()
            elif self.__st_sys_state.n_sys_state == 0 and sys_state.n_sys_state == 3:
                self.exit_cur_mod()
            elif self.__st_sys_state.n_sys_state == 1 and sys_state.n_sys_state == 2:
                self.stop_usr_process()
            elif self.__st_sys_state.n_sys_state == 1 and sys_state.n_sys_state == 3:
                self.exit_cur_mod()
            if (self.__st_sys_state.n_sys_state == 1 and sys_state.n_sys_state == 0) or \
                    sys_state.n_sys_state < 0 or sys_state.n_sys_state > 3:
                self.status_server.set_sys_state(self.__st_sys_state)
                self.log.error('an error state:{}'.format(sys_state.n_sys_state))
            else:
                self.__st_sys_state = copy.deepcopy(sys_state)
            if self.__st_sys_state.n_sys_state == 2:
                self.__st_sys_state.n_sys_state = 0
        self.log.debug('cur state:{}'.format(self.__st_sys_state.n_sys_state))

    def start_usr_process(self):
        """启动用户进程"""
        if self.pro_usr is not None and self.pro_usr.is_alive():
            self.pro_usr.terminate()
        self.pro_usr = Process(target=self.usr_process, args=(self.data_bus_server,))
        self.pro_usr.start()

        # for s_channel in self.list_channel:
        #     th_suber = threading.Thread(target=self.__th_data_subscriber, args=(s_channel,))
        #     th_suber.daemon = True
        #     th_suber.start()
        #     self.list_th_sub_data.append(th_suber)
        self.th_pub_data = threading.Thread(target=self.__th_data_publisher)
        self.th_pub_data.daemon = True
        self.th_pub_data.start()
        self.log.info('start usr process!')

    def stop_usr_process(self):
        """结束用户进程"""
        if self.pro_usr is not None and self.pro_usr.is_alive():
            self.pro_usr.terminate()
            self.pro_usr = None
            self.log.info('usr process killed!')

        while self.q_pub_data.qsize() > 0:
            try:
                self.q_pub_data.get(True, 0.1)
            except:
                continue
        # for s_channel in self.dict_q_sub:
        #     q_sub_data = self.dict_q_sub[s_channel]
        #     while q_sub_data.qsize() > 0:
        #         try:
        #             q_sub_data.get(True, 0.1)
        #         except:
        #             continue
        self.log.info('stop usr process!')

    def usr_process(self, data_bus_server):
        """用户进程，在继承接口类后需要重写方法"""
        pass

    # def __th_data_subscriber(self, s_channel):
    #     """模块数据订阅线程"""
    #     time.sleep(0.1)
    #     list_sub = self.data_bus_server.get_subscriber([s_channel])
    #     dict_sub_data = {'channel': '', 'data': None, 'time': time.time()}
    #     while self.__st_sys_state.b_mod_running and self.__st_sys_state.n_sys_state == 1:
    #         try:
    #             list_msg = list_sub[0].parse_response(False, 0.1)
    #             dict_sub_data['channel'] = str(list_msg[1], encoding="utf-8")
    #             dict_sub_data['data'] = list_msg[2]
    #             dict_sub_data['time'] = time.time()
    #             self.dict_q_sub[s_channel].put(dict_sub_data)
    #             time.sleep(0.09)
    #         except:
    #             continue
    #     list_sub[0].close()
    #     self.log.warning('data-server th_sub_{} exit...'.format(s_channel))

    def __th_data_publisher(self):
        """模块数据发布线程"""
        time.sleep(0.1)
        while self.__st_sys_state.b_mod_running and self.__st_sys_state.n_sys_state == 1:
            try:
                pub_data = self.q_pub_data.get(True, 0.01)
                self.data_bus_server.publish(pub_data['channel'], pub_data['data'])
            except:
                pass
            time.sleep(0.005)

        self.log.warning('data-server th_data_pub exit...')

    def exit_cur_mod(self):
        """模块数据发布线程"""
        self.status_server.exit_state_server()
        if self.pro_usr is not None and self.pro_usr.is_alive():
            self.pro_usr.terminate()

        while self.q_pub_data.qsize() > 0:
            try:
                self.q_pub_data.get(True, 0.01)
            except:
                continue
        # for s_channel in self.dict_q_sub:
        #     q_sub_data = self.dict_q_sub[s_channel]
        #     while q_sub_data.qsize() > 0:
        #         try:
        #             q_sub_data.get(True, 0.1)
        #         except:
        #             continue
        while self.q_sys_state.qsize() > 0:
            try:
                self.q_sys_state.get(True, 0.01)
            except:
                continue
        while self.q_model_state.qsize() > 0:
            try:
                self.q_model_state.get(True, 0.01)
            except:
                continue
        self.log.info('process exit...')

    def join(self):
        self.status_server.join()
