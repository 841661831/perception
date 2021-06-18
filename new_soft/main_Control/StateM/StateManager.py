from Common.CommonDefine import *
from Common.CommonInterface import parse_mod_state, get_sys_state
from AuthorM.Authorization import Authorization


def get_mod_list(s_config_path: str):
    exec_path = get_exe_path()
    list_path = os.listdir(s_config_path)
    main_win = ['win', 'main_Control']
    list_mod = []
    for s_path in list_path:
        s_cfg_path = os.path.join(exec_path + s_config_path[1:], s_path)
        if not judge_exist(s_cfg_path):
            continue
        tree = ET.parse(s_cfg_path)
        root = tree.getroot()
        s_name = root[0][1].text
        if s_name not in main_win:
            list_mod.append(s_name)
    return list_mod


def save_offline_file(s_file_path):
    exec_path = get_exe_path()
    s_cfg_path = os.path.join(exec_path + '/Configs', 'read_Param.xml')
    if not judge_exist(s_file_path) or not judge_exist(s_cfg_path):
        return
    tree = ET.parse(s_cfg_path)
    root = tree.getroot()
    root[1][1].text = s_file_path
    tree.write(s_cfg_path)
    return


class StructSysState:
    """系统状态结构体"""
    def __init__(self):
        self.n_sys_state = 0
        self.b_act_state = False
        self.b_sys_online = False
        self.b_update_param = False
        self.s_localId = ''
        self.n_active_res = 0
        self.b_noSpace = False
        self.s_timeLeft = False
        self.n_cnt = 0


class StructModelState:
    """模块状态结构体"""
    def __init__(self):
        self.n_mod_state = 0
        self.n_pid = 0
        self.n_p_pid = 0
        self.f_cpu_pct = 0.0
        self.f_mem_pct = 0.0
        self.n_th_num = 0
        self.t_last_state = time.time()


class StructWinState:
    """模块状态结构体"""
    def __init__(self):
        self.n_win_state = 0
        self.b_update_param = False
        self.b_sys_online = False
        self.s_act_code = ''
        self.s_offline_file = ''
        self.t_last_state = time.time()


class StateManager(object):
    """模块管理"""
    def __init__(self, data_bus_server, log, parent=None):
        """
        :param data_bus_server: redis服务
        :param log: 日志模块
        :param parent: 父类
        """
        self.b_running = True
        self.__parent = parent
        self.__log = log
        self.__mod_name = log.name
        self.__data_bus_server = data_bus_server
        # 授权模块
        self.__check_Auth = Authorization()
        # 订阅模块状态频道
        self.__list_state_channel = ['win_state', 'mod_state']
        self.__list_sub = self.__data_bus_server.get_subscriber(self.__list_state_channel)
        # 模块状态字典
        self.__dict_mod_state = {}
        self.__win_state = None
        self.__init_mod_state()
        # 系统状态类
        self.__st_sys_state = StructSysState()
        self.__start_state_th()
        self.__log.warning('StateManager Started!')

    def __init_mod_state(self):
        """初始化模块状态字典"""
        list_mod = get_mod_list(os.path.join(get_cfg_path(True), 'modParam/'))
        for mod_name in list_mod:
            self.__dict_mod_state[mod_name] = StructModelState()
        self.__win_state = StructWinState()

    def __start_state_th(self):
        """启动状态服务功能"""
        self.__th_win_state = threading.Thread(target=self.__th_sub_win_state)
        self.__th_win_state.daemon = True
        self.__th_mod_state = threading.Thread(target=self.__th_sub_mod_state)
        self.__th_mod_state.daemon = True
        self.__th_p_state = threading.Timer(0.1, self.__th_pub_state)
        self.__th_p_state.daemon = True
        self.__th_win_state.start()
        self.__th_mod_state.start()
        self.__th_p_state.start()

        self.__th_stateMonitor = threading.Timer(10, self.stateMonitor)
        self.__th_stateMonitor.daemon = True
        self.__th_stateMonitor.start()

    def __th_sub_win_state(self):
        """窗口状态订阅线程"""
        while self.b_running:
            try:
                list_msg = self.__list_sub[0].parse_response(False, 0.1)
                channel = str(list_msg[1], encoding="utf-8")
                json_state = json.loads(list_msg[2])
                self.__win_state.t_last_state = time.time()
                if json_state['winState'] == 4:
                    continue
                self.__win_state.n_win_state = json_state['winState']
                self.__win_state.b_update_param = bool(json_state['paramUpdate'])
                self.__win_state.b_sys_online = bool(json_state['onlineState'])
                self.__win_state.s_act_code = json_state['actCode']
                self.__win_state.s_offline_file = json_state['offlineFile']

                self.__st_sys_state.n_sys_state = self.__win_state.n_win_state
                self.__st_sys_state.b_sys_online = self.__win_state.b_sys_online
                self.__st_sys_state.b_update_param = self.__win_state.b_update_param

                if len(self.__win_state.s_offline_file) != 0:
                    save_offline_file(self.__win_state.s_offline_file)
                self.__win_state.s_offline_file = ''

                time.sleep(0.1)
            except:
                continue

    def __th_sub_mod_state(self):
        """模块状态订阅线程"""
        while self.b_running:
            try:
                list_msg = self.__list_sub[1].parse_response(False, 0.1)
                channel = str(list_msg[1], encoding="utf-8")
                st_mod_state = parse_mod_state(list_msg[2])
                s_mod_name = get_name_from_id(st_mod_state.n_mod_id)
                self.__dict_mod_state[s_mod_name].t_last_state = time.time()
                self.__dict_mod_state[s_mod_name].n_mod_state = st_mod_state.n_mod_state
                self.__dict_mod_state[s_mod_name].n_pid = st_mod_state.n_pid
                self.__dict_mod_state[s_mod_name].n_p_pid = st_mod_state.n_p_pid
                self.__dict_mod_state[s_mod_name].f_cpu_pct = st_mod_state.f_cpu_pct
                self.__dict_mod_state[s_mod_name].f_mem_pct = st_mod_state.f_mem_pct
                self.__dict_mod_state[s_mod_name].n_th_num = st_mod_state.n_th_num
                time.sleep(0.01)
            except:
                continue

    def __th_pub_state(self):
        """系统状态发布线程"""
        tLoop = time.time()
        if math.floor(tLoop - self.__win_state.t_last_state) % 10 == 0:
            if self.__win_state.n_win_state == 3:
                self.b_running = False
                self.__th_p_state.cancel()
            if self.__st_sys_state.n_cnt % 10 == 0:
                if len(self.__win_state.s_act_code) != 0 and not self.__st_sys_state.b_act_state:
                    self.__check_Auth.setActive(self.__win_state.s_act_code)
                self.__check_Auth.checkAuthorization()
                dict_Auth_info = self.__check_Auth.getActiveInfo()
                self.__st_sys_state.b_act_state = dict_Auth_info['act_state']
                self.__st_sys_state.s_localId = dict_Auth_info['localId']
                self.__st_sys_state.n_active_res = dict_Auth_info['active_res']
                self.__st_sys_state.s_timeLeft = dict_Auth_info['time_eft']
                self.__st_sys_state.b_noSpace = dict_Auth_info['space_state']
                self.__st_sys_state.n_cnt = 0

                dict_win_state = {'sysState': self.__st_sys_state.n_sys_state,
                                  'actState': int(self.__st_sys_state.b_act_state),
                                  'onlineState': int(self.__st_sys_state.b_sys_online),
                                  'localId': self.__st_sys_state.s_localId,
                                  'activeState': self.__st_sys_state.n_active_res,
                                  'actTimeLeft': self.__st_sys_state.s_timeLeft,
                                  'noSpace': int(self.__st_sys_state.b_noSpace)}
                json_win_state = json.dumps(dict_win_state)
                self.__data_bus_server.publish('winSys_state', json_win_state)
            self.__st_sys_state.n_cnt += 1
        byte_sys_state = get_sys_state(self.__st_sys_state)
        self.__data_bus_server.publish('sys_state', byte_sys_state)

        if self.b_running:
            self.__th_p_state = threading.Timer(0.1 - (time.time() - tLoop), self.__th_pub_state)
            self.__th_p_state.daemon = True
            self.__th_p_state.start()
        else:
            return

    def stateMonitor(self):
        """系统状态监控线程"""
        tLoop = time.time()
        if getattr(sys, 'frozen', False):
            for mod in self.__dict_mod_state:
                if time.time() - self.__dict_mod_state[mod].t_last_state > 10:
                    if os.system('./main_{} &'.format(mod.capitalize())) == 0:
                        self.__log.warning('{}模块重启...'.format(mod))

        if self.b_running:
            self.__th_stateMonitor = threading.Timer(10 - (time.time() - tLoop), self.stateMonitor)
            self.__th_stateMonitor.daemon = True
            self.__th_stateMonitor.start()
        else:
            self.__log.warning('State Monitor exit...')
            return

    def join(self):
        self.__th_win_state.join()
        self.__log.warning('State Manager exit...')