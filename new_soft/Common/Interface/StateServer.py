from Common.CommonDefine import *

dict_sys_state = {0: 'waiting', 1: 'running', 2: 'stop', 3: 'exit'}
dict_act_state = {0: 'unactivation', 1: 'activation'}
dict_sys_online = {0: 'offline', 1: 'online'}


def get_byte_state(st_info):
    """获取模块状态字节流"""
    byte_head = struct.pack('BB', 255, 255)
    byte_mod_info = struct.pack('B', int(st_info.n_mod_id)) + \
                    struct.pack('B', int(st_info.n_mod_state)) + \
                    struct.pack('>H', int(st_info.n_pid)) + \
                    struct.pack('>H', int(st_info.n_p_pid)) + \
                    struct.pack('>H', int(math.floor(st_info.f_cpu_pct * 10))) + \
                    struct.pack('>H', int(math.floor(st_info.f_mem_pct * 10))) + \
                    struct.pack('B', int(st_info.n_th_num))
    byte_len = struct.pack('>H', int(len(byte_mod_info)))
    byte_check = byte_head + byte_len + byte_mod_info
    byte_bcc = struct.pack('B', getBccCheckVal(byte_check))
    byte_tail = struct.pack('B', 255)
    return byte_check + byte_bcc + byte_tail


def parse_mod_state(byte_data):
    """解析模块状态字节流"""
    try:
        # len_state = len(byte_data)
        # byte_head = struct.unpack('BB', byte_data[0:2])
        # byte_len = struct.unpack('>H', byte_data[2:4])
        byte_mod_id = struct.unpack('B', byte_data[4:5])
        byte_state = struct.unpack('B', byte_data[5:6])
        byte_pid = struct.unpack('B', byte_data[6:7])
        byte_p_pid = struct.unpack('>H', byte_data[7:9])
        byte_cpu_pct = struct.unpack('>H', byte_data[9:11])
        byte_mem_pct = struct.unpack('>H', byte_data[11:13])
        byte_th_num = struct.unpack('>H', byte_data[13:15])
        # byte_bcc = struct.unpack('B', byte_data[15:16])
        # byte_tail = struct.unpack('B', byte_data[16:17])

        st_mod_state = StructModelState()
        st_mod_state.n_mod_id = byte_mod_id[0]
        st_mod_state.n_mod_state = byte_state[0]
        st_mod_state.n_pid = byte_pid[0]
        st_mod_state.n_p_pid = byte_p_pid[0]
        st_mod_state.f_cpu_pct = byte_cpu_pct[0] / 10
        st_mod_state.f_mem_pct = byte_mem_pct[0] / 10
        st_mod_state.n_th_num = byte_th_num[0]

        return st_mod_state
    except:
        return None


def get_sys_state(st_info):
    """获取模块状态字节流"""
    byte_head = struct.pack('BB', 255, 255)
    byte_mod_info = struct.pack('B', int(st_info.n_sys_state)) + \
                    struct.pack('B', int(st_info.b_act_state)) + \
                    struct.pack('B', int(st_info.b_sys_online)) + \
                    struct.pack('B', int(st_info.b_update_param))
    byte_len = struct.pack('>H', int(len(byte_mod_info)))
    byte_check = byte_head + byte_len + byte_mod_info
    byte_bcc = struct.pack('B', getBccCheckVal(byte_check))
    byte_tail = struct.pack('B', 255)
    return byte_check + byte_bcc + byte_tail


def parse_sys_state(byte_data):
    """解析xi状态字节流"""
    try:
        # len_state = len(byte_data)
        # byte_head = struct.unpack('BB', byte_data[0:2])
        # byte_len = struct.unpack('>H', byte_data[2:4])
        byte_sys_state = struct.unpack('B', byte_data[4:5])
        byte_act_state = struct.unpack('B', byte_data[5:6])
        byte_sys_online = struct.unpack('B', byte_data[6:7])
        byte_update = struct.unpack('B', byte_data[7:8])
        # byte_bcc = struct.unpack('B', byte_data[8:9])
        # byte_tail = struct.unpack('B', byte_data[9:10])

        return [int(byte_sys_state[0]), bool(byte_act_state[0]),
                bool(byte_sys_online[0]), bool(byte_update[0])]
    except:
        return None


class StructSysState:
    """系统状态结构体"""
    def __init__(self):
        self.n_sys_state = 0
        self.b_act_state = False
        self.b_sys_online = False
        self.b_mod_running = True
        self.b_get_state = True
        self.b_update_param = False


class StructModelState:
    """模块状态结构体"""
    def __init__(self):
        self.n_mod_id = 0
        self.n_mod_state = 0
        self.n_pid = 0
        self.n_p_pid = 0
        self.f_cpu_pct = 0.0
        self.f_mem_pct = 0.0
        self.n_th_num = 0


class UsrSysState(object):
    """用户进程状态类"""
    def __init__(self, q_sys_state, q_mod_state, log):
        self.b_sys_state = True
        self.b_online = True
        self.b_update_param = False
        self.__log = log
        self.__pro_info = None
        self.__q_sys_state = q_sys_state
        self.__q_mod_state = q_mod_state
        self.__th_get = threading.Thread(target=self.__th_get_sys_state)
        self.__th_get.daemon = True
        self.__th_send = threading.Thread(target=self.__th_send_mod_state)
        self.__th_send.daemon = True
        self.__th_pro_info = threading.Thread(target=self.__process_info)
        self.__th_pro_info.daemon = True

        self.__th_get.start()
        self.__th_send.start()
        self.__th_pro_info.start()

    def __th_get_sys_state(self):
        while self.b_sys_state:
            if self.__q_sys_state.qsize() > 0:
                try:
                    sys_state = self.__q_sys_state.get_nowait()
                    self.b_online = sys_state.b_sys_online
                    self.b_update_param = sys_state.b_update_param
                    if sys_state.n_sys_state == 1:
                        self.b_sys_state = True
                    else:
                        self.b_sys_state = False
                except:
                    continue
            time.sleep(0.1)
        self.__log.warning('usr-state_server th_get exit...')

    def __th_send_mod_state(self):
        while self.b_sys_state:
            if self.__pro_info is not None:
                byte_mod_info = get_byte_state(self.__pro_info)
                self.__q_mod_state.put(byte_mod_info)
            self.__pro_info = None
            time.sleep(0.1)
        self.__log.warning('usr-state_server th_send exit...')

    def __process_info(self):
        while self.b_sys_state:
            pid = os.getpid()
            pro_info = psutil.Process(pid)
            # 获取模块状态
            st_info = StructModelState()
            st_info.n_mod_id = dict_module_id[self.__log.name]
            st_info.n_mod_state = 1
            st_info.n_pid = pid
            st_info.n_p_pid = pro_info.ppid()
            st_info.f_cpu_pct = round(pro_info.cpu_percent(interval=1), 2)
            st_info.f_mem_pct = round(pro_info.memory_percent(), 2)
            st_info.n_th_num = round(pro_info.num_threads(), 2)

            self.__pro_info = copy.deepcopy(st_info)
        self.__log.warning('usr-state_server th_pro_info exit...')

    def get_sys_state(self):
        return self.b_sys_state

    def get_sys_online(self):
        return self.b_online

    def get_param_update(self):
        return self.b_update_param


class StateServer(object):
    """模块状态服务"""
    def __init__(self, data_bus_server, log, q_sys_state, q_model_state, parent=None):
        """
        :param data_bus_server: redis服务
        :param log: 日志模块
        :param q_sys_state: 日志队列
        """
        self.__parent = parent
        self.__log = log
        self.__mod_name = log.name
        self.__mod_state_channel = 'mod_state'
        self.__q_sys_state = q_sys_state
        self.__q_model_state = q_model_state
        self.__data_bus_server = data_bus_server
        # 订阅系统状态
        self.__sys_state = StructSysState()
        self.__sys_state_channel = 'sys_state'
        self.__sub_sys_state = self.__data_bus_server.get_subscriber([self.__sys_state_channel])
        self.__th_sub_state = None
        self.__th_pub_state = None
        self.__start_state_th()

    def __start_state_th(self):
        """启动状态服务功能"""
        self.__th_sub_state = threading.Thread(target=self.__th_sub_sys_state)
        self.__th_sub_state.daemon = True
        self.__th_pub_state = threading.Thread(target=self.__th_pub_mod_state)
        self.__th_pub_state.daemon = True
        self.__th_sub_state.start()
        self.__th_pub_state.start()

    def __th_sub_sys_state(self):  # 定义发布方法
        """系统状态订阅线程"""
        n_act_cnts = 0
        while len(self.__sub_sys_state) == 1 and self.__sys_state.b_mod_running:
            list_msg = self.__sub_sys_state[0].parse_response()
            # str_channel = list_msg[1]
            t = time.time()
            bytes_msg = list_msg[2]
            list_state = parse_sys_state(bytes_msg)
            if self.__log.name != 'win':
                self.__sys_state.n_sys_state = list_state[0]
                if self.__sys_state.n_sys_state == 3:
                    self.__sys_state.b_mod_running = False
                self.__sys_state.b_act_state = list_state[1]
                self.__sys_state.b_sys_online = list_state[2]
                self.__sys_state.b_update_param = list_state[3]

                if not self.__sys_state.b_act_state and self.__sys_state.n_sys_state != 3:
                    self.__sys_state.n_sys_state = 0
                    if n_act_cnts % 10 == 0:
                        self.__log.warning('Software is not activation,please active software in Window!')
                        n_act_cnts = 0
                    n_act_cnts += 1
                if self.__log.name == 'win':
                    self.__sys_state.n_sys_state = 1
                self.__log.debug('get sys state:'.format(self.__sys_state.n_sys_state))
                self.__log.debug('get sys active:'.format(self.__sys_state.b_act_state))
                self.__log.debug('get sys online:'.format(self.__sys_state.b_sys_online))
                if self.__parent is not None:
                    self.__parent.update_state()
                if self.__sys_state.n_sys_state == 1:
                    self.__q_sys_state.put(self.__sys_state)
            else:
                self.__sys_state.n_sys_state = list_state[0]
                self.__sys_state.b_act_state = list_state[1]
                self.__sys_state.b_sys_online = list_state[2]
                self.__sys_state.b_update_param = list_state[3]

            time.sleep(0.1)
        self.__sub_sys_state[0].close()
        self.__log.warning('state-server th_sub exit...')

    def __th_pub_mod_state(self):  # 定义订阅方法
        """模块状态发布线程"""
        while self.__sys_state.b_mod_running:
            try:
                if self.__sys_state.n_sys_state == 1:
                    byte_mod_state = self.__q_model_state.get(True, 0.01)
                    if byte_mod_state is not None:
                        time.sleep(0.09)
                        self.__data_bus_server.publish(self.__mod_state_channel, byte_mod_state)
                else:
                    st_info = StructModelState()
                    st_info.n_mod_id = dict_module_id[self.__log.name]
                    st_info.n_mod_state = 1
                    byte_mod_info = get_byte_state(st_info)
                    self.__data_bus_server.publish(self.__mod_state_channel, byte_mod_info)
            except:
                continue
            time.sleep(0.01)
        self.__log.warning('state-server th_pub exit...')

    def get_sys_state(self):
        """获取系统状态"""
        return self.__sys_state

    def set_sys_state(self, sys_state):
        """接收到的系统状态存在错误，进行修正"""
        self.__sys_state = copy.deepcopy(sys_state)

    def join(self):
        """等待线程退出"""
        self.__th_sub_state.join()

    def exit_state_server(self):
        """退出状态服务"""
        self.__log.warning('state-server exit...')
        return
