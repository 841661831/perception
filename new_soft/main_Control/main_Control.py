from Common.CommonDefine import *
from Common.CommonInterface import initLog, DataBus
from StateM.StateManager import StateManager
from DataM.DataManager import DataManager
from AuthorM.Authorization import global_log


s_config_path = os.path.join(get_cfg_path(True), 'modParam/main_Param.xml')


def sigintHandler(signum, frame):
    global_log.warning('Window Closed..')


def get_time_expire(s_param_path: str) -> int:
    """根据配置文件获取频道配置"""
    time_expire = 600
    try:
        if not judge_exist(s_param_path):
            return time_expire
        tree = ET.parse(s_param_path)
        root = tree.getroot()
        time_expire = root[1][2].text
        return time_expire
    except:
        return time_expire


class MainControl(object):
    """主控服务模块，封装"""

    def __init__(self, s_ParamPath):
        """
        :param s_ParamPath: 配置文件路径
        """
        self.log = initLog(s_ParamPath)
        self.list_channel = get_channel_param(s_ParamPath)
        self.expire_time = get_time_expire(s_ParamPath)
        # REDIS服务
        self.data_bus_server = DataBus(strHost)
        # 初始化状态管理
        self.state_manager = StateManager(self.data_bus_server, self.log, self)
        # 初始化数据管理
        self.pro_data = Process(target=self.data_process, daemon=True)
        self.pro_data.start()
        self.log.info('MainControl init successful!')

    def data_process(self):
        data_manager = DataManager(self.data_bus_server, self.log, self.list_channel, self.expire_time)
        data_manager.join()
        time.sleep(1)
        data_manager.dataBase.close()
        self.log.error('redis-server close...')

    def join(self):
        self.state_manager.join()


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, sigintHandler)
    mainControl = MainControl(s_config_path)
    mainControl.join()
    mainControl.log.error('MainControl exit...')
