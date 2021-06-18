# _*_ coding: utf-8 _*_
"""
Time:     2021/05/11 15:00
Author:   Chen Xingyu
Version:  V 1.0
File:     Mylog.py
Describe: Encapsulating log Module
"""
from Common.CommonDefine import *

# 日志级别
CRITICAL = 50
ERROR = 40
WARNING = 30
INFO = 20
DEBUG = 10
NOTSET = 0

log_default_format = {
    # 终端输出格式
    'std_format': '%(log_color)s %(asctime)s - %(levelname)s - %(filename)s-[line:%(lineno)d] - %(message)s',
    # 日志输出格式
    'file_format': '%(asctime)s - %(levelname)s - %(filename)s-[line:%(lineno)d] - %(message)s'
}
list_colors = ["black", "red", "green", "yellow", "blue", "purple", "cyan", "white"]


class StructLogParam(object):
    """日志配置结构体"""
    def __init__(self):
        self.s_path = ''
        self.s_name = 'mod'
        self.n_file_level = DEBUG
        self.n_std_level = DEBUG
        self.n_count = 7
        self.b_stdout = True
        self.b_fileout = True
        self.std_colors_config = {
            'NOTSET': 'white',
            'DEBUG': 'white',
            'INFO': 'white',
            'WARNING': 'white',
            'ERROR': 'white',
            'CRITICAL': 'white',
        }
        self.__s_current_path = os.path.abspath(logPath_default)
        self.__b_debug = True
        self.__get_current_path()

    def __get_current_path(self):
        if self.__b_debug:
            self.__s_current_path = os.path.abspath(logPath_default)
        self.s_path = os.path.join(self.__s_current_path, self.s_name)
        if not os.path.exists(self.s_path):
            os.makedirs(self.s_path)

    def update(self, s_param_path):
        """根据日志配置文件获取日志配置"""
        if not judge_exist(s_param_path):
            return False
        tree = ET.parse(s_param_path)
        root = tree.getroot()
        self.__s_current_path = root[0][0].text
        self.s_name = root[0][1].text
        self.n_file_level = int(root[0][2].text)
        self.n_std_level = int(root[0][3].text)
        self.n_count = int(root[0][4].text)
        self.b_stdout = bool(int(root[0][5].text))
        self.b_fileout = bool(int(root[0][6].text))
        list_cls_cfg = []
        for tag in root[0][7]:
            list_cls_cfg.append(int(tag.text))
        n_index = 0
        for key in self.std_colors_config:
            self.std_colors_config[key] = list_colors[list_cls_cfg[n_index]]
            n_index += 1
        self.__b_debug = bool(int(root[0][8].text))
        self.__get_current_path()
        return True


class Mylog(logging.Logger):
    """
    封装的日志类,输入为日志配置结构体
    日志输出调用：debug,info,warning,error,critical
    配置文件开放日志置可修改，提供日志初始化
    """
    def __init__(self, stParam):
        self.__s_path = stParam.s_path
        self.__n_count = stParam.n_count
        self.__s_name = stParam.s_name
        self.__n_file_level = stParam.n_file_level
        self.__n_std_level = stParam.n_std_level
        self.__colors_config = stParam.std_colors_config
        self.__b_stdout = stParam.b_stdout
        self.__b_fileout = stParam.b_fileout
        logging.Logger.__init__(self, self.__s_name, self.__n_file_level)
        self.setLevel(logging.DEBUG)  # 设置默认日志记录器记录级别
        self.__constuct_handler()

    def __del__(self):
        """删除日志收集器"""
        if self.__b_stdout:
            self.removeHandler(self.__file_log_handler)  # 避免日志输出重复问题
            self.__close_handler(self.__file_log_handler)
        if self.__b_fileout:
            self.removeHandler(self.__std_log_handle)

    @staticmethod
    def __init_log_handler(s_file_name, n_file_count):
        """
        创建日志记录器handler，用于收集日志
        :return: 日志记录器
        """
        # 设置日志回滚, 保存在log目录, 一天保存一个文件, 保留7天
        file_handler = TimedRotatingFileHandler(filename=s_file_name, when='D', interval=1, backupCount=n_file_count)
        file_handler.suffix = '%Y-%m-%d'
        return file_handler

    @staticmethod
    def __init_std_handler():
        """创建终端日志记录器handler，用于输出到控制台"""
        console_handle = colorlog.StreamHandler()
        return console_handle

    def __set_log_handler(self, logger_handler, level=logging.DEBUG):
        """
        设置handler级别并添加到logger收集器
        :param logger_handler: 日志记录器
        :param level: 日志记录器级别
        """
        logger_handler.setLevel(level=level)
        self.addHandler(logger_handler)

    def __set_std_log_handler(self, std_handler, level=logging.DEBUG):
        """
        设置handler级别并添加到终端logger收集器
        :param std_handler: 终端日志记录器
        :param level: 日志记录器级别
        """
        std_handler.setLevel(level)
        self.addHandler(std_handler)

    @staticmethod
    def __set_std_log_formatter(std_handler, color_config):
        """
        设置输出格式-控制台
        :param std_handler: 终端日志记录器
        :param color_config: 控制台打印颜色配置信息
        :return:
        """
        formatter = colorlog.ColoredFormatter(log_default_format["std_format"], log_colors=color_config)
        std_handler.setFormatter(formatter)

    @staticmethod
    def __set_log_formatter(file_handler):
        """
        设置日志输出格式-日志文件
        :param file_handler: 日志记录器
        """
        formatter = logging.Formatter(log_default_format["file_format"])
        file_handler.setFormatter(formatter)

    @staticmethod
    def __close_handler(file_handler):
        """
        关闭handler
        :param file_handler: 日志记录器
        """
        file_handler.close()

    def __constuct_handler(self):
        """构造日志收集器"""
        s_file_name = os.path.join(self.__s_path, '{name}.log'.format(name=self.__s_name))
        if self.__b_stdout:
            self.__file_log_handler = self.__init_log_handler(s_file_name, self.__n_count)  # 创建日志文件
            self.__set_log_formatter(self.__file_log_handler)  # 设置日志格式
            self.__set_log_handler(self.__file_log_handler, self.__n_file_level)  # 设置handler级别并添加到logger收集器
        if self.__b_fileout:
            self.__std_log_handle = self.__init_std_handler()
            self.__set_std_log_formatter(self.__std_log_handle, self.__colors_config)
            self.__set_std_log_handler(self.__std_log_handle, self.__n_std_level)


def initLog(s_log_path: str):
    """
    日志初始化
    :param s_log_path:日志配置文件路径
    :return: flag,
    """
    stParam = StructLogParam()
    if stParam.update(s_log_path):
        print('日志配置更新成功')
    else:
        print('日志配置更新失败，采用默认日志配置')
    return Mylog(stParam)

if __name__ == '__main__':
    logger = initLog('./logParam.xml')
    logger.debug('this is a debug1 log')
    logger.info('this is a info log')
    logger.warning('this is a info log')
    logger.error('this is a error log')
    logger.critical('this is a critical log')
