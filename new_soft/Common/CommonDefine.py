import redis
import logging
import json
import threading
from logging.handlers import TimedRotatingFileHandler
import colorlog
import datetime
import copy
import time
import psutil
from multiprocessing import Queue, Process, freeze_support
from subprocess import Popen, PIPE
import traceback
import struct
import math
import sys
import os
import shutil
import xml.etree.ElementTree as ET
import socket
import datetime
import numpy as np
import shlex
import signal
import base64
import cv2

PI_rads = math.pi / 180
strHost = socket.gethostbyname(socket.gethostname())
logPath_default = './log'
dict_module_id = {'mod': 0, 'read': 1, 'track': 2, 'event': 3, 'send': 4}
switch_color = {0: "None", 1: "white", 2: "black", 3: "red", 4: "Silver", 5: "yellow", 6: "blue", 7: "multicolor",
                8: "brown", 9: "gray"}
switch_send = {0: "None", 1: "car", 2: "truck", 3: "bus", 4: "person", 5: "bicycle", 6: "motorbike", 7: "Minibus"}
switch_pc = {0: "car", 1: "bicycle", 2: "bus", 3: "motorbike", 4: "person", 5: "cone", 6: "truck", 7: "None",
             8: "Minibus"}


def get_name_from_id(n_id):
    s_name = 'mod'
    for name in dict_module_id:
        if dict_module_id[name] == n_id:
            return name
    return s_name


def get_size(obj, seen=None) -> int:
    """Recursively finds size of objects"""
    size = sys.getsizeof(obj)
    if seen is None:
        seen = set()
    obj_id = id(obj)
    if obj_id in seen:
        return 0
    # Important mark as seen *before* entering recursion to gracefully handle
    # self-referential objects
    seen.add(obj_id)
    if isinstance(obj, dict):
        size += sum([get_size(v, seen) for v in obj.values()])
        size += sum([get_size(k, seen) for k in obj.keys()])
    elif hasattr(obj, '__dict__'):
        size += get_size(obj.__dict__, seen)
    elif hasattr(obj, '__iter__') and not isinstance(obj, (str, bytes, bytearray)):
        size += sum([get_size(i, seen) for i in obj])
    return size


def get_exe_path() -> str:
    """获取执行目录"""
    if getattr(sys, 'frozen', False):  # 查找 sys 中有没 frozen 属性，如果没有返回Fasle。
        # sys中的 'frozen' 属性 是打包成 一个EXE文件特有的属性。
        # print("case 1")
        bundle_dir = sys._MEIPASS
    else:
        # 这是在没打包成一个EXE文件的情况下，文件的当前路径。
        # bundle_dir = os.path.dirname(os.path.abspath(__file__))
        bundle_dir = os.getcwd()
    return bundle_dir


def get_cfg_path(mainControl=False) -> str:
    """获取配置文件目录"""
    if getattr(sys, 'frozen', False):  # 查找 sys 中有没 frozen 属性，如果没有返回Fasle。
        s_config_path = './Configs/'
    else:
        if mainControl:
            s_config_path = '../Configs/'
        else:
            s_config_path = '../../Configs/'
    return s_config_path


def judge_exist(path: str) -> bool:
    """判断文件、文件夹是否存在"""
    path = path.strip()
    path = path.rstrip("\\")
    isExists = os.path.exists(path)
    return isExists


def cre_dir(path: str) -> bool:
    """创建文件夹"""
    isExists = judge_exist(path)
    if not isExists:
        try:
            os.makedirs(path)
        except:
            isExists = judge_exist(path)
        if not isExists:
            return False
        return True
    return True


def del_dir(path: str) -> bool:
    """删除文件夹"""
    try:
        isExists = judge_exist(path)
        if isExists:
            if os.path.isdir(path):
                shutil.rmtree(path, ignore_errors=True)
                return True
            else:
                return False
    except:
        return False


def my_renameFile(oldFilePath, newFilePath, bOverWrite=False):
    """文件改名"""
    isOldExists = judge_exist(oldFilePath)
    isNewExists = judge_exist(newFilePath)
    if isOldExists and (bOverWrite or (not isNewExists)):
        os.rename(oldFilePath, newFilePath)


def get_channel_param(s_param_path: str) -> list:
    """根据配置文件获取频道配置"""
    list_sub_channel = []
    try:
        if not judge_exist(s_param_path):
            return list_sub_channel
        tree = ET.parse(s_param_path)
        root = tree.getroot()
        for tag in root[1][0]:
            list_sub_channel.append(tag.text)
        return list_sub_channel
    except:
        return list_sub_channel


def num2str0(num: int, cnt=2) -> str:
    """数字转换为字符串"""
    if cnt == 2:
        if num < 10:
            return '0' + str(num)
        else:
            return str(num)
    elif cnt == 3:
        if num < 10:
            return '00' + str(num)
        elif num < 100:
            return '0' + str(num)
        else:
            return str(num)


def getTimeStamp(timeStamp: float) -> str:
    """获取字符串时间"""
    time_stamp_temp1, time_stamp_temp2 = math.floor(timeStamp), int((timeStamp - math.floor(timeStamp)) * 1e3)
    localtime = time.localtime(time_stamp_temp1)
    strTime = '{}-{}-{} {}:{}:{}:{}'.format(str(localtime.tm_year), num2str0(localtime.tm_mon),
                                            num2str0(localtime.tm_mday),
                                            num2str0(localtime.tm_hour), num2str0(localtime.tm_min),
                                            num2str0(localtime.tm_sec),
                                            num2str0(time_stamp_temp2, cnt=3))
    return strTime


def process_info(p_name):
    """进程信息线程"""
    cre_dir('./pro_info/')
    if not judge_exist(r'./pro_info/{}.csv'.format(p_name)):
        with open(r'./pro_info/{}.csv'.format(p_name), 'a') as proinfo_file:
            proinfo_file.write(
                '{},{},{},{},{},{},{}\n'.format('time', 'pro_name', 'pid', 'p_pid', 'cpu_pct', 'mem_pct', 'th_num'))
    t_test = time.time()
    while True:
        if time.time() - t_test > 30:
            break
        pid = os.getpid()
        pro_info = psutil.Process(pid)
        sys_info = {'name': p_name,
                    'pid': pid,
                    'p_pid': pro_info.ppid(),
                    'cpu_pct': round(pro_info.cpu_percent(interval=1), 2),
                    'mem_pct': round(pro_info.memory_percent(), 2),
                    'th_num': round(pro_info.num_threads(), 2)}
        cre_dir('./pro_info/')

        with open(r'./pro_info/{}.csv'.format(p_name), 'a') as proinfo_file:
            ttime = time.time()
            strTime = getTimeStamp(ttime)
            proinfo_file.write('{},{},{},{},{}%,{}%,{}\n'.
                               format(strTime, sys_info['name'], sys_info['pid'],
                                      sys_info['p_pid'], sys_info['cpu_pct'],
                                      sys_info['mem_pct'], sys_info['th_num']))


def getBccCheckVal(byte_data):
    """获取奇偶校验位"""
    checkVal = 0x00
    for i in range(len(byte_data)):
        checkVal ^= byte_data[i]
    return checkVal


# 根据pid递归查找所有关联进程的pid
def getPidTreeByPid(pid, procs, pidTree):
    root = [p for p in procs if p['pid'] == pid][0]
    if root['pid'] not in pidTree:
        pidTree.append(root['pid'])
    if root['ppid'] not in pidTree:
        pidTree.append(root['ppid'])

    childs = [proc for proc in procs if proc['ppid'] == pid]
    if childs:
        for c in childs:
            getPidTreeByPid(c['pid'], procs, pidTree)


def split(s):
    s = s.split()
    return str(s[0], encoding="utf-8"), str(s[1], encoding="utf-8"), str(s[2], encoding="utf-8")


def get_ps():
    """获取ps结果"""
    cmd = 'ps ax -o pid,ppid,cmd'
    p = Popen(shlex.split(cmd), stdout=PIPE)
    procs = []
    for l in p.stdout.readlines()[1:]:
        pid, ppid, cmd = [i.strip() for i in split(l)]
        procs.append({'pid': int(pid), 'ppid': int(ppid), 'cmd': cmd})
    return procs


def killProExceptSelf():
    """杀掉除自己进程树以外的所有同名进程"""
    # 查找自己的关联进程
    pidTree = []
    procs = get_ps()
    currentId = os.getpid()
    getPidTreeByPid(currentId, procs, pidTree)
    # 更具当前进程的id，获取当前进程对象
    proc = psutil.Process(pid=currentId)
    # 查找所有与当前进程同名的进程id
    allPyIds = [p.pid for p in psutil.process_iter() if proc.name() in str(p.name)]
    # 去除自己以及自己的关联进程后，才是真正需要杀掉的进程
    PyIdsToKill = [x for x in allPyIds if x not in pidTree]
    if len(PyIdsToKill) > 0:
        for PyId in PyIdsToKill:
            os.kill(PyId, signal.SIGTERM)


def killSelfSamePro():
    """杀掉除自己以外的所有同名进程"""
    currentId = os.getpid()
    # 更具当前进程的id，获取当前进程对象
    proc = psutil.Process(pid=currentId)
    # 查找所有与当前进程同名的进程id
    allPyIds = [p.pid for p in psutil.process_iter() if proc.name() in str(p.name)]
    allPyIds.remove(currentId)
    # 去除自己以及自己的关联进程后，才是真正需要杀掉的进程
    if len(allPyIds) > 0:
        for PyId in allPyIds:
            os.kill(PyId, signal.SIGTERM)
