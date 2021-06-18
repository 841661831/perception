# -*-coding:utf-8-*-
from Common.CommonInterface import Interface, UsrSysState
from Common.CommonParam import lidarParam, structEventAlgParam, MyTime
from Common.CommonDefine import *
from Common.Protocol import *


class structEventModParam:
    def __init__(self, s_cfg_path):
        self.s_cfg_path = s_cfg_path
        self.B0 = 0
        self.B1 = 0
        self.B2 = 0
        self.B3 = 0
        self.B4 = 0
        self.B5 = 0

    def updateParam(self):
        try:
            if not judge_exist(self.s_cfg_path):
                return
            tree = ET.parse(self.s_cfg_path)
            root = tree.getroot()
            self.B0 = int(root[1][1][0].text)
            self.B1 = int(root[1][1][1].text)
            self.B2 = int(root[1][1][2].text)
            self.B3 = int(root[1][1][3].text)
            self.B4 = int(root[1][1][4].text)
            self.B5 = int(root[1][1][5].text)
        except:
            print("get EventModParam Failed!\n{}\n".format(traceback.format_exc()))
