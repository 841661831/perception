# -*-coding:utf-8-*-
from Common.CommonDefine import *

class structEventProperty:
    """事件检测算法事件属性参数"""
    def __init__(self):
        self.b_enable = True
        self.f_responseTime = 0.0
        self.f_limitVal = 0.0

    def setVal(self, listVal):
        self.b_enable = listVal[0]
        self.f_responseTime = listVal[1]
        if len(listVal) == 3:
            self.f_limitVal = listVal[2]

    def getVal(self):
        return self.b_enable, self.f_responseTime, self.f_limitVal


class structEventAlgParam:
    """事件检测算法参数"""
    def __init__(self, s_cfg_path):
        self.s_cfg_path = s_cfg_path
        self.occupy_dedicated_lane = structEventProperty()
        self.people_occupy_motor_lane = structEventProperty()
        self.retorgrade = structEventProperty()
        self.speeding = structEventProperty()
        self.stroll = structEventProperty()
        self.cross_line = structEventProperty()
        self.illegal_stop = structEventProperty()
        self.cross_lane = structEventProperty()
        self.run_the_red_light = structEventProperty()
        self.occupy_bus_lane = structEventProperty()
        self.changes_lane = structEventProperty()
        self.spills_detect = structEventProperty()
        self.accident_detect = structEventProperty()
        self.occupy_emergency_area = structEventProperty()

    def updateParam(self):
        """从配置文件读取参数"""
        try:
            if not judge_exist(self.s_cfg_path):
                return
            tree = ET.parse(self.s_cfg_path)
            root = tree.getroot()
            self.occupy_dedicated_lane.b_enable = bool(int(root[2][0][0].text))
            self.occupy_dedicated_lane.f_responseTime = float(root[2][0][1].text)
            self.occupy_dedicated_lane.f_limitVal = float(root[2][0][2].text)
            self.people_occupy_motor_lane.b_enable = bool(int(root[2][1][0].text))
            self.people_occupy_motor_lane.f_responseTime = float(root[2][1][1].text)
            self.people_occupy_motor_lane.f_limitVal = float(root[2][1][2].text)
            self.retorgrade.b_enable = bool(int(root[2][2][0].text))
            self.retorgrade.f_responseTime = float(root[2][2][1].text)
            self.retorgrade.f_limitVal = float(root[2][2][2].text)
            self.speeding.b_enable = bool(int(root[2][3][0].text))
            self.speeding.f_responseTime = float(root[2][3][1].text)
            self.speeding.f_limitVal = float(root[2][3][2].text)
            self.stroll.b_enable = bool(int(root[2][4][0].text))
            self.stroll.f_responseTime = float(root[2][4][1].text)
            self.stroll.f_limitVal = float(root[2][4][2].text)
            self.cross_line.b_enable = bool(int(root[2][5][0].text))
            self.cross_line.f_responseTime = float(root[2][5][1].text)
            self.cross_line.f_limitVal = float(root[2][5][2].text)
            self.illegal_stop.b_enable = bool(int(root[2][6][0].text))
            self.illegal_stop.f_responseTime = float(root[2][6][1].text)
            self.illegal_stop.f_limitVal = float(root[2][6][2].text)
            self.cross_lane.b_enable = bool(int(root[2][7][0].text))
            self.cross_lane.f_responseTime = float(root[2][7][1].text)
            self.cross_lane.f_limitVal = float(root[2][7][2].text)
            self.run_the_red_light.b_enable = bool(int(root[2][8][0].text))
            self.run_the_red_light.f_responseTime = float(root[2][8][1].text)
            self.run_the_red_light.f_limitVal = float(root[2][8][2].text)
            self.occupy_bus_lane.b_enable = bool(int(root[2][9][0].text))
            self.occupy_bus_lane.f_responseTime = float(root[2][9][1].text)
            self.occupy_bus_lane.f_limitVal = float(root[2][9][2].text)
            self.changes_lane.b_enable = bool(int(root[2][10][0].text))
            self.changes_lane.f_responseTime = float(root[2][10][1].text)
            self.changes_lane.f_limitVal = float(root[2][10][2].text)
            self.spills_detect.b_enable = bool(int(root[2][11][0].text))
            self.spills_detect.f_responseTime = float(root[2][11][1].text)
            self.spills_detect.f_limitVal = float(root[2][11][2].text)
            self.accident_detect.b_enable = bool(int(root[2][12][0].text))
            self.accident_detect.f_responseTime = float(root[2][12][1].text)
            self.accident_detect.f_limitVal = float(root[2][12][2].text)
            self.occupy_emergency_area.b_enable = bool(int(root[2][13][0].text))
            self.occupy_emergency_area.f_responseTime = float(root[2][13][1].text)
            self.occupy_emergency_area.f_limitVal = float(root[2][13][2].text)
        except:
            global_log.error("getEventParam Failed!\n{}\n".format(traceback.format_exc()))

    def saveParam(self):
        """将参数写入配置文件"""
        if not judge_exist(self.s_cfg_path):
            return
        tree = ET.parse(self.s_cfg_path)
        root = tree.getroot()
        root[2][0][0].text = str(int(self.occupy_dedicated_lane.b_enable))
        root[2][0][1].text = str(self.occupy_dedicated_lane.f_responseTime)
        root[2][0][2].text = str(self.occupy_dedicated_lane.f_limitVal)
        root[2][1][0].text = str(int(self.people_occupy_motor_lane.b_enable))
        root[2][1][1].text = str(self.people_occupy_motor_lane.f_responseTime)
        root[2][1][2].text = str(self.people_occupy_motor_lane.f_limitVal)
        root[2][2][0].text = str(int(self.retorgrade.b_enable))
        root[2][2][1].text = str(self.retorgrade.f_responseTime)
        root[2][2][2].text = str(self.retorgrade.f_limitVal)
        root[2][3][0].text = str(int(self.speeding.b_enable))
        root[2][3][1].text = str(self.speeding.f_responseTime)
        root[2][3][2].text = str(self.speeding.f_limitVal)
        root[2][4][0].text = str(int(self.stroll.b_enable))
        root[2][4][1].text = str(self.stroll.f_responseTime)
        root[2][4][2].text = str(self.stroll.f_limitVal)
        root[2][5][0].text = str(int(self.cross_line.b_enable))
        root[2][5][1].text = str(self.cross_line.f_responseTime)
        root[2][5][2].text = str(self.cross_line.f_limitVal)
        root[2][6][0].text = str(int(self.illegal_stop.b_enable))
        root[2][6][1].text = str(self.illegal_stop.f_responseTime)
        root[2][6][2].text = str(self.illegal_stop.f_limitVal)
        root[2][7][0].text = str(int(self.cross_lane.b_enable))
        root[2][7][1].text = str(self.cross_lane.f_responseTime)
        root[2][7][2].text = str(self.cross_lane.f_limitVal)
        root[2][8][0].text = str(int(self.run_the_red_light.b_enable))
        root[2][8][1].text = str(self.run_the_red_light.f_responseTime)
        root[2][8][2].text = str(self.run_the_red_light.f_limitVal)
        root[2][9][0].text = str(int(self.occupy_bus_lane.b_enable))
        root[2][9][1].text = str(self.occupy_bus_lane.f_responseTime)
        root[2][9][2].text = str(self.occupy_bus_lane.f_limitVal)
        root[2][10][0].text = str(int(self.changes_lane.b_enable))
        root[2][10][1].text = str(self.changes_lane.f_responseTime)
        root[2][10][2].text = str(self.changes_lane.f_limitVal)
        root[2][11][0].text = str(int(self.spills_detect.b_enable))
        root[2][11][1].text = str(self.spills_detect.f_responseTime)
        root[2][11][2].text = str(self.spills_detect.f_limitVal)
        root[2][12][0].text = str(int(self.accident_detect.b_enable))
        root[2][12][1].text = str(self.accident_detect.f_responseTime)
        root[2][12][2].text = str(self.accident_detect.f_limitVal)
        root[2][13][0].text = str(int(self.occupy_emergency_area.b_enable))
        root[2][13][1].text = str(self.occupy_emergency_area.f_responseTime)
        root[2][13][2].text = str(self.occupy_emergency_area.f_limitVal)
        tree.write(self.s_cfg_path)


class structTrackParam(object):
    """跟踪算法参数"""
    def __init__(self, s_param_path):
        self.s_param_path = s_param_path
        self.fConeThreshold = 0.75
        self.DisplayFrames = 4
        self.nMinHits = 2
        self.Horizon_threshoud = 20
        self.Vertical_threshoud = 30
        self.MaxLifetime = 100

    def getDisplayFrames(self):
        return self.DisplayFrames

    def setDisplayFrames(self, DisplayFrames):
        self.DisplayFrames = DisplayFrames

    def getHorizon_threshoud(self):
        return self.Horizon_threshoud

    def setHorizon_threshoud(self, Horizon_threshoud):
        self.Horizon_threshoud = Horizon_threshoud

    def getVertical_threshoud(self):
        return self.Vertical_threshoud

    def setVertical_threshoud(self, Vertical_threshoud):
        self.Vertical_threshoud = Vertical_threshoud

    def getMaxLifetime(self):
        return self.MaxLifetime

    def setMaxLifetime(self, MaxLifetime):
        self.MaxLifetime = MaxLifetime

    def updateParam(self):
        """根据配置文件获取算法参数"""
        try:
            if not judge_exist(self.s_param_path):
                return
            tree = ET.parse(self.s_param_path)
            root = tree.getroot()
            self.fConeThreshold = float(root[2][0].text)
            self.DisplayFrames = int(root[2][1].text)
            self.nMinHits = int(root[2][2].text)
            self.Horizon_threshoud = float(root[2][3].text)
            self.Vertical_threshoud = float(root[2][4].text)
            self.MaxLifetime = int(root[2][5].text)
        except:
            print(traceback.format_exc())

    def saveParam(self):
        try:
            if not judge_exist(self.s_param_path):
                return
            tree = ET.parse(self.s_param_path)
            root = tree.getroot()
            root[2][1].text = str(self.DisplayFrames)
            root[2][3].text = str(self.Horizon_threshoud)
            root[2][4].text = str(self.Vertical_threshoud)
            root[2][5].text = str(self.MaxLifetime)
            tree.write(self.s_param_path)
        except:
            print(traceback.format_exc())


class structStationDev:
    ''' 用于保存从配置文件中获取的雷达的IP地址,以及经纬度航向角信息 '''
    def __init__(self):
        self.strStationSrcIp = ""
        self.fLongitude = 116.2869907
        self.fLatitude = 40.0516922
        self.fAngleNorth = 0


class lidarParam(object):
    ''' 用于保存从配置文件中读取的数据 '''
    def __init__(self, s_param_path):
        self.s_param_path = s_param_path
        self.nStationCount = 1
        self.listStationDev = []
        self.listAttachedIp = []
        self.nUdpServicePort = 5555
        self.bHighSpeed = False
        self.bTimeMatch = False

    def getStationDev(self, nIndex):
        if nIndex < 0 or nIndex >= self.nStationCount:
            return None
        return self.listStationDev[nIndex]

    def getStationCount(self):
        return self.nStationCount

    def getLidarLongitude(self, index):
        return self.listStationDev[index].fLongitude

    def getLidarLatitude(self, index):
        return self.listStationDev[index].fLatitude

    def getAngleNorthT(self, index):
        return self.listStationDev[index].fAngleNorth

    def getListAttachedIp(self):
        return self.listAttachedIp

    def getUdpServicePort(self):
        return self.nUdpServicePort

    def getHighSpeed(self):
        return self.bHighSpeed

    def setHighSpeed(self, bHighSpeed):
        self.bHighSpeed = bHighSpeed

    def getTimeMatch(self):
        return self.bTimeMatch

    def setTimeMatch(self, bTimeMatch):
        self.bTimeMatch = bTimeMatch

    def updateParam(self):
        """根据配置文件获取雷达参数"""
        try:
            if not judge_exist(self.s_param_path):
                return
            tree = ET.parse(self.s_param_path)
            root = tree.getroot()
            self.nStationCount = int(root[2][0].text)
            listAttachedIp = []
            for tag in root[2][1]:
                stStationDev = structStationDev()
                stStationDev.strStationSrcIp = tag[0].text
                if stStationDev.strStationSrcIp is not None:
                    listAttachedIp.append(stStationDev.strStationSrcIp)
                else:
                    continue
                stStationDev.fLatitude = float(tag[1].text)
                stStationDev.fLongitude = float(tag[2].text)
                stStationDev.fAngleNorth = float(tag[3].text)

                self.listStationDev.append(stStationDev)
            self.listAttachedIp = listAttachedIp

            self.nUdpServicePort = int(root[2][2].text)
            self.bHighSpeed = bool(int(root[2][3].text))
            self.bTimeMatch = bool(int(root[2][4].text))
        except:
            print(traceback.format_exc())

    def saveParam(self):
        """保存雷达参数至配置文件"""
        try:
            if not judge_exist(self.s_param_path):
                return
            tree = ET.parse(self.s_param_path)
            root = tree.getroot()
            root[2][0].text = str(self.nStationCount)
            nIndex = 0
            for tag in root[2][1]:
                stStationDev = self.listStationDev[0]
                tag[0].text = stStationDev.strStationSrcIp
                tag[1].text = str(stStationDev.fLatitude)
                tag[2].text = str(stStationDev.fLongitude)
                tag[3].text = str(stStationDev.fAngleNorth)

            root[2][2].text = str(self.nUdpServicePort)
            root[2][3].text = str(int(self.bHighSpeed))
            root[2][4].text = str(int(self.bTimeMatch))
            tree.write(self.s_param_path)
        except:
            print(traceback.format_exc())


class Lon_Lat_Meter_Transform(object):
    '''经纬度与m坐标系相互转换'''

    def __init__(self, lon0_bmp, lat0_bmp, angle0_bmp):  # 与全域基站经纬度转m转换关系一致
        '''初始化
        @param:
            lon0_bmp,lat0_bmp,angle0_bmp:用于绘图时的粗标的经纬度及夹角
        '''
        self.R_a = 6378137.00  # 地球长半轴
        self.R_b = 6356752.3142  # 地球短半轴
        self.lon0_bmp = lon0_bmp
        self.lat0_bmp = lat0_bmp
        self.angle0_bmp = angle0_bmp
        self.R_bmp, _ = cv2.Rodrigues(np.array([0, 0, self.angle0_bmp * np.pi / 180]))
        self.R_bmp_inv = np.linalg.inv(self.R_bmp)

    def lonlat_to_xy_of_draw(self, lonlat):
        '''经纬度转到画车道图的激光雷达m坐标系下
        @param:
            lon_lat:(-1,2) np.array
        @return:
            xy:(-1,2) np.array
        '''
        x = (lonlat[:, 0] - self.lon0_bmp) * np.pi / 180 * self.R_a * np.cos(self.lat0_bmp * np.pi / 180)
        y = (lonlat[:, 1] - self.lat0_bmp) * np.pi / 180 * self.R_b
        xyz = np.hstack((x.reshape(-1, 1), y.reshape(-1, 1), np.ones((len(x), 1))))  # 旋转
        xyz = self.R_bmp.dot(xyz.T).T
        return xyz[:, :2]

    def xy_of_draw_to_lonlat(self, xy):
        '''画车道图时的激光雷达m坐标系转换到经纬度
        @param:
            xy:(-1,2)  np.array
        @return:
            lonlat:(-1,2) np.array
        '''
        xyz = np.hstack((xy, np.ones((len(xy), 1))))
        origin_xyz = self.R_bmp_inv.dot(xyz.T).T  # 反旋转
        lon = origin_xyz[:, 0] / (self.R_a * np.cos(self.lat0_bmp * np.pi / 180)) * 180 / np.pi + self.lon0_bmp
        lat = origin_xyz[:, 1] / (self.R_b * np.pi / 180) + self.lat0_bmp
        lonlat = np.hstack((lon.reshape(-1, 1), lat.reshape(-1, 1)))
        return lonlat


class MyTime(object):
    """自定义定时器"""
    def __init__(self):
        self.tTime = None

    def start(self):
        self.tTime = time.time()

    def getTime(self, wei=0):
        if self.tTime is None:
            return 0.0
        else:
            return round((time.time() - self.tTime) * 1000, wei)

    def restart(self):
        self.tTime = time.time()