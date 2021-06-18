import time
import cv2
import numpy as np
import math

switch_pc = {0: "car", 1: "bicycle", 2: "bus", 3: "motorbike", 4: "person", 5: "cone", 6: "truck", 7: "None",
             8: "Minibus"}
PI_rad = math.pi / 180


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


class algParam:
    def __init__(self):
        self.fConeThreshold = 0.75
        self.DisplayFrames = 4
        self.nMinHits = 2
        self.Horizon_threshoud = 20
        self.Vertical_threshoud = 30
        self.MaxLifetime = 100
