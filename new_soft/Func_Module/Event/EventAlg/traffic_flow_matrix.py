# cython: language_level=3
import cv2
import numpy as np
from copy import deepcopy
from timeit import time
from dateutil import tz
import pandas as pd
import csv
import os, sys
import pickle
from enum import Enum, auto  # Class.member.value才是数值
# from second.long_lat import XYZ_To_BLH
# import second.core.box_np_ops as box_np_ops
# 时间获取函数所需  copied from CommonDefine.py
import datetime
import traceback
from func_timeout import func_set_timeout
from datetime import timezone
from datetime import timedelta
import open3d as o3d
import math

import json

SECOND_1900_TO_1970 = 2208988800


def parse_GPS_time():  # transfer para:pl_data
    ###解析一包数据的GPS时间
    # data = pl_data
    # syq add
    dataTimeNow = datetime.datetime.now()
    # #print('now',dataTimeNow)
    # #print('mocro',dataTimeNow.microsecond)
    # dataTimePackage = dataTimeNow
    strDataTimePackage = dataTimeNow.astimezone(timezone(timedelta(hours=-0))).strftime("%Y-%m-%d %H:%M:%S")

    # 转换成时间数组
    timeArray = time.strptime(strDataTimePackage, "%Y-%m-%d %H:%M:%S")
    # 转换成时间戳
    unix_t = time.mktime(timeArray)
    unix_t = str(unix_t).split('.')[0]
    unix_t = int(unix_t) + SECOND_1900_TO_1970
    return unix_t, dataTimeNow.microsecond


class Line_Date(object):
    def __init__(self):
        pass

    @staticmethod
    def get_y(x, x1, y1, x2, y2):
        return (x - x1) * (y2 - y1) / (x2 - x1) + y1

    @staticmethod
    def get_x(y, x1, y1, x2, y2):
        return (y - y1) * (x2 - x1) / (y2 - y1) + x1

    @staticmethod
    def get_xy(x1, y1, x2, y2):
        xy = []
        small_x, big_x = None, None
        small_y, big_y = None, None
        if abs(x1 - x2) > abs(y1 - y2):
            if x1 < x2:
                small_x = x1
                big_x = x2
            else:
                small_x = x2
                big_x = x1
            for i in np.arange(small_x, big_x, 1):
                xy.append([i, Line_Date.get_y(i, x1, y1, x2, y2)])
        else:
            if y1 < y2:
                small_y = y1
                big_y = y2
            else:
                small_y = y2
                big_y = y1
            for i in np.arange(small_y, big_y, 1):
                xy.append([Line_Date.get_x(i, x1, y1, x2, y2), i])
        return xy


class Lon_Lat_Meter_Transform(object):
    """经纬度与m坐标系相互转换"""

    def __init__(self, lon0_bmp, lat0_bmp, angle0_bmp):  # 与全域基站经纬度转m转换关系一致
        """初始化

        @param:
            lon0_bmp,lat0_bmp,angle0_bmp:用于绘图时的粗标的经纬度及夹角
        """
        self.R_a = 6378137.00  # 地球长半轴
        self.R_b = 6356752.3142  # 地球短半轴
        self.lon0_bmp = lon0_bmp
        self.lat0_bmp = lat0_bmp
        self.angle0_bmp = angle0_bmp

        self.R_bmp, _ = cv2.Rodrigues(np.array([0, 0, self.angle0_bmp * np.pi / 180]))
        self.R_bmp_inv = np.linalg.inv(self.R_bmp)

    def lonlat_to_xy_of_draw(self, lonlat):
        """经纬度转到画车道图的激光雷达m坐标系下

        @param:
            lon_lat:(-1,2) np.array
        @return:
            xy:(-1,2) np.array
        """
        x = (lonlat[:, 0] - self.lon0_bmp) * np.pi / 180 * self.R_a * np.cos(self.lat0_bmp * np.pi / 180)
        y = (lonlat[:, 1] - self.lat0_bmp) * np.pi / 180 * self.R_b
        xyz = np.hstack((x.reshape(-1, 1), y.reshape(-1, 1), np.ones((len(x), 1))))  # 旋转

        xyz = self.R_bmp.dot(xyz.T).T
        return xyz[:, :2]

    def xy_of_draw_to_lonlat(self, xy):
        """画车道图时的激光雷达m坐标系转换到经纬度

        @param:
            xy:(-1,2)  np.array
        @return:
            lonlat:(-1,2) np.array
        """
        xyz = np.hstack((xy, np.ones((len(xy), 1))))
        origin_xyz = self.R_bmp_inv.dot(xyz.T).T  # 反旋转
        lon = origin_xyz[:, 0] / (self.R_a * np.cos(self.lat0_bmp * np.pi / 180)) * 180 / np.pi + self.lon0_bmp
        lat = origin_xyz[:, 1] / (self.R_b * np.pi / 180) + self.lat0_bmp
        lonlat = np.hstack((lon.reshape(-1, 1), lat.reshape(-1, 1)))
        return lonlat


class Event_Continue_Info(object):
    def __init__(self, start_forward_time=0, timenow=0, coincidence=0):
        # self.start_time=time.time() -start_forward_time                #出现、开始时间
        self.start_time = timenow - start_forward_time  # 出现、开始时间
        self.end_time = self.start_time  # 最新、结束时间
        parse_time = list(parse_GPS_time())
        parse_time[0] = parse_time[0] - start_forward_time
        self.format_start_time = parse_time  # 开始时间的格式化形式
        self.format_end_time = self.format_start_time  # 最新时间的格式化形式
        self.hit_count = 1  # 连续帧出现次数
        self.event_no = -1  # 事件编号
        self.longitude = 0  # 经度
        self.latitude = 0  # 纬度
        self.coincidence = coincidence


class Abnormal_Class_Index(object):
    illegal_stop = 1
    retrograde_for_non_motor = 2
    retrograde_for_motor = 3
    occupy_dedicated_lane = 4
    people_occupy_motor_lane = 5
    speeding_for_motor = 6
    speeding_for_non_motor = 7
    stroll = 8
    cross_line = 9
    cross_lane_for_non_motor = 10
    cross_lane_for_people = 11
    cross_lane_for_motor = 12
    run_the_red_light_for_motor = 13
    run_the_red_light_for_people = 14
    occupy_bus_lane = 15
    change_lanes = 16
    spills = 17
    accident = 18
    occupy_emergency_lane = 19
    hanging_in_tunnel = 20
    person_in_tunnel = 21
    non_motor_in_tunnel = 22


class History_Continue_Lane_Info(object):
    """保存历史连续帧车道相关信息
    """

    def __init__(self, lane, detect_info, v_px, v_py, v_px_by_angle, v_py_by_angle):
        self.lane = lane
        self.detect_info = detect_info
        self.v_px = v_px
        self.v_py = v_py
        self.v_px_by_angle = v_px_by_angle
        self.v_py_by_angle = v_py_by_angle


class History_Different_Lane_Info(object):
    """保存历史不同车道相关信息
    """

    def __init__(self, lane):
        self.lane = lane


class Use_Camera_Map(object):
    def __init__(self, camera_param_dict):
        self.id_cam = {}  # {(基站id,相机号):{全域id:True or False}}
        self.camera_map_dict = {}
        self.camera_param_dict = camera_param_dict

        for pc_id, params in self.camera_param_dict.items():  # 对每个基站
            for i in range(len(params)):  # 对每个相机
                in_param, rotate_vec, translation_vec, dist_vec = params[i].in_parameter, params[i].rotate_matrix, \
                                                                  params[i].translation_matrix, params[i].dist_matrix
                self.camera_map_dict[(pc_id, i)] = Camera_Map(in_param, rotate_vec, translation_vec, dist_vec)
                self.id_cam[(pc_id, i)] = {}

    def get_id_cam(self, all_pc_det_dict):
        """获取id对应的相机

        @param:
            all_pc_det_dict:{基站id:对应的Pc_detect_info}
        @return:
            id_cam:{(基站id,相机号):{全域id:True or False}}
        """
        for pc_id, params in self.camera_param_dict.items():  # 对每个基站
            for i in range(len(params)):  # 对每个相机
                this_pc_det = all_pc_det_dict[pc_id]
                if len(this_pc_det) == 0:
                    continue
                indics = self.camera_map_dict[(pc_id, i)].map(this_pc_det)
                for pc_det_i in range(len(this_pc_det)):
                    self.id_cam[(pc_id, i)][this_pc_det[pc_det_i, Pc_Info_Index.all_area_id.value]] = indics[pc_det_i]


class Camera_Map(object):
    """全域下进行相机映射"""

    def __init__(self, in_param, rotate_vec, translation_vec, dist_vec):
        self.in_param = in_param
        self.rotate_vec = rotate_vec
        self.translation_vec = translation_vec
        self.dist_vec = dist_vec

        self._get_tf_lidar_to_cam()

    def _get_tf_lidar_to_cam(self):
        """获取激光雷达到相机的变换矩阵"""
        tf_lidar_to_cam = np.zeros((4, 4))
        rotate_mat, _ = cv2.Rodrigues(self.rotate_vec)
        tf_lidar_to_cam[:3, :3] = rotate_mat
        tf_lidar_to_cam[:3, 3] = self.translation_vec
        tf_lidar_to_cam[3, 3] = 1
        self.tf_lidar_to_cam = tf_lidar_to_cam

    def lidar_to_cam(self, xyz_lidar):
        """将激光雷达坐标转换到相机坐标系下"""
        xyz_lidar = 1000 * xyz_lidar
        xyz_lidar = np.hstack([xyz_lidar, np.ones(xyz_lidar.shape[0]).reshape(-1, 1)])
        return self.tf_lidar_to_cam.dot(xyz_lidar.T)[:3].T

    def project_points(self, point3d):
        """3d点云投影到2d"""
        point3d = np.mat(1000 * point3d)
        point2d, _ = cv2.projectPoints(point3d, self.rotate_vec, self.translation_vec, \
                                       self.in_param, self.dist_vec)
        point2d = np.squeeze(point2d)
        return point2d

    def map(self, pc_det, img_size=(1920, 1080)):
        """
        pc_det:点云检测结果
        """
        indics = np.zeros((pc_det.shape[0],), dtype=bool)
        points = pc_det[:, Pc_Info_Index.bottom_front_right_x.value:Pc_Info_Index.cur_frame_time.value]
        points = points.reshape(-1, 3)

        xyz_cam = self.lidar_to_cam(points)  # 激光雷达坐标系转换到相机坐标系
        filter_cam = np.where(xyz_cam[:, 2] > 0, True, False)  # 点云检测结果在相机朝向的索引
        r = np.linalg.norm(points[:, :2], axis=1)  # 点云点到雷达的距离
        filter_r = np.where((r > 4), True, False)  # &(r<35) #距离激光雷达>4m点云索引

        filter_ = np.logical_and(filter_cam, filter_r)  # 距离激光雷达>4m且在相机朝向的点云索引

        img2d = self.project_points(points)  # 点云检测结果投影到图像

        filter_img = np.where(
            ((img2d[:, 0] > 0) & (img2d[:, 0] < img_size[0]) & (img2d[:, 1] > 0) & (img2d[:, 1] < img_size[1])), True,
            False)  # 角点在图像中的索引

        for i in range(int(len(img2d) / 8)):
            if (filter_[8 * i:8 * (i + 1)] == False).all():  # 去除8个角点都在相机背面的点云检测结果
                continue
            if (filter_img[8 * i:8 * (i + 1)] == False).all():  # 去除8个角点都不在图像的点云检测结果
                continue
            indics[i] = True
        return indics


class Judge_Camera(object):
    """用于判断目标是否在当前相机下的类"""

    def __init__(self, p1_x, p1_y, p2_x, p2_y):
        """p1,p2:当前相机在激光雷达bmp下的边界点像素值"""
        self.c_px, self.c_py = 1000, 1000
        self.points_for_camera_boundary = np.array([p1_x, p1_y, p2_x, p2_y]) - self.c_px
        self.is_in_this_camera = None  # 判断目标是否在当前相机下
        self.active()

    def get_angle(self, v1, v2=[0, 10]):  # 默认与负半轴夹角
        """获取与y轴负轴顺时针的夹角"""
        angle = np.arccos((v1[0] * v2[0] + v1[1] * v2[1]) / \
                          np.sqrt((v1[0] ** 2 + v1[1] ** 2) * (v2[0] ** 2 + v2[1] ** 2))) * 180 / np.pi
        angle = angle if v1[0] < 0 else 360 - angle
        return angle

    def active(self):
        angle_1 = self.get_angle(self.points_for_camera_boundary[:2])
        angle_2 = self.get_angle(self.points_for_camera_boundary[2:])
        if angle_1 < angle_2:
            self.small_angle = angle_1
            self.big_angle = angle_2
        else:
            self.small_angle = angle_2
            self.big_angle = angle_1

        if self.big_angle - self.small_angle < 180:
            self.is_in_this_camera = self._donot_contain_0
        else:
            self.is_in_this_camera = self._contain_0

    def _donot_contain_0(self, px, py):
        """不在0度之间"""
        angle = self.get_angle([px - self.c_px, py - self.c_py])
        return self.small_angle <= angle <= self.big_angle

    def _contain_0(self, px, py):
        """在0度之间"""
        angle = self.get_angle([px - self.c_px, py - self.c_py])
        return 0 <= angle <= self.small_angle or self.big_angle <= angle <= 360


class Lane_Fix_Info(object):
    def __init__(self, angle, lane_output_name):
        self.angle = angle  # 车道固定的航向角
        self.lane_output_name = lane_output_name  # 车道输出名


class Traffic_Light(Enum):
    """红绿灯状态"""
    INIT = auto()
    RED = auto()
    YELLOW = auto()
    GREEN = auto()


class Traffic_Light_State(object):
    """红绿灯状态值"""

    def __init__(self, left_state, left_time, straight_state, straight_time, right_state, right_time, \
                 people_state, people_time, non_motor_state, non_motor_time):
        self.left_state = left_state
        self.left_time = left_time
        self.straight_state = straight_state
        self.straight_time = straight_time
        self.right_state = right_state
        self.right_time = right_time
        self.people_state = people_state
        self.people_time = people_time
        self.non_motor_state = non_motor_state
        self.non_motor_time = non_motor_time


class Pc_Info_Index(Enum):
    """枚举点云输入信息对应索引"""
    x = 0  # 中心点x坐标0
    y = auto()  # 中心点y坐标1
    z = auto()  # 中心点z坐标2
    w = auto()  # 宽3
    l = auto()  # 长4
    h = auto()  # 高5
    a = auto()  # 航向角6
    c = auto()  # 类别7
    v = auto()  # 速度8
    all_area_id = auto()  # 全域id9
    s = auto()  # 置信度10

    src = auto()  # 来源11
    origin_id = auto()  # 原id12
    pc_id = auto()  # 基站id13
    color = auto()  # 颜色14

    longitude = auto()  # 经度15
    latitude = auto()  # 纬度16

    pc_x = auto()  # 单基站x17
    pc_y = auto()  # 单基站y18

    cam_id = auto()  # 目标在所在基站的哪个相机下 0代表不在相机下19

    bottom_front_right_x = auto()
    bottom_front_right_y = auto()
    bottom_front_right_z = auto()
    top_front_right_x = auto()
    top_front_right_y = auto()
    top_front_right_z = auto()

    top_behind_right_x = auto()
    top_behind_right_y = auto()
    top_behind_right_z = auto()
    bottom_behind_right_x = auto()
    bottom_behind_right_y = auto()
    bottom_behind_right_z = auto()

    bottom_front_left_x = auto()
    bottom_front_left_y = auto()
    bottom_front_left_z = auto()
    top_front_left_x = auto()
    top_front_left_y = auto()
    top_front_left_z = auto()

    top_behind_left_x = auto()
    top_behind_left_y = auto()
    top_behind_left_z = auto()
    bottom_behind_left_x = auto()
    bottom_behind_left_y = auto()
    bottom_behind_left_z = auto()

    cur_frame_time = auto()
    # cur_frame_id=auto()


class Detect_Class_Type(object):
    """点云检测结果的分类id"""
    # use alltrack offline test
    # # switch_send = {0: "None", 1: "car", 2: "truck", 3: "bus", 4: "person", 5: "bicycle", 6: "motorbike", 7: "cone", 8: "Minibus"}
    # motor_type=[1,2,3,6,8]
    # non_motor_type=[5]
    # people=[4]
    # bus=[3]
    # truck=[2]
    # minibus=[8]
    # car=[1]
    # motorcycle=[6]
    # bicycle=[5]
    # # cone = [7]
    # all=[1,2,3,4,5,6,8]

    #
    none_type = [5, 7]
    motor_type = [0, 2, 3, 6, 8]
    non_motor_type = [1]
    people = [4]
    bus = [2]
    truck = [6]
    minibus = [8]
    car = [0]
    motorcycle = [3]
    bicycle = [1]
    all = [0, 1, 2, 3, 4, 6, 8]


class Lane_Direction(object):
    """车道线方向"""
    STRAIGHT = 0
    LEFT = 1
    RIGHT = 2
    STRAIGHT_LEFT = 3
    STRAIGHT_RIGHT = 4
    STRAIGHT_LEFT_RIGHT = 5
    UNKOWN = 6


class Motorway_Type(object):
    """机动车道类型"""
    # 判断非机动车是否可以在当前车道行驶
    MOTORWAY = 0
    NON_MOTORWAY = 1
    ALL = 2


class Config_Matrix_Index(object):
    """配置矩阵的索引"""
    LANE_NO = 0
    TURN_LEFT_1 = 1  # 左转辅助道1
    TURN_LEFT_2 = 2  # 左转辅助道2
    TURN_LEFT_3 = 3  # 左转辅助道3
    EMERGENCY_AREA = 4  # emergency 4
    CALC_FLOW = 5  # 车流量统计区
    FORBID_CROSS_LINE = 6  # 禁止压线区
    NO_PARK = 7  # 不能停车
    CENTER_AREA = 8  # 十字中心,包括车道头
    SIDEWALK = 9  # 人行道


class Mean_V_Info(object):
    """平均速度对应信息,输出为km/h"""
    default_mean_v = 255

    def __init__(self, max_v):
        """max_v:km/h
        """
        self.__max_v = max_v
        self.sum_v = 0
        self.sum_num = 0
        self.mean_v = max_v

    def update(self, v):
        """v:m/s,减少一点*3.6的计算量,放到需要计算平均速度时*3.6
        """
        self.sum_v += v
        self.sum_num += 1

    def get_mean_v(self):
        """输出km/h
        """
        self.mean_v = Mean_V_Info.default_mean_v if self.sum_num == 0 else self.sum_v * 3.6 / self.sum_num

    def clear(self):
        self.sum_v = 0
        self.sum_num = 0
        # self.mean_v=self.__max_v #为了发送给平台,不对mean_v清零,而在每次判断道路拥堵时更新mean_v


class Mean_Space_Headway_Info(object):
    """平均车头间距对应信息"""

    def __init__(self, max_sh):
        """max_v:km/h
        """
        self.__max_sh = max_sh
        self.sum_sh = 0
        self.sum_num = 0
        self.mean_sh = max_sh

    def update(self, sh_list):
        """sh_list:车头间距的list
        """
        self.sum_sh += sum(sh_list)
        self.sum_num += len(sh_list)

    def get_mean_sh(self):
        self.mean_sh = self.__max_sh if self.sum_num == 0 else self.sum_sh / self.sum_num

    def clear(self):
        self.sum_sh = 0
        self.sum_num = 0
        # self.mean_sh=self.__max_sh #为了发送给平台,不对mean_sh清零


class Lane_Info(object):
    def __init__(self):
        self.start_px, self.start_py = None, None  # 车道头像素x、y坐标
        self.end_px, self.end_py = None, None  # 车道头像素x、y坐标
        self.length, self.width = None, None  # 车道长度,宽度
        self.lane_direction = None  # 车道线方向
        self.lane_class = None  # 快速路 or 主干路、支路
        self.min_v, self.max_v = None, None  # 限速
        self.is_non_motorway = None  # 是否非机动车道
        self.is_bus_lane = None  # 是否公交车道
        self.is_emergency_lane = None  # 是否应急车道
        self.out_in = None  # 驶出驶入
        self.v_px, self.v_py = None, None  # 当前车道在bmp中的正向速度
        self.angle_in, self.angle_out = None, None  # 车道与y轴负方向夹角
        self.small_angle, self.big_angle = None, None  # 左转辅助道的角度范围
        self.is_angle_in_this_turn_left_lane = None  # 判断当前车航向角是否在当前左转辅助道上
        self.angle_to_north = None  # 当前车道与正北方向顺时针的夹角(常规车道)

    def fill_data(self, origin_data, Congestion_Level_highway_table, Congestion_Level_main_branch_table):
        """将配置文件中的信息填充到结构体中"""
        lane_name, lane_direction, lane_class, is_non_motorway, is_bus_lane, is_emergency_lane, \
        min_v, max_v, length, width, start_px, start_py, end_px, end_py = origin_data

        self.start_px, self.start_py = start_px, start_py
        self.end_px, self.end_py = end_px, end_py
        self.length, self.width = length, width
        self.lane_direction = lane_direction
        self.lane_class = lane_class
        self.min_v, self.max_v = min_v / 3.6, max_v / 3.6
        self.is_non_motorway = is_non_motorway
        self.is_bus_lane = is_bus_lane
        self.is_emergency_lane = is_emergency_lane

        self.get_v_for_congestion_level(Congestion_Level_highway_table, Congestion_Level_main_branch_table)

    def get_v_for_congestion_level(self, Congestion_Level_highway_table, Congestion_Level_main_branch_table):
        """获取用于判断道路拥堵情况的速度边界"""
        this_table = Congestion_Level_highway_table if self.lane_class == Lane_Class.highway else Congestion_Level_main_branch_table
        self.unblocked_min_v = None
        self.slightly_congested_min_v, self.slightly_congested_max_v = None, None
        self.moderately_congested_min_v, self.moderately_congested_max_v = None, None
        self.seriously_congested_min_v, self.seriously_congested_max_v = None, None
        if self.max_v * 3.6 < this_table[-1, 0]:  # 低于表的最低速度
            self.unblocked_min_v, self.slightly_congested_min_v, self.slightly_congested_max_v, \
            self.moderately_congested_min_v, self.moderately_congested_max_v, \
            self.seriously_congested_min_v, self.seriously_congested_max_v = this_table[-1, 1:]
        elif self.max_v * 3.6 >= this_table[0, 0]:  # 高于表的最大速度
            self.unblocked_min_v, self.slightly_congested_min_v, self.slightly_congested_max_v, \
            self.moderately_congested_min_v, self.moderately_congested_max_v, \
            self.seriously_congested_min_v, self.seriously_congested_max_v = this_table[0, 1:]
        else:
            for i in range(len(this_table) - 2):
                if this_table[i + 1, 0] <= self.max_v * 3.6 < this_table[i, 0]:
                    self.unblocked_min_v, self.slightly_congested_min_v, self.slightly_congested_max_v, \
                    self.moderately_congested_min_v, self.moderately_congested_max_v, \
                    self.seriously_congested_min_v, self.seriously_congested_max_v = \
                        this_table[i + 1, 1:] + (this_table[i, 1:] - this_table[i + 1, 1:]) * \
                        (self.max_v * 3.6 - this_table[0, i + 1]) / (this_table[0, i] - this_table[0, i + 1])
                    break

    def fill_plus_data_for_non_converge(self, origin_data, negative_y_axis_xy, theta):
        """填充非交汇处的额外信息
        theta:y轴与正北方向夹角
        """
        lane_name, lane_direction, lane_class, is_non_motorway, is_bus_lane, is_emergency_lane, \
        min_v, max_v, length, width, start_px, start_py, end_px, end_py = origin_data

        self.out_in = int(str(lane_name)[1])

        self.v_px = start_px - end_px if self.out_in == 0 else end_px - start_px  # 向量:终点-起点
        self.v_py = start_py - end_py if self.out_in == 0 else end_py - start_py

        # 求当前车道与y轴负方向的夹角，从y轴负方向开始顺时针算角度
        this_lane_vec_xy = [end_px - start_px, end_py - start_py]
        angle = np.arccos((negative_y_axis_xy[0] * this_lane_vec_xy[0] + negative_y_axis_xy[1] * this_lane_vec_xy[1]) / \
                          np.sqrt((this_lane_vec_xy[0] ** 2 + this_lane_vec_xy[1] ** 2) * (
                                      negative_y_axis_xy[0] ** 2 + negative_y_axis_xy[1] ** 2))) * 180 / np.pi
        self.angle_in = angle if this_lane_vec_xy[0] < 0 else 360 - angle
        self.angle_out = self.angle_in - 180 if self.angle_in > 180 else self.angle_in + 180
        self.angle_to_north = (self.angle_in + 180 - theta) % 360

    def fill_plus_data_for_converge(self, start_lane_info, end_lane_info):
        """填充交汇处的额外信息"""
        self.out_in = 0

        # 求当前左转辅助道的角度范围
        reduce_angle = 15  # 左转辅助道起始车道角度、终止车道角度相减的角度，防止直行车辆的误判
        small_lane_angle, big_lane_angle = None, None
        if start_lane_info.angle_out < end_lane_info.angle_in:
            small_lane_angle, big_lane_angle = start_lane_info.angle_out, end_lane_info.angle_in
        else:
            small_lane_angle, big_lane_angle = end_lane_info.angle_in, start_lane_info.angle_out
        if big_lane_angle - small_lane_angle < 180:  # small_angle~big_angle
            self.small_angle = small_lane_angle + reduce_angle
            self.big_angle = big_lane_angle - reduce_angle
            self.is_angle_in_this_turn_left_lane = self._is_angle_in_this_turn_left_lane_normal
        else:
            angle_near_90 = small_lane_angle - reduce_angle
            if angle_near_90 < 0:
                angle_near_90 += 360
            angle_near_270 = big_lane_angle + reduce_angle
            if angle_near_270 > 360:
                angle_near_270 -= 360
            if np.abs(angle_near_90 - angle_near_270) < 180:  # angle_near_270~angle_near_90
                self.small_angle = angle_near_270
                self.big_angle = angle_near_90
                self.is_angle_in_this_turn_left_lane = self._is_angle_in_this_turn_left_lane_normal
            else:  # 0~angle_near_90 or angle_near_270~360
                self.small_angle = angle_near_90
                self.big_angle = angle_near_270
                self.is_angle_in_this_turn_left_lane = self._is_angle_in_this_turn_left_lane_near_0

    def _is_angle_in_this_turn_left_lane_normal(self, angle):
        """左转辅助道不在0度之间"""
        return self.small_angle <= angle <= self.big_angle

    def _is_angle_in_this_turn_left_lane_near_0(self, angle):
        """左转辅助道在0度之间"""
        return 0 <= angle <= self.small_angle or self.big_angle <= angle <= 360


class Lane_Class(object):
    """车道类别"""
    highway = 0  # 快速路
    main_branch = 1  # 主干路、支路


class Detect_Info(object):
    def __init__(self):
        self.x, self.y, self.z = None, None, None
        self.l, self.w, self.h = None, None, None
        self.angle = None
        self.class_id = None
        self.coincidence = None
        self.v = None
        self.center_px, self.center_py = None, None
        self.front_left_px, self.front_left_py = None, None
        self.front_right_px, self.front_right_py = None, None
        self.behind_left_px, self.behind_left_py = None, None
        self.behind_right_px, self.behind_right_py = None, None
        self.longitude = None  # 经度
        self.latitude = None  # 纬度
        self.cam_id = None
        self.cur_id_time = None
        self.pc_id = None  # 基站编号
        # self.frameNum=None
        self.pos_coincidence = None

    def fill_data(self, origin_data, px_data, cur_time):
        x, y, z, w, l, h, a, c, v, all_area_id, s, src, origin_id, pc_id, color, longitude, latitude, pc_x, pc_y, cam_id = origin_data
        self.x, self.y, self.z = x, y, z
        self.l, self.w, self.h = l, w, h
        self.angle = a
        self.class_id = c
        self.coincidence = s
        self.v = v
        self.pc_id = pc_id  # 基站编号
        self.longitude = longitude
        self.latitude = latitude
        self.cam_id = cam_id  # 所在基站对应的相机号

        self.center_px, self.center_py, self.front_right_px, self.front_right_py, \
        self.front_left_px, self.front_left_py, self.behind_left_px, self.behind_left_py, \
        self.behind_right_px, self.behind_right_py = px_data
        self.cur_id_time = cur_time
        # self.frameNum=framecnt
        # self.pos_coincidence=pos_coincidence


class Congestion_Level(Enum):
    """拥堵程度_智慧基站"""
    unblocked = auto()  # 畅通
    slightly_congested = auto()  # 轻度拥堵
    moderately_congested = auto()  # 中度拥堵
    seriously_congested = auto()  # 严重拥堵


"""linxiao0514"""


class History_Info(object):
    def __init__(self):
        self.t, self.x, self.y, self.v = [], [], [], []
        self.cal_speed = []
        self.moving_average_speed = []
        self.acceleration = []
        # self.mean_v = None
        # self.time_age = []


class Traffic_Flow(object):
    def __init__(self, lane_config_path, pc_num, stEventParam):
        """初始化

        @param:
            lane_config_path:配置文件所在文件夹
            pc_num:全域软件基站个数
        @commit:
            旧车道图经纬度+旧车道图   traffic_param:use_lastest_lonlat=1  基站1的lastest用旧的车道图的经纬度
            新Dev经纬度+旧车道图     traffic_param:use_lastest_lonlat=0  按配置文件说明来输入
            新Dev经纬度+新车道图     traffic_param:use_lastest_lonlat=1  按配置文件说明来输入
        """
        self.stEventParam = stEventParam
        if os.path.exists(lane_config_path + 'config_matrix.pkl') and \
                os.path.exists(lane_config_path + 'lane_no_lane_name.pkl'):
            self.config_matrix = self._load_info(lane_config_path + 'config_matrix.pkl')
            self.lane_no_lane_name = self._load_info(lane_config_path + 'lane_no_lane_name.pkl')
        else:
            self._generate_matrix_data(lane_config_path)

        # 由于配置矩阵生成慢,将配置矩阵生成和车道信息文件生成分离
        if os.path.exists(lane_config_path + 'lane_origin_info.pkl'):
            self.lane_origin_info = self._load_info(lane_config_path + 'lane_origin_info.pkl')
        else:
            self._generate_lane_origin_info(lane_config_path)

        self._get_traffic_param(lane_config_path)

        if self.param['use_lastest_lonlat'] == 1:
            self.lon_lat_meter_transform = Lon_Lat_Meter_Transform(self.param['lon1_lastest'],
                                                                   self.param['lat1_lastest'],
                                                                   self.param['angle1_lastest'])
        else:
            self.lon_lat_meter_transform = Lon_Lat_Meter_Transform(self.param['lon1_bmp'],
                                                                   self.param['lat1_bmp'], self.param['angle1_bmp'])

        self.congestion_matrix = np.zeros(self._bmp_to_congestion_map(self.config_matrix.shape[:2]),
                                          dtype=np.uint8)  # 拥堵热力图矩阵,1m为一个格子

        self.spills_matrix = np.zeros(self._bmp_to_spills_coordinate(self.config_matrix.shape[:2]),
                                      dtype=np.uint8)  # 抛洒物热力图矩阵,1m为一个格子
        self._init_spills_lon_lat(lane_config_path)

        self.accident_matrix = np.zeros(self._bmp_to_accident_coordinate(self.config_matrix.shape[:2]),
                                        dtype=np.uint8)  # 交通事故热力图矩阵,1m为一个格子
        self._init_accident_lon_lat(lane_config_path)

        self.B3_cnt = 0  # 发送给平台的计数器
        self.B4_cnt = 0  # 发送给平台的计数器
        self.event_cnt = 0  # 事件编号
        self.history_events = {}
        # self.history_events_time={}
        self.pc_num = pc_num
        self._normal_lane_str_len = 3  # 1(默认)  3(非左转)  6(左转)
        self._get_negative_y_axis_xy()
        self._format_lane_info(lane_config_path)
        self._init_total_info()
        self._init_abnormal_event_sign(lane_config_path)
        self._init_abnormal_event_continue_info()
        self._init_history_abnormal_event()
        self._init_lane_fix_info(lane_config_path)
        self._init_judge_camera()
        self._init_for_nearest_out_lane()
        self._init_pc_bmp_xy()
        # self._own_frame_id=0#调用traffic_flow的frame_id,在写入info_for_check时使用

        path = './use_time_traffic/'
        if not os.path.exists(path):
            os.makedirs(path)
        if os.path.exists(path + 'use_time_traffic.txt'):
            os.remove(path + 'use_time_traffic.txt')
        self.use_time_traffic_fd = open('./use_time_traffic/use_time_traffic.txt', 'a')

        mediumpoint = np.load(lane_config_path + 'centerline_medium.npy')
        mediumpoint[:, 2] = 0
        self.pcd = o3d.geometry.PointCloud()
        self.pcd.points = o3d.utility.Vector3dVector(mediumpoint)
        self.pcd_tree = o3d.geometry.KDTreeFlann(self.pcd)

    def _init_pc_bmp_xy(self):
        """获取各基站在车道图上的像素坐标
        """
        self.pc_bmp_xy = []
        suffix = 'lastest'  # if self.param['use_lastest_lonlat']==1 else 'bmp'
        for i in range(1, 1 + self.pc_num):
            xy_m = self.lon_lat_meter_transform.lonlat_to_xy_of_draw(
                np.array([self.param['lon%d_%s' % (i, suffix)], self.param['lat%d_%s' % (i, suffix)]]).reshape(-1, 2)
            )
            self.pc_bmp_xy.append(self._lidar_to_bmp(xy_m, [0], [1]))
        self.pc_bmp_xy = np.array(self.pc_bmp_xy).reshape(-1, 2)

    def updateParam(self, stEventParam):
        self.stEventParam = stEventParam
        pass

    # @func_set_timeout(0.5)
    def use(self, input_info, statistics_flag):
        """
        input_info:感知信息
        statistics_flag:是否进行统计信息
        """
        self.use_time_traffic_fd.close()
        record_txt = r'./use_time_traffic/use_time_traffic.txt'
        fs = round(os.path.getsize(record_txt) / float(1024 * 1024), 2)
        if fs > 5000:
            os.remove(r'./use_time_traffic/use_time_traffic.txt')
        self.use_time_traffic_fd = open('./use_time_traffic/use_time_traffic.txt', 'a')
        dataTimeNow = datetime.datetime.now()
        t_start_use = time.time()
        # strDataTimePackage = dataTimeNow.astimezone(timezone(timedelta(hours=-0))).strftime("%Y-%m-%d %H:%M:%S")
        # # 转换成时间数组
        # timeArray = time.strptime(strDataTimePackage, "%Y-%m-%d %H:%M:%S")
        self.use_time_traffic_fd.write(f'cur_time:{dataTimeNow}\n')
        t_now = time.time()
        try:
            # self.joint_pc_det=deepcopy(input_info)
            if input_info.shape[0] == 0:
                self.cur_time = time.time()
            else:
                self.cur_time = input_info[0][-2]
            self.get_pre_info(input_info)
            # self.use_time_traffic_fd.write(f'get_pre_info:{(time.time()-t_now)*1000}ms\n');t_now=time.time()
            self.get_instantaneous_info()
            # self.use_time_traffic_fd.write(f'get_instantaneous_info:{(time.time()-t_now)*1000}ms\n');t_now=time.time()
            self.get_statistics_info(statistics_flag)
            # self.use_time_traffic_fd.write(f'get_statistics_info:{(time.time()-t_now)*1000}ms\n');t_now=time.time()
            self.detect_abnormal_event(statistics_flag)
            # self.use_time_traffic_fd.write(f'detect_abnormal_event:{(time.time()-t_now)*1000}ms\n')
            self.use_time_traffic_fd.write(f'whole_use:{(time.time() - t_start_use) * 1000}ms\n')
        except:
            self.use_time_traffic_fd.write("{}[:{}] - Event detect alg Error \n {}\n!".
                                           format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                  sys._getframe().f_lineno), traceback.format_exc())

    def _init_history_abnormal_event(self):
        """初始化历史异常事件  id:[info]"""
        self.history_occupy_dedicated_lane = {}  # 占用专用车道
        self.history_people_occupy_motor_lane = {}  # 行人在机动车道逗留
        self.history_retrograde_for_motor = {}  # 机动车逆行
        self.history_retrograde_for_non_motor = {}  # 非机动车逆行
        self.history_speeding_for_motor = {}  # 机动车超速
        self.history_speeding_for_non_motor = {}  # 非机动车超速
        self.history_stroll = {}  # 慢行
        self.history_cross_line = {}  # 压线
        self.history_illegal_stop = {}  # 违停
        self.history_cross_lane_for_non_motor = {}  # 非机动车横穿马路
        self.history_cross_lane_for_people = {}  # 行人横穿马路
        self.history_cross_lane_for_motor = {}  # 机动车横穿马路
        self.history_run_the_red_light_for_motor = {}  # 机动车闯红灯
        self.history_run_the_red_light_for_people = {}  # 行人闯红灯
        self.history_occupy_bus_lane = {}  # 占用公交车道
        self.history_change_lanes = {}  # 变道
        self.history_spills = {}  # 抛洒物
        self.history_accident = {}  # 交通事故
        self.history_occupy_emergency_lane = {}
        self.id_dist_list = {}
        # zhushi
        self.save_change_lane = {}

    def _init_abnormal_event_sign(self, lane_config_path):
        """读取检测事件的标志"""
        self.event_sign = {}
        with open(lane_config_path + 'traffic_event.csv', 'r') as fd:
            dict_reader = csv.DictReader(fd)
            for row in dict_reader:
                event_name = row['event_name']
                sign = True if row['sign'] == '1' else False

                self.event_sign[event_name] = sign

    def _init_abnormal_event_continue_info(self):
        """初始化异常事件的连续出现"""
        self.id_continue_occupy_dedicated_lane_info = {}  # 记录当前id连续占用其他车道的信息,包括机动车占用非机动车道,非机动车占用机动车道
        self.id_continue_people_occupy_motor_lane_info = {}  # 记录当前行人连续占用机动车道的信息
        self.id_continue_occupy_emergency_lane_info = {}
        self.id_continue_retrograde_info_for_motor = {}  # 记录目标连续逆行的信息,机动车逆行
        self.id_continue_retrograde_info_for_non_motor = {}  # 记录目标连续逆行的信息,非机动车逆行
        self.id_continue_speeding_info_for_motor = {}  # 记录当前id连续超速的信息,机动车超速
        self.id_continue_speeding_info_for_non_motor = {}  # 记录当前id连续超速的信息,非机动车超速
        self.id_continue_stroll_info = {}  # 记录当前id连续慢行的信息,机动车慢行
        self.id_continue_cross_line_info = {}  # 记录当前id连续压线的信息,机动车压线
        self.id_continue_stop_info = {}  # 记录当前机动车id连续停的信息,机动车违停
        self.id_continue_cross_lane_info_for_people = {}  # 记录当前id连续横穿马路的信息,行人横穿马路
        self.id_continue_cross_lane_info_for_motor = {}  # 记录当前id连续横穿马路的信息,机动车横穿马路
        self.id_continue_cross_lane_info_for_non_motor = {}  # 记录当前id连续横穿马路的信息,非机动车横穿马路
        self.id_continue_run_the_red_light_info_for_motor = {}  # 记录当前id连续闯红灯的信息,包括机动车闯红灯,行人闯红灯
        self.id_continue_run_the_red_light_info_for_people = {}  # 记录当前id连续闯红灯的信息,包括机动车闯红灯,行人闯红灯
        self.id_continue_occupy_bus_lane_info = {}  # 记录当前id连续占用公交车道的信息
        self.id_continue_change_lanes_info = {}  # 记录当前id连续变道的信息
        self.spills_coordinate_continue_info = {}  # 记录当前位置连续检测出抛洒物的信息
        self.accident_coordinate_continue_info = {}  # 记录当前位置连续检测出交通事故的信息

    def detect_abnormal_event(self, del_flag):
        """异常事件检测

        @param:
            del_flag:是否清除过久的历史记录
        """
        self.del_flag = del_flag
        t_now = time.time()
        self._init_abnormal_event_info()
        # print('_init_abnormal_event_info',time.time()-t_now,file=self.use_time_traffic_fd,flush=True);t_now=time.time()
        if self.event_sign['occupy_dedicated_lane']:
            self._detect_occupy_dedicated_lane()  # 非机动车或行人进入隧道
            self.use_time_traffic_fd.write(f'_detect_occupy_dedicated_lane:{(time.time() - t_now) * 1000}ms\n');
            t_now = time.time()
        if self.event_sign['occupy_emergency_area']:
            self._detect_occupy_emergency_lane()  # 占用emergency车道
            self.use_time_traffic_fd.write(f'_detect_occupy_emergency_lane:{(time.time() - t_now) * 1000}ms\n');
            t_now = time.time()
        if self.event_sign['occupy_bus_lane']:
            self._detect_occupy_bus_lane([[0, 0, 24, 0]])  # 判断占用公交车道
            self.use_time_traffic_fd.write(f'_detect_occupy_bus_lane:{time.time() - t_now}ms\n');
            t_now = time.time()
        self._get_cur_pix_v_by_angle()
        if self.event_sign['retrograde']:
            self._get_cur_pix_v()
            self._judge_retrograde()  # 判断逆行
            self.use_time_traffic_fd.write(f'_judge_retrograde:{(time.time() - t_now) * 1000}ms\n');
            t_now = time.time()

        if self.event_sign['speeding']:
            self._judge_speeding()  # 判断超速
            self.use_time_traffic_fd.write(f'_judge_speeding:{(time.time() - t_now) * 1000}ms\n');
            t_now = time.time()
        if self.event_sign['stroll']:
            self._judge_stroll()  # 判断慢行
            self.use_time_traffic_fd.write(f'_judge_stroll:{(time.time() - t_now) * 1000}ms\n');
            t_now = time.time()
        if self.event_sign['cross_line']:
            self._judge_cross_line()  # 判断压线
            self.use_time_traffic_fd.write(f'_judge_cross_line:{time.time() - t_now}ms\n');
            t_now = time.time()
        if self.event_sign['illegal_stop']:
            self._judge_illegal_stop()  # 判断违停
            self.use_time_traffic_fd.write(f'_judge_illegal_stop:{(time.time() - t_now) * 1000}ms\n');
            t_now = time.time()

        self._get_id_history_lane()
        # print('_get_id_history_lane',time.time()-t_now,file=self.use_time_traffic_fd,flush=True);t_now=time.time()
        if self.event_sign['cross_lane']:
            self._judge_cross_lane()  # 判断横穿马路
            self.use_time_traffic_fd.write(f'_judge_cross_lane:{(time.time() - t_now) * 1000}ms\n');
            t_now = time.time()
        if self.event_sign['change_lanes']:
            self._judge_change_lanes()  # 判断变道
            self.use_time_traffic_fd.write(f'_judge_change_lanes:{(time.time() - t_now) * 1000}ms\n');
            t_now = time.time()
        if self.event_sign['run_the_red_light']:
            self._judge_run_the_red_light([[0, 2, 0, 0, 0], [0, 2, 0, 0, 0]])  # 判断闯红灯
            self.use_time_traffic_fd.write(f'_judge_run_the_red_light:{(time.time() - t_now) * 1000}ms\n');
            t_now = time.time()
        if self.event_sign['spills_detect']:
            self._spills_detect()
            self.use_time_traffic_fd.write(f'_spills_detect:{(time.time() - t_now) * 1000}ms\n');
            t_now = time.time()
        if self.event_sign['accident_detect']:
            self._accident_detect()
            self.use_time_traffic_fd.write(f'_accident_detect:{(time.time() - t_now) * 1000}ms\n');
            t_now = time.time()
        self._save_id_info()

    def _init_abnormal_event_info(self):
        """初始化异常事件检测的信息"""
        self.cur_occupy_dedicated_lane = []  # 占用专用车道
        self.cur_people_occupy_motor_lane = []  # 行人在机动车道逗留
        self.cur_id_pix_v = {}
        self.cur_id_pix_v_by_angle = {}  # 航向角方向的速度向量
        self.cur_retrograde_for_motor = []  # 机动车逆行
        self.cur_retrograde_for_non_motor = []  # 非机动车逆行
        self.cur_speeding_for_motor = []  # 机动车超速
        self.cur_speeding_for_non_motor = []  # 非机动车超速
        self.cur_stroll = []  # 慢行
        self.cur_cross_line = []  # 压线
        self.cur_illegal_stop = []  # 违停
        self.cur_cross_lane_for_non_motor = []  # 非机动车横穿马路
        self.cur_cross_lane_for_people = []  # 行人横穿马路
        self.cur_cross_lane_for_motor = []  # 机动车横穿马路
        self.cur_run_the_red_light_for_motor = []  # 机动车闯红灯
        self.cur_run_the_red_light_for_people = []  # 行人闯红灯
        self.cur_occupy_bus_lane = []  # 占用公交车道
        self.cur_change_lanes = []  # 变道
        self.cur_change_lanes_illegal = []  # 违规变道
        self.cur_spills = []  # 抛洒物的位置
        self.cur_accident = []  # 交通事故的位置
        self.cur_occupy_emergency_lane = []  # EMERGENCY AREA

    def __get_unique_xy_spills_one_line(self, x1, y1, x2, y2):
        """获取两端点之间的去重后的抛洒物坐标"""
        xy = Line_Date.get_xy(x1, y1, x2, y2)
        spills_xy = self._bmp_to_spills_coordinate(np.array(xy))
        unique_spills_xy = []
        is_unique = {}
        for this_spills_xy in spills_xy:
            if tuple(this_spills_xy) in is_unique:
                continue
            is_unique[tuple(this_spills_xy)] = True

            unique_spills_xy.append(this_spills_xy)
        unique_spills_xy = np.array(unique_spills_xy)
        return unique_spills_xy

    def _get_spills_coordinate_continue_info(self):
        """当前位置连续检测出抛洒物的次数、时间，记录历史异常事件"""
        all_unhit_spills_coordinate_continue = set(self.spills_coordinate_continue_info.keys())

        # 对spills_matrix进行赋值
        # 检测出了
        for id, info in self.id_continue_stop_info.items():
            if info.hit_count <= 10:
                continue

            this_id_info = self.cur_id_info[id]
            class_id = this_id_info.class_id
            if class_id in Detect_Class_Type.motor_type:
                continue
            # 对于检测不是机动车类型的
            px, py = this_id_info.center_px, this_id_info.center_py
            spills_xy = self._bmp_to_spills_coordinate(np.array([px, py]))
            self.spills_matrix[spills_xy[0], spills_xy[1]] += 1
            if self.spills_matrix[spills_xy[0], spills_xy[1]] > 40:
                self.spills_matrix[spills_xy[0], spills_xy[1]] = 40
        # 没检测出
        for id, lane in self.cur_id_lane.items():  # 在常规车道和左转辅助道上的目标
            this_id_info = self.cur_id_info[id]
            class_id = this_id_info.class_id

            if class_id not in Detect_Class_Type.motor_type:
                continue
            # 对于检测为机动车的
            # 获取整车的所属spill坐标
            unique_front_spills_xy = self.__get_unique_xy_spills_one_line( \
                this_id_info.front_left_px, this_id_info.front_left_py,
                this_id_info.front_right_px, this_id_info.front_right_py)
            unique_left_spills_xy = self.__get_unique_xy_spills_one_line( \
                this_id_info.front_left_px, this_id_info.front_left_py,
                this_id_info.behind_left_px, this_id_info.behind_left_py)
            unique_spills_xy = np.zeros((len(unique_front_spills_xy), len(unique_left_spills_xy), 2), dtype=np.int)
            if (unique_front_spills_xy[0, :] == unique_left_spills_xy[0, :]).all():
                for i in range(len(unique_front_spills_xy)):
                    unique_spills_xy[i, :, :] = unique_left_spills_xy + (
                                unique_front_spills_xy[i, :] - unique_front_spills_xy[0, :])
            elif (unique_front_spills_xy[0, :] == unique_left_spills_xy[-1, :]).all():
                for i in range(len(unique_front_spills_xy)):
                    unique_spills_xy[i, :, :] = unique_left_spills_xy + (
                                unique_front_spills_xy[i, :] - unique_front_spills_xy[0, :])
            elif (unique_front_spills_xy[-1, :] == unique_left_spills_xy[-1, :]).all():
                for i in range(len(unique_front_spills_xy)):
                    unique_spills_xy[i, :, :] = unique_left_spills_xy + (
                                unique_front_spills_xy[len(unique_front_spills_xy) - 1 - i, :] - unique_front_spills_xy[
                                                                                                 len(
                                                                                                     unique_front_spills_xy) - 1,
                                                                                                 :])
            elif (unique_front_spills_xy[-1, :] == unique_left_spills_xy[0, :]).all():
                for i in range(len(unique_front_spills_xy)):
                    unique_spills_xy[i, :, :] = unique_left_spills_xy + (
                                unique_front_spills_xy[len(unique_front_spills_xy) - 1 - i, :] - unique_front_spills_xy[
                                                                                                 len(
                                                                                                     unique_front_spills_xy) - 1,
                                                                                                 :])
            unique_spills_xy = unique_spills_xy.reshape(-1, 2)

            self.spills_matrix[unique_spills_xy[:, 0], unique_spills_xy[:, 1]] = 0

            if len(str(lane)) != self._normal_lane_str_len:
                continue
            # 需要在变道判断之后
            if id not in self.cur_change_lanes:
                continue
            # 一个变道行为只判断一次
            if self.id_last_lane[id][-2].lane == self.id_last_lane[id][-1].lane:
                continue
            if id in self.cur_cross_lane_for_motor:  # 不对横穿马路的行为进行判断
                continue

            used_history_lane_info = self.id_last_lane[id][-10 if len(self.id_last_lane[id]) >= 10 else 0]
            # 变道了,获取变道前bmp速度单位向量
            # change_lane_v_px,change_lane_v_py=used_history_lane_info.v_px,used_history_lane_info.v_py
            change_lane_v_px, change_lane_v_py = used_history_lane_info.v_px_by_angle, used_history_lane_info.v_py_by_angle
            norm_v = np.sqrt(change_lane_v_px ** 2 + change_lane_v_py ** 2)
            normal_v = np.array([change_lane_v_px, change_lane_v_py]) / norm_v if norm_v != 0 \
                else np.array([change_lane_v_px, change_lane_v_py])

            change_lane_front_all_xy = Line_Date.get_xy(used_history_lane_info.detect_info.front_left_px,
                                                        used_history_lane_info.detect_info.front_left_py,
                                                        used_history_lane_info.detect_info.front_right_px,
                                                        used_history_lane_info.detect_info.front_right_py)
            change_lane_front_all_spills_xy = self._bmp_to_spills_coordinate(np.array(change_lane_front_all_xy))
            spills_xy_already = {}
            front_bmp_xy_already = {}
            for this_spills_xy in change_lane_front_all_spills_xy:
                if tuple(this_spills_xy) in spills_xy_already:  # 对spills去重
                    continue
                spills_xy_already[tuple(this_spills_xy)] = True

                for i in range(10, 20):
                    front_spills_xy = np.array(this_spills_xy) + i * normal_v
                    front_spills_xy = front_spills_xy.astype(np.int)
                    front_bmp_xy = self._spills_coordinate_to_bmp(front_spills_xy)

                    if tuple(front_bmp_xy) in front_bmp_xy_already:  # 去重，防止重复累加
                        continue
                    front_bmp_xy_already[tuple(front_bmp_xy)] = True
                    if self.config_matrix[front_bmp_xy[0], front_bmp_xy[1], Config_Matrix_Index.LANE_NO] != -1:
                        self.spills_matrix[front_spills_xy[0], front_spills_xy[1]] += 1

            self.spills_matrix[unique_spills_xy[:, 0], unique_spills_xy[:, 1]] = 0  # 在变道前的区域进行清零

        # 判断更新历史抛洒物事件
        mask = deepcopy(self.spills_matrix)
        mask[mask < 3] = 0;
        mask[mask >= 3] = 1
        center = []
        bbox = []
        self.cur_bbox = {}
        if np.max(mask) != 0:
            _, _, bbox, center_info = cv2.connectedComponentsWithStats(mask.T, 8)
            center = center_info[bbox[:, 4] < 100].astype(np.int)
            bbox = bbox[bbox[:, 4] < 100]
        for xy, box in zip(center, bbox):
            spills_x, spills_y = xy
            self.cur_bbox[(spills_x, spills_y)] = box
            coincidence = 0
            self.update_now_and_history((spills_x, spills_y), coincidence, self.spills_coordinate_continue_info,
                                        self.history_spills,
                                        all_unhit_spills_coordinate_continue)
        for spills_coordinate in all_unhit_spills_coordinate_continue:
            self.spills_coordinate_continue_info.pop(spills_coordinate)

    def _spills_detect(self):
        self._get_spills_coordinate_continue_info()
        for idx in self.spills_coordinate_continue_info:
            self.cur_spills.append(idx)

    def __get_unique_xy_accident_one_line(self, x1, y1, x2, y2):
        """获取两端点之间的去重后的事故坐标"""
        xy = Line_Date.get_xy(x1, y1, x2, y2)
        accident_xy = self._bmp_to_accident_coordinate(np.array(xy))
        unique_accident_xy = []
        is_unique = {}
        for this_accident_xy in accident_xy:
            if tuple(this_accident_xy) in is_unique:
                continue
            is_unique[tuple(this_accident_xy)] = True

            unique_accident_xy.append(this_accident_xy)
        unique_accident_xy = np.array(unique_accident_xy)
        return unique_accident_xy

    def _get_accident_coordinate_continue_info(self):
        """当前位置连续检测出事故的次数、时间，记录历史异常事件"""
        all_unhit_accident_coordinate_continue = set(self.accident_coordinate_continue_info.keys())

        # 对accident_matrix进行赋值
        # 机动车静止状态(后续需要与违停区分开)
        for id, info in self.id_continue_stop_info.items():
            if info.hit_count <= 10:
                continue

            this_id_info = self.cur_id_info[id]
            class_id = this_id_info.class_id
            if class_id not in Detect_Class_Type.motor_type:
                continue
            # 机动车静止
            px, py = this_id_info.center_px, this_id_info.center_py
            accident_xy = self._bmp_to_accident_coordinate(np.array([px, py]))
            self.accident_matrix[accident_xy[0], accident_xy[1]] += 1
            if self.accident_matrix[accident_xy[0], accident_xy[1]] > 20:
                self.accident_matrix[accident_xy[0], accident_xy[1]] = 20
        # 其他车变道情况
        for id, lane in self.cur_id_lane.items():  # 在常规车道和左转辅助道上的目标
            this_id_info = self.cur_id_info[id]
            class_id = this_id_info.class_id

            if class_id not in Detect_Class_Type.motor_type:
                continue
            # 对于检测为机动车的
            # 获取整车的所属accident坐标
            unique_front_accident_xy = self.__get_unique_xy_accident_one_line( \
                this_id_info.front_left_px, this_id_info.front_left_py,
                this_id_info.front_right_px, this_id_info.front_right_py)
            unique_left_accident_xy = self.__get_unique_xy_accident_one_line( \
                this_id_info.front_left_px, this_id_info.front_left_py,
                this_id_info.behind_left_px, this_id_info.behind_left_py)
            unique_accident_xy = np.zeros((len(unique_front_accident_xy), len(unique_left_accident_xy), 2),
                                          dtype=np.int)
            if (unique_front_accident_xy[0, :] == unique_left_accident_xy[0, :]).all():
                for i in range(len(unique_front_accident_xy)):
                    unique_accident_xy[i, :, :] = unique_left_accident_xy + (
                                unique_front_accident_xy[i, :] - unique_front_accident_xy[0, :])
            elif (unique_front_accident_xy[0, :] == unique_left_accident_xy[-1, :]).all():
                for i in range(len(unique_front_accident_xy)):
                    unique_accident_xy[i, :, :] = unique_left_accident_xy + (
                                unique_front_accident_xy[i, :] - unique_front_accident_xy[0, :])
            elif (unique_front_accident_xy[-1, :] == unique_left_accident_xy[-1, :]).all():
                for i in range(len(unique_front_accident_xy)):
                    unique_accident_xy[i, :, :] = unique_left_accident_xy + (
                                unique_front_accident_xy[len(unique_front_accident_xy) - 1 - i,
                                :] - unique_front_accident_xy[len(unique_front_accident_xy) - 1, :])
            elif (unique_front_accident_xy[-1, :] == unique_left_accident_xy[0, :]).all():
                for i in range(len(unique_front_accident_xy)):
                    unique_accident_xy[i, :, :] = unique_left_accident_xy + (
                                unique_front_accident_xy[len(unique_front_accident_xy) - 1 - i,
                                :] - unique_front_accident_xy[len(unique_front_accident_xy) - 1, :])
            unique_accident_xy = unique_accident_xy.reshape(-1, 2)

            self.accident_matrix[unique_accident_xy[:, 0], unique_accident_xy[:, 1]] = 0

            if len(str(lane)) != self._normal_lane_str_len:
                continue
            # 需要在变道判断之后
            if id not in self.cur_change_lanes:
                continue
            # 一个变道行为只判断一次
            if self.id_last_lane[id][-2].lane == self.id_last_lane[id][-1].lane:
                continue
            if id in self.cur_cross_lane_for_motor:  # 不对横穿马路的行为进行判断
                continue

            used_history_lane_info = self.id_last_lane[id][-10 if len(self.id_last_lane[id]) >= 10 else 0]
            # 变道了,获取变道前bmp速度单位向量
            # change_lane_v_px,change_lane_v_py=used_history_lane_info.v_px,used_history_lane_info.v_py
            change_lane_v_px, change_lane_v_py = used_history_lane_info.v_px_by_angle, used_history_lane_info.v_py_by_angle
            norm_v = np.sqrt(change_lane_v_px ** 2 + change_lane_v_py ** 2)
            normal_v = np.array([change_lane_v_px, change_lane_v_py]) / norm_v if norm_v != 0 \
                else np.array([change_lane_v_px, change_lane_v_py])

            change_lane_front_all_xy = Line_Date.get_xy(used_history_lane_info.detect_info.front_left_px,
                                                        used_history_lane_info.detect_info.front_left_py,
                                                        used_history_lane_info.detect_info.front_right_px,
                                                        used_history_lane_info.detect_info.front_right_py)
            change_lane_front_all_accident_xy = self._bmp_to_accident_coordinate(np.array(change_lane_front_all_xy))
            accident_xy_already = {}
            front_bmp_xy_already = {}
            for this_accident_xy in change_lane_front_all_accident_xy:
                if tuple(this_accident_xy) in accident_xy_already:  # 对accident去重
                    continue
                accident_xy_already[tuple(this_accident_xy)] = True

                for i in range(10, 15):
                    front_accident_xy = np.array(this_accident_xy) + i * normal_v
                    front_accident_xy = front_accident_xy.astype(np.int)
                    front_bmp_xy = self._accident_coordinate_to_bmp(front_accident_xy)

                    if tuple(front_bmp_xy) in front_bmp_xy_already:  # 去重，防止重复累加
                        continue
                    front_bmp_xy_already[tuple(front_bmp_xy)] = True
                    if self.config_matrix[front_bmp_xy[0], front_bmp_xy[1], Config_Matrix_Index.LANE_NO] != -1:
                        self.accident_matrix[front_accident_xy[0], front_accident_xy[1]] += 1

            self.accident_matrix[unique_accident_xy[:, 0], unique_accident_xy[:, 1]] = 0  # 在变道前的区域进行清零

        # 判断更新历史抛洒物事件
        index = np.where(self.accident_matrix > 5)
        for accident_x, accident_y in zip(index[0], index[1]):
            if (accident_x, accident_y) not in self.accident_coordinate_continue_info:
                coincidence = 0
                self.update_now_and_history((accident_x, accident_y), coincidence,
                                            self.accident_coordinate_continue_info,
                                            self.history_accident,
                                            all_unhit_accident_coordinate_continue)

        for accident_coordinate in all_unhit_accident_coordinate_continue:
            self.accident_coordinate_continue_info.pop(accident_coordinate)

    def _accident_detect(self):
        self._get_accident_coordinate_continue_info()
        for idx in self.accident_coordinate_continue_info:
            self.cur_accident.append(idx)

    def _get_id_continue_occupy_bus_lane_info(self, time_intervals):
        """连续占用公交车道的次数、时间，记录历史异常事件"""

        def _can_not_occupy_bus_lane(time_intervals):
            """判断是否不在可占用区间内
            time_intervals:[[start_hour,start_minute,end_hour,end_minute],...]
            """
            hm = datetime.datetime.now(tz.gettz('Asia/Shanghai')).strftime('%R')
            now_hour, now_minute = int(hm[:2]), int(hm[-2:])
            for start_hour, start_minute, end_hour, end_minute in time_intervals:
                if (start_hour < now_hour or (start_hour == now_hour and start_minute <= now_minute)) and \
                        (end_hour > now_hour or (end_hour == now_hour and end_minute >= now_minute)):
                    return True
            return False

        all_unhit_id_continue_occupy_bus_lane = set(self.id_continue_occupy_bus_lane_info.keys())
        for lane, ids in self.cur_lane_id.items():
            if self.lane_info[lane].is_bus_lane == 1:  # 公交车道
                for id in ids:
                    if self.cur_id_info[id].class_id not in Detect_Class_Type.bus and \
                            _can_not_occupy_bus_lane(time_intervals):  # 在时间段内
                        coincidence = 0
                        self.update_now_and_history(id, coincidence, self.id_continue_occupy_bus_lane_info,
                                                    self.history_occupy_bus_lane,
                                                    all_unhit_id_continue_occupy_bus_lane)
        for id in all_unhit_id_continue_occupy_bus_lane:
            self.id_continue_occupy_bus_lane_info.pop(id)

    def _detect_occupy_bus_lane(self, time_intervals):
        """检测占用公交车道"""
        self._get_id_continue_occupy_bus_lane_info(time_intervals)
        for id, info in self.id_continue_occupy_bus_lane_info.items():
            if info.hit_count < 10:
                continue
            self.cur_occupy_bus_lane.append(id)
            # info.hit_count=1

    def _get_id_history_lane(self):
        """获取id对应的历史所属车道"""
        for id, lane in self.cur_id_lane.items():
            if id not in self.id_history_lane:
                self.id_history_lane[id] = [History_Different_Lane_Info(lane)]
            if lane != self.id_history_lane[id][-1].lane:
                self.id_history_lane[id].append(History_Different_Lane_Info(lane))

    def _get_id_continue_change_lanes_info(self):
        """获取连续变道次数、时间，记录历史事件"""
        all_unhit_id_continue_change_lanes = set(self.id_continue_change_lanes_info.keys())

        # for id,history_pos_info in self.id_dist_list.items():
        # self.history_id_lane={}

        # ###########zhushi#############
        timeprint = False

        for id, lane in self.cur_id_lane.items():
            if len(str(lane)) != self._normal_lane_str_len:
                continue

            # this_id_gap=np.array(self.id_dist_list[id]).reshape(-1,len(self.id_dist_list[id][0]))
            # # dis_mean=np.mean(this_id_gap[:,1])
            # useful_data=this_id_gap[np.where(this_id_gap[:,2]<=0.2),:]
            this_id_info = self.cur_id_info[id]

            # ###########zhushi#############
            if not timeprint:
                time_local = time.localtime(this_id_info.cur_id_time)
                # 转换成新的时间格式(2016-05-05 20:28:54)
                dt = time.strftime("%Y-%m-%d %H:%M:%S", time_local)
                # if id in [6037, 6234, 6468, 6483, 6825, 7206, 7457, 8554, 8630, 9006, 9087, 9248, 9289, 9347, 9402, 9420, 9585, 9639, 9709]:
                # print(f'id:{id},time:{dt}')
                timeprint = True
            # ###########zhushi#############
            # if this_id_info.coincidence<0.6:
            #     continue

            # if id==6261:
            #     print('stop')

            v_px, v_py = (0, 0) if id not in self.cur_id_pix_v else self.cur_id_pix_v[id]
            v_px_by_angle, v_py_by_angle = self.cur_id_pix_v_by_angle[id]
            if id not in self.id_last_lane:
                self.id_last_lane[id] = []
                self.id_last_lane[id].append(
                    History_Continue_Lane_Info(lane, this_id_info, v_px, v_py, v_px_by_angle, v_py_by_angle))
            else:
                self.id_last_lane[id].append(
                    History_Continue_Lane_Info(lane, this_id_info, v_px, v_py, v_px_by_angle, v_py_by_angle))
                if len(self.id_last_lane[id]) >= 50:  # 10s内
                    self.id_last_lane[id].pop(0)

                if len(self.id_last_lane[id]) < 5:
                    continue

                if len(str(self.cur_id_lane[id])) == self._normal_lane_str_len:  # 常规车道
                    change_back = False  # 是否变回原来的车道
                    lane_idx_dic = {}  # 历史连续10帧的  lane:idx
                    for i, this_history_continue_lane_info in enumerate(
                            self.id_last_lane[id][-10 if len(self.id_last_lane[id]) >= 10 else 0:]):
                        this_lane = this_history_continue_lane_info.lane
                        if this_lane not in lane_idx_dic:
                            lane_idx_dic[this_lane] = [i]
                        else:
                            if i - lane_idx_dic[this_lane][-1] != 1:  # 防止抖动
                                change_back = True
                                break
                            else:
                                lane_idx_dic[this_lane].append(i)
                    if change_back:  # 变回原来的车道
                        continue
                    if len(lane_idx_dic) == 1:  # 只有一条车道
                        continue
                    if this_id_info.class_id not in Detect_Class_Type.motor_type:
                        continue  # 只保留机动车
                    # if id not in self.history_id_lane.keys():
                    #     pos_dist_list=np.array(self.id_dist_list[id])[:,1].tolist()
                    #     pose_dist=np.median(pos_dist_list)
                    #     if -1.8<pose_dist<1.8:
                    #         laneNum=112
                    #         laneCoin=self.id_dist_list[id][pos_dist_list.index(pose_dist)]
                    #     elif 1.9<pose_dist<5.5:
                    #         laneNum=113
                    #     elif -5.5<pose_dist<-1.9:
                    #         laneNum=111
                    #     else:
                    #         laneNum=lane
                    #     laneCoin
                    # else:

                    hasChangeSuddent = False
                    reallyChange = False
                    change_lane_coincidence = 0
                    # if id==6037:
                    #     print('stop')
                    pos_dist_list_np = np.array(self.id_dist_list[id][0:6])
                    pos_dist_list = pos_dist_list_np[:, 1]
                    pose_dist = np.median(pos_dist_list)
                    # for pos_coin in self.id_dist_list[id]:
                    pos_coin = self.id_dist_list[id][-1]
                    st_ed_gap = abs(pos_coin[1] - pose_dist)
                    # if this_id_info.class_id in [2,6,8]:
                    #     gap=1.1
                    # else:
                    gap = 1.85
                    if st_ed_gap >= gap:
                        reallyChange = True
                        if pos_coin[3] < 0.78:
                            hasChangeSuddent = True
                            change_lane_coincidence = pos_coin[3]
                    if reallyChange:
                        if not hasChangeSuddent:
                            pose_value = np.array(self.id_dist_list[id])[:, 3]
                            change_lane_coincidence = np.mean(pose_value)
                        if this_id_info.class_id in [2, 6, 8]:
                            change_lane_coincidence = change_lane_coincidence * 0.9
                        if this_id_info.x >= -55:
                            change_lane_coincidence = change_lane_coincidence * 0.9
                        if this_id_info.x >= 0:
                            change_lane_coincidence = change_lane_coincidence * 0.9
                        if this_id_info.x >= 55:
                            change_lane_coincidence = change_lane_coincidence * 0.9
                        ###zhushi

                        if this_id_info.v > 20:
                            discret = 1 - (this_id_info.v - 20) / 100
                            change_lane_coincidence = change_lane_coincidence * discret
                        # if len(self.id_history_lane[id])!=1:#同一大路上的不同道,防止穿过十字路口导致的处于不同道路产生的误判
                        #     if str(self.id_history_lane[id][-2].lane)[0]!=str(self.id_history_lane[id][-1].lane)[0]:
                        #         continue
                    else:
                        continue
                    # print(f'change_lane_id:{id},coincidence:{change_lane_coincidence}')

                    # ###########zhushi#############
                    # if change_lane_coincidence<0.8:
                    #     continue
                    # else:
                    #     print(f'change_lane_id:{id},coincidence:{change_lane_coincidence}')
                    #     print(f'id:{id},type:{this_id_info.class_id},speed:{this_id_info.v}')
                    #     # self.id_dist_list.pop(id)
                    #     # self.id_dist_list[id]=[pos_coin]
                    #     # if id not in self.save_change_lane.keys():
                    #     #     self.save_change_lane[id]=[dt, change_lane_coincidence]
                    #     # else:
                    #     self.save_change_lane[id]=[dt, change_lane_coincidence]
                    # ###########zhushi#############

                    if id not in self.cur_cross_lane_for_motor:  # 横穿马路与变道存在交集
                        self.cur_change_lanes.append(id)  # 变道
                        if self.config_matrix[
                            this_id_info.center_px, this_id_info.center_py, Config_Matrix_Index.FORBID_CROSS_LINE] == 1:
                            self.update_now_and_history(id, change_lane_coincidence, self.id_continue_change_lanes_info,
                                                        self.history_change_lanes,
                                                        all_unhit_id_continue_change_lanes,
                                                        start_forward_time=0)  # 违规变道
        for id in all_unhit_id_continue_change_lanes:
            self.id_continue_change_lanes_info.pop(id)

    def _judge_change_lanes(self):
        """判断变道"""
        self._get_id_continue_change_lanes_info()
        for id, info in self.id_continue_change_lanes_info.items():
            if info.hit_count < 4: continue
            self.cur_change_lanes_illegal.append(id)  # 违规变道
            # info.hit_count=1
            point_new = self.id_dist_list[id][-1]
            self.id_dist_list.pop(id)
            self.id_dist_list[id] = [point_new]
            # ###########zhushi#############
            self.save_change_lane[id] = [point_new]
            savejson = json.dumps(self.save_change_lane)
            with open(r'./save_change_lane.json', 'w') as f:
                f.write(savejson)

    def _get_id_continue_cross_lane_info(self):
        """获取横穿马路的连续次数、时间，记录历史事件"""

        def _get_angle_with_lane(angle, angle_out, angle_in):
            """获取目标朝向与路之间的小夹角"""
            angle_1 = abs(angle_out - angle)
            if angle_1 > 180: angle_1 -= 180  # 目标朝向与路的夹角
            angle_2 = 180 - angle_1
            return angle_1 if angle_1 < angle_2 else angle_2

        all_unhit_id_continue_cross_lane_for_people = set(self.id_continue_cross_lane_info_for_people.keys())
        all_unhit_id_continue_cross_lane_for_motor = set(self.id_continue_cross_lane_info_for_motor.keys())
        all_unhit_id_continue_cross_lane_for_non_motor = set(self.id_continue_cross_lane_info_for_non_motor.keys())
        for id, info in self.cur_id_info.items():
            if id in self.cur_id_lane and len(str(self.cur_id_lane[id])) == self._normal_lane_str_len:  # 常规车道
                if info.v < 0.5:
                    continue
                # if info.coincidence<0.6:
                #     continue
                # 历史车道为连续2车道
                if len(self.id_history_lane[id]) < 2:
                    continue
                if str(self.id_history_lane[id][-1].lane)[0] != str(self.id_history_lane[id][-2].lane)[0]:  # 东西南北不同
                    continue
                angle_with_lane = _get_angle_with_lane(info.angle / np.pi * 180, \
                                                       self.lane_info[self.id_history_lane[id][-1].lane].angle_out, \
                                                       self.lane_info[self.id_history_lane[id][-1].lane].angle_in)
                if angle_with_lane > 30:  # 朝向与车道夹角超过30度则判断为横穿马路
                    if info.class_id in Detect_Class_Type.people:
                        coincidence = 0.5
                        self.update_now_and_history(id, coincidence, self.id_continue_cross_lane_info_for_people,
                                                    self.history_cross_lane_for_people,
                                                    all_unhit_id_continue_cross_lane_for_people)
                    elif info.class_id in Detect_Class_Type.non_motor_type:
                        coincidence = 0.5
                        self.update_now_and_history(id, coincidence, self.id_continue_cross_lane_info_for_non_motor,
                                                    self.history_cross_lane_for_non_motor,
                                                    all_unhit_id_continue_cross_lane_for_non_motor)
                    elif info.class_id in Detect_Class_Type.motor_type:
                        coincidence = 0.5
                        self.update_now_and_history(id, coincidence, self.id_continue_cross_lane_info_for_motor,
                                                    self.history_cross_lane_for_motor,
                                                    all_unhit_id_continue_cross_lane_for_motor)
            else:  # 不在常规车道
                if info.class_id in Detect_Class_Type.people:
                    # 在十字中心区且不在人行道
                    if self.config_matrix[info.center_px, info.center_py, Config_Matrix_Index.CENTER_AREA] == 1 and \
                            self.config_matrix[info.center_px, info.center_py, Config_Matrix_Index.SIDEWALK] == -1 and \
                            info.v > 0.5:
                        coincidence = 0.5
                        self.update_now_and_history(id, coincidence, self.id_continue_cross_lane_info_for_people,
                                                    self.history_cross_lane_for_people,
                                                    all_unhit_id_continue_cross_lane_for_people)
        for id in all_unhit_id_continue_cross_lane_for_people:
            self.id_continue_cross_lane_info_for_people.pop(id)
        for id in all_unhit_id_continue_cross_lane_for_non_motor:
            self.id_continue_cross_lane_info_for_non_motor.pop(id)
        for id in all_unhit_id_continue_cross_lane_for_motor:
            self.id_continue_cross_lane_info_for_motor.pop(id)

    def _judge_cross_lane(self):
        """判断是否横穿马路"""
        self._get_id_continue_cross_lane_info()
        for id, info in self.id_continue_cross_lane_info_for_people.items():
            if info.hit_count < 10: continue
            self.cur_cross_lane_for_people.append(id)
        for id, info in self.id_continue_cross_lane_info_for_motor.items():
            if info.hit_count < 10: continue
            self.cur_cross_lane_for_motor.append(id)
        for id, info in self.id_continue_cross_lane_info_for_non_motor.items():
            if info.hit_count < 10: continue
            self.cur_cross_lane_for_non_motor.append(id)

    def _get_id_continue_run_the_red_light_info(self, states):
        """获取连续闯红灯的次数时间、记录历史事件"""
        all_unhit_id_continue_run_the_red_light_for_motor = set(
            self.id_continue_run_the_red_light_info_for_motor.keys())
        all_unhit_id_continue_run_the_red_light_for_people = set(
            self.id_continue_run_the_red_light_info_for_people.keys())

        self._get_traffic_light_state(states)
        self._get_id_states_of_red_light()

        for id in self.cur_id_info:
            if self.cur_id_info[id].class_id in Detect_Class_Type.motor_type:  # 机动车
                if id not in self.id_states_of_red_light_for_motor:
                    continue
                start_lane, start_px, start_py, end_px, end_py, angle, direction = \
                self.id_states_of_red_light_for_motor[id]
                traffic_light_index = int(str(start_lane)[0])

                if self.cur_id_info[id].v <= 1:  # 不考虑静止目标
                    continue

                # 瞬时判断(通过车道方向与红绿灯判断),瞬时判断成功的不进行后续判断
                if self.lane_info[start_lane].lane_direction == Lane_Direction.STRAIGHT:
                    if self.traffic_light_states[traffic_light_index].straight_state == Traffic_Light.RED:
                        coincidence = 1
                        self.update_now_and_history(id, coincidence, self.id_continue_run_the_red_light_info_for_motor,
                                                    self.history_run_the_red_light_for_motor,
                                                    all_unhit_id_continue_run_the_red_light_for_motor)
                        continue
                elif self.lane_info[start_lane].lane_direction == Lane_Direction.LEFT:
                    if self.traffic_light_states[traffic_light_index].left_state == Traffic_Light.RED:
                        coincidence = 1
                        self.update_now_and_history(id, coincidence, self.id_continue_run_the_red_light_info_for_motor,
                                                    self.history_run_the_red_light_for_motor,
                                                    all_unhit_id_continue_run_the_red_light_for_motor)
                        continue
                elif self.lane_info[start_lane].lane_direction == Lane_Direction.RIGHT:
                    if self.traffic_light_states[traffic_light_index].right_state == Traffic_Light.RED:
                        coincidence = 1
                        self.update_now_and_history(id, coincidence, self.id_continue_run_the_red_light_info_for_motor,
                                                    self.history_run_the_red_light_for_motor,
                                                    all_unhit_id_continue_run_the_red_light_for_motor)
                        continue
                elif self.lane_info[start_lane].lane_direction == Lane_Direction.STRAIGHT_LEFT:
                    if self.traffic_light_states[traffic_light_index].straight_state == Traffic_Light.RED \
                            and self.traffic_light_states[traffic_light_index].left_state == Traffic_Light.RED:
                        coincidence = 1
                        self.update_now_and_history(id, coincidence, self.id_continue_run_the_red_light_info_for_motor,
                                                    self.history_run_the_red_light_for_motor,
                                                    all_unhit_id_continue_run_the_red_light_for_motor)
                        continue
                elif self.lane_info[start_lane].lane_direction == Lane_Direction.STRAIGHT_RIGHT:
                    if self.traffic_light_states[traffic_light_index].straight_state == Traffic_Light.RED \
                            and self.traffic_light_states[traffic_light_index].right_state == Traffic_Light.RED:
                        coincidence = 1
                        self.update_now_and_history(id, coincidence, self.id_continue_run_the_red_light_info_for_motor,
                                                    self.history_run_the_red_light_for_motor,
                                                    all_unhit_id_continue_run_the_red_light_for_motor)
                        continue
                elif self.lane_info[start_lane].lane_direction == Lane_Direction.STRAIGHT_LEFT_RIGHT:
                    if self.traffic_light_states[traffic_light_index].straight_state == Traffic_Light.RED \
                            and self.traffic_light_states[traffic_light_index].left_state == Traffic_Light.RED \
                            and self.traffic_light_states[traffic_light_index].right_state == Traffic_Light.RED:
                        coincidence = 1
                        self.update_now_and_history(id, coincidence, self.id_continue_run_the_red_light_info_for_motor,
                                                    self.history_run_the_red_light_for_motor,
                                                    all_unhit_id_continue_run_the_red_light_for_motor)
                        continue

                # 延迟判断(通过车实际的左右拐情况和红绿灯判断)
                if angle < 15:  # 直行
                    if self.traffic_light_states[traffic_light_index].straight_state == Traffic_Light.RED:
                        coincidence = 1
                        self.update_now_and_history(id, coincidence, self.id_continue_run_the_red_light_info_for_motor,
                                                    self.history_run_the_red_light_for_motor,
                                                    all_unhit_id_continue_run_the_red_light_for_motor)
                else:  # 拐弯
                    if direction == Lane_Direction.LEFT and \
                            self.traffic_light_states[traffic_light_index].left_state == Traffic_Light.RED:
                        coincidence = 1
                        self.update_now_and_history(id, coincidence, self.id_continue_run_the_red_light_info_for_motor,
                                                    self.history_run_the_red_light_for_motor,
                                                    all_unhit_id_continue_run_the_red_light_for_motor)
                    elif direction == Lane_Direction.RIGHT and \
                            self.traffic_light_states[traffic_light_index].right_state == Traffic_Light.RED:
                        coincidence = 1
                        self.update_now_and_history(id, coincidence, self.id_continue_run_the_red_light_info_for_motor,
                                                    self.history_run_the_red_light_for_motor,
                                                    all_unhit_id_continue_run_the_red_light_for_motor)
            elif self.cur_id_info[id].class_id in Detect_Class_Type.people:  # 人
                # if id not in self.id_states_of_red_light_for_people:
                #     continue
                lane = self.id_states_of_red_light_for_people[id]
                if lane == -1:
                    continue
                lane = self.id_states_of_red_light_for_people[id]
                traffic_light_index = int(str(lane)[0])
                if self.cur_id_info[id].v <= 0.5:  # 不考虑静止目标
                    continue

                if self.traffic_light_states[traffic_light_index].people_state == Traffic_Light.RED:
                    coincidence = 0
                    self.update_now_and_history(id, coincidence, self.id_continue_run_the_red_light_info_for_people,
                                                self.history_run_the_red_light_for_people,
                                                all_unhit_id_continue_run_the_red_light_for_people)

        for id in all_unhit_id_continue_run_the_red_light_for_motor:
            self.id_continue_run_the_red_light_info_for_motor.pop(id)
        for id in all_unhit_id_continue_run_the_red_light_for_people:
            self.id_continue_run_the_red_light_info_for_people.pop(id)

    def _judge_run_the_red_light(self, states):
        """判断是否闯红灯"""
        self._get_id_continue_run_the_red_light_info(states)
        for id in self.id_continue_run_the_red_light_info_for_motor:
            self.cur_run_the_red_light_for_motor.append(id)
        for id in self.id_continue_run_the_red_light_info_for_people:
            self.cur_run_the_red_light_for_people.append(id)

    def _get_traffic_light_state(self, states):
        """红绿灯解析接口
        states:[[0,0,0,0,0],[0,0,0,0,0]] #车道1_3红绿灯状态  车道2_4红绿灯状态
        假设只有红黄绿三种颜色状态0,1,2. 左转、直行、右转、非机动车、人行道
        """

        def convert_state(in_state):
            out_state = Traffic_Light.INIT
            if in_state == 0:
                out_state = Traffic_Light.RED
            elif in_state == 1:
                out_state = Traffic_Light.YELLOW
            elif in_state == 2:
                out_state = Traffic_Light.GREEN
            return out_state

        self.traffic_light_states = {}
        for i in range(1, 5):
            this_index = (i + 1) % 2  # 东西向、南北向的红绿灯情况一致
            this_state = states[this_index]

            left_state = convert_state(this_state[0])
            straight_state = convert_state(this_state[1])
            right_state = convert_state(this_state[2])
            non_motor_state = convert_state(this_state[3])
            people_state = convert_state(this_state[4])

            self.traffic_light_states[i] = Traffic_Light_State(left_state, 0, straight_state, 0, right_state, 0, \
                                                               non_motor_state, 0, people_state, 0)

    def _is_a_right_b(self, a_x, a_y, b_x, b_y):
        """判断a在b的顺时针(往右拐)还是逆时针(往左拐)方向"""
        return np.cross(np.array([a_x, a_y, 0]), np.array([b_x, b_y, 0]))[2] > 0

    def _get_id_states_of_red_light(self):
        """获取 id对应的刚进交汇区的位置 和 当前位置  角度和方向"""
        for id, info in self.cur_id_info.items():
            c_px, c_py = self.cur_id_info[id].center_px, self.cur_id_info[id].center_py
            if info.class_id in Detect_Class_Type.motor_type:
                center_area = self.config_matrix[c_px, c_py, Config_Matrix_Index.CENTER_AREA]
                if center_area != 1:
                    if id in self.id_states_of_red_light_for_motor:
                        self.id_states_of_red_light_for_motor.pop(id)
                    continue
                if id not in self.id_states_of_red_light_for_motor:
                    start_lane = self._get_nearest_out_lane(c_px, c_py)
                    # if start_lane is None:#现阶段只对在车道检测到之后进入十字中心区的目标进行记录
                    #     continue
                    if self.lane_info[start_lane].out_in == 1:  # 只记录驶入十字中心区的目标
                        continue
                    # 进入十字中间区的起始车道,起始x像素,起始y像素,当前x像素,当前y像素,角度,方向
                    self.id_states_of_red_light_for_motor[id] = [start_lane, c_px, c_py, c_px, c_py, 0, 0]
                else:
                    now_lane = self.config_matrix[c_px, c_py, Config_Matrix_Index.LANE_NO] \
                        if self.config_matrix[c_px, c_py, Config_Matrix_Index.LANE_NO] != -1 else -1
                    if now_lane == self.id_states_of_red_light_for_motor[id][0]:  # 不对在车道内等待的车辆更新
                        continue
                    self.id_states_of_red_light_for_motor[id][3] = c_px
                    self.id_states_of_red_light_for_motor[id][4] = c_py
                    start_lane, start_px, start_py, end_px, end_py, _, _ = self.id_states_of_red_light_for_motor[id]
                    vector = [end_px - start_px, end_py - start_py]
                    lane_vector = [self.lane_info[start_lane].v_px, self.lane_info[start_lane].v_py]

                    cos_angle = (vector[0] * lane_vector[0] + vector[1] * lane_vector[1]) / \
                                (np.sqrt(vector[0] ** 2 + vector[1] ** 2) * np.sqrt(
                                    lane_vector[0] ** 2 + lane_vector[1] ** 2))
                    angle = np.arccos(cos_angle) * 180 / np.pi
                    self.id_states_of_red_light_for_motor[id][5] = angle

                    # 图像以左上角为原点,下方为正。用叉乘判断后需要翻转,叉乘判断为左,则实际在右
                    is_right = self._is_a_right_b(vector[0], vector[1], lane_vector[0], lane_vector[1])
                    direction = Lane_Direction.LEFT if is_right else Lane_Direction.RIGHT
                    self.id_states_of_red_light_for_motor[id][6] = direction
            elif info.class_id in Detect_Class_Type.people:
                self.id_states_of_red_light_for_people[id] = self.config_matrix[
                    c_px, c_py, Config_Matrix_Index.SIDEWALK]

    def _init_for_nearest_out_lane(self):
        """初始化驶出向的左转右转角度幅值"""
        self._out_lane = []  # 驶出车道的车道名
        self._pxy_of_out_lane = []  # 驶出车道对应的起始像素坐标
        for lane, info in self.lane_info.items():
            if len(str(lane)) != self._normal_lane_str_len:
                continue
            self._out_lane.append(lane)
            self._pxy_of_out_lane.append([info.start_px, info.start_py])

        self._pxy_of_out_lane = np.array(self._pxy_of_out_lane)

    def _get_nearest_out_lane(self, px, py):
        """获取刚进交汇区的位置最靠近的路"""
        if self.config_matrix[px, py, Config_Matrix_Index.LANE_NO] != -1:
            return self.lane_no_lane_name[self.config_matrix[px, py, Config_Matrix_Index.LANE_NO]]
        else:
            nearest_lane_index = np.argmin(np.linalg.norm(self._pxy_of_out_lane - [px, py], axis=1))
            return self._out_lane[nearest_lane_index]  # 在路中间容易将起点误判到错误车道
            # return None

    def _del_long_time_miss_history(self, history_event):
        """删除太长时间未出现的历史记录"""
        long_time_miss_idxs = set(history_event.keys())
        for idx in history_event:
            if self.cur_time - history_event[idx][-1].end_time <= 10:
                long_time_miss_idxs.remove(idx)
        for idx in long_time_miss_idxs:  # 超过10s
            history_event.pop(idx)

    def update_now_and_history(self, idx, coincidence, idx_continue_info, history_event, unhit_idxs,
                               start_forward_time=0):
        """更新最新事件连续出现的帧数和时间段，并保存历史时间段中

        @param:
            idx: id或者是spills的(x,y)
            unhit_idxs: 之前idx_continue_info中未出现的idx
            start_forward_time: 变道这种不能实时判断得到变道前的行为,规定前2s作为开始时间
        """
        if idx not in idx_continue_info:
            idx_continue_info[idx] = Event_Continue_Info(start_forward_time, self.cur_time, coincidence)
            this_idx_continue_info = idx_continue_info[idx]
            this_idx_continue_info.longitude = self.spills_lon_lat[
                idx[0], idx[1], 0] if idx not in self.cur_id_info else self.cur_id_info[idx].longitude
            this_idx_continue_info.latitude = self.spills_lon_lat[idx[0], idx[1], 1] if idx not in self.cur_id_info else \
            self.cur_id_info[idx].latitude

            # 添加到历史事件中
            if idx not in history_event:
                self.event_cnt += 1
                this_idx_continue_info.event_no = self.event_cnt
                history_event[idx] = [this_idx_continue_info]
            else:
                this_history_event = history_event[idx]
                if this_idx_continue_info.start_time - this_history_event[-1].end_time <= 60:  # 60s内同一idx检测出同一类型事件
                    this_history_event[-1].end_time = this_idx_continue_info.end_time
                    this_history_event[-1].longitude = this_idx_continue_info.longitude
                    this_history_event[-1].latitude = this_idx_continue_info.latitude
                    this_idx_continue_info.event_no = this_history_event[-1].event_no
                else:
                    self.event_cnt += 1
                    this_idx_continue_info.event_no = self.event_cnt
                    history_event[idx] = [this_idx_continue_info]  # 只记录最新的  之前是append保存历史的所有记录(耗内存)
        else:
            unhit_idxs.remove(idx)  # 移除当前帧出现的上一帧出现的目标
            this_idx_continue_info = idx_continue_info[idx]
            this_idx_continue_info.hit_count += 1
            if this_idx_continue_info.coincidence >= coincidence:
                this_idx_continue_info.coincidence = coincidence
            # this_idx_continue_info.end_time=time.time()
            this_idx_continue_info.end_time = self.cur_id_info[idx].cur_id_time
            this_idx_continue_info.longitude = self.spills_lon_lat[
                idx[0], idx[1], 0] if idx not in self.cur_id_info else self.cur_id_info[idx].longitude
            this_idx_continue_info.latitude = self.spills_lon_lat[idx[0], idx[1], 1] if idx not in self.cur_id_info else \
            self.cur_id_info[idx].latitude

            if idx not in history_event:  # 超时被删
                self.event_cnt += 1
                this_idx_continue_info.event_no = self.event_cnt
                history_event[idx] = [this_idx_continue_info]
            else:
                # 更新历史事件
                history_event[idx][-1].end_time = this_idx_continue_info.end_time

        if self.del_flag:  # 删除超时的历史记录,每帧调用太耗时
            self._del_long_time_miss_history(history_event)

    def _get_id_continue_occupy_dedicated_lane_info(self):
        """连续占用其他车道的帧数、时间，并保存到历史异常事件中"""
        all_unhit_id_continue_occupy_dedicated_lane = set(self.id_continue_occupy_dedicated_lane_info.keys())
        all_unhit_id_continue_people_occupy_motor_lane = set(self.id_continue_people_occupy_motor_lane_info.keys())
        for lane, ids in self.cur_lane_id.items():
            if self.lane_info[lane].is_non_motorway == Motorway_Type.MOTORWAY:  # 机动车道
                for id_ in ids:
                    if self.cur_id_info[id_].coincidence < 0.6:
                        continue
                    coincidence = self.id_dist_list[id_][-1][3] * self.cur_id_info[id_].coincidence
                    volum = self.cur_id_info[id_].w * self.cur_id_info[id_].l * self.cur_id_info[id_].h
                    if self.cur_id_info[id_].class_id in Detect_Class_Type.non_motor_type:  # 非机动车
                        self.update_now_and_history(id_, coincidence, self.id_continue_occupy_dedicated_lane_info,
                                                    self.history_occupy_dedicated_lane,
                                                    all_unhit_id_continue_occupy_dedicated_lane)
                    elif self.cur_id_info[id_].class_id in Detect_Class_Type.people:  # 行人在机动车道逗留
                        if volum < 1:
                            continue
                        if volum >= 3:
                            continue
                        if id_ in self.history_speed_info.keys():
                            v_people = self.history_speed_info[id_].moving_average_speed[-1]
                            if v_people < 0.5:
                                continue
                        self.update_now_and_history(id_, coincidence, self.id_continue_people_occupy_motor_lane_info,
                                                    self.history_people_occupy_motor_lane,
                                                    all_unhit_id_continue_people_occupy_motor_lane)
            # elif self.lane_info[lane].is_non_motorway==Motorway_Type.NON_MOTORWAY:#非机动车道
            #     for id_ in ids:
            #         if self.cur_id_info[id_].class_id in Detect_Class_Type.motor_type:#机动车
            #             self.update_now_and_history(id_,self.id_continue_occupy_dedicated_lane_info,
            #                                         self.history_occupy_dedicated_lane,
            #                                         all_unhit_id_continue_occupy_dedicated_lane)
        # 删除不连续
        for id in all_unhit_id_continue_occupy_dedicated_lane:
            self.id_continue_occupy_dedicated_lane_info.pop(id)
        for id in all_unhit_id_continue_people_occupy_motor_lane:
            self.id_continue_people_occupy_motor_lane_info.pop(id)

    def _get_id_continue_occupy_emergency_lane_info(self):
        """连续占用其他车道的帧数、时间，并保存到历史异常事件中"""
        all_unhit_id_continue_occupy_emergency_lane = set(self.id_continue_occupy_emergency_lane_info.keys())
        for id_, this_id_info in self.cur_id_info.items():
            if self.config_matrix[
                this_id_info.center_px, this_id_info.center_py, Config_Matrix_Index.EMERGENCY_AREA] == 1:
                coincidence = self.id_dist_list[id_][-1][3]
                self.update_now_and_history(id_, coincidence, self.id_continue_occupy_emergency_lane_info,
                                            self.history_occupy_emergency_lane,
                                            all_unhit_id_continue_occupy_emergency_lane)
        # 删除不连续
        for id in all_unhit_id_continue_occupy_emergency_lane:
            self.id_continue_occupy_emergency_lane_info.pop(id)

    def _detect_occupy_dedicated_lane(self):
        """检测非机动车占用机动车道,机动车占用非机动车道,行人在机动车道逗留"""
        self._get_id_continue_occupy_dedicated_lane_info()

        for id, info in self.id_continue_occupy_dedicated_lane_info.items():
            if info.hit_count < 10:
                continue
            self.cur_occupy_dedicated_lane.append(id)
            # info.hit_count=1

        for id, info in self.id_continue_people_occupy_motor_lane_info.items():
            if info.hit_count < 10:
                continue
            self.cur_people_occupy_motor_lane.append(id)
            # info.hit_count=1

    def _detect_occupy_emergency_lane(self):
        """occupy emergency lane"""
        self._get_id_continue_occupy_emergency_lane_info()

        for id, info in self.id_continue_occupy_emergency_lane_info.items():
            if info.hit_count < 10: continue
            self.cur_occupy_emergency_lane.append(id)
            # info.hit_count=1

    def _get_cur_pix_v(self):
        """获取当前帧目标在雷达bmp的像素速度
        """
        for id in self.cur_id_info:
            if id in self.last_id_info:  # 在直道上并且上一帧也存在该目标
                self.cur_id_pix_v[id] = [self.cur_id_info[id].center_px - self.last_id_info[id].center_px, \
                                         self.cur_id_info[id].center_py - self.last_id_info[id].center_py]  # 当前速度

    def _get_cur_pix_v_by_angle(self):
        """获取当前帧目标在雷达bmp的像素速度向量
        """
        for id, info in self.cur_id_info.items():
            theta = 1.5 * np.pi - info.angle
            self.cur_id_pix_v_by_angle[id] = [np.cos(theta), np.sin(theta)]  # 当前速度

    def _get_id_continue_retrograde_info(self):
        """获取连续逆行的帧数、时间"""
        all_unhit_id_continue_retrograde_for_motor = set(self.id_continue_retrograde_info_for_motor.keys())
        all_unhit_id_continue_retrograde_for_non_motor = set(self.id_continue_retrograde_info_for_non_motor.keys())
        for id in self.cur_id_pix_v:
            if id not in self.cur_id_lane:  # 排除非道路区
                continue
            if len(str(self.cur_id_lane[id])) != self._normal_lane_str_len:  # 排除左转辅助道等非常规车道
                continue
            this_v_px, this_v_py = self.cur_id_pix_v[id]
            norm_v_pxy = np.sqrt(this_v_px ** 2 + this_v_py ** 2)
            if norm_v_pxy < 3:  # 防止基本静止的物体抖动造成的误判
                continue
            this_lane_v_px = self.lane_info[self.cur_id_lane[id]].v_px
            this_lane_v_py = self.lane_info[self.cur_id_lane[id]].v_py
            cos_angle = (this_lane_v_px * this_v_px + this_lane_v_py * this_v_py) / \
                        (np.sqrt(this_lane_v_px ** 2 + this_lane_v_py ** 2) * norm_v_pxy)
            theta = np.arccos(cos_angle) * 180 / np.pi
            if theta > 180 - 30:
                coincidence = self.id_dist_list[id][-1][3] * self.cur_id_info[id].coincidence
                # coincidence=self.id_dist_list[id][-1][3]
                if self.cur_id_info[id].coincidence < 0.6:
                    continue
                if self.cur_id_info[id].class_id in Detect_Class_Type.motor_type:
                    self.update_now_and_history(id, coincidence, self.id_continue_retrograde_info_for_motor,
                                                self.history_retrograde_for_motor,
                                                all_unhit_id_continue_retrograde_for_motor)
                elif self.cur_id_info[id].class_id in Detect_Class_Type.non_motor_type:
                    self.update_now_and_history(id, coincidence, self.id_continue_retrograde_info_for_non_motor,
                                                self.history_retrograde_for_non_motor,
                                                all_unhit_id_continue_retrograde_for_non_motor)

        for id in all_unhit_id_continue_retrograde_for_motor:
            self.id_continue_retrograde_info_for_motor.pop(id)
        for id in all_unhit_id_continue_retrograde_for_non_motor:
            self.id_continue_retrograde_info_for_non_motor.pop(id)

    def _judge_retrograde(self):
        """判断是否逆行. 机动车 和 非机动车
        """
        self._get_id_continue_retrograde_info()

        for id, info in self.id_continue_retrograde_info_for_motor.items():
            if info.hit_count < 20: continue
            self.cur_retrograde_for_motor.append(id)
            # info.hit_count=10

        for id, info in self.id_continue_retrograde_info_for_non_motor.items():
            if info.hit_count < 20: continue
            self.cur_retrograde_for_non_motor.append(id)
            # info.hit_count=10

    def _get_id_continue_speeding_info(self):
        """获取连续超速帧数、时间"""
        ### linxiao0517 ####
        all_unhit_id_continue_speeding_for_motor = set(self.id_continue_speeding_info_for_motor.keys())
        all_unhit_id_continue_speeding_for_non_motor = set(self.id_continue_speeding_info_for_non_motor.keys())
        for id, info in self.cur_id_info.items():
            if id in self.history_speed_info and len(
                    self.history_speed_info[id].moving_average_speed) < 50:  # tracking time > 5s think is real target
                continue
            max_v = self.lane_info[self.cur_id_lane[id]].max_v if id in self.cur_id_lane else self.param['max_v'] / 3.6
            try:
                mean_v = sum(self.history_speed_info[id].moving_average_speed) / len(
                    self.history_speed_info[id].moving_average_speed)
                if self.history_speed_info[id].moving_average_speed[-1] > max_v and mean_v > max_v and abs(
                        self.history_speed_info[id].acceleration[-1]) < 5:

                    # coincidence = self.id_dist_list[id][-1][3]
                    coincidence = self.cur_id_info[id].coincidence
                    if self.cur_id_info[id].class_id in Detect_Class_Type.motor_type:
                        self.update_now_and_history(id, coincidence, self.id_continue_speeding_info_for_motor,
                                                    self.history_speeding_for_motor,
                                                    all_unhit_id_continue_speeding_for_motor)
                    elif self.cur_id_info[id].class_id in Detect_Class_Type.non_motor_type:
                        self.update_now_and_history(id, coincidence, self.id_continue_speeding_info_for_non_motor,
                                                    self.history_speeding_for_non_motor,
                                                    all_unhit_id_continue_speeding_for_non_motor)
            except:
                print('ERROR_speeding:', id)

        for id in all_unhit_id_continue_speeding_for_motor:  # 删除历史记录中在当前帧未出现的id对应记录
            self.id_continue_speeding_info_for_motor.pop(id)
        for id in all_unhit_id_continue_speeding_for_non_motor:
            self.id_continue_speeding_info_for_non_motor.pop(id)

        # all_unhit_id_continue_speeding_for_motor=set(self.id_continue_speeding_info_for_motor.keys())
        # all_unhit_id_continue_speeding_for_non_motor=set(self.id_continue_speeding_info_for_non_motor.keys())
        # for id,info in self.cur_id_info.items():
        #     max_v=self.lane_info[self.cur_id_lane[id]].max_v if id in self.cur_id_lane else self.param['max_v']/3.6
        #     if self.cur_id_info[id].v> max_v:
        #         if id in self.last_id_info:
        #             id_accerlate=(self.cur_id_info[id].v - self.last_id_info[id].v)/ \
        #                          (self.cur_id_info[id].cur_id_time-self.last_id_info[id].cur_id_time)
        #             # 加速过大，速度异常
        #             if id_accerlate >= 10:
        #                 continue
        #         else:
        #             continue
        #         # if self.cur_id_info[id].coincidence<0.5:
        #         #     continue
        #         coincidence=self.id_dist_list[id][-1][3]
        #         if self.cur_id_info[id].class_id in Detect_Class_Type.motor_type:
        #             self.update_now_and_history(id,coincidence,self.id_continue_speeding_info_for_motor,
        #                                         self.history_speeding_for_motor,
        #                                         all_unhit_id_continue_speeding_for_motor)
        #         elif self.cur_id_info[id].class_id in Detect_Class_Type.non_motor_type:
        #             self.update_now_and_history(id,coincidence,self.id_continue_speeding_info_for_non_motor,
        #                                         self.history_speeding_for_non_motor,
        #                                         all_unhit_id_continue_speeding_for_non_motor)
        #
        # for id in all_unhit_id_continue_speeding_for_motor:#删除历史记录中在当前帧未出现的id对应记录
        #     self.id_continue_speeding_info_for_motor.pop(id)
        # for id in all_unhit_id_continue_speeding_for_non_motor:
        #     self.id_continue_speeding_info_for_non_motor.pop(id)

    def _judge_speeding(self):
        """判断是否超速"""
        self._get_id_continue_speeding_info()
        for id, info in self.id_continue_speeding_info_for_motor.items():
            if info.hit_count < 10: continue
            self.cur_speeding_for_motor.append(id)
            # info.hit_count=1
        for id, info in self.id_continue_speeding_info_for_non_motor.items():
            if info.hit_count < 10: continue
            self.cur_speeding_for_non_motor.append(id)
            # info.hit_count=1

    def _get_id_continue_stroll_info(self):
        """获取连续慢行帧数、时间,保存历史事件"""
        """linxiao0522"""
        all_unhit_id_continue_stroll = set(self.id_continue_stroll_info.keys())
        for id, info in self.cur_id_info.items():
            if id in self.history_speed_info and len(
                    self.history_speed_info[id].moving_average_speed) < 50:  # tracking time > 5s think is real target
                continue
            min_v = self.lane_info[self.cur_id_lane[id]].min_v if id in self.cur_id_lane else self.param['min_v'] / 3.6
            try:
                mean_v = sum(self.history_speed_info[id].moving_average_speed) / len(
                    self.history_speed_info[id].moving_average_speed)
                if (0.5 < self.history_speed_info[id].moving_average_speed[-1] < min_v) and (
                        0.5 < mean_v < min_v) and abs(self.history_speed_info[id].acceleration[-1]) < 1:
                    if self.cur_id_info[id].class_id in Detect_Class_Type.motor_type:
                        # if self.cur_id_info[id].coincidence<0.6:
                        #     continue
                        # coincidence = self.id_dist_list[id][-1][3]
                        coincidence = self.cur_id_info[id].coincidence
                        self.update_now_and_history(id, coincidence, self.id_continue_stroll_info, self.history_stroll,
                                                    all_unhit_id_continue_stroll)
            except:
                print('ERROR_speeding:', id)
        for id in all_unhit_id_continue_stroll:
            self.id_continue_stroll_info.pop(id)

        # all_unhit_id_continue_stroll=set(self.id_continue_stroll_info.keys())
        # for id,info in self.cur_id_info.items():
        #     min_v=self.lane_info[self.cur_id_lane[id]].min_v if id in self.cur_id_lane else self.param['min_v']/3.6
        #     if 0.5<self.cur_id_info[id].v< min_v:
        #         if self.cur_id_info[id].class_id in Detect_Class_Type.motor_type:
        #             # if self.cur_id_info[id].coincidence<0.6:
        #             #     continue
        #             coincidence=self.id_dist_list[id][-1][3]
        #             self.update_now_and_history(id,coincidence,self.id_continue_stroll_info,
        #                                         self.history_stroll,
        #                                         all_unhit_id_continue_stroll)
        # for id in all_unhit_id_continue_stroll:#删除历史记录中在当前帧未出现的id对应记录
        #     self.id_continue_stroll_info.pop(id)

    def _judge_stroll(self):
        """判断慢行"""
        self._get_id_continue_stroll_info()
        for id, info in self.id_continue_stroll_info.items():
            if info.hit_count < 10: continue
            # 连续10帧速度满足慢行条件
            if id in self.cur_id_lane:
                lane = self.cur_id_lane[id]
                this_id_info = self.cur_id_info[id]
                pc_id = this_id_info.pc_id
                if self.lane_local_static_congestion_state[lane][
                    pc_id].value >= Congestion_Level.moderately_congested.value:
                    continue  # 如果当前车道中度拥堵及以上，不判断为慢行
            self.cur_stroll.append(id)
            # info.hit_count=1

    def _get_id_continue_cross_line_info(self):
        """获取连续压线的帧数、时间，保存历史事件"""
        all_unhit_id_continue_cross_line = set(self.id_continue_cross_line_info.keys())
        # N=5#分4份,5个点
        for lane, ids in self.cur_lane_id.items():
            for id in ids:
                c_px, c_py = self.cur_id_info[id].center_px, self.cur_id_info[id].center_py
                if self.cur_id_info[id].class_id not in Detect_Class_Type.motor_type:
                    continue
                fl_px, fr_px = self.cur_id_info[id].front_left_px, self.cur_id_info[id].front_right_px
                fl_py, fr_py = self.cur_id_info[id].front_left_py, self.cur_id_info[id].front_right_py
                bl_px, br_px = self.cur_id_info[id].behind_left_px, self.cur_id_info[id].behind_right_px
                bl_py, br_py = self.cur_id_info[id].behind_left_py, self.cur_id_info[id].behind_right_py
                if not (self.config_matrix[fl_px, fl_py, Config_Matrix_Index.FORBID_CROSS_LINE] == 1 and \
                        self.config_matrix[fr_px, fr_py, Config_Matrix_Index.FORBID_CROSS_LINE] == 1 and \
                        self.config_matrix[bl_px, bl_py, Config_Matrix_Index.FORBID_CROSS_LINE] == 1 and \
                        self.config_matrix[br_px, br_py, Config_Matrix_Index.FORBID_CROSS_LINE] == 1):
                    continue
                # 四周都在禁止压线区

                # all_px,all_py=[],[]
                # all_px.append(fl_px);all_py.append(fl_py)
                # if fl_px<fr_px:
                #     for i in range(N):
                #         all_px.append(int(fl_px+(i+1)/(N+1)*(fr_px-fl_px)))
                # else:
                #     for i in range(N):
                #         all_px.append(int(fl_px-(i+1)/(N+1)*(fl_px-fr_px)))

                # if fl_py<fr_py:
                #     for i in range(N):
                #         all_py.append(int(fl_py+(i+1)/(N+1)*(fr_py-fl_py)))
                # else:
                #     for i in range(N):
                #         all_py.append(int(fl_py-(i+1)/(N+1)*(fl_py-fr_py)))
                # all_px.append(fr_px);all_py.append(fr_py)

                # lane_count={}
                # for px,py in zip(all_px,all_py):
                #     this_lane=self.config_matrix[px,py,Config_Matrix_Index.LANE_NO]
                #     if this_lane not in lane_count:
                #         lane_count[this_lane]=0
                #     lane_count[this_lane]+=1
                # occupy=max(lane_count.values())/sum(lane_count.values())
                # if occupy<N/(N+2):
                #     self.cur_cross_line.append(id)
                #     continue

                # all_px,all_py=[],[]
                # all_px.append(bl_px);all_py.append(bl_py)
                # if bl_px<br_px:
                #     for i in range(N):
                #         all_px.append(int(bl_px+(i+1)/(N+1)*(br_px-bl_px)))
                # else:
                #     for i in range(N):
                #         all_px.append(int(bl_px-(i+1)/(N+1)*(bl_px-br_px)))

                # if bl_py<br_py:
                #     for i in range(N):
                #         all_py.append(int(bl_py+(i+1)/(N+1)*(br_py-bl_py)))
                # else:
                #     for i in range(N):
                #         all_py.append(int(bl_py-(i+1)/(N+1)*(bl_py-br_py)))
                # all_px.append(br_px);all_py.append(br_py)

                # lane_count={}
                # for px,py in zip(all_px,all_py):
                #     this_lane=self.config_matrix[px,py,Config_Matrix_Index.LANE_NO]
                #     if this_lane not in lane_count:
                #         lane_count[this_lane]=0
                #     lane_count[this_lane]+=1
                # occupy=max(lane_count.values())/sum(lane_count.values())

                # if occupy<N/(N+2):
                #     self.cur_cross_line.append(id)

                if self.config_matrix[fl_px, fl_py, Config_Matrix_Index.LANE_NO] == -1 or \
                        self.config_matrix[fr_px, fr_py, Config_Matrix_Index.LANE_NO] == -1 or \
                        self.config_matrix[br_px, br_py, Config_Matrix_Index.LANE_NO] == -1 or \
                        self.config_matrix[bl_px, bl_py, Config_Matrix_Index.LANE_NO] == -1:
                    continue  # 如果有在非车道区的不判断

                if not (self.config_matrix[fl_px, fl_py, Config_Matrix_Index.LANE_NO] == self.config_matrix[
                    fr_px, fr_py, Config_Matrix_Index.LANE_NO] and
                        self.config_matrix[fl_px, fl_py, Config_Matrix_Index.LANE_NO] == self.config_matrix[
                            br_px, br_py, Config_Matrix_Index.LANE_NO] and
                        self.config_matrix[fl_px, fl_py, Config_Matrix_Index.LANE_NO] == self.config_matrix[
                            bl_px, bl_py, Config_Matrix_Index.LANE_NO]):
                    coincidence = self.id_dist_list[id][-1][3]
                    self.update_now_and_history(id, coincidence, self.id_continue_cross_line_info,
                                                self.history_cross_line,
                                                all_unhit_id_continue_cross_line)

        for id in all_unhit_id_continue_cross_line:
            self.id_continue_cross_line_info.pop(id)

    def _judge_cross_line(self):
        """判断压线"""
        self._get_id_continue_cross_line_info()
        for id, info in self.id_continue_cross_line_info.items():
            if info.hit_count < 10: continue
            self.cur_cross_line.append(id)
            # info.hit_count=1

    def _get_id_continue_stop_info(self):
        """获取连续停车帧数、时间"""
        all_unhit_id_continue_stop = set(self.id_continue_stop_info.keys())
        for id, info in self.cur_id_info.items():
            if id in self.history_speed_info.keys():
                car_v = self.history_speed_info[id].moving_average_speed[-1]
                if car_v >= 1:
                    continue
                # if info.coincidence<0.6:
                #     continue
            coincidence = self.id_dist_list[id][-1][3] * info.coincidence
            self.update_now_and_history(id, coincidence, self.id_continue_stop_info,
                                        self.history_illegal_stop,
                                        all_unhit_id_continue_stop)
        for id in all_unhit_id_continue_stop:  # 删除历史记录中在当前帧未出现的id对应记录
            self.id_continue_stop_info.pop(id)

    def _judge_illegal_stop(self):
        """判断违停"""
        self._get_id_continue_stop_info()

        # 停车时间大于一个红绿灯周期判断违停
        for id, info in self.id_continue_stop_info.items():
            if self.cur_id_info[id].class_id not in Detect_Class_Type.motor_type:  # 只对机动车
                continue
            volum_car = self.cur_id_info[id].w * self.cur_id_info[id].l * self.cur_id_info[id].h
            if volum_car < 3:
                continue
            px, py = self.cur_id_info[id].center_px, self.cur_id_info[id].center_py
            if self.config_matrix[px, py, Config_Matrix_Index.NO_PARK] != 1:
                continue
            if id in self.cur_id_lane:
                lane = self.cur_id_lane[id]
                this_id_info = self.cur_id_info[id]
                pc_id = this_id_info.pc_id
                if self.lane_local_static_congestion_state[lane][
                    pc_id].value == Congestion_Level.seriously_congested.value:
                    continue  # 如果当前车道为重度拥堵，不判断为违停

            # 禁止停车区
            if info.end_time - info.start_time > 1:  # 机动车停在非机动车道上,超过1s
                if id in self.cur_id_lane:  # 当前帧id对应的车道,常规车道和左转辅助道
                    if self.lane_info[self.cur_id_lane[id]].is_non_motorway == Motorway_Type.NON_MOTORWAY:
                        self.cur_illegal_stop.append(id)
            if info.end_time - info.start_time > 90:  # 停止时间超过一个红绿灯周期
                if id not in self.cur_illegal_stop:
                    self.cur_illegal_stop.append(id)

    def _save_id_info(self):
        """保存当前帧的id信息，用于下一帧的雷达bmp像素速度计算
        """
        self.last_id_info = self.cur_id_info

    ### linxiao0514 ###
    def _save_history_id_speed(self):

        for id, curinfo in self.cur_id_info.items():
            if id not in self.history_speed_info:
                self.history_speed_info[id] = History_Info()

            self.history_speed_info[id].t.append(self.cur_id_info[id].cur_id_time)
            self.history_speed_info[id].x.append(self.cur_id_info[id].x)
            self.history_speed_info[id].y.append(self.cur_id_info[id].y)
            self.history_speed_info[id].v.append(self.cur_id_info[id].v)
            # self.history_speed_info[id].cal_speed.append(self.cur_id_info[id].v)
            if len(self.history_speed_info[id].t) == 1:
                self.history_speed_info[id].cal_speed.append(self.cur_id_info[id].v)
                self.history_speed_info[id].moving_average_speed.append(self.cur_id_info[id].v)
                self.history_speed_info[id].acceleration.append(0)  # frame0 acceleration is 0
            elif len(self.history_speed_info[id].t) > 1 and (
                    self.history_speed_info[id].t[-1] - self.history_speed_info[id].t[0] != 0):
                d_range = round(((self.history_speed_info[id].x[-1] - self.history_speed_info[id].x[0]) ** 2 + (
                        self.history_speed_info[id].y[-1] - self.history_speed_info[id].y[0]) ** 2) ** 0.5, 3)
                self.history_speed_info[id].cal_speed.append(
                    round(d_range / (self.history_speed_info[id].t[-1] - self.history_speed_info[id].t[0]), 2))
                moving_average_speed = round(
                    sum(self.history_speed_info[id].cal_speed) / len(self.history_speed_info[id].cal_speed), 2)
                self.history_speed_info[id].moving_average_speed.append(moving_average_speed)

                if len(self.history_speed_info[id].moving_average_speed) > 10:
                    a = round((self.history_speed_info[id].moving_average_speed[-1] -
                               self.history_speed_info[id].moving_average_speed[
                                   len(self.history_speed_info[id].moving_average_speed) - 10]) / (
                                      self.history_speed_info[id].t[-1] - self.history_speed_info[id].t[0]), 2)
                else:
                    a = round((self.history_speed_info[id].moving_average_speed[-1] -
                               self.history_speed_info[id].moving_average_speed[0]) / (
                                      self.history_speed_info[id].t[-1] - self.history_speed_info[id].t[0]), 2)
                self.history_speed_info[id].acceleration.append(a)  # acceleration
            else:
                # print("ERROR:", id)
                pass

            if len(self.history_speed_info[id].t) > 9:
                self.history_speed_info[id].t = self.history_speed_info[id].t[-9:]
                self.history_speed_info[id].x = self.history_speed_info[id].x[-9:]
                self.history_speed_info[id].y = self.history_speed_info[id].y[-9:]
                self.history_speed_info[id].v = self.history_speed_info[id].v[-9:]
                self.history_speed_info[id].cal_speed = self.history_speed_info[id].cal_speed[-9:]
                self.history_speed_info[id].acceleration = self.history_speed_info[id].acceleration[-9:]
            if len(self.history_speed_info[id].moving_average_speed) > 50:
                self.history_speed_info[id].moving_average_speed = self.history_speed_info[id].moving_average_speed[
                                                                   -50:]
        ## delete ##
        currentframe_time = self.cur_id_info[id].cur_id_time
        for his_id in list(self.history_speed_info.keys()):
            if abs(currentframe_time - self.history_speed_info[his_id].t[
                -1]) > 3:  # 3 second do not update, then kill the id
                del self.history_speed_info[his_id]
        # history_speed_info_copy = copy.deepcopy(self.history_speed_info)
        # delete_target = []
        # for his_id, his_info in self.history_speed_info.items():
        #     if abs(currentframe_time - self.history_speed_info[his_id].t[
        #         -1]) > 3:  # 3 second do not update, then kill the id
        #         delete_target.append(his_id)
        #         del self.history_speed_info[his_id]
        # # print('len(self.history_speed_info)-------------------->',len(self.history_speed_info))

    def get_pre_info(self, input_info):
        """获取计算统计量和异常事件检测的预备信息"""
        t_now = time.time()
        # self._own_frame_id+=1
        self._init_cur_info()
        self.use_time_traffic_fd.write(f'_init_cur_info:{(time.time() - t_now) * 1000}ms\n');
        t_now = time.time()
        self._get_cur_id_info(input_info)
        self.use_time_traffic_fd.write(f'_get_cur_id_info:{(time.time() - t_now) * 1000}ms\n');
        t_now = time.time()
        self._get_current_lane_id()
        self.use_time_traffic_fd.write(f'_get_current_lane_id:{(time.time() - t_now) * 1000}ms\n');
        t_now = time.time()
        self._get_id_count()
        self.use_time_traffic_fd.write(f'_get_id_count:{(time.time() - t_now) * 1000}ms\n');
        t_now = time.time()
        start_time = time.time()
        self._save_history_id_speed()  ###linxiao 0517###
        # print('-----------------------------time:', (time.time() - start_time)*1000)

    def get_instantaneous_info(self):
        """计算瞬时量"""
        t_now = time.time()
        self._get_sorted_car_info_by_dis_to_out()
        self.use_time_traffic_fd.write(f'_get_sorted_car_info_by_dis_to_out:{(time.time() - t_now) * 1000}ms\n');
        t_now = time.time()
        self._get_space_headway()
        self.use_time_traffic_fd.write(f'_get_space_headway:{(time.time() - t_now) * 1000}ms\n');
        t_now = time.time()
        # print('_get_space_headway',time.time()-t_now,file=self.use_time_traffic_fd,flush=True);t_now=time.time()
        # self._get_space_tailway()
        # print('_get_space_tailway',time.time()-t_now,file=self.use_time_traffic_fd,flush=True);t_now=time.time()
        # self._get_queue_length_to_out()
        # print('_get_queue_length_to_out',time.time()-t_now,file=self.use_time_traffic_fd,flush=True);t_now=time.time()
        self._get_queue_length_by_space_headway()
        self.use_time_traffic_fd.write(f'_get_queue_length_by_space_headway:{(time.time() - t_now) * 1000}ms\n');
        t_now = time.time()
        # print('_get_queue_length_by_space_headway',time.time()-t_now,file=self.use_time_traffic_fd,flush=True);t_now=time.time()
        # self._get_queue_length_by_first_to_last() #以第一辆车到最后一辆车的直线距离作为排队长度
        # #print('_get_queue_length',time.time()-t_now,file=self.use_time_traffic_fd,flush=True);t_now=time.time()
        # self._get_time_headway_instantaneous() #瞬时车头时距   车头间距/后车速度
        # #print('_get_time_headway_instantaneous',time.time()-t_now,file=self.use_time_traffic_fd,flush=True);t_now=time.time()
        # self._get_time_tailway_instantaneous() #瞬时车尾时距   车尾间距/后车速度
        # #print('_get_time_tailway_instantaneous',time.time()-t_now,file=self.use_time_traffic_fd,flush=True);t_now=time.time()
        self._get_lane_occupancy()
        self.use_time_traffic_fd.write(f'_get_lane_occupancy:{(time.time() - t_now) * 1000}ms\n');
        t_now = time.time()
        # print('_get_lane_occupancy',time.time()-t_now,file=self.use_time_traffic_fd,flush=True);t_now=time.time()
        self._get_cur_lane_classes_num()
        self.use_time_traffic_fd.write(f'_get_cur_lane_classes_num:{(time.time() - t_now) * 1000}ms\n');
        t_now = time.time()
        # print('_get_cur_lane_classes_num',time.time()-t_now,file=self.use_time_traffic_fd,flush=True);t_now=time.time()
        self._get_cur_lane_mean_v()
        self.use_time_traffic_fd.write(f'_get_cur_lane_mean_v:{(time.time() - t_now) * 1000}ms\n');
        t_now = time.time()
        # print('_get_cur_lane_mean_v',time.time()-t_now,file=self.use_time_traffic_fd,flush=True);t_now=time.time()

    def get_statistics_info(self, statistics_flag):
        """计算统计量
        statistics_flag:是否进行统计
        """
        t_now = time.time()
        self._get_history_lane_id_info_of_section()
        self.use_time_traffic_fd.write(f'_get_history_lane_id_info_of_section:{(time.time() - t_now) * 1000}ms\n');
        t_now = time.time()
        # print('_get_history_lane_id_info_of_section',time.time()-t_now,file=self.use_time_traffic_fd,flush=True);t_now=time.time()
        self._get_time_headway_section(statistics_flag)
        self.use_time_traffic_fd.write(f'_get_time_headway_section:{(time.time() - t_now) * 1000}ms\n');
        t_now = time.time()
        # print('_get_time_headway_section',time.time()-t_now,file=self.use_time_traffic_fd,flush=True);t_now=time.time()
        self._get_follow_car_percent(statistics_flag)
        self.use_time_traffic_fd.write(f'_get_follow_car_percent:{(time.time() - t_now) * 1000}ms\n');
        t_now = time.time()
        # print('_get_follow_car_percent',time.time()-t_now,file=self.use_time_traffic_fd,flush=True);t_now=time.time()
        self._get_time_occupancy_by_id_time(statistics_flag)
        self.use_time_traffic_fd.write(f'_get_time_occupancy_by_id_time:{(time.time() - t_now) * 1000}ms\n');
        t_now = time.time()
        # print('_get_time_occupancy_by_id_time',time.time()-t_now,file=self.use_time_traffic_fd,flush=True);t_now=time.time()
        self._get_lane_flow(statistics_flag)
        self.use_time_traffic_fd.write(f'_get_lane_flow:{(time.time() - t_now) * 1000}ms\n');
        t_now = time.time()
        # print('_get_lane_flow',time.time()-t_now,file=self.use_time_traffic_fd,flush=True);t_now=time.time()
        self._get_lane_mean_v_info_by_section(statistics_flag)  # 断面的平均速度
        self.use_time_traffic_fd.write(f'_get_lane_mean_v_info_by_section:{(time.time() - t_now) * 1000}ms\n');
        t_now = time.time()
        # print('_get_lane_mean_v_info_by_section',time.time()-t_now,file=self.use_time_traffic_fd,flush=True);t_now=time.time()
        self._get_lane_mean_v_info_by_all_v(statistics_flag)  # 所有车辆在每一时刻的平均速度
        self.use_time_traffic_fd.write(f'_get_lane_mean_v_info_by_all_v:{(time.time() - t_now) * 1000}ms\n');
        t_now = time.time()
        # print('_get_lane_mean_v_info_by_all_v',time.time()-t_now,file=self.use_time_traffic_fd,flush=True);t_now=time.time()
        self._get_local_static_congestion_state(statistics_flag)
        self.use_time_traffic_fd.write(f'_get_local_static_congestion_state:{(time.time() - t_now) * 1000}ms\n');
        t_now = time.time()
        # print('_get_local_static_congestion_state',time.time()-t_now,file=self.use_time_traffic_fd,flush=True);t_now=time.time()
        self._get_lane_mean_space_headway_info(statistics_flag)
        self.use_time_traffic_fd.write(f'_get_lane_mean_space_headway_info:{(time.time() - t_now) * 1000}ms\n');
        t_now = time.time()
        # print('_get_lane_mean_space_headway_info',time.time()-t_now,file=self.use_time_traffic_fd,flush=True);t_now=time.time()
        self._clear_history_lane_id_info_of_section(statistics_flag)

    def _init_total_info(self):
        """初始化时间段里的统计量"""
        self.total_statistics = {}  # 当前时间段的统计信息:车流量、时间占有率
        self.history_lane_id_time_info = {}  # 当前车道累计的车辆出现时间信息
        self.history_lane_id_info_of_section = {}  # 当前车道统计区累计出现的id
        self.history_lane_id_order_of_section = {}  # 目标进入车道统计区的先后顺序
        self.last_id_info = {}  # 保存上一阵的id对应信息，用于计算bmp下的速度
        self.id_count = {}  # 记录当前id出现次数
        self.id_history_lane = {}  # 记录id对应的历史所属不同车道
        self.id_last_lane = {}  # 记录id对应的之前X时刻车道  {id:[最新x帧车道]}
        self.id_states_of_red_light_for_motor = {}  # 记录id对应的刚进交汇区的位置 和 当前位置 与刚进十字位置的角度和方向
        self.id_states_of_red_light_for_people = {}  # 记录id对应的行人状态  -1:不在人行道  其他:人行道id号
        self.history_speed_info = {}  # linxiao_0514

        for lane in self.lane_info:
            self.history_lane_id_time_info[lane] = {}
            self.history_lane_id_info_of_section[lane] = {}
            self.history_lane_id_order_of_section[lane] = []
            for class_id in Detect_Class_Type.all:  # 为了统计不同类别的车流量
                self.history_lane_id_info_of_section[lane][class_id] = {}

            self.total_statistics[lane] = {}
            self.total_statistics[lane]['lane_flow'] = {}  # 为了统计不同类别的车流量
            for class_id in Detect_Class_Type.all:
                self.total_statistics[lane]['lane_flow'][class_id] = 0

            self.total_statistics[lane]['time_occupancy'] = 0
            self.total_statistics[lane]['follow_car_percent'] = 0

        """初始化平均速度、拥堵评估所需信息"""
        self.lane_mean_v_info_by_all_v = {}  # 每条车道的mean_v_info
        self.lane_mean_v_info_by_section = {}  # 通过断面求每条车道上的mean_v_info
        self.lane_local_static_congestion_state = {}  # 每条车道的  拥堵状态
        self.lane_mean_space_headway_info = {}  # 每条车道的sh信息

        for lane, info in self.lane_info.items():
            self.lane_mean_v_info_by_all_v[lane] = {}
            for class_id in Detect_Class_Type.all:
                self.lane_mean_v_info_by_all_v[lane][class_id] = Mean_V_Info(Mean_V_Info.default_mean_v)
            self.lane_mean_v_info_by_all_v[lane]['all'] = Mean_V_Info(Mean_V_Info.default_mean_v)
            self.lane_mean_v_info_by_all_v[lane]['pc_id'] = {}
            for pc_id in range(1, self.pc_num + 1):
                self.lane_mean_v_info_by_all_v[lane]['pc_id'][pc_id] = Mean_V_Info(Mean_V_Info.default_mean_v)
            self.lane_mean_v_info_by_all_v[lane]['meter'] = {}
            for i in range(self.congestion_matrix.shape[0]):  # 以x作为分界，每车道隔param中的congestion_to_meter作为1格
                self.lane_mean_v_info_by_all_v[lane]['meter'][i] = Mean_V_Info(Mean_V_Info.default_mean_v)

            self.lane_mean_v_info_by_section[lane] = {}
            for class_id in Detect_Class_Type.all:
                self.lane_mean_v_info_by_section[lane][class_id] = Mean_V_Info(Mean_V_Info.default_mean_v)
            self.lane_mean_v_info_by_section[lane]['all'] = Mean_V_Info(Mean_V_Info.default_mean_v)

            self.lane_local_static_congestion_state[lane] = {}
            self.lane_local_static_congestion_state[lane]['all'] = Congestion_Level.unblocked
            for pc_id in range(1, self.pc_num + 1):
                self.lane_local_static_congestion_state[lane][pc_id] = Congestion_Level.unblocked
            self.lane_local_static_congestion_state[lane]['meter'] = {}
            for i in range(self.congestion_matrix.shape[0]):  # 以x作为分界，每车道隔param中的congestion_to_meter作为1格
                self.lane_local_static_congestion_state[lane]['meter'][i] = Congestion_Level.unblocked

            self.lane_mean_space_headway_info[lane] = Mean_Space_Headway_Info(1000)

    def _init_cur_info(self):
        """初始化当前帧信息"""
        self.cur_id_info = {}  # 当前帧id对应的检测信息
        self.cur_lane_id = {}  # 当前帧车道上有哪些id
        self.cur_id_lane = {}  # 当前帧id对应的车道,常规车道和左转辅助道,在其他区域的目标不记录
        self.cur_lane_sorted_id = {}  # 当前帧车道上按排序后的id
        self.cur_lane_id_for_flow = {}  # 当前帧在车流量统计区的id。由于远处点云检测结果id容易跳变,采用一段进行车流量统计
        self.cur_statistics = {}  # 当前帧的瞬时信息:排队长度、空间占有率、车头间距、车头时距
        self.cur_lane_classes_num = {}  # 当前时刻各类别目标的数量
        self.cur_lane_mean_v = {}  # 当前时刻车道内目标的平均速度

        for lane in self.lane_info:
            self.cur_lane_classes_num[lane] = {}
            for class_id in Detect_Class_Type.all:  # 为了统计不同类别的目标数
                self.cur_lane_classes_num[lane][class_id] = 0

            self.cur_lane_id[lane] = []
            self.cur_lane_id_for_flow[lane] = []
            self.cur_statistics[lane] = {}
            self.cur_lane_mean_v[lane] = Mean_V_Info.default_mean_v  # 没车时发送的平均速度值 km/h

    def _change_bmp_input_info(self, input_info):
        """将目标转换到画车道图时的xy上
        """
        # for info in input_info:
        #     pc_id=info[Pc_Info_Index.pc_id.value]
        #     a=info[Pc_Info_Index.a.value]
        #     pc_x=info[Pc_Info_Index.pc_x.value]
        #     pc_y=info[Pc_Info_Index.pc_y.value]

        #     # lon,lat=XYZ_To_BLH(self.param['lon%d_bmp'%pc_id],self.param['lat%d_bmp'%pc_id],pc_x,pc_y,self.param['angle%d_bmp'%pc_id])
        #     # angle=(a/np.pi*180-self.param['angle%d_lastest'%pc_id]+self.param['angle%d_bmp'%pc_id])%360*np.pi/180

        #     # 实际更准确的经纬度
        #     lon,lat=XYZ_To_BLH(self.param['lon%d_lastest'%pc_id],self.param['lat%d_lastest'%pc_id],pc_x,pc_y,self.param['angle%d_lastest'%pc_id])
        #     angle=a#(a/np.pi*180-self.param['angle1_lastest']+self.param['angle1_bmp'])%360*np.pi/180
        #     xy=self.lon_lat_meter_transform.lonlat_to_xy_of_draw(np.array([lon,lat]).reshape(-1,2))#转到画车道图时的像素坐标系

        #     info[Pc_Info_Index.a.value]=angle
        #     info[Pc_Info_Index.x.value]=xy[0,0]
        #     info[Pc_Info_Index.y.value]=xy[0,1]
        #     info[Pc_Info_Index.bottom_front_right_x.value:]=box_np_ops.center_to_corner_box3d(np.array(info[:Pc_Info_Index.z.value+1]).reshape(-1,3),
        #                                     np.array(info[Pc_Info_Index.w.value:Pc_Info_Index.h.value+1]).reshape(-1,3),
        #                                     np.array(angle).reshape(-1),
        #                                     origin=[0.5, 0.5, 0.5],
        #                                     axis=2).reshape(-1, 24)

        # 直接转相差角度
        R_bmp, _ = cv2.Rodrigues(
            np.array([0, 0, (self.param['angle1_bmp'] - self.param['angle1_lastest']) * np.pi / 180]))
        input_info[:, Pc_Info_Index.x.value:Pc_Info_Index.y.value + 2] = R_bmp.dot(
            input_info[:, Pc_Info_Index.x.value:Pc_Info_Index.y.value + 2].T).T
        input_info[:, Pc_Info_Index.bottom_front_right_x.value:Pc_Info_Index.cur_frame_time.value] = R_bmp.dot(
            input_info[:, Pc_Info_Index.bottom_front_right_x.value:Pc_Info_Index.cur_frame_time.value].reshape(-1,
                                                                                                               3).T).T.reshape(
            -1, 24)

    # def _evaluate_position(self,input_info):

    def _get_cur_id_info(self, input_info):
        """获取当前检测信息"""
        if self.param['use_lastest_lonlat'] == 0:  # 画车道的各基站经纬度和当前最新的经纬度不同
            self._change_bmp_input_info(input_info)

        id_index = Pc_Info_Index.all_area_id.value
        # index: 中心点，右前，左前，左后，右后的x,y
        index = [Pc_Info_Index.x.value, Pc_Info_Index.y.value, \
                 Pc_Info_Index.bottom_front_right_x.value, Pc_Info_Index.bottom_front_right_y.value, \
                 Pc_Info_Index.bottom_front_left_x.value, Pc_Info_Index.bottom_front_left_y.value, \
                 Pc_Info_Index.bottom_behind_left_x.value, Pc_Info_Index.bottom_behind_left_y.value, \
                 Pc_Info_Index.bottom_behind_right_x.value, Pc_Info_Index.bottom_behind_right_y.value]
        index_x = [0, 2, 4, 6, 8]
        index_y = [1, 3, 5, 7, 9]  # 中心点，左前，右前，左后，右后的y在index的索引
        bmp_xy = [] if len(input_info) == 0 else self._lidar_to_bmp(input_info[:, index], index_x, index_y)
        # cur_time = time.time()
        for this_info, this_bmp_xy in zip(input_info, bmp_xy):
            # if this_info[id_index] in [6037, 6234, 6468, 6483, 6825, 7206, 7457, 8554, 8630, 9006, 9087, 9248, 9289, 9347, 9402, 9420, 9585, 9639, 9709]:
            # if this_info[9]==6037:
            #     print(f'id:{this_info[9]}')
            if this_info[Pc_Info_Index.c.value] not in Detect_Class_Type.all:
                continue
            c_px, c_py = this_bmp_xy[index[0]], this_bmp_xy[index[1]]
            if c_px >= 15700 or c_py >= 15700:
                continue
            center = np.array([this_info[Pc_Info_Index.x.value], this_info[Pc_Info_Index.y.value], 0])
            # print(center.shape)
            # targetsearch=time.time()
            [_, idx_, distVec] = self.pcd_tree.search_knn_vector_3d(center, 1)
            pos_coincidence = 0
            time_gap_each = 0
            if len(idx_) != 0:
                dis_gap = 0
                dist = np.sqrt(distVec[0])
                # dist=np.sqrt((self.pcd.points[idx_[0]][0]-center[0])**2+(self.pcd.points[idx_[0]][1]-center[1])**2)
                if self.pcd.points[idx_[0]][1] - center[1] > 0:
                    dist = -dist
                if this_info[id_index] not in self.id_dist_list.keys():
                    self.id_dist_list[this_info[id_index]] = [
                        [this_info[Pc_Info_Index.cur_frame_time.value], dist, dis_gap, \
                         pos_coincidence, time_gap_each]]
                else:
                    # use 横移速度
                    time_gap_each = abs(
                        this_info[Pc_Info_Index.cur_frame_time.value] - self.id_dist_list[this_info[id_index]][-1][0])
                    dis_gap = abs(dist - self.id_dist_list[this_info[id_index]][-1][1])
                    dis_gap = 0.1 * dis_gap / time_gap_each
                    if dis_gap <= 0.15:
                        pos_coincidence = 1 - dis_gap * 4 / 3
                    else:
                        # dis_gap=(dis_gap/time_gap_each)*0.1
                        pos_coincidence = math.exp(-2 * (dis_gap - 0.15) ** 2 / 0.25) / (0.5 * math.sqrt(2 * math.pi))

                    self.id_dist_list[this_info[id_index]].append([this_info[Pc_Info_Index.cur_frame_time.value], \
                                                                   dist, dis_gap, pos_coincidence, time_gap_each])
            else:
                continue
            if len(self.id_dist_list[this_info[id_index]]) > 50:
                self.id_dist_list[this_info[id_index]].pop(0)
            this_lane_no = self.config_matrix[c_px, c_py, Config_Matrix_Index.LANE_NO]
            this_lane = self.lane_no_lane_name[this_lane_no] if this_lane_no in self.lane_no_lane_name else -1
            if this_lane != -1:
                self.cur_id_info[this_info[id_index]] = Detect_Info()
                self.cur_id_info[this_info[id_index]].fill_data( \
                    this_info[:Pc_Info_Index.bottom_front_right_x.value], this_bmp_xy,
                    this_info[Pc_Info_Index.cur_frame_time.value])
            # print(f'search one time:{(time.time()-targetsearch)*1000}ms')
        # handle=time.time()
        for idx_ in list(self.id_dist_list.keys()):
            # print(f'id:{idx_}')
            cur_frame_time = input_info[0, 44]
            if cur_frame_time - self.id_dist_list[idx_][-1][0] >= 3:
                del self.id_dist_list[idx_]
        # print(f'copytime:{(time.time()-handle)*1000}ms')
        # print(f'targetnum:{len(self.id_dist_list)},pre_time:{(time.time()-cur_time)*1000}ms')

    def _get_current_lane_id(self):
        """获取各车道上的id信息"""
        for id, info in self.cur_id_info.items():
            c_px, c_py = self.cur_id_info[id].center_px, self.cur_id_info[id].center_py
            this_lane_no = self.config_matrix[c_px, c_py, Config_Matrix_Index.LANE_NO]

            this_lane = self.lane_no_lane_name[this_lane_no] if this_lane_no in self.lane_no_lane_name else -1
            calc_flow = self.config_matrix[c_px, c_py, Config_Matrix_Index.CALC_FLOW]
            len_lane_name = len(str(this_lane))  # 1(默认)  3(非左转)  6(左转)
            if len_lane_name == self._normal_lane_str_len:
                self.cur_lane_id[this_lane].append(id)
                self.cur_id_lane[id] = this_lane
                if calc_flow == 1:  # 统计车流量的区域
                    self.cur_lane_id_for_flow[this_lane].append(id)
            else:  # 非常规车道
                angle = self.cur_id_info[id].angle / np.pi * 180
                for i, turn_left_lane_name in enumerate(self.turn_left_info):
                    if self.turn_left_info[turn_left_lane_name].is_angle_in_this_turn_left_lane(angle):
                        if self.config_matrix[c_px, c_py, i + Config_Matrix_Index.TURN_LEFT_1] == 1:  # 左转辅助道
                            self.cur_lane_id[turn_left_lane_name].append(id)
                            self.cur_id_lane[id] = turn_left_lane_name
                            if calc_flow == 1:  # 统计车流量的区域
                                self.cur_lane_id_for_flow[turn_left_lane_name].append(id)
                        break
                # if id not in self.cur_id_lane:
                #     self.cur_id_lane[id]=0

    def _get_id_count(self):
        """获取id计数器"""
        for id in self.cur_id_info:
            if id not in self.id_count:
                self.id_count[id] = 0
            self.id_count[id] += 1

    def _get_cur_lane_classes_num(self):
        """获取车道上各类别的数量"""
        for lane, sorted_ids in self.cur_lane_sorted_id.items():
            for id in sorted_ids:
                this_class_id = self.cur_id_info[id].class_id
                self.cur_lane_classes_num[lane][this_class_id] += 1

            self.cur_lane_classes_num[lane]['non_motor_type'] = 0
            for class_id in Detect_Class_Type.non_motor_type:
                self.cur_lane_classes_num[lane]['non_motor_type'] += self.cur_lane_classes_num[lane][class_id]

            self.cur_lane_classes_num[lane]['motor_type'] = 0
            for class_id in Detect_Class_Type.motor_type:
                self.cur_lane_classes_num[lane]['motor_type'] += self.cur_lane_classes_num[lane][class_id]

            self.cur_lane_classes_num[lane]['all'] = 0
            for class_id in Detect_Class_Type.all:
                self.cur_lane_classes_num[lane]['all'] += self.cur_lane_classes_num[lane][class_id]

    def _get_sorted_car_info_by_dis_to_out(self):
        """获取每个车道上依照到车道出口距离排序的车的信息"""
        for lane, ids in self.cur_lane_id.items():
            if len(ids) == 0:
                self.cur_lane_sorted_id[lane] = []
                continue
            out_px, out_py = None, None
            if self.lane_info[lane].out_in == 0:
                out_px, out_py = self.lane_info[lane].start_px, self.lane_info[lane].start_py
            else:
                out_px, out_py = self.lane_info[lane].end_px, self.lane_info[lane].end_py

            self.cur_lane_sorted_id[lane] = sorted(ids, key=lambda x: ( \
                        (self.cur_id_info[x].center_px - out_px) ** 2 + \
                        (self.cur_id_info[x].center_py - out_py) ** 2))

    def _get_cur_lane_mean_v(self):
        """计算当前帧每条车道的平均速度"""
        for lane, sorted_ids in self.cur_lane_sorted_id.items():
            if len(sorted_ids) == 0:
                continue

            sum_v = 0
            for id in sorted_ids:
                sum_v += self.cur_id_info[id].v

            self.cur_lane_mean_v[lane] = sum_v * 3.6 / len(sorted_ids)

    def _get_queue_length_by_first_to_last(self):
        """计算当前帧的排队长度,以最远的车到最近的车距离作为排队长度。"""
        for lane, sorted_ids in self.cur_lane_sorted_id.items():
            self.cur_statistics[lane]['queue_length'] = 0
            if len(sorted_ids) == 0:
                continue
            if self.lane_info[lane].out_in == 1:  # 排除车进入的常规车道
                continue
            # 车出的直道和左转辅助道
            first_car_id = sorted_ids[0]
            last_car_id = sorted_ids[-1]  # 距离当前车道起始位置最远的车的id
            first_car_info = self.cur_id_info[first_car_id]
            last_car_info = self.cur_id_info[last_car_id]
            self.cur_statistics[lane]['queue_length'] = np.sqrt(
                (last_car_info.x - first_car_info.x) ** 2 + (last_car_info.y - first_car_info.y) ** 2) + \
                                                        last_car_info.l / 2 + first_car_info.l / 2

    def _get_queue_length_to_out(self):
        """统计车距10m内的最后一辆到出口的距离"""
        for lane, sorted_ids in self.cur_lane_sorted_id.items():
            self.cur_statistics[lane]['queue_length'] = 0
            if len(sorted_ids) <= 1:
                continue
            # 车出的直道和左转辅助道
            sum_length = 0
            for i, id in enumerate(sorted_ids):
                if i == 0:
                    sum_length += self.cur_id_info[id].l
                else:
                    if self.cur_statistics[lane]['space_headway'][i - 1] < 10:
                        sum_length += self.cur_id_info[id].l + self.cur_statistics[lane]['space_headway'][i - 1]
                    else:
                        break
            self.cur_statistics[lane]['queue_length'] = sum_length

    def _get_queue_length_by_space_headway(self):
        """车头间距+所有车长"""
        for lane, sorted_ids in self.cur_lane_sorted_id.items():
            self.cur_statistics[lane]['queue_length'] = 0
            if len(sorted_ids) == 0:
                continue
            if self.lane_info[lane].out_in == 1:  # 排除车进入的常规车道
                continue
            # 车出的直道和左转辅助道
            sum_space_headway = 0 if len(self.cur_statistics[lane]['space_headway']) == 0 \
                else np.sum(self.cur_statistics[lane]['space_headway'])
            sum_car_length = 0
            for id in sorted_ids:
                sum_car_length += self.cur_id_info[id].l
            self.cur_statistics[lane]['queue_length'] = sum_space_headway + sum_car_length

    def _get_space_headway(self):
        """计算当前帧的车头间距"""
        for lane, sorted_ids in self.cur_lane_sorted_id.items():
            self.cur_statistics[lane]['space_headway'] = []
            if len(sorted_ids) >= 2:
                for i in range(len(sorted_ids) - 1):
                    this_car_id, next_car_id = sorted_ids[i], sorted_ids[i + 1]
                    this_car_info, next_car_info = self.cur_id_info[this_car_id], self.cur_id_info[next_car_id]
                    self.cur_statistics[lane]['space_headway'].append(
                        np.sqrt((next_car_info.x - this_car_info.x) ** 2 + (next_car_info.y - this_car_info.y) ** 2) + \
                        this_car_info.l / 2 - next_car_info.l / 2)

    def _get_time_headway_instantaneous(self):
        """计算当前帧的瞬时车头时距"""
        for lane, sorted_ids in self.cur_lane_sorted_id.items():
            self.cur_statistics[lane]['time_headway'] = []
            if len(sorted_ids) == 0:
                continue
            for i in range(len(self.cur_statistics[lane]['space_headway'])):
                self.cur_statistics[lane]['time_headway'].append(self.cur_statistics[lane]['space_headway'][i] / \
                                                                 (self.cur_id_info[sorted_ids[i + 1]].v + 1e-10))  # s

    def _get_space_tailway(self):
        """计算当前帧的车尾间距"""
        for lane, sorted_ids in self.cur_lane_sorted_id.items():
            self.cur_statistics[lane]['space_tailway'] = []
            if len(sorted_ids) >= 2:
                for i in range(len(sorted_ids) - 1):
                    this_car_id, next_car_id = sorted_ids[i], sorted_ids[i + 1]
                    this_car_info, next_car_info = self.cur_id_info[this_car_id], self.cur_id_info[next_car_id]
                    self.cur_statistics[lane]['space_tailway'].append(
                        np.sqrt((next_car_info.x - this_car_info.x) ** 2 + (next_car_info.y - this_car_info.y) ** 2) - \
                        this_car_info.l / 2 + next_car_info.l / 2)

    def _get_time_tailway_instantaneous(self):
        """计算当前帧的瞬时车尾时距"""
        for lane, sorted_ids in self.cur_lane_sorted_id.items():
            self.cur_statistics[lane]['time_tailway'] = []
            if len(sorted_ids) == 0:
                continue
            for i in range(len(self.cur_statistics[lane]['space_tailway'])):
                self.cur_statistics[lane]['time_tailway'].append(self.cur_statistics[lane]['space_tailway'][i] / \
                                                                 (self.cur_id_info[sorted_ids[i + 1]].v + 1e-10))  # s

    def _get_lane_occupancy(self):
        """计算车道占有率"""
        for lane, sorted_ids in self.cur_lane_sorted_id.items():
            if len(sorted_ids) == 0:
                self.cur_statistics[lane]['lane_occupancy'] = 0
                continue

            length = 0
            for id_ in sorted_ids:
                length += self.cur_id_info[id_].l

            self.cur_statistics[lane]['lane_occupancy'] = length / self.lane_info[lane].length

    def _get_time_occupancy_by_id_time(self, flag):
        """计算时间占有率.!!!!当前阶段的存在问题:如果车变道,那么这种方式有问题
        flag:是否进行统计
        """
        # 获取当前车道累计车辆信息。id:起始出现时间,最新的出现时间
        for lane, ids in self.cur_lane_id_for_flow.items():
            if len(ids) == 0:
                continue

            for id_ in ids:
                if id_ not in self.history_lane_id_time_info[lane]:
                    self.history_lane_id_time_info[lane][id_] = [self.cur_time, self.cur_time]
                else:
                    self.history_lane_id_time_info[lane][id_][1] = self.cur_time

        # 统计
        if flag:
            # 统计时间占有率
            for lane, info in self.history_lane_id_time_info.items():  # id:[st,et]
                intervals = []
                if len(info) != 0:
                    intervals = self.merge(lane, info)  # 若当前车道没有车则info为{}

                occupy_time = 0
                for interval in intervals:
                    occupy_time += interval[1] - interval[0]
                self.total_statistics[lane]['time_occupancy'] = 1 if occupy_time > self.param[
                    'statistics_time'] else occupy_time / self.param['statistics_time']

            # 清零
            for lane in self.lane_info:
                self.history_lane_id_time_info[lane] = {}

    def merge(self, lane, intervals):
        """合并区间
        intervals: [(id1,[min1,max1]),(id2,[min2,max2]),...]
        """
        this_intervals = []
        for interval in intervals.values():
            this_intervals.append(deepcopy(interval))  # append地址,由于_get_lane_info和计算result时会修改地址指向的东西,注意!!!
        sorted_lane_info = sorted(this_intervals, key=lambda x: x[0])  # 左边界排序

        result = []
        for item in sorted_lane_info:
            if not result or result[-1][1] < item[0]:  # 放入的第一个数或者没有交集
                result.append(item)
            else:  # 存在交集
                result[-1][1] = max(result[-1][1], item[1])
        return result

    def _get_history_lane_id_info_of_section(self):
        """获取统计时间段里断面不同id对应的所需信息,time,v"""
        for lane, ids in self.cur_lane_id_for_flow.items():
            for id_ in ids:
                if self.id_count[id_] <= 4:  # 在统计时舍弃累计出现低于4次的目标
                    continue
                # self.history_lane_id_info_of_section[lane][self.cur_id_info[id_].class_id][id_]= \
                #     [self.cur_id_info[id_].v,time.time()]
                self.history_lane_id_info_of_section[lane][self.cur_id_info[id_].class_id][id_] = \
                    [self.cur_id_info[id_].v, self.cur_time]
                self.history_lane_id_order_of_section[lane].append([id_, self.cur_id_info[id_].class_id])

    def _clear_history_lane_id_info_of_section(self, flag):
        """统计时间段内的信息清零
        flag:是否进行统计
        """
        if flag:
            for lane, ids in self.cur_lane_id_for_flow.items():
                for class_id in Detect_Class_Type.all:  # 为了统计不同类别的车流量
                    self.history_lane_id_info_of_section[lane][class_id] = {}
                    self.history_lane_id_order_of_section[lane] = []

    def _get_lane_mean_space_headway_info(self, flag):
        """计算统计时间段内的平均车头间距"""
        for lane in self.lane_info:
            self.lane_mean_space_headway_info[lane].update(self.cur_statistics[lane]['space_headway'])

        if flag:
            for lane, info in self.lane_mean_space_headway_info.items():
                # 获取车道平均车距
                self.lane_mean_space_headway_info[lane].get_mean_sh()
                # 状态清零(sum、num)
                self.lane_mean_space_headway_info[lane].clear()

    def _get_local_static_congestion_state(self, flag):
        """局部静态的拥堵评估
        flag:进行评估的标志
        """
        if flag:
            used_lane_mean_v_info = self.lane_mean_v_info_by_all_v  # self.lane_mean_v_info_by_section
            for lane, info in used_lane_mean_v_info.items():
                # 对各车道进行拥堵评估
                self.lane_local_static_congestion_state[lane]['all'] = \
                    self._judge_congestion_level_by_mean_v(self.lane_info[lane],
                                                           used_lane_mean_v_info[lane]['all'].mean_v)
                # 对各车道各基站进行拥堵评估
                for pc_id in range(1, self.pc_num + 1):
                    self.lane_local_static_congestion_state[lane][pc_id] = \
                        self._judge_congestion_level_by_mean_v(self.lane_info[lane],
                                                               used_lane_mean_v_info[lane]['pc_id'][pc_id].mean_v)
                # 对各车道的拥堵热力图进行拥堵评估
                for congestion_map_x in range(self.congestion_matrix.shape[0]):
                    self.lane_local_static_congestion_state[lane]['meter'][congestion_map_x] = \
                        self._judge_congestion_level_by_mean_v(self.lane_info[lane],
                                                               used_lane_mean_v_info[lane]['meter'][
                                                                   congestion_map_x].mean_v)

    def _judge_congestion_level_by_mean_v(self, lane_info, v):
        """根据车道类别、速度判断拥堵程度
        v:km/h
        """
        state = None
        if v >= lane_info.unblocked_min_v:
            state = Congestion_Level.unblocked
        elif lane_info.slightly_congested_min_v <= v < lane_info.slightly_congested_max_v:
            state = Congestion_Level.slightly_congested
        elif lane_info.moderately_congested_min_v <= v < lane_info.moderately_congested_max_v:
            state = Congestion_Level.moderately_congested
        elif lane_info.seriously_congested_min_v <= v < lane_info.seriously_congested_max_v:
            state = Congestion_Level.seriously_congested
        return state

    def _get_lane_mean_v_info_by_section(self, flag):
        """通过断面求平均车速"""
        # 每帧更新平均速度所需信息
        for lane, ids in self.cur_lane_id_for_flow.items():
            for id in ids:
                this_id_info = self.cur_id_info[id]
                self.lane_mean_v_info_by_section[lane][this_id_info.class_id].update(this_id_info.v)
                self.lane_mean_v_info_by_section[lane]['all'].update(this_id_info.v)
        if flag:
            # 在统计平均速度时，一次性更新平均速度所需信息
            # for lane,id_class_ids in self.history_lane_id_order_of_section.items():
            #     for id,class_id in id_class_ids:
            #         this_id_v,this_id_time=self.history_lane_id_info_of_section[lane][class_id][id]
            #         self.lane_mean_v_info_by_section[lane][class_id].update(this_id_v)
            #         self.lane_mean_v_info_by_section[lane]['all'].update(this_id_v)

            for lane, info in self.lane_mean_v_info_by_section.items():
                # 获取各车道各类型的平均速度
                for class_id in Detect_Class_Type.all:
                    self.lane_mean_v_info_by_section[lane][class_id].get_mean_v()
                self.lane_mean_v_info_by_section[lane]['all'].get_mean_v()
                # 状态清零(sum、num)
                for class_id in Detect_Class_Type.all:
                    self.lane_mean_v_info_by_section[lane][class_id].clear()
                self.lane_mean_v_info_by_section[lane]['all'].clear()

    def _get_lane_mean_v_info_by_all_v(self, flag):
        """获取整个统计时间段内检测到目标的所有速度、速度数量、平均速度"""
        for lane, ids in self.cur_lane_id.items():
            for id in ids:
                this_info = self.cur_id_info[id]
                if this_info.class_id not in Detect_Class_Type.motor_type:
                    continue
                if len(ids) <= 3 and this_info.v < 0.5:  # 防止道路空旷时由于误检的静止目标造成干扰
                    continue
                self.lane_mean_v_info_by_all_v[lane][this_info.class_id].update(this_info.v)
                self.lane_mean_v_info_by_all_v[lane]['all'].update(this_info.v)
                self.lane_mean_v_info_by_all_v[lane]['pc_id'][this_info.pc_id].update(this_info.v)

                congestion_map_x = self._bmp_to_congestion_map(np.array([this_info.center_px, this_info.center_py]))[0]
                self.lane_mean_v_info_by_all_v[lane]['meter'][congestion_map_x].update(this_info.v)

        if flag:
            for lane, info in self.lane_mean_v_info_by_all_v.items():
                # 获取各车道各类型的平均速度
                for class_id in Detect_Class_Type.all:
                    self.lane_mean_v_info_by_all_v[lane][class_id].get_mean_v()
                self.lane_mean_v_info_by_all_v[lane]['all'].get_mean_v()
                for pc_id in range(1, self.pc_num + 1):
                    self.lane_mean_v_info_by_all_v[lane]['pc_id'][pc_id].get_mean_v()
                for congestion_map_x in range(self.congestion_matrix.shape[0]):
                    self.lane_mean_v_info_by_all_v[lane]['meter'][congestion_map_x].get_mean_v()

                # 状态清零(sum、num)
                for class_id in Detect_Class_Type.all:
                    self.lane_mean_v_info_by_all_v[lane][class_id].clear()
                self.lane_mean_v_info_by_all_v[lane]['all'].clear()
                for pc_id in range(1, self.pc_num + 1):
                    self.lane_mean_v_info_by_all_v[lane]['pc_id'][pc_id].clear()
                for congestion_map_x in range(self.congestion_matrix.shape[0]):
                    self.lane_mean_v_info_by_all_v[lane]['meter'][congestion_map_x].clear()

    def _get_time_headway_section(self, flag):
        """计算断面法得到的车头时距:两车通过某断面的时间差
        flag:是否进行统计
        """
        if flag:
            # 统计断面法的车头时距
            for lane, id_class_ids in self.history_lane_id_order_of_section.items():
                self.total_statistics[lane]['time_headway_section'] = []
                for i in range(len(id_class_ids) - 1):
                    this_id, this_class_id = id_class_ids[i + 1][0], id_class_ids[i + 1][1]
                    last_id, last_class_id = id_class_ids[i][0], id_class_ids[i][1]
                    this_id_v, this_id_time = self.history_lane_id_info_of_section[lane][this_class_id][this_id]
                    last_id_v, last_id_time = self.history_lane_id_info_of_section[lane][last_class_id][last_id]
                    self.total_statistics[lane]['time_headway_section'].append(this_id_time - last_id_time)

    def _get_follow_car_percent(self, flag):
        """计算跟车百分比
        flag:是否进行统计
        """
        if flag:
            for lane in self.total_statistics:
                self.total_statistics[lane]['follow_car_percent'] = 0
                if len(self.total_statistics[lane]['time_headway_section']) == 0:
                    continue
                cnt = 0
                for delta_time in self.total_statistics[lane]['time_headway_section']:
                    if delta_time < self.param['min_time_headway']:  # min_time_headway:最小的车头时距  从 param  csv中读取
                        cnt += 1
                self.total_statistics[lane]['follow_car_percent'] = cnt / len(
                    self.total_statistics[lane]['time_headway_section'])

    def _get_lane_flow(self, flag):
        """计算车道的车流量，假设时间段都满足，只进行车流量的统计
        flag:是否进行统计
        """
        if flag:
            # 统计车流量
            for lane, item in self.history_lane_id_info_of_section.items():
                for class_id, id_dic in item.items():
                    self.total_statistics[lane]['lane_flow'][class_id] = len(id_dic)

                self.total_statistics[lane]['lane_flow']['motor_type'] = 0
                for class_id in Detect_Class_Type.motor_type:
                    self.total_statistics[lane]['lane_flow']['motor_type'] += self.total_statistics[lane]['lane_flow'][
                        class_id]

    def _get_negative_y_axis_xy(self):
        """获取y轴负方向在bmp上的向量"""
        origin_xy = self._lidar_to_bmp(np.array([0.0, 0.0]).reshape(-1, 2), [0], [1])  # 原点
        negative_xy = self._lidar_to_bmp(np.array([0.0, -1.0]).reshape(-1, 2), [0], [1])  # (0,-1)
        self.negative_y_axis_xy = (negative_xy - origin_xy).reshape(-1)

    def _save_info(self, output_path, info):
        """将变量保存成文件"""
        file_info = open(output_path, 'wb')
        pickle.dump(info, file_info)
        file_info.close()

    def _load_info(self, data_path):
        """将文件变为变量"""
        file_info = open(data_path, 'rb')
        return pickle.load(file_info)

    def _generate_lane_origin_info(self, lane_config_path):
        """生成lane_origin_info(csv里的车道属性)
        """
        self.lane_origin_info = []
        with open(lane_config_path + 'traffic_lane.csv', 'r') as fd:
            dict_reader = csv.DictReader(fd)
            for lane_no, row in enumerate(dict_reader):
                lane_name = int(row['lane_name'])
                lane_direction = int(row['lane_direction'])
                lane_class = int(row['lane_class'])
                is_non_motorway = int(row['is_non_motorway'])
                is_bus_lane = int(row['is_bus_lane'])
                is_emergency_lane = int(row['is_emergency_lane'])
                min_v = float(row['min_v'])
                max_v = float(row['max_v'])
                length = float(row['length'])
                width = float(row['width'])
                start_px = int(row['start_px'])
                start_py = int(row['start_py'])
                end_px = int(row['end_px'])
                end_py = int(row['end_py'])

                self.lane_origin_info.append([lane_name, lane_direction, lane_class, is_non_motorway, \
                                              is_bus_lane, is_emergency_lane, min_v, max_v, length, width, start_px, \
                                              start_py, end_px, end_py])

        self._save_info(lane_config_path + 'lane_origin_info.pkl', self.lane_origin_info)

    def _generate_matrix_data(self, lane_config_path):
        """生成(M,N,O)的配置矩阵,(M,N)为点云bev图分辨率,O为点云bev中每个像素点所具有的O个属性.
        同时生成lane_origin_info(csv里的车道属性)
        """
        """模拟得到的最终配置文件生成软件得到的信息 config_1"""
        self.lane_origin_info = []
        color_lane = {}
        self.lane_no_lane_name = {}  # 行索引:车道名
        with open(lane_config_path + 'traffic_lane.csv', 'r') as fd:
            dict_reader = csv.DictReader(fd)
            for lane_no, row in enumerate(dict_reader):
                lane_name = int(row['lane_name'])
                lane_direction = int(row['lane_direction'])
                lane_class = int(row['lane_class'])
                is_non_motorway = int(row['is_non_motorway'])
                is_bus_lane = int(row['is_bus_lane'])
                is_emergency_lane = int(row['is_emergency_lane'])
                min_v = float(row['min_v'])
                max_v = float(row['max_v'])
                length = float(row['length'])
                width = float(row['width'])
                start_px = int(row['start_px'])
                start_py = int(row['start_py'])
                end_px = int(row['end_px'])
                end_py = int(row['end_py'])
                r = int(row['r'])
                g = int(row['g'])
                b = int(row['b'])

                self.lane_no_lane_name[lane_no] = lane_name

                self.lane_origin_info.append([lane_name, lane_direction, lane_class, is_non_motorway, \
                                              is_bus_lane, is_emergency_lane, min_v, max_v, length, width, start_px, \
                                              start_py, end_px, end_py])
                if not (r == 0 and g == 0 and b == 0):
                    color_lane[(r, g, b)] = (lane_name, lane_no)

        color_area = {}
        with open(lane_config_path + 'traffic_area.csv', 'r') as fd:
            dict_reader = csv.DictReader(fd)
            for row in dict_reader:
                area_name = row['area_name']
                r = int(row['r'])
                g = int(row['g'])
                b = int(row['b'])

                color_area[area_name] = (r, g, b)

        """模拟得到的最终配置文件生成软件得到的信息 config_2"""
        # 所属车道编号,左转辅助道区域 1,左转辅助道区域 2,左转辅助道区域 3,左转辅助道区域 4,
        # 是否统计区,能否禁止压线,能否停车,是否十字中心区,人行道
        img = cv2.imread(lane_config_path + 'lane.png')
        h, w, c = img.shape
        config_matrix = (np.zeros((w, h, 10)) - 1).astype(np.int8)

        # 车道编号
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = np.transpose(img, (1, 0, 2))
        for i in range(img.shape[0]):
            for j in range(img.shape[1]):
                if tuple(img[i, j, :]) in color_lane:
                    config_matrix[i, j, Config_Matrix_Index.LANE_NO] = color_lane[tuple(img[i, j, :])][1]

        # 左转辅助道区域 1
        img = cv2.imread(lane_config_path + 'left_1.png')
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = np.transpose(img, (1, 0, 2))
        r, g, b = color_area['left_1']
        for i in range(img.shape[0]):
            for j in range(img.shape[1]):
                if img[i, j, 0] == r and img[i, j, 1] == g and img[i, j, 2] == b:
                    config_matrix[i, j, Config_Matrix_Index.TURN_LEFT_1] = 1

        # 左转辅助道区域 2
        img = cv2.imread(lane_config_path + 'left_2.png')
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = np.transpose(img, (1, 0, 2))
        r, g, b = color_area['left_2']
        for i in range(img.shape[0]):
            for j in range(img.shape[1]):
                if img[i, j, 0] == r and img[i, j, 1] == g and img[i, j, 2] == b:
                    config_matrix[i, j, Config_Matrix_Index.TURN_LEFT_2] = 1

        # 左转辅助道区域 3
        img = cv2.imread(lane_config_path + 'left_3.png')
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = np.transpose(img, (1, 0, 2))
        r, g, b = color_area['left_3']
        for i in range(img.shape[0]):
            for j in range(img.shape[1]):
                if img[i, j, 0] == r and img[i, j, 1] == g and img[i, j, 2] == b:
                    config_matrix[i, j, Config_Matrix_Index.TURN_LEFT_3] = 1

        # emergency area 4
        img = cv2.imread(lane_config_path + 'emergency.png')
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = np.transpose(img, (1, 0, 2))
        r, g, b = color_area['emergency']
        for i in range(img.shape[0]):
            for j in range(img.shape[1]):
                if img[i, j, 0] == r and img[i, j, 1] == g and img[i, j, 2] == b:
                    config_matrix[i, j, Config_Matrix_Index.EMERGENCY_AREA] = 1

        # 是否统计区
        img = cv2.imread(lane_config_path + 'statistics.png')
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = np.transpose(img, (1, 0, 2))
        r, g, b = color_area['statistics']
        for i in range(img.shape[0]):
            for j in range(img.shape[1]):
                if img[i, j, 0] == r and img[i, j, 1] == g and img[i, j, 2] == b:
                    config_matrix[i, j, Config_Matrix_Index.CALC_FLOW] = 1

        # 是否禁止压线
        img = cv2.imread(lane_config_path + 'cross_line.png')
        # img=cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
        img = np.transpose(img, (1, 0, 2))
        # r,g,b=color_area['cross_line']
        for i in range(img.shape[0]):
            for j in range(img.shape[1]):
                # if img[i,j,0]==r and img[i,j,1]==g and img[i,j,2]==b:
                config_matrix[i, j, Config_Matrix_Index.FORBID_CROSS_LINE] = 1

        # 能否停车
        # 8号门都不能停车,默认值为-1
        # config_matrix[:,:,Config_Matrix_Index.NO_PARK]=-1

        img = cv2.imread(lane_config_path + 'center_area.png')  # 先以存在图层的区域不能停车作为根据
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = np.transpose(img, (1, 0, 2))
        for i in range(img.shape[0]):
            for j in range(img.shape[1]):
                if img[i, j, 0] == 255 and img[i, j, 1] == 0 and img[i, j, 2] == 255:
                    continue
                else:
                    config_matrix[i, j, Config_Matrix_Index.NO_PARK] = 1

        # img=cv2.imread(lane_config_path+'no_park.png')# for simulation
        # img=cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
        # img=np.transpose(img,(1,0,2))
        # r,g,b=color_area['no_park']
        # for i in range(img.shape[0]):
        #     for j in range(img.shape[1]):
        #         if img[i,j,0]==r and img[i,j,1]==g and img[i,j,2]==b:
        #             config_matrix[i,j,Config_Matrix_Index.NO_PARK]=1

        # 十字路口区、路口交汇区
        img = cv2.imread(lane_config_path + 'center_area.png')
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = np.transpose(img, (1, 0, 2))
        r, g, b = color_area['center_area']
        for i in range(img.shape[0]):
            for j in range(img.shape[1]):
                if img[i, j, 0] == r and img[i, j, 1] == g and img[i, j, 2] == b:
                    config_matrix[i, j, Config_Matrix_Index.CENTER_AREA] = 1

        # 人行道
        img = cv2.imread(lane_config_path + 'sidewalk.png')
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = np.transpose(img, (1, 0, 2))
        r_24, g_24, b_24 = color_area['sidewalk_24']
        r_31, g_31, b_31 = color_area['sidewalk_31']
        r_42, g_42, b_42 = color_area['sidewalk_42']
        r_13, g_13, b_13 = color_area['sidewalk_13']
        for i in range(img.shape[0]):
            for j in range(img.shape[1]):
                if img[i, j, 0] == r_24 and img[i, j, 1] == g_24 and img[i, j, 2] == b_24:
                    config_matrix[i, j, Config_Matrix_Index.SIDEWALK] = 24
                elif img[i, j, 0] == r_31 and img[i, j, 1] == g_31 and img[i, j, 2] == b_31:
                    config_matrix[i, j, Config_Matrix_Index.SIDEWALK] = 31
                elif img[i, j, 0] == r_42 and img[i, j, 1] == g_42 and img[i, j, 2] == b_42:
                    config_matrix[i, j, Config_Matrix_Index.SIDEWALK] = 42
                elif img[i, j, 0] == r_13 and img[i, j, 1] == g_13 and img[i, j, 2] == b_13:
                    config_matrix[i, j, Config_Matrix_Index.SIDEWALK] = 13

        self.config_matrix = config_matrix

        """保存模拟得到的配置文件"""
        self._save_info(lane_config_path + 'config_matrix.pkl', self.config_matrix)
        self._save_info(lane_config_path + 'lane_origin_info.pkl', self.lane_origin_info)
        self._save_info(lane_config_path + 'lane_no_lane_name.pkl', self.lane_no_lane_name)

    def _format_lane_info(self, lane_config_path):
        """从配置文件中读取的信息获取更多有用信息
        """
        Congestion_Level_highway_table = np.genfromtxt(lane_config_path + 'Congestion_Level_highway.csv', delimiter=',')
        Congestion_Level_main_branch_table = np.genfromtxt(lane_config_path + 'Congestion_Level_main_branch.csv',
                                                           delimiter=',')

        self.lane_info = {}  # 常规车道和左转辅助道
        self.turn_left_info = {}  # 左转辅助道，降低判断左转辅助道的耗时
        for origin_data in self.lane_origin_info:
            lane_name = origin_data[0]
            str_lane_name = str(lane_name)
            if len(str_lane_name) == self._normal_lane_str_len:
                self.lane_info[lane_name] = Lane_Info()
                self.lane_info[lane_name].fill_data(origin_data, Congestion_Level_highway_table,
                                                    Congestion_Level_main_branch_table)
                self.lane_info[lane_name].fill_plus_data_for_non_converge(origin_data, self.negative_y_axis_xy,
                                                                          self.param['angle1_bmp'] if self.param[
                                                                                                          'use_lastest_lonlat'] == 0 else
                                                                          self.param['angle1_lastest'])
            elif len(str_lane_name) == 2 * self._normal_lane_str_len:  # 左转辅助道
                self.lane_info[lane_name] = Lane_Info()
                self.lane_info[lane_name].fill_data(origin_data, Congestion_Level_highway_table,
                                                    Congestion_Level_main_branch_table)
                start_lane_name, end_lane_name = int(str_lane_name[:self._normal_lane_str_len]), int(
                    str_lane_name[self._normal_lane_str_len:])
                self.lane_info[lane_name].fill_plus_data_for_converge(
                    self.lane_info[start_lane_name], self.lane_info[end_lane_name])

                self.turn_left_info[lane_name] = self.lane_info[lane_name]

    def _lidar_to_bmp(self, lidar_coordinate_xy, x_index, y_index):
        """将激光雷达坐标转换到bmp的像素坐标中."""
        sx = self.param['sx'] * self.param['meter_to_pixel']
        sy = self.param['sy'] * self.param['meter_to_pixel']
        ex = self.param['ex'] * self.param['meter_to_pixel']
        ey = self.param['ey'] * self.param['meter_to_pixel']

        bmp_xy = lidar_coordinate_xy * self.param['meter_to_pixel']  # m->pixel
        bmp_xy[:, y_index] = -bmp_xy[:, y_index]  # 翻转y轴
        # 平移
        bmp_xy[:, x_index] = bmp_xy[:, x_index] - sx - 1
        bmp_xy[:, y_index] = bmp_xy[:, y_index] + sy - 1

        bmp_xy = self.param['bmp_ratio'] * bmp_xy
        # bmp_xy=3/4*bmp_xy#对于fuqi隧道ps导出文件后最大可以保存3/4倍的图片

        bmp_xy = bmp_xy.astype(np.int)
        return bmp_xy

    def _init_accident_lon_lat(self, lane_config_path):
        """初始化交通事故坐标系每个格子下的经纬度
        """
        if os.path.exists(lane_config_path + 'accident_lon_lat.pkl'):
            self.accident_lon_lat = self._load_info(lane_config_path + 'accident_lon_lat.pkl')
        else:
            w, h = self.accident_matrix.shape
            self.accident_lon_lat = np.zeros((w, h, 2))
            xy = np.meshgrid(range(w), range(h))
            x = xy[0].reshape(-1, 1);
            y = xy[1].reshape(-1, 1)
            xy = np.hstack((x, y))
            xy = self._accident_coordinate_to_bmp(xy)
            lon_lat = self.lon_lat_meter_transform.xy_of_draw_to_lonlat(xy)
            self.accident_lon_lat = lon_lat.reshape(h, w, 2).transpose((1, 0, 2))

            self._save_info(lane_config_path + 'accident_lon_lat.pkl', self.accident_lon_lat)

    def _init_spills_lon_lat(self, lane_config_path):
        """初始化抛洒物坐标系每个格子下的经纬度
        """
        if os.path.exists(lane_config_path + 'spills_lon_lat.pkl'):
            self.spills_lon_lat = self._load_info(lane_config_path + 'spills_lon_lat.pkl')
        else:
            w, h = self.spills_matrix.shape
            self.spills_lon_lat = np.zeros((w, h, 2))
            xy = np.meshgrid(range(w), range(h))
            x = xy[0].reshape(-1, 1);
            y = xy[1].reshape(-1, 1)
            xy = np.hstack((x, y))
            xy = self._spills_coordinate_to_bmp(xy)
            lon_lat = self.lon_lat_meter_transform.xy_of_draw_to_lonlat(xy)
            self.spills_lon_lat = lon_lat.reshape(h, w, 2).transpose((1, 0, 2))

            self._save_info(lane_config_path + 'spills_lon_lat.pkl', self.spills_lon_lat)

    def _bmp_to_congestion_map(self, bmp_xy):
        """像素坐标系转换到拥堵热力图坐标系"""
        bmp_xy = np.array(bmp_xy)
        congestion_xy = bmp_xy / self.param['bmp_ratio'] / self.param['meter_to_pixel'] / self.param['congestion_meter']
        congestion_xy = congestion_xy.astype(np.int)
        return congestion_xy

    def _congestion_map_to_bmp(self, congestion_xy):
        """拥堵热力图坐标系转换到像素坐标系"""
        congestion_xy = np.array(congestion_xy)
        bmp_xy = congestion_xy * self.param['bmp_ratio'] * self.param['meter_to_pixel'] * self.param['congestion_meter']
        bmp_xy = bmp_xy.astype(np.int)
        return bmp_xy

    def _bmp_to_spills_coordinate(self, bmp_xy):
        """像素坐标系转换到抛洒物坐标系"""
        bmp_xy = np.array(bmp_xy)
        spills_xy = bmp_xy / self.param['bmp_ratio'] / self.param['meter_to_pixel'] / self.param['spills_meter']
        spills_xy = spills_xy.astype(np.int)
        return spills_xy

    def _spills_coordinate_to_bmp(self, spills_xy):
        """抛洒物坐标系转换到像素坐标系"""
        spills_xy = np.array(spills_xy)
        bmp_xy = spills_xy * self.param['bmp_ratio'] * self.param['meter_to_pixel'] * self.param['spills_meter']
        bmp_xy = bmp_xy.astype(np.int)
        return bmp_xy

    def _bmp_to_accident_coordinate(self, bmp_xy):
        """像素坐标系转换到交通事故坐标系"""
        bmp_xy = np.array(bmp_xy)
        accident_xy = bmp_xy / self.param['bmp_ratio'] / self.param['meter_to_pixel'] / self.param['accident_meter']
        accident_xy = accident_xy.astype(np.int)
        return accident_xy

    def _accident_coordinate_to_bmp(self, accident_xy):
        """交通事故坐标系转换到像素坐标系"""
        accident_xy = np.array(accident_xy)
        bmp_xy = accident_xy * self.param['bmp_ratio'] * self.param['meter_to_pixel'] * self.param['accident_meter']
        bmp_xy = bmp_xy.astype(np.int)
        return bmp_xy

    def _get_traffic_param(self, lane_config_path):
        """获取配置参数"""
        self.param = {}
        with open(lane_config_path + 'traffic_param.csv', 'r', encoding="utf-8") as fd:
            dict_reader = csv.DictReader(fd)
            for row in dict_reader:
                self.param[row['key']] = float(row['value'])

    def _init_judge_camera(self):
        """初始化判断相机信息"""
        self.judge_camera_lists = []
        self.judge_camera_lists.append(Judge_Camera(1110, 971, 1025, 1176))
        self.judge_camera_lists.append(Judge_Camera(868, 1127, 1094, 1069))
        self.judge_camera_lists.append(Judge_Camera(1000, 900, 1060, 1019))

    def get_camera(self, output_ids):
        """判断异常事件目标所在相机"""
        self.cur_id_cam = {}
        for id in output_ids:
            for i, judge_camera in enumerate(self.judge_camera_lists):
                if judge_camera.is_in_this_camera(self.cur_id_info[id].center_px, self.cur_id_info[id].center_py):
                    if id not in self.cur_id_cam:
                        self.cur_id_cam[id] = []
                    self.cur_id_cam[id].append(i)
            if id not in self.cur_id_cam:
                self.cur_id_cam[id] = -1

    def get_event_output(self):
        """向平台输出的异常事件信息"""
        all = []
        all.extend(self.cur_illegal_stop)
        all.extend(self.cur_retrograde_for_non_motor)
        all.extend(self.cur_retrograde_for_motor)
        all.extend(self.cur_occupy_dedicated_lane)
        all.extend(self.cur_people_occupy_motor_lane)
        all.extend(self.cur_occupy_emergency_lane)
        all.extend(self.cur_speeding_for_motor)
        all.extend(self.cur_speeding_for_non_motor)
        all.extend(self.cur_stroll)
        all.extend(self.cur_cross_line)
        all.extend(self.cur_cross_lane_for_non_motor)
        all.extend(self.cur_cross_lane_for_people)
        all.extend(self.cur_cross_lane_for_motor)
        all.extend(self.cur_run_the_red_light_for_motor)
        all.extend(self.cur_run_the_red_light_for_people)
        all.extend(self.cur_occupy_bus_lane)
        all.extend(self.cur_change_lanes_illegal)
        self.get_camera(set(all))

        self.all_event_output_for_B2 = []
        max_distance = 300

        def one_event_info_for_B2(cur_event, id_continue_info, abnormal_class_index):
            one_event_info = []
            for id in cur_event:
                if self.cur_id_cam[id] == -1:  # 舍弃在相机视场外的
                    continue
                if np.sqrt((self.cur_id_info[id].center_px - 1000) ** 2 + (
                        self.cur_id_info[id].center_py - 1000) ** 2) > max_distance:
                    continue  # 舍弃超过范围
                one_event_info.append([id,
                                       abnormal_class_index,
                                       len(self.cur_id_cam[id]),
                                       self.cur_id_cam[id],
                                       id_continue_info[id].format_start_time,
                                       id_continue_info[id].end_time - id_continue_info[id].start_time,
                                       0
                                       ])
            if len(one_event_info) != 0:
                self.all_event_output_for_B2.extend(one_event_info)

        one_event_info_for_B2(self.cur_illegal_stop, self.id_continue_stop_info, Abnormal_Class_Index.illegal_stop)
        one_event_info_for_B2(self.cur_retrograde_for_non_motor, self.id_continue_retrograde_info_for_non_motor,
                              Abnormal_Class_Index.retrograde_for_non_motor)
        one_event_info_for_B2(self.cur_retrograde_for_motor, self.id_continue_retrograde_info_for_motor,
                              Abnormal_Class_Index.retrograde_for_motor)
        one_event_info_for_B2(self.cur_occupy_dedicated_lane, self.id_continue_occupy_dedicated_lane_info,
                              Abnormal_Class_Index.non_motor_in_tunnel)
        one_event_info_for_B2(self.cur_people_occupy_motor_lane, self.id_continue_people_occupy_motor_lane_info,
                              Abnormal_Class_Index.person_in_tunnel)
        one_event_info_for_B2(self.cur_occupy_emergency_lane, self.id_continue_occupy_emergency_lane_info,
                              Abnormal_Class_Index.occupy_emergency_lane)
        one_event_info_for_B2(self.cur_speeding_for_motor, self.id_continue_speeding_info_for_motor,
                              Abnormal_Class_Index.speeding_for_motor)
        one_event_info_for_B2(self.cur_speeding_for_non_motor, self.id_continue_speeding_info_for_non_motor,
                              Abnormal_Class_Index.speeding_for_non_motor)
        one_event_info_for_B2(self.cur_stroll, self.id_continue_stroll_info, Abnormal_Class_Index.stroll)
        one_event_info_for_B2(self.cur_cross_line, self.id_continue_cross_line_info, Abnormal_Class_Index.cross_line)
        one_event_info_for_B2(self.cur_cross_lane_for_non_motor, self.id_continue_cross_lane_info_for_non_motor,
                              Abnormal_Class_Index.cross_lane_for_non_motor)
        one_event_info_for_B2(self.cur_cross_lane_for_people, self.id_continue_cross_lane_info_for_people,
                              Abnormal_Class_Index.cross_lane_for_people)
        one_event_info_for_B2(self.cur_cross_lane_for_motor, self.id_continue_cross_lane_info_for_motor,
                              Abnormal_Class_Index.cross_lane_for_motor)
        one_event_info_for_B2(self.cur_run_the_red_light_for_motor, self.id_continue_run_the_red_light_info_for_motor,
                              Abnormal_Class_Index.run_the_red_light_for_motor)
        one_event_info_for_B2(self.cur_run_the_red_light_for_people, self.id_continue_run_the_red_light_info_for_people,
                              Abnormal_Class_Index.run_the_red_light_for_people)
        one_event_info_for_B2(self.cur_occupy_bus_lane, self.id_continue_occupy_bus_lane_info,
                              Abnormal_Class_Index.occupy_bus_lane)
        one_event_info_for_B2(self.cur_change_lanes_illegal, self.id_continue_change_lanes_info,
                              Abnormal_Class_Index.change_lanes)

    def get_camera_by_id_cam(self, id_cam):
        """判断异常事件目标所在相机

        @param:
            id_cam:融合结果输出的  {idx1:{id:True or False,...},idx2:{}}
        @get:
            cur_id_cam:{id:[idx1,idx2]} #idx:(pc_id,all_area_id)
        """
        self.cur_id_cam = {}
        for idx, item in id_cam.items():
            for id, state in item.items():
                if state:
                    if id not in self.cur_id_cam:
                        self.cur_id_cam[id] = []
                    self.cur_id_cam[id].append(idx)

    def get_event_output_by_id_cam(self, id_cam):
        """不用角度判断目标是否在相机视场，而用映射的结果"""
        self.get_camera_by_id_cam(id_cam)

        self.all_event_output_for_B2 = []

        def one_event_info_for_B2_by_id_cam(cur_event, id_continue_info, abnormal_class_index):
            one_event_info = []
            for id in cur_event:
                if id not in self.cur_id_cam:  # 舍弃在相机视场外的
                    continue
                this_id_continue_info = id_continue_info[id]
                one_event_info.append([int(id),
                                       abnormal_class_index,
                                       len(self.cur_id_cam[id]),
                                       self.cur_id_cam[id],
                                       this_id_continue_info.format_start_time,
                                       this_id_continue_info.end_time - this_id_continue_info.start_time,
                                       0
                                       ])
            if len(one_event_info) != 0:
                self.all_event_output_for_B2.extend(one_event_info)

        one_event_info_for_B2_by_id_cam(self.cur_illegal_stop, self.id_continue_stop_info,
                                        Abnormal_Class_Index.illegal_stop)
        one_event_info_for_B2_by_id_cam(self.cur_retrograde_for_non_motor,
                                        self.id_continue_retrograde_info_for_non_motor,
                                        Abnormal_Class_Index.retrograde_for_non_motor)
        one_event_info_for_B2_by_id_cam(self.cur_retrograde_for_motor, self.id_continue_retrograde_info_for_motor,
                                        Abnormal_Class_Index.retrograde_for_motor)
        one_event_info_for_B2_by_id_cam(self.cur_occupy_dedicated_lane, self.id_continue_occupy_dedicated_lane_info,
                                        Abnormal_Class_Index.non_motor_in_tunnel)
        one_event_info_for_B2_by_id_cam(self.cur_people_occupy_motor_lane,
                                        self.id_continue_people_occupy_motor_lane_info,
                                        Abnormal_Class_Index.person_in_tunnel)
        one_event_info_for_B2_by_id_cam(self.cur_occupy_emergency_lane, self.id_continue_occupy_emergency_lane_info,
                                        Abnormal_Class_Index.occupy_emergency_lane)
        one_event_info_for_B2_by_id_cam(self.cur_speeding_for_motor, self.id_continue_speeding_info_for_motor,
                                        Abnormal_Class_Index.speeding_for_motor)
        one_event_info_for_B2_by_id_cam(self.cur_speeding_for_non_motor, self.id_continue_speeding_info_for_non_motor,
                                        Abnormal_Class_Index.speeding_for_non_motor)
        one_event_info_for_B2_by_id_cam(self.cur_stroll, self.id_continue_stroll_info, Abnormal_Class_Index.stroll)
        one_event_info_for_B2_by_id_cam(self.cur_cross_line, self.id_continue_cross_line_info,
                                        Abnormal_Class_Index.cross_line)
        one_event_info_for_B2_by_id_cam(self.cur_cross_lane_for_non_motor,
                                        self.id_continue_cross_lane_info_for_non_motor,
                                        Abnormal_Class_Index.cross_lane_for_non_motor)
        one_event_info_for_B2_by_id_cam(self.cur_cross_lane_for_people, self.id_continue_cross_lane_info_for_people,
                                        Abnormal_Class_Index.cross_lane_for_people)
        one_event_info_for_B2_by_id_cam(self.cur_cross_lane_for_motor, self.id_continue_cross_lane_info_for_motor,
                                        Abnormal_Class_Index.cross_lane_for_motor)
        one_event_info_for_B2_by_id_cam(self.cur_run_the_red_light_for_motor,
                                        self.id_continue_run_the_red_light_info_for_motor,
                                        Abnormal_Class_Index.run_the_red_light_for_motor)
        one_event_info_for_B2_by_id_cam(self.cur_run_the_red_light_for_people,
                                        self.id_continue_run_the_red_light_info_for_people,
                                        Abnormal_Class_Index.run_the_red_light_for_people)
        one_event_info_for_B2_by_id_cam(self.cur_occupy_bus_lane, self.id_continue_occupy_bus_lane_info,
                                        Abnormal_Class_Index.occupy_bus_lane)
        one_event_info_for_B2_by_id_cam(self.cur_change_lanes_illegal, self.id_continue_change_lanes_info,
                                        Abnormal_Class_Index.change_lanes)

    def get_event_output_for_B5(self):
        """获取B5帧所需的信息"""
        self.all_event_output_for_B5 = []
        self.has_new_event = False

        def judge_new_event(cur_event, history_infos):
            for id in cur_event:
                this_history_info = history_infos[id][-1]
                event_num = this_history_info.event_no
                # time_his=self.history_events[id]
                # ltajjj = this_history_info.end_time-time_his
                if event_num not in self.history_events.keys():
                    self.has_new_event = True
                    self.history_events[event_num] = this_history_info.end_time
                else:
                    if this_history_info.end_time - self.history_events[event_num] >= 5:
                        self.has_new_event = True
                        self.history_events[event_num] = this_history_info.end_time

            if len(self.history_events) > 100:
                long_time_miss_idxs = sorted(self.history_events.keys())
                self.history_events.pop(long_time_miss_idxs[0])

        def one_event_info_for_B5(cur_event, history_infos, abnormal_class_index):
            B5_event = []
            if self.has_new_event:
                for id in cur_event:
                    this_history_info = history_infos[id][-1]
                    # this_history_info.hit_count=5
                    time_list = str(this_history_info.end_time).split('.')
                    second = int(time_list[0])
                    miscros = int(time_list[1])
                    B5_event.append([this_history_info.event_no,
                                     abnormal_class_index,
                                     self.cur_id_info[id].pc_id,
                                     0 if id not in self.cur_id_lane else self.lane_fix_info[
                                         self.cur_id_lane[id]].lane_output_name,  # 如果为0代表不在车道内
                                     1,  # 事件涉及目标个数
                                     [[id, this_history_info.coincidence]],
                                     this_history_info.longitude,  # 经度
                                     this_history_info.latitude,  # 维度
                                     second,
                                     miscros,
                                     0,
                                     self.cur_id_info[id].cam_id])

            if len(B5_event) != 0:
                self.all_event_output_for_B5.extend(B5_event)

        def one_detect_event_info_for_B5(cur_event, history_infos, abnormal_class_index):
            B5_event = []
            if self.has_new_event:
                for idx in cur_event:
                    this_history_info = history_infos[idx][-1]
                    # this_history_info.hit_count=5
                    time_list = str(this_history_info.end_time).split('.')
                    second = int(time_list[0])
                    miscros = int(time_list[1])
                    this_xy = self._spills_coordinate_to_bmp(np.array([idx[0], idx[1]]))
                    this_lane_no = self.config_matrix[this_xy[0], this_xy[1], Config_Matrix_Index.LANE_NO]
                    this_lane = self.lane_no_lane_name[this_lane_no] if this_lane_no in self.lane_no_lane_name else -1

                    pc_id = 1 + np.argmin(np.linalg.norm(this_xy - np.array(self.pc_bmp_xy).reshape(-1, 2), axis=1),
                                          axis=0)

                    B5_event.append([this_history_info.event_no,
                                     abnormal_class_index,
                                     pc_id,
                                     0 if this_lane not in self.lane_fix_info else self.lane_fix_info[
                                         this_lane].lane_output_name,  # 如果为0代表不在车道
                                     0,  # 事件涉及目标个数
                                     [],
                                     this_history_info.longitude,  # 经度
                                     this_history_info.latitude,  # 维度
                                     second,
                                     miscros,
                                     0,
                                     -1])  # cam_id 待完善
            if len(B5_event) != 0:
                self.all_event_output_for_B5.extend(B5_event)

        judge_new_event(self.cur_illegal_stop, self.history_illegal_stop)
        judge_new_event(self.cur_retrograde_for_non_motor, self.history_retrograde_for_non_motor)
        judge_new_event(self.cur_retrograde_for_motor, self.history_retrograde_for_motor)
        judge_new_event(self.cur_occupy_dedicated_lane, self.history_occupy_dedicated_lane)
        judge_new_event(self.cur_people_occupy_motor_lane, self.history_people_occupy_motor_lane)
        judge_new_event(self.cur_occupy_emergency_lane, self.history_occupy_emergency_lane)
        judge_new_event(self.cur_speeding_for_motor, self.history_speeding_for_motor)
        judge_new_event(self.cur_speeding_for_non_motor, self.history_speeding_for_non_motor)
        judge_new_event(self.cur_stroll, self.history_stroll)
        judge_new_event(self.cur_cross_line, self.history_cross_line)
        judge_new_event(self.cur_cross_lane_for_non_motor, self.history_cross_lane_for_non_motor)
        judge_new_event(self.cur_cross_lane_for_people, self.history_cross_lane_for_people)
        judge_new_event(self.cur_cross_lane_for_motor, self.history_cross_lane_for_motor)
        judge_new_event(self.cur_run_the_red_light_for_motor, self.history_run_the_red_light_for_motor)
        judge_new_event(self.cur_run_the_red_light_for_people, self.history_run_the_red_light_for_people)
        judge_new_event(self.cur_occupy_bus_lane, self.history_occupy_bus_lane)
        judge_new_event(self.cur_change_lanes_illegal, self.history_change_lanes)
        judge_new_event(self.cur_spills, self.history_spills)

        one_event_info_for_B5(self.cur_illegal_stop, self.history_illegal_stop, Abnormal_Class_Index.illegal_stop)
        # last_num=len(self.all_event_output_for_B5)
        # print('illegal_stop_num',last_num,file=self.use_time_traffic_fd,flush=True)

        one_event_info_for_B5(self.cur_retrograde_for_non_motor, self.history_retrograde_for_non_motor,
                              Abnormal_Class_Index.retrograde_for_non_motor)
        # print('retrograde_for_non_motor_num',len(self.all_event_output_for_B5)-last_num,file=self.use_time_traffic_fd,flush=True)
        # last_num=len(self.all_event_output_for_B5)

        one_event_info_for_B5(self.cur_retrograde_for_motor, self.history_retrograde_for_motor,
                              Abnormal_Class_Index.retrograde_for_motor)
        # print('retrograde_for_motor_num',len(self.all_event_output_for_B5)-last_num,file=self.use_time_traffic_fd,flush=True)
        # last_num=len(self.all_event_output_for_B5)

        one_event_info_for_B5(self.cur_occupy_dedicated_lane, self.history_occupy_dedicated_lane,
                              Abnormal_Class_Index.non_motor_in_tunnel)
        # print('occupy_dedicated_lane_num',len(self.all_event_output_for_B5)-last_num,file=self.use_time_traffic_fd,flush=True)
        # last_num=len(self.all_event_output_for_B5)

        one_event_info_for_B5(self.cur_people_occupy_motor_lane, self.history_people_occupy_motor_lane,
                              Abnormal_Class_Index.person_in_tunnel)
        # print('people_occupy_motor_lane_num',len(self.all_event_output_for_B5)-last_num,file=self.use_time_traffic_fd,flush=True)
        # last_num=len(self.all_event_output_for_B5)

        one_event_info_for_B5(self.cur_speeding_for_motor, self.history_speeding_for_motor,
                              Abnormal_Class_Index.speeding_for_motor)
        # print('speeding_for_motor_num',len(self.all_event_output_for_B5)-last_num,file=self.use_time_traffic_fd,flush=True)
        # last_num=len(self.all_event_output_for_B5)

        one_event_info_for_B5(self.cur_speeding_for_non_motor, self.history_speeding_for_non_motor,
                              Abnormal_Class_Index.speeding_for_non_motor)
        # print('speeding_for_non_motor_num',len(self.all_event_output_for_B5)-last_num,file=self.use_time_traffic_fd,flush=True)
        # last_num=len(self.all_event_output_for_B5)

        one_event_info_for_B5(self.cur_stroll, self.history_stroll, Abnormal_Class_Index.stroll)
        # print('stroll_num',len(self.all_event_output_for_B5)-last_num,file=self.use_time_traffic_fd,flush=True)
        # last_num=len(self.all_event_output_for_B5)

        one_event_info_for_B5(self.cur_cross_line, self.history_cross_line, Abnormal_Class_Index.cross_line)
        # print('cross_line_num',len(self.all_event_output_for_B5)-last_num,file=self.use_time_traffic_fd,flush=True)
        # last_num=len(self.all_event_output_for_B5)

        one_event_info_for_B5(self.cur_cross_lane_for_non_motor, self.history_cross_lane_for_non_motor,
                              Abnormal_Class_Index.cross_lane_for_non_motor)
        # print('cross_lane_for_non_motor_num',len(self.all_event_output_for_B5)-last_num,file=self.use_time_traffic_fd,flush=True)
        # last_num=len(self.all_event_output_for_B5)

        one_event_info_for_B5(self.cur_cross_lane_for_people, self.history_cross_lane_for_people,
                              Abnormal_Class_Index.cross_lane_for_people)
        # print('cross_lane_for_people_num',len(self.all_event_output_for_B5)-last_num,file=self.use_time_traffic_fd,flush=True)
        # last_num=len(self.all_event_output_for_B5)

        one_event_info_for_B5(self.cur_cross_lane_for_motor, self.history_cross_lane_for_motor,
                              Abnormal_Class_Index.cross_lane_for_motor)
        # print('cross_lane_for_motor_num',len(self.all_event_output_for_B5)-last_num,file=self.use_time_traffic_fd,flush=True)
        # last_num=len(self.all_event_output_for_B5)

        one_event_info_for_B5(self.cur_run_the_red_light_for_motor, self.history_run_the_red_light_for_motor,
                              Abnormal_Class_Index.run_the_red_light_for_motor)
        # print('run_the_red_light_for_motor_num',len(self.all_event_output_for_B5)-last_num,file=self.use_time_traffic_fd,flush=True)
        # last_num=len(self.all_event_output_for_B5)

        one_event_info_for_B5(self.cur_run_the_red_light_for_people, self.history_run_the_red_light_for_people,
                              Abnormal_Class_Index.run_the_red_light_for_people)
        # print('run_the_red_light_for_people_num',len(self.all_event_output_for_B5)-last_num,file=self.use_time_traffic_fd,flush=True)
        # last_num=len(self.all_event_output_for_B5)

        one_event_info_for_B5(self.cur_occupy_bus_lane, self.history_occupy_bus_lane,
                              Abnormal_Class_Index.occupy_bus_lane)
        # print('occupy_bus_lane_num',len(self.all_event_output_for_B5)-last_num,file=self.use_time_traffic_fd,flush=True)
        # last_num=len(self.all_event_output_for_B5)

        one_event_info_for_B5(self.cur_occupy_emergency_lane, self.history_occupy_emergency_lane,
                              Abnormal_Class_Index.occupy_emergency_lane)

        one_event_info_for_B5(self.cur_change_lanes_illegal, self.history_change_lanes,
                              Abnormal_Class_Index.change_lanes)
        # print('change_lanes_num',len(self.all_event_output_for_B5)-last_num,file=self.use_time_traffic_fd,flush=True)
        # last_num=len(self.all_event_output_for_B5)

        one_detect_event_info_for_B5(self.cur_spills, self.history_spills, Abnormal_Class_Index.spills)
        # print('spills_num',len(self.all_event_output_for_B5)-last_num,file=self.use_time_traffic_fd,flush=True)

    def _init_lane_fix_info(self, lane_config_path):
        """车道对应航向角、输出车道号"""
        self.lane_fix_info = {}
        with open(lane_config_path + 'traffic_lane_fix.csv', 'r') as fd:
            dict_reader = csv.DictReader(fd)
            for row in dict_reader:
                lane_name = int(row['lane_name'])
                angle = float(row['angle']) / 180 * np.pi
                lane_output_name = int(row['lane_output_name'])

                self.lane_fix_info[lane_name] = Lane_Fix_Info(angle, lane_output_name)

    def fix_angle_for_pc(self, input_info, from_config=True):
        """按照车道固定航向角"""
        for i in range(len(input_info)):
            id = input_info[i, Pc_Info_Index.all_area_id.value]
            if id not in self.cur_id_lane:
                continue
            lane = self.cur_id_lane[id]
            if len(str(lane)) != self._normal_lane_str_len:
                continue
            # 在常规车道上
            if from_config:
                input_info[i, Pc_Info_Index.a.value] = self.lane_fix_info[lane].angle
            else:
                input_info[i, Pc_Info_Index.a.value] = self.lane_info[lane].angle_out \
                    if self.lane_info[lane].out_in == 0 else self.lane_info[lane].angle_in

    def get_lane_for_id(self, id):
        """输出id对应车道号"""
        if id not in self.cur_id_lane:
            return -1
        lane = self.cur_id_lane[id]
        if len(str(lane)) != self._normal_lane_str_len:
            return -1
        # 在常规车道上
        return lane

    def get_B0(self):
        t_now = time.time()

        B0_context = []
        normal_lane_num = len(self.lane_info) - len(self.turn_left_info)
        for lane in self.lane_info:
            if len(str(lane)) != self._normal_lane_str_len:
                continue
            this_lane_info = self.lane_info[lane]
            this_cur_statistics = self.cur_statistics[lane]
            B0_context.append([0,
                               lane,
                               this_lane_info.angle_to_north,
                               this_lane_info.lane_direction,
                               this_lane_info.is_non_motorway,
                               this_lane_info.max_v,
                               0,
                               this_cur_statistics['queue_length'],
                               this_cur_statistics['lane_occupancy'],
                               len(self.cur_lane_id[lane]),
                               0,
                               this_cur_statistics['space_headway'],
                               0]
                              )
        B0 = [0, 0, 0, normal_lane_num, 0, B0_context, 0]

        # print('B0',time.time()-t_now,file=self.use_time_traffic_fd,flush=True)
        return B0

    def get_B1(self):
        t_now = time.time()

        B1_context = []
        normal_lane_num = len(self.lane_info) - len(self.turn_left_info)
        for lane in self.lane_info:
            if len(str(lane)) != self._normal_lane_str_len:
                continue
            this_lane_info = self.lane_info[lane]
            this_total_statistics = self.total_statistics[lane]
            this_lane_flow = this_total_statistics['lane_flow']

            B1_context.append([0,
                               lane,
                               this_lane_info.angle_to_north,
                               this_lane_info.lane_direction,
                               this_lane_info.is_non_motorway,
                               this_lane_info.max_v * 3.6,
                               self.lane_local_static_congestion_state[lane]['all'].value,
                               this_lane_flow['motor_type'],
                               this_total_statistics['time_occupancy'],
                               this_lane_flow[Detect_Class_Type.car[0]],
                               this_lane_flow[Detect_Class_Type.truck[0]],
                               this_lane_flow[Detect_Class_Type.bus[0]],
                               this_lane_flow[Detect_Class_Type.motorcycle[0]],
                               0]
                              )
        B1 = [0, 0, 0, 0, normal_lane_num, 0, B1_context]

        # print('B1',time.time()-t_now,file=self.use_time_traffic_fd,flush=True)
        return B1

    # def get_B2(self):
    #     t_now=time.time()
    #
    #     B2_context=[]
    #
    #     all_pc_det_dict={}
    #     for pc_id in range(1,1+self.pc_num):
    #         all_pc_det_dict[pc_id]=[]
    #     for info in self.joint_pc_det:
    #         all_pc_det_dict[info[Pc_Info_Index.pc_id.value]].append(info)
    #     for pc_id in range(1,1+self.pc_num):
    #         all_pc_det_dict[pc_id]=np.array(all_pc_det_dict[pc_id])
    #
    #     self.use_camera_map.get_id_cam(all_pc_det_dict)
    #     self.get_event_output_by_id_cam(self.use_camera_map.id_cam)
    #     B2=[0,0,0,len(self.all_event_output_for_B2),0,self.all_event_output_for_B2,0]
    #
    #     #print('B2',time.time()-t_now,file=self.use_time_traffic_fd,flush=True)
    #     return B2

    def get_B3(self):
        t_now = time.time()

        self.B3_cnt += 1

        B3_context = []
        normal_lane_num = len(self.lane_info) - len(self.turn_left_info)
        for lane in self.lane_info:
            if len(str(lane)) != self._normal_lane_str_len:
                continue
            this_lane_info = self.lane_info[lane]
            this_cur_statistics = self.cur_statistics[lane]
            this_cur_lane_classes_num = self.cur_lane_classes_num[lane]

            B3_context.append([self.lane_fix_info[lane].lane_output_name,
                               3 if this_lane_info.is_emergency_lane == 1 else 1 if this_lane_info.is_non_motorway == 0 else 2,
                               this_lane_info.angle_to_north,
                               this_lane_info.max_v * 3.6,
                               self.cur_lane_mean_v[lane],
                               this_cur_statistics['queue_length'],
                               0 if len(this_cur_statistics['space_headway']) == 0 else np.mean(
                                   this_cur_statistics['space_headway']),
                               this_cur_statistics['lane_occupancy'],
                               0,
                               this_cur_lane_classes_num['motor_type'],
                               this_cur_lane_classes_num[Detect_Class_Type.car[0]],
                               this_cur_lane_classes_num[Detect_Class_Type.truck[0]],
                               this_cur_lane_classes_num[Detect_Class_Type.bus[0]],
                               this_cur_lane_classes_num[Detect_Class_Type.minibus[0]],
                               0,
                               this_cur_lane_classes_num[Detect_Class_Type.people[0]],
                               this_cur_lane_classes_num['non_motor_type'],
                               0]
                              )
        B3 = [0, 0, 0, 0, self.B3_cnt, normal_lane_num, 0, B3_context, 0]

        # print('B3',time.time()-t_now,file=self.use_time_traffic_fd,flush=True)
        return B3

    def get_B4(self):
        t_now = time.time()

        self.B4_cnt += 1

        B4_context = []
        normal_lane_num = len(self.lane_info) - len(self.turn_left_info)

        for lane in self.lane_info:
            if len(str(lane)) != self._normal_lane_str_len:
                continue
            this_lane_info = self.lane_info[lane]
            this_lane_local_static_congestion_state = self.lane_local_static_congestion_state[lane]
            this_total_statistics = self.total_statistics[lane]
            this_lane_flow = this_total_statistics['lane_flow']
            this_lane_mean_v_info_by_section = self.lane_mean_v_info_by_section[lane]

            B4_context.append([self.lane_fix_info[lane].lane_output_name,
                               3 if this_lane_info.is_emergency_lane == 1 else 1 if this_lane_info.is_non_motorway == 0 else 2,
                               this_lane_info.angle_to_north,
                               this_lane_info.max_v * 3.6,
                               self.pc_num,  # 基站数量
                               0,
                               [[pc_id, this_lane_local_static_congestion_state[pc_id].value, 0] for pc_id in
                                range(1, self.pc_num + 1)],
                               this_total_statistics['follow_car_percent'],
                               this_total_statistics['time_occupancy'],
                               self.lane_mean_space_headway_info[lane].mean_sh,
                               this_lane_flow[Detect_Class_Type.car[0]],
                               this_lane_flow[Detect_Class_Type.truck[0]],
                               this_lane_flow[Detect_Class_Type.bus[0]],
                               this_lane_flow[Detect_Class_Type.minibus[0]],
                               0,
                               this_lane_mean_v_info_by_section[Detect_Class_Type.car[0]].mean_v,
                               this_lane_mean_v_info_by_section[Detect_Class_Type.truck[0]].mean_v,
                               this_lane_mean_v_info_by_section[Detect_Class_Type.bus[0]].mean_v,
                               this_lane_mean_v_info_by_section[Detect_Class_Type.minibus[0]].mean_v,
                               Mean_V_Info.default_mean_v,
                               0
                               ])
        B4 = [0, 0, 0, 0, 0, 0, self.B4_cnt, normal_lane_num, 0, B4_context]

        return B4

    def get_B5(self):
        self.get_event_output_for_B5()
        B5 = [0, 0, 0, 0, len(self.all_event_output_for_B5), 0, self.all_event_output_for_B5, 0]

        # print('abnormal_num',len(self.all_event_output_for_B5),file=self.use_time_traffic_fd,flush=True)
        # if self.all_event_output_for_B5!=[]:
        #     for item in self.all_event_output_for_B5:
        # print('B5_item',item,file=self.use_time_traffic_fd,flush=True)
        # if len(self.all_event_output_for_B5) != 0:
        #     self.get_info_for_check()#每帧输出
        #     # print('事件编号','事件类型','基站号','目标车道','事件涉及目标个数','id','经度','纬度','开始时间s','开始时间ms',0)
        #     # for item in B5[6]:
        #     #     print(item)
        return B5

    # def get_info_for_check(self):
    #     """获取用于异常事件验证的信息,现在在B3帧调用"""
    #     self.info_for_check=[]
    #     corner={}
    #     for this_input_info in self.joint_pc_det:
    #         corner[this_input_info[Pc_Info_Index.all_area_id.value]]= \
    #             this_input_info[Pc_Info_Index.bottom_front_right_x.value:Pc_Info_Index.cur_frame_time.value]
    #
    #     def one_event_info_for_check(cur_event,history_infos,abnormal_class_index):
    #         event_info=[]
    #         for id in cur_event:
    #             this_history_info=history_infos[id][-1]
    #             # if id == 1261:
    #             #     print('aaa')
    #             time_list = str(this_history_info.end_time).split('.')
    #             second = int(time_list[0])
    #             miscros=int(time_list[1])
    #             this_event_info=[this_history_info.event_no,id]
    #             this_event_info.extend(corner[id])
    #             this_event_info.extend([abnormal_class_index,
    #                                     this_history_info.longitude,#经度
    #                                     this_history_info.latitude,#维度
    #                                     second,
    #                                     miscros])
    #             event_info.append(this_event_info)
    #         if len(event_info)!=0:
    #             self.info_for_check.extend(event_info)
    #
    #     one_event_info_for_check(self.cur_illegal_stop,self.history_illegal_stop,Abnormal_Class_Index.illegal_stop)
    #     one_event_info_for_check(self.cur_retrograde_for_non_motor,self.history_retrograde_for_non_motor,Abnormal_Class_Index.retrograde_for_non_motor)
    #     one_event_info_for_check(self.cur_retrograde_for_motor,self.history_retrograde_for_motor,Abnormal_Class_Index.retrograde_for_motor)
    #     one_event_info_for_check(self.cur_occupy_dedicated_lane,self.history_occupy_dedicated_lane,Abnormal_Class_Index.non_motor_in_tunnel)
    #     one_event_info_for_check(self.cur_people_occupy_motor_lane,self.history_people_occupy_motor_lane,Abnormal_Class_Index.person_in_tunnel)
    #     one_event_info_for_check(self.cur_speeding_for_motor,self.history_speeding_for_motor,Abnormal_Class_Index.speeding_for_motor)
    #     one_event_info_for_check(self.cur_speeding_for_non_motor,self.history_speeding_for_non_motor,Abnormal_Class_Index.speeding_for_non_motor)
    #     one_event_info_for_check(self.cur_stroll,self.history_stroll,Abnormal_Class_Index.stroll)
    #     one_event_info_for_check(self.cur_cross_line,self.history_cross_line,Abnormal_Class_Index.cross_line)
    #     one_event_info_for_check(self.cur_cross_lane_for_non_motor,self.history_cross_lane_for_non_motor,Abnormal_Class_Index.cross_lane_for_non_motor)
    #     one_event_info_for_check(self.cur_cross_lane_for_people,self.history_cross_lane_for_people,Abnormal_Class_Index.cross_lane_for_people)
    #     one_event_info_for_check(self.cur_cross_lane_for_motor,self.history_cross_lane_for_motor,Abnormal_Class_Index.cross_lane_for_motor)
    #     one_event_info_for_check(self.cur_run_the_red_light_for_motor,self.history_run_the_red_light_for_motor,Abnormal_Class_Index.run_the_red_light_for_motor)
    #     one_event_info_for_check(self.cur_run_the_red_light_for_people,self.history_run_the_red_light_for_people,Abnormal_Class_Index.run_the_red_light_for_people)
    #     one_event_info_for_check(self.cur_occupy_bus_lane,self.history_occupy_bus_lane,Abnormal_Class_Index.occupy_bus_lane)
    #     one_event_info_for_check(self.cur_change_lanes_illegal,self.history_change_lanes,Abnormal_Class_Index.change_lanes)
    #     one_event_info_for_check(self.cur_occupy_emergency_lane,self.history_occupy_emergency_lane,Abnormal_Class_Index.occupy_emergency_lane)
    #     self.info_for_check=np.array(self.info_for_check)
    #
    #
    #     check_path='./event_info_for_check/'
    #     if not os.path.exists(check_path):
    #         os.makedirs(check_path)
    #     data_df=pd.DataFrame(self.info_for_check)
    #     data_df.to_csv(check_path+"info_check_%d.csv"%self._own_frame_id,index=False,header=False)
    #     print(f'_own_frame_id:{self._own_frame_id}')


if __name__ == "__main__":
    import copy


    class Param(object):
        def __init__(self, in_parameter, rotate_matrix, translation_matrix, dist_matrix):
            self.in_parameter = in_parameter
            self.rotate_matrix = rotate_matrix
            self.translation_matrix = translation_matrix
            self.dist_matrix = dist_matrix


    camera_param_dict = {
        1: [
            Param(np.mat([[1147.9, 0, 951.6894], [0, 1143.4, 566.33], [0, 0, 1.0]]),
                  np.mat([1.06539647, -1.82687668, 1.37913931]),
                  np.mat([43.518, 171.177, 74.362]),
                  np.mat([-0.3335, 0.109, -0.00068918, 0.0013, 0])),
            Param(np.mat([[1152.4, 0, 959.4092], [0, 1147.4, 550.0649], [0, 0, 1.0]]),
                  np.mat([0.01584703, -2.5926309, 1.76196554]),
                  np.mat([87.55, 236.735, -122.542]),
                  np.mat([-0.3341, 0.1085, 0.000413, 0.00070638, 0])),
        ]
    }

    # traffic_flow=Traffic_Flow('/data/git_wj/WJ-ISFP/second/DepthViewer-NoSplitPro/Configs/Video/chedao/'+'virtual_config/',1)
    # traffic_flow=Traffic_Flow('/data/git_wj/WJ-ISFP/second/DepthViewer-NoSplitPro/Configs/Video/chedao/'+'virtual_config_fuqi/',6)
    # traffic_flow=Traffic_Flow('/data/git_wj/WJ-ISFP/second/DepthViewer-NoSplitPro/Configs/Video/chedao/'+'virtual_config_fuqi_0318/',12)
    traffic_flow = Traffic_Flow(r'/data/topxc/DepthViewer/Configs/Video/chedao/' + 'virtual_config/', 12)

    # origin_show_img=cv2.imread('/data/git_wj/WJ-ISFP/second/DepthViewer-NoSplitPro/Configs/Video/chedao/'+'virtual_config/lane.png')
    # origin_show_img=cv2.imread('/data/git_wj/WJ-ISFP/second/DepthViewer-NoSplitPro/Configs/Video/chedao/'+'virtual_config_fuqi/statistics.png')
    # origin_show_img=cv2.imread('/data/git_wj/WJ-ISFP/second/DepthViewer-NoSplitPro/Configs/Video/chedao/'+'virtual_config_fuqi_0318/statistics.png')
    origin_show_img = cv2.imread(r'/data/topxc/DepthViewer/Configs/Video/chedao/' + 'virtual_config/statistics.png')
    firstpoint = False
    for px, py in traffic_flow.pc_bmp_xy:
        cv2.rectangle(origin_show_img, (px - 10, py - 10), (px + 10, py + 10), (0, 255, 0), thickness=5)
        if not firstpoint:
            firstpoint = True
            cv2.circle(origin_show_img, (px, py), 200, (0, 0, 255), thickness=1)
            # cv2.circle(origin_show_img,)
    cv2.imwrite(r'./display.png', origin_show_img)

    offset = {
        102: [0, 20],
        111: [0, 40],
        112: [0, 60],
        102211: [0, 100],
        206311: [-100, 0],
        301411: [0, -100],
        406111: [100, 0]
    }

    # lane_pix={}#{lane:{congestion_x:np.array(pix_x,pix_y)}}
    # for i in range(traffic_flow.config_matrix.shape[0]):
    #     for j in range(traffic_flow.config_matrix.shape[1]):
    #         this_lane_no=traffic_flow.config_matrix[i,j,0]
    #         if this_lane_no==-1:
    #             continue
    #         this_lane=traffic_flow.lane_no_lane_name[this_lane_no]
    #         if this_lane not in lane_pix:
    #             lane_pix[this_lane]={}
    #         congestion_x=traffic_flow._bmp_to_congestion_map(np.array([i,j]))[0]
    #         if congestion_x not in lane_pix[this_lane]:
    #             lane_pix[this_lane][congestion_x]=[]
    #         lane_pix[this_lane][congestion_x].append([i,j])
    # for lane in lane_pix:
    #     for congestion_x in lane_pix[lane]:
    #         lane_pix[lane][congestion_x]=np.array(lane_pix[lane][congestion_x])

    begin_i = 28601
    draw_statistics_flag = False

    np.random.seed(122504)
    COLORS = np.random.randint(0, 255, size=(500, 3), dtype="uint8")

    occupy_dedicated_lane_set = []
    occupy_bus_lane_set = []
    people_occupy_motor_lane_set = []
    retrograde_for_motor_set = []
    retrograde_for_non_motor_set = []
    speeding_for_motor_set = []
    stroll_set = []
    cross_line_set = []
    illegal_stop_set = []
    cross_lane_for_motor_set = []
    cross_lane_for_non_motor_set = []
    cross_lane_for_people_set = []
    change_lanes_set = []
    occupy_emergency_lane_set = []

    B0_time = time.time()
    B1_time = time.time()
    B2_time = time.time()
    B3_time = time.time()
    B4_time = time.time()
    B5_time = time.time()
    B0_flag = False
    B1_flag = False
    B2_flag = False
    B3_flag = False
    B4_flag = False
    B5_flag = False

    path = './use_time_traffic/'
    if not os.path.exists(path):
        os.makedirs(path)
    if os.path.exists(path + 'event_result.txt'):
        os.remove(path + 'event_result.txt')
    event_traffic_fd = open('./use_time_traffic/event_result.txt', 'a')
    # for h in range(20):
    for frame_i in range(begin_i, 64221):
        statime = time.time()
        # while True:
        #     time.sleep(0.1)
        #     break
        # real_frame = frame_i+34824
        # if frame_i==45:
        #     print(frame_i)
        print(frame_i)
        try:
            # lidar_info=np.genfromtxt('/data/WJ-ISFP/PointData/%d.csv'%frame_i,delimiter=',')
            # lidar_info=np.hstack([lidar_info[:,:Pc_Info_Index.s.value],np.ones((len(lidar_info),10)),lidar_info[:,Pc_Info_Index.s.value:]])

            # lidar_info=np.genfromtxt('/data/git_wj/WJ-ISFP/event/eventDetectData%d.csv'%frame_i,delimiter=',')
            # lidar_info=np.hstack([lidar_info[:,:Pc_Info_Index.latitude.value],np.ones((len(lidar_info),2)),lidar_info[:,Pc_Info_Index.latitude.value:]])

            # lidar_info=np.genfromtxt('/data/wj-alltracker-python/second/traffic_events/traffic_source/EventData/eventDetectData%d.csv'%frame_i,delimiter=',')

            # lidar_info=np.genfromtxt(r'/data/event_0420/simulation/EventData_050308/eventDetectData%d.csv'%real_frame,delimiter=',')
            # lidar_info=np.genfromtxt(r'/data/topxc/DepthViewer/savedata/EventData05141617/eventDetectData%d.csv'%frame_i,delimiter=',')
            lidar_info = np.genfromtxt(
                r'/data/event_0420/result_pic/0513/1617/EventData05131617/eventDetectData%d.csv' % frame_i,
                delimiter=',')
            lidar_info = lidar_info.reshape((-1, 46))
            # lidar_info=np.genfromtxt('/data/git_wj/WJ-ISFP/fusion_pc_csv_1/ch%d.csv'%frame_i,delimiter=',')
            # lidar_info=np.hstack([lidar_info[:,:Pc_Info_Index.src.value],np.ones((len(lidar_info),9)),lidar_info[:,Pc_Info_Index.src.value:]])

            # 模拟定时发送
            t_now = time.time()
            if t_now - B0_time >= 0.1:  # 10Hz
                B0_flag = True
                B0_time = t_now
            else:
                B0_flag = False

            if t_now - B1_time >= 60:  # 1/60Hz
                B1_flag = True
                B1_time = t_now
            else:
                B1_flag = False

            if t_now - B2_time >= 0.1:  # 10Hz
                B2_flag = True
                B2_time = t_now
            else:
                B2_flag = False

            if t_now - B3_time >= 1:  # 1Hz
                B3_flag = True
                B3_time = t_now
            else:
                B3_flag = False

            if t_now - B4_time >= 60:  # 1/60Hz
                B4_flag = True
                B4_time = t_now
            else:
                B4_flag = False

            if t_now - B5_time >= 5:  # 0.2Hz
                B5_flag = True
                B5_time = t_now
            else:
                B5_flag = False

            t_1 = time.time()
            if B1_flag:  # 进行统计统计量  B1与B4应该同频率
                traffic_flow.use(lidar_info, True)
            else:
                traffic_flow.use(lidar_info, False)

            b0, b1, b2, b3, b4, b5 = 0, 0, 0, 0, 0, 0

            # if B0_flag:
            #     b0=traffic_flow.get_B0()
            # if B1_flag:
            #     b1=traffic_flow.get_B1()
            # if B2_flag:
            #     b2=traffic_flow.get_B2()
            if B3_flag:
                b3 = traffic_flow.get_B3()
            if B4_flag:
                b4 = traffic_flow.get_B4()
            # if B5_flag:
            b5 = traffic_flow.get_B5()
            eventPub = False
            if b5[6] != []:
                eventPub = True
                event_traffic_fd.close()
                event_traffic_fd = open('./use_time_traffic/event_result.txt', 'a')
                event_traffic_fd.write(f'frame_id:{frame_i}\n')
                print('----------------------------b5--------------------------------------')
                print('事件编号', '事件类型', '基站号', '目标车道', '事件涉及目标个数', 'id', '经度', '纬度', '开始时间s', '开始时间ms', 0)
                for item in b5[6]:
                    print(item)
                    event_traffic_fd.write(f'{item}\n')

            # sssss=time.time()
            # print(f'use___time:{(sssss-statime)*1000}ms')
            # t_used=time.time()-t_1
            # #print('all',t_used,file=traffic_flow.use_time_traffic_fd,flush=True)
            #
            # # traffic_flow.fix_angle_for_pc(lidar_info)
            # # lane_for_all_id=traffic_flow.get_lane_for_all_id(lidar_info)
            # # output_info=traffic_flow.get_event_output()
            #
            # show_img=copy.copy(origin_show_img)
            # for id,info in traffic_flow.cur_id_info.items():
            #     cv2.putText(show_img,'%d'%id,(int(info.center_px),int(info.center_py)),cv2.FONT_HERSHEY_COMPLEX,6e-1,(0,0,255),2)
            #     cv2.circle(show_img,(int(info.center_px),int(info.center_py)),10,(0,0,255))
            #     cv2.circle(show_img,(int(info.front_left_px),int(info.front_left_py)),3,(255,0,0))#蓝
            #     cv2.circle(show_img,(int(info.front_right_px),int(info.front_right_py)),3,(0,255,0))#绿
            #     cv2.circle(show_img,(int(info.behind_left_px),int(info.behind_left_py)),3,(0,0,255))#红
            #     cv2.circle(show_img,(int(info.behind_right_px),int(info.behind_right_py)),3,(0,0,0))#黑
            #
            # # for lane,info in traffic_flow.lane_info.items():
            # #     this_offset=[0,0] if lane not in offset else offset[lane]
            # #     cv2.putText(show_img,'ql:%.2f'%traffic_flow.cur_statistics[lane]['queue_length'],\
            # #         (traffic_flow.lane_info[lane].start_px+this_offset[0],traffic_flow.lane_info[lane].start_py+this_offset[1]),\
            # #             cv2.FONT_HERSHEY_COMPLEX,6e-1,(0,0,255),2)
            # #     cv2.putText(show_img,'lo:%.2f'%traffic_flow.cur_statistics[lane]['lane_occupancy'],\
            # #         (traffic_flow.lane_info[lane].start_px+this_offset[0]+100,traffic_flow.lane_info[lane].start_py+this_offset[1]),\
            # #             cv2.FONT_HERSHEY_COMPLEX,6e-1,(0,0,255),2)
            # #     cv2.putText(show_img,'sh:'+'%s'%traffic_flow.cur_statistics[lane]['space_headway'] if len(traffic_flow.cur_statistics[lane]['space_headway'])>=2 else 'None',
            # #         (traffic_flow.lane_info[lane].start_px+this_offset[0]+200,traffic_flow.lane_info[lane].start_py+this_offset[1]),\
            # #             cv2.FONT_HERSHEY_COMPLEX,6e-1,(0,0,255),2)
            # #     # cv2.putText(show_img,'th:'+'%s'%traffic_flow.cur_statistics[lane]['time_headway'] if len(traffic_flow.cur_statistics[lane]['time_headway'])>=2 else 'None',
            # #     #     (traffic_flow.lane_info[lane].start_px+this_offset[0]+200,traffic_flow.lane_info[lane].start_py+this_offset[1]),\
            # #     #         cv2.FONT_HERSHEY_COMPLEX,6e-1,(0,0,255),2)
            #
            # # if B1_flag:
            # #     for lane,info in traffic_flow.lane_info.items():
            # #         this_offset=[0,0] if lane not in offset else offset[lane]
            # #         cv2.putText(show_img,'fl:%s'%traffic_flow.total_statistics[lane]['lane_flow']['motor_type'],\
            # #             (traffic_flow.lane_info[lane].start_px+this_offset[0],traffic_flow.lane_info[lane].start_py+this_offset[1]),\
            # #                 cv2.FONT_HERSHEY_COMPLEX,6e-1,(0,0,255),2)
            # #         cv2.putText(show_img,'to:%.2f'%traffic_flow.total_statistics[lane]['time_occupancy'],\
            # #             (traffic_flow.lane_info[lane].start_px+this_offset[0]+50,traffic_flow.lane_info[lane].start_py+this_offset[1]),\
            # #                 cv2.FONT_HERSHEY_COMPLEX,6e-1,(0,0,255),2)
            # #         cv2.putText(show_img,'ths:%s'%traffic_flow.total_statistics[lane]['time_headway_section'],\
            # #             (traffic_flow.lane_info[lane].start_px+this_offset[0]+100,traffic_flow.lane_info[lane].start_py+this_offset[1]),\
            # #                 cv2.FONT_HERSHEY_COMPLEX,6e-1,(0,0,255),2)
            # #         cv2.putText(show_img,'fcp:%.2f'%traffic_flow.total_statistics[lane]['follow_car_percent'],\
            # #             (traffic_flow.lane_info[lane].start_px+this_offset[0]+150,traffic_flow.lane_info[lane].start_py+this_offset[1]),\
            # #                 cv2.FONT_HERSHEY_COMPLEX,6e-1,(0,0,255),2)
            #
            # occupy_dedicated_lane_set.extend(traffic_flow.cur_occupy_dedicated_lane)
            # occupy_bus_lane_set.extend(traffic_flow.cur_occupy_bus_lane)
            # people_occupy_motor_lane_set.extend(traffic_flow.cur_people_occupy_motor_lane)
            # retrograde_for_motor_set.extend(traffic_flow.cur_retrograde_for_motor)
            # retrograde_for_non_motor_set.extend(traffic_flow.cur_retrograde_for_non_motor)
            # speeding_for_motor_set.extend(traffic_flow.cur_speeding_for_motor)
            # stroll_set.extend(traffic_flow.cur_stroll)
            # cross_line_set.extend(traffic_flow.cur_cross_line)
            # illegal_stop_set.extend(traffic_flow.cur_illegal_stop)
            # cross_lane_for_motor_set.extend(traffic_flow.cur_cross_lane_for_motor)
            # cross_lane_for_non_motor_set.extend(traffic_flow.cur_cross_lane_for_non_motor)
            # cross_lane_for_people_set.extend(traffic_flow.cur_cross_lane_for_people)
            # change_lanes_set.extend(traffic_flow.cur_change_lanes_illegal)
            # occupy_emergency_lane_set.extend(traffic_flow.cur_occupy_emergency_lane)
            # entime=time.time()
            # print(f'time_one_frame:{(entime-statime)*1000}ms')
            # # if frame_i>30000000+begin_i and eventPub:
            # if frame_i>3+begin_i:
            #     for i in range(1,11):#1~10的颜色
            #         show_img[50:150,100*i-50:100*i+50,:]=COLORS[i]
            #
            #     spills_size=int(traffic_flow.param['spills_meter']*traffic_flow.param['bmp_ratio']*traffic_flow.param['meter_to_pixel']/2)
            #     spills_data=traffic_flow.spills_matrix#w,h,1
            #     for i in range(spills_data.shape[0]):#w
            #         for j in range(spills_data.shape[1]):#h
            #             if spills_data[i,j]==0:
            #                 continue
            #             this_xy=traffic_flow._spills_coordinate_to_bmp(np.array([i,j]))
            #             show_img[int(np.clip(this_xy[1]-spills_size,0,traffic_flow.config_matrix.shape[0])):int(np.clip(this_xy[1]+spills_size,0,traffic_flow.config_matrix.shape[0])),
            #             int(np.clip(this_xy[0]-spills_size,0,traffic_flow.config_matrix.shape[1])):int(np.clip(this_xy[0]+spills_size,0,traffic_flow.config_matrix.shape[1])),:]=COLORS[spills_data[i,j]]
            #
            #     for x,y in traffic_flow.cur_spills:
            #         this_xy=traffic_flow._spills_coordinate_to_bmp(np.array([x,y]))
            #         px,py=this_xy
            #         box=traffic_flow.cur_bbox[(x,y)]
            #         wh=traffic_flow._spills_coordinate_to_bmp(np.array([box[2],box[3]]))
            #         cv2.rectangle(show_img,(px-wh[0]//2,py-wh[1]//2),(px+wh[0]//2,py+wh[1]//2),(0,255,0),thickness=5)
            #
            #     # for i in range(traffic_flow.congestion_matrix.shape[0]):
            #     #     for j in range(traffic_flow.congestion_matrix.shape[1]):
            #     #         this_xy=traffic_flow._congestion_map_to_bmp(np.array([i,j]))
            #
            #     #         this_lane_no=traffic_flow.config_matrix[this_xy[0],this_xy[1],0]
            #     #         if this_lane_no==-1:
            #     #             continue
            #     #         this_lane_name=traffic_flow.lane_no_lane_name[this_lane_no]
            #     #         # cv2.putText(show_img,'%d'%traffic_flow.lane_mean_v_info_by_all_v[this_lane_name]['meter'][i].mean_v,\
            #     #         #     (this_xy[0],this_xy[1]),\
            #     #         #         cv2.FONT_HERSHEY_COMPLEX,6e-1,(0,0,255),2)
            #     #         show_img[lane_pix[this_lane_name][i][:,1],lane_pix[this_lane_name][i][:,0],:]=\
            #     #                     COLORS[traffic_flow.lane_local_static_congestion_state[this_lane_name]['meter'][i].value]
            #     # for lane in lane_pix:
            #     #     for congestion_x in lane_pix[lane]:
            #     #         show_img[lane_pix[lane][congestion_x][:,1],lane_pix[lane][congestion_x][:,0],:]=\
            #     #             COLORS[traffic_flow.lane_local_static_congestion_state[lane]['meter'][congestion_x].value]
            #
            #     cv2.namedWindow('show',cv2.WINDOW_KEEPRATIO)
            #     cv2.imshow('show',show_img)
            #     cv2.waitKey() & 0xFF==ord('q')

        except Exception as e:
            # print(frame_i,e.args,file=traffic_flow.use_time_traffic_fd,flush=True)
            print(frame_i, e.args)

    # occupy_dedicated_lane_set=np.unique(np.array(occupy_dedicated_lane_set))
    # occupy_bus_lane_set=np.unique(np.array(occupy_bus_lane_set))
    # people_occupy_motor_lane_set=np.unique(np.array(people_occupy_motor_lane_set))
    # retrograde_for_motor_set=np.unique(np.array(retrograde_for_motor_set))
    # retrograde_for_non_motor_set=np.unique(np.array(retrograde_for_non_motor_set))
    # speeding_for_motor_set=np.unique(np.array(speeding_for_motor_set))
    # stroll_set=np.unique(np.array(stroll_set))
    # cross_line_set=np.unique(np.array(cross_line_set))
    # illegal_stop_set=np.unique(np.array(illegal_stop_set))
    # cross_lane_for_motor_set=np.unique(np.array(cross_lane_for_motor_set))
    # cross_lane_for_non_motor_set=np.unique(np.array(cross_lane_for_non_motor_set))
    # cross_lane_for_people_set=np.unique(np.array(cross_lane_for_people_set))
    # change_lanes_set=np.unique(np.array(change_lanes_set))
    # occupy_emergency_lane_set=np.unique(np.array(occupy_emergency_lane_set))
    #
    # import csv
    # fp=open("./result.csv","w+",encoding='utf-8-sig')
    # wcsv=csv.writer(fp)
    # wcsv.writerow(('异常类型','id'))
    # wcsv.writerow(('机动车占用非机动车道、非机动车占用机动车道',occupy_dedicated_lane_set))
    # wcsv.writerow(('占用公交专用车道',occupy_bus_lane_set))
    # wcsv.writerow(('行人占用机动车道',people_occupy_motor_lane_set))
    # wcsv.writerow(('机动车逆行',retrograde_for_motor_set))
    # wcsv.writerow(('非机动车逆行',retrograde_for_non_motor_set))
    # wcsv.writerow(('机动车超速',speeding_for_motor_set))
    # wcsv.writerow(('慢行',stroll_set))
    # wcsv.writerow(('压线',cross_line_set))
    # wcsv.writerow(('违停',illegal_stop_set))
    # wcsv.writerow(('机动车横穿马路',cross_lane_for_motor_set))
    # wcsv.writerow(('非机动车横穿马路',cross_lane_for_non_motor_set))
    # wcsv.writerow(('行人横穿马路',cross_lane_for_people_set))
    # wcsv.writerow(('变道',change_lanes_set))
    # wcsv.writerow(('emergency area',occupy_emergency_lane_set))
