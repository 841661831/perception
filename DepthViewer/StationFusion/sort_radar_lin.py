"""
    SORT: A Simple, Online and Realtime Tracker
    Copyright (C) 2016 Alex Bewley alex@dynamicdetection.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""
from __future__ import print_function
from numba import jit
import numpy as np
import time
import math
from sklearn.utils.linear_assignment_ import linear_assignment
from filterpy.kalman import KalmanFilter
import second.core.box_np_ops as box_np_ops
import os
Rads_cov = 180 / math.pi
label_dict={0:[1.95017717, 4.60718145, 1.72270761],1:[0.60058911, 1.68452161, 1.27192197],2:[2.94046906, 11.1885991, 3.47030982],
            3:[0.76279481, 2.09973778, 1.44403034],4:[0.66344886, 0.7256437, 1.75748069],5:[0.39694519, 0.40359262, 1.06232151],
            6:[2.4560939, 6.73778078, 2.73004906]}
#############对radar计算iou值##############
def rotate_nms_cc(dets,trackers):
    trackers_corners = box_np_ops.center_to_corner_box2d(trackers[:, :2], trackers[:, 2:4], trackers[:, 4])
    trackers_standup = box_np_ops.corner_to_standup_nd(trackers_corners)
    dets_corners = box_np_ops.center_to_corner_box2d(dets[:, :2], dets[:, 2:4],dets[:, 4])
    dets_standup = box_np_ops.corner_to_standup_nd(dets_corners)
    standup_iou = box_np_ops.iou_jit(dets_standup, trackers_standup, eps=0.0)
    return standup_iou


@jit(nopython=True)
def iou_jit(boxes, query_boxes, eps=0.0):
    """calculate box iou. note that jit version runs 2x faster than cython in
    my machine!
    Parameters
    ----------
    boxes: (N, 4) ndarray of float
    query_boxes: (K, 4) ndarray of float
    Returns
    -------
    overlaps: (N, K) ndarray of overlap between boxes and query_boxes
    """
    N = boxes.shape[0]
    K = query_boxes.shape[0]
    overlaps = np.zeros((N, K), dtype=boxes.dtype)
    for k in range(K):
        box_area = ((query_boxes[k, 2] - query_boxes[k, 0] + eps) *
                    (query_boxes[k, 3] - query_boxes[k, 1] + eps))
        for n in range(N):
            iw = (min(boxes[n, 2], query_boxes[k, 2]) - max(
                boxes[n, 0], query_boxes[k, 0]) + eps)
            if iw > 0:
                ih = (min(boxes[n, 3], query_boxes[k, 3]) - max(
                    boxes[n, 1], query_boxes[k, 1]) + eps)
                if ih > 0:
                    ua = (
                        (boxes[n, 2] - boxes[n, 0] + eps) *
                        (boxes[n, 3] - boxes[n, 1] + eps) + box_area - iw * ih)
                    overlaps[n, k] = iw * ih / ua
    return overlaps

def cal_jg_angle(angle_box, bbox, center_angle, speed):
    if len(angle_box) != 0:
        head_angle = angle_box[-1][4]
        count_head = 0
        more_25 = []
        less_25 = []
        for k in angle_box:
            if abs(k[4] - head_angle) > 90 and abs(k[4] - head_angle) < 270:
                count_head += 1
                temp_head = k[4]
                more_25.append(k[4])
            else:
                less_25.append(k[4])

        if count_head >= 0.5 * len(angle_box):
            head_final = sum(more_25) / len(more_25)
        else:
            head_final = sum(less_25) / len(less_25)
    else:
        head_final = bbox[4]
    if speed > 0.5 and (bbox[7] in [1, 3, 4]) and center_angle != 1000:
        head_final = center_angle

    return head_final

# def cal_cg_angle(angle_box, bbox, center_angle):
#     #####*****利用相邻帧位置计算航向角并存储*****#####
#     c_angle = []
#     list_len = len(angle_box)
#     if list_len > 1:
#         dx = angle_box[-1][0] - angle_box[-2][0]
#         dy = angle_box[-1][1] - angle_box[-2][1]
#         lenth = (dx ** 2 + dy ** 2) ** 0.5
#         angle = math.acos(dx / lenth) * Rads_cov - 180
#         if dy > 0:
#             angle = 90 - angle
#         else:
#             angle = 90 - (360 - angle)
#     else:
#         angle = bbox[4]
#     c_angle.append(angle)
#     if len(c_angle) > 10:
#         c_angle = c_angle[-10:]
#     if len(c_angle) > 10:
#         c_angle = np.array(c_angle)
#         ave_angle = (np.sum(c_angle) - np.max(c_angle) - np.min(c_angle)) / 8
#     else:
#         ave_angle = center_angle
#     return ave_angle

def cal_angle(state_list, angle_start, thresh, angle_list):
    dis_x = state_list[-1][0] - state_list[0][0]
    dis_y = state_list[-1][1] - state_list[0][1]
    dis_len = (dis_x * dis_x + dis_y * dis_y) ** 0.5
    if dis_len > thresh:
        dis_angle = math.acos(dis_x / dis_len) * Rads_cov - 180
        if dis_y > 0:
            dis_angle = 90 - dis_angle
        else:
            dis_angle = 90 - (360 - dis_angle)
        dis_angle = (dis_angle % 360)
        angle_diff = abs(angle_list[-1] - dis_angle)
        if angle_diff > (180-angle_start) and angle_diff < (180+angle_start):
            angle_list[-1] += 180
            angle_list[-1] = angle_list[-1] % 360
        elif angle_diff < angle_start or angle_diff > (360-angle_start):
            pass
        else:
            angle_list[-1] = dis_angle
        # #####*****解决航向角横摆90度的问题*****#####
        # elif angle_diff > angle_start and angle_diff < (180-angle_start):
        #     angle_list[-1] = dis_angle
        # elif angle_diff > (180+angle_start) and angle_diff < (360-angle_start):
        #     angle_list[-1] = dis_angle
    else:
        if len(angle_list) > 1:
            angle_diff_s = abs(angle_list[-1] - angle_list[-2])
            # #####*****  2020-0811 add for meng  *****#####
            if angle_diff_s > (180-angle_start) and angle_diff_s < (180+angle_start):
                angle_list[-1] += 180
                angle_list[-1] = angle_list[-1] % 360
            elif angle_diff_s < angle_start or angle_diff_s > (360 - angle_start):
                pass
            else:
                angle_list[-1] = angle_list[-2]

    return angle_list

# def cal_lane(start, end):
#     k = (end[1] - start[1]) / (end[0] - start[0])
#     b = end[1] - k * end[0]
#     return k, b


class KalmanBoxTracker(object):
  """
  This class represents the internel state of individual tracked objects observed as bbox.
  """
  count = 0
  def __init__(self, bbox):
    """
    Initialises a tracker using initial bounding box.
    """
    # define constant velocity model
    self.kf = KalmanFilter(dim_x=7, dim_z=6)
    dt = 0.1
    self.kf.F = np.array([[1,0,0,0,dt,0,0],
                          [0,1,0,0,0,dt,0],
                          [0,0,1,0,0,0,0],
                          [0,0,0,1,0,0,0],
                          [0,0,0,0,1,0,0],
                          [0,0,0,0,0,1,0],
                          [0,0,0,0,0,0,1]]) #状态转移矩阵
    self.kf.H = np.array([[1,0,0,0,0,0,0],
                          [0,1,0,0,0,0,0],
                          [0,0,1,0,0,0,0],
                          [0,0,0,1,0,0,0],
                          [0,0,0,0,1,0,0],
                          [0,0,0,0,0,1,0]])#测量矩阵

    self.kf.R[2:,2:] *= 10.
    self.kf.P[4:,4:] *= 1000 #give high uncertainty to the unobservable initial velocities
    self.kf.P *= 10.
    self.kf.Q[-1,-1] *= 0.01
    self.kf.Q[4:,4:] *= 0.01
    ####使用box_lidar,里面包含中心点，面积和航向角度
    # self.kf.x[:4] = bbox[:4].reshape((-1,1))
    self.kf.x[:6] = bbox[:6].reshape((-1, 1))       #初始化的时候用了前6个状态x,y,l,w,vx,vy
    self.bbox=bbox          ###对应存储的状态值,只有第一帧的时候是用的输入值
    self.time_since_update = 0
    self.id = KalmanBoxTracker.count
    KalmanBoxTracker.count += 1
    self.history = []
    self.head_angle = []
    self.kf_speed = []
    self.kf_speed_x = []
    self.kf_speed_y = []
    ######self.state存储卡尔曼滤波的状态量，目前来看存储的是预测的状态量，有更新时存储的是更新后的状态量
    self.state = []
    self.speed_box_out = []

    self.angle_box=[]#####对应history
    self.label_box=[]#####存储历史10帧的类别消息，用途是，寻找每个目标10帧内出现次数最多的标签作为新一帧目标
    self.label_size = [[1.95017717, 4.60718145, 1.72270761], [0.60058911, 1.68452161, 1.27192197],
                       [2.94046906, 11.1885991, 3.47030982], [0.76279481, 2.09973778, 1.44403034],
                       [0.66344886, 0.7256437, 1.75748069], [0.39694519, 0.40359262, 1.06232151],
                       [2.4560939, 6.73778078, 2.73004906]]
    self.hits = 0
    self.hit_streak = 0
    self.age = 0 #####表示跟踪目标可以存活的时间（帧数）
    ################追踪目标的速度################
    self.speed = 0
    self.center_angle = 0
    #####*****是否采用jun brother的方法*****#####
    self.use_jg_way = 1
    #####*****曹工方法的航向角存储*****#####
    self.c_angle = []
    #####*****是否对卡尔曼预测的速度进行限制，防止检测框飞出的现象*****#####
    self.kf_vl_limit_mode = 0
    #####**********航向角是否平滑**********#####
    self.head_angle_smooth_mode = 1
    #####*****存储更改的label信息*****#####
    self.label = 0
    #####*****针对于速度较快的人,一般是误识别为人,该参数表示是否显示在屏幕上*****#####
    self.show = 1
    #####*****行人速度阈值*****#####
    self.speed_thresh = 3
    #####*****匹配后的id*****######
    self.match_id = bbox[-1]
  def update(self,bbox):
    # print('update')
    """
    Updates the state vector with observed bbox.
    """
    self.match_id = bbox[-1]
    self.time_since_update = 0 ####表示距离上一次更新后没有再匹配更新的次数
    self.history = []
    self.hits += 1 ###表示在age时间内其能够匹配上的次数
    self.hit_streak += 1##表示连续匹配上的次数，从第一次开始
    ####使用匹配到的box信息对当前的卡尔曼状态量进行更新
    # self.kf.update(bbox[:4].reshape((-1,1)))
    self.kf.update(bbox[:6].reshape((-1, 1)))       #更新变为六行
    self.bbox = bbox

    #######计算的平滑速度
    delta_x = bbox[0] - self.kf.x[0]
    delta_y = bbox[1] - self.kf.x[1]
    if len(self.state)>=4:
        delta_x1 = self.state[-1][0][0] - self.state[-4][0][0]
        delta_y1 = self.state[-1][1][0] - self.state[-4][1][0]
        length = (delta_x1 ** 2 + delta_y1 ** 2) ** 0.5
        speed = length / 0.3

    else:
        delta_x1 = delta_x[0]
        delta_y1 = delta_y[0]
        length = (delta_x1 ** 2 + delta_y1 ** 2) ** 0.5
        speed = length / 0.1

    # #####*****  2020-0810 for speed change,add it  *****#####
    self.speed_box_out.append(speed)
    # print('now,the speed is :',speed)
    if len(self.speed_box_out)>5:
        self.speed_box_out = self.speed_box_out[-5:]
    # print('更新前',bbox)
    # print('update_bbox', self.bbox)
    # print('update_x', self.kf.x[:4])

######计算的平滑航向角   相邻4帧计算平滑的速度和航向角，感觉应该根据状态量计算
    # if self.head_angle_smooth_mode == 1:
    #     if speed>0.5 and len(self.angle_box)>=4:
    #         center_angle = math.acos(delta_x1 / length) * Rads_cov - 180
    #         if delta_y1 > 0:
    #             self.center_angle = 90 - center_angle
    #         else:
    #             self.center_angle = 90 - (360 - center_angle)
    #     else:
    #         self.center_angle=1000
    # #######不使用平滑的航向角
    # else:
    #     self.center_angle = self.bbox[4]
    # self.center_angle = (self.center_angle % 360)
    #
    # # #####*****  2020-0810  add new for speed  *****#####
    # if len(self.speed_box_out)>4:
    #     ck_speed = sum(self.speed_box_out)/5
    #     if abs(self.speed_box_out[-1] - ck_speed) > 1.5:
    #         self.speed_box_out[-1] = ck_speed
    # elif len(self.speed_box_out)>1 :
    #     if abs(self.speed_box_out[-1] - self.speed_box_out[-2]) > 1.5:
    #         self.speed_box_out[-1] = self.speed_box_out[-2]
    #
    self.speed = self.speed_box_out[-1]

  def predict(self):
    """
    Advances the state vector and returns the predicted bounding box estimate.
    """
    # print('predict')
    if((self.kf.x[6]+self.kf.x[2])<=0):
      self.kf.x[6] *= 0.0
    self.kf.predict()       # kf.x 变为预测值
    self.age += 1
    if(self.time_since_update>0):
      self.hit_streak = 0
    self.time_since_update += 1
    ########30##直接使用box_lidar#####不需要进行转换##########
    output_history = self.kf.x[:4].reshape((1, 4))  #取预测值的前四个x,y,l,w
    self.history.append(output_history)     #加入到self.history
    self.label_box.append(self.bbox[7])
    ######self.angle_box存储的是检测状态量
    self.angle_box.append(self.bbox)
    ######保证列表的长度，防止长度过长
    if len(self.label_box) > 10:
        self.label_box = self.label_box[-10:]
        self.angle_box = self.angle_box[-10:]
    # print('anglebox', self.angle_box[-1])
    ######分析历史帧的标签，取次数最多的label，并赋对应的box尺寸
    if len(self.label_box) > 0:
        more_label = max(self.label_box, key=self.label_box.count)# 取self.label_box里面重复次数最多的元素
        self.label = more_label
        # q = label_dict[self.label][0]
        # print(label_dict[self.label][0])
        self.bbox[2] = label_dict[self.label][1]    #长
        self.bbox[3] = label_dict[self.label][0]    #宽
        self.bbox[5] = label_dict[self.label][2]    #高
        ###discuss，这个标签在之前的十帧里面出现了7帧以上，就把当前的标签也令为该标签
        if self.label_box.count(self.label) / len(self.label_box) > 0.7:
            self.bbox[7] = self.label

    else:
        self.label = self.bbox[7]

    #####为了限制卡尔曼速度的变化太剧烈，一般是误检或者遮挡目标会出现剧烈的速度变化
    # if self.kf_vl_limit_mode == 1:
    #     kf_speed_x = self.kf.x[4]
    #     kf_speed_y = self.kf.x[5]
    #     kf_speed = (self.kf.x[4] * self.kf.x[4] + self.kf.x[5] * self.kf.x[5]) ** 0.5
    #
    #     ########kalman速度的计算########
    #     self.kf_speed.append(kf_speed[0])
    #     self.kf_speed_x.append(kf_speed_x[0])
    #     self.kf_speed_y.append(kf_speed_y[0])
    #     if len(self.kf_speed) > 10:
    #         self.kf_speed = self.kf_speed[-10:]
    #         self.kf_speed_x = self.kf_speed_x[-10:]
    #         self.kf_speed_y = self.kf_speed_y[-10:]
    #     ##########对卡尔曼的估计速度进行限制，加速度不得超过5m/s2    状态量包括加速度时候可以对加速度分析
    #     if len(self.kf_speed_x) > 1:
    #         diff_x = self.kf_speed_x[-1] - self.kf_speed_x[-2]
    #         if abs(diff_x) > 0.35:
    #             self.kf_speed_x[-1] = self.kf_speed_x[-2] + 0.1 * 3.5 * diff_x / abs(diff_x)
    #             self.kf.x[4] = self.kf_speed_x[-2] + 0.1 * 3.5 * diff_x / abs(diff_x)
    #         if abs(self.kf.x[4]) < 0.5:
    #             self.kf.x[4] = 0
    #         diff_y = self.kf_speed_y[-1] - self.kf_speed_y[-2]
    #         if abs(diff_y) > 0.35:
    #             self.kf_speed_y[-1] = self.kf_speed_y[-2] + 0.1 * 3.5 * diff_y / abs(diff_y)
    #             self.kf.x[5] = self.kf_speed_y[-2] + 0.1 * 3.5 * diff_y / abs(diff_y)
    #         if abs(self.kf.x[5]) < 0.5:
    #             self.kf.x[5] = 0

    ######返回状态量的前四项，分别是质心x, y和长宽   和检测的目标状态
    # print('label_box', self.label_box)
    # print('angle_box', self.angle_box)
    # print('self.bbox', self.bbox)
    # print('predict_x and anglebox', self.history[-1], self.angle_box[-1])
    return self.history[-1], self.angle_box[-1]

  def get_state(self):
    """
    Returns the current bounding box estimate.返回当前边界框估计值
    """
    ##########直接使用box_lidar#####不需要进行转换##########
    output_x=self.kf.x[:4].reshape((1,4))
    if self.speed < 0.5 and len(self.angle_box) > 1:        #速度小于一定值
        # print('self.angle_box', self.angle_box)
        x_mean = np.asarray(self.angle_box)     #angle_box放的是设定的长宽，而output_x是雷达检测的长宽
        output_x = np.mean(x_mean[:, :4], axis=0).reshape((1, 4))   #取angle_box的平均值
    # print(self.kf.x[:4])
    # 幸好返回值用的是output_x的前两个output_x[0,1],self.bbox的[2，3，5，6],而self.bbox在update时已经改为了雷达检测的长宽
    return output_x, self.bbox, self.speed, self.angle_box, self.center_angle

def associate_detections_to_trackers(detections,trackers,iou_threshold = 0.1):
  """
  Assigns detections to tracked o192.168.3.181bject (both represented as bounding boxes)
  Returns 3 lists of matches, unmatched_detections and unmatched_trackers
  返回三个列表：匹配上的、没匹配上的目标、没匹配上的轨迹
  """
  if(len(trackers)==0):
    return np.empty((0,2),dtype=int), np.arange(len(detections)), np.empty((0, 5), dtype=int)

    #######distance match ###############
  # det_xy = detections[:, :2]
  # trk_xy = trackers[:, :2]
  # distance_matrix = (np.reshape(np.sum(det_xy ** 2, axis=1), (det_xy.shape[0], 1)) + np.sum(trk_xy ** 2,axis=1) - 2 * det_xy.dot(trk_xy.T)) ** 0.5
  # distance_matrix = 1 - (distance_matrix / 160)

  ######直接使用lidar部分的iou，这里直接使用矩阵就行#####
  iou_matrix = rotate_nms_cc(detections,trackers)   #计算iou
  # print('iou_matrix', iou_matrix)
  # final_matrix = 0.3 * distance_matrix + 0.7 *iou_matrix
  # # matched_indices = linear_assignment(-final_matrix)
  matched_indices = linear_assignment(-iou_matrix)
  unmatched_detections = []
  for d, det in enumerate(detections):
    if(d not in matched_indices[:,0]):
      unmatched_detections.append(d)
  unmatched_trackers = []
  for t, trk in enumerate(trackers):
    if(t not in matched_indices[:,1]):
      unmatched_trackers.append(t)

  #filter out matched with low IOU
  matches = []
  for m in matched_indices:
    if(iou_matrix[m[0],m[1]]<iou_threshold):
      unmatched_detections.append(m[0])
      unmatched_trackers.append(m[1])
    else:
      matches.append(m.reshape(1,2))
  if(len(matches)==0):
    matches = np.empty((0,2),dtype=int)
  else:
    matches = np.concatenate(matches,axis=0)
  return matches, np.array(unmatched_detections), np.array(unmatched_trackers)

class Sort(object):
  def __init__(self,max_age=4,min_hits=2):
    """
    Sets key parameters for SORT
    """
    self.max_age = max_age
    self.min_hits = min_hits
    self.trackers = []
    self.frame_count = 0
    #####*****判断动态目标和静态目标的阈值(暂时觉得速度作为判断条件不太靠谱，采用位移作为判断条件)*****#####
    self.dis_thresh = 1.0
  def update(self,dets):

    """
    Params:
      dets - a numpy array of detections in the format [[x1,y1,x2,y2,score],[x1,y1,x2,y2,score],...]
    Requires: this method must be called once for each frame even with empty detections.参数输入为
    x,y,长,宽，角度（框的方位角），
    Returns the a similar array, where the last column is the object ID.返回一个类似的数组，其中最后一列是对象ID
    NOTE: The number of objects returned may diffkf_speed_xer from the number of detections provided.
    注意：返回的对象数可能与提供的检测数不同
    """
    # print('box', dets)
    self.frame_count += 1
    #get predicted locations from existing trackers.
    # print('1111111', self.trackers)
    trks = np.zeros((len(self.trackers), 5))            #为什么是5列：x,y,l,w,theta
    # print('trks1', trks)
    to_del = []
    ret = []
    for t, trk in enumerate(trks):
      pos, bbox = self.trackers[t].predict()  #预测,pos是预测到的状态，bbox是
      pos = pos[0]
      trk[:] = [pos[0], pos[1], pos[2], pos[3], bbox[4]]    #

      if(np.any(np.isnan(pos))):
        to_del.append(t)
    trks = np.ma.compress_rows(np.ma.masked_invalid(trks))  #预测后的情况，前4个是状态量，后一个是
    # print('trks2', trks)
    for t in reversed(to_del):
      self.trackers.pop(t)
    '''dets是当前帧的输入数据，即观测值，trks是在上一次得到结果的基础上再预测的值(当前帧预测量)---两者匹配'''
    matched, unmatched_dets, unmatched_trks = associate_detections_to_trackers(dets, trks)   #dets是输入
    # print('matched: ', matched)
    # print('unmatched_dets', unmatched_dets)
    # print('unmatched_trks', unmatched_trks)

    #update matched trackers with assigned detections,匹配之后更新

    for t, trk in enumerate(self.trackers):        # if ((trk.time_since_update < self.max_age) and (trk.hits >= self.min_hits or self.frame_count <= self.min_hits)):
      if(t not in unmatched_trks):
        d = matched[np.where(matched[:,1] == t)[0],0] #找到下标t的跟踪器对应匹配的观测值det的目标索引
        trk.update(dets[d, :][0])        # 对当前匹配上的观测目标进行更新
      elif trk.kf_vl_limit_mode == 1 and len(trk.kf_speed_x) > 1:   #kf_vl_limit_mode一直都是0，所以这个条件不会满足
        trk.kf.x[4] = trk.kf_speed_x[-2]
        trk.kf.x[5] = trk.kf_speed_y[-2]
    #create and initialise new trackers for unmatched detections
    # unmatched_dets为没匹配的目标，一第一帧数据的目标肯定都是没匹配的，所以一开始为这些目标创建跟踪器
    # 每个目标一个跟踪器，把他们放入self.trackers
    for i in unmatched_dets:
        trk = KalmanBoxTracker(dets[i,:])
        self.trackers.append(trk)
    i = len(self.trackers)
    # print('2222222', self.trackers)
    for trk in reversed(self.trackers):     #为什么要翻转
        d, x_temp, trk_speed, trk_angle_box, center_angle = trk.get_state()
        # print('self.kf.x', trk.kf.x)
        # print('output_x', d)
        # print('x_temp', x_temp)
        # print('trk_speed', trk_speed)
        # print('trk_angle', trk_angle_box)
        # print('center_angle', center_angle)
        trk.state.append(trk.kf.x)  #状态
        if len(trk.state)>10:
            trk.state=trk.state[-10:]
        d = d[0]        #x轴
        if center_angle < 0:
            center_angle += 360

        if ((trk.time_since_update < self.max_age) and (trk.hits >= self.min_hits or self.frame_count <= self.min_hits)):
            # print('heeeeeeeeeeeeeeee')
            # #####*****line param*****#####
            # #####*****加入车道信息限制航向角*****#####
            # #####*****此处的车道信息是手工选择车道点云计算的车道直线方程*****#####
            # #####*****车道的定义如下,括号里是该车道对应的航向角************#####
            # #####*****              l1    l2     l3
            # #####*****               |     |     |
            # #####*****               |     |     |    l表示车道线
            # #####*****               |  1  |  2  |
            # #####*****               | (0) |(180)|    *****L为雷达的安装位置*****
            # #####***** l12 __________|     |     |__________ l4
            # #####*****    8(270)                 3(270)
            # #####***** l11 __________             __________ l5
            # #####*****    7 (90)                 4(90)
            # #####***** l10 __________             __________ l6
            # #####*****             L |     |     |              ^ y
            # #####*****               |     |     |              |
            # #####*****               |  6  |  5  |              |
            # #####*****               | (0) |(180)|              |
            # #####*****               |     |     |   坐标系定义   -------> x
            # #####*****               |     |     |
            # #####*****               l9    l8    l7
            # #####***** start, end分别为取车道线上的两点，为了计算车道的直线方程 *****#####
            # #####***** 分别计算每一条车道直线的方程 *****#####
            # #####***** 根据示例判断当前检测目标的质心是否在车道内，或者说是在哪个车道内，不同的车道对应不同的航向角 *****#####
            # ##########*******************##########
            # ##########******example******##########
            # ##########*******************##########
            # start1 = np.array([0,1])
            # end1 = np.array([6,6])
            # start2 = np.array([2,3])
            # end2 = np.array([9,9])
            # k1, b1 = cal_lane(start1, end1)
            # k2, b2 = cal_lane(start2, end2)
            # # #####*****判断检测目标的质心是否在车道1内*****#####
            # diff_l1 = d[1] - (k1 * d[0] + b1)
            # diff_l2 = d[1] - (k2 * d[0] + b2)
            # # #####*****region1_thresh是为了限定路口之外的行车区域，我们在路口中有自定义的航向角策略*****#####
            # if (diff_l1 * diff_l2) < 0 and d[1] > region1_thresh:
            #     head_final = 0
            #     trk.head_angle.append(head_final)

            # #####***** 采用jun brother计算航向角的方法
            if trk.use_jg_way == 1:
                head_final = cal_jg_angle(trk_angle_box, x_temp, center_angle, trk_speed)
                if head_final < 0:
                    head_final += 360
                head_final = head_final % 360
                trk.head_angle.append(head_final)
                if len(trk.head_angle) > 10:
                    trk.head_angle = trk.head_angle[-10:]


                #####*****条件合并的判断语句  (new)*****#####
                #####*****显示已经调整航向角的目标检测框信息*****#####
            # print('trk.label', trk.label)
            if trk.label != 2 and trk.label != 6:
                if len(trk.state) > 9:
                    angle_start = 10
                    thresh = self.dis_thresh
                    trk.head_angle = cal_angle(trk.state, angle_start, thresh, trk.head_angle)
                    # head_final = trk.head_angle[-1]
                elif len(trk.state) > 4:
                    angle_start = 30
                    thresh = 0.5 * self.dis_thresh
                    trk.head_angle = cal_angle(trk.state, angle_start, thresh, trk.head_angle)
                    # head_final = trk.head_angle[-1]
                #####*****显示航向角调整阶段的目标检测框信息*****#####
                else:
                    # #####*****  22020-0811  add for meng  *****#####
                    if len(trk.head_angle) > 1:
                        angle_start = 50
                        angle_diff = abs(trk.head_angle[-1] - trk.head_angle[-2])
                        if angle_diff > (180-angle_start) and angle_diff < (180+angle_start):
                                trk.head_angle[-1] += 180
                                trk.head_angle[-1] = trk.head_angle[-1] % 360
                        elif angle_diff < angle_start or angle_diff > (360-angle_start):
                            pass
                        else:
                            trk.head_angle[-1] = trk.head_angle[-2]
                head_final = trk.head_angle[-1]
            # x,y,
            # d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
            d_conv = [d[0], d[1], trk.kf.x[2][0], trk.kf.x[3][0], head_final, x_temp[5], x_temp[6], trk.label]
            # ret.append(np.concatenate((d_conv, [trk_speed], [trk.id + 1])).reshape(1, -1))
            ret.append(np.concatenate((d_conv, [trk_speed], [trk.match_id])).reshape(1, -1))
        i -= 1

            #remove dead tracklet
        if(trk.time_since_update > self.max_age):
            self.trackers.pop(i)
    # print('ret', ret)
    if(len(ret)>0):
        return np.concatenate(ret)
    return np.empty((0,5))

'''mayuan'''
# p=Sort(max_age=4,min_hits=2)
# # path="E:\smarttest/1103data/05/"
# # path = "D:/wanji/Linxiao/1030Radar_Radar/radar_radar_view/TrackData/doubleradar1/"
# path = "D:/wanji/Linxiao/1030Radar_Radar/radar_radar_view/TrackData/testdata2/"  #孟
# filenames = os.listdir(path)
# # 循环每一帧目标数据
# for i in range(len(filenames)):
#     pcapPath = path + filenames[i]
#     f=open(pcapPath)
#     points=[]
#     #遍历当前帧的目标
#     for line in f.readlines():
#         line = line.split(' ')
#         line = [float(str(i)) for i in line]
#         points.append([line[0],line[1],line[2],line[3],line[4],line[5],line[6],line[7]])
#     points=np.array(points)
#     a = p.update(points)      #当前帧的目标数据作为输入，a是输出
#     print('result', a)
#     print('================================')
# =============================
    # savePath = "D:/wanji/Linxiao/1030Radar_Radar/radar_radar_view/TrackData/result/"
    # if not os.path.exists(savePath):
    #     os.makedirs(savePath)
    #     break
    # with open(savePath + str(i+100) + ".txt", 'w') as f:
    #     print('here')
    #     np.savetxt(f, np.array(a), fmt='%f', delimiter=' ')

'''林潇'''
# p=Sort(max_age=4,min_hits=2)
# points = np.array([[ 97.7278653, -4.50979493, 5., 3., 0., 1., -8.3, 0.],
#  [112.0560657, -2.52689063, 5.6, 3., 0., 1.,-10.4, 0.],
#  [124.7535507, -8.81395286, 5.6, 3., 0., 1., 9.3, 0.],
#  [142.32657956, -2.66105436, 5., 3., 0., 1., -8.4, 0.],
#  [ 65.408, 0.512, 5.6, 3., 0., 1., 0., 0.],
#  [ 77.00295662, 2.22115865, 4.6, 3., 0., 1., -0., 0.],
#  [ 71.8891567, 1.41290465, 1., 3., 0., 1., -0., 0.]])
# a = p.update(points)      #当前帧的目标数据作为输入，a是输出
# print(a)
