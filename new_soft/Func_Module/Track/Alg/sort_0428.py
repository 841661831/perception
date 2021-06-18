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
import math
from sklearn.utils.linear_assignment_ import linear_assignment
import Alg.core.box_np_ops as box_np_ops
from Alg.all_track import get_iou, cal_lateral_dis, get_pixel_location, cal_box_lane, cal_map_angle
from Alg.new_kalman_filter import KalmanFilter

lidarLongitude = [113.3295452925999, 113.3288191047671, 113.3278804022179, 113.3268057132405, 113.3256247957487, 113.3242191281648, 113.3228005937,
                  113.3213102846250, 113.3198098654866, 113.3183278528113, 113.3168669564, 113.3154122596798]
lidarLatitude = [22.9768546291, 22.9759441589820, 22.9752029989194, 22.9746839336564, 22.9743325240624, 22.9740019427648, 22.9736703894, 22.9733216358291,
                 22.9729631680335, 22.9725601601250, 22.9725601601250, 22.9721048663, 22.9716423540572]
angleNorthT = [300.82, 314.314, 321.377, 335.656, 342.9, 345.271, 343.25, 343.5, 342.25, 341.008, 338.408, 340.2]
Rads_cov = 180 / math.pi

#############对lidar计算iou值##############
def rotate_nms_cc(dets,trackers):
    trackers_corners = box_np_ops.center_to_corner_box2d(trackers[:, :2], trackers[:, 2:4], trackers[:, 4]*np.pi/180)
    dets_corners = box_np_ops.center_to_corner_box2d(dets[:, :2], dets[:, 2:4], dets[:, 4]*np.pi/180)
    standup_iou, standup_iou_new = get_iou(dets_corners, trackers_corners, 1)
    return standup_iou, standup_iou_new


@jit(nopython=True)
def iou_jit_new(boxes, query_boxes, eps=0.0):
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
    overlaps_new = np.zeros((N, K), dtype=boxes.dtype)
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
                    overlaps_new[n, k] = iw * ih / box_area
    return overlaps, overlaps_new

def cal_angle(state_list, thresh):
    dis_x = state_list[-1][0][0] - state_list[0][0][0]
    dis_y = state_list[-1][0][1] - state_list[0][0][1]
    dis_len = (dis_x * dis_x + dis_y * dis_y) ** 0.5
    if dis_len > thresh:
        dis_angle = math.acos(dis_x / dis_len) * Rads_cov - 180
        if dis_y > 0:
            dis_angle = 90 - dis_angle
        else:
            dis_angle = 90 - (360 - dis_angle)
        dis_angle = (dis_angle % 360)
    else:
        dis_angle = None
    return dis_angle


class KalmanBoxTracker(object):
  """
  This class represents the internel state of individual tracked objects observed as bbox.
  """
  count = 0
  def __init__(self,bbox, left_center, medium_center, right_center, bmp_img):
    """
    Initialises a tracker using initial bounding box.
    """
    # #####*****  车道中心线信息  *****#####
    self.left_center = left_center
    self.medium_center = medium_center
    self.right_center = right_center

    # #####*****  车道BMP  *****#####
    self.bmp_img = bmp_img

    self.use_acc_model = 0
    if bbox[7] in list([0, 2, 5, 6]):
    # if bbox[7] in list([0, 1, 2, 3, 4, 5, 6]):
    # if bbox[7] in list([10]):
        self.use_acc_model = 1
        self.kf = KalmanFilter(dim_x=8, dim_z=4)
        self.kf.F = np.array([[1, 0, 0, 0, 0.1, 0, 0.005, 0],
                              [0, 1, 0, 0, 0, 0.1, 0, 0.005],
                              [0, 0, 1, 0, 0, 0, 0, 0],
                              [0, 0, 0, 1, 0, 0, 0, 0],
                              [0, 0, 0, 0, 1, 0, 0.1, 0],
                              [0, 0, 0,  0, 0, 1, 0, 0.1],
                              [0, 0,  0, 0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 0, 0, 0, 1]])
        self.kf.H = np.array(
            [[1, 0, 0, 0, 0, 0, 0, 0],
             [0, 1, 0, 0, 0, 0, 0, 0],
             [0, 0, 1, 0, 0, 0, 0, 0],
             [0, 0, 0, 1, 0, 0, 0, 0]
             ])
        self.kf.Q *= 0.1
        self.kf.R *= 1
        self.kf.P *= 10
        self.kf.P[4:6, 4:6] *= 1000
    else:
        self.use_acc_model = 0
        self.kf = KalmanFilter(dim_x=7, dim_z=4)
        self.kf.F = np.array([[1, 0, 0, 0, 1, 0, 0],
                              [0, 1, 0, 0, 0, 1, 0],
                              [0, 0, 1, 0, 0, 0, 1],
                              [0, 0, 0, 1, 0, 0, 0],
                              [0, 0, 0, 0, 1, 0, 0],
                              [0, 0, 0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 0, 0, 1]])
        self.kf.H = np.array([[1, 0, 0, 0, 0, 0, 0],
                              [0, 1, 0, 0, 0, 0, 0],
                              [0, 0, 1, 0, 0, 0, 0],
                              [0, 0, 0, 1, 0, 0, 0],
                              ])
        self.kf.Q[-1, -1] *= 0.01
        self.kf.Q[4:, 4:] *= 0.01
        self.kf.R *= 10
        self.kf.P *= 10
        self.kf.P[4:6, 4:6] *= 1000

    ####使用box_lidar,里面包含中心点，面积和航向角度
    self.kf.x[:4] = bbox[:4].reshape((-1, 1))
    self.bbox=bbox###对应存储的状态值
    self.time_since_update = 0

    # #####*****  减小和增大卡尔曼Q矩阵的标志位  *****#####
    self.min_q = True
    self.max_q = False

    # #####*****  存储检测目标的稳定航向角，以车道角和轨迹角为标准
    self.final_angle = None

    # #####*****  self.id 表示检测目标的ID  *****#####
    self.id = KalmanBoxTracker.count
    KalmanBoxTracker.count += 1



    # #####*****  存储目标的状态，在预测函数中会输出  *****#####
    self.history = []
    self.head_angle = []
    # #####*****  self.state存储卡尔曼滤波的状态量，目前来看存储的是预测的状态量，有更新时存储的是更新后的状态量(状态添加的位置可变，视情况而定)  *****#####
    self.state = []
    # #####*****  self.label_dict存储的对应的目标的anchor信息  *****#####
    self.label_dict = {0: [1.80, 4.31, 1.59], 1: [0.68, 1.76, 1.68],
                  2: [2.95, 10.96, 3.24],
                  3: [1.43, 2.73, 1.89], 4: [0.64, 0.78, 1.73],
                  5: [3.01, 14.96, 3.91],
                  6: [2.53, 7.20, 3.08]}

    # #####*****  存储检测出来的航向角，保证检测角不跳变  *****#####
    self.angle_list = []

    # #####*****  高速目标  低速目标  *****#####
    self.high_speed = False
    self.low_speed = False

    # #####*****  动态目标  静态目标  *****#####
    self.dynamic = False
    self.static = False

    # #####*****  存储状态量判断目标的动静属性,不使用动静属性的时候可以不用  *****#####
    self.state_judge = []

    # #####*****  车道航向角  轨迹航向角  检测航向角  *****#####
    self.lane_angle = None
    self.track_angle = None
    self.detec_angle = None

    # #####*****  存储的也是状态量，是一个列表，长度在10，在get_state()中会操作输出  *****#####
    self.angle_box=[]
    # #####*****  存储历史10帧的类别消息，用途是，寻找每个目标10帧内出现次数最多的标签作为新一帧目标  *****#####
    self.label_box=[]
    # #####*****  击中次数 也就是更新的次数  *****#####
    self.hits = 0

    ################追踪目标的速度################
    self.speed = 0
    self.acc = 0
    #####*****存储更改的label信息*****#####
    self.label = 0
    #####*****高速和低速的阈值*****#####
    self.speed_thresh = 3

    # #####*****  记录目标最大的尺寸  *****#####
    self.l_max = -1000
    self.w_max = -1000
    self.h_max = -1000

    # #####*****  车道信息  *****#####
    self.lane = bbox[-1]
    # #####*****  存储历史状态  *****#####
    self.state_list = []

    # #####*****  存储历史的速度  *****#####
    self.velocity_list = []
    self.acc_list = []

    # #####*****   可信速度  *****#####
    self.brief_velocity = None
    self.brief_acc = None

    # #####*****  目标在高精地图上的航向角  *****#####
    self.map_head_angle = None

    # # #####*****  基站坐标系的经纬度和正北夹角  *****#####
    # self.original_long = lidarLongitude[1]
    # self.original_lat = lidarLatitude[1]
    # self.original_north = angleNorthT[1]


    # # #####*****  参考坐标系的经纬度和正北夹角  *****#####
    # self.reference_long = lidarLongitude[0]
    # self.reference_lat = lidarLatitude[0]
    # self.reference_north = angleNorthT[0]

  def update_id(self):
      self.id = KalmanBoxTracker.count
      KalmanBoxTracker.count += 1

  def update(self,bbox):
    """
    Updates the state vector with observed bbox.
    """
    # # #####*****  全域目标的尺寸保持  *****#####
    # if self.l_max > bbox[3]:
    #     bbox[3] = self.l_max
    # if self.w_max > bbox[2]:
    #     bbox[2] = self.w_max
    # if self.h_max > bbox[6]:
    #     bbox[6] = self.h_max
    # if abs(bbox[0]) < 40:
    #     if self.l_max < bbox[3]:
    #         self.l_max = bbox[3]
    #     if self.w_max < bbox[2]:
    #         self.w_max = bbox[2]
    #     if self.h_max < bbox[6]:
    #         self.h_max = bbox[6]


    reduce = 0
    # if self.kf.x.shape[0] == 8:
    #     if self.label in [2, 5, 6] and self.hits > 10:
    #         reduce = 1

    ####表示距离上一次更新后没有再匹配更新的次数
    self.time_since_update = 0
    self.history = []
    self.hits += 1 ###表示在age时间内其能够匹配上的次数
    ####使用匹配到的box信息对当前的卡尔曼状态量进行更新
    self.kf.update(bbox[:4].reshape((-1,1)),reduce, 1)
    # self.map_head_angle = bbox[4]

    # #####*****  根据历史状态与当前帧更新状态，计算新的合速度  *****#####
    diff_velocity = None
    if self.hits > 10:
        # dis = math.sqrt((self.kf.x[0][0] - self.state_list[-2]) ** 2 + (self.kf.x[1][0] - self.state_list[-1]) ** 2)
        dis_x = self.kf.x[0][0] - self.state_list[-2]
        dis_y = self.kf.x[1][0] - self.state_list[-1]
        angle = ((-1 * bbox[4] - 90) % 360) * np.pi / 180
        head_dis = dis_x * math.cos(angle) + dis_y * math.sin(angle)
        # update_velocity = dis / 0.1
        update_velocity = head_dis / 0.1
        if self.brief_velocity is not None:
            diff_velocity = abs(update_velocity - self.brief_velocity)


    # #####*****  更新存储的目标信息  *****#####
    self.bbox=bbox
    self.lane = bbox[-1]

    # # #####*****  该部分只是为了解决新的模型类别个数与旧模型不匹配，确定新模型的时候，可以修改self.label_dict，并去掉这部分
    # if self.bbox[7] == 5:
    #     self.bbox[7] = 4
    self.label_box.append(self.bbox[7])
    ######保证列表的长度，防止长度过长
    if len(self.label_box) > 50:
        self.label_box = self.label_box[-50:]
    ######分析历史帧的标签，取次数最多的label，并赋对应的box尺寸
    if len(self.label_box) > 0:
        more_label = max(self.label_box, key=self.label_box.count)
        self.label = more_label
        # #####*****  只限制框的宽度，是因为聚类时框的宽度会变的特别窄  *****#####
        # self.bbox[2] = self.label_dict[self.label][0]
        # # # #####*****  这部分是修改目标的尺寸  *****#####
        # # # #####*****  针对之前的关于修正卡车和公交车的部分，在此处改车和非机动车的尺寸  *****#####
        # if self.label in [0, 1, 3, 4, 5]:
        #     self.bbox[2] = self.label_dict[self.label][0]
        #     self.bbox[3] = self.label_dict[self.label][1]
        #     self.bbox[6] = self.label_dict[self.label][2]
        # #####*****  满足条件时修改目标状态的标签信息，或者同时修改存储标签信息的列表，保证目标类别不跳变
        if self.label_box.count(self.label) / len(self.label_box) > 0.7:
            self.bbox[7] = self.label
            if self.label_box.count(self.label) > 35:
                self.label_box[-1] = more_label
    else:
        self.label = self.bbox[7]

    # #####*****  因为加速和匀速模型对于F矩阵的定义不一样，造成速度计算方式不一样，后续可以直接修改F矩阵即可，就不用这部分的判断条件了  *****#####
    if self.use_acc_model:
        self.speed = math.sqrt(self.kf.x[4] ** 2 + self.kf.x[5] ** 2)
        self.acc = math.sqrt(self.kf.x[6] ** 2 + self.kf.x[7] ** 2)
    else:
        self.speed = 10 * math.sqrt(self.kf.x[4] ** 2 + self.kf.x[5] ** 2)
        self.acc = 0

    # # #####*****  保证相邻两帧速度变化不至于太大  *****#####
    # if diff_velocity is not None:
    #     if diff_velocity > 3.5:
    #         self.speed = self.velocity_list[-1]
    #     else:
    #         self.speed = update_velocity

    # #####*****  高速低速目标的判别  *****#####
    if self.speed > self.speed_thresh:
        self.high_speed = True
        self.low_speed = False
    else:
        self.high_speed = False
        self.low_speed = True
    # # #####*****  低速的时候，将卡尔曼的Q矩阵变小，让轨迹更平滑，即受离谱检测目标的影响会小些，待测试的功能  *****#####
    # # #####*****  考虑对机动车采用Q矩阵策略 (可以按照动静和高速低速来进行判断的方式) *****#####
    if self.label in [0, 2, 5, 6]:
        if (not self.high_speed) and self.min_q:
            self.kf.Q *= 0.1
            self.min_q = False
            self.max_q = True
        if self.high_speed and self.max_q:
            self.kf.Q *= 10
            self.min_q = True
            self.max_q = False

    ######self.angle_box存储的是检测状态量
    self.angle_box.append(self.bbox)
    ######保证列表的长度，防止长度过长
    if len(self.angle_box) > 10:
        self.angle_box = self.angle_box[-10:]

  def predict(self):
    """
    Advances the state vector and returns the predicted bounding box estimate.
    """
    # #####*****  利用高精度地图计算目标的车道航向角，对没有更新的目标，赋予新的航向角信息  *****#####
    # #####*****  同时，最后一位保留的是目标的车道号，从左到右依次是123  *****#####
    if self.time_since_update > 0:
        state_info = np.array([[self.kf.x[0][0], self.kf.x[1][0], self.kf.x[2][0], self.kf.x[3][0], self.bbox[4], self.bbox[5], self.bbox[6], self.lane]])
        # state_info = XYZ_To_BLH_batch(self.original_long, self.original_lat, state_info, self.original_north)
        # state_info = lonlat_to_xyz_batch(state_info, self.reference_long, self.reference_lat, self.reference_north)
        state_info = cal_box_lane(self.bmp_img, state_info)
        # #####*****  angle_flag, 1表示第7列是航向角, 0表示第5列是航向角  *****#####
        angle_flag = 0
        state_info = cal_map_angle(state_info, self.left_center, self.medium_center, self.right_center, angle_flag)
        self.map_head_angle = state_info[0][4]
        # self.brief_velocity += self.acc * 0.1
        self.speed = self.brief_velocity + self.acc * 0.1
        self.lane = state_info[0][-1]
    else:
        self.map_head_angle = self.bbox[4]

    # if self.speed > 2 or self.time_since_update < 3:
    start_speed = 0
    if self.hits > 10:
        start_speed = 1
    self.kf.predict_new(start_speed, self.brief_velocity, self.map_head_angle, self.brief_acc)
    # self.kf.predict()

    # # #####*****  assign the displacement  *****#####
    # diff_x = self.kf.x[0][0] - self.state_list[-2]
    # diff_y = self.kf.x[1][0] - self.state_list[-1]
    # dis = math.sqrt(diff_x*diff_x+diff_y*diff_y)
    # new_angle = (self.map_head_angle * -1 - 90) % 360
    # self.kf.x[0][0] = self.state_list[-2] + dis * math.cos(new_angle*np.pi/180)
    # self.kf.x[1][0] = self.state_list[-1] + dis * math.sin(new_angle*np.pi/180)

    self.time_since_update += 1
    ########30##直接使用box_lidar#####不需要进行转换##########
    output_history = self.kf.x[:4].reshape((1, 4))
    self.history.append(output_history)

    # #####*****  这里存储的是判断动静的状态量，在预测中给出，保证状态的连续性  *****#####
    self.state_judge.append(self.kf.x)
    if len(self.state_judge) > 10:
        self.state_judge = self.state_judge[-10:]

    return self.history[-1],self.bbox

  def get_state(self):
    """
    Returns the current bounding box estimate.
    """
    ##########直接使用box_lidar#####不需要进行转换##########
    output_x=self.kf.x[:4].reshape((1,4))

    # #####*****  对于速度低的目标做一个平滑  *****#####
    if self.speed < 0.5 and len(self.angle_box) > 1:
        x_mean = np.asarray(self.angle_box)
        output_x = np.mean(x_mean[:, :4], axis=0).reshape((1, 4))
    return output_x, self.bbox, self.speed, self.angle_box, 0

def associate_detections_to_trackers(detections, trackers, flag, stParam):
  """
  Assigns detections to tracked o192.168.3.181bject (both represented as bounding boxes)

  Returns 3 lists of matches, unmatched_detections and unmatched_trackers
  """
  if(len(trackers)==0):
    return np.empty((0,2),dtype=int), np.arange(len(detections)), np.empty((0,5),dtype=int), 0
  head_dis, ano_dis, dis_matrix = cal_lateral_dis(detections, trackers, 0)
  if flag == 1:
      #####直接使用lidar部分的iou，这里直接使用矩阵就行#####
      detections[:, 2] = detections[:, 2] * stParam.fConeThreshold
      trackers[:, 2] = trackers[:, 2] * stParam.fConeThreshold
      iou_matrix, iou_matrix_new = rotate_nms_cc(detections, trackers)

      detection_lane = detections[:, -1].reshape(-1, 1)
      trackers_lane = trackers[:, -1].reshape(1, -1)
      lane_similar = detection_lane - trackers_lane
      iou_matrix = iou_matrix - np.abs(lane_similar)

      cost_matrix = iou_matrix
      iou_threshold = 0.000001
      matched_indices = linear_assignment(-cost_matrix)
      detections[:, 2] = detections[:, 2] / stParam.fConeThreshold
      trackers[:, 2] = trackers[:, 2] / stParam.fConeThreshold
  else:
      # head_dis, ano_dis, dis_matrix = cal_lateral_dis(detections, trackers, flag)
      final_dis = head_dis + 3 * ano_dis
      dis_thresh = stParam.Horizon_threshoud + 3 * stParam.Vertical_threshoud + 30     #  [00] hor    01  ver
      final_dis[final_dis > dis_thresh] = 10000
      cost_matrix = final_dis
      dis_threshold = 18
      matched_indices = linear_assignment(cost_matrix)
  unmatched_detections = []
  for d,det in enumerate(detections):
    if(d not in matched_indices[:,0]):
      unmatched_detections.append(d)
  unmatched_trackers = []
  for t,trk in enumerate(trackers):
    if(t not in matched_indices[:,1]):
      unmatched_trackers.append(t)

  matches = []
  for m in matched_indices:
    if flag == 1:
        if cost_matrix[m[0], m[1]] < iou_threshold or ano_dis[m[0], m[1]] > stParam.Vertical_threshoud:
            unmatched_detections.append(m[0])
            unmatched_trackers.append(m[1])
        else:
            matches.append(m.reshape(1, 2))
    else:
        if head_dis[m[0], m[1]] > stParam.Horizon_threshoud or ano_dis[m[0], m[1]] > stParam.Vertical_threshoud:
            unmatched_detections.append(m[0])
            unmatched_trackers.append(m[1])
        else:
            matches.append(m.reshape(1, 2))

  # detections[:, 2] = detections
  if(len(matches)==0):
    matches = np.empty((0,2),dtype=int)
  else:
    matches = np.concatenate(matches,axis=0)
  return matches, np.array(unmatched_detections), np.array(unmatched_trackers), cost_matrix

class Sort(object):
  def __init__(self, left_center, medium_center, right_center, bmp_img, max_age=4,min_hits=2):
    """
    跟踪器的初始化
    Sets key parameters for SORT
    """
    self.max_age = max_age
    self.max_age_new = 30
    self.min_hits = min_hits
    self.trackers = []
    self.trackers_bak = []
    self.frame_count = 0
    #####*****判断是否利用位移计算航向角的阈值(暂时觉得速度作为判断条件不太靠谱，采用位移作为判断条件)自己看情况修改吧，可以适当大*****#####
    self.dis_thresh = 3
    # #####*****  角度差，将相邻帧的航向角的变化限制在正负self.angle_judge内  *****#####
    self.angle_judge = 10

    # #####*****  车道中心线信息  *****#####
    self.left_center = left_center
    self.medium_center = medium_center
    self.right_center = right_center

    # #####*****  车道BMP  *****#####
    self.bmp_img = bmp_img

  def update(self, dets, stParam):
      """
      Params:
        dets - a numpy array of detections in the format [[x1,y1,x2,y2,score],[x1,y1,x2,y2,score],...]
      Requires: this method must be called once for each frame even with empty detections.参数输入为
      x,y,l,角度，z坐标，高度
      line_k: 车道的斜率
      line_b: 车道的截距
      line_limit: 车道限制的截止坐标值
      line_angle: 车道对应的航向角
      Returns the a similar array, where the last column is the object ID.

      NOTE: The number of objects returned may diffkf_speed_xer from the number of detections provided.
      """
      self.max_age_new = stParam.MaxLifetime   #max_age MaxLifetime
      self.frame_count += 1
      trks = np.zeros((len(self.trackers), 7))
      to_del = []
      ret = []
      for t, trk in enumerate(trks):
          pos, bbox = self.trackers[t].predict()
          pos = pos[0]
          trk[:] = [pos[0], pos[1], bbox[2], bbox[3], bbox[4], self.trackers[t].label,
                    self.trackers[t].lane]
          if (np.any(np.isnan(pos))):
              to_del.append(t)
      trks = np.ma.compress_rows(np.ma.masked_invalid(trks))
      for t in reversed(to_del):
          self.trackers.pop(t)

      # #####*****  执行数据关联函数  *****#####
      matched, unmatched_dets, unmatched_trks, cost_matrix = associate_detections_to_trackers(dets, trks, 1, stParam)


      for t, trk in enumerate(
              self.trackers):  # if ((trk.time_since_update < self.max_age) and (trk.hits >= self.min_hits or self.frame_count <= self.min_hits)):
          if (t not in unmatched_trks):
              d = matched[np.where(matched[:, 1] == t)[0], 0]
              trk.update(dets[d, :][0])

      unmatched_trks.tolist()
      new_un_match_trks = []
      for my_num in range(len(unmatched_trks.tolist())):
          new_un_match_trks.append(self.trackers[unmatched_trks.tolist()[my_num]])

      # #####*****  没有匹配上的直接进入到距离匹配  *****#####
      un_match_object = dets[unmatched_dets.tolist(), :]
      un_match_track = trks[unmatched_trks.tolist(), :]
      matched_bak, unmatched_dets_bak, unmatched_trks_bak, cost_matrix = associate_detections_to_trackers(un_match_object, un_match_track, 0, stParam)

      # #####*****  新的匹配策略对应的目标提取方式  *****#####
      for t, trk in enumerate(new_un_match_trks):  # if ((trk.time_since_update < self.max_age) and (trk.hits >= self.min_hits or self.frame_count <= self.min_hits)):
          if (t not in unmatched_trks_bak):
              d = matched_bak[np.where(matched_bak[:, 1] == t)[0], 0]
              trk.update(un_match_object[d, :][0])
              # print('!!!!!!!')
              # print(un_match_object[d, :][0])
      for i in unmatched_dets_bak:
          if un_match_object[i, 10] < 2:
              trk = KalmanBoxTracker(un_match_object[i, :], self.left_center, self.medium_center, self.right_center, self.bmp_img)
              self.trackers.append(trk)

      num_tra = len(self.trackers)
      for trk in reversed(self.trackers):
          d, x_temp, trk_speed, trk_angle_box, center_angle = trk.get_state()
          d = d[0]
          head_final = trk.map_head_angle

          trk.state_list.append(d[0])
          trk.state_list.append(d[1])
          if len(trk.state_list) > 20:
              trk.state_list = trk.state_list[-20:]

          if trk.speed > 33.5:
              if len(trk.velocity_list) > 1:
                  trk.speed = trk.velocity_list[-1]
              else:
                  trk.speed = 33.5
          trk.velocity_list.append(trk.speed)
          if len(trk.velocity_list) > 10:
              trk.velocity_list = trk.velocity_list[-10:]
          trk.brief_velocity = np.mean(trk.velocity_list)
          # trk.brief_velocity = trk.velocity_list[-1]

          if trk.acc > 3.5:
              if len(trk.acc_list) > 1:
                  trk.acc = trk.acc_list[-1]
              else:
                  trk.acc = 3.5
          trk.acc_list.append(trk.acc)
          if len(trk.acc_list) > 10:
              trk.acc_list = trk.acc_list[-10:]
          trk.brief_acc = np.mean(trk.acc_list)
          # trk.brief_acc = trk.acc_list[-1]


          # state_info = np.array([[trk.kf.x[0][0], trk.kf.x[1][0], trk.kf.x[2][0], trk.kf.x[3][0], trk.bbox[4],
          #                         trk.bbox[5], trk.bbox[6], trk.lane]])
          # state_info = cal_box_lane(trk.bmp_img, state_info)
          # # #####*****  angle_flag, 1表示第7列是航向角, 0表示第5列是航向角  *****#####
          # angle_flag = 1
          # state_info = cal_map_angle(state_info, trk.left_center, trk.medium_center, trk.right_center, angle_flag)
          # trk.map_head_angle = state_info[0][4]
          # # trk.map_head_angle = state_info[0][4] - (trk.original_north - trk.reference_north)
          # trk.lane = state_info[0][-1]


          # if ((trk.time_since_update < self.max_age) and (
          #         trk.hits >= self.min_hits or self.frame_count <= self.min_hits)) or (
          #         (trk.time_since_update < self.max_age_new) and (trk.hits >= 10)):
          if trk.time_since_update < self.max_age_new and trk.hits >= self.min_hits:
              # if trk.time_since_update < self.max_age:
              if trk.time_since_update < self.max_age:
                  if trk.time_since_update > 0:
                      d_conv = [d[0], d[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                      ret.append(np.concatenate((d_conv, [trk.speed], [trk.id + 1], [x_temp[8]], [x_temp[9]],[x_temp[10]],
                                                 [x_temp[11]], [x_temp[12]], [x_temp[13]], [x_temp[14]], [x_temp[15]],[x_temp[16]],
                                                 [trk.time_since_update])).reshape(1, -1))
                  else:
                      d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], head_final, x_temp[5], x_temp[6], trk.label]
                      ret.append(np.concatenate((d_conv, [trk.speed], [trk.id + 1], [x_temp[8]], [x_temp[9]],[x_temp[10]],
                                                 [x_temp[11]], [x_temp[12]], [x_temp[13]], [x_temp[14]], [x_temp[15]],[x_temp[16]],
                                                 [trk.time_since_update])).reshape(1, -1))
                  # if trk.time_since_update > 0:
                  #     d_conv = [d[0], d[1], x_temp[2], x_temp[3], trk.map_head_angle, x_temp[5], x_temp[6], trk.label]
                  #     ret.append(np.concatenate((d_conv, [trk.lane], [trk.id+1], [x_temp[8]], [x_temp[-7]],[x_temp[-6]],
                  #                                [x_temp[-5]], [x_temp[-4]], [x_temp[-3]], [x_temp[-2]], [x_temp[-1]],
                  #                                [trk.time_since_update])).reshape(1, -1))
                  # else:
                  #     d_conv = [x_temp[0], x_temp[1], x_temp[2], x_temp[3], trk.map_head_angle, x_temp[5], x_temp[6], trk.label]
                  #     ret.append(np.concatenate((d_conv, [trk.lane], [trk.id+1], [x_temp[8]], [x_temp[-7]],[x_temp[-6]],
                  #                                [x_temp[-5]], [x_temp[-4]], [x_temp[-3]], [x_temp[-2]], [x_temp[-1]],
                  #                                [trk.time_since_update])).reshape(1, -1))
          num_tra -= 1
          # if trk.time_since_update > self.max_age_new or d[0] < -1350:
          if trk.time_since_update > self.max_age_new:
              self.trackers.pop(num_tra)
      if (len(ret) > 0):
          return np.concatenate(ret)
      return np.empty((0, 17))