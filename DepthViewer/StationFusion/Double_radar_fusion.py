# from Functions import TwoRadarFusion
import numpy as np
from sklearn.utils.linear_assignment_ import linear_assignment
import second.core.box_np_ops as box_np_ops
from StationFusion import IOU_Calculation
def rotate_nms_cc(dets,trackers, usespeed = 0):
    '''数据形式：x,y,l,w,vx,vy,theta'''
    trackers_corners = box_np_ops.center_to_corner_box2d(trackers[:, :2], trackers[:, 2:4], trackers[:, 6])
    trackers_standup = box_np_ops.corner_to_standup_nd(trackers_corners)
    dets_corners = box_np_ops.center_to_corner_box2d(dets[:, :2], dets[:, 2:4],dets[:, 6])
    dets_standup = box_np_ops.corner_to_standup_nd(dets_corners)
    standup_iou = box_np_ops.iou_jit(dets_standup, trackers_standup, eps=0.0)       #位置iou
    # print('standup_iou',standup_iou)
    # 速度的iou
    if usespeed == 1:
        trackers_boxlist = IOU_Calculation.point_to_box(trackers[:, 4:6], 0.4, 0.2)     #vx,vy,vx_width,vy_width
        dets_boxlist = IOU_Calculation.point_to_box(dets[:, 4:6], 0.4, 0.2)
        speed_iou = IOU_Calculation.iou_matrix(dets_boxlist, trackers_boxlist)
        # print('speed_iou', speed_iou)
        IOU_matrix = standup_iou * 0.8 + speed_iou * 0.2        #分配权重
        return IOU_matrix
    else:
        return standup_iou


def associate_detections_to_trackers(radar1, radar2,iou_threshold = 0.1):
  """
  Assigns detections to tracked o192.168.3.181bject (both represented as bounding boxes)
  Returns 3 lists of matches, unmatched_detections and unmatched_trackers
  返回三个np.array：匹配上的、没匹配上的目标、没匹配上的轨迹
  """
  if len(radar1) == 0:
      return np.empty((0,2),dtype=int), np.empty((0,7), dtype=int), np.arange(len(radar2))
  if len(radar2) == 0:
      return np.empty((0,2), dtype=int), np.arange(len(radar1)), np.empty((0,7), dtype=int)
  iou_matrix = rotate_nms_cc(radar1, radar2)
  # print('iou_matrix', iou_matrix)
  matched_indices = linear_assignment(-iou_matrix)
  # print('matched_indices', matched_indices)
  unmatched_radar1 = []
  for i, radar in enumerate(radar1):
      if i not in matched_indices[:,0]:
          unmatched_radar1.append(i)        #radar1第i个目标没匹配上
  unmatched_radar2 = []
  for j, radar in enumerate(radar2):
      if j not in matched_indices[:,1]:
          unmatched_radar2.append(j)
  matches = []
  for m in matched_indices:
      if iou_matrix[m[0],m[1]]<iou_threshold:  # 没匹配上
          unmatched_radar1.append(m[0])
          unmatched_radar2.append(m[1])
      else:
          matches.append(m.reshape(1,2))
  if (len(matches) == 0):
      matches = np.empty((0, 2), dtype=int)
  else:
      matches = np.concatenate(matches, axis=0)
  return matches, np.array(unmatched_radar1), np.array(unmatched_radar2)

class Tracker(object):
    def __init__(self, priradar, secradar):
        self.Age = 0
        self.ConfirmedNum = 0
        self.CenterDistance = priradar[1:4] - secradar[1:4]             # leng_dis,x_dis,y_dis
        l, x, y, vx, vy = (priradar[1:6] + secradar[1:6]) / 2  # 求均值
        self.heading = priradar[7]
        self.priradar_list = [priradar]     #记录主雷达的原始数据
        self.secradar_list = [secradar]     #记录副雷达的原始数据
        # self.box = []           # x,y,l,w
        # self.speed = []         # vx,vy
        self.track = [[x,y,l,3.0,vx,vy,self.heading]]
        self.state = priradar

    def update_trk(self, priradar, secradar,flag):
        if flag == 0:
        # if priradar and secradar:
            self.priradar_list.append(priradar)
            self.secradar_list.append(secradar)
            if len(self.priradar_list) > 5:
                self.priradar_list = self.priradar_list[-5:]
            if len(self.secradar_list) > 5:
                self.secradar_list = self.secradar_list[-5:]
            l,x,y,vx,vy = (priradar[1:6] + secradar[1:6]) / 2   #求均值
            heading = priradar[6]           #航向角用主雷达
            self.track.append([x,y,l,3.0,vx,vy,heading])

            if len(self.track) > 1:
                self.track = self.track[-1:]
            len_trk = len(self.track)
            # print('self.track', np.array(self.track), type(self.track))
            x_mean = sum(np.array(self.track)[:,0]) / len_trk
            y_mean = sum(np.array(self.track)[:,1]) / len_trk
            l_mean = sum(np.array(self.track)[:,2]) / len_trk
            self.state[1:4] = [l_mean, x_mean, y_mean]       #滑动平均
        elif flag == 1:  # 只有主雷达有数据
        # elif not secradar:  #只有主雷达有数据
            self.priradar_list.append(priradar)
            if len(self.priradar_list) > 5:
                self.priradar_list = self.priradar_list[-5:]
            l, x, y, vx, vy, heading = priradar[1:7]
            self.track.append([x, y, l, 3.0, vx, vy, heading])
            if len(self.track) > 1:
                self.track = self.track[-1:]
            len_trk = len(self.track)
            x_mean = sum(np.array(self.track)[:, 0]) / len_trk
            y_mean = sum(np.array(self.track)[:, 1]) / len_trk
            l_mean = sum(np.array(self.track)[:, 2]) / len_trk
            self.state[1:4] = [l_mean, x_mean, y_mean]  # 滑动平均
        elif flag == 2:  # 只有主雷达有数据
        # elif not priradar:  #只有副雷达有数据
            self.secradar_list.append(secradar)
            if len(self.secradar_list) > 5:
                self.secradar_list = self.secradar_list[-5:]
            l, x, y, vx, vy, heading = secradar[1:7]
            self.track.append([x, y, l, 3.0, vx, vy, heading])
            if len(self.track) > 1:
                self.track = self.track[-1:]
            len_trk = len(self.track)
            x_mean = sum(np.array(self.track)[:, 0]) / len_trk
            y_mean = sum(np.array(self.track)[:, 1]) / len_trk
            l_mean = sum(np.array(self.track)[:, 2]) / len_trk
            self.state[1:4] = [l_mean, x_mean, y_mean]  # 滑动平均

class Match_Tracker(object):
    def __init__(self):
        self.state = []
        self.max_age = 5
        self.matchtrack = {}
        self.track_pri2sec = {}
        self.track_sec2pri = {}
    # 匹配上的进行更新
    def update(self, matched, radar_pri, radar_sec):
        if len(radar_pri) !=0 and len(radar_sec) != 0:
            PriID = radar_pri[:,0]
            SecID = radar_sec[:,0]
        for i in matched:           #遍历匹配上的目标
            index_pri = i[0]
            index_sec = i[1]
            P_id = PriID[index_pri]
            S_id = SecID[index_sec]
            index = (P_id, S_id)    #匹配的id
            # 匹配的id对 没在self.matchtrack轨迹里面
            if self.matchtrack.get(index, 0) == 0:
                try:
                    p_index = [k[0] for k in self.matchtrack.keys()].index(P_id)
                    p_id, s_id = list(self.matchtrack.keys())[p_index]
                    del self.matchtrack[(p_id, s_id)]
                    del self.track_pri2sec[p_id]
                    del self.track_sec2pri[s_id]
                except:
                    pass
                try:
                    s_index = [k[1] for k in self.matchtrack.keys()].index(S_id)
                    p_id, s_id = list(self.matchtrack.keys())[s_index]
                    del self.matchtrack[(p_id, s_id)]
                    del self.track_pri2sec[p_id]
                    del self.track_sec2pri[s_id]
                except:
                    pass
                # print('index:', index)
                self.matchtrack[index] = Tracker(radar_pri[i[0]], radar_sec[i[1]])      #创建轨迹
                self.matchtrack[index].update_trk(radar_pri[i[0]], radar_sec[i[1]], 0)
            #匹配的id对在self.matchtrack轨迹里面，
            else:
                # print('in track')
                self.matchtrack[index].Age = 0
                self.matchtrack[index].ConfirmedNum += 1
                self.matchtrack[index].update_trk(radar_pri[i[0]], radar_sec[i[1]], 0)
            # 匹配上的放入track_pri2sec和track_sec2pri
            self.track_pri2sec[index[0]] = index[1]
            self.track_sec2pri[index[1]] = index[0]
        print('self.matchtrack', self.matchtrack)
        print('self.track_pri2sec', self.track_pri2sec)
        print('self.track_sec2pri', self.track_sec2pri)
        # 删除
        # delete = []
        # for i in self.matchtrack:
        #     print('self.matchtrack[i].Age', self.matchtrack[i].Age)
        #     self.matchtrack[i].Age += 1
        #     if self.matchtrack[i].Age > self.max_age:
        #         delete.append(i)
        # print('delate', delete)
        # for i in delete:
        #     del self.matchtrack[i]
        #     if self.track_pri2sec.get(i[0], -1) != -1:
        #         del self.track_pri2sec[i[0]]
        #     if self.track_sec2pri.get(i[1], -1) != -1:
        #         del self.track_sec2pri[i[1]]

    def get_unmatchid(self,matched, unmatched_pri, radar_pri, unmatched_sec, radar_sec, ):
        unmatched_pri_new = list(unmatched_pri)
        unmatched_sec_new = list(unmatched_sec)
        matched_new = list(matched)
        unmatch_pri = []
        unmatch_sec = []
        bflag = False
        nRmCount = 0
        for i in range(len(unmatched_pri)):
            for j in range(len(unmatched_sec)):
                pri_id, pri_src = radar_pri[unmatched_pri[i]][0], radar_pri[unmatched_pri[i]][-4]
                sec_id, sec_src = radar_sec[unmatched_sec[j]][0], radar_sec[unmatched_sec[j]][-4]
                # 这里处理的并不好
                if pri_src == 0 or sec_src == 0:
                    continue
                if self.matchtrack.get((pri_id, sec_id), -1) != -1 and self.track_pri2sec.get(pri_id, -1) != -1 and self.track_sec2pri.get(sec_id,-1) != -1:
                    if self.track_pri2sec[pri_id] == sec_id and self.track_sec2pri[sec_id] == pri_id:# 匹配上
                        # print('oooooooooooooo')
                        # print('pri_id,sec_id',pri_id, sec_id,np.array(radar_pri)[:,0], np.array(radar_sec)[:,0])
                        matched_new.append([unmatched_pri[i], unmatched_sec[j]])
                        unmatched_pri_new.pop(i - nRmCount)
                        unmatched_sec_new.pop(j - nRmCount)
                        nRmCount += 1

        # 再看其他没匹配上的
        for i in unmatched_pri_new:
            pri_id = radar_pri[i][0]
            if self.track_pri2sec.get(pri_id, -1) == -1 or self.matchtrack.get((pri_id, self.track_pri2sec[pri_id]),
                                                                               -1) == -1:
                unmatch_pri.append(i)
            else:
                self.matchtrack[(pri_id, self.track_pri2sec[pri_id])].ConfirmedNum += 1
                self.matchtrack[(pri_id, self.track_pri2sec[pri_id])].update_trk(radar_pri[i], 0, 1)   #只有主雷达有数据
        for j in unmatched_sec_new:
            sec_id = radar_sec[j][0]
            if self.track_sec2pri.get(sec_id, -1) == -1 or self.matchtrack.get((self.track_sec2pri[sec_id], sec_id),
                                                                               -1) == -1:
                unmatch_sec.append(j)
            else:
                self.matchtrack[(self.track_sec2pri[sec_id], sec_id)].ConfirmedNum += 1
                self.matchtrack[(self.track_sec2pri[sec_id], sec_id)].update_trk(0, radar_sec[j], 2)   #只有副雷达有数据

        return matched_new, unmatch_pri, unmatch_sec


    def Match_RR(self, radar_pri, radar_sec, matched, unmatched_pri, unmatched_sec):
        self.update(matched, radar_pri, radar_sec)      #更新
        result_data = []
        if len(radar_pri) !=0 and len(radar_sec) != 0:
            PriID = radar_pri[:,0]
            SecID = radar_sec[:,0]

        for trk in self.matchtrack:
            res = self.matchtrack[trk].state
            res[-9] = 11
            result_data.append(np.concatenate((res, [2])))
        for i in unmatched_pri:
            # P_id = PriID[i]
            result_data.append(np.concatenate((radar_pri[i], [0])))
        for i in unmatched_sec:
            result_data.append(np.concatenate((radar_sec[i], [1])))
        # print('result_data', result_data)
        self.delete()
        return result_data

    def delete(self):
        # 删除
        delete = []
        for i in self.matchtrack:
            # print('self.matchtrack[i].Age', self.matchtrack[i].Age)
            self.matchtrack[i].Age += 1
            if self.matchtrack[i].Age > self.max_age:
                delete.append(i)
        # print('delate', delete)
        for i in delete:
            del self.matchtrack[i]
            if self.track_pri2sec.get(i[0], -1) != -1:
                del self.track_pri2sec[i[0]]
            if self.track_sec2pri.get(i[1], -1) != -1:
                del self.track_sec2pri[i[1]]



class Fusion(object):
    def __init__(self, max_age=4):
        self.max_age = max_age
        self.tracker_pri = []
        self.tracker_sec = []
        self.tracker_fusion = []
        self.fusion_track = {}
        self.fusion_data = []       #输出匹配后的数据
        # 158-159
        # self.R = np.array([[-0.99921932, -0.03950628], [0.03950628, -0.99921932]])
        # self.t = np.array([215.41618313, -13.73957331])
        '''后厂村路'''
        self.R = np.array([[-0.99022542, 0.1394762], [-0.1394762, -0.99022542]])
        self.t = np.array([355.56001629, 35.75723811])
        self.RRmatch = Match_Tracker()
    def run(self, radar_1, radar_2):
        '''数据过来，先iou匹配'''
        # RRmatch = Match_Tracker()
        if radar_1.shape[0] == 0 and radar_2.shape[0] == 0: # 1和2都没数据
            return []
        else:
            radar_pri = radar_1[:, [10,4,0,1,11,11,6,2,3,5,7,8,9,12,13,14,15,16,17,18,19]]
            radar_sec = radar_2[:, [10,4,0,1,11,11,6,2,3,5,7,8,9,12,13,14,15,16,17,18,19]]
            # radar_sec = self.calibration(radar_sec)  # 转换到主雷达坐标系下
            # print('radar_pri', radar_pri)  # array
            # print('radar_sec', radar_sec)
            PriRiou = np.zeros((len(radar_pri), 7))
            SecRiou = np.zeros((len(radar_sec), 7))
            for i, radar in enumerate(PriRiou):
                radar[:] = [radar_pri[i][2], radar_pri[i][3], radar_pri[i][1], radar_pri[i][8], radar_pri[i][4], radar_pri[i][5], radar_pri[i][6]] #x,y,l,w,vx,vy,theta
            for i, radar in enumerate(SecRiou):
                radar[:] = [radar_sec[i][2], radar_sec[i][3], radar_sec[i][1], radar_sec[i][8], radar_sec[i][4], radar_sec[i][5], radar_sec[i][6]]
            # print('PriRiou', PriRiou)
            # print('SecRiou', SecRiou)
            matched, unmatched_pri, unmatched_sec = associate_detections_to_trackers(PriRiou, SecRiou)  # 关联匹配
            if len(radar_pri) != 0 and len(radar_sec) != 0:
                PriID = radar_pri[:, 0]
                SecID = radar_sec[:, 0]
            for i in matched:  # 遍历匹配上的目标
                index_pri = i[0]
                index_sec = i[1]
                P_id = PriID[index_pri]
                S_id = SecID[index_sec]
                index = (P_id, S_id)  # 匹配的id
                print(index)
            # print('matched', matched)
            # print('unmatched_pri', unmatched_pri)
            # print('unmatched_sec', unmatched_sec)
            # print('!!!!!!')
            # 获取新的没匹配的目标,在第一次的基础上再次匹配关联
            matched, unmatched_pri,unmatched_sec = self.RRmatch.get_unmatchid(matched, unmatched_pri, radar_pri, unmatched_sec, radar_sec)
            if len(radar_pri) != 0 and len(radar_sec) != 0:
                PriID = radar_pri[:, 0]
                SecID = radar_sec[:, 0]
            for i in matched:  # 遍历匹配上的目标
                index_pri = i[0]
                index_sec = i[1]
                P_id = PriID[index_pri]
                S_id = SecID[index_sec]
                index = (P_id, S_id)  # 匹配的id
                print(index)
            # print('matched_new', matched)
            # print('unmatched_pri_new', unmatched_pri)
            # print('unmatched_sec_new', unmatched_sec)
            # unmatched_pri = self.RRmatch.get_secid(unmatched_pri, radar_pri)     #找到确实没匹配上的
            # unmatched_sec = self.RRmatch.get_priid(unmatched_sec, radar_sec)
            result = self.RRmatch.Match_RR(radar_pri, radar_sec, matched, unmatched_pri, unmatched_sec)
        return result




    def output_match(self):
        return self.fusion_data



    def calibration(self, radar_sec):
        # 假如副雷达没有数据返回空列表
        if len(radar_sec) == 0:
            return []
        radar_sec = np.array(radar_sec)
        points = []
        speed = []
        for data in radar_sec:
            points.append([data[2], data[3]])       #x,y
            speed.append([data[4], data[5]])


        points = np.array(points)
        points_ =  np.matmul(points, self.R.T) + self.t.reshape([1, 2])
        speed_ = np.matmul(speed, self.R.T)
        for i in range(len(radar_sec)):
            radar_sec[i][2], radar_sec[i][3] = points_[i][0], points_[i][1]
            # radar_sec[i][4] = -radar_sec[i][4]
            radar_sec[i][4], radar_sec[i][5] =speed_[i][0], speed_[i][1]
        # 返回的是列表
        return radar_sec







# from Functions import TwoRadarFusion
# import numpy as np
# from sklearn.utils.linear_assignment_ import linear_assignment
# from Functions import box_np_ops
# from Functions import IOU_Calculation
# def rotate_nms_cc(dets,trackers, usespeed = 1):
#     '''数据形式：x,y,l,w,vx,vy,theta'''
#     trackers_corners = box_np_ops.center_to_corner_box2d(trackers[:, :2], trackers[:, 2:4], trackers[:, 6])
#     trackers_standup = box_np_ops.corner_to_standup_nd(trackers_corners)
#     dets_corners = box_np_ops.center_to_corner_box2d(dets[:, :2], dets[:, 2:4],dets[:, 6])
#     dets_standup = box_np_ops.corner_to_standup_nd(dets_corners)
#     standup_iou = box_np_ops.iou_jit(dets_standup, trackers_standup, eps=0.0)       #位置iou
#     print('standup_iou',standup_iou)
#     # 速度的iou
#     if usespeed == 1:
#         trackers_boxlist = IOU_Calculation.point_to_box(trackers[:, 4:6], 0.4, 0.2)     #vx,vy,vx_width,vy_width
#         dets_boxlist = IOU_Calculation.point_to_box(dets[:, 4:6], 0.4, 0.2)
#         speed_iou = IOU_Calculation.iou_matrix(dets_boxlist, trackers_boxlist)
#         IOU_matrix = standup_iou * 0.8 + speed_iou * 0.2
#         return IOU_matrix
#     else:
#         return standup_iou
#
#
# def associate_detections_to_trackers(radar1, radar2,iou_threshold = 0.1):
#   """
#   Assigns detections to tracked o192.168.3.181bject (both represented as bounding boxes)
#   Returns 3 lists of matches, unmatched_detections and unmatched_trackers
#   返回三个np.array：匹配上的、没匹配上的目标、没匹配上的轨迹
#   """
#   if len(radar1) == 0:
#       return np.empty((0,2),dtype=int), np.empty((0,7), dtype=int), np.arange(len(radar2))
#   if len(radar2) == 0:
#       return np.empty((0,2), dtype=int), np.arange(len(radar1)), np.empty((0,7), dtype=int)
#   iou_matrix = rotate_nms_cc(radar1, radar2)
#   print('iou_matrix', iou_matrix)
#   matched_indices = linear_assignment(-iou_matrix)
#   # print('matched_indices', matched_indices)
#   unmatched_radar1 = []
#   for i, radar in enumerate(radar1):
#       if i not in matched_indices[:,0]:
#           unmatched_radar1.append(i)        #radar1第i个目标没匹配上
#   unmatched_radar2 = []
#   for j, radar in enumerate(radar2):
#       if j not in matched_indices[:,1]:
#           unmatched_radar2.append(j)
#   matches = []
#   for m in matched_indices:
#       if iou_matrix[m[0],m[1]]<iou_threshold:  # 没匹配上
#           unmatched_radar1.append(m[0])
#           unmatched_radar2.append(m[1])
#       else:
#           matches.append(m.reshape(1,2))
#   if (len(matches) == 0):
#       matches = np.empty((0, 2), dtype=int)
#   else:
#       matches = np.concatenate(matches, axis=0)
#   return matches, np.array(unmatched_radar1), np.array(unmatched_radar2)
#
# class Tracker(object):
#     def __init__(self, priradar, secradar):
#         self.Age = 0
#         self.ConfirmedNum = 0
#         self.CenterDistance = priradar[1:4] - secradar[1:4]             # leng_dis,x_dis,y_dis
#         l, x, y, vx, vy = (priradar[1:6] + secradar[1:6]) / 2  # 求均值
#         self.heading = priradar[7]
#         self.priradar_list = [priradar]     #记录主雷达的原始数据
#         self.secradar_list = [secradar]     #记录副雷达的原始数据
#         # self.box = []           # x,y,l,w
#         # self.speed = []         # vx,vy
#         self.track = [[x,y,l,3.0,vx,vy,self.heading]]
#         self.state = priradar
#
#     def update_trk(self, priradar, secradar):
#         self.priradar_list.append(priradar)
#         self.secradar_list.append(secradar)
#         if len(self.priradar_list) > 5:
#             self.priradar_list = self.priradar_list[-5:]
#         if len(self.secradar_list) > 5:
#             self.secradar_list = self.secradar_list[-5:]
#         l,x,y,vx,vy = (priradar[1:6] + secradar[1:6]) / 2   #求均值
#         heading = priradar[7]           #航向角用主雷达
#         self.track.append([x,y,l,3.0,vx,vy,heading])
#
#         if len(self.track) > 5:
#             self.track = self.track[-5:]
#         len_trk = len(self.track)
#         # print('self.track', np.array(self.track), type(self.track))
#         x_mean = sum(np.array(self.track)[:,0]) / len_trk
#         y_mean = sum(np.array(self.track)[:,1]) / len_trk
#         l_mean = sum(np.array(self.track)[:,2]) / len_trk
#         self.state[1:4] = [l_mean, x_mean, y_mean]       #滑动平均
#
#
#
#
# class Match_Tracker(object):
#     def __init__(self):
#         self.state = []
#         self.max_age = 5
#         self.matchtrack = {}
#         self.track_pri2sec = {}
#         self.track_sec2pri = {}
#     # 匹配上的进行更新
#     def update(self, matched, radar_pri, radar_sec):
#         if len(radar_pri) !=0 and len(radar_sec) != 0:
#             PriID = radar_pri[:,0]
#             SecID = radar_sec[:,0]
#         for i in matched:
#             index_pri = i[0]
#             index_sec = i[1]
#             P_id = PriID[index_pri]
#             S_id = SecID[index_sec]
#             index = (P_id, S_id)    #匹配的id
#             # 匹配的id对 没在self.matchtrack轨迹里面
#             if self.matchtrack.get(index, 0) == 0:
#                 print('index:', index)
#                 self.matchtrack[index] = Tracker(radar_pri[i[0]], radar_sec[i[1]])      #轨迹
#                 self.matchtrack[index].update_trk(radar_pri[i[0]], radar_sec[i[1]])
#             #匹配的id对在self.matchtrack轨迹里面，
#             else:
#                 print('in track')
#                 self.matchtrack[index].Age = 0
#                 self.matchtrack[index].ConfirmedNum += 1
#                 self.matchtrack[index].update_trk(radar_pri[i[0]], radar_sec[i[1]])
#
#             self.track_pri2sec[index[0]] = index[1]
#             self.track_sec2pri[index[1]] = index[0]
#         print('self.matchtrack', self.matchtrack)
#         print('self.track_pri2sec', self.track_pri2sec)
#         print('self.track_sec2pri', self.track_sec2pri)
#         # 删除
#         # delete = []
#         # for i in self.matchtrack:
#         #     print('self.matchtrack[i].Age', self.matchtrack[i].Age)
#         #     self.matchtrack[i].Age += 1
#         #     if self.matchtrack[i].Age > self.max_age:
#         #         delete.append(i)
#         # print('delate', delete)
#         # for i in delete:
#         #     del self.matchtrack[i]
#         #     if self.track_pri2sec.get(i[0], -1) != -1:
#         #         del self.track_pri2sec[i[0]]
#         #     if self.track_sec2pri.get(i[1], -1) != -1:
#         #         del self.track_sec2pri[i[1]]
#     def get_secid(self, unmatched_pri, radar_pri):
#         unmatch = []
#         for i in unmatched_pri:
#             pri_id = radar_pri[i][0]
#             if self.track_pri2sec.get(pri_id,-1) == -1 or self.matchtrack.get((pri_id, self.track_pri2sec[pri_id]), -1) == -1:
#                 unmatch.append(i)
#             else:
#                 # self.matchtrack[(pri_id, self.track_pri2sec[pri_id])].Age = 0
#                 self.matchtrack[(pri_id, self.track_pri2sec[pri_id])].ConfirmedNum += 1
#                 # 当前帧只有一个雷达有数据， 轨迹的状态如何更新
#                 # 获取前一次的副雷达数据
#                 radar_sec = self.matchtrack[(pri_id, self.track_pri2sec[pri_id])].secradar_list[-1]
#                 self.matchtrack[(pri_id, self.track_pri2sec[pri_id])].update_trk(radar_pri[i],radar_sec)
#         print('get_secid', unmatch)
#         return unmatch      #返回没匹配上的
#
#     def get_priid(self, unmatched_sec, radar_sec):
#         unmatch = []
#         for i in unmatched_sec:
#             sec_id = radar_sec[i][0]
#             if self.track_sec2pri.get(sec_id, -1) == -1 or self.matchtrack.get((self.track_sec2pri[sec_id], sec_id), -1) == -1:
#                 unmatch.append(i)
#             else:
#                 # self.matchtrack[(self.track_sec2pri[sec_id], sec_id)].Age = 0
#                 self.matchtrack[(self.track_sec2pri[sec_id], sec_id)].ConfirmedNum += 1
#                 radar_pri = self.matchtrack[(self.track_sec2pri[sec_id], sec_id)].priradar_list[-1]
#                 self.matchtrack[(self.track_sec2pri[sec_id], sec_id)].update_trk(radar_pri, radar_sec[i])
#         print('get_priid', unmatch)
#         return unmatch
#     # def Match_
#     def Match_RR(self, radar_pri, radar_sec, matched, unmatched_pri, unmatched_sec):
#         self.update(matched, radar_pri, radar_sec)      #更新
#         result_data = []
#         if len(radar_pri) !=0 and len(radar_sec) != 0:
#             PriID = radar_pri[:,0]
#             SecID = radar_sec[:,0]
#         # 这个函数输出当前帧目标结果
#         # for i in matched:
#         #     index_pri = i[0]
#         #     index_sec = i[1]
#         #     P_id = PriID[index_pri]
#         #     S_id = SecID[index_sec]
#         #     index = (P_id, S_id)  # 匹配的id
#         #     if self.matchtrack.get(index, -1) == -1:
#         #         print('应该不可能进入这！！！')
#         #         continue
#         #     else:
#         #
#         #         result_data.append(np.concatenate((self.matchtrack[index].state, [2])))
#         for trk in self.matchtrack:
#             result_data.append(np.concatenate((self.matchtrack[trk].state, [2])))
#         for i in unmatched_pri:
#             # P_id = PriID[i]
#             result_data.append(np.concatenate((radar_pri[i], [0])))
#         for i in unmatched_sec:
#             result_data.append(np.concatenate((radar_sec[i], [1])))
#         print('result_data', result_data)
#         self.delete()
#         return result_data
#     def delete(self):
#         # 删除
#         delete = []
#         for i in self.matchtrack:
#             print('self.matchtrack[i].Age', self.matchtrack[i].Age)
#             self.matchtrack[i].Age += 1
#             if self.matchtrack[i].Age > self.max_age:
#                 delete.append(i)
#         print('delate', delete)
#         for i in delete:
#             del self.matchtrack[i]
#             if self.track_pri2sec.get(i[0], -1) != -1:
#                 del self.track_pri2sec[i[0]]
#             if self.track_sec2pri.get(i[1], -1) != -1:
#                 del self.track_sec2pri[i[1]]
#
#
#
# class Fusion(object):
#     def __init__(self, max_age=4):
#         self.max_age = max_age
#         self.tracker_pri = []
#         self.tracker_sec = []
#         self.tracker_fusion = []
#         self.fusion_track = {}
#         self.fusion_data = []       #输出匹配后的数据
#         # 158-159
#         self.R = np.array([[-0.99921932, -0.03950628], [0.03950628, -0.99921932]])
#         self.t = np.array([215.41618313, -13.73957331])
#         self.RRmatch = Match_Tracker()
#     def run(self, radar_pri, radar_sec):
#         '''数据过来，先iou匹配'''
#         # RRmatch = Match_Tracker()
#         if not radar_pri and not radar_sec: # 1和2都没数据
#             return []
#         else:
#             radar_pri = np.array(radar_pri)
#             radar_sec = np.array(radar_sec)
#             print('radar_pri', radar_pri)  # array
#             print('radar_sec', radar_sec)
#
#             radar_sec = self.calibration(radar_sec)  # 转换到主雷达坐标系下
#             PriRiou = np.zeros((len(radar_pri), 7))
#             SecRiou = np.zeros((len(radar_sec), 7))
#             for i, radar in enumerate(PriRiou):
#                 radar[:] = [radar_pri[i][2], radar_pri[i][3], radar_pri[i][1], 3.0, radar_pri[i][4], radar_pri[i][5], radar_pri[i][6]] #x,y,l,w,vx,vy,theta
#             for i, radar in enumerate(SecRiou):
#                 radar[:] = [radar_sec[i][2], radar_sec[i][3], radar_sec[i][1], 3.0, radar_sec[i][4], radar_sec[i][5], radar_sec[i][6]]
#             print('PriRiou', PriRiou)
#             print('SecRiou', SecRiou)
#             matched, unmatched_pri, unmatched_sec = associate_detections_to_trackers(PriRiou, SecRiou)  # 关联匹配
#             print('matched', matched)
#             print('unmatched_pri', unmatched_pri)
#             print('unmatched_sec', unmatched_sec)
#
#             unmatched_pri = self.RRmatch.get_secid(unmatched_pri, radar_pri)     #找到确实没匹配上的
#             unmatched_sec = self.RRmatch.get_priid(unmatched_sec, radar_sec)
#             result = self.RRmatch.Match_RR(radar_pri, radar_sec, matched, unmatched_pri, unmatched_sec)
#         return result
#
#
#
#
#     def output_match(self):
#         return self.fusion_data
#
#             # for match in matched:
#             #     radar_pri[match[0]]
#             #     self.fusion_track[]
#             # self.track['']
#             # for
#             # for i in unmatched_pri:
#
#
#
#     def calibration(self, radar_sec):
#         # 假如副雷达没有数据返回空列表
#         if len(radar_sec) == 0:
#             return []
#         radar_sec = np.array(radar_sec)
#         points = []
#         for data in radar_sec:
#             points.append([data[2], data[3]])       #x,y
#
#         points = np.array(points)
#         points_ =  np.matmul(points, self.R.T) + self.t.reshape([1, 2])
#         for i in range(len(radar_sec)):
#             radar_sec[i][2], radar_sec[i][3] = points_[i][0], points_[i][1]
#             radar_sec[i][4] = -radar_sec[i][4]
#         # 返回的是列表
#         return radar_sec
#
