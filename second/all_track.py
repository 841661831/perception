import open3d as o3d
import numpy as np
import second.core.box_np_ops as box_np_ops
from shapely.geometry import Polygon
import second.cal_iou as cal_iou
import operator
from sklearn.utils.linear_assignment_ import linear_assignment
import math
from numba import jit
import cv2
import time

def cal_lateral_dis(detections, trackers, flag):
    # #####*****  批量计算距离  *****#####
    det_xy = detections[:, :2].reshape(-1, 2)
    tra_xy = trackers[:, :2].reshape(-1, 2)
    asso_vector_x = (det_xy[:, 0].reshape(-1, 1) - tra_xy[:, 0].reshape(1, -1))
    asso_vector_y = (det_xy[:, 1].reshape(-1, 1) - tra_xy[:, 1].reshape(1, -1))
    # square_dis = np.sqrt(np.square(asso_vector_x) + np.square(asso_vector_y))
    square_dis = 0
    if flag == 1:
        theta = (trackers[:, 6]).reshape(1, -1)
    else:
        theta = trackers[:, 4].reshape(1, -1)
    theta = (-1 * theta - 90) * np.pi / 180

    head_vector_x = np.cos(theta[0, :]).reshape(1, -1)
    head_vector_y = np.sin(theta[0, :]).reshape(1, -1)

    new_head_vector_x = np.tile(head_vector_x, (asso_vector_x.shape[0], 1))
    new_head_vector_y = np.tile(head_vector_y, (asso_vector_y.shape[0], 1))

    # #####*****  head_dis_new是沿着车航向的距离  *****#####
    # #####*****  ano_dis_new是垂直航向的距离  *****#####
    head_dis_new = np.abs(asso_vector_x * new_head_vector_x + asso_vector_y * new_head_vector_y)
    ano_dis_new = np.abs(asso_vector_x * new_head_vector_y - asso_vector_y * new_head_vector_x)
    # ano_dis_new = np.abs(np.sqrt(square_dis - np.square(head_dis_new)))

    return head_dis_new, ano_dis_new, square_dis


def intersection(g, p):
    g = np.asarray(g)
    p = np.asarray(p)
    g = Polygon(g[:8].reshape((4, 2)))
    p = Polygon(p[:8].reshape((4, 2)))
    if not g.is_valid or not p.is_valid:
        return 0
    inter = Polygon(g).intersection(Polygon(p)).area
    union = g.area + p.area - inter
    if union == 0:
        return 0
    else:
        return inter / union

# #####*****  输入的航向角信息是角度制  *****#####
def get_min_max(trackers_cov, width_coe):
    trackers_cov[:, 3] *= width_coe
    trackers_cov_corner = box_np_ops.center_to_corner_box2d(trackers_cov[:, :2], trackers_cov[:, 3:5],
                                                    trackers_cov[:, 6] * np.pi / 180)
    # min_x_and_y_sta = np.min(trackers_cov_corner, axis=1)
    # max_x_and_y_sta = np.max(trackers_cov_corner, axis=1)
    # add_dimension_sta = np.concatenate((min_x_and_y_sta, max_x_and_y_sta), axis=1)
    # trackers_cov = np.hstack((trackers_cov, add_dimension_sta))
    trackers_cov[:, 3] = trackers_cov[:, 3] / width_coe

    return trackers_cov, trackers_cov_corner

# #####*****  新的计算倾斜框IOU的函数  *****#####
# #####*****  输入参数  *****#####
# #####*****  dets_corners 是检测目标的二维角点信息  *****#####
# #####*****  track_corners 是跟踪轨迹的二维角点信息  *****#####
# #####*****  flag 是标志位,表示检测和轨迹是否一样，主要是为了区别应用于跟踪和检测的IOU计算  *****#####
# #####*****  返回值是IOU矩阵  *****#####
def get_iou(dets_corners, track_corners, flag):
    if flag == 1:
        dets_standup = box_np_ops.corner_to_standup_nd(dets_corners)
        track_standup = box_np_ops.corner_to_standup_nd(track_corners)
    else:
        dets_standup = box_np_ops.corner_to_standup_nd(dets_corners)
        track_standup = dets_standup
    standup_iou = box_np_ops.iou_jit(dets_standup, track_standup, eps=0.0)
    standup_iou_new = standup_iou
    if flag != 1:
        for i in range(standup_iou.shape[0]):
            standup_iou[i][i] = 0
    re_cal = np.where(standup_iou > 0)
    no_care = []
    if re_cal[0].shape[0] == 0:
        return standup_iou, standup_iou_new
    for i in range(re_cal[0].shape[0]):
        if re_cal[0][i] == re_cal[1][i]:
            continue
        if re_cal[0][i] in no_care:
            continue
        # # #####*****  调用模块进行IOU计算  *****#####
        # IOU = intersection(dets_corners[re_cal[0][i]].tolist(), trackers_corners[re_cal[1][i]].tolist())
        # #####*****  调用自定义函数进行IOU计算耗时稍微小一些  *****#####
        interpoly = []
        b = cal_iou.PolygonClip(dets_corners[re_cal[0][i]].tolist(), track_corners[re_cal[1][i]].tolist(), interpoly)
        if b:
            list2 = []
            for num_i in range(len(interpoly)):
                list2.append(interpoly[num_i])
            for num_i in range(len(list2) - 1):
                for num_j in range(num_i + 1, len(list2)):
                    if operator.eq(list2[num_i], list2[num_j]):
                        interpoly.pop(num_j)
            area = cal_iou.CalArea(interpoly)
            area1 = cal_iou.CalArea(dets_corners[re_cal[0][i]].tolist())
            area2 = cal_iou.CalArea(track_corners[re_cal[1][i]].tolist())
            IOU = area / (area2 + area1 - area)
            IOU_new = area / area1
        else:
            IOU = 0
            IOU_new = 0
        no_care.append(re_cal[1][i])
        if flag:
            standup_iou[re_cal[0][i]][re_cal[1][i]] = IOU
            standup_iou_new[re_cal[0][i]][re_cal[1][i]] = IOU_new
        else:
            standup_iou[re_cal[0][i]][re_cal[1][i]] = IOU
            standup_iou[re_cal[1][i]][re_cal[0][i]] = IOU
            standup_iou_new[re_cal[0][i]][re_cal[1][i]] = IOU_new
            standup_iou_new[re_cal[1][i]][re_cal[0][i]] = IOU_new
    return standup_iou, standup_iou_new



def fusion_box(dets_corners, track_corners, flag):
    if flag == 1:
        dets_standup = box_np_ops.corner_to_standup_nd(dets_corners)
        track_standup = box_np_ops.corner_to_standup_nd(track_corners)
    else:
        dets_standup = box_np_ops.corner_to_standup_nd(dets_corners)
        track_standup = dets_standup
    standup_iou = box_np_ops.iou_jit(dets_standup, track_standup, eps=0.0)
    if flag != 1:
        for i in range(standup_iou.shape[0]):
            standup_iou[i][i] = 0
    re_cal = np.where(standup_iou > 0)
    no_care = []
    if re_cal[0].shape[0] == 0:
        return np.empty((0, 2), dtype=int), np.arange(len(dets_corners)), np.empty((0, 5), dtype=int), 0
    for i in range(re_cal[0].shape[0]):
        if re_cal[0][i] == re_cal[1][i]:
            continue
        if re_cal[0][i] in no_care:
            continue
        # # #####*****  调用模块进行IOU计算  *****#####
        # IOU = intersection(dets_corners[re_cal[0][i]].tolist(), trackers_corners[re_cal[1][i]].tolist())
        # #####*****  调用自定义函数进行IOU计算耗时稍微小一些  *****#####
        interpoly = []
        b = cal_iou.PolygonClip(dets_corners[re_cal[0][i]].tolist(), track_corners[re_cal[1][i]].tolist(), interpoly)
        if b:
            list2 = []
            for num_i in range(len(interpoly)):
                list2.append(interpoly[num_i])
            for num_i in range(len(list2) - 1):
                for num_j in range(num_i + 1, len(list2)):
                    if operator.eq(list2[num_i], list2[num_j]):
                        interpoly.pop(num_j)
            area = cal_iou.CalArea(interpoly)
            area1 = cal_iou.CalArea(dets_corners[re_cal[0][i]].tolist())
            area2 = cal_iou.CalArea(track_corners[re_cal[1][i]].tolist())
            IOU = area / (area2 + area1 - area)
        else:
            IOU = 0
        no_care.append(re_cal[1][i])
        if flag:
            standup_iou[re_cal[0][i]][re_cal[1][i]] = IOU
        else:
            standup_iou[re_cal[0][i]][re_cal[1][i]] = IOU
            standup_iou[re_cal[1][i]][re_cal[0][i]] = IOU

    iou_threshold = 0.000001
    matched_indices = linear_assignment(-standup_iou)
    unmatched_detections = []
    for d, det in enumerate(dets_corners):
        if (d not in matched_indices[:, 0]):
            unmatched_detections.append(d)
    unmatched_trackers = []
    for t, trk in enumerate(track_corners):
        if (t not in matched_indices[:, 1]):
            unmatched_trackers.append(t)
    matches = []
    for m in matched_indices:
        if standup_iou[m[0], m[1]] < iou_threshold:
            unmatched_detections.append(m[0])
            unmatched_trackers.append(m[1])
        else:
            matches.append(m.reshape(1, 2))
    if (len(matches) == 0):
        matches = np.empty((0, 2), dtype=int)
    else:
        matches = np.concatenate(matches, axis=0)

    return matches, np.array(unmatched_detections), np.array(unmatched_trackers), standup_iou

def cal_trans(x, y, z):
    R_x = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(x), -1 * math.sin(x)], [0.0, math.sin(x), math.cos(x)]])
    R_y = np.array([[math.cos(y), 0.0, math.sin(y)], [0.0, 1.0, 0.0], [-1 * math.sin(y), 0.0, math.cos(y)]])
    R_z = np.array([[math.cos(z), -1 * math.sin(z), 0.0], [math.sin(z), math.cos(z), 0.0], [0.0, 0.0, 1.0]])
    rotate = np.dot(R_z, R_y)
    rotate = np.dot(rotate, R_x)
    return rotate

# #####*****  box的前三项是目标的质心  *****#####
def rotate_box(in_points, box, angle_x, angle_y, angle_z, trans_x, trans_y, trans_z):
    rotate = cal_trans(angle_x, angle_y, angle_z)
    A = np.dot(rotate, in_points[:, :3].T)
    in_points[:, :3] = A.T
    in_points[:, 0] += trans_x
    in_points[:, 1] += trans_y
    in_points[:, 2] += trans_z

    B = np.dot(rotate, box[:, :3].T)
    box[:, :3] = B.T
    box[:, 0] += trans_x
    box[:, 1] += trans_y
    box[:, 2] += trans_z

    return in_points, box

@jit(nopython=True)
def fix_angle(boxes_lidar, angle1, angle2, angle3):
    for num in range(boxes_lidar.shape[0]):
        if boxes_lidar[num][0] < -40:
            boxes_lidar[num][6] = angle1*np.pi/180
        elif boxes_lidar[num][0] > 40:
            boxes_lidar[num][6] = angle2*np.pi/180
        else:
            boxes_lidar[num][6] = angle3*np.pi/180

    return boxes_lidar

# #####*****  single station  *****#####
def get_pixel(center_x, center_y):
    img_corner = 2000//2
    dist2pixel = 2000 // 200
    tempx = round(center_x * dist2pixel)
    tempy = round(center_y * dist2pixel)
    if (tempx >= img_corner) or (tempy >= img_corner) or (tempx <= -img_corner) or (tempy <= -img_corner):
        tempx = -1000
        tempy = -1000
    tempx = int(img_corner - 1 + tempx)
    tempy = int(img_corner - 1 - tempy)

    return tempx, tempy

def cache_bmp(img):
    img_convert = np.ones_like(img)

    (height, width, color) = img.shape
    for y in range(height):
        conver_y = height - y - 1
        img_convert[y, :, :] = img[conver_y, :, :]
    return img_convert


# #####*****  如果使用车道bmp进行车道判断时候需要传入bmp_img  *****#####
def fix_cluster(box, flag):
    # row_x = box[:, 0].reshape(-1, 1)
    # column_x = box[:, 0].reshape(1, -1)
    # row_y = box[:, 1].reshape(-1, 1)
    # column_y = box[:, 1].reshape(1, -1)
    #
    # dis_x_matrix = np.abs(row_x - column_x)
    # dis_y_matrix = np.abs(row_y - column_y)
    # np.fill_diagonal(dis_x_matrix, 10000)
    # np.fill_diagonal(dis_y_matrix, 10000)
    head_dis, ano_dis, square_dis = cal_lateral_dis(box, box, flag)
    np.fill_diagonal(head_dis, 10000)
    # # # #####*****  利用二分配的方法  *****#####
    # matched_indices = linear_assignment(head_dis)
    # #####*****  利用每行求最小值的方法去除重复框或者距离近的框  *****#####
    min_indice = np.argmin(head_dis, axis=1)
    save_indice = []
    delete_indice = []

    for i in range(min_indice.shape[0]):
        cluster1 = box[i]
        cluster2 = box[min_indice[i]]
        cluster1_l = cluster1[3]
        cluster2_l = cluster2[3]
        delta_y = ano_dis[i][min_indice[i]]
        if i in delete_indice:
            continue
        if delta_y > 2:
            save_indice.append(i)
            continue
        else:
            delta_x = head_dis[i][min_indice[i]]
            delta_l = abs(cluster1_l + cluster2_l) * 0.5
            save_indice.append(i)
            if delta_x < (delta_l + 1.5):
                # if delta_x * 2 >= 0.5*(box[i][4] + box[min_indice[i]][4]):
                #     box[i][4] = delta_x * 2
                #     box[i][0] = 0.5*(box[i][0] + box[min_indice[i]][0])
                #     box[i][1] = 0.5 * (
                #                 box[i][1] + box[min_indice[i]][1])
                # else:
                #     box[i][4] = 0.5*(box[i][4] + box[min_indice[i]][4]) + delta_x * 0.5
                #     box[i][0] = 0.5 * (
                #                 box[i][0] + box[min_indice[i]][0])
                #     box[i][1] = 0.5 * (
                #             box[i][1] + box[min_indice[i]][1])
                delete_indice.append(min_indice[i])

    # for i in range(matched_indices.shape[0]):
    #     if matched_indices[i][0] in delete_indice:
    #         continue
    #     cluster1 = box[matched_indices[i][0]]
    #     cluster2 = box[matched_indices[i][1]]
    #     cluster1_x = cluster1[0]
    #     cluster1_y = cluster1[1]
    #     cluster1_l = cluster1[4]
    #     cluster2_x = cluster2[0]
    #     cluster2_y = cluster2[1]
    #     cluster2_l = cluster2[4]
    #     # temp1_x, temp1_y = get_pixel(cluster1_x, cluster1_y)
    #     # temp2_x, temp2_y = get_pixel(cluster2_x, cluster2_y)
    #     # stand_1 = bmp_img[temp1_x][temp1_y][0] + 2 * bmp_img[temp1_x][temp1_y][1] + 4 * bmp_img[temp1_x][temp1_y][2]
    #     # stand_2 = bmp_img[temp2_x][temp2_y][0] + 2 * bmp_img[temp2_x][temp2_y][1] + 4 * bmp_img[temp2_x][temp2_y][2]
    #     # #####*****  delta_y是垂直航向的垂直距离  *****#####
    #     delta_y = ano_dis[matched_indices[i][0]][matched_indices[i][1]]
    #     # delta_y = abs(cluster2_y - cluster1_y)
    #     # if stand_1 != stand_2:
    #     if delta_y > 2:
    #         save_indice.append(matched_indices[i][0])
    #         continue
    #     else:
    #         # delta_x = abs(cluster1_x - cluster2_x)
    #         delta_x = head_dis[matched_indices[i][0]][matched_indices[i][1]]
    #         delta_l = abs(cluster1_l + cluster2_l) * 0.5
    #         save_indice.append(matched_indices[i][0])
    #         if delta_x < (delta_l + 1.5):
    #             if delta_x * 2 >= 0.5*(box[matched_indices[i][0]][4] + box[matched_indices[i][1]][4]):
    #                 box[matched_indices[i][0]][4] = delta_x * 2
    #                 box[matched_indices[i][0]][0] = 0.5*(box[matched_indices[i][0]][0] + box[matched_indices[i][1]][0])
    #                 box[matched_indices[i][0]][1] = 0.5 * (
    #                             box[matched_indices[i][0]][1] + box[matched_indices[i][1]][1])
    #             else:
    #                 box[matched_indices[i][0]][4] = 0.5*(box[matched_indices[i][0]][4] + box[matched_indices[i][1]][4]) + delta_x * 0.5
    #                 box[matched_indices[i][0]][0] = 0.5 * (
    #                             box[matched_indices[i][0]][0] + box[matched_indices[i][1]][0])
    #                 box[matched_indices[i][0]][1] = 0.5 * (
    #                         box[matched_indices[i][0]][1] + box[matched_indices[i][1]][1])
    #             delete_indice.append(matched_indices[i][1])
    # save_indice = [i for i in save_indice if i not in delete_indice]
    new_box = box[save_indice, :]
    # print(save_indice)
    # print(delete_indice)
    return new_box

# #####*****  计算目标对应在车道图上的索引 *****#####
def get_pixel_location(center_x, center_y):
    dist2pixel = 10
    # # #####*****  广州祈福隧道参数  *****#####
    # img_corner_x = 14700
    # img_corner_y = 10900
    # tempx = round((center_x + 1350) * dist2pixel) - 1
    # tempy = round((center_y - 1040) * dist2pixel) * (-1) - 1

    # #####*****  秦岭隧道参数  *****#####
    img_corner_x = 16000
    img_corner_y = 5000
    tempx = round((center_x + 1570) * dist2pixel) - 1
    tempy = round((center_y - 450) * dist2pixel) * (-1) - 1

    # tempy = round((1200 - center_y) * dist2pixel) - 1
    if (tempx >= img_corner_x) or (tempy >= img_corner_y) or (tempx < 0) or (tempy < 0):
        tempx = -1000
        tempy = -1000
    # #####*****  广州祈福隧道参数  *****#####
    # return int(tempx * 0.79), int(tempy * 0.79)
    #####*****  秦岭隧道参数  *****#####
    return int(tempx), int(tempy)

# # #####*****  计算目标对应在车道图上的索引 *****#####
# def get_pixel_location(center_x, center_y):
#     dist2pixel = 10
#     img_corner_x = 14700
#     img_corner_y = 10900
#     tempx = round((center_x + 1350) * dist2pixel) - 1
#     tempy = round((center_y - 1040) * dist2pixel) * (-1) - 1
#     # tempy = round((1200 - center_y) * dist2pixel) - 1
#     if (tempx >= img_corner_x) or (tempy >= img_corner_y) or (tempx < 0) or (tempy < 0):
#         tempx = -1000
#         tempy = -1000
#     return int(tempx * 0.79), int(tempy * 0.79)


# #####*****  计算目标信息对应的车道  *****#####
def cal_box_lane(bmp_img, box_final):
    for i in range(box_final.shape[0]):
        center_x = box_final[i][0]
        center_y = box_final[i][1]
        temp_x, temp_y = get_pixel_location(center_x, center_y)
        # #####*****  广州祈福隧道参数  *****#####
        # if temp_x > -1 and temp_y > -1:
        #     if bmp_img[temp_y][temp_x][2] == 178:
        #         lane = 1
        #     elif bmp_img[temp_y][temp_x][2] == 126:
        #         lane = 2
        #     elif bmp_img[temp_y][temp_x][2] == 67:
        #         lane = 3
        #     else:
        #         lane = 0
        # else:
        #     lane = 0

        # #####*****  秦岭隧道参数  *****#####
        if temp_x > -1 and temp_y > -1:
            if bmp_img[temp_y][temp_x][0] == 255:
                lane = 1
            elif bmp_img[temp_y][temp_x][0] == 128:
                lane = 2
            else:
                lane = 0
        else:
            lane = 0

        box_final[i][-1] = lane
    return box_final

# #####*****  根据高精度地图矢量线计算目标的航向角  *****#####
def cal_map_angle(box_final, left_center, medium_center, right_center, flag):
    center_line = None
    for i in range(box_final.shape[0]):
        if box_final[i][-1] == 1:
            center_line = left_center
        elif box_final[i][-1] == 2:
            center_line = medium_center
        elif box_final[i][-1] == 3:
            center_line = right_center
        else:
            center_line = None
        if center_line is not None:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(center_line[:, :3])
            k = 1
            pcd_tree = o3d.geometry.KDTreeFlann(pcd)
            center = np.array([box_final[i][0], box_final[i][1], 0])
            [k, idx2, dis] = pcd_tree.search_knn_vector_3d(center, k)
            low_indice = idx2[0] - 5
            hight_indice = idx2[0] + 5
            if low_indice < 0:
                theta = math.atan2((center_line[idx2[0] + 10][1] - center_line[idx2[0]][1]),
                                   (center_line[idx2[0] + 10][0] - center_line[idx2[0]][0])) * 180 / np.pi
            elif hight_indice > (center_line.shape[0] - 1):
                theta = math.atan2((center_line[idx2[0]][1] - center_line[idx2[0] - 10][1]),
                                   (center_line[idx2[0]][0] - center_line[idx2[0] - 10][0])) * 180 / np.pi
            else:
                theta = math.atan2((center_line[idx2[0] + 5][1] - center_line[idx2[0] - 5][1]),
                                   (center_line[idx2[0] + 5][0] - center_line[idx2[0] - 5][0])) * 180 / np.pi
            theta = ((-1 * theta - 90) + 180) % 360
            if flag:
                box_final[i][6] = theta
            else:
                box_final[i][4] = theta

    return box_final



def find_angle(box_info, lane_with_angle, flag):
    for i in range(box_info.shape[0]):
        center_x = box_info[i][0]
        center_y = box_info[i][1]
        temp_x, temp_y = get_pixel_location(center_x, center_y)
        if temp_x > -1 and temp_y > -1:
            if flag:
                box_info[i][-1] = float(lane_with_angle[temp_y][temp_x][0])
                if float(lane_with_angle[temp_y][temp_x][1]) != 0:
                    box_info[i][6] = float(lane_with_angle[temp_y][temp_x][1])
            else:
                box_info[i][-1] = float(lane_with_angle[temp_y][temp_x][0])
                if float(lane_with_angle[temp_y][temp_x][1]) != 0:
                    box_info[i][2] = float(lane_with_angle[temp_y][temp_x][1])
    return box_info


if __name__ == '__main__':
    lidar_adr = r'/data/cluster_test/Lidar3/point_cloud/'
    box_adr = r'/data/cluster_test/Lidar3/fusion_box/'

    state_npy = r'/data/new_state.npy'
    state = np.load(state_npy)
    print(max(abs(state[:, -2])))
    print(state)

    my_count = 520

    # #####*****  加载车道bmp  *****#####
    bmp_path = r'/data/station_2_lane.bmp'
    img_cv2 = cv2.imread(bmp_path)
    imgg = cv2.cvtColor(img_cv2, cv2.COLOR_BGR2RGB)
    bmp_img = cache_bmp(imgg)

    for i in range(100):
        box_file = box_adr + str(my_count) + '.npy'
        lidar_file = lidar_adr + str(my_count) + '.npy'
        my_count += 1
        box = np.load(box_file)
        point = np.load(lidar_file)

        corners = box_np_ops.center_to_corner_box3d(
            box[:, :3],
            box[:, 3:6],
            box[:, 6],
            origin=[0.5, 0.5, 0.5],
            axis=2)
        start_time = time.time()
        # new_box = fix_cluster(box, bmp_img)
        new_box = fix_cluster(box)
        print('the running time is : ', (time.time()-start_time)*10)
        new_box[:, 1] += 1
        new_corners = box_np_ops.center_to_corner_box3d(
            new_box[:, :3],
            new_box[:, 3:6],
            new_box[:, 6],
            origin=[0.5, 0.5, 0.5],
            axis=2)
        line_box = np.array(
            [[0, 1], [1, 2], [0, 3], [2, 3], [4, 5], [4, 7], [5, 6], [6, 7], [0, 4], [1, 5], [2, 6],
             [3, 7]])

        line_set_ori = []
        line_set_pro = []
        for my_num in range(corners.shape[0]):
            line_set = o3d.geometry.LineSet()
            line_set.points = o3d.utility.Vector3dVector(corners[my_num])
            line_set.lines = o3d.utility.Vector2iVector(line_box)
            colors = np.array([[1, 0, 0] for jj in range(len(line_box))])
            line_set.colors = o3d.utility.Vector3dVector(colors)
            line_set_ori.append(line_set)

        for my_num in range(new_corners.shape[0]):
            line_set = o3d.geometry.LineSet()
            line_set.points = o3d.utility.Vector3dVector(new_corners[my_num])
            line_set.lines = o3d.utility.Vector2iVector(line_box)
            colors = np.array([[0, 1, 0] for jj in range(len(line_box))])
            line_set.colors = o3d.utility.Vector3dVector(colors)
            line_set_pro.append(line_set)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(point[:, :3])
        pcd.paint_uniform_color([0.5, 0.5, 0.5])

        o3d.visualization.draw_geometries(line_set_ori + line_set_pro + [pcd])




