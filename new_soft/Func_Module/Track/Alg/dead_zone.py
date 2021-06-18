import numpy as np
import argparse
import open3d as o3d
import math
from numba import jit
import time


# #####*****   动态展示盲区检测效果   *****#####
@jit(nopython=True)
def cal_angle(box_all, in_region, center_mask_x, center_mask_y, angle, l_angle):
    for i in range(in_region[0].shape[0]):
        diff_x = box_all[in_region[0][i]][0] - center_mask_x
        diff_y = box_all[in_region[0][i]][1] - center_mask_y
        angle_temp = math.atan2(diff_y, diff_x) * 180 / np.pi
        if diff_y < 0:
            angle_temp += 360
        angle_temp = angle_temp % 360
        angle[i] = angle_temp
        l_angle[in_region[0][i]] = angle_temp

@jit(nopython=True)
def update_mask(l_angle, indices, l_dis, mask):
    for i in range(indices.shape[0]):
        for j in range(indices.shape[0] - 1):
            angle_diff = abs(l_angle[indices[i]][0] - l_angle[indices[j + 1]][0])
            if abs(angle_diff) > 180:
                angle_diff = 360 - abs(angle_diff)
            if angle_diff < 30 and angle_diff > 0:
                if l_dis[indices[i]] > l_dis[indices[j + 1]]:
                    print('i am running')
                    mask[indices[i]] = 0
                else:
                    mask[indices[j + 1]] = 0


def get_object(box_mask, box_all, edge_info, roi_dis, find_dis):  # 中心点坐标， trackers_cov， box_data， 
    # #####*****  此处是为了计算场景中所有检测目标的质心，如果能够输入检测目标的box信息，就可以省去该步骤直接获得质心  *****#####
    mask = np.zeros((box_all.shape[0], 1))
    angle = np.zeros((box_all.shape[0], 1))
    center_mask_x = box_mask[0]
    center_mask_y = box_mask[1]
    l_x = box_all[:, 0] - center_mask_x
    l_y = box_all[:, 1] - center_mask_y
    l_dis = [math.sqrt(l_x[i] ** 2 + l_y[i] ** 2) for i in range(l_x.shape[0])]
    l_dis = np.array(l_dis)
    index_mask = np.argwhere(l_dis < 0.001)
    mask[index_mask[0][0]][0] = -1
    in_region = np.where(l_dis < find_dis)
    for i in range(in_region[0].shape[0]):
        if mask[in_region[0][i]][0] > -1:
            mask[in_region[0][i]][0] = 2

    l_angle = np.zeros((edge_info.shape[0], 1))

    cal_angle(box_all, in_region, center_mask_x, center_mask_y, angle, l_angle)

    indices = np.where(mask == 2)

    update_mask(l_angle, indices[0], l_dis, mask)

    indices = np.where(mask == 2)
    for i in range(indices[0].shape[0]):
        box_con = edge_info[indices[0][i]]
        con_theta_list = []
        for j in range(0, 8, 2):
            con_x = box_con[j][0] - center_mask_x
            con_y = box_con[j][1] - center_mask_y
            con_theta = math.atan2(con_y, con_x) * 180 / np.pi
            if con_y < 0:
                con_theta += 360
            if con_theta < 0:
                con_theta += 360
            con_theta = con_theta % 360

            con_theta_list.append(con_theta)
        # print('######',con_theta_list)

        temp = []
        zuhe = np.array([[0, 1], [0, 2], [0, 3], [1, 2], [1, 3], [2, 3]])
        temp.append(abs(con_theta_list[0] - con_theta_list[1]))
        temp.append(abs(con_theta_list[0] - con_theta_list[2]))
        temp.append(abs(con_theta_list[0] - con_theta_list[3]))
        temp.append(abs(con_theta_list[1] - con_theta_list[2]))
        temp.append(abs(con_theta_list[1] - con_theta_list[3]))
        temp.append(abs(con_theta_list[2] - con_theta_list[3]))
        a = max(temp)
        a_ind = temp.index(a)
        angle = []
        angle.append(con_theta_list[zuhe[a_ind][0]])
        angle.append(con_theta_list[zuhe[a_ind][1]])
        # print(angle)
        start = min(angle)
        end = max(angle)
        l_start = edge_info[indices[0][i]][zuhe[a_ind][0] * 2]
        l_end = edge_info[indices[0][i]][zuhe[a_ind][1] * 2]

        print('start is :', start)
        print('end is :', end)
        for num in range(edge_info.shape[0]):
            if mask[num] != -1 and mask[num] != 2:
                diff_x = box_all[num][0] - center_mask_x
                diff_y = box_all[num][1] - center_mask_y
                dis_angle = math.atan2(diff_y, diff_x) * 180 / np.pi

                if diff_y < 0:
                    dis_angle += 360
                if dis_angle < 0:
                    dis_angle += 360
                dis_angle = dis_angle % 360

                if (end - start) > 180:
                    limit_start = start - 5
                    limit_end = (end + 5) % 360
                    if limit_start < 0:
                        limit_start = limit_start + 360
                        if dis_angle > limit_end and dis_angle < start and l_dis[num] < roi_dis:
                        # if dis_angle > limit_end and dis_angle < start:
                            mask[num] = 6
                            print('###11111###')
                            print('the new limit start:', limit_start)
                            print('the new limit end:', limit_end)
                            print(dis_angle)
                    else:
                        if dis_angle < limit_start or dis_angle > limit_end:
                            if l_dis[num] < roi_dis:
                                mask[num] = 6
                else:
                    limit_start = (start + 5) % 360
                    limit_end = (end - 5) % 360
                    if dis_angle > start + 5 and dis_angle < end - 5:
                        if l_dis[num] < roi_dis:
                            mask[num] = 6
    mask = np.array(mask).reshape(box_all.shape[0],)
    dead_id = box_all[mask == 6][:,9].astype(np.int32).reshape(-1,1)
    occlu_id = box_all[mask == 2][:,9].astype(np.int32).reshape(-1,1)

    return mask, dead_id, occlu_id





if __name__ == '__main__':
    info_path = r'/data/PC_data/qiangge/'
    edge_path = r'/data/PC_data/qiangge_edges/'
    root_path = r'/data/PC_data/collision/'
    parser = argparse.ArgumentParser()
    while True:
        for frame in range(35,131,5):
            bin_path = root_path + str(frame) + '.bin'
            n = str(frame)
            s = n.zfill(6)
            info_name = info_path + s + '.npy'
            box_info = np.load(info_name)
            edge_name = edge_path + s + '.npy'
            edge_info = np.load(edge_name)


            PC_data = np.fromfile(bin_path, dtype=np.float32, count=-1).reshape((-1, 4))
            print(box_info.shape)
            start_time = time.time()
            box_mask = box_info[1]
            roi_dis = 80
            find_dis = 15
            mask, dead_id, occlu_id = get_object(box_mask,box_info, edge_info, roi_dis, find_dis)
            print(mask)
            print(mask.shape)
            print(time.time() - start_time)
            view = []
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(PC_data[:, :3])
            pcd.paint_uniform_color([0, 1, 1])
            view.append(pcd)

            for i in range(edge_info.shape[0]):
                if mask[i] == -1:
                    points = edge_info[i]
                    lines = [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6], [6, 7], [7, 4], [0, 4], [1, 5], [2, 6], [3, 7]]
                    colors = [[0, 0, 1] for j in range(len(lines))]
                    line_pcd = o3d.geometry.LineSet()
                    line_pcd.lines = o3d.utility.Vector2iVector(lines)
                    line_pcd.colors = o3d.utility.Vector3dVector(colors)
                    line_pcd.points = o3d.utility.Vector3dVector(points)
                    view.append(line_pcd)
                elif mask[i] == 2:
                    points = edge_info[i]
                    lines = [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6], [6, 7], [7, 4], [0, 4], [1, 5], [2, 6], [3, 7]]
                    colors = [[0.1, 0.1, 0.1] for j in range(len(lines))]
                    line_pcd = o3d.geometry.LineSet()
                    line_pcd.lines = o3d.utility.Vector2iVector(lines)
                    line_pcd.colors = o3d.utility.Vector3dVector(colors)
                    line_pcd.points = o3d.utility.Vector3dVector(points)
                    view.append(line_pcd)

                elif mask[i] == 6:
                    points = edge_info[i]
                    lines = [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6], [6, 7], [7, 4], [0, 4], [1, 5], [2, 6], [3, 7]]
                    colors = [[1, 0, 0] for j in range(len(lines))]
                    line_pcd = o3d.geometry.LineSet()
                    line_pcd.lines = o3d.utility.Vector2iVector(lines)
                    line_pcd.colors = o3d.utility.Vector3dVector(colors)
                    line_pcd.points = o3d.utility.Vector3dVector(points)
                    view.append(line_pcd)

                else:
                    points = edge_info[i]
                    lines = [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6], [6, 7], [7, 4], [0, 4], [1, 5], [2, 6], [3, 7]]
                    colors = [[1, 1, 0] for j in range(len(lines))]
                    line_pcd = o3d.geometry.LineSet()
                    line_pcd.lines = o3d.utility.Vector2iVector(lines)
                    line_pcd.colors = o3d.utility.Vector3dVector(colors)
                    line_pcd.points = o3d.utility.Vector3dVector(points)
                    view.append(line_pcd)

            o3d.visualization.draw_geometries(view)
