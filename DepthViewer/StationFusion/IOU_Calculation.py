import numpy as np
from numba import jit


'''points:[[116,1.83],[65,0.512],...,[]]'''
def point_to_box(points, vx_width, vy_width):
    box_lists = []
    for i in range(len(points)):
        if points[i][0] < 0.2 and points[i][1] < 0.2:
            vx1, vy1, vx2, vy2 = 0, 0, 0, 0
        else:
            vx1 = points[i][0] + vx_width
            vy1 = points[i][1] + vy_width
            vx2 = points[i][0] - vx_width
            vy2 = points[i][1] - vy_width
        box_lists.append([vx1, vy1, vx2, vy2])
    box_lists = np.array(box_lists)
    return box_lists


# @jit(nopython=True)#gyes
def IOU_calc(radar1_box, radar2_box):
    xmax1, ymax1, xmin1, ymin1 = radar1_box
    xmax2, ymax2, xmin2, ymin2 = radar2_box
    # 交叠区域
    xmax = np.min(np.array([xmax1, xmax2]))
    ymax = np.min(np.array([ymax1, ymax2]))
    xmin = np.max(np.array([xmin1, xmin2]))
    ymin = np.max(np.array([ymin1, ymin2]))

    area1 = (xmax1 - xmin1) * (ymax1 - ymin1)
    area2 = (xmax2 - xmin2) * (ymax2 - ymin2)
    inter_area = (np.max(np.array([0, xmax - xmin]))) * (np.max(np.array([0, ymax - ymin])))
    # area_box = (xmax2 - xmin2) * (ymax2 - ymin2)
    return inter_area / (area1 + area2 - inter_area + 1e-6)

# @jit(nopython=True)
def iou_matrix(radar1_box, radar2_box):
    N = radar1_box.shape[0]
    K = radar2_box.shape[0]
    overlaps = np.zeros((N, K), dtype=radar1_box.dtype)
    for n in range(N):
        for k in range(K):
            iou = IOU_calc(radar1_box[n], radar2_box[k])
            overlaps[n, k] = iou
    return overlaps


@jit(nopython=True)
def iou_jit(boxes, query_boxes, eps=1.0):
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