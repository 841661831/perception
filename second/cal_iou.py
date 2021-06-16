# #####*****  计算IOU的新算法  *****#####

import time
import operator
import numpy as np
import matplotlib.pyplot as plt


def CalArea(poly):
    pt = poly[-1]
    area = 0
    for i in range(len(poly)-1):
        line1 = []
        line2 = []
        line1.append(poly[i][0]-pt[0])
        line1.append(poly[i][1]-pt[1])
        line2.append(poly[i+1][0]-pt[0])
        line2.append(poly[i+1][1]-pt[1])
        area += 0.5 * abs(line1[0]*line2[1]-line1[1]*line2[0])
    return area

def PointCmp(a, b, center_x, center_y):
    if a[0] >= 0 and b[0] < 0:
        return True
    if (a[0] == 0 and b[0] == 0):
        return a[1] > b[1]
    det = int((a[0] - center_x) * (b[1] - center_y) - (b[0] - center_x) * (a[1] - center_y))
    if det < 0:
        return True
    if det > 0:
        return False
    d1 = int((a[0] - center_x) * (a[0] - center_x) + (a[1] - center_y) * (a[1] - center_y))
    d2 = int((b[0] - center_x) * (b[0] - center_y) + (b[1] - center_y) * (b[1] - center_y))
    return d1 > d2

def ClockwiseSortPoints(vPoints):
    x = 0
    y = 0
    for i in range(len(vPoints)):
        x += vPoints[i][0]
        y += vPoints[i][1]
    center_x = int(x / len(vPoints))
    center_y = int(y / len(vPoints))
    for i in range(len(vPoints)-1):
        for j in range(len(vPoints)-i-1):
            if PointCmp(vPoints[j], vPoints[j+1], center_x, center_y):
                tmp = vPoints[j]
                vPoints[j] = vPoints[j+1]
                vPoints[j+1] = tmp


def IsPointInPolygon(poly, pt):
    area = 0
    area_tmp = 0
    for i in range(len(poly)):
        vec_1 = []
        vec_2 = []
        num = i % len(poly)
        num_next = (i+1) % len(poly)
        vec_1.append(poly[num][0]-pt[0])
        vec_1.append(poly[num][1]-pt[1])
        vec_2.append(poly[num_next][0]-pt[0])
        vec_2.append(poly[num_next][1]-pt[1])
        area += 0.5 * abs(vec_1[0]*vec_2[1]-vec_1[1]*vec_2[0])

    for i in range(len(poly)-2):
        vec_1 = []
        vec_2 = []
        pt = poly[-1]
        num = i % len(poly)
        num_next = (i+1) % len(poly)
        vec_1.append(poly[num][0]-pt[0])
        vec_1.append(poly[num][1]-pt[1])
        vec_2.append(poly[num_next][0]-pt[0])
        vec_2.append(poly[num_next][1]-pt[1])
        area_tmp += 0.5 * abs(vec_1[0]*vec_2[1]-vec_1[1]*vec_2[0])

    # c = False
    # for i in range(len(poly)):
    #     if i==0:
    #         j = len(poly) - 1
    #     else:
    #         j = i
    #     if ((((poly[i][1] <= pt[1]) and (pt[1] < poly[j][1])) or ((poly[j][1] <= pt[1]) and (pt[1] < poly[i][1]))) and (pt[0] < (poly[j][0] - poly[i][0]) * (pt[1] - poly[i][1])/(poly[j][1] - poly[i][1]) + poly[i][0])):
    #         c = not(c)
    area = round(area, 3)
    area_tmp = round(area_tmp, 3)
    return area == area_tmp




def IsRectCross(p1, p2, q1, q2):
    ret = min(p1[0],p2[0]) <= max(q1[0],q2[0]) and min(q1[0],q2[0]) <= max(p1[0],p2[0]) and min(p1[1],p2[1]) <= max(q1[1],q2[1]) and min(q1[1],q2[1]) <= max(p1[1],p2[1])
    return ret

def IsLineSegmentCross(pFirst1, pFirst2, pSecond1, pSecond2):

    # line = []
    # line.append(pFirst2[0]-pFirst1[0])
    # line.append(pFirst2[1]-pFirst1[1])
    # line1 = []
    # line1.append(pSecond1[0] - pFirst1[0])
    # line1.append(pSecond1[1] - pFirst1[1])
    # line2 = []
    # line2.append(pSecond2[0] - pFirst1[0])
    # line2.append(pSecond2[1] - pFirst1[1])
    # if (line1[0]*line[1]-line1[1]*line[0])*(line2[0]*line[1]-line2[1]*line[0])<0:
    #     return True
    #
    # line = []
    # line.append(pSecond2[0] - pSecond1[0])
    # line.append(pSecond2[1] - pSecond1[1])
    # line1 = []
    # line1.append(pFirst1[0] - pSecond1[0])
    # line1.append(pFirst1[1] - pSecond1[1])
    # line2 = []
    # line2.append(pFirst2[0] - pSecond1[0])
    # line2.append(pFirst2[1] - pSecond1[1])
    # if (line1[0]*line[1]-line1[1]*line[0])*(line2[0]*line[1]-line2[1]*line[0])<0:
    #     return True

    # line1 = int(pFirst1[0] * (pSecond1[1] - pFirst2[1]) + pFirst2[0] * (pFirst1[1] - pSecond1[1]) + pSecond1[0] * (pFirst2[1] - pFirst1[1]))
    # line2 = int(pFirst1[0] * (pSecond2[1] - pFirst2[1]) + pFirst2[0] * (pFirst1[1] - pSecond2[1]) + pSecond2[0] * (pFirst2[1] - pFirst1[1]))
    # if ((line1 ^ line2) >= 0) and (not(line1 == 0 and line2 == 0)):
    #     return False
    # line1 = int(pSecond1[0] * (pFirst1[1] - pSecond2[1]) + pSecond2[0] * (pSecond1[1] - pFirst1[1]) + pFirst1[0] * (pSecond2[1] - pSecond1[1]))
    # line2 = int(pSecond1[0] * (pFirst2[1] - pSecond2[1]) + pSecond2[0] * (pSecond1[1] - pFirst2[1]) + pFirst2[0] * (pSecond2[1] - pSecond1[1]))
    # if ((line1 ^ line2) >= 0) and (not(line1 == 0 and line2 == 0)):
    #     return False
    line1 = float(pFirst1[0] * (pSecond1[1] - pFirst2[1]) + pFirst2[0] * (pFirst1[1] - pSecond1[1]) + pSecond1[0] * (
                pFirst2[1] - pFirst1[1]))
    line2 = float(pFirst1[0] * (pSecond2[1] - pFirst2[1]) + pFirst2[0] * (pFirst1[1] - pSecond2[1]) + pSecond2[0] * (
                pFirst2[1] - pFirst1[1]))
    if ((line1 * line2) >= 0) and (not (line1 == 0 and line2 == 0)):
        return False
    line1 = float(pSecond1[0] * (pFirst1[1] - pSecond2[1]) + pSecond2[0] * (pSecond1[1] - pFirst1[1]) + pFirst1[0] * (
                pSecond2[1] - pSecond1[1]))
    line2 = float(pSecond1[0] * (pFirst2[1] - pSecond2[1]) + pSecond2[0] * (pSecond1[1] - pFirst2[1]) + pFirst2[0] * (
                pSecond2[1] - pSecond1[1]))
    if ((line1 * line2) >= 0) and (not (line1 == 0 and line2 == 0)):
        return False

    return True





def GetCrossPoint(p1, p2, q1, q2):
    x = 0.0
    y = 0.0
    if IsRectCross(p1, p2, q1, q2):
        if IsLineSegmentCross(p1, p2, q1, q2):
            # tempLeft = (q2[0] - q1[0])*(p1[1] - p2[1]) - (p2[0] - p1[0])*(q1[1] - q2[1])
            # tempRight = (p1[1] - q1[1]) * (p2[0] - p1[0]) * (q2[0] - q1[0]) + q1[0] * (q2[1] - q1[1]) * (p2[0] - p1[0]) - p1[0] * (p2[1] - p1[1]) * (q2[0] - q1[0])
            # x = int(float(tempRight)/float(tempLeft))
            #
            # tempLeft = (p1[0] - p2[0]) * (q2[1] - q1[1]) - (p2[1] - p1[1]) * (q1[0] - q2[0])
            # tempRight = p2[1] * (p1[0] - p2[0]) * (q2[1] - q1[1]) + (q2[0] - p2[0]) * (q2[1] - q1[1]) * (p1[1] - p2[1]) - q2[1] * (q1[0] - q2[0]) * (p2[1] - p1[1])
            # y = int(float(tempRight)/float(tempLeft))

            # #####*****  计算交点  *****#####
            x1 = p1[0]
            x2 = p2[0]
            x3 = q1[0]
            x4 = q2[0]
            y1 = p1[1]
            y2 = p2[1]
            y3 = q1[1]
            y4 = q2[1]
            x = ((x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4)) / ((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4))
            y = ((x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4)) / ((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4))

            return True, x, y
    return False, x, y


def PolygonClip(poly1, poly2, interpoly):
    if len(poly1) < 3 or len(poly2) < 3:
        return False
    # #####*****  计算多边形的交点  *****#####
    if operator.eq(poly1, poly2):
        return False
    else:
        for i in range(len(poly1)):
            poly1_next_idx = (i+1) % len(poly1)
            for j in range(len(poly2)):
                poly2_next_idx = (j+1) % len(poly2)
                a, x, y =GetCrossPoint(poly1[i], poly1[poly1_next_idx], poly2[j], poly2[poly2_next_idx])
                if a:
                    interpoly.append([x, y])
        # print('*****')
        # #####*****  计算多边形内部点  *****#####
        for i in range(len(poly1)):
            if IsPointInPolygon(poly2, poly1[i]):
                interpoly.append(poly1[i])

        for i in range(len(poly2)):
            if IsPointInPolygon(poly1, poly2[i]):
                interpoly.append(poly2[i])
        # print(interpoly)
        if len(interpoly) <= 0:
            return False
        ClockwiseSortPoints(interpoly)
    return True

from shapely.geometry import Polygon
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

if __name__ == '__main__':
    # poly1 = [[1,1],[1,4],[3,4],[3,1]]
    # poly2 = [[2,2],[2,6],[5,6],[5,2]]
    # poly1 = [[2.0, 0.0], [0.0, 2.0], [3.0, 5.0], [5.0, 3.0]]
    # poly2 = [[3.0, 0.0], [3.0, 6.0], [6.0, 6.0], [6.0, 0.0]]
    start_time = time.time()
    # for i in range(200):
    poly1 = [[2.0, 0.0], [0.0, 2.0], [3.0, 5.0], [5.0, 3.0]]
    poly2 = [[3.0, 0.0], [3.0, 6.0], [6.0, 6.0], [6.0, 0.0]]

    # poly1 = [[30.15327502, -14.8477377], [28.20593986, -15.01807183], [28.56548261, -19.12852344], [30.51281777, -18.95818931]]
    # poly2 = [[27.56745447, -14.64647501], [29.74246499, -16.29425593], [32.32699259, -12.88277406], [30.15198207, -11.23499314]]

    poly1 = [[ -69.22071549,10.95034856],[ -63.8047219, 10.56015365],
  [ -63.94485908, 8.61501803],
  [ -69.36085266,    9.00521294]]
    poly2 = [[ -68.56731931 ,  10.3199109 ],
  [ -64.07367287 ,  10.08086773],
  [ -64.17726765 ,   8.13344402],
  [ -68.67091409 ,   8.37248719]]
    # print('xixixix')
    # print(len(poly1))
    # print(len(poly1[0]))
    # poly1 = [[-69.22071549044922, 10.950348560086605],
    #          [-63.8047219047974, 10.56015365039],
    #          [-63.944859075263324, 8.615018033564526],
    #          [-69.36085266091514, 9.005212943261132]]
    # poly2 = [[-68.5673193091727, 10.319910897765952],
    #          [-64.07367286519779, 10.080867728962373],
    #          [-64.17726764974118, 8.133444016719805],
    #          [-68.67091409371609, 8.372487185523385]]
    print('hahaha')
    print(len(poly1))
    print(len(poly1[0]))
    start_time = time.time()
    iou = intersection(poly1, poly2)
    print('the running time is : ',time.time()-start_time)
    print('heiheihie')
    print(iou)
    # poly1 = [[-162.89524092, 22.1337662],
    #                      [-149.29098451, 20.17799717],
    #                      [-149.568492, 18.24766544],
    #                      [-163.1727484, 20.20343447]]
    # poly2 = [[-161.94539619, 26.37430005],
    #                      [-146.0330751, 24.08296155],
    #                      [-146.38313641, 21.65194236],
    #                      [-162.29545749, 23.94328086]]
    start_time = time.time()
    for num in range(1):
        interpoly = []
        b = PolygonClip(poly1, poly2, interpoly)
        print(interpoly)
        if b:
            list2 = []
            for i in range(len(interpoly)):
                list2.append(interpoly[i])
            for i in range(len(list2)-1):
                for j in range(i+1, len(list2)):
                    if operator.eq(list2[i], list2[j]):
                        interpoly.pop(j)
            area = CalArea(interpoly)
            area1 = CalArea(poly1)
            area2 = CalArea(poly2)
            print(area)
            print(area1)
            print(area2)
            IOU = area / (area2+area1-area)
        else:
            IOU = 0
    print('the old running time is : ', time.time()-start_time)
    print(IOU)
    # x1 = []
    # x2 = []
    # y1 = []
    # y2 = []
    # for i in range(len(poly1)):
    #     x1.append(poly1[i][0])
    #     y1.append(poly1[i][1])
    # x1.append(poly1[0][0])
    # y1.append(poly1[0][1])
    #
    # for i in range(len(poly2)):
    #     x2.append(poly2[i][0])
    #     y2.append(poly2[i][1])
    # x2.append(poly2[0][0])
    # y2.append(poly2[0][1])
    # plt.figure(1)
    # plt.plot(np.asarray(x1), np.asarray(y1), 'r')
    # plt.plot(np.asarray(x2), np.asarray(y2), 'g')
    # plt.show()



# [[-64.07367286519779, 10.080867728962373], [-68.64001832316539, 8.953280479738726]]
# [[-64.07367287, 10.08086773], [-68.56731931, 10.3199109], [-68.64001832118939, 8.953280476306581]]


# [[-68.64001832316539, 8.953280479738726], [-64.15086043126584, 8.629859388014383], [-64.07367286519779, 10.080867728962373]]
# 3.2693848159320087
# 10.589522834524752
# 8.775797265000008
# the old running time is :  0.0001266002655029297
# 0.2031186606015139

# [[-68.64001832118939, 8.953280476306581], [-64.1508604330264, 8.629859384262462], [-64.07367287, 10.08086773], [-68.56731931, 10.3199109]]
# 6.348650893650216
# 10.58952285226179
# 8.775797246219742
# the old running time is :  0.00013399124145507812
# 0.4877323679158895






