import math
import time
import numpy as np
PI=3.1415926535897932
ARC=6371004

# 地球长半轴
R_a = 6378137.00
# 地球短半轴
R_b = 6356752.3142

# // 椭球体长半轴, 米
__A = 6378137
# // 椭球体短半轴, 米
# __B = 6356752.3142
__B = 6356725

# // 标准纬度, 弧度（取激光雷达所在纬度）
__B0=0.0
# // 原点经度, 弧度(0°)
__L0=0.0
# //反向转换程序中的迭代初始值
__IterativeValue=10

#角度转换成弧度
def FG_degree2rad(degree):
    return degree*PI/180.0

#弧度转角度
def FG_rad2degree(rad):
    return rad*180.0/PI

#获取距离值
def FG_getdistance(p_dLatObj, p_dlngObj, p_dLatTag, p_dlngTag):
    p_dDistance = 0
    radLat1 = FG_degree2rad(p_dLatObj)
    radLat2 = FG_degree2rad(p_dLatTag)
    a = radLat1 - radLat2
    b = FG_degree2rad(p_dlngObj) - FG_degree2rad(p_dlngTag)
    dst = 2 * math.asin((math.sqrt(math.pow(math.sin(a / 2), 2) + math.cos(radLat1) * math.cos(radLat2) * math.pow(math.sin(b / 2), 2))))
    dst = dst * ARC / 1000
    p_dDistance = (dst * 10000000) / 10000
    return p_dDistance

#获取真实
def FG_GetTrueBear(p_dLatObj,p_dlngObj, p_dLatTag, p_dlngTag):
    p_dTrueBear = 0
    Azi = 0
    azi=0
    cosc = 0
    cosc2 = 0
    sinc = 0
    asinA = 0

    #角度转弧度
    radLat1 = FG_degree2rad(p_dLatObj)
    radLng1 = FG_degree2rad(p_dlngObj)
    radLat2 = FG_degree2rad(p_dLatTag)
    radLng2 = FG_degree2rad(p_dlngTag)

    #算法求解
    DeltaLat = p_dLatTag - p_dLatObj
    DeltaLng = p_dlngTag - p_dlngObj

    cosc = math.cos(PI/2 - radLat2) * math.cos(PI/2 - radLat1) + math.sin(PI/2 - radLat2) * math.sin(PI/2 - radLat1) * math.cos(radLng2 - radLng1)
    cosc2 = math.pow(cosc, 2)
    sinc = math.pow(1 - cosc2, 0.5)
    asinA = math.sin(PI/2 - radLat2) * math.sin(radLng2 - radLng1) / sinc
    if asinA > 1:
        asinA = 1
    if asinA<-1:
        asinA = -1
    azi = math.asin(asinA)
    #位置判断 两车在同一经度
    if DeltaLng ==0:
            if DeltaLat >0:
                Azi = 0
            if DeltaLat <0:
                Azi = PI
    # 两车在同一纬度
    elif DeltaLat ==0:
            if DeltaLng >0:
                Azi = PI/2
            if DeltaLng <0:
                Azi = PI * 3/2

    else:
        # 目标车B在自车A南方
        if DeltaLat <0:
            Azi = PI - azi
        if DeltaLat >0:
            # B在A东北方
            if DeltaLng >0:
                Azi = azi
            # B在A西北方
            if DeltaLng <0:
                Azi = 2 * PI + azi

    p_dTrueBear = FG_rad2degree(Azi)
    return p_dTrueBear
# class WayPoint:
#     def __init__(self):
#         self.x=0
#         self.y=0
#经纬度转换成墨卡托坐标
def LonLat2Mercator(B, L):
    # f / * 扁率 * /, e / * 第一偏心率 * /, e_ / * 第二偏心率 * /, NB0 / * 卯酉圈曲率半径 * /, K, dtemp;
    # f, e, e_, NB0, K, dtemp=0
    # print('lonlat2mercator')
    f = 0.0
    e = 0.0
    e_ = 0.0
    NB0 = 0.0
    E = 0.0
    dtemp = 0.0
    E = float(math.exp(1))
    # E = math.exp(1)
    # print(f'B:{B},L:{L}')

    # print('1')
    __B0 = B
    __L0 = 0
    if  L < -PI or L > PI or B < -PI / 2 or B > PI / 2:
        # print('2')
        return False
    if __A <= 0 or __B <= 0:
        # print('3')
        return False
    f = (__A - __B) / __A

    dtemp = 1 - (__B / __A) * (__B / __A)
    if dtemp < 0:
        # print('4')
        return False
    # print(f'dtemp1-->{dtemp}')
    e = math.sqrt(dtemp)

    dtemp = (__A / __B) * (__A / __B) - 1
    if dtemp < 0:
        # print('5')
        return False
    # print('6')
    # print(f'dtemp2-->{dtemp}')
    e_ = math.sqrt(dtemp)
    NB0 = ((__A * __A) / __B) / math.sqrt(1 + e_ * e_ * math.cos(__B0) * math.cos(__B0))
    K = NB0 * math.cos(__B0)
    x = K * (L - __L0)
    y = K * math.log(math.tan(PI / 4 + B / 2) * math.pow((1 - e * math.sin(B)) / (1 + e * math.sin(B)), e / 2))
    # print(f'x:{x},y:{y}')
    # return 0
    # print(f'__B0:{__B0}')
    # print(f'L2M:__B0:{__B0},__L0{__L0}')
    return x,y
#墨卡托坐标转经纬度
def Mercator2LonLat(B,L,X, Y):
    # double f/*扁率*/, e/*第一偏心率*/, e_/*第二偏心率*/, NB0/*卯酉圈曲率半径*/, K, dtemp;
    # double E = exp(1);
    f = 0.0
    e = 0.0
    e_ = 0.0
    NB0 = 0.0
    E = 0.0
    dtemp = 0.0
    E = float(math.exp(1))
    __B0 = B
    __L0 = 0


    if __A <= 0 or __B <= 0:
        return False

    f = (__A - __B) / __A
    dtemp = 1 - (__B / __A) * (__B / __A)
    if dtemp < 0:
        return False
    e = math.sqrt(dtemp)

    dtemp = (__A / __B) * (__A / __B) - 1
    if dtemp < 0:
        return False
    e_ = math.sqrt(dtemp)
    NB0 = ((__A * __A) / __B) / math.sqrt(1 + e_ * e_ * math.cos(__B0) * math.cos(__B0))
    K = NB0 * math.cos(__B0)
    Object_Long = FG_rad2degree(Y / K + __L0)
    # print(f'__B0:{__B0}')
    B = 0.0
    for i in range(__IterativeValue):
        B=PI/2-2*math.atan(math.pow(E, (-X/K)) * math.pow(E, (e/2)*math.log((1-e*math.sin(B))/(1+e*math.sin(B)))))
    Object_Lat= FG_rad2degree(B)
    # print(f'm2l:__B0:{__B0},__L0{__L0}')
    # print(f'object_long {Object_Long},object_lat{Object_Lat}')
    return Object_Long,Object_Lat

# /*
#   XYZ_To_BLH()：特征物体XY坐标转经纬度
#   输入：
#   WayPoint BLH_Origin: 激光雷达原始经纬度，角度
#   WayPoint XYZ_Move: 特征物体XY坐标，米
#   double rotaionangle： 激光雷达坐标系Y轴相对于正北方向夹角(0°～360°)顺时针
#   输出：
#   WayPoint *BLH_Move: 特征物体经纬度，角度
#  */
def XYZ_To_BLH(original_long,original_lat, move_x,move_y, rotaionangle):

    RadAngle =float(FG_degree2rad(rotaionangle))
    # Lat,Lng=0
    # Mercator_X, Mercator_Y=0



    # //激光器经纬度转墨卡托XY
    # WayPoint LiDAR_XYZ;
    # mer_x=0.0
    # mer_y=0.0
    mer_x,mer_y=LonLat2Mercator(FG_degree2rad(original_lat),FG_degree2rad(original_long))

    # //坐标轴旋转到正北方向,计算Move点墨卡托XY坐标
    # WayPoint Move;
    mer_move_x = move_x * math.cos(RadAngle) + move_y * math.sin(RadAngle) + mer_x
    mer_move_y = move_y * math.cos(RadAngle) - move_x * math.sin(RadAngle) +mer_y
    # print(f'mer_move_x:{mer_move_x},mer_move_y:{mer_move_y}')
    # print(f'dd:{math.sqrt( mer_move_x/1000* mer_move_x/1000+ mer_move_y/1000* mer_move_y/1000)/1000}')
    Object_Long,Object_Lat=Mercator2LonLat(FG_degree2rad(original_lat),FG_degree2rad(original_long),mer_move_y,mer_move_x)
    return Object_Long,Object_Lat


def cal_trans(x, y, z):
    R_x = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(x), -1 * math.sin(x)], [0.0, math.sin(x), math.cos(x)]])
    R_y = np.array([[math.cos(y), 0.0, math.sin(y)], [0.0, 1.0, 0.0], [-1 * math.sin(y), 0.0, math.cos(y)]])
    R_z = np.array([[math.cos(z), -1 * math.sin(z), 0.0], [math.sin(z), math.cos(z), 0.0], [0.0, 0.0, 1.0]])
    rotate = np.dot(R_z, R_y)
    rotate = np.dot(rotate, R_x)

    return rotate

def XYZ_To_BLH_batch(original_long,original_lat, box, rotaionangle):
    RadAngle = float(FG_degree2rad(rotaionangle))
    mer_x, mer_y = LonLat2Mercator(FG_degree2rad(original_lat), FG_degree2rad(original_long))
    for i in range(box.shape[0]):
        move_x = box[i][0]
        move_y = box[i][1]
        mer_move_x = move_x * math.cos(RadAngle) + move_y * math.sin(RadAngle) + mer_x
        mer_move_y = move_y * math.cos(RadAngle) - move_x * math.sin(RadAngle) +mer_y
        Object_Long,Object_Lat = Mercator2LonLat(FG_degree2rad(original_lat),FG_degree2rad(original_long),mer_move_y,mer_move_x)
        box[i][-2] = Object_Long
        box[i][-1] = Object_Lat
    return box


def lonlat_to_xyz_batch(box, lon0, lat0, angle_north):
    for i in range(box.shape[0]):
        lon = box[i][-2]
        lat = box[i][-1]
        x = (lon - lon0) * math.pi / 180 * R_a * math.cos(lat0 * math.pi / 180)
        y = (lat - lat0) * math.pi / 180 * R_b
        xyz = np.array([[x, y, -4.2]])  # 旋转
        R_bmp = cal_trans(0, 0, angle_north * np.pi / 180)
        A = np.dot(R_bmp, xyz[:, :3].T)
        xyz[:, :3] = A.T
        box[i][-4] = xyz[0][0]
        box[i][-3] = xyz[0][1]
    return box


if __name__=='__main__':
    original_long=120.0
    original_lat=45.0
    move_x=-100
    move_y=100
    rotaionangle=0.0
    object_long=0.0
    object_lat=0.0
    a=time.time()
    # for i in range(1000):
    object_long, object_lat=XYZ_To_BLH(original_long, original_lat, move_x, move_y, rotaionangle)
    print(f'haoshi:{time.time()-a}')
    print(f'object_long {object_long},object_lat {object_lat}')
    dist = FG_getdistance(original_lat, original_long, object_lat, object_long)
    print(f'dist:{dist}')

    true_bear=FG_GetTrueBear(original_lat, original_long, object_lat, object_long)
    print(f'truebear:{true_bear}')