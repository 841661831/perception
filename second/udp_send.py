import socket
import struct
import numpy as np
import math
from second.long_lat import XYZ_To_BLH
from CommonDefine import *
PI_rads = math.pi / 180

stLidarParam = structLidarParam()
serializationParam(stLidarParam=stLidarParam)
logti_t = stLidarParam.getLidarLongitude(0)
lat_t = stLidarParam.getLidarLatitude(0)
angle_north_t = stLidarParam.getAngleNorthT(0)

def packet_head(head_data, timeStramp):
    #print('head_data:',head_data)
    # print('data[0]',head_data[0])
    head_data=np.array([int(i) for i in head_data])
    Equiq_id=struct.pack('BB',head_data[1],head_data[0])
    # print(head_data[0]+head_data[1]*256)
    data_reserve1=struct.pack(">H",0)
    fram_id=struct.pack('BBBB',0,0,head_data[3],head_data[2])
    time_stramp_temp1, time_stramp_temp2 = math.floor(timeStramp) + SECOND_1900_TO_1970 , int((timeStramp - math.floor(timeStramp))*1e6)
    # time_stramp_temp1,time_stramp_temp2=parse_GPS_time()
    # st1=struct.pack('>I', time_stramp_temp1)
    # print('st1',time_stramp_temp1)
    # st2=struct.pack('>I', time_stramp_temp2)
    # print('st2', time_stramp_temp2)
    time_stramp=struct.pack('>I',time_stramp_temp1)+struct.pack('>I',time_stramp_temp2)
    # time_stramp=struct.pack('BBBBBBBB',head_data[23],head_data[22],head_data[21],head_data[20],head_data[19],head_data[18],head_data[17],head_data[16])
    #time_p=struct.pack('')
    #Head=+
    #nance
    # logti = struct.pack('>I', 116290075)
    # lat = struct.pack('>I', 40048372)
    #beice
    # logti = struct.pack('>I', 116289267)
    # lat = struct.pack('>I', 40050532)
    #8haomen
    logti=struct.pack('>I',int(logti_t * 1e7))
    lat=struct.pack('>I',int(lat_t * 1e7))
    #print(f'==object_long {logti},==object_lat{lat}')

    angle_north=struct.pack('>H',int(round(angle_north_t)))
    head_pack=Equiq_id+data_reserve1+fram_id+time_stramp+logti+lat+angle_north
    return head_pack

def object_packet_data(trackers_cov):
    m, n = trackers_cov.shape
    # print('trans_num', m)
    # trans_num=struct.pack('B',m)

    trans_pack=[]
    # m = min(100, m)
    for i in range(m):
        object_id=struct.pack('>H',int(trackers_cov[i,9]%65000))
        #print('object_id',object_id)
        nType = getSendEnumFromPcEnum(int(trackers_cov[i,7]))
        if nType == -1 or nType == 7:
            nType = 0
        if nType == 8:
            nType = 7
        # print("PcClass:{}-{}, SendClass:{}-{}".format(int(trackers_cov[i,7], switch_pc[int(trackers_cov[i,7])], nType, switch_send[nType])))
        object_type=struct.pack('B',int(nType))
        # object_type=struct.pack('B', int(trackers_cov[i,7]))
        object_confidence=struct.pack('B',int(trackers_cov[i, 10] * 100))
        # print('track_cov')
        # print(len(trackers_cov[i]))
        # if len(trackers_cov[i]) > 10: # avoid store and add color
        #     color=trackers_cov[i,-1]
        #     object_color=struct.pack('B',int(color))
        #     source=trackers_cov[i,-2]
        #     object_source=struct.pack('B',int(source))
        # else:
        #     object_color = struct.pack('B', 0)
        #     object_source = struct.pack('B', 0)
        object_color = struct.pack('B', 0)
        object_source = struct.pack('B', 0)
        object_reserve1=struct.pack('>H',0)
        logti ,lat= XYZ_To_BLH(logti_t,lat_t,trackers_cov[i, 0],trackers_cov[i,1],angle_north_t)
        #print(f'--logti {logti},--lat{lat}')
        #print(f'long{int(logti*10000000)}')
        object_log=struct.pack('>i',int(logti*1e7))
        object_lat=struct.pack('>i',int(lat*1e7))
        #print(f'--long{o}')
        #print(f'--object_long {object_log},--object_lat{object_lat}')
        object_alti=struct.pack('>h',0)
        speed=trackers_cov[i,8]*100
        object_speed=struct.pack('>H',int(round(speed)))
        # pitch=trackers_cov[i,6]/ PI_rads
        pitch=(180+angle_north_t+trackers_cov[i,6] / PI_rads)%360
        object_pitch=struct.pack('>H',int(round(pitch)))
        length=trackers_cov[i,4]*100
        object_length=struct.pack('>H',int(round(length)))
        wide=trackers_cov[i, 3]*100
        object_wide = struct.pack('>H', int(round(wide)))
        height=trackers_cov[i, 5]*100
        object_height = struct.pack('>H',int(round(height)))
        center_x=trackers_cov[i, 0]*100
        object_center_x = struct.pack('>h',int(round(center_x)))
        center_y=trackers_cov[i, 1]*100
        object_center_y = struct.pack('>h',int(round(center_y)))
        center_z=trackers_cov[i, 2]*100
        object_center_z = struct.pack('>h',int(round(center_z)))
        object_reserve2 = struct.pack('>H', 0)
        sigle_object=object_id+object_type+object_confidence+object_color+object_source+\
                     object_reserve1+object_log+object_lat+object_alti+object_speed+object_pitch+object_length+\
                     object_wide+object_height+object_center_x+object_center_y+object_center_z+object_reserve2
        trans_pack.append(sigle_object)

    return trans_pack

def object_nums(trackers_cov):
    m, n = trackers_cov.shape
    # m = min(100, m)
    trans_num = struct.pack('B', m)
    object_reserve2=struct.pack('BBBBB',0,0,0,0,0)
    trans=trans_num+object_reserve2
    return trans


def information_head(trackers_cov):
    m, n = trackers_cov.shape
    # m = min(100, m)

    start=struct.pack('>BB',255,255)
    serial_num=struct.pack('B',0)
    main_command=struct.pack('B',225)
    sub_command=struct.pack('B',2)
    state=struct.pack('B',0)
    length=m*36+36
    information_lenth=struct.pack('>H',int(length))
    information = start+serial_num+main_command+sub_command+state+information_lenth
    return information










# def main():
#     #创建套件字
#     udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     while 1:
#         b1=struct.pack('BB', 12,24)
#
#         b2=struct.pack('>I', 255)
#
#         send_data=b1+b2
#         # send_data =np.array([1,2,3,4])
#         # udp_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
#         udp_socket.sendto(send_data,("192.168.2.89",8888))
#         print('success')
#         break
#
#
#
#     udp_socket.close()
#
#
# if __name__=="__main__":
#     main()
