from Common.CommonDefine import *
import cv2
import numpy as np
import time
import struct
switch_send = {0: "None", 1: "car", 2: "truck", 3: "bus", 4: "person", 5: "bicycle", 6: "motorbike", 7: "cone", 8: "Minibus"}
switch_pc = {0: "car", 1: "bicycle", 2: "bus", 3: "motorbike", 4: "person", 5: "cone", 6: "truck", 7: "None", 8: "Minibus"}
MAX_FRAME_ID = 1e10  # 帧号最大值，超过该值，则重新计数

class SendData:
    ''' 用于存储read进程中需要发送的信息'''
    def __init__(self):
        self.nFrameId = 0
        self.nSingleFrame = {}
        self.nSingleStamp = {}
        self.nSingleSource = {}
        self.recvTime = {}
        # self.n0TimeStamp = -1
        # self.n0Frame = -1
        self.stationState = {}
        self.trunAlg = 0
        self.nEqupId = []
        self.tTimeStamp = 0

class Lon_Lat_Meter_Transform(object):
    '''经纬度与m坐标系相互转换'''
    def __init__(self,lon0_bmp,lat0_bmp,angle0_bmp):#与全域基站经纬度转m转换关系一致
        '''初始化
        @param:
            lon0_bmp,lat0_bmp,angle0_bmp:用于绘图时的粗标的经纬度及夹角
        '''
        self.R_a = 6378137.00 # 地球长半轴
        self.R_b = 6356752.3142 # 地球短半轴
        self.lon0_bmp=lon0_bmp
        self.lat0_bmp=lat0_bmp
        self.angle0_bmp=angle0_bmp
        self.R_bmp,_=cv2.Rodrigues(np.array([0, 0, self.angle0_bmp * np.pi/180]))
        self.R_bmp_inv=np.linalg.inv(self.R_bmp)

    def lonlat_to_xy_of_draw(self,lonlat):
        '''经纬度转到画车道图的激光雷达m坐标系下
        @param:
            lon_lat:(-1,2) np.array
        @return:
            xy:(-1,2) np.array
        '''
        x = (lonlat[:,0] - self.lon0_bmp) * np.pi / 180 * self.R_a * np.cos(self.lat0_bmp * np.pi / 180)
        y = (lonlat[:,1] - self.lat0_bmp) * np.pi / 180 * self.R_b
        xyz=np.hstack((x.reshape(-1,1),y.reshape(-1,1),np.ones((len(x),1))))#旋转
        xyz=self.R_bmp.dot(xyz.T).T
        return xyz[:,:2]

    def xy_of_draw_to_lonlat(self,xy):
        '''画车道图时的激光雷达m坐标系转换到经纬度
        @param:
            xy:(-1,2)  np.array
        @return:
            lonlat:(-1,2) np.array
        '''
        xyz=np.hstack((xy,np.ones((len(xy),1))))
        origin_xyz=self.R_bmp_inv.dot(xyz.T).T#反旋转
        lon=origin_xyz[:,0]/(self.R_a*np.cos(self.lat0_bmp*np.pi/180))*180/np.pi+self.lon0_bmp
        lat=origin_xyz[:,1]/(self.R_b*np.pi/180)+self.lat0_bmp
        lonlat=np.hstack((lon.reshape(-1,1),lat.reshape(-1,1)))
        return lonlat

def parsingHeadData(recvData):
    data = struct.unpack(">H4BH", recvData[0:8])
    return data

def parseBaseStationData_np(recvData):
    '''
    解析单基站通过udp发送过来的数据  -> numpy
    return  npBoxInfo   单基站传来每个目标的具体信息

            time_Stamp  单基站识别结果时间戳
            equip_id    单基站传来的设备id
            frame_id    单基点云数据帧号
    '''
    time_Stamp2 = time.time()
    # len_recvdata = len(recvData)
    unpack_information = struct.unpack(">6BH", recvData[0:8])
    # len_info= len(unpack_information)
    # len2 = struct.calcsize(">6BH")
    equip_id = struct.unpack(">2B", recvData[8:10])
    # Equiq_id = struct.unpack(">H", recvData[8:10])
    # alg_time = struct.unpack(">H", recvData[10:12])
    frame_id = struct.unpack("4B", recvData[12:16])
    # frame_id = struct.unpack(">H", recvData[14:16])
    time_Stamp_ = struct.unpack(">2I", recvData[16:24])
    time_Stamp = time_Stamp_[0] - 2208988800 + time_Stamp_[1] / 1000000.0
    # logti = struct.unpack(">I",recvData[24:28])
    # lat = struct.unpack(">I",recvData[28:32])
    # angle_north = struct.unpack(">H", recvData[32:34])

    trans_num = struct.unpack("B", recvData[34:35])
    # trans = struct.unpack("5B", recvData[35:40])
    npBoxInfo = None
    try:
        if trans_num[0] > 0:
            npBoxInfo = np.zeros((trans_num[0], 21))
            for i in range(trans_num[0]):
                object_id = struct.unpack('>H',recvData[40+i*36:42+i*36])
                object_type = struct.unpack('B', recvData[42+i*36:43+i*36])
                object_confidence = struct.unpack('B', recvData[43+i*36:44+i*36])
                object_color = struct.unpack('B', recvData[44+i*36:45+i*36])
                object_source = struct.unpack('B', recvData[45+i*36:46+i*36])
                object_symbol = struct.unpack('B', recvData[46+i*36:47+i*36])
                object_nBoxCamId = struct.unpack('B', recvData[47+i*36:48+i*36])
                object_log = struct.unpack('>i', recvData[48+i*36:52+i*36])
                object_lat = struct.unpack('>i', recvData[52+i*36:56+i*36])
                # object_alti = struct.unpack('>h', recvData[56+i*36:58+i*36])
                object_speed = struct.unpack('>H', recvData[58+i*36:60+i*36])
                object_pitch = struct.unpack('>H', recvData[60+i*36:62+i*36])
                object_length = struct.unpack('>H', recvData[62+i*36:64+i*36])
                object_wide = struct.unpack('>H', recvData[64+i*36:66+i*36])
                object_height = struct.unpack('>H', recvData[66+i*36:68+i*36])
                object_center_x = struct.unpack('>H', recvData[68+i*36:70+i*36])
                object_center_y = struct.unpack('>H', recvData[70+i*36:72+i*36])
                object_center_z = struct.unpack('>h', recvData[72+i*36:74+i*36])
                object_hits = struct.unpack('>H', recvData[74+i*36:76+i*36])

                listCenter = []
                listBoxSize = []
                if object_symbol[0] == 1:
                    listCenter.append(object_center_x[0])
                    listCenter.append(-object_center_y[0])
                elif object_symbol[0] == 2:
                    listCenter.append(-object_center_x[0])
                    listCenter.append(object_center_y[0])
                elif object_symbol[0] == 3:
                    listCenter.append(object_center_x[0])
                    listCenter.append(object_center_y[0])
                else:
                    listCenter.append(-object_center_x[0])
                    listCenter.append(-object_center_y[0])
                listCenter.append(object_center_z[0])
                listCenter = [xyz / 100 for xyz in listCenter]
                listBoxSize.append(object_wide[0])
                listBoxSize.append(object_length[0])
                listBoxSize.append(object_height[0])
                listBoxSize = [wlh / 100 for wlh in listBoxSize]
                npBoxInfo[i, :] = np.array([listCenter[0], listCenter[1], listCenter[2],
                                            listBoxSize[0], listBoxSize[1], listBoxSize[2],
                                            object_pitch[0], getPcEnumFromStr(switch_send[object_type[0]]), object_confidence[0]/100,
                                            object_source[0], object_id[0], object_speed[0]/100,
                                            0, object_id[0], object_color[0], listCenter[0],
                                            listCenter[1], object_hits[0], object_nBoxCamId[0],
                                            object_log[0] / 1e7, object_lat[0] / 1e7])
        print("parse time",(time.time() -time_Stamp2)*1000)
        return npBoxInfo, [time_Stamp,equip_id,frame_id]
    except:
        return None, []

def getPcEnumFromStr(strClass):
    for i in range(len(switch_pc)):
        if strClass == switch_pc[i]:
            return i

    return -1

class BsData:
    '''
    用于存储单个基站发送过来的数据
    @param:
        time  新一帧数据到达的系统时间
        new_data 存放当前帧处理数据的列表，只储存最新一帧数据 ,默认为None 赋值后为recvData类的实例
        online_flag 该基站在线状态，在线:True，离线:False
        data_catch 历史缓存数据，用于时间匹配，最大长度为10
    '''
    def __init__(self):
        self.time = 0
        self.new_data = None
        self.online_flag = True
        self.data_catch = []

class recvData:
    '''
    用于存储单基站数据信息
    @param:
        boxInfo  单基站传来每个目标的具体信息，numpy矩阵，每列的具体含义参照全域软件协议文档
        time_Stamp 单基站识别结果时间戳
        equip_id 单基站传来的设备id
        frame_id  单基点云数据帧号
    '''
    def __init__(self):
        self.boxInfo = None
        self.time_Stamp = 0
        self.equip_id = 0
        self.frame_id = 0

class lidarParam(object):
    ''' 用于保存从配置文件中读取的数据 '''
    def __init__(self):

        self.nStationCount = 1
        self.listStationDev = []
        self.listAttachedIp = []
        self.nUdpServicePort = 5555
        self.bHighSpeed = False
        self.bTimeMatch = False

    def getLidarLongitude(self, index):
        return self.listStationDev[index].fStationLongitude

    def getLidarLatitude(self, index):
        return self.listStationDev[index].fStationLatitude

    def getAngleNorthT(self, index):
        return self.listStationDev[index].fAngleNorthT

    def getListAttachedIp(self):
        return self.listAttachedIp

    def getUdpServicePort(self):
        return self.nUdpServicePort

    def getHighSpeed(self):
        return self.bHighSpeed

    def boolTimeMatch(self):
        return self.bTimeMatch

class structStationDev:
    ''' 用于保存从配置文件中获取的雷达的IP地址,以及经纬度航向角信息 '''
    def __init__(self):
        self.strStationSrcIp = ""
        self.fStationLongitude = 116.2869907
        self.fStationLatitude = 40.0516922
        self.fAngleNorthT = 0

def parseResultData_new(recvData):
    len_recvdata = len(recvData)
    unpack_information = struct.unpack(">6BH", recvData[0:8])
    len_info= len(unpack_information)
    len2 = struct.calcsize(">6BH")
    Equiq_id = struct.unpack(">H", recvData[8:10])
    alg_time = struct.unpack(">H", recvData[10:12])
    frame_id = struct.unpack(">H", recvData[14:16])
    time_Stamp_ = struct.unpack(">2I", recvData[16:24])
    time_Stamp = time_Stamp_[0] + time_Stamp_[1] / 1000000.0
    logti = struct.unpack(">I",recvData[24:28])
    lat = struct.unpack(">I",recvData[28:32])
    angle_north = struct.unpack(">H", recvData[32:34])
    trans_num = struct.unpack(">H", recvData[34:36])
    trans = struct.unpack("4B", recvData[38:42])
    sigle_object = ()
    if trans_num[0] > 0:
        npBoxInfo = np.zeros((trans_num[0], 35))
        for i in range(trans_num[0]):
            object_id = struct.unpack('>H',recvData[40+i*38:42+i*38])
            object_type = struct.unpack('B', recvData[42+i*38:43+i*38])
            object_confidence = struct.unpack('B', recvData[43+i*38:44+i*38])
            object_color = struct.unpack('B', recvData[44+i*38:45+i*38])
            object_source = struct.unpack('B', recvData[45+i*38:46+i*38])
            object_laneid = struct.unpack('B', recvData[46+i*38:47+i*38])
            object_basestationId = struct.unpack('B', recvData[47+i*38:48+i*38])
            object_log = struct.unpack('>i', recvData[48+i*38:52+i*38])
            object_lat = struct.unpack('>i', recvData[52+i*38:56+i*38])
            object_alti = struct.unpack('>h', recvData[56+i*38:58+i*38])
            object_speed = struct.unpack('>H', recvData[58+i*38:60+i*38])
            object_pitch = struct.unpack('>H', recvData[60+i*38:62+i*38])
            object_length = struct.unpack('>H', recvData[62+i*38:64+i*38])
            object_wide = struct.unpack('>H', recvData[64+i*38:66+i*38])
            object_height = struct.unpack('>H', recvData[66+i*38:68+i*38])
            object_center_x = struct.unpack('>H', recvData[68+i*38:70+i*38])
            object_center_y = struct.unpack('>H', recvData[70+i*38:72+i*38])
            object_center_z = struct.unpack('>h', recvData[72+i*38:74+i*38])
            object_orgId = struct.unpack('>H', recvData[74+i*38:76+i*38])
            object_singleFrame = struct.unpack('>H', recvData[76+i*38:78+i*38])
            sigle_object = sigle_object + object_id + object_type +object_confidence + object_color + object_source + \
                           object_laneid + object_basestationId + object_log + object_lat + object_alti + object_speed + object_pitch + \
                           object_length + object_wide + object_height + object_center_x + object_center_y + \
                           object_center_z + object_orgId + object_singleFrame
            '''
            deviceID	frameId	timeStamp	lidarLongitude	lidarLatitude	lidarAngle	participantNum
            participants.ID	            participants.type	            participants.confidence	        participants.vehicleColor
            participants.source	        participants.globalLaneNum	    participants.longitude	        participants.latitude
            participants.altitude	    participants.speed	            participants.courseAngle	    participants.length
            participants.width	        participants.height	            participants.xCoordinate	    participants.yCoordinate
            participants.zCoordinate    participants.frameId	        participants.baseStationSource	participants.picLicense
            participants.laneNum	    participants.picId	            participants.stationPicId	    participants.pointsDataId
            participants.licenseColor   participants.isMatchStateFirst	participants.plateFlag	        participants.stationFlag
            '''
            npBoxInfo[i, :] = np.array([Equiq_id[0],frame_id[0],time_Stamp,logti[0],lat[0],angle_north[0],trans_num[0],
                                        object_id[0],getPcEnumFromStr(switch_send[object_type[0]]),object_confidence[0]/100,object_color[0],
                                        object_source[0],object_laneid[0],object_log[0] / 1e7, object_lat[0] / 1e7,
                                        object_alti[0] / 100,object_speed[0],object_pitch[0],object_length[0],
                                        object_wide[0],object_height[0],object_center_x[0] / 100,object_center_y[0] / 100,
                                        object_center_z[0] / 100,frame_id[0],object_basestationId[0],object_singleFrame[0],
                                        object_laneid[0],object_orgId[0],0,0,
                                        0,0,0,0])

        return npBoxInfo, [time_Stamp,Equiq_id[0],frame_id[0]]
    else:
        return None, []

def parseResultData(recvData):
    len_recvdata = len(recvData)
    unpack_information = struct.unpack(">6BH", recvData[0:8])
    len_info= len(unpack_information)
    len2 = struct.calcsize(">6BH")
    Equiq_id = struct.unpack(">H", recvData[8:10])
    alg_time = struct.unpack(">H", recvData[10:12])
    frame_id = struct.unpack(">H", recvData[14:16])
    time_Stamp_ = struct.unpack(">2I", recvData[16:24])
    time_Stamp = time_Stamp_[0] + time_Stamp_[1] / 1000000.0
    logti = struct.unpack(">I",recvData[24:28])
    lat = struct.unpack(">I",recvData[28:32])
    angle_north = struct.unpack(">H", recvData[32:34])
    trans_num = struct.unpack(">H", recvData[34:36])
    trans = struct.unpack("4B", recvData[36:40])
    sigle_object = ()
    if trans_num[0] > 0:
        npBoxInfo = np.zeros((trans_num[0], 35))
        for i in range(trans_num[0]):
            object_id = struct.unpack('>H',recvData[40+i*36:42+i*36])
            object_type = struct.unpack('B', recvData[42+i*36:43+i*36])
            object_confidence = struct.unpack('B', recvData[43+i*36:44+i*36])
            object_color = struct.unpack('B', recvData[44+i*36:45+i*36])
            object_source = struct.unpack('B', recvData[45+i*36:46+i*36])
            object_laneid = struct.unpack('B', recvData[46+i*36:47+i*36])
            object_basestationId = struct.unpack('B', recvData[47+i*36:48+i*36])
            object_log = struct.unpack('>i', recvData[48+i*36:52+i*36])
            object_lat = struct.unpack('>i', recvData[52+i*36:56+i*36])
            object_alti = struct.unpack('>h', recvData[56+i*36:58+i*36])
            object_speed = struct.unpack('>H', recvData[58+i*36:60+i*36])
            object_pitch = struct.unpack('>H', recvData[60+i*36:62+i*36])
            object_length = struct.unpack('>H', recvData[62+i*36:64+i*36])
            object_wide = struct.unpack('>H', recvData[64+i*36:66+i*36])
            object_height = struct.unpack('>H', recvData[66+i*36:68+i*36])
            object_center_x = struct.unpack('>H', recvData[68+i*36:70+i*36])
            object_center_y = struct.unpack('>H', recvData[70+i*36:72+i*36])
            object_center_z = struct.unpack('>h', recvData[72+i*36:74+i*36])
            object_orgId = struct.unpack('>H', recvData[74+i*36:76+i*36])
            sigle_object = sigle_object + object_id + object_type +object_confidence + object_color + object_source + \
                           object_laneid + object_basestationId + object_log + object_lat + object_alti + object_speed + object_pitch + \
                           object_length + object_wide + object_height + object_center_x + object_center_y + \
                           object_center_z + object_orgId
            '''
            deviceID	frameId	timeStamp	lidarLongitude	lidarLatitude	lidarAngle	participantNum
            participants.ID	            participants.type	            participants.confidence	        participants.vehicleColor
            participants.source	        participants.globalLaneNum	    participants.longitude	        participants.latitude
            participants.altitude	    participants.speed	            participants.courseAngle	    participants.length
            participants.width	        participants.height	            participants.xCoordinate	    participants.yCoordinate
            participants.zCoordinate    participants.frameId	        participants.baseStationSource	participants.picLicense
            participants.laneNum	    participants.picId	            participants.stationPicId	    participants.pointsDataId
            participants.licenseColor   participants.isMatchStateFirst	participants.plateFlag	        participants.stationFlag
            '''
            npBoxInfo[i, :] = np.array([Equiq_id[0],frame_id[0],time_Stamp,logti[0],lat[0],angle_north[0],trans_num[0],
                                        object_id[0],getPcEnumFromStr(switch_send[object_type[0]]),object_confidence[0]/100,object_color[0],
                                        object_source[0],object_laneid[0],object_log[0] / 1e7, object_lat[0] / 1e7,
                                        object_alti[0] / 100,object_speed[0],object_pitch[0],object_length[0],
                                        object_wide[0],object_height[0],object_center_x[0] / 100,object_center_y[0] / 100,
                                        object_center_z[0] / 100,frame_id[0],object_basestationId[0],0,
                                        object_laneid[0],0,0,0,
                                        0,0,0,0])

        return npBoxInfo, [time_Stamp,Equiq_id[0],frame_id[0]]
    else:
        return None, []

def str2num(strinput):
    if strinput == 'F':
        return 15
    elif strinput == 'E':
        return 14
    elif strinput == 'D':
        return 13
    elif strinput == 'C':
        return 12
    elif strinput == 'B':
        return 11
    elif strinput == 'A':
        return 10
    else:
        return int(strinput)

def get_dict_from_csv(strFilePath):
    '''
    Args:
        strFilePath:在线保存的发送结果文件路径，为csv文件
    Returns:
        dictFrame:以帧号为值的字典，值为np数组，格式见解析代码
        nStartFrameId:起始帧号
        nEndFrameId:结束帧号
    '''
    if judge_exist(strFilePath):
        npfile = np.loadtxt(strFilePath, delimiter=',',dtype=str)
        try:
            npFrame = None
            nStartFrameId = None
            dictFrame = {}
            listFrameId = []
            for i in range(1,npfile.shape[0]):
                # nNowFrameId = int(npfile[i,1])
                nNowFrameId =int(float(npfile[i,1]))
                if nNowFrameId not in listFrameId:
                    listFrameId.append(nNowFrameId)
                    dictFrame[nNowFrameId] = npfile[i,:].reshape((1,npfile.shape[1]))
                dictFrame[nNowFrameId] = np.concatenate((dictFrame[nNowFrameId],npfile[i,:].reshape((1,npfile.shape[1]))),axis=0)
                if nStartFrameId is None:
                    nStartFrameId = nNowFrameId
            nEndFrameId = max(listFrameId)
            return dictFrame, nStartFrameId, nEndFrameId
        except:
            return {}, 0, 0
    else:
        return {}, 0, 0

def get_dict_from_dir(strFilePath):
    '''
    Args:
        strFilePath:在线保存的发送结果文件路径，为csv文件
    Returns:
        dictFrame:以帧号为值的字典，值为np数组，格式见解析代码
        nStartFrameId:起始帧号
        nEndFrameId:结束帧号
    '''
    if judge_exist(strFilePath):
        try:
            strDataDir = os.path.dirname(strFilePath)
            dir_list = os.listdir(strDataDir)
            listDataFile = []
            if len(dir_list) > 1:
                for file in dir_list:
                    nFrameIdTemp = int(file[7:file.index('csv')-1])
                    path_file = os.path.join(strDataDir,file)
                    if os.path.isfile(path_file) and "orgData" in file:
                        listDataFile.append(nFrameIdTemp)
            if len(listDataFile) > 1:
                listDataFile = sorted(listDataFile)
            dictFrame = {}
            nStartFrameId = listDataFile[0]
            nEndFrameId = listDataFile[-1]
            for nFrameIdTemp in listDataFile:
                strfilename = os.path.join(strDataDir,'orgData{}.csv'.format(nFrameIdTemp))
                npStationdata0 = np.loadtxt(strfilename, delimiter=',',dtype=float)
                dictFrame[nFrameIdTemp] = npStationdata0

            return dictFrame, nStartFrameId, nEndFrameId
        except:
            return {}, 0, 0
    else:
        return {}, 0, 0

def get_dict_from_txt(strFilePath:str):
    '''
    Args:
        strFilePath:在线保存的发送结果文件路径，为txt文件
    Returns:
        dictFrame:以帧号为值的字典，值为np数组，格式见解析代码
        nStartFrameId:起始帧号
        nEndFrameId:结束帧号
    '''
    if judge_exist(strFilePath):
        try:
            with open(strFilePath,'r') as f:
                listFrame = f.readlines()
                nStartFrameId = None
                nFrameIndex = 0
                dictFrame = {}
                for Frame in listFrame:
                    try:

                        listOneFrame = Frame.split(':')
                        strResult = listOneFrame[-1]
                        strResult = strResult.replace(' ','')

                        byteResult = bytes()
                        for i in range(math.floor(len(strResult)/2)):
                            strTemp = strResult[2*i:+2*i+2]
                            num = str2num(strTemp[0])*16+str2num(strTemp[1])
                            byteResult = byteResult + struct.pack('B',num)
                        npBoxInfo, listDevInfo = parseResultData(byteResult)
                        print("11")
                        if nStartFrameId is None:
                            nStartFrameId = listDevInfo[2]
                        nNowFrameId = nStartFrameId + nFrameIndex
                        # nNowFrameId = listDevInfo[2]
                        dictFrame[nNowFrameId] = npBoxInfo
                        nFrameIndex += 1
                    except:
                        continue
                nEndFrameId = nNowFrameId
                return dictFrame, nStartFrameId, nEndFrameId
        except:
            return {}, 0, 0
    else:
        return {}, 0, 0

def get_dict_from_json(strFilePath:str):
    '''
    Args:
        strFilePath:在线保存的发送结果文件路径，为json文件
    Returns:
        dictFrame:以帧号为值的字典，值为np数组，格式见解析代码
        nStartFrameId:起始帧号
        nEndFrameId:结束帧号
    '''
    if judge_exist(strFilePath):
        try:
            with open(strFilePath,'r') as jsonFile:
                listJsonData = jsonFile.readlines()
                dictFrame = {}
                nStartFrameId = 0
                nFrameCount = 0
                for jsonData in listJsonData:
                    dictOneFrame = json.loads(jsonData)
                    if nStartFrameId == 0:
                        nStartFrameId = dictOneFrame['frameID']
                    npOneFrame = np.zeros((dictOneFrame['participantNum'],35))
                    npOneFrame[:,0:7] = np.array([dictOneFrame['deviceID'],
                                                  dictOneFrame['frameID'],
                                                  0,
                                                  dictOneFrame['lidarLongitude'],
                                                  dictOneFrame['lidarLatitude'],
                                                  dictOneFrame['lidarAngle'],
                                                  dictOneFrame['participantNum'],
                                                  ])
                    listOneFrame = dictOneFrame['participants']
                    for i in range(len(listOneFrame)):
                        dictOneTarget = listOneFrame[i]
                        npOneFrame[i,7:35] = np.array([dictOneTarget['ID'],dictOneTarget['type'],dictOneTarget['confidence'],dictOneTarget['vehicleColor'],
                                                       dictOneTarget['source'],dictOneTarget['globalLaneNum'],dictOneTarget['longitude'],dictOneTarget['latitude'],
                                                       dictOneTarget['altitude'],dictOneTarget['speed'],dictOneTarget['courseAngle'],dictOneTarget['length'],
                                                       dictOneTarget['width'],dictOneTarget['height'],dictOneTarget['xCoordinate'],dictOneTarget['yCoordinate'],
                                                       dictOneTarget['zCoordinate'],dictOneFrame['frameID'],dictOneTarget['baseStationSource'],0,
                                                       dictOneTarget['laneNum'],0,0,0,
                                                       dictOneTarget['licenseColor'],0,0,0])
                    dictFrame[nStartFrameId + nFrameCount] = npOneFrame
                    nFrameCount += 1
                nEndFrameId = nStartFrameId + nFrameCount - 1
            return dictFrame, nStartFrameId, nEndFrameId
        except:
            return {}, 0, 0
    else:
        return {}, 0, 0



