from Common.interface import Interface, UsrSysState
from Common.Interface.DataBus import DataBus
from Common.CommonDefine import *
from Alg.sort_0428 import Sort
from Alg.all_track import rotate_box, fusion_box, get_min_max, fix_cluster, cal_box_lane, cal_map_angle
import Alg.core.box_np_ops as box_np_ops
from track_common import *
from Common.Protocol import parseNdarrayofByte, getByteofNdarray


MAX_FRAME_ID = 1e10  # 帧号最大值，超过该值，则重新计数

strExePath = get_exe_path()
# #####*****  加载npy文件  *****#####
bmp_img = np.load(strExePath + '/Configs/Lane/lane_new.npy')
left_center = np.load(strExePath + '/Configs/Lane/left_center.npy')
medium_center = np.load(strExePath + '/Configs/Lane/medium_center.npy')
right_center = np.load(strExePath + '/Configs/Lane/right_center.npy')
left_center = left_center[left_center[:, 0].argsort()]
medium_center = medium_center[medium_center[:, 0].argsort()]
right_center = right_center[right_center[:, 0].argsort()]


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

def get_lidarDev_param(s_param_path: str) -> lidarParam:
    """根据配置文件获取雷达参数"""
    stLidarParam = lidarParam()
    try:
        if not judge_exist(s_param_path):
            return stLidarParam
        tree = ET.parse(s_param_path)
        root = tree.getroot()
        if stLidarParam is not None:
            stLidarParam.nStationCount = int(root[2][0].text)
            listAttachedIp = []
            for tag in root[2][1]:
                stStationDev = structStationDev()
                stStationDev.strStationSrcIp = tag[0].text
                if stStationDev.strStationSrcIp is not None:
                    listAttachedIp.append(stStationDev.strStationSrcIp)
                else:
                    continue
                stStationDev.fStationLatitude = float(tag[1].text)
                stStationDev.fStationLongitude = float(tag[2].text)
                stStationDev.fAngleNorthT = float(tag[3].text)

                stLidarParam.listStationDev.append(stStationDev)
            stLidarParam.listAttachedIp = listAttachedIp

            stLidarParam.nUdpServicePort = int(root[2][2].text)
            stLidarParam.bHighSpeed = bool(int(root[2][3].text))
            stLidarParam.bTimeMatch = bool(int(root[2][4].text))
        return stLidarParam
    except:
        return stLidarParam

def get_alg_param(s_param_path: str) -> algParam:
    """根据配置文件获取算法参数"""
    stparm = algParam()
    try:
        if not judge_exist(s_param_path):
            return stparm
        tree = ET.parse(s_param_path)
        root = tree.getroot()
        stparm.fConeThreshold = float(root[2][0].text)
        stparm.DisplayFrames = int(root[2][1].text)
        stparm.nMinHits = int(root[2][2].text)
        stparm.Horizon_threshoud = float(root[2][3].text)
        stparm.Vertical_threshoud = float(root[2][4].text)
        stparm.MaxLifetime = float(root[2][5].text)
        return stparm
    except:
        return stparm

class TestMod(Interface):
    def __init__(self):
        self.s_Parampath = './Configs/track_Param.xml'
        Interface.__init__(self, self.s_Parampath)
        self.s_Parampath2 = './Configs/read_Param.xml'


    def usr_process(self, data_bus_server):
        """重写方法"""

        self.log.info("track process start, pid={} ".format(os.getpid()))
        # 从配置文件中获取算法参数
        st_alg_param = get_alg_param(self.s_Parampath)


        sys_status_channel = 'track_data'    # 数据发送频道名称
        data_bus_server = DataBus(strHost)

        nFrameIdTemp = 0

        # #####*****  加载全域车道中心线信息  *****#####
        global left_center, right_center, medium_center, bmp_img
        mot_tracker_merge = Sort(left_center, medium_center, right_center, bmp_img, st_alg_param.DisplayFrames, st_alg_param.nMinHits)

        # 功能接口状态类，用于接收接口类中的系统状态，同时上报功能进程的相关信息
        sys_state = UsrSysState(self.q_sys_state, self.q_model_state, self.log)
        # 获取用户订阅的subscriber列表，每个频道对应一个消息，为列表中的一个
        list_sub = data_bus_server.get_subscriber(self.list_channel)
        if sys_state.get_sys_online():
            while sys_state.get_sys_state():
                try:
                    try:
                        t0 = time.time()
                        list_msg = list_sub[0].parse_response(False, 0.01)
                        channel = str(list_msg[1], encoding="utf-8")
                        self.log.info('step 1:get data from {}'.format(channel))
                        trunAlg_start = time.time()
                    except:
                        self.log.warning('step 0:waitting for data')
                        continue

                    try:
                        t1 = time.time()
                        recv_data = json.loads(list_msg[2])

                        npStationdata0 = parseNdarrayofByte(recv_data["targetInfo"]).reshape((-1,19))
                        basestation_info = recv_data["baseStation"]

                        self.log.info('step 2:parse {} success,use time:{} ms'.format((channel),round((time.time() - t1)*1000,3)))
                    except:
                        self.log.error('step 2:parse wrong...')
                        npStationdata0 = None

                        continue
                    # print(npStationdata0)

                    tWait = time.time()
                    if npStationdata0 is not None:
                        nFrameIdTemp = int((nFrameIdTemp + 1) % MAX_FRAME_ID)
                        # print("nFrameIdTemp",nFrameIdTemp)
                        self.log.info('nFrameIdTemp: {}'.format(nFrameIdTemp))
                        # trackers_cov_old_for_sort = npStationdata0[:, [0, 1, 3, 4, 6, 2, 5, 7, 10, 9, 8, 11, 12, 13,14,15,16,17,18,19]]
                        # trackers_cov_old_for_sort[:, 4] = trackers_cov_old_for_sort[:, 4] * np.pi / 180
                        trackers_cov_old_for_sort = npStationdata0
                        # box_with_min_max, box_corners = get_min_max(trackers_cov_old_for_sort)

                        # #####*****  2021-0509 my add  *****#####
                        # #####*****  在去重前进行车道航向角的规定  *****#####
                        add_dimension = np.zeros((trackers_cov_old_for_sort.shape[0], 1))
                        trackers_cov_old_for_sort = np.concatenate((trackers_cov_old_for_sort, add_dimension), axis=1)
                        trackers_cov_old_for_sort = cal_box_lane(bmp_img, trackers_cov_old_for_sort)
                        # #####*****  flag是标志位, 1表示第7列是航向角, 0表示第5列是航向角  *****#####
                        flag = 1
                        trackers_cov_old_for_sort = cal_map_angle(trackers_cov_old_for_sort, left_center, medium_center, right_center, flag)

                        box_with_min_max, box_corners = get_min_max(trackers_cov_old_for_sort, st_alg_param.fConeThreshold)
                        mathch, un_match1, un_match2, cost_matrix = fusion_box(box_corners, box_corners, 0)

                        trackers_cov_old_for_sort = box_with_min_max
                        save_indice = []
                        delete_indice = []
                        for i in range(mathch.shape[0]):
                            match_row_indice = mathch[i][0]
                            match_column_indice = mathch[i][1]
                            if mathch[i][0] in delete_indice:
                                continue
                            if trackers_cov_old_for_sort[match_row_indice][17] < trackers_cov_old_for_sort[match_column_indice][17]:
                                # # #####*****  当目标的航向角不与坐标轴平行或者垂直时  *****#####
                                trackers_cov_old_for_sort[match_row_indice][7] = trackers_cov_old_for_sort[match_column_indice][7]
                                save_indice.append(match_row_indice)
                                delete_indice.append(match_column_indice)
                        trackers_cov_old_for_sort_new = trackers_cov_old_for_sort[save_indice, :]

                        # # #####*****  但是对于场景中有的误检在目标旁边，采用距离去除  *****#####
                        un_match_object = trackers_cov_old_for_sort[un_match1.tolist(), :]
                        un_match_object = fix_cluster(un_match_object, 1)
                        new_trackers_cov_old_for_sort = np.concatenate((trackers_cov_old_for_sort_new, un_match_object), axis=0)

                        # print('my Time:', round((time.time()-tWait) *1000, 2), 'ms')
                        npStationdata0 = new_trackers_cov_old_for_sort
                        #tracker###

                        # #####*****  2021-0509 my add  *****#####
                        box_final = np.zeros((len(npStationdata0), 17))
                        box_final[:,[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,13,14,15,16]]=npStationdata0[:, [0, 1, 3, 4, 6, 2, 5, 7, 8, 9, 12, 13, 14,15,16,18,19]]

                        self.log.info('step 3:fusion time:{}ms'.format(round((time.time()-tWait) *1000, 2)))
                        tWait = time.time()
                        trackers_all = mot_tracker_merge.update(box_final,st_alg_param)  # 将所有的跟踪到的物体都使用其进行跟踪
                        self.log.info('step 4:tracker time:{}ms'.format(round((time.time()-tWait) *1000, 2)))

                        trackers_covall = np.zeros((trackers_all.shape[0], 20))
                        trackers_covall[:, [0, 1, 3, 4, 6, 2, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,17,18,19]] = trackers_all[:, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,17,18,19]]
                        trackers_covall[:, 6] = trackers_covall[:, 6] * PI_rad

                        box_datall = box_np_ops.center_to_corner_box3d(
                            trackers_covall[:, :3],
                            trackers_covall[:, 3:6],
                            trackers_covall[:, 6],
                            origin=[0.5, 0.5, 0.5],
                            axis=2)

                        boxes_corners_trackersall_ = box_datall.reshape(-1, 24)
                        trackers_cov_ = np.hstack((trackers_covall, boxes_corners_trackersall_))
                        trunAlg = math.floor((time.time() - trunAlg_start)*1000)
                        self.log.info('step 5:trunAlg time:{}ms'.format(trunAlg))

                        dict_result_send = {}
                        dict_result_send["deviceId"] = 158
                        dict_result_send["algorithmDur"] = trunAlg
                        dict_result_send["globalFrameNo"] = nFrameIdTemp
                        dict_result_send["globalTimeStamp"] = time.time()
                        dict_result_send["participantNum"] = trackers_cov_.shape[0]
                        dict_result_send["e1FrameParticipant"] = getByteofNdarray(trackers_cov_)
                        dict_result_send["stationNum"] = len(basestation_info)
                        dict_result_send["sourceInfoList"] = basestation_info
                        self.log.info('e1FrameParticipant: {}'.format(trackers_cov_.shape[0]))
                        bytesSend = json.dumps(dict_result_send)
                        try:
                            data_bus_server.publish(sys_status_channel, bytesSend)
                            self.log.info('step 6:publish result_data success')
                        except:
                            self.log.error('step 6:publish result_data failed')

                        self.log.info('step 7:the whole track time use {} ms!'.format(round((time.time() - t0)*1000,3) ))
                        # time.sleep(1)
                    else:
                        print("npStationdata0 is None")
                        continue

                except:
                    self.log.error('unexpected error')
                    pass
        else:
            try:
                self.log.info("track offline process start, pid={} ".format(os.getpid()))
                st_lidar_param = get_lidarDev_param(self.s_Parampath2)
                fLongitudeT = st_lidar_param.getLidarLongitude(0)
                fLatitudeT = st_lidar_param.getLidarLatitude(0)
                fAngleNorthT = st_lidar_param.getAngleNorthT(0)
                stTransform = Lon_Lat_Meter_Transform(fLongitudeT, fLatitudeT, fAngleNorthT)
                while sys_state.get_sys_state():
                    try:
                        t0 = time.time()
                        list_msg = list_sub[0].parse_response(False, 0.01)
                        channel = str(list_msg[1], encoding="utf-8")
                        self.log.info('step 1:get data from {}'.format(channel))
                        trunAlg_start = time.time()
                        t1 = time.time()
                        recv_data = json.loads(list_msg[2])

                        npStationdata0 = parseNdarrayofByte(recv_data["targetInfo"]).reshape((-1,35))
                        basestation_info = recv_data["baseStation"]
                        self.log.info('step 2:parse {} success,use time:{} ms'.format((channel),round((time.time() - t1)*1000,3)))
                    except:
                        continue

                    e1FrameParticipant = parse_offline_data(npStationdata0,stTransform,fAngleNorthT)

                    dict_result_send = {}
                    dict_result_send["deviceId"] = 158
                    dict_result_send["algorithmDur"] = 0
                    dict_result_send["globalFrameNo"] = 1
                    dict_result_send["globalTimeStamp"] = time.time()
                    dict_result_send["participantNum"] = npStationdata0.shape[0]
                    dict_result_send["e1FrameParticipant"] = getByteofNdarray(e1FrameParticipant)
                    dict_result_send["stationNum"] = len(basestation_info)
                    dict_result_send["sourceInfoList"] = basestation_info
                    self.log.info('e1FrameParticipant: {}'.format(e1FrameParticipant.shape[0]))
                    bytesSend = json.dumps(dict_result_send)
                    try:
                        data_bus_server.publish(sys_status_channel, bytesSend)
                        self.log.info('step 3:publish result_data success')
                    except:
                        self.log.error('step 3:publish result_data failed')

            except:
                self.log.error('track offline process wrong...')
                npStationdata0 = None


        self.log.error('usr_process {} exit...'.format(self.log.name))


def parse_offline_data(npFrameTarget,stTransform,fAngle):
    '''
        npBoxInfo[i, :] = np.array([Equiq_id[0], frame_id[0], time_Stamp, logti[0], lat[0],     0-4
                                    angle_north[0],trans_num[0],object_id[0], getPcEnumFromStr(switch_send[object_type[0]]),object_confidence[0]/100,    5-9
                                    object_color[0],object_source[0],object_laneid[0],object_log[0] / 1e7, object_lat[0] / 1e7,          10-14
                                    object_alti[0] / 100, object_speed[0],object_pitch[0],object_length[0],object_wide[0],             15- 19
                                    object_height[0],object_center_x[0] / 100,object_center_y[0] / 100, object_center_z[0] / 100, frame_id[0],   20- 24
                                    object_basestationId[0],object_singleFrame[0], object_laneid[0],object_orgId[0],0,0, 0,0,0,0])       25 --
    '''

    trackers_cov_ =npFrameTarget[1:,[21,22,23,19,18,20,17,8,16,7,9,11,25,28,10,13,14,29,27,26]].astype(np.float64)



    return trackers_cov_


if __name__ == '__main__':
    testMod = TestMod()
    testMod.join()
    testMod.log.error('main exit...')
    sys.exit(0)
