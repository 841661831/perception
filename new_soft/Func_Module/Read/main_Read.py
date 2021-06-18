from Common.interface import Interface, UsrSysState
from Common.Interface.DataBus import DataBus
from PyQt5.QtCore import *
from read_common import *
from Common.Protocol import getByteofNdarray

version = 'mainControl_v1.0_20210520'

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

class TestMod(Interface):
    def __init__(self):
        self.s_Parampath = './Configs/read_Param.xml'
        Interface.__init__(self, self.s_Parampath)


    def usr_process(self, data_bus_server):
        """重写方法"""
        # 功能接口状态类，用于接收接口类中的系统状态，同时上报功能进程的相关信息
        sys_state = UsrSysState(self.q_sys_state, self.q_model_state, self.log)

        if sys_state.get_sys_online():
            self.log.info("read online process start, pid={} ".format(os.getpid()))
            # 从配置文件中获取雷达参数
            st_lidar_param = get_lidarDev_param(self.s_Parampath)


            sys_status_channel = 'src_data'    # 数据发送频道名称
            data_bus_server = DataBus(strHost)

            # dict_src_data为保存接收各个基站数据的字典，键为每个基站的序号，
            # 值为对应基站的数据,为BsData类的对象,BsData类保存了数据的各种信息
            dict_src_data = {}

            listBaseStationId = []
            listNorthAngleT = []
            for i in range(len(st_lidar_param.getListAttachedIp())):
                dict_src_data["{}".format(i)] = BsData()
                listBaseStationId.append(0)
                listNorthAngleT.append(st_lidar_param.getAngleNorthT(i))

            listSortNorthAngleT = sorted(listNorthAngleT)

            for i in range(len(st_lidar_param.getListAttachedIp())):
                if st_lidar_param.getHighSpeed():
                    listBaseStationId[i] = i + 1
                else:
                    listBaseStationId[i] = listSortNorthAngleT.index(listNorthAngleT[i]) + 1

            timeForce10Fps = QTime()  # TODO
            timeForce10Fps.start()  # TODO
            timeExcept = QTime()   # TODO
            send_data = SendData()

            getCoordinateFlag = True
            bGetBS1data = False
            BUFSIZE = 4096
            nPort = st_lidar_param.getUdpServicePort()
            ip_port = ('', nPort)
            server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # udp协议
            server.bind(ip_port)
            self.log.info("bind ip: {}".format(ip_port))

            listAttachedIp = st_lidar_param.getListAttachedIp()
            fLongitudeT = st_lidar_param.getLidarLongitude(0)
            fLatitudeT = st_lidar_param.getLidarLatitude(0)
            fAngleNorthT = st_lidar_param.getAngleNorthT(0)
            stTransform = Lon_Lat_Meter_Transform(fLongitudeT, fLatitudeT, fAngleNorthT)

            bTimeMatch = st_lidar_param.boolTimeMatch()


            while sys_state.get_sys_state():
                try:
                    t0 = time.time()
                    data, client_addr = server.recvfrom(BUFSIZE)
                    self.log.info("step 1: get data from {} success, time is {} ".format((client_addr), getTimeStamp(time.time())))

                    tfirst = time.time()
                    headData = parsingHeadData(data)
                    if headData is None:
                        continue
                    if headData[2] != 0xA0:
                        if client_addr[0] in listAttachedIp and headData[2] == 0xE1 and headData[3] == 0x02:
                            i = listAttachedIp.index(client_addr[0])
                            t3 = time.time()
                            npBoxInfo, listDevInfo = parseBaseStationData_np(data)
                            if npBoxInfo is not None:
                                self.log.info("step 2: parse BaseStationData success, use {} ms".format(round((time.time()- t3)*1000,3)))
                                if getCoordinateFlag:
                                    npBoxInfo[:,6] = (npBoxInfo[:,6] + 180 - fAngleNorthT + 360) % 360
                                    npBoxInfo[:, 0:2] = stTransform.lonlat_to_xy_of_draw(npBoxInfo[:, 19:21])
                                else:
                                    npBoxInfo[:,6] = (npBoxInfo[:,6] + 180 - st_lidar_param.getAngleNorthT(i) + 360) % 360
                            else:
                                self.log.warning('parse BaseStationData_np failed...npBoxInfo is None! ')
                                continue
                            ts = time.time()
                            dict_src_data["{}".format(i)].time = time.time()
                            if dict_src_data["{}".format(i)].new_data is None:
                                new_data = recvData()
                                new_data.boxInfo = npBoxInfo[:,:19]
                                new_data.time_Stamp = listDevInfo[0]
                                new_data.equip_id = listDevInfo[1]
                                new_data.frame_id = listDevInfo[2]
                                dict_src_data["{}".format(i)].new_data = new_data
                            else:
                                dict_src_data["{}".format(i)].new_data.boxInfo = npBoxInfo[:,:19]
                                dict_src_data["{}".format(i)].new_data.time_Stamp = listDevInfo[0]
                                dict_src_data["{}".format(i)].new_data.equip_id = listDevInfo[1]
                                dict_src_data["{}".format(i)].new_data.frame_id = listDevInfo[2]

                            dict_src_data["{}".format(i)].online_flag = True
                            if bTimeMatch:
                                #=====================time match=====================#
                                if i == 0:
                                    bGetBS1data = True
                                data_put = recvData()
                                data_put.boxInfo = npBoxInfo[:,:19]
                                data_put.frame_id = listDevInfo[2]
                                data_put.time_Stamp = listDevInfo[0]
                                data_put.equip_id = listDevInfo[1]
                                dict_src_data["{}".format(i)].data_catch.append(data_put)
                                if len(dict_src_data["{}".format(i)].data_catch) > 10:
                                    dict_src_data["{}".format(i)].data_catch.pop(0)
                                #=====================time match=====================#

                            self.log.info("step 3:  put into dict_src_data over,time is : {}".format((time.time() - ts) * 1000))

                            if (time.time() - dict_src_data["0"].time ) > 0.5:
                                dict_src_data["0"].online_flag = False
                        else:
                            self.log.error("reached ip address is not in listAttachedIp")
                            continue
                    if bTimeMatch:
                        self.log.info("use Time Match ! ")
                        #=====================time match=====================#
                        ngetFlag = False
                        if dict_src_data["0"].online_flag and bGetBS1data and len(dict_src_data["0"].data_catch) > 2:
                            bGetBS1data = False
                            tBSData1 = dict_src_data["0"].data_catch[-3].time_Stamp
                            dict_src_data["0"].new_data = dict_src_data["0"].data_catch[-3]
                            for i in range(len(dict_src_data)):
                                if i == 0 or len(dict_src_data["{}".format(i)].data_catch) == 0:
                                    continue
                                listTError = [abs(listBSData.time_Stamp - tBSData1) for listBSData in dict_src_data["{}".format(i)].data_catch]
                                minTError = min(listTError)
                                self.log.info("BS{} time error is {},index is {}".format(i,minTError,listTError.index(minTError)))
                                if minTError < 0.05:
                                    dict_src_data["{}".format(i)].new_data = copy.deepcopy(dict_src_data["{}".format(i)].data_catch[listTError.index(minTError)])
                                else:
                                    dict_src_data["{}".format(i)].new_data = None
                            timeForce10Fps.restart()
                            ngetFlag = True
                        elif not dict_src_data["0"].online_flag and timeForce10Fps.elapsed() > 100:
                            timeForce10Fps.restart()
                            ngetFlag = True
                        #=====================time match=====================#
                    else:
                        self.log.info("use Newest Frame ! ")
                        ngetFlag = False
                        if dict_src_data["0"].new_data is not None:
                            timeForce10Fps.restart()
                            ngetFlag = True
                        elif not dict_src_data["0"].online_flag and timeForce10Fps.elapsed() > 100:
                            timeForce10Fps.restart()
                            ngetFlag = True

                    # ngetFlag标志为True，往下进行，否则continue
                    if not ngetFlag:
                        if timeExcept.elapsed() > 0:
                            timeExcept.restart()
                        continue

                except:
                    self.log.error("except! udpServerProcess!")
                    pass

                # ngetFlag标志为True，往下进行 往下执行
                try:
                    self.log.info("step 4: start concat, time is : {}".format(getTimeStamp(time.time())))
                    t1 = time.time()
                    list_matrix_base = []
                    index =0
                    # nFrameID = None
                    # nEqupId = None
                    list_data = []
                    for src_data in dict_src_data.values():
                        # 当前基站处于离线状态或者当前帧无数据:
                        if not src_data.online_flag or src_data.new_data is None:
                            if (time.time() - src_data.time) > 0.5 and src_data.new_data is None:
                                self.log.warning("station No:{} has no Data".format(index + 1))
                                if src_data.online_flag:
                                    src_data.online_flag = False
                                    self.log.warning("station No:{} is  offline".format(index + 1))

                            # 保存单基站数据发送给平台 以基站号作为键将信息保存在字典，用于发送信息时根据基站号取出相应数据
                            send_data.nSingleFrame["{}".format(listBaseStationId[index])] = 0
                            send_data.nSingleStamp["{}".format(listBaseStationId[index])] = time.time()
                            send_data.nSingleSource["{}".format(listBaseStationId[index])] = listBaseStationId[index]
                            send_data.recvTime["{}".format(listBaseStationId[index])] = time.time()
                            send_data.stationState["{}".format(listBaseStationId[index])] = 1
                            if not src_data.online_flag:
                                send_data.stationState["{}".format(listBaseStationId[index])] = 2

                            index += 1
                            list_matrix_base.append(None)
                            continue

                        # if nFrameID is None:
                        #     nFrameID = src_data.new_data.frame_id
                        #     nEqupId = src_data.new_data.equip_id
                        #     send_data.nEqupId = nEqupId

                        # 当前基站为第一号基站(入口基站)
                        # if index == 0:
                        #     # 保存入口基站信息发送给平台 以基站号作为键将信息保存在字典，用于发送信息时根据基站号取出相应数据
                        #     send_data.n0Frame = (src_data.new_data.frame_id[2]*256 + src_data.new_data.frame_id[3])
                        #     send_data.n0TimeStamp = src_data.new_data.time_Stamp

                        # 保存单基站数据发送给平台 以基站号作为键将信息保存在字典，用于发送信息时根据基站号取出相应数据
                        send_data.nSingleFrame["{}".format(listBaseStationId[index])] = (src_data.new_data.frame_id[2]*256
                                                                                         + src_data.new_data.frame_id[3])
                        send_data.nSingleStamp["{}".format(listBaseStationId[index])] = src_data.new_data.time_Stamp
                        send_data.nSingleSource["{}".format(listBaseStationId[index])] = listBaseStationId[index]
                        send_data.recvTime["{}".format(listBaseStationId[index])] = src_data.time
                        send_data.stationState["{}".format(listBaseStationId[index])] = 0

                        data_base = src_data.new_data.boxInfo
                        data_base[:, 12] = listBaseStationId[index]
                        list_matrix_base.append(src_data.new_data.boxInfo)
                        list_data.append(index)
                        index += 1


                    ##### 不同路口ip转发的数据进行坐标转换 ####
                    npStationdata0 = None
                    for i in range(len(list_matrix_base)):
                        if list_matrix_base[i] is None:
                            continue

                        npStationdata1 = list_matrix_base[i]
                        if npStationdata0 is None:
                            npStationdata0 = npStationdata1
                            continue
                        else:
                            npStationdata0 = np.concatenate((npStationdata0, npStationdata1), axis=0)

                    self.log.info("step 5: concatenate over ,use time: {} ms".format(round((time.time()- t1)*1000,3)))

                    # print(npStationdata0)
                    if npStationdata0 is None:
                        self.log.warning("all baseStation has no data ")
                        continue

                    ##### 拼接转发数据，准备转发 ####
                    tm = time.time()
                    BaseStationInfoList = []
                    for i in send_data.nSingleFrame.keys():
                        dict_tmp = {}
                        dict_tmp["baseStationSource"] = send_data.nSingleSource["{}".format(i)]
                        dict_tmp["state"] = send_data.stationState["{}".format(i)]
                        dict_tmp["sourceframeNo"] = send_data.nSingleFrame["{}".format(i)]
                        dict_tmp["sourcetimeStamp"] = getTimeStamp(send_data.nSingleStamp["{}".format(i)])
                        dict_tmp["sourceRecievetimeStamp"] = getTimeStamp(send_data.recvTime["{}".format(i)])
                        dict_tmp["sourcelongitude"] = round(st_lidar_param.getLidarLongitude(int(i) - 1), 7)
                        dict_tmp["sourcelatitude"] = round(st_lidar_param.getLidarLatitude(int(i) - 1), 7)
                        dict_tmp["sourceangle"] = int(st_lidar_param.getAngleNorthT(int(i) - 1))
                        BaseStationInfoList.append(dict_tmp)
                    npStationdata = getByteofNdarray(npStationdata0)
                    # entrance_info = [send_data.n0Frame, send_data.n0TimeStamp ]
                    dict_send_message = {}
                    dict_send_message["targetInfo"] = npStationdata
                    dict_send_message["baseStation"] = BaseStationInfoList
                    # dict_send_message["entrance_info"] = entrance_info

                    req = json.dumps(dict_send_message)
                    # req = npStationdata0.tobytes()
                    data_bus_server.publish(sys_status_channel, req)
                    self.log.info("step 6: send message ,use time: {} ms".format(round((time.time()- tm)*1000,3)))

                    # 将使用过的基站的new_data置为None
                    for i in list_data:
                        dict_src_data["{}".format(i)].new_data = None
                    # print("dict_src_data", dict_src_data)

                    # if npStationdata0 is None or npStationdata0.shape[0] == 0:
                    #     continue
                    self.log.info("step 7: the whole process use time: {} ms".format(round((time.time()- t0)*1000,3)))

                except:
                    traceback.print_exc()
                continue

            self.log.error('usr_process {} exit...'.format(self.log.name))
            server.close()
        else:
            self.log.info("read offline process start, pid={} ".format(os.getpid()))

            sys_status_channel = 'src_data'    # 数据发送频道名称
            data_bus_server = DataBus(strHost)

            t0 = time.time()
            # strSelectFilePath = '/data/org_data_0602/orgData1.csv'
            strSelectFilePath = '/data/data0430-08.txt'
            strDataFile = strSelectFilePath
            self.log.info("Start get data from {}!".format(strDataFile))

            if 'orgData' in strDataFile:
                dictFrame, nStartFrameId, nEndFrameId = get_dict_from_dir(strDataFile)
            elif '.txt' in strDataFile:
                dictFrame, nStartFrameId, nEndFrameId = get_dict_from_txt(strDataFile)
            elif '.json' in strDataFile:
                dictFrame, nStartFrameId, nEndFrameId = get_dict_from_json(strDataFile)
            else:
                dictFrame, nStartFrameId, nEndFrameId = get_dict_from_csv(strDataFile)



            # 功能接口状态类，用于接收接口类中的系统状态，同时上报功能进程的相关信息
            sys_state = UsrSysState(self.q_sys_state, self.q_model_state, self.log)
            while sys_state.get_sys_state():

                try:
                    nFrameIdTemp = nStartFrameId
                    while nFrameIdTemp <= nEndFrameId:
                        npFrameData = dictFrame[nFrameIdTemp]
                        print("npFrameData",npFrameData)
                        nFrameIdTemp = int((nFrameIdTemp + 1) % MAX_FRAME_ID)

                        npStationdata = getByteofNdarray(npFrameData)
                        dict_send_message = {}
                        dict_send_message["targetInfo"] = npStationdata
                        dict_send_message["baseStation"] = []
                        req = json.dumps(dict_send_message)

                        data_bus_server.publish(sys_status_channel, req)
                        self.log.info("step 6: send message ")
                        time.sleep(0.1)
                    else:
                        self.log.info("over!! ")
                        continue

                except:
                    traceback.print_exc()
                continue


if __name__ == '__main__':
    testMod = TestMod()
    testMod.join()
    testMod.log.error('main exit...')
    sys.exit(0)
