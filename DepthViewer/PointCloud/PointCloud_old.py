# #####*****  添加的新的去重方式  *****#####
# -*-coding:utf-8-*
from CommonDefine import *
import open3d as o3d
# =================== 添加 点云算法 开始 =====================
# from second.sort_0325 import Sort
from second.sort_0428 import Sort
import second.core.box_np_ops as box_np_ops
from spconv.utils import rotate_non_max_suppression_cpu
from traffic_flow_matrix import Traffic_Flow,Use_Camera_Map
from StationFusion import Double_radar_fusion,sort_radar_lin
from sklearn.utils.linear_assignment_ import linear_assignment
from shapely.geometry import Polygon
from second.all_track import rotate_box, fusion_box, get_min_max, fix_cluster, cal_box_lane, cal_map_angle
import cv2

def cache_bmp(img):
    img_convert = np.ones_like(img)

    (height, width, color) = img.shape

    for y in range(height):
        # conver_y = height - y - 1
        conver_y = y
        img_convert[y, :, :] = img[conver_y, :, :]
    return img_convert

# # #####*****  加载车道bmp  *****#####
strExePath = getExcutePath()
# bmp_path = strExePath + '/Configs/Lane/lane_new.png'
# img_cv2 = cv2.imread(bmp_path)
# imgg = cv2.cvtColor(img_cv2, cv2.COLOR_BGR2RGB)
# bmp_img = cache_bmp(imgg)
bmp_img = np.load(strExePath + '/Configs/Lane/lane_new.npy')

# #####*****  2021-0509 my add  *****#####
left_center = np.load(strExePath + '/Configs/Lane/left_center.npy')
medium_center = np.load(strExePath + '/Configs/Lane/medium_center.npy')
right_center = np.load(strExePath + '/Configs/Lane/right_center.npy')
left_center = left_center[left_center[:, 0].argsort()]
medium_center = medium_center[medium_center[:, 0].argsort()]
right_center = right_center[right_center[:, 0].argsort()]


nTimeOut = 0.05  # 50ms
# mapErrCode 数字表示数组listErrCode下标，某一个下标位置处为真，则表示出现该异常
mapErrCode = {0: "Missing frame", 1: "Rain", 2: "Shaking", 3: "Abnormal intensity", 4: "Abnormal distribution"}
nPlatformB0Frequency = 0.1
nPlatformB1Frequency = 60
nPlatformB2Frequency = 0.1
nPlatformB5Frequency = 5




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

#############对lidar计算iou值##############
def rotate_nms_cc_iou(dets,trackers):
    trackers_corners = box_np_ops.center_to_corner_box2d(trackers[:, :2], trackers[:, 3:5], trackers[:, 6]*np.pi/180.0)
    trackers_standup = box_np_ops.corner_to_standup_nd(trackers_corners)
    dets_corners = box_np_ops.center_to_corner_box2d(dets[:, :2], dets[:, 3:5], dets[:, 6]*np.pi/180.0)
    dets_standup = box_np_ops.corner_to_standup_nd(dets_corners)
    standup_iou = box_np_ops.iou_jit(dets_standup, trackers_standup, eps=0.0)
    # standup_iou, standup_iou_new = iou_jit_new(dets_standup, trackers_standup, eps=0.0)
    # iou_matrix = np.zeros((dets_corners.shape[0], trackers_corners.shape[0]))
    # for num_i in range(dets_corners.shape[0]):
    #     for num_j in range(trackers_corners.shape[0]):
    #         if num_j == num_i:
    #             continue
    #         IOU = intersection(dets_corners[num_i].tolist(), trackers_corners[num_j].tolist())
    #         iou_matrix[num_i][num_j] = IOU
    return standup_iou

def associate_detections_to_trackers(detections, trackers):
    """
    数据关联的函数，将跟踪轨迹和检测目标关联起来
    Assigns detections to tracked o192.168.3.181bject (both represented as bounding boxes)

    Returns 3 lists of matches, unmatched_detections and unmatched_trackers
    """
    if (len(trackers) == 0):
        return np.empty((0, 2), dtype=int), np.arange(len(detections)), np.empty((0, 5), dtype=int), 0
    #####直接使用lidar部分的iou，这里直接使用矩阵就行#####
    start_time = time.time()
    iou_matrix = rotate_nms_cc_iou(detections, trackers)
    print('cal iou time is : ', time.time()-start_time)
    for i in range(iou_matrix.shape[0]):
        iou_matrix[i][i] = 0
    cost_matrix = iou_matrix
    iou_threshold = 0.000001
    # #####****  匈牙利算法获取匹配的检测和跟踪轨迹  *****#####
    matched_indices = linear_assignment(-cost_matrix)
    unmatched_detections = []
    for d, det in enumerate(detections):
        if (d not in matched_indices[:, 0]):
            unmatched_detections.append(d)
    unmatched_trackers = []
    for t, trk in enumerate(trackers):
        if (t not in matched_indices[:, 1]):
            unmatched_trackers.append(t)

    # filter out matched with low IOU
    matches = []
    for m in matched_indices:
        if cost_matrix[m[0], m[1]] < iou_threshold:
            unmatched_detections.append(m[0])
            unmatched_trackers.append(m[1])
        else:
            matches.append(m.reshape(1, 2))
    if (len(matches) == 0):
        matches = np.empty((0, 2), dtype=int)
    else:
        matches = np.concatenate(matches, axis=0)

    return matches, np.array(unmatched_detections), np.array(unmatched_trackers), cost_matrix

def rotate_nms_cc(dets, thresh):
    scores = dets[:, 8]
    order = scores.argsort()[::-1].astype(np.int32)  # highest->lowest
    dets_corners = box_np_ops.center_to_corner_box2d(dets[:, :2], dets[:, 3:5],
                                                     dets[:, 6]/180*math.pi)

    dets_standup = box_np_ops.corner_to_standup_nd(dets_corners)

    standup_iou = box_np_ops.iou_jit(dets_standup, dets_standup, eps=0.0)
    # print(dets_corners.shape, order.shape, standup_iou.shape)
    return rotate_non_max_suppression_cpu(dets_corners, order, standup_iou,
                                          thresh)

def readOffline(stLidarParam, pcBaseQueueList):
    MyLog.writeLog("{}[:{}] - readPcOffline start, pid={}".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, os.getpid()), LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)


    stLidarParam.setReadChanged(False)
    stLidarParam.setNoPcData(False)
    stLidarParam.setManualReplay(False)
    stLidarParam.setAutoLoopPlay(False)
    stLidarParam.setPause(False)
    listPcOver = np.full(len(stLidarParam.getListOfflineFiles()), False, bool)
    stLidarParam.setPcOver(listPcOver)
    stLidarParam.setStatus(0)
    stLidarParam.setFrameId(0)
    listAttachedIp = stLidarParam.getlistAttachedIp()
    # stParam.setParamChanged(True)

    # 判断选择的文件列表是否为空,即使不选择,FileList的len依然是2,如果选择了文件,则FileList中的[0]元素长度必然>0
    if len(stLidarParam.getListOfflineFiles()[0]) < 1:
        MyLog.writeLog("{}[:{}] - exit readPcOffline 1...".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_ERROR, ENABLE_WRITE_LOG, ENABLE_PRINT)
        return

    stPcAlgParam = structPcAlgParam()
    serializationParam(stPcAlgParam=stPcAlgParam)
    nSyncIdLimitPcOffline = stPcAlgParam.nSyncIdLimitPcOffline

    listIndexOffset = []  # 存放当前pcap文件每帧第一包的文件位置
    nFrameCountPerFile = 0
    nCount = 0
    nFileIdTemp = 0
    nFrameIdTemp = 0
    now_angle_np = np.ones(1)
    head_data = np.ones(30)
    timeOver = QTime()
    while stLidarParam.getRunning() and (not stLidarParam.getOnline()) and (not stLidarParam.getReadChanged()):
        bIsDataRight = False
        if stLidarParam.getStatus() == 5:  # 5表示文件全部读取完毕，等带用户输入
            # timeOver.start()
            nSize = pcBaseQueueList.qsize()
            # print("pc Src = %d, dst = %d" % (Point_data.qsize(), dstQueue.qsize()))
            if nSize == 0:  # 确保数据被读取并显示完毕
                stLidarParam.setPause(True)
                listPcOver = np.full(len(stLidarParam.getListOfflineFiles()), True, bool)
                stLidarParam.setPcOver(listPcOver)
                timeOver.restart()
                if stLidarParam.getAutoLoopPlay() or stLidarParam.getManualReplay():
                    stLidarParam.setStatus(0)
                    stLidarParam.setFrameId(0)
                    MyLog.writeLog("{}[:{}] - Loop Play...".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_INFO, ENABLE_WRITE_LOG, True)
            time.sleep(0.01)
            continue
        elif stLidarParam.getStatus() == 6:
            stLidarParam.setStatus(0)

        nLen = len(listAllIndexOffset)
        nFrameCountAllFile = 0
        for i in range(nLen):
            # print("nFrameCountAllFile += listAllIndexOffset[i]...")
            nFrameCountAllFile += len(listAllIndexOffset[i])

        usStatusTemp = stLidarParam.getStatus()
        bPcOver = True
        listOver = stLidarParam.getPcOver()
        for i in range(len(listOver)):
            bPcOver &= listOver[i]
        if (stLidarParam.getFrameId() + nSyncIdLimitPcOffline > nFrameCountAllFile - 1) or \
                (bPcOver and (
                        usStatusTemp == 0 or usStatusTemp == 3 or usStatusTemp == 4)):  # 界面当前帧大于上次读取的帧数，则需要继续读取到指定帧，再显示，该过程中也要缓存!
            stLidarParam.setFrameId(0)

        bFirst = True
        listPcOver = np.full(len(stLidarParam.getListOfflineFiles()), False, bool)
        stLidarParam.setPcOver(listPcOver)
        stLidarParam.setPause(False)
        stLidarParam.setAutoLoopPlay(False)
        stLidarParam.setManualReplay(False)
        nFrameIdLast = 0
        bRefind = False
        nFrameIdReFind = 0
        nFrameIdTemp = 0

        while nFileIdTemp < len(stLidarParam.getListOfflineFiles()) and stLidarParam.getRunning() and (
                not stLidarParam.getOnline()) and (not stLidarParam.getReadChanged()):
            # print("while nFileIdTemp < len(stParam.listOfflineFiles):...")
            strSelectFilePath = stLidarParam.getListOfflineFiles()[nFileIdTemp]
            strPcapFile = strSelectFilePath + ".pcap"

            if not my_isExist(strPcapFile):
                stLidarParam.setNoPcData(True)
                listVideoPath = []
                bIsExist = False
                for i in range(len(stLidarParam.getListOfflineFiles())):
                    strVideoFilePath = "{}_Channel{}.avi".format(strSelectFilePath, (i + 1))
                    listVideoPath.append(strVideoFilePath)
                    if not my_isExist(strVideoFilePath):
                        MyLog.writeLog("{}[:{}] - video {} is not exist!".format(
                            __file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno,
                            strVideoFilePath), LOG_ERROR, ENABLE_WRITE_LOG, ENABLE_PRINT)
                    else:
                        bIsExist = True

                if not bIsExist:
                    MyLog.writeLog("{}[:{}] - point clout {} is not exist!".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                                   sys._getframe().f_lineno, strPcapFile),
                                   LOG_ERROR, ENABLE_WRITE_LOG, ENABLE_PRINT)
                    stLidarParam.setNoVideoData(True)
                    time.sleep(1)
                    continue

                while stLidarParam.getRunning() and (not stLidarParam.getOnline()) and (not stLidarParam.getReadChanged()):
                    time.sleep(1)
                continue
            stLidarParam.setNoPcData(False)
            timeTemp = QTime()
            timeTemp.start()
            with open(strPcapFile, 'rb') as f:
                while stLidarParam.getRunning() and (not stLidarParam.getOnline()) and (not stLidarParam.getReadChanged()):
                    if nFrameIdTemp < nSyncIdLimitPcOffline:
                        nFrameIdTemp = nSyncIdLimitPcOffline
                    while stLidarParam.getPause() and (stLidarParam.getStatus() == 0):
                        if stLidarParam.getOnline() or stLidarParam.getReadChanged():
                            MyLog.writeLog("{}[:{}] - exit readPcOffline 2...".format(
                                __file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno),
                                LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                            return
                        time.sleep(0.01)

                    bReadOver = False
                    pcap = dpkt.pcap.Reader(f)
                    for timestamp, raw_buf in pcap:  # timestamp时间戳，raw_buf包中的原始数据
                        eth = dpkt.ethernet.Ethernet(raw_buf)
                        ####判断这个包是不是IP数据报########
                        if not isinstance(eth.data, dpkt.ip.IP):
                            continue
                        if not isinstance(eth.data.data, dpkt.udp.UDP):  # 解包，判断传输层协议是否是TCP，即当你只需要TCP时，可用来过滤
                            continue
                        packet = eth.data  # 让以太网帧的ip数据报部分等于一个新的对象，packet
                        udp_data = packet.data  ##每个UDP数据
                        data = udp_data.data  ##每一包的数据包含的东西
                        if packet.src in listAttachedIp:
                            pass
                        headData = parsingHeadData(data)
                        listAttachedIp = stLidarParam.getlistAttachedIp()
                        if headData is None:
                            continue
                        if headData[2] != 0xA0:
                            for i in range(len(listAttachedIp)):
                                listsrcip = list(packet.src)
                                listdstip = list(packet.dst)
                                AttachedIp = [int(x) for x in listAttachedIp[i].split('.')]
                                if list(packet.src) == [int(x) for x in listAttachedIp[i].split('.')] and \
                                        headData[2] == 0xE1 and headData[3] == 0x02:
                                    listData, time_stramp = parseBaseStationData(data)
                                    pcBaseQueueList[i].put([listData, time_stramp])
                                    while pcBaseQueueList[i].qsize() > 100:
                                        time.sleep(nTimeOut)  # 防止队列堆积！
                        else:
                            recvData = parsingData(data)
                            if recvData is None or recvData[2] != 0xA0:
                                continue
                # 下面的while循环处理可能会有问题
                nSize = pcBaseQueueList.qsize()
                while nSize > 0 and stLidarParam.getRunning() and (not stLidarParam.getOnline()) and (
                        not stLidarParam.getReadChanged()):
                    if stLidarParam.getStatus() in [1, 2]:
                        nFileIdTemp = 0
                        break

                    nSize = pcBaseQueueList.qsize()
                    time.sleep(0.1)

        if bIsDataRight:
            stLidarParam.setStatus(5)
            nFileIdTemp = 0
        else:
            stLidarParam.setStatus(6)
            stLidarParam.setNoPcData(True)
            MyLog.writeLog("{}[:{}] - pc src data error!".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                 sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
            time.sleep(1)

    MyLog.writeLog("{}[:{}] - exit readPcOffline 13...".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)

def readOffline_csv(stLidarParam, pcBaseQueueList,resultQueue):
    MyLog.writeLog("{}[:{}] - readPcOffline start, pid={}".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, os.getpid()), LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)


    stLidarParam.setReadChanged(False)
    stLidarParam.setNoPcData(False)
    stLidarParam.setManualReplay(False)
    stLidarParam.setAutoLoopPlay(False)
    stLidarParam.setPause(False)
    listPcOver = np.full(len(stLidarParam.getListOfflineFiles()), False, bool)
    stLidarParam.setPcOver(listPcOver)
    stLidarParam.setStatus(0)
    stLidarParam.setFrameId(0)
    stLidarParam.setParamChanged(False)

    # 判断选择的文件列表是否为空,即使不选择,FileList的len依然是2,如果选择了文件,则FileList中的[0]元素长度必然>0
    if len(stLidarParam.getListOfflineFiles()[0]) < 1:
        MyLog.writeLog("{}[:{}] - exit readPcOffline 1...".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_ERROR, ENABLE_WRITE_LOG, ENABLE_PRINT)
        return

    stPcAlgParam = structPcAlgParam()
    serializationParam(stPcAlgParam=stPcAlgParam)
    nSyncIdLimitPcOffline = stPcAlgParam.nSyncIdLimitPcOffline

    nFileIdTemp = 0
    timeOver = QTime()
    timeWait = QTime()
    timeWait.start()
    while stLidarParam.getRunning() and (not stLidarParam.getOnline()) and (not stLidarParam.getReadChanged()):
        bIsDataRight = False
        if stLidarParam.getStatus() == 5:  # 5表示文件全部读取完毕，等带用户输入
            nSize = pcBaseQueueList[0].qsize()
            if nSize == 0:  # 确保数据被读取并显示完毕
                stLidarParam.setPause(True)
                listPcOver = np.full(len(stLidarParam.getListOfflineFiles()), True, bool)
                stLidarParam.setPcOver(listPcOver)
                timeOver.restart()
                if stLidarParam.getAutoLoopPlay() or stLidarParam.getManualReplay():
                    stLidarParam.setStatus(0)
                    stLidarParam.setFrameId(0)
                    MyLog.writeLog("{}[:{}] - Loop Play...".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_INFO, ENABLE_WRITE_LOG, True)
            time.sleep(0.01)
            continue
        elif stLidarParam.getStatus() == 6:
            stLidarParam.setStatus(0)

        nLen = len(listAllIndexOffset)
        nFrameCountAllFile = 0
        for i in range(nLen):
            nFrameCountAllFile += len(listAllIndexOffset[i])

        usStatusTemp = stLidarParam.getStatus()
        bPcOver = True
        listOver = stLidarParam.getPcOver()
        for i in range(len(listOver)):
            bPcOver &= listOver[i]
        if (stLidarParam.getFrameId() + nSyncIdLimitPcOffline > nFrameCountAllFile - 1) or \
                (bPcOver and (
                        usStatusTemp == 0 or usStatusTemp == 3 or usStatusTemp == 4)):  # 界面当前帧大于上次读取的帧数，则需要继续读取到指定帧，再显示，该过程中也要缓存!
            stLidarParam.setFrameId(0)

        listPcOver = np.full(len(stLidarParam.getListOfflineFiles()), False, bool)
        stLidarParam.setPcOver(listPcOver)
        stLidarParam.setPause(False)
        stLidarParam.setAutoLoopPlay(False)
        stLidarParam.setManualReplay(False)

        while nFileIdTemp < len(stLidarParam.getListOfflineFiles()) and stLidarParam.getRunning() and (
                not stLidarParam.getOnline()) and (not stLidarParam.getReadChanged()):
            strSelectFilePath = stLidarParam.getListOfflineFiles()[nFileIdTemp]
            strDataFile = strSelectFilePath

            if not my_isExist(strDataFile):
                stLidarParam.setNoPcData(True)
                bIsExist = False

                if not bIsExist:
                    MyLog.writeLog("{}[:{}] - point clout {} is not exist!".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                                   sys._getframe().f_lineno, strDataFile),
                                   LOG_ERROR, ENABLE_WRITE_LOG, ENABLE_PRINT)
                    stLidarParam.setNoVideoData(True)
                    time.sleep(1)
                    continue

                while stLidarParam.getRunning() and (not stLidarParam.getOnline()) and (not stLidarParam.getReadChanged()):
                    time.sleep(1)
                continue
            stLidarParam.setNoPcData(False)
            timeTemp = QTime()
            timeTemp.start()
            MyLog.writeLog("{}[:{}] - Start get data from {}!".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                      sys._getframe().f_lineno, strDataFile),
                           LOG_ERROR, ENABLE_WRITE_LOG, ENABLE_PRINT)
            if 'orgData' in strDataFile:
                dictFrame, nStartFrameId, nEndFrameId = get_dict_from_dir(strDataFile)
            elif '.txt' in strDataFile:
                dictFrame, nStartFrameId, nEndFrameId = get_dict_from_txt(strDataFile)
            elif '.json' in strDataFile:
                dictFrame, nStartFrameId, nEndFrameId = get_dict_from_json(strDataFile)
            else:
                dictFrame, nStartFrameId, nEndFrameId = get_dict_from_csv(strDataFile)
            nFrameIdTemp = nStartFrameId
            MyLog.writeLog("{}[:{}] - Get data success, nStartFrameId:{},nEndFrameId:{}!".
                           format(__file__.split('/')[len(__file__.split('/')) - 1],sys._getframe().f_lineno, nStartFrameId, nEndFrameId),
                           LOG_ERROR, ENABLE_WRITE_LOG, ENABLE_PRINT)
            stLidarParam.setLidarFrameCount(nEndFrameId)
            stLidarParam.setFrameId(nStartFrameId)
            while stLidarParam.getRunning() and (not stLidarParam.getOnline()) and (not stLidarParam.getReadChanged()):
                while stLidarParam.getPause() and (stLidarParam.getStatus() in [0,3]):
                    while resultQueue.qsize() > 0:
                        try:
                            resultQueue.get_nowait()
                        except:
                            pass
                    if stLidarParam.getOnline() or stLidarParam.getReadChanged():
                        MyLog.writeLog("{}[:{}] - exit readPcOffline 2...".format(
                            __file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno),
                            LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                        return
                    time.sleep(0.01)
                if stLidarParam.getParamChanged():
                    nFrameIdTemp = stLidarParam.getFrameId()
                    stLidarParam.setParamChanged(False)
                bReadOver = False
                if timeWait.elapsed() < 100:
                    continue
                timeWait.restart()
                if stLidarParam.getStatus() in [0,3]:  # 开始自动播放
                    try:
                        npFrameData = dictFrame[nFrameIdTemp]
                    except:
                        nFrameIdTemp = int((nFrameIdTemp + 1) % MAX_FRAME_ID)
                        continue
                    pcBaseQueueList[0].put([npFrameData,nFrameIdTemp])
                    nFrameIdTemp = int((nFrameIdTemp + 1) % MAX_FRAME_ID)
                    if nFrameIdTemp > nEndFrameId:
                        break
                    while (pcBaseQueueList[0].qsize() > 2) and (stLidarParam.getStatus() not in [1, 2]):
                        if stLidarParam.getOnline() or stLidarParam.getReadChanged():
                            MyLog.writeLog("{}[:{}] - exit readPcOffline 3...".format(
                                __file__.split('/')[len(__file__.split('/')) - 1],
                                sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG,
                                ENABLE_PRINT)
                            return
                        time.sleep(nTimeOut)  # 防止队列堆积！
                    continue

                while (0 < stLidarParam.getStatus() <= 2) and (not stLidarParam.getOnline()) and (
                        not stLidarParam.getReadChanged()):

                    while (stLidarParam.getStatus() == 2) and stLidarParam.getRunning():
                        # print("pc sleep1 ...")
                        if stLidarParam.getOnline() or stLidarParam.getReadChanged():
                            MyLog.writeLog("{}[:{}] - exit readPcOffline 4...".format(
                                __file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno),
                                LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                            return
                        time.sleep(0.01)
                    if stLidarParam.getStatus() != 1:
                        break
                    while pcBaseQueueList[0].qsize() > 0:
                        # print("while Point_data.qsize() > 0: 1...")
                        try:
                            toBeDel = pcBaseQueueList[0].get_nowait()
                        except:
                            continue
                    stLidarParam.setStatus(2)
                    nFrameIdTemp = stLidarParam.getFrameId()
                    if nFrameIdTemp > nEndFrameId:
                        bReadOver = True
                    try:
                        npFrameData = dictFrame[nFrameIdTemp]
                    except:
                        nFrameIdTemp = int((nFrameIdTemp + 1) % MAX_FRAME_ID)
                        continue
                    pcBaseQueueList[0].put([npFrameData,nFrameIdTemp])
                    time.sleep(nTimeOut)  # 添加延时，使算法线程获取锁，从而能从队列中get()！
                    continue

                if bReadOver:
                    nFileIdTemp += 1
                    break

            nSize = pcBaseQueueList[0].qsize()
            while nSize > 0 and stLidarParam.getRunning() and (not stLidarParam.getOnline()) and (
                    not stLidarParam.getReadChanged()):
                if stLidarParam.getStatus() in [1, 2]:
                    nFileIdTemp = 0
                    break

                nSize = pcBaseQueueList[0].qsize()
                time.sleep(0.1)

        if bIsDataRight:
            stLidarParam.setStatus(5)
            nFileIdTemp = 0
        else:
            stLidarParam.setStatus(6)
            stLidarParam.setNoPcData(True)
            MyLog.writeLog("{}[:{}] - pc src data error!".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                 sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
            time.sleep(1)

    MyLog.writeLog("{}[:{}] - exit readPcOffline 13...".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)

def readOnline(stLidarParam, pcBaseQueueList):
    MyLog.writeLog("{}[:{}] - readOnline start, pid={}".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                               sys._getframe().f_lineno, os.getpid()),
                   LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)
    BUFSIZE = 4096
    nPort = stLidarParam.getUdpServicePort()
    ip_port = ('', nPort)
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # udp协议
    server.bind(ip_port)
    MyLog.writeLog("{}[:{}] - bind ip: {}".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                  sys._getframe().f_lineno, ip_port),
                   LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)
    tLast = time.time()
    listAttachedIp = stLidarParam.getlistAttachedIp()
    listIpLastTime = []
    for i in range(len(listAttachedIp)):
        listIpLastTime.append(time.time())
    fLongitudeT = stLidarParam.getLidarLongitude(0)
    fLatitudeT = stLidarParam.getLidarLatitude(0)
    fAngleNorthT = stLidarParam.getAngleNorthT(0)
    stTransform = Lon_Lat_Meter_Transform(fLongitudeT, fLatitudeT, fAngleNorthT)
    while stLidarParam.getRunning():
        try:
            data, client_addr = server.recvfrom(BUFSIZE)
            tLast = time.time()
            headData = parsingHeadData(data)
            if headData is None:
                continue
            if headData[2] != 0xA0:
                if client_addr[0] in listAttachedIp and headData[2] == 0xE1 and headData[3] == 0x02:
                    i = listAttachedIp.index(client_addr[0])
                    npBoxInfo, listDevInfo = parseBaseStationData_np(data)
                    if npBoxInfo is not None:
                        if stLidarParam.getCoordinateFlag():
                            npBoxInfo[:,6] = (npBoxInfo[:,6] + 180 - fAngleNorthT + 360) % 360
                            npBoxInfo[:, 0:2] = stTransform.lonlat_to_xy_of_draw(npBoxInfo[:, 19:21])
                        else:
                            npBoxInfo[:,6] = (npBoxInfo[:,6] + 180 - stLidarParam.getAngleNorthT(i) + 360) % 360
                    else:
                        continue
                    pcBaseQueueList[i].put([npBoxInfo[:,:19], listDevInfo])
                    MyLog.writeLog("{}[:{}] - get data from {}".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                       sys._getframe().f_lineno, client_addr[0]),
                                   LOG_INFO, ENABLE_WRITE_LOG, 0)
                    while pcBaseQueueList[i].qsize() > 1:
                        try:
                            pcBaseQueueList[i].get(True, 0.01)
                        except:
                            MyLog.writeLog("{}[:{}] - pcBaseQueue1.get(True, 0.01) err!".format(
                                __file__.split('/')[len(__file__.split('/')) - 1],
                                sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                            continue
            else:
                recvData = parsingData(data)
                if recvData is None or recvData[2] != 0xA0:
                    continue

                print("recv:", recvData)
                stLidarParam.setPause(True)
                listForwardIpPort = stLidarParam.getListForwardDstDev()
                if recvData[13] == 1:
                    stLidarParam.setSendResult(False)
                    stLidarParam.setParamChanged(True)
                    stLidarParam.setPause(False)
                    continue
                elif recvData[12] == 0:
                    stLidarParam.setSendResult(False)
                elif recvData[12] == 1:
                    stLidarParam.setSendResult(True)
                elif recvData[12] == 2:
                    stLidarParam.setSendResult(True)

                strSrcPcDataIpPort = "{}.{}.{}.{}-{}".format(recvData[6], recvData[7], recvData[8], recvData[9], recvData[10])
                strPcResultIpPort = "{}.{}.{}.{}-{}".format(recvData[6], recvData[7], recvData[8], recvData[9], recvData[11])
                listForwardIpPort[0] = [strSrcPcDataIpPort, False]
                listForwardIpPort[1] = [strPcResultIpPort, False]

                MyLog.writeLog("{}[:{}] - Forward ip changed! setParamChanged(True)!".format(
                    __file__.split('/')[len(__file__.split('/')) - 1],
                    sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                stLidarParam.setListForwardDstDev(listForwardIpPort)
                stLidarParam.setParamChanged(True)
                stLidarParam.setPause(False)

        except:
            print(traceback.format_exc())
            MyLog.writeLog("{}[:{}] - except! udpServerProcess! Call stack:\n{}!"
                           .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno,
                                   traceback.format_exc()), LOG_ERROR, ENABLE_WRITE_LOG, ENABLE_PRINT)

    server.close()

# 点云读取的进程函数
def readData(resultQueue,pcBaseQueueList, stParam, lockVid):
    while stParam.getRunning():
        if stParam.getOnline():
            readOnline(stParam, pcBaseQueueList)
        else:
            readOffline_csv(stParam, pcBaseQueueList,resultQueue)

    MyLog.writeLog("{}[:{}] - exit proReadPc...".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)

# =============================== 算法进程执行函数 ================================
'''点云算法在在一个进程中执行，然后把执行结果队列，传入该函数，这样一来，除了第一次时，融合算法会等待点云一次，之后便是并行了！
因为在融合算法执行第一次的时候，点云算法将执行第二次，二当融合算法执行第二次时，点云第二次已执行完毕，正好把结果给融合算法！
这样，虽然融合算法依然需要点云算法的结果，但不会出现组摄的情况，从而实现并行！'''
def pcAlgOnline_old(stPcAlgParam, dstResultQueue, srcEventDataQueue, pcBaseList, stParam, lockVid):
    MyLog.writeLog("{}[:{}] - pcAlgOnline start, pid={}".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, os.getpid()), LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)

    file_path = './savedata/org_data/'
    file_path_2 = './savedata/EventData/'
    saveFileTime = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    file_name = os.path.join(file_path, '{}'.format(saveFileTime))
    file_name_2 = os.path.join(file_path_2, '{}'.format(saveFileTime))
    stParam.setAlgChanged(False)
    stParam.setNoPcData(False)
    usStatusLast = -1
    cacheQueue = Queue()
    thCache = threading.Thread(target=cacheThread, args=(cacheQueue, stParam, lockVid))

    # #####*****  2021-0509 my add  *****#####
    # #####*****  加载全域车道中心线信息  *****#####
    global left_center, right_center, medium_center, bmp_img
    mot_tracker_merge = Sort(left_center, medium_center, right_center, bmp_img, stPcAlgParam.nMaxAge, stPcAlgParam.nMinHits, stPcAlgParam.nLastCount)

    # mot_tracker_merge = Sort(stPcAlgParam.nMaxAge, stPcAlgParam.nMinHits)
    # 开始循环
    timeTemp = QTime()
    timeTemp.start()
    timeExcept = QTime()
    tLast = time.time()
    nFrameIdTemp = 0
    nFrameIdLast = 0
    nTt = 0
    pcFrameLast = None
    tn = time.time()
    fusion = Double_radar_fusion.Fusion(max_age=4)         # 加入id对匹配
    srcDataList = []
    listNorthAngleT = []
    listBaseStationId = []
    for i in range(len(stParam.getlistAttachedIp())):
        srcDataList.append([time.time(), [None], True,[]])
        '''
        srcDataList:存放基站结果数据的列表:
                [0]:新一帧数据到达的系统时间
                [1]:存放当前帧处理数据的列表，默认长度为1
                [2]:该基站在线状态，在线:True，离线:False
                [3]:历史缓存数据，用于时间匹配，最大长度为10
        '''
        listNorthAngleT.append(stParam.getAngleNorthT(i))
        listBaseStationId.append(0)
    listSortNorthAngleT = sorted(listNorthAngleT)
    for i in range(len(stParam.getlistAttachedIp())):
        if stParam.getHighSpeed():
            listBaseStationId[i] = i + 1
        else:
            listBaseStationId[i] = listSortNorthAngleT.index(listNorthAngleT[i]) + 1
    dictCameraParam = {}
    for index in range(len(stParam.getlistAttachedIp())):
        listCameraDev = stParam.getStationCameraDev(index)
        dictCameraParam[listBaseStationId[index]] = listCameraDev
    dictLaneId = {}

    timeForce10Fps = QTime()
    timeForce10Fps.start()
    bGetBS1data = False
    while True:
        try:
            tTest = time.time()
            MyLog.writeLog("{}[:{}] - pcAlgOnline step 1".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                 sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)

            # 超时则认为没有点云数据或者点云数据异常
            if (time.time() - tLast) > 3 and (not stParam.getPause()) and (stParam.getStatus() == 0 or stParam.getStatus() == 3):
                stParam.setNoPcData(True)
                MyLog.writeLog("{}[:{}] - No pc data or data error! time:{}, bPause:{}, status:{}"
                               .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno,
                                       round((time.time() - tLast), 3), stParam.getPause(),
                                       stParam.getStatus()),
                               LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                tLast = time.time()  # 要重新计时
            nIndex = 0
            if stParam.getStatus() in [0, 5] or stParam.getStatus() in [3, 4]:
                for Basequeue in pcBaseList:
                    try:
                        data = Basequeue.get(True,0.001)
                        srcDataList[nIndex][0] = time.time()
                        if srcDataList[nIndex][1][0] is None:
                            srcDataList[nIndex][1][0] = data
                        else:
                            srcDataList[nIndex][1].pop(0)
                            srcDataList[nIndex][1].append(data)
                        #=====================time match=====================#
                        if nIndex == 0:
                            bGetBS1data = True
                        srcDataList[nIndex][3].append(data)
                        if len(srcDataList[nIndex][3]) > 10:
                            srcDataList[nIndex][3].pop(0)
                        #=====================time match=====================#
                        srcDataList[nIndex][2] = True
                        nIndex += 1
                    except:
                        if (time.time() - srcDataList[nIndex][0]) > 0.5 and srcDataList[nIndex][1][0] is None and srcDataList[nIndex][2]:
                            srcDataList[nIndex][2] = False
                            MyLog.writeLog("{}[:{}] - baseStation{} is offline".
                                           format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno,nIndex),
                                           LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                        nIndex += 1
                        continue
                ngetFlag = False
                #=====================time match=====================#
                # if srcDataList[0][2] and  bGetBS1data and len(srcDataList[0][3]) > 2:
                #     bGetBS1data = False
                #     tBSData1 = srcDataList[0][3][-3][1][0]
                #     srcDataList[0][1][0] = srcDataList[0][3][-3]
                #     for i in range(len(srcDataList)):
                #         if i == 0 or len(srcDataList[i][3]) == 0:
                #             continue
                #         listTError = [abs(listBSData[1][0] - tBSData1) for listBSData in srcDataList[i][3]]
                #         minTError = min(listTError)
                #         print("BS{} time error is {},index is {}".format(i,minTError,listTError.index(minTError)))
                #         if minTError < 0.05:
                #             srcDataList[i][1][0] = copy.deepcopy(srcDataList[i][3][listTError.index(minTError)])
                #         else:
                #             srcDataList[i][1][0] = None
                #     timeForce10Fps.restart()
                #     ngetFlag = True
                # elif not srcDataList[0][2] and timeForce10Fps.elapsed() > 100:
                #     timeForce10Fps.restart()
                #     ngetFlag = True
                #=====================time match=====================#
                if srcDataList[0][1][0] is not None:
                    timeForce10Fps.restart()
                    ngetFlag = True
                elif not srcDataList[0][2] and timeForce10Fps.elapsed() > 100:
                    timeForce10Fps.restart()
                    ngetFlag = True
            else:
                continue
            if (not stParam.getRunning()) or stParam.getAlgChanged():
                MyLog.writeLog("{}[:{}] - exit pcAlgOnline 1...".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                return

            if stParam.getPause() and 1 <= stParam.getStatus() <=2 or not ngetFlag:
                time.sleep(0.01)
                if timeExcept.elapsed() > 0:
                    timeExcept.restart()
                continue
            tWait = time.time()
            MyLog.writeLog("{}[:{}] - pcAlgOnline step 2".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                 sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)

            trunAlg_start = time.time()
            dstData = structData()
            nFrameIdTemp = int((nFrameIdTemp + 1) % MAX_FRAME_ID)
            dstData.nFrameId = nFrameIdTemp
            trackers_covall = None

            # current_time_s, current_time_us = parse_GPS_time()
            # current_timestamp = current_time_s + current_time_us / 1000000.0
            if stParam.getStatus() in [0, 5] or stParam.getStatus() in [3, 4]:
                try:
                    list_matrix_base = []
                    index =0
                    nFrameID = None
                    nEqupId = None
                    for listsrcData in srcDataList:
                        if not listsrcData[2] or listsrcData[1][0] is None or len(listsrcData[1][0][0]) == 0:
                            index += 1
                            list_matrix_base.append(None)
                            continue
                        if nFrameID is None:
                            nFrameID = listsrcData[1][0][1][2]
                            nEqupId = listsrcData[1][0][1][1]
                            dstData.nEqupId = nEqupId
                            dstData.headData = tuple([nEqupId[1],nEqupId[0]])
                            time_stamp = listsrcData[1][0][1][0]
                        data_base = listsrcData[1][0][0]
                        data_base[:, 12] = listBaseStationId[index]
                        list_matrix_base.append(listsrcData[1][0][0])
                        index += 1

                    for i in range(len(srcDataList)):
                        srcDataList[i][1][0] = None
                        ##### 不同路口ip转发的数据进行坐标转换 ####
                    npStationdata0 = None
                    for i in range(len(list_matrix_base)):
                        if list_matrix_base[i] is None:
                            continue
                        if not stParam.getCoordinateFlag() and i != 0:
                            fRotationVector = stParam.getRotationVector(i)
                            fTranslationVector = stParam.getTranslationVector(i)
                            rotation = np.array([fRotationVector[0] * PI_rads, fRotationVector[1] * PI_rads, fRotationVector[2] * PI_rads])
                            #########rotate point ##################
                            rotation_pcd = o3d.geometry.PointCloud()
                            rotation_pcd.points = o3d.utility.Vector3dVector(list_matrix_base[i][:, :3])
                            rotation_pcd.rotate(rotation=rotation,center=False)
                            rotation_points = np.asarray(rotation_pcd.points)
                            list_matrix_base[i][:, :3] = rotation_points + np.array([fTranslationVector[0], fTranslationVector[1], fTranslationVector[2]]).reshape(1,3)
                            list_matrix_base[i][:, 6] -= fRotationVector[2]

                        box_nms=rotate_nms_cc(list_matrix_base[i], 0.01)
                        npStationdata1=list_matrix_base[i][box_nms,:]
                        if npStationdata0 is None:
                            npStationdata0 = npStationdata1
                            continue
                        if False:
                            listStationFusiondata = fusion.run(npStationdata0, npStationdata1)
                            if len(listStationFusiondata) == 0:
                                break
                            npStationFusiondata = np.array(listStationFusiondata)
                            npStationdata0 = np.zeros((len(data_base), 12))
                            npStationdata0 = npStationFusiondata[:,[2,3,7,8,1,9,6,10,11,12,0,4,13,14,15]]
                        else:
                            npStationdata0 = np.concatenate((npStationdata0, npStationdata1), axis=0)

                    if npStationdata0 is None or npStationdata0.shape[0] == 0:
                        continue
                    #----------------------test----------------------#
                    # if stUiParam.bSaveEvent == 1:
                    #     if nFrameIdTemp == 1 and my_isExist('./savedata/org_data/'):
                    #         for strfilename in os.listdir('./savedata/org_data/'):
                    #             path_file = os.path.join('./savedata/org_data/',strfilename)
                    #             if os.path.isfile(path_file):
                    #                 os.remove(path_file)
                    #     if my_mkdir('./savedata/org_data/'):
                    #         res = np.savetxt('./savedata/org_data/orgData' + str(nFrameIdTemp) + '.csv',npStationdata0,fmt='%f',delimiter=',')
                    if stUiParam.bSaveE1 == 1:
                        if my_mkdir(file_name):
                            res = np.savetxt('./savedata/org_data/{}/orgData'.format(saveFileTime) + str(nFrameIdTemp) + '.csv',npStationdata0,fmt='%f',delimiter=',')

                    #----------------------test----------------------#
                    print('taxis:',round((time.time()-tTest)*1000,2),'ms',npStationdata0.shape[0])
                    MyLog.writeLog("{}[:{}] - pcAlgOnline step 3".
                                   format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno),
                                   LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                    # box_nms=rotate_nms_cc(npStationdata0, 0.01)
                    # npStationdata0=npStationdata0[box_nms,:]
                    tWait = time.time()
                    if stParam.getShowRLFusion():
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

                        box_with_min_max, box_corners = get_min_max(trackers_cov_old_for_sort, stPcAlgParam.fConeThreshold)
                        mathch, un_match1, un_match2, cost_matrix = fusion_box(box_corners, box_corners, 0)
                        # trackers_cov_old_for_sort = box_with_min_max[:, [0, 1, 3, 4, 6, 2, 5, 7, 10, 9, 8, 11, 12, 13,14,15,16,17,18,19,20,21]]
                        # trackers_cov_old_for_sort[:, 4] = trackers_cov_old_for_sort[:, 4] * np.pi / 180
                        # mathch, un_match1, un_match2, cost_matrix = associate_detections_to_trackers(trackers_cov_old_for_sort,
                        #                                                                              trackers_cov_old_for_sort)
                        trackers_cov_old_for_sort = box_with_min_max
                        # save_indice = un_match1.tolist()
                        save_indice = []
                        delete_indice = []
                        for i in range(mathch.shape[0]):
                            match_row_indice = mathch[i][0]
                            match_column_indice = mathch[i][1]
                            if mathch[i][0] in delete_indice:
                                continue
                            if trackers_cov_old_for_sort[match_row_indice][17] < trackers_cov_old_for_sort[match_column_indice][17]:
                                # x_min = trackers_cov_old_for_sort[match_row_indice][-4]
                                # y_min = trackers_cov_old_for_sort[match_row_indice][-3]
                                # x_max = trackers_cov_old_for_sort[match_row_indice][-2]
                                # y_max = trackers_cov_old_for_sort[match_row_indice][-1]
                                # # print("x_min",x_min)
                                # stand_l = trackers_cov_old_for_sort[match_column_indice][4]
                                # # trackers_cov_old_for_sort[match_row_indice][0] = x_min + 0.5 * stand_l
                                # # #####*****  当目标的航向角不与坐标轴平行或者垂直时  *****#####
                                # theta = trackers_cov_old_for_sort[match_row_indice, 4]
                                # trackers_cov_old_for_sort[match_row_indice][0] = 0.5 * (x_min + x_max) - 0.5 * stand_l * math.cos((-1*theta-90)*np.pi/180)
                                # trackers_cov_old_for_sort[match_row_indice][1] = 0.5 * (y_min + y_max) - 0.5 * stand_l * math.sin((-1*theta-90)*np.pi/180)
                                # trackers_cov_old_for_sort[match_row_indice][4] = trackers_cov_old_for_sort[match_column_indice][4]
                                # trackers_cov_old_for_sort[match_row_indice][5] = trackers_cov_old_for_sort[match_column_indice][5]
                                # trackers_cov_old_for_sort[match_row_indice][2] = trackers_cov_old_for_sort[match_column_indice][2]
                                # trackers_cov_old_for_sort[match_row_indice][3] = trackers_cov_old_for_sort[match_column_indice][3]
                                trackers_cov_old_for_sort[match_row_indice][7] = trackers_cov_old_for_sort[match_column_indice][7]
                                save_indice.append(match_row_indice)
                                delete_indice.append(match_column_indice)
                        trackers_cov_old_for_sort_new = trackers_cov_old_for_sort[save_indice, :]

                        # # #####*****  但是对于场景中有的误检在目标旁边，采用距离去除  *****#####
                        un_match_object = trackers_cov_old_for_sort[un_match1.tolist(), :]
                        un_match_object = fix_cluster(un_match_object, 1)
                        new_trackers_cov_old_for_sort = np.concatenate((trackers_cov_old_for_sort_new, un_match_object), axis=0)

                        # trackers_cov_old_for_sort_for_view = np.zeros((trackers_cov_old_for_sort.shape[0], 20))
                        # trackers_cov_old_for_sort_for_view[:,
                        # [0, 1, 3, 4, 6, 2, 5, 7, 8, 9, 10, 11, 12, 13, 14,15,16,17,18,19]] = trackers_cov_old_for_sort[:, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,15,16,17,18,19]]

                        print('my Time:', round((time.time()-tWait) *1000, 2), 'ms')
                        npStationdata0 = new_trackers_cov_old_for_sort
                        #tracker###

                        # #####*****  2021-0509 my add  *****#####
                        box_final = np.zeros((len(npStationdata0), 17))
                        box_final[:,[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,13,14,15,16]]=npStationdata0[:, [0, 1, 3, 4, 6, 2, 5, 7, 8, 9, 12, 13, 14,15,16,18,19]]

                        # box_final = np.zeros((len(npStationdata0), 16))
                        # box_final[:,[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,13,14,15]]=npStationdata0[:, [0, 1, 3, 4, 6, 2, 5, 7, 8, 9, 12, 13, 14,15,16,18]]
                        MyLog.writeLog("{}[:{}] - pcAlgOnline step 4 ,fusion time:{}ms".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                                               sys._getframe().f_lineno,round((time.time()-tWait) *1000, 2)), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                        tWait = time.time()
                        trackers_all = mot_tracker_merge.update(box_final,stParam,stPcAlgParam.fConeThreshold)  # 将所有的跟踪到的物体都使用其进行跟踪
                        # print("box_all_Source",trackers_all.shape[0], box_all_Source.shape[0])
                        MyLog.writeLog("{}[:{}] - pcAlgOnline step 5,tracker_time:{} ms"
                                       .format(__file__.split('/')[len(__file__.split('/')) - 1],
                                               sys._getframe().f_lineno,round((time.time()-tWait) *1000, 2)),
                                       LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                        # trackers_covall = np.zeros((trackers_all.shape[0], 20))
                        # trackers_covall[:, [0, 1, 3, 4, 6, 2, 5, 7, 8, 9, 10, 11, 12, 13,14,15,16,17,18]] = trackers_all[:, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,17,18]]


                        trackers_covall = np.zeros((trackers_all.shape[0], 20))
                        trackers_covall[:, [0, 1, 3, 4, 6, 2, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,17,18,19]] = trackers_all[:, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,17,18,19]]
                        trackers_covall[:, 6] = trackers_covall[:, 6] * PI_rad
                    else:
                        trackers_covall = np.zeros((trackers_all.shape[0], 19))
                        trackers_covall = npStationdata0
                        trackers_covall[:,10] = 0
                        trackers_covall[:,11] = 0
                        trackers_covall[:, 6] = trackers_covall[:, 6] * PI_rad

                    box_datall = box_np_ops.center_to_corner_box3d(
                        trackers_covall[:, :3],
                        trackers_covall[:, 3:6],
                        trackers_covall[:, 6],
                        origin=[0.5, 0.5, 0.5],
                        axis=2)

                    boxes_corners_trackersall_ = box_datall.reshape(-1, 24)
                    trackers_cov_ = np.hstack((trackers_covall, boxes_corners_trackersall_))
                    dstData.pcTrackersCov = trackers_covall
                    dstData.pcBaseTrackersCovToVideo = trackers_cov_
                    #trackers_list = []
                    trackers_boxIdMax, dstData.listBaseBoxInfo, trackersToEvent, tTimeStamp = getPcBoxInfo(trackers_cov_, stParam=stParam)
                    trunAlg = math.floor((time.time() - trunAlg_start)*1000)
                    dstData.trunAlg = trunAlg
                    dstData.tTimeStamp = tTimeStamp
                    MyLog.writeLog("{}[:{}] - pcAlgOnline step 6,trunAlg:{}ms".
                                   format(__file__.split('/')[len(__file__.split('/')) - 1],
                                          sys._getframe().f_lineno,trunAlg),
                                   LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                except:
                    traceback.print_exc()
                    MyLog.writeLog("{}[:{}] - get srcData time out, time > 100ms!".format(
                        __file__.split('/')[len(__file__.split('/')) - 1],
                        sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                    continue

            MyLog.writeLog("{}[:{}] - pcAlgOnline step 7".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                 sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
            # tn = time.time()
            srcEventDataQueue.put(trackersToEvent)
            while srcEventDataQueue.qsize() > 1:
                try:
                    srcEventDataQueue.get(True, 0.01)
                except:
                    continue
            #----------------------test----------------------#
            # if stUiParam.bSaveEvent == 2 and my_mkdir('./savedata/EventData/'):
            #     res = np.savetxt('./savedata/EventData/eventDetectData' + str(nFrameIdTemp) + '.csv',trackersToEvent,fmt='%f',delimiter=',')
            if stUiParam.bSaveEvent == 1:
                if my_mkdir(file_name_2):
                    res = np.savetxt('./savedata/EventData/{}/eventDetectData'.format(saveFileTime) + str(nFrameIdTemp) + '.csv',trackersToEvent,fmt='%f',delimiter=',')
            #----------------------test----------------------#
            # # 获取目标所在车道
            # for stBoxInfo in dstData.listBaseBoxInfo:
            #     if stBoxInfo.boxId in stParam.getDictBoxinLane():
            #         stBoxInfo.boxLaneId = stParam.getDictBoxinLane()[stBoxInfo.boxId] % 10
            #     else:
            #         stBoxInfo.boxLaneId = 0
            #
            # print("pcAlg time:%.2fms" % ((time.time() - tTest) * 1000))
            dstResultQueue.put(dstData)
            stParam.setNoPcData(False)
            tLast = time.time()
            if timeExcept.elapsed() > 0:
                timeExmecept.restart()
            #数据发送
            MyLog.writeLog("{}[:{}] - pcAlgOnline step 8".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                 sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
            if stUiParam.nCacheFileCountLimit > 0:
                if not thCache.is_alive():
                    thCache.start()
                else:
                    cacheQueue.put(dstData)

            # else:
            while dstResultQueue.qsize() > stPcAlgParam.nSyncIdLimitPcOnline:
                try:
                    dstResultQueue.get(True, 0.01)
                except:
                    MyLog.writeLog("{}[:{}] - dstResultQueue.get(True, 0.01) err!".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                                          sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                    continue

            if timeTemp.elapsed() > 5000:
                MyLog.writeLog("{}[:{}] - out pc result successful!".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                timeTemp.restart()
            print('Result process Time:', round((time.time() - tTest) * 1000, 2), 'ms')
        except:
            MyLog.writeLog("{}[:{}] - except!"
                           .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno)
                           , LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
            MyLog.writeLog("{}[:{}] - except! Call stack:\n{}"
                           .format(__file__.split('/')[len(__file__.split('/')) - 1],
                                   sys._getframe().f_lineno, traceback.format_exc())
                           , LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
            if timeExcept.elapsed() == 0:
                timeExcept.start()
            elif timeExcept.elapsed() > 3000:
                stParam.setNoPcData(True)
                timeExcept.restart()
                MyLog.writeLog("{}[:{}] - except! setNoPcData(True)!"
                               .format(__file__.split('/')[len(__file__.split('/')) - 1],
                                       sys._getframe().f_lineno)
                               , LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
        # print("proPcAlg time:%.2fms" % ((time.time() - tTest) * 1000))

    MyLog.writeLog("{}[:{}] - exit pcAlgOnline 3...".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)

def pcAlgOnline(stPcAlgParam, dstResultQueue, srcEventDataQueue, pcBaseList, stParam, lockVid):
    MyLog.writeLog("{}[:{}] - pcAlgOnline start, pid={}".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, os.getpid()), LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)

    # 每次启动程序，进入pcalgonline，创建时间戳，以时间戳作为文件名，将org_data和eventData保存在相应路径中
    file_path_orgData = './savedata/org_data/'
    file_path_EvtData = './savedata/EventData/'
    saveFileTime = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    saveFileTime_EvtData = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    saveFileTimeStamp = time.time()
    file_name_orgData = os.path.join(file_path_orgData, '{}'.format(saveFileTime))
    file_name_EvtData = os.path.join(file_path_EvtData, '{}'.format(saveFileTime_EvtData))

    stParam.setAlgChanged(False)
    stParam.setNoPcData(False)
    usStatusLast = -1
    cacheQueue = Queue()
    thCache = threading.Thread(target=cacheThread, args=(cacheQueue, stParam, lockVid))

    # #####*****  2021-0509 my add  *****#####
    # #####*****  加载全域车道中心线信息  *****#####
    global left_center, right_center, medium_center, bmp_img
    mot_tracker_merge = Sort(left_center, medium_center, right_center, bmp_img, stPcAlgParam.nMaxAge, stPcAlgParam.nMinHits,stPcAlgParam.nLastCount)
    # print("stPcAlgParam.nLastCount", stPcAlgParam.nLastCount)
    # t1 = time.time()
    # strExePath = getExcutePath()
    # strFilePath = strExePath + '/Configs/Dev/DevParam.xml'
    # if not my_isExist(strFilePath):
    #     MyLog.writeLog("{}[:{}] - DevParam.xml is not exist!.".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_ERROR, True, True)
    #     return
    # tree = ET.parse(strFilePath)
    # root = tree.getroot()
    # root[2][5].text = str(500 + stPcAlgParam.nLastCount )
    # tree.write(strFilePath)
    # print("stPcAlgParam.nLastCount", round((time.time() - t1)*1000 ,3))

    # mot_tracker_merge = Sort(stPcAlgParam.nMaxAge, stPcAlgParam.nMinHits)
    # 开始循环
    timeTemp = QTime()
    timeTemp.start()
    timeExcept = QTime()
    tLast = time.time()
    nFrameIdTemp = 0
    nFrameIdLast = 0
    nTt = 0
    pcFrameLast = None
    tn = time.time()
    fusion = Double_radar_fusion.Fusion(max_age=4)         # 加入id对匹配
    srcDataList = []
    listNorthAngleT = []
    listBaseStationId = []
    for i in range(len(stParam.getlistAttachedIp())):
        srcDataList.append([time.time(), [None], True,[]])
        '''
        srcDataList:存放基站结果数据的列表:
                [0]:新一帧数据到达的系统时间
                [1]:存放当前帧处理数据的列表，默认长度为1
                [2]:该基站在线状态，在线:True，离线:False
                [3]:历史缓存数据，用于时间匹配，最大长度为10
        '''
        listNorthAngleT.append(stParam.getAngleNorthT(i))
        listBaseStationId.append(0)
    listSortNorthAngleT = sorted(listNorthAngleT)
    for i in range(len(stParam.getlistAttachedIp())):
        if stParam.getHighSpeed():
            listBaseStationId[i] = i + 1
        else:
            listBaseStationId[i] = listSortNorthAngleT.index(listNorthAngleT[i]) + 1
    dictCameraParam = {}
    for index in range(len(stParam.getlistAttachedIp())):
        listCameraDev = stParam.getStationCameraDev(index)
        dictCameraParam[listBaseStationId[index]] = listCameraDev
    dictLaneId = {}

    timeForce10Fps = QTime()
    timeForce10Fps.start()
    bGetBS1data = False
    while True:
        try:
            tTest = time.time()
            MyLog.writeLog("{}[:{}] - pcAlgOnline step 1".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                 sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)

            # 超时则认为没有点云数据或者点云数据异常
            if (time.time() - tLast) > 3 and (not stParam.getPause()) and (stParam.getStatus() == 0 or stParam.getStatus() == 3):
                stParam.setNoPcData(True)
                MyLog.writeLog("{}[:{}] - No pc data or data error! time:{}, bPause:{}, status:{}"
                               .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno,
                                       round((time.time() - tLast), 3), stParam.getPause(),
                                       stParam.getStatus()),
                               LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                tLast = time.time()  # 要重新计时
            nIndex = 0
            if stParam.getStatus() in [0, 5] or stParam.getStatus() in [3, 4]:
                for Basequeue in pcBaseList:
                    try:
                        data = Basequeue.get(True,0.001)
                        srcDataList[nIndex][0] = time.time()
                        if srcDataList[nIndex][1][0] is None:
                            srcDataList[nIndex][1][0] = data
                        else:
                            srcDataList[nIndex][1].pop(0)
                            srcDataList[nIndex][1].append(data)
                        #=====================time match=====================#
                        if nIndex == 0:
                            bGetBS1data = True
                        srcDataList[nIndex][3].append(data)
                        if len(srcDataList[nIndex][3]) > 10:
                            srcDataList[nIndex][3].pop(0)
                        #=====================time match=====================#
                        srcDataList[nIndex][2] = True
                        nIndex += 1
                    except:
                        if (time.time() - srcDataList[nIndex][0]) > 0.5 and srcDataList[nIndex][1][0] is None and srcDataList[nIndex][2]:
                            srcDataList[nIndex][2] = False
                            MyLog.writeLog("{}[:{}] - baseStation{} is offline".
                                           format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno,nIndex),
                                           LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                        nIndex += 1
                        continue
                ngetFlag = False
                #=====================time match=====================#
                # if srcDataList[0][2] and  bGetBS1data and len(srcDataList[0][3]) > 2:
                #     bGetBS1data = False
                #     tBSData1 = srcDataList[0][3][-3][1][0]
                #     srcDataList[0][1][0] = srcDataList[0][3][-3]
                #     for i in range(len(srcDataList)):
                #         if i == 0 or len(srcDataList[i][3]) == 0:
                #             continue
                #         listTError = [abs(listBSData[1][0] - tBSData1) for listBSData in srcDataList[i][3]]
                #         minTError = min(listTError)
                #         print("BS{} time error is {},index is {}".format(i,minTError,listTError.index(minTError)))
                #         if minTError < 0.05:
                #             srcDataList[i][1][0] = copy.deepcopy(srcDataList[i][3][listTError.index(minTError)])
                #         else:
                #             srcDataList[i][1][0] = None
                #     timeForce10Fps.restart()
                #     ngetFlag = True
                # elif not srcDataList[0][2] and timeForce10Fps.elapsed() > 100:
                #     timeForce10Fps.restart()
                #     ngetFlag = True
                #=====================time match=====================#
                if srcDataList[0][1][0] is not None:
                    timeForce10Fps.restart()
                    ngetFlag = True
                elif not srcDataList[0][2] and timeForce10Fps.elapsed() > 100:
                    timeForce10Fps.restart()
                    ngetFlag = True
            else:
                continue
            if (not stParam.getRunning()) or stParam.getAlgChanged():
                MyLog.writeLog("{}[:{}] - exit pcAlgOnline 1...".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                return

            if stParam.getPause() and 1 <= stParam.getStatus() <=2 or not ngetFlag:
                time.sleep(0.01)
                if timeExcept.elapsed() > 0:
                    timeExcept.restart()
                continue
            tWait = time.time()
            MyLog.writeLog("{}[:{}] - pcAlgOnline step 2".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                 sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)

            trunAlg_start = time.time()
            dstData = structData()
            nFrameIdTemp = int((nFrameIdTemp + 1) % MAX_FRAME_ID)
            dstData.nFrameId = nFrameIdTemp

            trackers_covall = None

            # current_time_s, current_time_us = parse_GPS_time()
            # current_timestamp = current_time_s + current_time_us / 1000000.0
            if stParam.getStatus() in [0, 5] or stParam.getStatus() in [3, 4]:
                try:
                    list_matrix_base = []
                    index =0
                    nFrameID = None
                    nEqupId = None
                    for listsrcData in srcDataList:
                        # 当前基站处于离线状态或者当前帧无数据或者当前帧数据长度为0
                        if not listsrcData[2] or listsrcData[1][0] is None or len(listsrcData[1][0][0]) == 0:

                            # 保存单基站数据发送给平台 以基站号作为键将信息保存在字典，用于发送信息时根据基站号取出相应数据
                            dstData.nSingleFrame["{}".format(listBaseStationId[index])] = 0
                            dstData.nSingleStamp["{}".format(listBaseStationId[index])] = time.time()
                            dstData.nSingleEqupId["{}".format(listBaseStationId[index])] = listBaseStationId[index]
                            dstData.recvTime["{}".format(listBaseStationId[index])] = time.time()
                            dstData.stationState["{}".format(listBaseStationId[index])] = 1
                            if not listsrcData[2]:
                               dstData.stationState["{}".format(listBaseStationId[index])] = 2

                            index += 1
                            list_matrix_base.append(None)
                            continue

                        # 当前基站为第一号基站(入口基站)
                        if nFrameID is None:
                            nFrameID = listsrcData[1][0][1][2]
                            nEqupId = listsrcData[1][0][1][1]
                            dstData.nEqupId = nEqupId
                            dstData.headData = tuple([nEqupId[1],nEqupId[0]])

                            # 保存入口基站信息发送给平台 以基站号作为键将信息保存在字典，用于发送信息时根据基站号取出相应数据
                            dstData.n0Frame = (listsrcData[1][0][1][2][2]*256 + listsrcData[1][0][1][2][3])
                            dstData.n0TimeStamp = listsrcData[1][0][1][0]

                        # 保存单基站数据发送给平台 以基站号作为键将信息保存在字典，用于发送信息时根据基站号取出相应数据
                        dstData.nSingleFrame["{}".format(listBaseStationId[index])] = (listsrcData[1][0][1][2][2]*256 + listsrcData[1][0][1][2][3])
                        dstData.nSingleStamp["{}".format(listBaseStationId[index])] = (listsrcData[1][0][1][0])
                        dstData.nSingleEqupId["{}".format(listBaseStationId[index])] = listBaseStationId[index]
                        dstData.recvTime["{}".format(listBaseStationId[index])] = listsrcData[0]
                        dstData.stationState["{}".format(listBaseStationId[index])] = 0

                        data_base = listsrcData[1][0][0]
                        data_base[:, 12] = listBaseStationId[index]
                        list_matrix_base.append(listsrcData[1][0][0])
                        index += 1


                    for i in range(len(srcDataList)):
                        srcDataList[i][1][0] = None

                        ##### 不同路口ip转发的数据进行坐标转换 ####
                    npStationdata0 = None
                    for i in range(len(list_matrix_base)):
                        if list_matrix_base[i] is None:
                            continue
                        if not stParam.getCoordinateFlag() and i != 0:
                            fRotationVector = stParam.getRotationVector(i)
                            fTranslationVector = stParam.getTranslationVector(i)
                            rotation = np.array([fRotationVector[0] * PI_rads, fRotationVector[1] * PI_rads, fRotationVector[2] * PI_rads])
                            #########rotate point ##################
                            rotation_pcd = o3d.geometry.PointCloud()
                            rotation_pcd.points = o3d.utility.Vector3dVector(list_matrix_base[i][:, :3])
                            rotation_pcd.rotate(rotation=rotation,center=False)
                            rotation_points = np.asarray(rotation_pcd.points)
                            list_matrix_base[i][:, :3] = rotation_points + np.array([fTranslationVector[0], fTranslationVector[1], fTranslationVector[2]]).reshape(1,3)
                            list_matrix_base[i][:, 6] -= fRotationVector[2]

                        box_nms=rotate_nms_cc(list_matrix_base[i], 0.01)
                        npStationdata1=list_matrix_base[i][box_nms,:]
                        if npStationdata0 is None:
                            npStationdata0 = npStationdata1
                            continue
                        if False:
                            listStationFusiondata = fusion.run(npStationdata0, npStationdata1)
                            if len(listStationFusiondata) == 0:
                                break
                            npStationFusiondata = np.array(listStationFusiondata)
                            npStationdata0 = np.zeros((len(data_base), 12))
                            npStationdata0 = npStationFusiondata[:,[2,3,7,8,1,9,6,10,11,12,0,4,13,14,15]]
                        else:
                            npStationdata0 = np.concatenate((npStationdata0, npStationdata1), axis=0)

                    if npStationdata0 is None or npStationdata0.shape[0] == 0:
                        continue
                    #----------------------test----------------------#
                    # if stUiParam.bSaveE1 == 1:
                    #     if nFrameIdTemp == 1 and my_isExist('./savedata/org_data/'):
                    #         for strfilename in os.listdir('./savedata/org_data/'):
                    #             path_file = os.path.join('./savedata/org_data/',strfilename)
                    #             if os.path.isfile(path_file):
                    #                 os.remove(path_file)
                    #     if my_mkdir('./savedata/org_data/'):
                    #         res = np.savetxt('./savedata/org_data/orgData' + str(nFrameIdTemp) + '.csv',npStationdata0,fmt='%f',delimiter=',')
                    if stUiParam.bSaveE1 == 1:
                        if int(time.time() - saveFileTimeStamp) > 3600:
                            saveFileTime = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                            file_name_orgData = os.path.join(file_path_orgData, '{}'.format(saveFileTime))
                            saveFileTimeStamp = time.time()
                        if my_mkdir(file_name_orgData):
                            res = np.savetxt('{}/orgData'.format(file_name_orgData) + str(nFrameIdTemp) + '.csv',npStationdata0,fmt='%f',delimiter=',')
                    #----------------------test----------------------#
                    print('taxis:',round((time.time()-tTest)*1000,2),'ms',npStationdata0.shape[0])
                    MyLog.writeLog("{}[:{}] - pcAlgOnline step 3".
                                   format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno),
                                   LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                    # box_nms=rotate_nms_cc(npStationdata0, 0.01)
                    # npStationdata0=npStationdata0[box_nms,:]
                    tWait = time.time()
                    if stParam.getShowRLFusion():
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

                        box_with_min_max, box_corners = get_min_max(trackers_cov_old_for_sort, stPcAlgParam.fConeThreshold)
                        mathch, un_match1, un_match2, cost_matrix = fusion_box(box_corners, box_corners, 0)
                        # trackers_cov_old_for_sort = box_with_min_max[:, [0, 1, 3, 4, 6, 2, 5, 7, 10, 9, 8, 11, 12, 13,14,15,16,17,18,19,20,21]]
                        # trackers_cov_old_for_sort[:, 4] = trackers_cov_old_for_sort[:, 4] * np.pi / 180
                        # mathch, un_match1, un_match2, cost_matrix = associate_detections_to_trackers(trackers_cov_old_for_sort,
                        #                                                                              trackers_cov_old_for_sort)
                        trackers_cov_old_for_sort = box_with_min_max
                        # save_indice = un_match1.tolist()
                        save_indice = []
                        delete_indice = []
                        for i in range(mathch.shape[0]):
                            match_row_indice = mathch[i][0]
                            match_column_indice = mathch[i][1]
                            if mathch[i][0] in delete_indice:
                                continue
                            if trackers_cov_old_for_sort[match_row_indice][17] < trackers_cov_old_for_sort[match_column_indice][17]:
                                # x_min = trackers_cov_old_for_sort[match_row_indice][-4]
                                # y_min = trackers_cov_old_for_sort[match_row_indice][-3]
                                # x_max = trackers_cov_old_for_sort[match_row_indice][-2]
                                # y_max = trackers_cov_old_for_sort[match_row_indice][-1]
                                # # print("x_min",x_min)
                                # stand_l = trackers_cov_old_for_sort[match_column_indice][4]
                                # # trackers_cov_old_for_sort[match_row_indice][0] = x_min + 0.5 * stand_l
                                # # #####*****  当目标的航向角不与坐标轴平行或者垂直时  *****#####
                                # theta = trackers_cov_old_for_sort[match_row_indice, 4]
                                # trackers_cov_old_for_sort[match_row_indice][0] = 0.5 * (x_min + x_max) - 0.5 * stand_l * math.cos((-1*theta-90)*np.pi/180)
                                # trackers_cov_old_for_sort[match_row_indice][1] = 0.5 * (y_min + y_max) - 0.5 * stand_l * math.sin((-1*theta-90)*np.pi/180)
                                # trackers_cov_old_for_sort[match_row_indice][4] = trackers_cov_old_for_sort[match_column_indice][4]
                                # trackers_cov_old_for_sort[match_row_indice][5] = trackers_cov_old_for_sort[match_column_indice][5]
                                # trackers_cov_old_for_sort[match_row_indice][2] = trackers_cov_old_for_sort[match_column_indice][2]
                                # trackers_cov_old_for_sort[match_row_indice][3] = trackers_cov_old_for_sort[match_column_indice][3]
                                trackers_cov_old_for_sort[match_row_indice][7] = trackers_cov_old_for_sort[match_column_indice][7]
                                save_indice.append(match_row_indice)
                                delete_indice.append(match_column_indice)
                        trackers_cov_old_for_sort_new = trackers_cov_old_for_sort[save_indice, :]

                        # # #####*****  但是对于场景中有的误检在目标旁边，采用距离去除  *****#####
                        un_match_object = trackers_cov_old_for_sort[un_match1.tolist(), :]
                        un_match_object = fix_cluster(un_match_object, 1)
                        new_trackers_cov_old_for_sort = np.concatenate((trackers_cov_old_for_sort_new, un_match_object), axis=0)

                        # trackers_cov_old_for_sort_for_view = np.zeros((trackers_cov_old_for_sort.shape[0], 20))
                        # trackers_cov_old_for_sort_for_view[:,
                        # [0, 1, 3, 4, 6, 2, 5, 7, 8, 9, 10, 11, 12, 13, 14,15,16,17,18,19]] = trackers_cov_old_for_sort[:, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,15,16,17,18,19]]

                        print('my Time:', round((time.time()-tWait) *1000, 2), 'ms')
                        npStationdata0 = new_trackers_cov_old_for_sort
                        #tracker###
                        # #####*****  2021-0509 my add  *****#####
                        box_final = np.zeros((len(npStationdata0), 17))
                        box_final[:,[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,13,14,15,16]]=npStationdata0[:, [0, 1, 3, 4, 6, 2, 5, 7, 8, 9, 12, 13, 14,15,16,18,19]]

                        # box_final = np.zeros((len(npStationdata0), 16))
                        # box_final[:,[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,13,14,15]]=npStationdata0[:, [0, 1, 3, 4, 6, 2, 5, 7, 8, 9, 12, 13, 14,15,16,18]]
                        MyLog.writeLog("{}[:{}] - pcAlgOnline step 4 ,fusion time:{}ms".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                                               sys._getframe().f_lineno,round((time.time()-tWait) *1000, 2)), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                        tWait = time.time()
                        trackers_all = mot_tracker_merge.update(box_final,stParam,stPcAlgParam.fConeThreshold)  # 将所有的跟踪到的物体都使用其进行跟踪
                        # print("box_all_Source",trackers_all.shape[0], box_all_Source.shape[0])
                        MyLog.writeLog("{}[:{}] - pcAlgOnline step 5,tracker_time:{} ms"
                                       .format(__file__.split('/')[len(__file__.split('/')) - 1],
                                               sys._getframe().f_lineno,round((time.time()-tWait) *1000, 2)),
                                       LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                        # trackers_covall = np.zeros((trackers_all.shape[0], 20))
                        # trackers_covall[:, [0, 1, 3, 4, 6, 2, 5, 7, 8, 9, 10, 11, 12, 13,14,15,16,17,18]] = trackers_all[:, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,17,18]]


                        trackers_covall = np.zeros((trackers_all.shape[0], 20))
                        trackers_covall[:, [0, 1, 3, 4, 6, 2, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,17,18,19]] = trackers_all[:, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,17,18,19]]
                        trackers_covall[:, 6] = trackers_covall[:, 6] * PI_rad
                    else:
                        trackers_covall = np.zeros((trackers_all.shape[0], 20))
                        trackers_covall = npStationdata0
                        add_dimension = np.zeros((npStationdata0.shape[0], 1))
                        trackers_covall = np.concatenate((trackers_covall, add_dimension), axis=1)
                        trackers_covall[:,10] = 0
                        trackers_covall[:,11] = 0
                        trackers_covall[:, 6] = trackers_covall[:, 6] * PI_rad

                    box_datall = box_np_ops.center_to_corner_box3d(
                        trackers_covall[:, :3],
                        trackers_covall[:, 3:6],
                        trackers_covall[:, 6],
                        origin=[0.5, 0.5, 0.5],
                        axis=2)

                    boxes_corners_trackersall_ = box_datall.reshape(-1, 24)
                    trackers_cov_ = np.hstack((trackers_covall, boxes_corners_trackersall_))
                    dstData.pcTrackersCov = trackers_covall
                    dstData.pcBaseTrackersCovToVideo = trackers_cov_
                    #trackers_list = []
                    trackers_boxIdMax, dstData.listBaseBoxInfo, trackersToEvent, tTimeStamp = getPcBoxInfo(trackers_cov_, stParam=stParam)
                    trunAlg = math.floor((time.time() - trunAlg_start)*1000)
                    dstData.trunAlg = trunAlg
                    dstData.tTimeStamp = tTimeStamp
                    MyLog.writeLog("{}[:{}] - pcAlgOnline step 6,trunAlg:{}ms".
                                   format(__file__.split('/')[len(__file__.split('/')) - 1],
                                          sys._getframe().f_lineno,trunAlg),
                                   LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                except:
                    traceback.print_exc()
                    MyLog.writeLog("{}[:{}] - get srcData time out, time > 100ms!".format(
                        __file__.split('/')[len(__file__.split('/')) - 1],
                        sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                    continue

            MyLog.writeLog("{}[:{}] - pcAlgOnline step 7".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                 sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
            # tn = time.time()
            srcEventDataQueue.put(trackersToEvent)
            while srcEventDataQueue.qsize() > 1:
                try:
                    srcEventDataQueue.get(True, 0.01)
                except:
                    continue
            #----------------------test----------------------#
            # if stUiParam.bSaveEvent == 2 and my_mkdir('./savedata/EventData/'):
            #     res = np.savetxt('./savedata/EventData/eventDetectData' + str(nFrameIdTemp) + '.csv',trackersToEvent,fmt='%f',delimiter=',')
            if stUiParam.bSaveEvent == 1:
                if my_mkdir(file_name_EvtData):
                    res = np.savetxt('./savedata/EventData/{}/eventDetectData'.format(saveFileTime_EvtData) + str(nFrameIdTemp) + '.csv',trackersToEvent,fmt='%f',delimiter=',')
            #----------------------test----------------------#
            # # 获取目标所在车道
            # for stBoxInfo in dstData.listBaseBoxInfo:
            #     if stBoxInfo.boxId in stParam.getDictBoxinLane():
            #         stBoxInfo.boxLaneId = stParam.getDictBoxinLane()[stBoxInfo.boxId] % 10
            #     else:
            #         stBoxInfo.boxLaneId = 0
            #
            # print("pcAlg time:%.2fms" % ((time.time() - tTest) * 1000))
            dstResultQueue.put(dstData)
            stParam.setNoPcData(False)
            tLast = time.time()
            if timeExcept.elapsed() > 0:
                timeExmecept.restart()
            #数据发送
            MyLog.writeLog("{}[:{}] - pcAlgOnline step 8".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                 sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
            if stUiParam.nCacheFileCountLimit > 0:
                if not thCache.is_alive():
                    thCache.start()
                else:
                    cacheQueue.put(dstData)

            # else:
            while dstResultQueue.qsize() > stPcAlgParam.nSyncIdLimitPcOnline:
                try:
                    dstResultQueue.get(True, 0.01)
                except:
                    MyLog.writeLog("{}[:{}] - dstResultQueue.get(True, 0.01) err!".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                                          sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                    continue

            if timeTemp.elapsed() > 5000:
                MyLog.writeLog("{}[:{}] - out pc result successful!".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
                timeTemp.restart()
            print('Result process Time:', round((time.time() - tTest) * 1000, 2), 'ms')
            MyLog.writeLog("{}[:{}] - Result process Time{} 'ms'!"
                           .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno,round((time.time() - tTest) * 1000, 2))
                           , LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)
        except:
            MyLog.writeLog("{}[:{}] - except!"
                           .format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno)
                           , LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
            MyLog.writeLog("{}[:{}] - except! Call stack:\n{}"
                           .format(__file__.split('/')[len(__file__.split('/')) - 1],
                                   sys._getframe().f_lineno, traceback.format_exc())
                           , LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
            if timeExcept.elapsed() == 0:
                timeExcept.start()
            elif timeExcept.elapsed() > 3000:
                stParam.setNoPcData(True)
                timeExcept.restart()
                MyLog.writeLog("{}[:{}] - except! setNoPcData(True)!"
                               .format(__file__.split('/')[len(__file__.split('/')) - 1],
                                       sys._getframe().f_lineno)
                               , LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
        # print("proPcAlg time:%.2fms" % ((time.time() - tTest) * 1000))

    MyLog.writeLog("{}[:{}] - exit pcAlgOnline 3...".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)

###################现场的orgData离线数据，做融合与跟踪后输入给显示和事件检测################
def pcAlgOffline(stPcAlgParam, dstResultQueue, srcEventDataQueue, pcBaseList, stParam, lockVid):
    MyLog.writeLog("{}[:{}] - pcAlgOffline start, pid={}".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, os.getpid()), LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)
    stParam.setAlgChanged(False)
    stParam.setNoPcData(False)
    tLoop = QTime()
    tLoop.start()
    stTrasform = None

    # #####*****  加载全域车道中心线信息  *****#####
    global left_center, right_center, medium_center, bmp_img
    mot_tracker_merge = Sort(left_center, medium_center, right_center, bmp_img, stPcAlgParam.nMaxAge, stPcAlgParam.nMinHits, stPcAlgParam.nLastCount)

    # mot_tracker_merge = Sort(stPcAlgParam.nMaxAge, stPcAlgParam.nMinHits)
    while True:
        try:
            while tLoop.elapsed() < 100:
                time.sleep(0.001)
            tLoop.restart()
            npStationdata0 = None
            while npStationdata0 is None:
                if pcBaseList[0].qsize() > 0:
                    npStationdata0,nFrameIdTemp = pcBaseList[0].get(True, 0.01)
            tTest = time.time()
            dstData = structData()
            trunAlg_start = time.time()
            dstData.nFrameId = nFrameIdTemp
            MyLog.writeLog("{}[:{}] - pcAlgOffline step 1".
                           format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno),
                           LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
            trackers_cov_old_for_sort = npStationdata0

            # #####*****  2021-0509 my add  *****#####
            # #####*****  在去重前进行车道航向角的规定  *****#####
            add_dimension = np.zeros((trackers_cov_old_for_sort.shape[0], 1))
            trackers_cov_old_for_sort = np.concatenate((trackers_cov_old_for_sort, add_dimension), axis=1)
            trackers_cov_old_for_sort = cal_box_lane(bmp_img, trackers_cov_old_for_sort)
            # #####*****  flag是标志位, 1表示第7列是航向角, 0表示第5列是航向角  *****#####
            flag = 1
            trackers_cov_old_for_sort = cal_map_angle(trackers_cov_old_for_sort, left_center, medium_center, right_center, flag)

            box_with_min_max, box_corners = get_min_max(trackers_cov_old_for_sort, stPcAlgParam.fConeThreshold)
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
                    # trackers_cov_old_for_sort[match_row_indice][4] = trackers_cov_old_for_sort[match_column_indice][4]
                    # trackers_cov_old_for_sort[match_row_indice][5] = trackers_cov_old_for_sort[match_column_indice][5]
                    trackers_cov_old_for_sort[match_row_indice][7] = trackers_cov_old_for_sort[match_column_indice][7]
                    save_indice.append(match_row_indice)
                    delete_indice.append(match_column_indice)
            trackers_cov_old_for_sort_new = trackers_cov_old_for_sort[save_indice, :]

            # # #####*****  但是对于场景中有的误检在目标旁边，采用距离去除  *****#####
            un_match_object = trackers_cov_old_for_sort[un_match1.tolist(), :]
            un_match_object = fix_cluster(un_match_object, 1)
            new_trackers_cov_old_for_sort = np.concatenate((trackers_cov_old_for_sort_new, un_match_object), axis=0)
            npStationdata0 = new_trackers_cov_old_for_sort
            #tracker###

            # #####*****  2021-0509 my add  *****#####
            box_final = np.zeros((len(npStationdata0), 17))
            box_final[:,[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,13,14,15,16]]=npStationdata0[:, [0, 1, 3, 4, 6, 2, 5, 7, 8, 9, 12, 13, 14,15,16,18,19]]

            # box_final = np.zeros((len(npStationdata0), 16))
            # box_final[:,[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,13,14,15]]=npStationdata0[:, [0, 1, 3, 4, 6, 2, 5, 7, 8, 9, 12, 13, 14,15,16,18]]
            MyLog.writeLog("{}[:{}] - pcAlgOffline step 2".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                 sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
            tWait = time.time()
            trackers_all = mot_tracker_merge.update(box_final,stParam,stPcAlgParam.fConeThreshold)  # 将所有的跟踪到的物体都使用其进行跟踪
            MyLog.writeLog("{}[:{}] - pcAlgOffline step 3".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                 sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
            print('tracker_time Time:', round((time.time()-tWait) *1000, 2), 'ms')
            trackers_covall = np.zeros((trackers_all.shape[0], 20))
            trackers_covall[:, [0, 1, 3, 4, 6, 2, 5, 7, 8, 9, 10, 11, 12, 13,14,15,16,17,18]] = trackers_all[:, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,17,18]]
            trackers_covall[:, 6] = trackers_covall[:, 6] * PI_rad


            ############ 生成boundbox #########################
            box_datall = box_np_ops.center_to_corner_box3d(
                trackers_covall[:, :3],
                trackers_covall[:, 3:6],
                trackers_covall[:, 6],
                origin=[0.5, 0.5, 0.5],
                axis=2)

            dstData.pcBox = box_datall
            dstData.nEqupId = struct.unpack('2B',struct.pack('>H',0))
            dstData.headData = tuple([dstData.nEqupId[1],dstData.nEqupId[0]])

            boxes_corners_trackersall_ = box_datall.reshape(-1, 24)
            trackers_cov_ = np.hstack((trackers_covall, boxes_corners_trackersall_))
            dstData.pcTrackersCov = trackers_covall
            dstData.pcBaseTrackersCovToVideo = trackers_cov_
            #trackers_list = []
            trackers_boxIdMax, dstData.listBaseBoxInfo, trackersToEvent, tTimeStamp = getPcBoxInfo(trackers_cov_, stParam=stParam)
            MyLog.writeLog("{}[:{}] - pcAlgOffline step 4".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                                     sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
            print('algoffline time',round((time.time() - tTest)*1000,2))



            trunAlg = math.floor((time.time() - trunAlg_start)*1000)
            dstData.trunAlg = trunAlg
            dstData.tTimeStamp = tTimeStamp
            # 获取目标所在车道
            # ttmp = time.time()
            # for stBoxInfo in dstData.listBaseBoxInfo:
            #     if stBoxInfo.boxId in stParam.getDictBoxinLane():
            #         stBoxInfo.boxLaneId = stParam.getDictBoxinLane()[stBoxInfo.boxId] % 10
            #     else:
            #         stBoxInfo.boxLaneId = 0
            # print('lane time',time.time() - ttmp)

            srcEventDataQueue.put(trackersToEvent)
            #----------------------test----------------------#
            # if stUiParam.bSaveEvent == 1:
            #     if nFrameIdTemp == 1 and my_isExist('./savedata/EventData/'):
            #         for strfilename in os.listdir('./savedata/EventData/'):
            #             path_file = os.path.join('./savedata/EventData/',strfilename)
            #             if os.path.isfile(path_file):
            #                 os.remove(path_file)
            #     if my_mkdir('./savedata/EventData/'):
            #         res = np.savetxt('./savedata/EventData/eventDetectData' + str(nFrameIdTemp) + '.csv',trackersToEvent,fmt='%f',delimiter=',')
            #----------------------test----------------------#
            dstResultQueue.put(dstData)
        except:
            print(traceback.format_exc())
            continue
###################给平台修数据的算法,仅做10帧定时和结构化输出################
def pcAlgOffline_modify_1(dstResultQueue, srcEventDataQueue, pcBaseList, stParam, lockVid):
    MyLog.writeLog("{}[:{}] - pcAlgOffline start, pid={}".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, os.getpid()), LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)
    stParam.setAlgChanged(False)
    stParam.setNoPcData(False)
    tLoop = QTime()
    tLoop.start()
    stTrasform = None
    bDel = True
    while True:
        try:
            while tLoop.elapsed() < 100:
                time.sleep(0.001)
            npFrameData = None
            while npFrameData is None:
                if pcBaseList[0].qsize() > 0:
                    try:
                        npFrameData, nFrameIdTemp = pcBaseList[0].get(True, 0.01)
                    except:
                        continue
                    stDstData = structData()
                    stDstData.nFrameId = nFrameIdTemp
            tLoop.restart()
            if stTrasform is None:
                fLongitude = stParam.getLidarLongitude(0)
                fLatitude = stParam.getLidarLatitude(0)
                fAngle = stParam.getAngleNorthT(0)
                stTrasform = Lon_Lat_Meter_Transform(fLongitude, fLatitude, fAngle)
            npFrameTarget =npFrameData[1:,[21,22,23,19,18,20,17,8,16,7,9,11,25,0,10,13,14,27]].astype(np.float64)
            # npFrameTarget =npFrameData[1:,[21,22,23,19,18,20,17,8,16,7,9,11,25,26,10,13,14,27]].astype(np.float64)
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
            npFrameTarget[:,3:6] = npFrameTarget[:,3:6] / 100
            if '.txt' in stParam.getListOfflineFiles()[0] or '.json' in stParam.getListOfflineFiles()[0] or '.csv' in stParam.getListOfflineFiles()[0]:
                # if '.txt' in stParam.getListOfflineFiles()[0] or '.json' in stParam.getListOfflineFiles()[0]:
                npFrameTarget[:,0:2] = stTrasform.lonlat_to_xy_of_draw(npFrameTarget[:,15:17])
                npFrameTarget[:,2] = -4.2
            npFrameTarget[:,6] = (npFrameTarget[:,6] + 180 - fAngle + 360) % 360
            npFrameTarget[:,6] = npFrameTarget[:,6] * PI_rad
            box_datall = box_np_ops.center_to_corner_box3d(
                npFrameTarget[:, :3],
                npFrameTarget[:, 3:6],
                npFrameTarget[:, 6],
                origin=[0.5, 0.5, 0.5],
                axis=2)

            box_all = box_datall.reshape(-1, 24)
            trackers_cov_ = np.hstack((npFrameTarget, box_all))
            stDstData.pcTrackersCov = npFrameTarget
            stDstData.pcBaseTrackersCovToVideo = trackers_cov_
            listBoxInfo = []
            trackersToEvent = np.zeros((npFrameTarget.shape[0], 44))
            for i in range(npFrameTarget.shape[0]):
                stBoxInfo = structBoxInfo()
                # 获取目标框数据，用于填充表格
                stBoxInfo.boxCenter = npFrameTarget[i, 0:3]
                stBoxInfo.boxSize = npFrameTarget[i, 3:6]
                stBoxInfo.boxAngle = float(npFrameTarget[i, 6])
                nType = int(npFrameTarget[i, 7])
                stBoxInfo.boxSpeed = float(npFrameTarget[i, 8]/100)
                stBoxInfo.boxId = int(npFrameTarget[i][9])
                stBoxInfo.boxVertexPc = box_datall[i]
                stBoxInfo.nPoints = 0
                stBoxInfo.strClass = switch_pc[nType]
                stBoxInfo.strSource = "PointCloud"
                if npFrameTarget[i, 11] == 3:
                    stBoxInfo.strSource = "Radar"
                elif npFrameTarget[i, 11] == 4:
                    stBoxInfo.strSource = "PcRfusion"
                stBoxInfo.boxLongitude = npFrameTarget[i, 15]
                stBoxInfo.boxLatitude = npFrameTarget[i, 16]
                stBoxInfo.nConfidence = int(npFrameTarget[i, 10] * 100)
                stBoxInfo.nBaseStationId = npFrameTarget[i, 12]
                stBoxInfo.nBaseStationBoxId = npFrameTarget[i, 13]
                stBoxInfo.fcolor = npFrameTarget[i, 14]
                stBoxInfo.nLane = npFrameTarget[i, 17]
                listBoxInfo.append(stBoxInfo)
            trackersToEvent[:, 0: 12] = npFrameTarget[:, 0: 12]
            trackersToEvent[:, 12] = npFrameTarget[:, 13]
            trackersToEvent[:, 13] = npFrameTarget[:, 12]
            trackersToEvent[:, 14:18] = npFrameTarget[:, 14:18]
            trackersToEvent[:, 18:20] = npFrameTarget[:, 0:2]
            trackersToEvent[:, 20:] = box_all
            stDstData.listBoxInfo = listBoxInfo
            stDstData.listBaseBoxInfo = listBoxInfo
            stDstData.pcBox = box_datall
            stDstData.trunAlg = 0
            stDstData.nEqupId = struct.unpack('2B',struct.pack('>H',int(float(npFrameData[1,0]))))
            stDstData.headData = tuple([stDstData.nEqupId[1],stDstData.nEqupId[0]])
            srcEventDataQueue.put(trackersToEvent)

            dstResultQueue.put(stDstData)

        except:
            print(traceback.format_exc())
            continue

def pcAlgOffline_modify(dstResultQueue, srcEventDataQueue, pcBaseList, stParam, lockVid):
    MyLog.writeLog("{}[:{}] - pcAlgOffline start, pid={}".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno, os.getpid()), LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)
    stParam.setAlgChanged(False)
    stParam.setNoPcData(False)
    tLoop = QTime()
    tLoop.start()
    stTrasform = None
    bDel = True
    while True:
        try:
            while tLoop.elapsed() < 100:
                time.sleep(0.001)
            npFrameData = None
            while npFrameData is None:
                if pcBaseList[0].qsize() > 0:
                    try:
                        npFrameData, nFrameIdTemp = pcBaseList[0].get(True, 0.01)
                        # print("npFrameData", npFrameData)
                    except:
                        continue
                    stDstData = structData()
                    stDstData.nFrameId = nFrameIdTemp
            tLoop.restart()
            if stTrasform is None:
                fLongitude = stParam.getLidarLongitude(0)
                fLatitude = stParam.getLidarLatitude(0)
                fAngle = stParam.getAngleNorthT(0)
                stTrasform = Lon_Lat_Meter_Transform(fLongitude, fLatitude, fAngle)
            # npFrameTarget =npFrameData[1:,[21,22,23,19,18,20,17,8,16,7,9,11,25,0,10,13,14,27]].astype(np.float64)
            npFrameTarget =npFrameData[1:,[21,22,23,19,18,20,17,8,16,7,9,11,25,28,10,13,14,27,26]].astype(np.float64)
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
            '''
            npBoxInfo[i, :] = np.array([Equiq_id[0], frame_id[0], time_Stamp, logti[0], lat[0],     0-4
                                       angle_north[0],trans_num[0],object_id[0], getPcEnumFromStr(switch_send[object_type[0]]),object_confidence[0]/100,    5-9
                                       object_color[0],object_source[0],object_laneid[0],object_log[0] / 1e7, object_lat[0] / 1e7,          10-14
                                        object_alti[0] / 100, object_speed[0],object_pitch[0],object_length[0],object_wide[0],             15- 19
                                        object_height[0],object_center_x[0] / 100,object_center_y[0] / 100, object_center_z[0] / 100, frame_id[0],   20- 24
                                         object_basestationId[0],object_singleFrame[0], object_laneid[0],object_orgId[0],0,0, 0,0,0,0])       25 --
            '''



            npFrameTarget[:,3:6] = npFrameTarget[:,3:6] / 100
            if '.txt' in stParam.getListOfflineFiles()[0] or '.json' in stParam.getListOfflineFiles()[0] or '.csv' in stParam.getListOfflineFiles()[0]:
                # if '.txt' in stParam.getListOfflineFiles()[0] or '.json' in stParam.getListOfflineFiles()[0]:
                npFrameTarget[:,0:2] = stTrasform.lonlat_to_xy_of_draw(npFrameTarget[:,15:17])
                npFrameTarget[:,2] = -4.2
            npFrameTarget[:,6] = (npFrameTarget[:,6] + 180 - fAngle + 360) % 360
            npFrameTarget[:,6] = npFrameTarget[:,6] * PI_rad
            box_datall = box_np_ops.center_to_corner_box3d(
                npFrameTarget[:, :3],
                npFrameTarget[:, 3:6],
                npFrameTarget[:, 6],
                origin=[0.5, 0.5, 0.5],
                axis=2)

            box_all = box_datall.reshape(-1, 24)
            trackers_cov_ = np.hstack((npFrameTarget, box_all))
            stDstData.pcTrackersCov = npFrameTarget
            stDstData.pcBaseTrackersCovToVideo = trackers_cov_
            listBoxInfo = []
            trackersToEvent = np.zeros((npFrameTarget.shape[0], 44))
            for i in range(npFrameTarget.shape[0]):
                stBoxInfo = structBoxInfo()
                # 获取目标框数据，用于填充表格
                stBoxInfo.boxCenter = npFrameTarget[i, 0:3]
                stBoxInfo.boxSize = npFrameTarget[i, 3:6]
                stBoxInfo.boxAngle = float(npFrameTarget[i, 6])
                nType = int(npFrameTarget[i, 7])
                stBoxInfo.boxSpeed = float(npFrameTarget[i, 8]/100)
                stBoxInfo.boxId = int(npFrameTarget[i][9])
                stBoxInfo.boxVertexPc = box_datall[i]
                stBoxInfo.nPoints = 0
                stBoxInfo.strClass = switch_pc[nType]
                stBoxInfo.strSource = "PointCloud"
                if npFrameTarget[i, 11] == 3:
                    stBoxInfo.strSource = "Radar"
                elif npFrameTarget[i, 11] == 4:
                    stBoxInfo.strSource = "PcRfusion"
                stBoxInfo.boxLongitude = npFrameTarget[i, 15]
                stBoxInfo.boxLatitude = npFrameTarget[i, 16]
                stBoxInfo.nConfidence = int(npFrameTarget[i, 10] * 100)
                stBoxInfo.nBaseStationId = npFrameTarget[i, 12]
                stBoxInfo.nBaseStationBoxId = npFrameTarget[i, 13]
                stBoxInfo.fcolor = npFrameTarget[i, 14]
                stBoxInfo.nLane = npFrameTarget[i, 17]
                stBoxInfo.nPreFrameCnt = npFrameTarget[i, 18]

                listBoxInfo.append(stBoxInfo)
            trackersToEvent[:, 0: 12] = npFrameTarget[:, 0: 12]
            trackersToEvent[:, 12] = npFrameTarget[:, 13]
            trackersToEvent[:, 13] = npFrameTarget[:, 12]
            trackersToEvent[:, 14:18] = npFrameTarget[:, 14:18]
            trackersToEvent[:, 18:20] = npFrameTarget[:, 0:2]
            trackersToEvent[:, 20:] = box_all
            stDstData.listBoxInfo = listBoxInfo
            stDstData.listBaseBoxInfo = listBoxInfo
            stDstData.pcBox = box_datall
            stDstData.trunAlg = 0
            stDstData.nEqupId = struct.unpack('2B',struct.pack('>H',int(float(npFrameData[1,0]))))
            stDstData.headData = tuple([stDstData.nEqupId[1],stDstData.nEqupId[0]])
            srcEventDataQueue.put(trackersToEvent)

            dstResultQueue.put(stDstData)

        except:
            print(traceback.format_exc())
            continue




def call_back():
    print("suc")

def call_back_err():
    print("err")

# =============================== 算法进程执行函数 ================================


def proPcAlg(dstResultQueue, srcEventDataQueue, pcBaseList, stParam, lockVid):
    stPcAlgParam = structPcAlgParam()
    serializationParam(stPcAlgParam=stPcAlgParam)

    while stParam.getRunning():
        if stParam.getOnline():
            pcAlgOnline(stPcAlgParam, dstResultQueue, srcEventDataQueue, pcBaseList, stParam, lockVid)
        else:
            if 'orgData' in stParam.getListOfflineFiles()[0]:
                pcAlgOffline(stPcAlgParam, dstResultQueue, srcEventDataQueue, pcBaseList, stParam, lockVid)
            else:
                pcAlgOffline_modify(dstResultQueue, srcEventDataQueue, pcBaseList, stParam, lockVid)
    MyLog.writeLog("{}[:{}] - exit proPcAlg...".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)

# =================== 添加 点云算法 结束 =====================

def proEventDetect(srcEventDataQueue, dstEventResultQueue, stParam, lockVid):
    MyLog.writeLog("{}[:{}] - proEventDetect start".format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno), LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
    listNorthAngleT = []
    listBaseStationId = []
    for i in range(len(stParam.getlistAttachedIp())):
        listNorthAngleT.append(stParam.getAngleNorthT(i))
        listBaseStationId.append(0)
    listSortNorthAngleT = sorted(listNorthAngleT)
    for i in range(len(stParam.getlistAttachedIp())):
        listBaseStationId[i] = listSortNorthAngleT.index(listNorthAngleT[i]) + 1
    dictCameraParam = {}
    for index in range(len(stParam.getlistAttachedIp())):
        if stParam.getHighSpeed():
            nBaseStationId = index + 1
        else:
            nBaseStationId = listBaseStationId[index]
        listCameraDev = stParam.getStationCameraDev(index)
        dictCameraParam[nBaseStationId] = listCameraDev
    traffic_flow = None
    try:
        traffic_flow=Traffic_Flow('./Configs/Video/chedao/virtual_config/', len(stParam.getlistAttachedIp()))#qifu
        MyLog.writeLog("{}[:{}] - Traffic_Flow init sucessful".
                       format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno),
                       LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
    except:
        MyLog.writeLog("{}[:{}] - Traffic_Flow init Failed".
                       format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno),
                       LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)
    use_camera_map=Use_Camera_Map(dictCameraParam)
    listEventTime = []
    for i in range(6):
        listEventTime.append(QTime())
        listEventTime[i].start()
    while stParam.getRunning() and traffic_flow is not None:
        try:
            trackersToEvent = srcEventDataQueue.get(True, 0.01)
        except:
            time.sleep(0.01)
            continue

        B0, B1, B2, B3, B4, B5= [], [], [], [], [], []
        try:
            # 进行统计统计量  B1与B4应该同频率
            tstart = time.time()
            if listEventTime[4].elapsed() >= stUiParam.nPlatformB4Frequency * 1000:  # 1/60Hz
                traffic_flow.use(trackersToEvent, True)
                B4 = traffic_flow.get_B4()
                listEventTime[4].restart()
                if stUiParam.bSaveEvent == 1 and my_mkdir('./savedata/Eventlog/'):
                    with open(r'./savedata/Eventlog/B4get.log','a') as log_file:
                        tEvent = time.time()
                        strTimeStamp = getTimeStamp(tEvent)
                        log_file.write(strTimeStamp + " - " + str(B4[6]) + "\n")
            else:
                traffic_flow.use(trackersToEvent, False)
            if not stParam.getHighSpeed():
                if listEventTime[0].elapsed() >= nPlatformB0Frequency * 1000:  # 10Hz
                    B0 = traffic_flow.get_B0()
                    listEventTime[0].restart()

                if listEventTime[1].elapsed() >= nPlatformB1Frequency * 1000:  # 1/60Hz
                    B1 = traffic_flow.get_B1()
                    listEventTime[1].restart()

                if listEventTime[2].elapsed() >= nPlatformB2Frequency * 1000:  # 10Hz
                    B2 = traffic_flow.get_B2()
                    listEventTime[2].restart()
            else:
                if listEventTime[3].elapsed() >= stUiParam.nPlatformB3Frequency * 1000:  # 1Hz
                    B3 = traffic_flow.get_B3()
                    listEventTime[3].restart()
                    if stUiParam.bSaveEvent == 1 and my_mkdir('./savedata/Eventlog/'):
                        with open(r'./savedata/Eventlog/B3get.log','a') as log_file:
                            tEvent = time.time()
                            strTimeStamp = getTimeStamp(tEvent)
                            log_file.write(strTimeStamp + " - " + str(B3[4]) + "\n")

                # if listEventTime[4].elapsed() >= stUiParam.nPlatformB4Frequency * 1000:  # 1/60Hz
                #     B4 = traffic_flow.get_B4()
                #     listEventTime[4].restart()
                #     if stUiParam.bSaveEvent == 1 and my_mkdir('./savedata/Eventlog/'):
                #         with open(r'./savedata/Eventlog/B4get.log','a') as log_file:
                #             tEvent = time.time()
                #             strTimeStamp = getTimeStamp(tEvent)
                #             log_file.write(strTimeStamp + " - " + str(B4[6]) + "\n")


                # if listEventTime[5].elapsed() >= nPlatformB5Frequency * 1000:  # 0.2Hz
                B5 = traffic_flow.get_B5()
                # listEventTime[5].restart()
                if stUiParam.bSaveEvent == 1 and my_mkdir('./savedata/Eventlog/'):
                    with open(r'./savedata/Eventlog/B5get.log','a') as log_file:
                        tEvent = time.time()
                        strTimeStamp = getTimeStamp(tEvent)
                        strB5 = ''
                        if B5[4] == 1:
                            strB5 = strB5 + str(B5[6])
                        log_file.write(strTimeStamp + " - " + str(B5[4]) +"\n" + strB5 + "\n")

        except:
            MyLog.writeLog("{}[:{}] - Event detect alg Error !usetime:{}ms".
                           format(__file__.split('/')[len(__file__.split('/')) - 1],
                                  sys._getframe().f_lineno,round((time.time()-tstart)*1000,2)), LOG_ERROR, ENABLE_WRITE_LOG, ENABLE_PRINT)
            with open(r'./savedata/Eventlog/Eventerror.log','a') as log_file:
                tEvent = time.time()
                strTimeStamp = getTimeStamp(tEvent)
                log_file.write(strTimeStamp +"\n" + '{}'.format(traceback.format_exc()) + "\n")

        if len(B3) != 0 or len(B4) != 0 or (len(B5) != 0 and B5[4] != 0):
            dstEventResultQueue.put((B0, B1, B2, B3, B4, B5))
        stParam.setDictBoxinLane(traffic_flow.cur_id_lane)
    MyLog.writeLog("{}[:{}] - exit proEventDetect...".
                   format(__file__.split('/')[len(__file__.split('/')) - 1], sys._getframe().f_lineno),
                   LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)


