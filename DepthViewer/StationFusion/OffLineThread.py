from PyQt5.Qt import *
import timeit
# from Functions import GetFusionVideo
import cv2
from decimal import Decimal
from Functions.AnalysisParam import *
from Functions import TwoRadarFusion, Double_radar_fusion
from Functions import sort_radar_lin
from copy import deepcopy
import numpy as np
import os



class OffLineThread(QThread):
    drawSig = pyqtSignal(object, object, object)  # 更新相机信号
    updateRadar = pyqtSignal(list, list, int, str, float, int)     # 更新雷达信号
    # updateRadar = pyqtSignal(list, list, list, list, list)
    finished = pyqtSignal()     # 显示完毕信号
    alive = True        # 控制线程存活
    running = False     # 控制线程运行
    currentTime = 0   # 当前播放时间,控制播放间隔和回放,0.1s为最小单位,整除interval即运行一次

    def __init__(self, record, videoName, stVideoAlgParam, interval):
        super(OffLineThread, self).__init__()
        self.record = record  # 离线数据
        self.videoName = videoName    # 离线视频文件
        self.currentIndex = -1      # 读取数据下标
        self.runIndex = -1        # 当前运行下标
        self.cache = {}     # 帧的缓存，最大100
        self.cap = cv2.VideoCapture(self.videoName)
        self.stVideoAlgParam = stVideoAlgParam
        self.interval = interval
        self.cond = QWaitCondition()       # 多线程同步
        self.mutex = QMutex()

        self.len_record = min(len(self.record[0]), len(self.record[1]))
        print('两离线数据最大长度', self.len_record)
        '''157-158外参'''
        # self.R = np.array([[-0.99996906, 0.00786675],[-0.00786675, -0.99996906]])
        # self.t = np.array([187.83734339, -11.88337143])
        '''158-159外参'''
        # self.R = np.array([[-0.99921932, -0.03950628],[0.03950628, -0.99921932]])
        # self.t = np.array([215.41618313, -13.73957331])
        '''后厂村路对向双毫米波外参'''
        self.R = np.array([[-0.99022542, 0.1394762], [-0.1394762, -0.99022542]])
        self.t = np.array([355.56001629, 35.75723811])
        # 跟踪
        # self.Sort_track = sort_for_all_env_update.Sort(max_age=4, min_hits=2)     #马源
        self.Sort_track = sort_radar_lin.Sort(max_age=4, min_hits=2)      #林潇
        self.fusion = Double_radar_fusion.Fusion(max_age=4)         # 加入id对匹配


    def run(self):
        while self.alive:
        # while self.cap.isOpened() and self.alive:
            print('thread run!')
            self.mutex.lock()
            runTime = 0

            if not self.running:
                self.cond.wait(self.mutex)      # 阻塞等待

            if self.currentIndex == self.runIndex:
                self.currentTime = round(self.currentTime + 0.1, 1)  # python 对保留小数点有坑，遇到问题在这改

            # 整除interval即运行一次
            if Decimal(str(self.currentTime)) % Decimal(str(self.interval)) == 0:
                startTime = timeit.default_timer()

                if self.currentIndex < self.runIndex:
                    self.currentIndex += 1
                    data, frame, time = self.cache[str(self.currentIndex)]

                else:
                    success, frame = self.cap.read()
                    # # 视频放完，线程自动结束
                    # if success == False:
                    #     print("video finished")
                    #     self.finished.emit()  # 线程结束信号
                    #     self.mutex.unlock()
                    #     self.cap.release()
                    #     self.running = False
                    #     self.alive = False
                    #     return

                    self.runIndex += 1
                    self.currentIndex += 1

                    # 离线数据显示完，线程自动结束
                    # len_record = max(len(self.record[0]), len(self.record[1]))
                    if (not self.record[0] and self.record[1]) or self.runIndex >= self.len_record:
                        print("radar finished")
                        self.finished.emit()  # 线程结束信号
                        self.mutex.unlock()
                        self.cap.release()
                        self.running = False
                        self.alive = False
                        return
                    print('runIndex', self.runIndex)
                    data1 = self.record[0][self.runIndex]
                    data2 = self.record[1][self.runIndex]
                    data = [data1, data2]
                    if len(self.cache) == 100:
                        del self.cache[str(self.runIndex - 100)]
                    self.cache[str(self.runIndex)] = (data , frame, self.currentTime)

                    time = self.currentTime


                    # if len(self.cache) == 100:
                    #     del self.cache[str(self.runIndex - 100)]
                    # self.cache[str(self.runIndex)] = (data, frame, self.currentTime)
                    #
                    # time = self.currentTime

                frame_ = deepcopy(frame)
                self.emitSignal(data, frame_, time)

                runTime = int((timeit.default_timer() - startTime) * 1000)     # 计算运行时间

            # print('cache', self.cache)
            '''
            减去计算运行时间，减小时间误差
            '''
            print("runtime", runTime)
            self.msleep(max(0, 100 - runTime))
            # self.msleep(100)

            self.mutex.unlock()

        self.cap.release()


    # 运行线程
    def goon(self):
        self.alive = True
        self.running = True
        self.cond.wakeAll()

    # 暂停线程
    def pause(self):
        self.running = False

    # 停止线程, 参数回调
    def stop(self):
        self.running = False
        self.currentTime = 0
        self.runIndex = -1
        self.currentIndex = -1
        self.cache = {}
        self.cap = cv2.VideoCapture(self.videoName)

    # 结束线程
    def kill(self):
        self.alive = False


    # 下一帧
    def nextFrame(self):
        self.currentIndex += 1
        if self.currentIndex > self.runIndex:
            self.runIndex += 1
            success, frame = self.cap.read()
            # 视频放完，回调参数
            # if success == False:
            #     print("video finished")
            #     self.stop()
            #     return

            # 离线数据显示完，回调参数
            # len_record = max(len(self.record[0]), len(self.record[1]))
            if not self.record or self.runIndex >= self.len_record:
                print("radar finished")
                self.stop()
                return

            # data = self.record[self.runIndex]
            data1 = self.record[0][self.runIndex]
            data2 = self.record[1][self.runIndex]
            data = [data1, data2]
            '''
            暂停瞬间时间是interval整数倍，则快进至下一interval
            暂停瞬间时间不是interval整数倍，快进至此interval
            '''
            if Decimal(str(self.currentTime)) % Decimal(str(self.interval)) == 0:
                self.currentTime = round(self.currentTime + self.interval, 1)
            else:
                while Decimal(str(self.currentTime)) % Decimal(str(self.interval)) != 0:
                    self.currentTime = round(self.currentTime + 0.1, 1)

            if len(self.cache) == 100:
                del self.cache[str(self.runIndex - 100)]
            self.cache[str(self.runIndex)] = (data1, frame, self.currentTime)

            time = self.currentTime



        else:
            data, frame, time = self.cache[str(self.currentIndex)]

        frame_ = deepcopy(frame)

        self.emitSignal(data, frame_, time)


    # 上一帧
    def preFrame(self):
        print('上一帧')
        if str(max(self.currentIndex - 1, 0)) not in self.cache:
            return
        print('self.currentIndex', self.currentIndex)
        self.currentIndex = max(self.currentIndex-1, 0)

        data, frame, time = self.cache[str(self.currentIndex)]

        frame_ = deepcopy(frame)

        self.emitSignal(data, frame_, time)


    def emitSignal(self, data, frame, time):
        '''雷达'''
        analysis = Analysis()
        vehiclesData1, staticData1, carsFlow1, unixTime1 = analysis.trans(data[0])  #157
        vehiclesData2, staticData2, carsFlow2, unixTime2 = analysis.trans(data[1])  #158
        print('======================================================================')
        # print(vehiclesData1)
        # print(vehiclesData2)
        '''解析出的vehiclesData1，vehiclesData2都是[{},{},...,{}],字典里面的value是字符型'''
        Flag = 2
        if Flag == 0:
            '''调时间对齐,空间标定'''
            self.drawSig.emit(None, None, None)
            self.updateRadar.emit(vehiclesData1, staticData1, carsFlow1, unixTime1, float(time), 0)  # 发送雷达更新信号
            self.updateRadar.emit(vehiclesData2, staticData2, carsFlow2, unixTime2, float(time), 1)  # 发送雷达更新信号
        elif Flag == 1:
            # 加了航向角和颜色选项
            for data in vehiclesData1:
                data['Flag'] = 2
            for data in vehiclesData2:
                data['Flag'] = 1
            '''空间标定后的'''
            point_sec = []  # 次radar.规定vehiclesData1为主雷达，vehiclesData2为次雷达
            for vehicle in vehiclesData2:
                point_sec.append([float(vehicle['X Point']), float(vehicle['Y Point'])])
            point_sec = np.array(point_sec)
            if vehiclesData2:
                points_pri = np.matmul(point_sec, self.R.T) + self.t.reshape([1, 2])
            vehiclesData_sec2pri = deepcopy(vehiclesData2)
            for i in range(len(vehiclesData_sec2pri)):  # 158转到了157坐标系
                vehiclesData_sec2pri[i]['X Point'], vehiclesData_sec2pri[i]['Y Point'] = str(points_pri[i][0]), str(
                    points_pri[i][1])
                vehiclesData1.append(vehiclesData_sec2pri[i])
            print('vehiclesData1', vehiclesData1)
            print('vehiclesData2', vehiclesData2)

            self.drawSig.emit(None, None, None)
            self.updateRadar.emit(vehiclesData1, staticData1, carsFlow1, unixTime1, float(time), 0)  # 发送雷达更新信号
            # self.updateRadar.emit(vehiclesData2, staticData2, carsFlow2, unixTime2, float(time), 1)  # 发送雷达更新信号
        elif Flag == 2:
            '''匹配后的结果：匹配，跟踪'''
            radar1, radar2 = [], []
            # 加了航向角和颜色选项
            for data in vehiclesData1:
                data['Heading'] = 0.0
                data['Flag'] = 2
                radar1.append(list(map(float, list(data.values()))))
            for data in vehiclesData2:
                data['Heading'] = 0.0
                data['Flag'] = 1
                radar2.append(list(map(float, list(data.values()))))

            point_sec = []
            for vehicle in vehiclesData2:
                point_sec.append([float(vehicle['X Point']), float(vehicle['Y Point'])])
            point_sec = np.array(point_sec)
            if vehiclesData2:
                points_pri = np.matmul(point_sec, self.R.T) + self.t.reshape([1, 2])
            vehiclesData_sec2pri = deepcopy(vehiclesData2)
            for i in range(len(vehiclesData_sec2pri)):  # 158转到了157坐标系
                vehiclesData_sec2pri[i]['X Point'], vehiclesData_sec2pri[i]['Y Point'] = str(points_pri[i][0]), str(
                    points_pri[i][1])
                vehiclesData1.append(vehiclesData_sec2pri[i])

            fusion_data = self.fusion.run(radar1, radar2)
            print('fusiondata', fusion_data)
            currentpoints = []
            if fusion_data:
                for radardata in fusion_data:
                    # 跟踪模块输入：x,y,l,w,vx,vy,heading,flag,id
                    currentpoints.append(
                        [radardata[2], radardata[3], radardata[1], 3.0, radardata[4], radardata[5], radardata[6],
                         radardata[7], radardata[0]])
            points = np.asarray(currentpoints).reshape((-1, 9))
            track_result = self.Sort_track.update(points)
            # print('track_result', track_result)

            # '''匹配后的结果'''
            # temp_data = []
            # if fusion_data:
            #     for radardata in fusion_data:
            #         temp_data.append({"Vehicle ID": str(radardata[0]), "Vehicle Length": str(radardata[1]),
            #                           "X Point": str(radardata[2]), "Y Point": str(radardata[3]),
            #                           "X Speed": str(radardata[4]), "Y Speed": str(radardata[5]),
            #                           "Heading": str(radardata[6]), "Flag": str(0)})
            # print('temp_data', temp_data)
            # emit_data = vehiclesData1 + temp_data

            track_data = []
            for track in track_result:
                track_data.append({"Vehicle ID": str(int(track[-1])), "Vehicle Length": str(track[2]),
                                   "X Point": str(track[0]), "Y Point": str(track[1]),
                                   "X Speed": str(track[4]), "Y Speed": str(track[5]),
                                   "radar ID": str(157), "Flag": str(0)})
            for vehicle in vehiclesData1:
                track_data.append(vehicle)


            self.drawSig.emit(None, None, None)
            # self.updateRadar.emit(vehiclesData1, staticData1, carsFlow1, unixTime1, float(time), 0)  # 发送雷达更新信号
            # self.updateRadar.emit(vehiclesData2, staticData2, carsFlow2, unixTime2, float(time), 1)  # 发送雷达更新信号
            self.updateRadar.emit(track_data, staticData1, carsFlow1, unixTime1, float(time), 0)



            # radar1, radar2 = [], []
            # for data in vehiclesData1:
            #     data['Flag'] = 2
            #     radar1.append(list(map(float, list(data.values()))))
            # for data in vehiclesData2:
            #     data['Flag'] = 2
            #     radar2.append(list(map(float, list(data.values()))))
            # # 以上将数据转为列表[[],[],...[]]
            # fusion = TwoRadarFusion.RadarFusion(radar1, radar2, self.R, self.t)
            # fusion_data = fusion.runFusion()
            # # print('fusion_data', fusion_data)
            # # fusion_data是匹配得到的目标,在此基础上跟踪,需要改变下数据格式
            # currentpoints = []
            # if fusion_data:
            #     for radar in fusion_data:
            #         # # x,y,l,w,航向角,点云个数,速度,0
            #         # currentpoints.append([radar[2], radar[3], radar[1], 3.0, 0.1, 1, radar[4], 0])
            #         # x,y,l,w,vx,vy,0,0
            #         currentpoints.append([radar[2], radar[3], radar[1], 3.0, radar[4], radar[5], 0, 0])
            # points = np.asarray(currentpoints).reshape((-1, 8))
            # # '''保存离线数据'''
            # # savePath = "D:/wanji/Linxiao/1030Radar_Radar/radar_radar_view/TrackData/doubleradar2/"
            # # if not os.path.exists(savePath):
            # #     os.makedirs(savePath)
            # #
            # # with open(savePath + str(self.runIndex + 100) + ".txt", 'w') as f:
            # #     print('here')
            # #     np.savetxt(f, points, fmt='%f', delimiter=' ')
            # # ===============================
            # print('points', points)
            # track_result = self.Sort_track.update(points)
            # print('track_result', track_result)
            #
            # '''匹配后的结果'''
            # emit_data = []
            # if fusion_data:
            #     for radardata in fusion_data:
            #         emit_data.append({"Vehicle ID": str(radardata[0]), "Vehicle Length": str(radardata[1]),
            #                           "X Point": str(radardata[2]), "Y Point": str(radardata[3]),
            #                           "X Speed": str(radardata[4]), "Y Speed": str(radardata[5]),
            #                           "radar ID": str(radardata[6]), "Flag": str(radardata[7])})
            # print('emit_data', emit_data)
            # '''跟踪后的结果'''
            # track_data = []
            # for track in track_result:
            #     track_data.append({"Vehicle ID": str(int(track[-1])), "Vehicle Length": str(track[2]),
            #                        "X Point": str(track[0]), "Y Point": str(track[1]),
            #                        "X Speed": str(track[6]), "Y Speed": str(0),
            #                        "radar ID": str(157), "Flag": str(2)})
            # print('track_data', track_data)
            # self.drawSig.emit(None, None, None)
            # # 匹配融合之后
            # # self.updateRadar.emit(emit_data, staticData1, carsFlow1, unixTime1, float(time), 0)  # 发送雷达更新信号
            # # self.updateRadar.emit(vehiclesData2, staticData2, carsFlow2, unixTime2, float(time), 1)  # 发送雷达更新信号
            #
            # # 跟踪之后
            # self.updateRadar.emit(track_data, staticData1, carsFlow1, unixTime1, float(time), 0)




        # radar1, radar2 = [], []
        # for data in vehiclesData1:
        #     data['Flag'] = 2
        #     radar1.append(list(map(float, list(data.values()))))
        # for data in vehiclesData2:
        #     data['Flag'] = 2
        #     radar2.append(list(map(float, list(data.values()))))
        # #以上将数据转为列表[[],[],...[]]
        # fusion = TwoRadarFusion.RadarFusion(radar1, radar2, self.R, self.t)
        # fusion_data = fusion.runFusion()
        # # print('fusion_data', fusion_data)
        # # fusion_data是匹配得到的目标,在此基础上跟踪,需要改变下数据格式
        # currentpoints = []
        # if fusion_data:
        #     for radar in fusion_data:
        #         # # x,y,l,w,航向角,点云个数,速度,0
        #         # currentpoints.append([radar[2], radar[3], radar[1], 3.0, 0.1, 1, radar[4], 0])
        #         # x,y,l,w,vx,vy,0,0
        #         currentpoints.append([radar[2], radar[3], radar[1], 3.0, radar[4], radar[5], 0, 0])
        # points = np.asarray(currentpoints).reshape((-1, 8))
        # # '''保存离线数据'''
        # # savePath = "D:/wanji/Linxiao/1030Radar_Radar/radar_radar_view/TrackData/doubleradar2/"
        # # if not os.path.exists(savePath):
        # #     os.makedirs(savePath)
        # #
        # # with open(savePath + str(self.runIndex + 100) + ".txt", 'w') as f:
        # #     print('here')
        # #     np.savetxt(f, points, fmt='%f', delimiter=' ')
        # # ===============================
        # print('points', points)
        # track_result = self.Sort_track.update(points)
        # print('track_result',track_result)
        #
        #
        # '''匹配后的结果'''
        # emit_data = []
        # if fusion_data:
        #     for radardata in fusion_data:
        #         emit_data.append({"Vehicle ID": str(radardata[0]), "Vehicle Length": str(radardata[1]),
        #                               "X Point": str(radardata[2]), "Y Point": str(radardata[3]),
        #                               "X Speed": str(radardata[4]), "Y Speed": str(radardata[5]),
        #                               "radar ID": str(radardata[6]), "Flag": str(radardata[7])})
        # print('emit_data', emit_data)
        # '''跟踪后的结果'''
        # track_data = []
        # for track in track_result:
        #     track_data.append({"Vehicle ID": str(int(track[-1])), "Vehicle Length": str(track[2]),
        #                               "X Point": str(track[0]), "Y Point": str(track[1]),
        #                               "X Speed": str(track[6]), "Y Speed": str(0),
        #                               "radar ID": str(157), "Flag": str(2)})
        # print('track_data', track_data)
        #
        #
        # '''调时间对齐,标定'''
        # # points_158 = []
        # # for vehicle in vehiclesData2:
        # #     points_158.append([float(vehicle['X Point']), float(vehicle['Y Point'])])
        # # points_158 = np.array(points_158)
        # # # print(type(points_158), points_158)
        # # # print(bool(points_158))
        # # # if points_158:
        # # if vehiclesData2:
        # #     points_1582157 = np.matmul(points_158, self.R.T) + self.t.reshape([1, 2])
        # # vehiclesData_158to157 = deepcopy(vehiclesData2)
        # # for i in range(len(vehiclesData_158to157)):    #158转到了157坐标系
        # #     vehiclesData_158to157[i]['X Point'], vehiclesData_158to157[i]['Y Point'] = str(points_1582157[i][0]), str(points_1582157[i][1])
        # #     vehiclesData1.append(vehiclesData_158to157[i])
        # # # for i in range(len(vehiclesData2)):    #158转到了157坐标系
        # # #     vehiclesData2[i]['X Point'], vehiclesData2[i]['Y Point'] = str(points_1582157[i][0]), str(points_1582157[i][1])
        # # #     vehiclesData1.append(vehiclesData2[i])
        # # print('vehiclesData1', vehiclesData1)
        # # print('vehiclesData2', vehiclesData2)
        #
        # '''
        # 相机
        # '''
        # oriFrame = deepcopy(frame)
        # '''
        # 发送信号
        # '''
        # self.drawSig.emit(None, None, None)
        #
        # # 标定
        # # self.updateRadar.emit(vehiclesData1, staticData1, carsFlow1, unixTime1, float(time), 0)  # 发送雷达更新信号
        # # self.updateRadar.emit(vehiclesData2, staticData2, carsFlow2, unixTime2, float(time), 1)  # 发送雷达更新信号
        #
        #
        # # 融合之后
        # self.updateRadar.emit(emit_data, staticData1, carsFlow1, unixTime1, float(time), 0)  # 发送雷达更新信号
        # self.updateRadar.emit(vehiclesData2, staticData2, carsFlow2, unixTime2, float(time), 1)  # 发送雷达更新信号
        #
        # # 跟踪之后
        # # self.updateRadar.emit(track_data, staticData1, carsFlow1, unixTime1, float(time), 0)



