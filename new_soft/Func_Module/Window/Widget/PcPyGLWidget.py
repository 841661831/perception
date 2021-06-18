# -*-coding:utf-8-*
from Widget.PyGLWidget import PyGLWidget
from WinSubPub.WinSubPub import proSubData
from WinCommon import *

BACKGROUND_BLACK = True


class PcPyGLWidget(PyGLWidget):
    sigPlayOver = pyqtSignal(bool)
    sigSelectBox = pyqtSignal(int)
    sigDrawRoiOver = pyqtSignal()

    def __init__(self, Mylog, parent=None):
        PyGLWidget.__init__(self, parent)
        self.parent = parent
        self.Mylog = Mylog
        self.dataBase = DataBase(strHost)
        self.stSysState = None
        self.stStationParam = None
        self.stDstData = None

        self.modelview = np.zeros((4, 4))
        self.projection = np.zeros((4, 4))
        self.viewport = [0, 0, self.fWidth, self.fHeight]
        self.listRect = []
        self.listBoxInfo = []
        self.nSelectBoxId = -1
        self.listSelectBoxId = []
        self.dictTargetHisFrame = {}
        self.selectBoxInfo = None

        self.rectInfo = None
        self.painter = QPainter()

        self.proSubPub = None

        self.bShowBox = True
        self.bShowLane = True

        self.GridWidth = 200
        self.GridHeight = 200
        self.GridStep = 5
        self.resize(450, 260)  # 重置一下大小，防止显示异常

        self.stUiParam = stUiParam

    def __del__(self):
        if self.proSubPub is not None and self.proSubPub.is_alive():
            self.proSubPub.terminate()

    def setSysParam(self, stSysState=None, stStationParam=None):
        # 从结构体bufParam中解析出相关的变量
        # self.stop()
        if stSysState is not None:
            self.stSysState = stSysState
        if stStationParam is not None:
            self.stStationParam = stStationParam
        self.stSysState.set_pause(True)

        while self.dataEventQueue.qsize() > 0:
            try:
                toBeDel = self.srcEventDataQueue.get_nowait()
            except:
                continue

        while self.dataTrackQueue.qsize() > 0:
            try:
                toBeDel = self.dataTrackQueue.get_nowait()
            except:
                continue

        self.stSysState.set_state(0)
        self.stSysState.set_noData(False)
        self.stSysState.set_frameId(0)
        self.stSysState.set_pause(False)

        self.start()
        return

    def setSelectBoxId(self, nSelectBoxId):
        self.nSelectBoxId = nSelectBoxId
        self.listSelectBoxId.append(nSelectBoxId)
        self.updateGL()

    def setAutoLoopPlay(self, bLoop):
        self.stSysState.set_loop(bLoop)

    def setShowBox(self, bShowBox):
        self.bShowBox = bShowBox
        self.update()

    def setShowLane(self, bShowLane):
        self.bShowLane = bShowLane
        self.update()

    def setManualReplay(self, bReplay):
        self.stSysState.set_replay(bReplay)

    def isPlayOver(self):
        bOver = True
        return bOver

    def setSyncParam(self, dataTrackQueue, dataEventQueue):
        self.dataTrackQueue = dataTrackQueue
        self.dataEventQueue = dataEventQueue

    def slotTimerOut(self, stDstData):
        try:
            self.stDstData = stDstData
            if stDstData is None:
                self.MyLog.error("Data processing error!...")
                return 0, 0  # 表示数据有误

            self.listBoxInfo = copy.deepcopy(self.stDstData.listBoxInfo)
            for boxInfo in self.listBoxInfo:
                if boxInfo.boxId not in self.dictTargetHisFrame:
                    self.dictTargetHisFrame[boxInfo.boxId] = []
                self.dictTargetHisFrame[boxInfo.boxId].append(self.stDstData.nFrameId)


            self.updateGL()
        except:
            print(traceback.format_exc())
        return self.stDstData.nFrameId

    def openFile(self, strFileName):
        # 判断选择的文件列表是否为空,即使不选择,FileList的len依然是2,如果选择了文件,则FileList中的[0]元素长度必然>0
        if len(strFileName) < 1:
            return

        self.stSysState.set_pause(True)
        self.stSysState.set_offline_file(strFileName)

        while self.dataEventQueue.qsize() > 0:
            try:
                toBeDel = self.srcEventDataQueue.get_nowait()
            except:
                continue

        while self.dataTrackQueue.qsize() > 0:
            try:
                toBeDel = self.dataTrackQueue.get_nowait()
            except:
                continue

        self.stSysState.set_state(0)
        self.stSysState.set_frameId(0)
        self.stSysState.set_online(False)
        self.stSysState.set_pause(False)

        # self.stop()
        self.start()

    def stopPlay(self):
        self.stop()
        self.stSysState.set_pause(True)

        while self.dataEventQueue.qsize() > 0:
            try:
                toBeDel = self.srcEventDataQueue.get_nowait()
            except:
                continue

        while self.dataTrackQueue.qsize() > 0:
            try:
                toBeDel = self.dataTrackQueue.get_nowait()
            except:
                continue
        self.listRect = []
        self.listBoxInfo = []
        self.nSelectBoxId = -1
        self.listSelectBoxId = []
        self.selectBoxInfo = None

    def startPlay(self):
        self.stSysState.set_state(0)
        self.stSysState.set_frameId(0)
        self.stSysState.set_pause(False)

    def stop(self):
        self.stSysState.set_running(False)
        self.stSysState.set_pause(False)

        while self.proSubPub is not None and self.proSubPub.is_alive():
            self.proSubPub.terminate()

        # 停止显示
        self.stSysState.set_pause(True)

        # 清空数据
        self.stSysState.set_frameId(0)

    def start(self):
        if self.proSubPub is None or (not self.proSubPub.is_alive()):
            self.stSysState.set_running(True)
            self.stSysState.set_pause(False)
            self.stSysState.set_state(0)
            # 判断选择的文件列表是否为空,即使不选择,FileList的len依然是2,如果选择了文件,则FileList中的[0]元素长度必然>0
            listOfflineFiles = self.stSysState.get_offline_file()
            if (not self.stSysState.get_online()) and (len(listOfflineFiles) < 1 or len(listOfflineFiles[0]) < 1):
                return

            self.proSubPub = Process(target=proSubData,
                                     args=(self.dataTrackQueue, self.dataEventQueue,
                                           self.stSysState, self.parent.list_subChannel,
                                           self.Mylog))
            self.proSubPub.start()

    def pause(self):
        self.stSysState.set_pause(True)

    def play(self):
        if 1 <= self.stSysState.get_state() <= 2:
            self.stSysState.set_state(3)
        if self.isPlayOver():
            self.stSysState.set_state(0)

        self.stSysState.set_pause(False)
        self.start()

    def restart(self):
        # stop
        self.stop()

        # start
        self.start()

    def previous(self):
        self.stSysState.set_frameId(max(0, self.stSysState.get_frameId() - 1))
        self.stSysState.set_state(1)
        self.stSysState.set_pause(False)
        self.MyLog.info("PcPyGLWidget need:{}".format(self.stSysState.get_frameId()))

    def next(self):
        self.stSysState.set_frameId(self.stSysState.get_frameId() + 1)
        self.stSysState.set_state(1)
        self.stSysState.set_pause(False)
        self.MyLog.info("PcPyGLWidget need:{}".format(self.stSysState.get_frameId()))

    def skipTo(self, nFrameId):
        self.stSysState.set_frameId(max(0, nFrameId))
        self.stSysState.set_state(1)
        self.stSysState.set_pause(False)
        self.MyLog.info("PcPyGLWidget skipTo:{}".format(self.stSysState.get_frameId()))

    def onLine(self):
        self.stSysStateset_online(True)
        self.start()
        # self.restart()

    def offLine(self):
        self.stSysStateset_online(False)
        # self.restart()

    def drawAxis(self):
        # Local Space
        s_Scale = 10
        font = QtGui.QFont()
        font.setPointSize(12)

        glPushAttrib(GL_LINE_BIT)
        glLineWidth(1.0)
        glBegin(GL_LINES)

        # X轴
        glColor3ub(255, 0, 0)
        glVertex3i(0, 0, 0)
        glVertex3i(s_Scale, 0, 0)

        # Y轴
        glColor3ub(0, 255, 0)
        glVertex3i(0, 0, 0)
        glVertex3i(0, s_Scale, 0)

        # Z轴
        glColor3ub(0, 0, 255)
        glVertex3i(0, 0, 0)
        glVertex3i(0, 0, s_Scale)

        glEnd()
        glPopAttrib()
        glColor3ub(255, 0, 0)
        self.renderText(s_Scale, 0, 0, 'x', font)
        glColor3ub(0, 255, 0)
        self.renderText(0, s_Scale, 0, 'y', font)
        glColor3ub(0, 0, 255)
        self.renderText(0, 0, s_Scale, 'z', font)
        glColor3ub(255, 255, 255)

    def drawGrid(self, nWidth, nHeight, GridStep):
        glColor3f(0.5, 0.5, 0.5)
        glBegin(GL_LINES)
        z = -5
        # 画竖线
        step = 1
        for i in range(-int(nWidth), int(nWidth + 1), GridStep):
            glVertex3f(i, -nWidth * step, z)
            glVertex3f(i, nWidth * step, z)
        # # 划横线
        for j in range(-int(nHeight), int(nHeight + 1), GridStep):
            glVertex3f(-nHeight * step, j, z)
            glVertex3f(nHeight * step, j, z)
        glEnd()

    def GridRefresh(self, x, y, step):
        self.GridWidth = x
        self.GridHeight = y
        self.GridStep = step
        self.updateGL()

    def drawSelectCube(self):
        t1 = time.time()
        # 将要使用的顶点的序号保存到一个数组里面
        # 目前八个顶点是按照如下顺序排列的：上顶面：1,2,6,5（逆时针排序），下底面：0,3,7,4（逆时针排序）
        index_list = np.array([[1, 2], [2, 6], [6, 5], [5, 1],
                               [0, 3], [3, 7], [7, 4], [4, 0],
                               [0, 1], [3, 2], [7, 6], [4, 5]])
        try:
            nCol = 0
            for i in range(2):
                if len(self.listSelectCubeCenter[i]) == 0:
                    continue
                nRow = 0
                for listboxCeter in self.listSelectCubeCenter[i]:
                    if len(listboxCeter) == 0:
                        continue
                    glPointSize(5)
                    if i == 1:
                        glColor3f(0.0, 1.0, 1.0)
                    else:
                        glColor3f(1.0, 0.0, 0.0)
                    if self.nSelectedTableRow == nRow and self.nSelectedTableCol == nCol:
                        glColor3f(0.0, 1.0, 0.0)
                    glBegin(GL_POINTS)
                    glVertex3f(listboxCeter[0], listboxCeter[1], listboxCeter[2])
                    glEnd()
                    glPointSize(1)
                    nRow += 1
                nCol += 1
                for listboxVertex in self.listSelectVertexPc[i]:
                    if listboxVertex[1].shape[0] == 0:
                        continue
                    glLineWidth(3)
                    glBegin(GL_LINES)
                    for j in range(12):  # 12条线段
                        for k in range(2):  # 每条线段 2个顶点
                            # n行, 每一行是8个点, 每个点是(x,y,z)三坐标值, 所以box_data[i][j][k]中i对应“行”, j对应8个顶点中的“点序号”, k对应x，y，z
                            if i == 1:
                                glColor3f(0.0, 1.0, 1.0)
                            else:
                                glColor3f(1.0, 0.0, 0.0)
                            glVertex3fv(listboxVertex[1][index_list[j][k]])
                    glEnd()
                    glColor3f(1.0, 1.0, 1.0)
                    glLineWidth(1)
        except:
            print(traceback.format_exc())
            self.MyLog.error("Except!! drawnewCube ERROR at Frame-{}!".format(self.stSysState.get_frameId()))

    def drawBaseStation(self, nWide):
        stStationDev = self.stStationParam.getStationDev(0)
        if stStationDev is None:
            return
        stTransform = Lon_Lat_Meter_Transform(stStationDev.fLongitude, stStationDev.fLatitude,
                                              stStationDev.fAngleNorth)
        for i in range(self.stStationParam.getStationCount()):
            stStationDev = self.stStationParam.getStationDev(i)
            x, y = stTransform.lonlat_to_xy_of_draw(
                np.array([stStationDev.fLongitude, stStationDev.fLatitude]).reshape((1, 2))).tolist()[0]
            z = 0
            font = QtGui.QFont()
            font.setPointSize(15)
            self.renderText(x, y, z, '%d' % (i + 1), font)

    def getTraceList(self, boxId):
        listTrack = []
        date_time = datetime.datetime.now().strftime("%Y%m%d")
        listFrame = self.dictTargetHisFrame[boxId]
        nFrameCount = 0
        for nframeId in listFrame:
            byteBoxInfo = self.dataBase.read(date_time, 'E1', nframeId, targetId=boxId)
            npBoxCenter = np.frombuffer(byteBoxInfo, dtype=np.float64).reshape((-1,14))[:, 0:3]
            listTrack.append(npBoxCenter)
            nFrameCount += 1
            if nFrameCount > 60:
                break


    def drawCube(self):
        t1 = time.time()
        # 将要使用的顶点的序号保存到一个数组里面
        # 目前八个顶点是按照如下顺序排列的：上顶面：1,2,6,5（逆时针排序），下底面：0,3,7,4（逆时针排序）
        index_list = np.array([[1, 2], [2, 6], [6, 5], [5, 1],
                               [0, 3], [3, 7], [7, 4], [4, 0],
                               [0, 1], [3, 2], [7, 6], [4, 5]])

        # glColor3f(1.0, 0.0, 0.0)
        nCount = len(self.listBoxInfo)
        if nCount > 0:
            self.listRect = []
            # print("Box count: %d" % nCount)
            try:
                for i in range(nCount):
                    # if self.listBoxInfo[i].strClass in self.stDstData.filterList:
                    boxInfo = self.listBoxInfo[i]
                    boxVertexPc = boxInfo.boxVertexPc
                    if len(boxVertexPc) == 0:
                        continue

                    maxX = 1e-10
                    minX = 1e10
                    maxY = 1e-10
                    minY = 1e10
                    for n in range(8):
                        p = boxVertexPc[n]
                        # winPoint = np.zeros(2)
                        x, y, z = gluProject(p[0], p[1], p[2], self.modelview, self.projection, self.viewport)
                        y = self.fHeight - y
                        maxX = max(maxX, x)
                        minX = min(minX, x)
                        maxY = max(maxY, y)
                        minY = min(minY, y)

                    rect = QRectF(QPointF(minX, minY), QPointF(maxX, maxY))
                    self.listRect.append([rect, i])
                    if self.listBoxInfo[i].boxId in self.listSelectBoxId:
                        listTrack = self.getTraceList(self.listBoxInfo[i].boxId)
                        glLineWidth(1)
                        glBegin(GL_LINES)
                        glColor3f(0, 1, 0)
                        for i in range(len(listTrack) - 1):
                            glVertex3fv(np.array([listTrack[i]]))
                            glVertex3fv(np.array([listTrack[i + 1]]))
                        glEnd()
                        glLineWidth(5)
                    else:
                        glLineWidth(1)

                    glBegin(GL_LINES)
                    for j in range(12):  # 12条线段
                        for k in range(2):  # 每条线段 2个顶点
                            # n行, 每一行是8个点, 每个点是(x,y,z)三坐标值, 所以box_data[i][j][k]中i对应“行”, j对应8个顶点中的“点序号”, k对应x，y，z
                            if j in [3, 7, 8, 11]:
                                glColor3f(1.0, 1.0, 0.0)
                            else:
                                glColor3f(1.0, 1.0, 1.0)
                                if boxInfo.strSource == "Complement":
                                    glColor3f(0, 1, 1.0)
                                elif boxInfo.strSource == "PcRfusion":
                                    glColor3f(1.0, 0.0, 0.0)
                                elif boxInfo.strSource == "Radar":
                                    glColor3f(1, 0.0, 0.0)
                                elif boxInfo.strSource == "RRfusion":
                                    glColor3f(0.0, 1.0, 0.0)
                                if self.npDeadId.__contains__(boxInfo.boxId):
                                    glColor3f(0, 0, 1.0)
                                elif self.npOccluId.__contains__(boxInfo.boxId):
                                    glColor3f(0, 0.55, 0.55)
                            glVertex3fv(boxVertexPc[index_list[j][k]])

                    glColor3f(1.0, 1.0, 1.0)
                    glEnd()
            except:
                self.MyLog.error("Except!! drawCube ERROR at Frame-{}!".format(self.stSysState.get_frameId()))

    def drawCubeInfo(self):
        bIsExistSelectId = False
        nCount = len(self.listBoxInfo)
        if nCount > 0:
            boxInfoList = []
            try:
                for i in range(nCount):
                    boxInfo = self.listBoxInfo[i]
                    # if boxInfo.strClass in self.stDstData.filterList:
                    boxVertexPc = boxInfo.boxVertexPc
                    if len(boxVertexPc) == 0:
                        continue

                    if boxInfo.boxId == self.nSelectBoxId and self.stDstData is not None and i < len(self.listBoxInfo):
                        # 计算选中目标绘制信息的矩形框, 在二维绘制
                        pBottomLeft = boxVertexPc[0]
                        x, y, z = gluProject(pBottomLeft[0], pBottomLeft[1], pBottomLeft[2], self.modelview,
                                             self.projection, self.viewport)
                        y = self.fHeight - y
                        self.rectInfo = QRectF(QPointF(x, y - 86), QPointF(x + 70, y))
                        self.selectBoxInfo = self.listBoxInfo[i]
                        bIsExistSelectId = True
                    else:
                        # n行, 每一行是8个点, 每个点是(x,y,z)三坐标值, 所以box_data[i][j][k]中i对应“行”, j对应8个顶点中的“点序号”, k对应x，y，z
                        fx = boxVertexPc[1][0]
                        fy = boxVertexPc[1][1]
                        fz = boxVertexPc[1][2]

                        font = QtGui.QFont()
                        font.setPointSize(12)
                        self.renderText(fx, fy, fz, '{}-{}-{}-{}'.format(boxInfo.boxId,
                                                                         int(boxInfo.nBaseStationId),
                                                                         int(boxInfo.nPreFrameCnt),
                                                                         int(boxInfo.nBaseStationBoxId)), font)
                        # int(self.stDstData.nFrameId)), font)
            except:
                self.MyLog.error("Except!! drawCubeInfo ERROR at Frame-{}!".format(self.stSysState.get_frameId()))

        if not bIsExistSelectId:
            self.rectInfo = None

    def paintGL(self):
        # gltime = QTime()
        # gltime.start()
        try:
            maxHeight = None
            if self.selectBoxInfo is not None:
                if self.selectBoxInfo.strSource == "PcRfusion":
                    strId = "PcRF_ID:{}".format(self.nSelectBoxId)
                else:
                    strId = "ID:{}".format(self.nSelectBoxId)
                strLen = "Length:{}".format(self.selectBoxInfo.boxSize[1])
                strWidth = "Width:{}".format(self.selectBoxInfo.boxSize[0])
                strHeight = "Height:{}".format(self.selectBoxInfo.boxSize[2])
                if strLen != 'None' and strLen != 'N':
                    strLen = "Length:{}".format(round(self.selectBoxInfo.boxSize[1], 3))
                    strWidth = "Width:{}".format(round(self.selectBoxInfo.boxSize[0], 3))
                    strHeight = "Height:{}".format(round(self.selectBoxInfo.boxSize[2], 3))

                strSpeed = "Speed:{}".format(self.selectBoxInfo.boxSpeed)
                if strSpeed != 'None':
                    strSpeed = "Speed:{}".format(round(self.selectBoxInfo.boxSpeed, 3))

                strAngle = "Angle:{}".format(self.selectBoxInfo.boxAngle)
                if strAngle != 'None':
                    fAngle = self.selectBoxInfo.boxAngle / math.pi * 180
                    if fAngle > 360:
                        fAngle = fAngle - 360
                    strAngle = "Angle:{}".format(round(fAngle, 3))

                strClass = "Class:{}".format(self.selectBoxInfo.strClass)

                fm = QFontMetrics(QFont('WenQuanYi Micro Hei'))
                maxWidth = max(fm.width(strLen), fm.width(strWidth))
                maxWidth = max(maxWidth, fm.width(strHeight))
                maxWidth = max(maxWidth, fm.width(strSpeed))
                maxWidth = max(maxWidth, fm.width(strAngle))
                maxWidth = max(maxWidth, fm.width(strClass))
                maxHeight = fm.height()
        except:
            self.MyLog.info("Except!! paintGL ERROR at Frame-{}!".format(self.stSysState.get_frameId()))
            PyGLWidget.paintGL(self507)
            return

        PyGLWidget.paintGL(self)
        if not self.stSysState.get_showBox():
            return

        glPushMatrix()  # 把当前矩阵入栈保存
        self.modelview = np.zeros((4, 4))
        self.projection = np.zeros((4, 4))
        self.viewport = [0, 0, self.fWidth, self.fHeight]

        # glGetIntegerv(GL_VIEWPORT, viewport)
        glGetDoublev(GL_MODELVIEW_MATRIX, self.modelview)
        glGetDoublev(GL_PROJECTION_MATRIX, self.projection)

        self.drawAxis()

        if self.bShowBox and self.stDstData is not None:
            self.drawCube()  # 绘制目标框
            self.drawCubeInfo()
        time1 = QTime()
        time1.start()
        nFrameId = self.stSysState.get_frameId()
        if self.isPlayOver() or self.stSysState.get_pause():
            nFrameId = max(nFrameId - 1, 0)
        if self.stDstData is not None:
            self.renderText(0, 0, 0, 'Frame: %d' % self.stDstData.nFrameId)
        elif self.stSysState.get_running():
            self.renderText(0, 0, 0, 'Frame: %d' % nFrameId)
        else:
            self.renderText(0, 0, 0, 'Frame: %d' % 0)
        # if self.bShowLane and self.stSysState.get_running():
        if self.bShowLane:
            self.drawBaseStation(2)
            glLineWidth(1)
            glPointSize(1)
            glEnableClientState(GL_VERTEX_ARRAY)
            glEnableClientState(GL_COLOR_ARRAY)
            for VBO in self.parent.listVBO:
                vtx_disp, clr_disp, cnt_disp = VBO
                vtx_disp.bind()
                glVertexPointer(3, GL_FLOAT, 0, vtx_disp)
                vtx_disp.unbind()
                clr_disp.bind()
                glColorPointer(3, GL_FLOAT, 0, clr_disp)
                clr_disp.unbind()
                glDrawArrays(GL_LINES, 0, cnt_disp)

            glDisableClientState(GL_VERTEX_ARRAY)
            glDisableClientState(GL_COLOR_ARRAY)
        glPopMatrix()  # 恢复之前保存的矩阵

        # 注意：self.painter在创建时，一定不能传入self,否则会与paintGl冲突，导致崩溃。可在paintGL接口内部调用painter的begin（self）接口，指定绘制设备。
        if self.rectInfo is not None:
            pen = QPen(QColor("yellow"))
            pen.setWidth(2)
            self.painter.begin(self)
            self.painter.setPen(pen)

            if self.bShowBox and self.rectInfo is not None and maxHeight is not None:
                self.rectInfo = QRectF(self.rectInfo.x(), self.rectInfo.y() + self.rectInfo.height() - maxHeight * 7,
                                       maxWidth, maxHeight * 7)
                brush = QBrush(QColor(0, 191, 255, 200))
                self.painter.setBrush(brush)
                self.painter.drawRect(self.rectInfo)

                pen.setColor(QColor("red"))
                self.painter.setFont(QFont('WenQuanYi Micro Hei'))
                self.painter.setPen(pen)
                pInfoRectTopLeft = self.rectInfo.topLeft()
                self.painter.drawText(QPoint(pInfoRectTopLeft.x() + 2, pInfoRectTopLeft.y() + maxHeight), strId)
                self.painter.drawText(QPoint(pInfoRectTopLeft.x() + 2, pInfoRectTopLeft.y() + maxHeight * 2), strLen)
                self.painter.drawText(QPoint(pInfoRectTopLeft.x() + 2, pInfoRectTopLeft.y() + maxHeight * 3), strWidth)
                self.painter.drawText(QPoint(pInfoRectTopLeft.x() + 2, pInfoRectTopLeft.y() + maxHeight * 4), strHeight)
                self.painter.drawText(QPoint(pInfoRectTopLeft.x() + 2, pInfoRectTopLeft.y() + maxHeight * 5), strSpeed)
                self.painter.drawText(QPoint(pInfoRectTopLeft.x() + 2, pInfoRectTopLeft.y() + maxHeight * 6), strAngle)
                self.painter.drawText(QPoint(pInfoRectTopLeft.x() + 2, pInfoRectTopLeft.y() + maxHeight * 7), strClass)
            self.painter.end()

    def initializeGL(self):
        # OpenGL state
        if BACKGROUND_BLACK:
            glClearColor(0.0, 0.0, 0.0, 0.0)
        else:
            glClearColor(1.0, 1.0, 1.0, 1.0)

        glEnable(GL_DEPTH_TEST)
        self.resetView()

    def mousePressEvent(self, _event):
        self.bIsLeftButton = False
        if _event.buttons() & QtCore.Qt.LeftButton:
            self.bIsLeftButton = True
            # print("self.firstPoint = ", self.firstPoint)
            # 处理选中目标框
            if self.stDstData is not None:
                curPoint = _event.pos()
                for i in range(len(self.listRect)):
                    rect = self.listRect[i][0]
                    if i >= len(self.listBoxInfo):
                        break
                    if self.listRect[i][0].contains(curPoint):
                        self.nSelectBoxId = int(self.listBoxInfo[self.listRect[i][1]].boxId)
                        if self.nSelectBoxId not in self.listSelectBoxId:
                            self.listSelectBoxId.append(self.nSelectBoxId)
                        self.selectBoxInfo = self.listBoxInfo[self.listRect[i][1]]
                        self.sigSelectBox.emit(self.nSelectBoxId)
                        break

        elif _event.buttons() & QtCore.Qt.RightButton:
            self.nSelectBoxId = -1
            self.listSelectBoxId = []
            self.sigSelectBox.emit(self.nSelectBoxId)

        self.updateGL()
        PyGLWidget.mousePressEvent(self, _event)

    def mouseMoveEvent(self, _event):
        PyGLWidget.mouseMoveEvent(self, _event)

    def mouseReleaseEvent(self, _event):
        PyGLWidget.mouseReleaseEvent(self, _event)

# ================================== PcPyGLWidget end =============================================
