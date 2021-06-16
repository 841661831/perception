from PyQt5.QtWidgets import *
import pyqtgraph as pg
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5 import QtCore, QtWidgets, QtGui
import numpy as np
import math
from pic import *
import timeit
import logging


class RadarRoadWidget(QFrame):
    selectCar = pyqtSignal(str)     # 鼠标选定车信号
    curCarId = '-1'  # 当前选中车ID


    def __init__(self, colors):
        super(RadarRoadWidget, self).__init__()
        self.setStyleSheet("PlotWidget{border: 0px solid #148CD2;}")    # plotwidget无边框
        pg.setConfigOption('background', (0,0,0,0))     # 背景透明
        # pg.setConfigOption('foreground', (0,0,0,255))     # 坐标系颜色
        colorList = QColor.colorNames()
        print("colorList",colorList)
        self.roadColor, self.zoneColor, self.carColor, self.idColor = colorList[colors[0]], colorList[colors[1]], colorList[colors[2]], colorList[colors[3]]
        self.initUI()



    def initUI(self):
        self.resize(800,620)
        self.setWindowTitle("RadarRoadWidget")
        self.gridLayout = QGridLayout(self)
        self.setLayout(self.gridLayout)

        self.drawPlt = pg.PlotWidget()
        self.drawPlt.setXRange(-50, 50)
        self.drawPlt.hideAxis('bottom')
        self.drawPlt.hideAxis('left')
        self.cars = pg.ScatterPlotItem(pen=pg.mkPen(None), pxMode=False, useCache=False)    # 车框点集
        self.texts = pg.ScatterPlotItem()   # 文字部分点集，不能用pxMode=True否则会显示出错
        self.drawPlt.addItem(self.cars)
        self.drawPlt.addItem(self.texts)
        self.carsItem = {}

        self.carNumPlt = pg.PlotWidget(title = "车流量")
        self.carNumPlt.showGrid(x=True, y=True)
        self.carNumPlt.setYRange(0, 30)
        self.carNumPlt.setXRange(0, 10)
        self.curve = self.carNumPlt.plot()
        # self.carNumPlt.setLabel('bottom', 'Time', 's')
        self.carNums = []  # 车流量
        self.xIndex = []    # 时间下标
        self.carIndex = 0     # 时间


        self.carSpeedPlt = pg.PlotWidget(title = "车速")
        self.carSpeedPlt.showGrid(x=True, y=True)
        self.carSpeedPlt.setYRange(0, 20)
        self.carSpeedPlt.setXRange(0, 10)
        self.curve2 = self.carSpeedPlt.plot()
        self.carSpeed = []      # 车速数据
        self.xIndex2 = []       # 时间下标
        self.carIndex2 = 0      # 时间
        self.drawingCarId = '-1'    # 正在绘图车ID
        self.carSpeedPlt.setLabel('bottom', 'ID', self.drawingCarId)

        # 调整角度部分
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setSpacing(0)
        spacerItem1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        pixmap = QtGui.QPixmap(":/pic/left.png")
        fitPixmap = pixmap.scaled(28, 28, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        self.leftIcon = QtWidgets.QToolButton()
        self.leftIcon.setIcon(QIcon(fitPixmap))
        self.leftIcon.setIconSize(QSize(28, 28))
        pixmap = QtGui.QPixmap(":/pic/right.png")
        fitPixmap = pixmap.scaled(28, 28, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        self.rightIcon = QtWidgets.QToolButton()
        self.rightIcon.setIcon(QIcon(fitPixmap))
        self.rightIcon.setIconSize(QSize(28, 28))
        self.angleText = QLineEdit()
        pixmap = QtGui.QPixmap(":/pic/minus.png")
        fitPixmap = pixmap.scaled(28, 28, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        self.minusIcon = QtWidgets.QToolButton()
        self.minusIcon.setIcon(QIcon(fitPixmap))
        self.minusIcon.setIconSize(QSize(28, 28))
        pixmap = QtGui.QPixmap(":/pic/add.png")
        fitPixmap = pixmap.scaled(28, 28, QtCore.Qt.IgnoreAspectRatio,
                                  QtCore.Qt.SmoothTransformation)  # 注意 scaled() 返回一个 QtGui.QPixmap
        self.addIcon = QtWidgets.QToolButton()
        self.addIcon.setIcon(QIcon(fitPixmap))
        self.addIcon.setIconSize(QSize(28, 28))
        spacerItem2 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)

        self.angleText.setMinimumSize(QSize(50, 28))

        self.horizontalLayout.addItem(spacerItem1)
        self.horizontalLayout.addWidget(self.leftIcon)
        self.horizontalLayout.addWidget(self.minusIcon)
        self.horizontalLayout.addWidget(self.angleText)
        self.horizontalLayout.addWidget(self.addIcon)
        self.horizontalLayout.addWidget(self.rightIcon)
        self.horizontalLayout.addItem(spacerItem2)          # 一定要用两个spaceritem对象，否则删除widget会出错

        self.angle = 0
        self.angleText.returnPressed.connect(self.angleChange)
        self.minusIcon.clicked.connect(self.minusAngle)
        self.addIcon.clicked.connect(self.addAngle)
        self.leftIcon.clicked.connect(self.leftAngle)
        self.rightIcon.clicked.connect(self.rightAngle)


        self.gridLayout.addWidget(self.carNumPlt, 0, 0, 1, 1)
        self.gridLayout.addWidget(self.carSpeedPlt, 1, 0, 1, 1)
        self.gridLayout.addLayout(self.horizontalLayout, 2, 0, 1, 1)
        self.gridLayout.addWidget(self.drawPlt, 0, 1, 3, 1)
        self.gridLayout.setColumnStretch(0, 1)
        self.gridLayout.setColumnStretch(1, 2)
        self.gridLayout.setRowStretch(0, 1)
        self.gridLayout.setRowStretch(1, 1)
        self.gridLayout.setRowStretch(2, 1)


    def angleChange(self):
        try:
            self.angle = int(self.angleText.text()) % 360   # 显示360°内
        except:
            return  # 无效输入则不变
        self.drawPlt.clear()
        self.drawRoad(self.roadData)
        self.angleText.setText(str(self.angle))
        self.cars = pg.ScatterPlotItem(pen=pg.mkPen(None), pxMode=False, useCache=False)
        self.texts = pg.ScatterPlotItem()
        self.drawPlt.addItem(self.cars)
        self.drawPlt.addItem(self.texts)
        self.carsItem = {}


    def minusAngle(self):
        self.angleText.setText(str(self.angle - 1))
        self.angleChange()

    def addAngle(self):
        self.angleText.setText(str(self.angle + 1))
        self.angleChange()

    def leftAngle(self):
        self.angleText.setText(str(self.angle - 15))
        self.angleChange()

    def rightAngle(self):
        self.angleText.setText(str(self.angle + 15))
        self.angleChange()


    # 画道路
    def drawRoad(self, data):
        self.roadData = data
        self.roadLane = data['roadLane']
        self.zone = data['zone']
        self.sensor = data["sensor"]
        self.zoneToRoadLane = data["zoneToRoadLane"]
        sensorX = float(self.sensor["x"])
        sensorY = float(self.sensor["y"])
        self.sensorAngle = float(self.sensor["angle"])

        for d in self.roadLane:
            roadId = int(d['roadId'])
            # 改成绘图坐标系且改成相对于雷达坐标
            x1 = -(float(d['y1']) - sensorY)
            x2 = -(float(d['y2']) - sensorY)
            y1 = float(d['x1']) - sensorX
            y2 = float(d['x2']) - sensorX
            length = float(d['length'])
            width = float(d['width'])
            if roadId >= 0:  # 来向
                # print(">0 ", x1, x2, y1, y2)
                self.cos = (y1 - y2) / length
                self.sin = (x1 - x2) / length
            else:
                # print("<0 ", x1, x2, y1, y2)
                self.cos = (y2 - y1) / length
                self.sin = (x2 - x1) / length

            # 道路改直且视角改成雷达出发
            tempX1 = -(x1 * self.cos - y1 * self.sin)
            tempX2 = -(x2 * self.cos - y2 * self.sin)
            tempX1Left = tempX1 - width/2
            tempX1Right = tempX1 + width/2
            tempX2Left = tempX2 - width/2
            tempX2Right = tempX2 + width/2
            tempY1 = -(x1 * self.sin + y1 * self.cos)
            tempY2 = -(x2 * self.sin + y2 * self.cos)

            realX1Left = tempX1Left * math.cos(self.angle*math.pi/180) + tempY1 * math.sin(self.angle*math.pi/180)
            realY1Left = tempY1 * math.cos(self.angle*math.pi/180) - tempX1Left * math.sin(self.angle*math.pi/180)
            realX1Right = tempX1Right * math.cos(self.angle*math.pi/180) + tempY1 * math.sin(self.angle*math.pi/180)
            realY1Right = tempY1 * math.cos(self.angle * math.pi / 180) - tempX1Right * math.sin(self.angle * math.pi / 180)
            realX2Left = tempX2Left * math.cos(self.angle*math.pi/180) + tempY2 * math.sin(self.angle*math.pi/180)
            realY2Left = tempY2 * math.cos(self.angle*math.pi/180) - tempX2Left * math.sin(self.angle*math.pi/180)
            realX2Right = tempX2Right * math.cos(self.angle*math.pi/180) + tempY2 * math.sin(self.angle*math.pi/180)
            realY2Right = tempY2 * math.cos(self.angle * math.pi / 180) - tempX2Right * math.sin(self.angle * math.pi / 180)
            

            # 画路
            pen = QPen(QColor(self.roadColor), 0.3, Qt.SolidLine)
            self.drawPlt.plot([realX1Left, realX2Left], [realY1Left, realY2Left], pen=pen)
            self.drawPlt.plot([realX1Right, realX2Right], [realY1Right, realY2Right], pen=pen)


            # 绘制虚线，每20m
            pen = QPen(QColor(self.roadColor), 0.1, Qt.DashLine)
            if roadId >= 0:
                distance = tempY1
                while distance <= tempY2:
                    realX1Left = tempX1Left * math.cos(self.angle * math.pi / 180) + distance * math.sin(
                        self.angle * math.pi / 180)
                    realY1Left = distance * math.cos(self.angle * math.pi / 180) - tempX1Left * math.sin(
                        self.angle * math.pi / 180)
                    realX1Right = tempX1Right * math.cos(self.angle * math.pi / 180) + distance * math.sin(
                        self.angle * math.pi / 180)
                    realY1Right = distance * math.cos(self.angle * math.pi / 180) - tempX1Right * math.sin(
                        self.angle * math.pi / 180)
                    self.drawPlt.plot([realX1Left, realX1Right], [realY1Left, realY1Right], pen = pen)
                    distance += 20
            else:
                distance = tempY2
                while distance <= tempY1:
                    realX2Left = tempX2Left * math.cos(self.angle * math.pi / 180) + distance * math.sin(
                        self.angle * math.pi / 180)
                    realY2Left = distance * math.cos(self.angle * math.pi / 180) - tempX2Left * math.sin(
                        self.angle * math.pi / 180)
                    realX2Right = tempX2Right * math.cos(self.angle * math.pi / 180) + distance * math.sin(
                        self.angle * math.pi / 180)
                    realY2Right = distance * math.cos(self.angle * math.pi / 180) - tempX2Right * math.sin(
                        self.angle * math.pi / 180)
                    self.drawPlt.plot([realX2Left, realX2Right], [realY2Left, realY2Right], pen = pen)
                    distance += 20


            # 画道路方向
            pen = QPen(QColor(self.roadColor), 0.5, Qt.SolidLine)
            if roadId >= 0:
                buttomX = tempX1 * math.cos(self.angle * math.pi / 180) + \
                          (tempY1 + 2) * math.sin(self.angle * math.pi / 180)
                buttomY = (tempY1 + 2) * math.cos(self.angle * math.pi / 180) - tempX1 * math.sin(
                        self.angle * math.pi / 180) 
                topX = tempX1 * math.cos(self.angle * math.pi / 180) + (tempY1 + 12) * math.sin(
                        self.angle * math.pi / 180) 
                topY = (tempY1 + 12) * math.cos(self.angle * math.pi / 180) - tempX1 * math.sin(
                        self.angle * math.pi / 180) 
                arrowLeftX = (tempX1 - 1) * math.cos(self.angle * math.pi / 180) + (tempY1 + 3.732) * math.sin(
                        self.angle * math.pi / 180) 
                arrowLeftY = (tempY1 + 3.732) * math.cos(self.angle * math.pi / 180) - (tempX1 - 1) * math.sin(
                        self.angle * math.pi / 180) 
                arrowRightX = (tempX1 + 1) * math.cos(self.angle * math.pi / 180) + (tempY1 + 3.732) * math.sin(
                        self.angle * math.pi / 180) 
                arrowRightY = (tempY1 + 3.732) * math.cos(self.angle * math.pi / 180) - (tempX1 + 1) * math.sin(
                        self.angle * math.pi / 180) 
                self.drawPlt.plot([buttomX, topX], [buttomY, topY], pen=pen)
                self.drawPlt.plot([arrowLeftX, buttomX], [arrowLeftY, buttomY], pen=pen)
                self.drawPlt.plot([buttomX, arrowRightX], [buttomY, arrowRightY], pen=pen)
            else:
                buttomX = tempX2 * math.cos(self.angle * math.pi / 180) + (tempY2 + 2) * math.sin(
                        self.angle * math.pi / 180) 
                buttomY = (tempY2 + 2) * math.cos(self.angle * math.pi / 180) - tempX2 * math.sin(
                        self.angle * math.pi / 180) 
                topX = tempX2 * math.cos(self.angle * math.pi / 180) + (tempY2 + 12) * math.sin(
                        self.angle * math.pi / 180) 
                topY = (tempY2 + 12) * math.cos(self.angle * math.pi / 180) - tempX2 * math.sin(
                        self.angle * math.pi / 180) 
                arrowLeftX = (tempX2 - 1) * math.cos(self.angle * math.pi / 180) + (tempY2 + 12 - 1.732) * math.sin(
                        self.angle * math.pi / 180) 
                arrowLeftY = (tempY2 + 12 - 1.732) * math.cos(self.angle * math.pi / 180) - (tempX2 - 1) * math.sin(
                        self.angle * math.pi / 180) 
                arrowRightX = (tempX2 + 1) * math.cos(self.angle * math.pi / 180) + (tempY2 + 12 - 1.732) * math.sin(
                        self.angle * math.pi / 180) 
                arrowRightY = (tempY2 + 12 - 1.732) * math.cos(self.angle * math.pi / 180) - (tempX2 + 1) * math.sin(
                        self.angle * math.pi / 180) 
                self.drawPlt.plot([buttomX, topX], [buttomY, topY], pen=pen)
                self.drawPlt.plot([arrowLeftX, topX], [arrowLeftY, topY], pen=pen)
                self.drawPlt.plot([topX, arrowRightX], [topY, arrowRightY], pen=pen)


        # 绘制检测区域
        for z in self.zone:
            zoneId = str(z['zoneId'])
            x1 = -(float(z['y1']) - sensorY)
            x2 = -(float(z['y2']) - sensorY)
            y1 = float(z['x1']) - sensorX
            y2 = float(z['x2']) - sensorX
            length = float(z['length'])
            width = float(z['width'])
            roadLane = self.zoneToRoadLane[zoneId]
            if int(roadLane) >= 0:
                cos = (y1 - y2) / length
                sin = (x1 - x2) / length
            else:
                cos = (y2 - y1) / length
                sin = (x2 - x1) / length

            tempX1 = -(x1 * cos - y1 * sin)
            tempX2 = -(x2 * cos - y2 * sin)
            tempX1Left = tempX1 - width/2
            tempX1Right = tempX1 + width/2
            tempX2Left = tempX2 - width/2
            tempX2Right = tempX2 + width/2
            tempY1 = -(x1 * sin + y1 * cos)
            tempY2 = -(x2 * sin + y2 * cos)

            realX1Left = tempX1Left * math.cos(self.angle*math.pi/180) + tempY1 * math.sin(self.angle*math.pi/180)
            realY1Left = tempY1 * math.cos(self.angle*math.pi/180) - tempX1Left * math.sin(self.angle*math.pi/180)
            realX1Right = tempX1Right * math.cos(self.angle*math.pi/180) + tempY1 * math.sin(self.angle*math.pi/180)
            realY1Right = tempY1 * math.cos(self.angle * math.pi / 180) - tempX1Right * math.sin(self.angle * math.pi / 180)
            realX2Left = tempX2Left * math.cos(self.angle*math.pi/180) + tempY2 * math.sin(self.angle*math.pi/180)
            realY2Left = tempY2 * math.cos(self.angle*math.pi/180) - tempX2Left * math.sin(self.angle*math.pi/180)
            realX2Right = tempX2Right * math.cos(self.angle*math.pi/180) + tempY2 * math.sin(self.angle*math.pi/180)
            realY2Right = tempY2 * math.cos(self.angle * math.pi / 180) - tempX2Right * math.sin(self.angle * math.pi / 180)

            pen = QPen(QColor(self.zoneColor), 0.4, Qt.SolidLine)
            self.drawPlt.plot([realX1Left, realX2Left], [realY1Left, realY2Left], pen=pen)
            self.drawPlt.plot([realX1Right, realX2Right], [realY1Right, realY2Right], pen=pen)


        # 绘制雷达
        r = np.array([1, 2, 3], dtype=float)
        thrata = np.arange(0, 2 * np.pi, 0.01)
        x = np.zeros([r.shape[0], thrata.shape[0]])
        y = np.zeros([r.shape[0], thrata.shape[0]])
        k = 0
        for i in r:
            x[k, :] = i * np.cos(thrata)
            y[k, :] = i * np.sin(thrata) - 3
            self.drawPlt.plot(x[k, :], y[k, :],  pen=(0,255,0))
            k = k + 1

        self.drawPlt.plot([0, 0], [0, -6], pen=(0, 255, 0))
        self.drawPlt.plot([-3, 3], [-3, -3], pen=(0, 255, 0))
        self.drawPlt.plot([0, 2.12], [-3, -0.78], pen=(0, 255, 0))
        # self.drawPlt.plot([0, 0], [0, 0], pen=(255, 0, 0), symbolBrush=(255,0,0), symbolPen='w')


    '''
    画车
    车框由分散点集实现，每个点为创造的不同形状，点集一次性画上从而大大缩减绘图时间
    文字框也由分散点集实现
    '''
    def drawCars(self, data, showCarInfo):
        self.carsItem = {}
        start = timeit.default_timer()
        carSpots = []
        textSpots = []

        for c in data:
            id = c["Vehicle ID"]
            oriX = float(c['X Point'])
            oriY = float(c['Y Point'])
            x = -(oriY * math.cos(-self.sensorAngle*math.pi/180) - oriX * math.sin(-self.sensorAngle*math.pi/180))
            y = oriX * math.cos(-self.sensorAngle*math.pi/180) + oriY * math.sin(-self.sensorAngle*math.pi/180)
            speed = c['X Speed']
            length = float(c['Vehicle Length'])
            tempX = -(x * self.cos - y * self.sin)      # 坐标系变换：http://blog.sina.com.cn/s/blog_3fd642cf0101cc8w.html
            tempY = -(x * self.sin + y * self.cos)

            realX = tempX * math.cos(self.angle*math.pi/180) + tempY * math.sin(self.angle*math.pi/180)
            realY = tempY * math.cos(self.angle*math.pi/180) - tempX * math.sin(self.angle*math.pi/180)

            car = self.createCar('-', 90-self.angle, length)    # rotate为逆时针旋转
            dis = 0.75 * math.cos(self.angle * math.pi / 180)
            carSpots.append({'pos': [realX + dis, realY], 'data': id, 'brush': pg.mkBrush(QColor(self.carColor)), 'symbol': car, 'size': 1.5})
            # 这里+0.75是因为spots画点为向左画，而实际坐标应该是目标框中点，所以向右移动车框宽度的一半，即1.5/2=0.75
            # 由于有角度调整，所以映射到X轴上要*cos

            self.carsItem[id] = [realX, realY, length, speed]

            if showCarInfo:
                text = self.createLabel(id)
                textSpots.append({'pos': [realX -1 + dis, realY - 1], 'data': 1, 'brush': pg.mkBrush(QColor(self.idColor)), 'symbol': text, 'size': 15})

        self.cars.setData(carSpots)
        self.texts.setData(textSpots)
        self.drawPlt.repaint()      # 主动调用重绘，否则会有残影，原因不明

        runtime = timeit.default_timer() - start
        # print(runtime)
        if runtime >= 0.08:
            logging.warning("Draw car overtime {}".format(runtime))
            logging.warning(len(data))



    '''
    显示车流量统计
    '''
    def drawCarNums(self, carsFlow):
        if self.carIndex == 0:
            self.carNumPlt.removeItem(self.curve)
            self.carNumPlt.setXRange(0, 10)
            self.carNumPlt.setYRange(0, 30)
            self.curve = self.carNumPlt.plot()


        self.carNums.append(carsFlow)
        self.xIndex.append(float(self.carIndex / 10))
        self.carIndex += 1
        if self.carIndex >= 100:
            self.carNumPlt.setXRange(float(self.carIndex-100) / 10, float(self.carIndex / 10))

        self.curve.setData(self.xIndex, self.carNums, pen='g')


    '''
    显示选中车的速度曲线
    '''
    def drawCarSpeed(self):
        # 改变选中对象，则清空重新画
        if self.curCarId != self.drawingCarId:
            self.carSpeedPlt.removeItem(self.curve2)
            self.carSpeedPlt.setYRange(0, 20)
            self.carSpeedPlt.setXRange(0, 10)
            self.curve2 = self.carSpeedPlt.plot()
            self.carSpeed = []
            self.xIndex2 = []
            self.carIndex2 = 0

        self.drawingCarId = self.curCarId
        self.carSpeedPlt.setLabel('bottom', 'ID', self.drawingCarId)

        if self.drawingCarId in self.carsItem:
            speed = self.carsItem[self.drawingCarId][3]
            self.carSpeed.append(abs(float(speed)))
            self.xIndex2.append(float(self.carIndex2 / 10))
            self.carIndex2 += 1
            if self.carIndex2 >= 100:
                self.carSpeedPlt.setXRange(float(self.carIndex2 - 100) / 10, float(self.carIndex2 / 10))
            self.curve2.setData(self.xIndex2, self.carSpeed, pen='g')


    # 重置车辆数目统计参数
    def resetCarNums(self):
        self.carNums = []
        self.xIndex = []
        self.carIndex = 0


    # 重置车辆速度统计参数
    def resetCarSpeed(self):
        self.carSpeed = []
        self.xIndex2 = []
        self.carIndex2 = 0
        self.drawingCarId = '-1'


    # 清除已画车框和文字
    def clean(self):
        start = timeit.default_timer()
        self.cars.clear()
        self.texts.clear()
        self.carsItem = {}
        # self.drawPlt.repaint()      # 主动调用重绘，否则会有残影，原因不明
        # print("Clear time", timeit.default_timer() - start)

    def clearSelectedCar(self):
        for p in self.cars.points():
            p.setBrush(pg.mkBrush(QColor(self.carColor)))
        self.drawPlt.repaint()
        self.curCarId = '-1'
        self.selectCar.emit('-1')  # 发出-1信号

    '''
    点击车框，则变色标记，并发出信号；
    点击外部区域，则全部恢复颜色
    '''
    def mousePressEvent(self, _event):
        if _event.buttons() != QtCore.Qt.LeftButton:
            return
        curPoint = _event.globalPos()
        curX = self.drawPlt.mapFromGlobal(curPoint).x()
        curY = self.drawPlt.mapFromGlobal(curPoint).y()

        selectedId = self.curCarId

        for id in self.carsItem:
            x1, y1, length, speed = self.carsItem[id]
            x2 = x1 + length * math.sin(self.angle * math.pi / 180)
            y2 = y1 + length * math.cos(self.angle * math.pi / 180)
            '''
            车雷达图坐标由坐标系转换成车道绘图窗口坐标，然后判断鼠标点击是否在当前车框内
            判断是否在车框内：鼠标点击位置离车框左右和上下的距离小于宽/长
            '''
            cos = math.cos(-self.angle * math.pi / 180)
            sin = math.sin(-self.angle * math.pi / 180)
            tan = math.tan(-self.angle * math.pi / 180)
            leftButtomX, leftButtomY = x1 - 0.75 * cos, y1 + 0.75 * sin
            leftButtom = self.cars.mapToDevice(pg.Point(leftButtomX, leftButtomY))
            buttom = self.cars.mapToDevice(pg.Point(x1, y1))
            top = self.cars.mapToDevice(pg.Point(x2, y2))

            if self.angle == 0 or self.angle == 180:
                if abs(curX - buttom.x()) <= abs(buttom.x() - leftButtom.x()) and abs(curY - buttom.y()) <= abs(buttom.y() - top.y()) and abs(curY - top.y()) <= abs(buttom.y() - top.y()):
                    # 选中发出信号，改变当前选中车ID
                    self.selectCar.emit(str(id))
                    selectedId = str(id)
                    break
            elif self.angle == 90 or self.angle == 270:
                if abs(curY - buttom.y()) <= abs(buttom.y() - leftButtom.y()) and abs(curX - buttom.x()) <= abs(buttom.x() - top.x()) and abs(curX - top.x()) <= abs(buttom.x() - top.x()):
                    # 选中发出信号，改变当前选中车ID
                    self.selectCar.emit(str(id))
                    selectedId = str(id)
                    break
            else:
                # 点到左右中线的距离小于等于宽的一半；点到顶和底的距离小于等于长
                if self.disFromSpotToLine(top.x(), top.y(), 1/tan, curX, curY) <= self.disBetweenTwoSpots(buttom.x(), buttom.y(), leftButtom.x(), leftButtom.y())   \
                    and self.disFromSpotToLine(buttom.x(), buttom.y(), -1/tan, curX, curY) <= self.disBetweenTwoSpots(buttom.x(), buttom.y(), top.x(), top.y())    \
                    and self.disFromSpotToLine(top.x(), top.y(), -1/tan, curX, curY) <= self.disBetweenTwoSpots(buttom.x(), buttom.y(), top.x(), top.y()):
                    # 选中发出信号，改变当前选中车ID
                    self.selectCar.emit(str(id))
                    selectedId = str(id)
                    break

        if selectedId != self.curCarId:
            self.curCarId = selectedId
            # 改变车框选中情况
            for p in self.cars.points():
                if p.data() == self.curCarId:
                    p.setBrush(pg.mkBrush(255,0,0))
                else:
                    p.setBrush(pg.mkBrush(QColor(self.carColor)))
            self.drawPlt.repaint()


    # 点到线的距离 https://baike.baidu.com/item/点到直线距离
    def disFromSpotToLine(self, x1, y1, tan, x2, y2):
        distance = abs(tan * x2 - y2 + y1 - tan * x1) / math.sqrt(pow(tan, 2) + 1)
        # print("Spots to Line:" + str(distance))
        return distance


    # 点到点的距离
    def disBetweenTwoSpots(self, x1, y1, x2, y2):
        distance = math.sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2))
        # print("Bewtween spots:" + str(distance))
        return distance


    # 设定选定车
    def setSelectedCar(self, selectedId):
        self.curCarId = str(selectedId)
        # print(selectedId)
        for p in self.cars.points():
            if p.data() == self.curCarId:
                p.setBrush(pg.mkBrush(255, 0, 0))
            else:
                p.setBrush(pg.mkBrush(QColor(self.carColor)))
        self.drawPlt.repaint()


    # 目标框点
    def createCar(self, label, angle, length):
        symbol = QtGui.QPainterPath()
        f = QtGui.QFont()
        f.setPointSize(1)
        f.setBold(True)
        symbol.addText(0, 0, f, label)
        br = symbol.boundingRect()
        tr = QtGui.QTransform()
        tr.rotate(angle)
        tr.scale(length / 1.5 / br.width(), 1. / br.height())
        tr.translate(-br.x(), -br.y())
        s = tr.map(symbol)
        return s


    # 文字框点
    def createLabel(self, label):
        symbol = QtGui.QPainterPath()
        f = QtGui.QFont()
        f.setPointSize(1)
        f.setLetterSpacing(QtGui.QFont.PercentageSpacing, 60)
        symbol.addText(0, 0, f, label)
        br = symbol.boundingRect()
        tr = QtGui.QTransform()
        scale = min(1. / br.width(), 1. / br.height())
        tr.scale(scale, scale)
        tr.translate(-br.x() - br.width() / 2., -br.y() - br.height() / 2.)     # 必须居中，否则显示不全
        s = tr.map(symbol)
        return s

