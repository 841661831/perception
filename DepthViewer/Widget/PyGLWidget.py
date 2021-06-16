# -*- coding: utf-8 -*-
from PyQt5 import QtCore, QtOpenGL
import math
import numpy
import numpy.linalg as linalg
import OpenGL
OpenGL.ERROR_CHECKING = True
from OpenGL.GL import *
from OpenGL.GLU import *
from CommonDefine import *

ZOOM_Z_INIT = -50


# QGLWidget类已经内置了对OpenGL的处理，就是通过对initializeGL()、paintGL()和resizeGL()这个三个函数实现的
class PyGLWidget(QtOpenGL.QGLWidget):
    def __init__(self, parent=None):
        format = QtOpenGL.QGLFormat()
        format.setSampleBuffers(True)
        QtOpenGL.QGLWidget.__init__(self, format, parent)

        self.modelview = []
        self.fNear = 0.1
        self.fFar = 10000.0
        self.fFovy = 45.0
        self.fRadius = 5.0
        self.fWidth = 0
        self.fHeight = 0
        self.p2DLast = QtCore.QPoint()
        self.p3DLast = [1.0, 0.0, 0.0]
        self.bLastPointOk = False
        self.bIsInRotation = False
        self.nZoomValue = 0
        self.bRegistration = False
        self.bSelectCube = False

        self.setCursor(QtCore.Qt.OpenHandCursor)
        self.setMouseTracking(False)
        self.setFocusPolicy(QtCore.Qt.ClickFocus)

    # initializeGL()是用来初始化这个OpenGL窗口部件的，可以在里面设定一些有关选项,这个函数直到OpenGL窗口创建之后才会被调用!
    def initializeGL(self):
        # OpenGL state
        # glClearColor设置清除屏幕时所用的颜色。如果您对色彩的工作原理不清楚的话，我快速解释一下。
        # 色彩值的范围从0.0到1.0。0.0代表最黑的情况，1.0就是最亮的情况。
        # glClearColor后的第一个参数是红色,第二个是绿色，第三个是蓝色。最大值也是1.0，代表特定颜色分量的最亮情况。
        # 最后一个参数是Alpha值。当它用来清除屏幕的时候，我们不用关心第四个数字。现在让它为0.0。
        # 通过混合三种原色（红、绿、蓝），您可以得到不同的色彩。因此，当您使用glClearColor(0.0, 0.0, 1.0, 0.0 )，您将用亮蓝色来清除屏幕。
        # 如果您用glClearColor(0.5, 0.0, 0.0, 0.0 )的话，您将使用中红色来清除屏幕。不是最亮(1.0)，也不是最暗 (0.0)。
        # 要得到白色背景，您应该将所有的颜色设成最亮(1.0)。要黑色背景的话，您该将所有的颜色设为最暗(0.0)。
        glClearColor(0.0, 0.0, 0.0, 0.0)

        # 启用深度测试。将深度缓存设想为屏幕后面的层,深度缓存不断的对物体进入屏幕内部有多深进行跟踪。
        # 几乎所有在屏幕上显示3D场景OpenGL程序都使用深度缓存。它的排序决定那个物体先画。
        # 这样您就不会将一个圆形后面的正方形画到圆形上来。深度缓存是OpenGL十分重要的部分
        glEnable(GL_DEPTH_TEST)
        self.resetView()
        # print("initializeGL:", self.modelview)

    # resizeGL()就是用来处理窗口大小变化这一事件的，width和height就是新的大小状态下的宽和高了，另外resizeGL()在处理完后会自动刷新屏幕。
    # 这个函数的作用是重新设置OpenGL场景的大小，而不管窗口的大小是否已经改变（假定您没有使用全屏模式）。甚至您无法改变窗口的大小时（例如您在全屏模式下），
    # 它至少仍将运行一次——在程序开始时设置我们的透视图。OpenGL场景的尺寸将被设置成它显示时所在窗口的大小。
    def resizeGL(self, width, height):
        self.fWidth = width
        self.fHeight = height
        glViewport(0, 0, width, height)
        self.setProjection(self.fNear, self.fFar, self.fFovy)
        self.updateGL()
        # print("resizeGL:", self.modelview)

    # paintGL()就是用来绘制OpenGL的窗口了，只要有更新发生，这个函数就会被调用
    def paintGL(self):
        # 清楚屏幕和深度缓存。
        # glViewport(0, 0, 100, 100)
        # glEnable(GL_SCISSOR_TEST)
        # glScissor(0, 0, 100, 100)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # glMatrixMode(GL_MODELVIEW)指明任何新的变换将会影响 modelview matrix（模型观察矩阵）。模型观察矩阵中存放了我们的物体讯息。
        glMatrixMode(GL_MODELVIEW)

        # 最后我们重置模型观察矩阵,如果您想获得一个精彩的透视场景的话，必须这么做
        glLoadMatrixd(self.modelview)
        # print("paintGL:", self.modelview)

    def translate(self, _trans):
        # Translate the object by _trans
        # Update modelview_matrix_
        self.makeCurrent()
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslated(_trans[0], _trans[1], _trans[2])
        glMultMatrixd(self.modelview)
        self.modelview = glGetDoublev(GL_MODELVIEW_MATRIX)
        self.nZoomValue += _trans[2]
        # MyLog.writeLog("{}[:{}] - nZoomValue = {}".format(__file__.split('/')[len(__file__.split('/')) - 1],
        #                                             sys._getframe().f_lineno, self.nZoomValue),
        #                                             LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)

    def translate_mid(self, _trans):
        # Translate the object by _trans
        # Update modelview_matrix_
        self.makeCurrent()
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslated(_trans[0], _trans[1], _trans[2])
        glMultMatrixd(self.modelview)
        self.modelview = glGetDoublev(GL_MODELVIEW_MATRIX)
        self.nZoomValue = ZOOM_Z_INIT
        MyLog.writeLog("{}[:{}] - nZoomValue = {}".format(__file__.split('/')[len(__file__.split('/')) - 1],
                                                          sys._getframe().f_lineno, self.nZoomValue),
                       LOG_WARNING, ENABLE_WRITE_LOG, ENABLE_PRINT)


    def rotate(self, _axis, _angle):
        t = [self.modelview[0][0] * self.aCenter[0] +
             self.modelview[1][0] * self.aCenter[1] +
             self.modelview[2][0] * self.aCenter[2] +
             self.modelview[3][0],
             self.modelview[0][1] * self.aCenter[0] +
             self.modelview[1][1] * self.aCenter[1] +
             self.modelview[2][1] * self.aCenter[2] +
             self.modelview[3][1],
             self.modelview[0][2] * self.aCenter[0] +
             self.modelview[1][2] * self.aCenter[1] +
             self.modelview[2][2] * self.aCenter[2] +
             self.modelview[3][2]]

        self.makeCurrent()
        glLoadIdentity()
        glTranslatef(t[0], t[1], t[2])  # 平移
        glRotated(_angle, _axis[0], _axis[1], _axis[2])
        glTranslatef(-t[0], -t[1], -t[2])
        glMultMatrixd(self.modelview)
        self.modelview = glGetDoublev(GL_MODELVIEW_MATRIX)

    def viewInit(self):
        self.translate([-(self.modelview[0][0] * self.aCenter[0] +
                          self.modelview[0][1] * self.aCenter[1] +
                          self.modelview[0][2] * self.aCenter[2] +
                          self.modelview[0][3]),
                        -(self.modelview[1][0] * self.aCenter[0] +
                          self.modelview[1][1] * self.aCenter[1] +
                          self.modelview[1][2] * self.aCenter[2] +
                          self.modelview[1][3]),
                        -(self.modelview[2][0] * self.aCenter[0] +
                          self.modelview[2][1] * self.aCenter[1] +
                          self.modelview[2][2] * self.aCenter[2] +
                          self.modelview[2][3] +
                          self.fRadius / 2.0)])

    def coordinateConversion(self, p2D):
        p3D = [0.0, 0.0, 0.0]
        if ((p2D.x() >= 0) and (p2D.x() <= self.width()) and
                (p2D.y() >= 0) and (p2D.y() <= self.height())):
            x = float(p2D.x() - 0.5 * self.width()) / self.width()
            y = float(0.5 * self.height() - p2D.y()) / self.height()

            p3D[0] = x
            p3D[1] = y
            z = 2.0 * 0.5 * 0.5 - x * x - y * y
            p3D[2] = math.sqrt(max(z, 0.0))

            n = linalg.norm(p3D)
            p3D = numpy.array(p3D) / n

            return True, p3D
        else:
            return False, p3D

    def setProjection(self, fNear, fFar, fFovy):
        self.fNear = fNear
        self.fFar = fFar
        self.fFovy = fFovy
        self.makeCurrent()

        # 下面几行为透视图设置屏幕。意味着越远的东西看起来越小。这么做创建了一个现实外观的场景!
        # glMatrixMode(GL_PROJECTION)指明接下来的两行代码将影响projection matrix（投影矩阵）。投影矩阵负责为我们的场景增加透视。
        glMatrixMode(GL_PROJECTION)

        # glLoadIdentity()近似于重置。它将所选的矩阵状态恢复成其原始状态。调用glLoadIdentity()之后我们为场景设置透视图。
        # 当您调用glLoadIdentity()之后，您实际上将当前点移到了屏幕中心，X坐标轴从左至右，Y坐标轴从下至上，Z坐标轴从里至外。
        # OpenGL屏幕中心的坐标值是X和Y轴上的0.0点。中心左面的坐标值是负值，右面是正值。移向屏幕顶端是正值，移向屏幕底端是负值。移入屏幕深处是负值，移出屏幕则是正值。
        glLoadIdentity()

        # 此处透视按照基于窗口宽度和高度的fFovy度视角来计算。near_, fFar 是我们在场景中所能绘制深度的起点和终点。
        gluPerspective(self.fFovy, float(self.width()) / float(self.height()), self.fNear, self.fFar)
        self.updateGL()

    def setCenter(self, aCenter):
        self.aCenter = aCenter
        self.viewInit()

    # def set_radius(self, fRadius):
    #     self.fRadius= fRadius
    #     self.setProjection(fRadius / 100.0, fRadius * 100.0, self.fFovy)
    #     self.resetView()
    #     self.translate([0, 0, -fRadius * 2.0])
    #     self.viewInit()
    #     self.updateGL()

    def resetView(self):
        # scene pos and size
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()  # 调用glLoadIdentity()之后，您实际上将当前点移到了屏幕中心：类似于一个复位操作
        self.modelview = glGetDoublev(GL_MODELVIEW_MATRIX)
        self.setCenter([0.0, 0.0, 0.0])
        self.translate([0.0, 0.0, ZOOM_Z_INIT - self.nZoomValue])

    def resetView_selectStation(self,x=0.0,y=0.0,viewHeight=100):

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()  # 调用glLoadIdentity()之后，您实际上将当前点移到了屏幕中心：类似于一个复位操作
        self.modelview = glGetDoublev(GL_MODELVIEW_MATRIX)
        self.setCenter([x, y, 0.0])
        self.translate([0.0, 0.0, ZOOM_Z_INIT - viewHeight])
        self.updateGL()

    def resetRotation(self):
        self.modelview[0] = [1.0, 0.0, 0.0, 0.0]
        self.modelview[1] = [0.0, 1.0, 0.0, 0.0]
        self.modelview[2] = [0.0, 0.0, 1.0, 0.0]
        glMatrixMode(GL_MODELVIEW)
        glLoadMatrixd(self.modelview)

    def resetTranslate(self):
        self.translate([0.0, 0.0, -self.nZoomValue + ZOOM_Z_INIT])

    def wheelEvent(self, event):
        d = float(event.angleDelta().y()) / 200.0 * self.fRadius  # d > 0 zoom out    d < 0 zoom in
        self.translate([0.0, 0.0, d])
        self.updateGL()
        event.accept()

    def mousePressEvent(self, event):
        self.p2DLast = event.pos()
        self.bLastPointOk, self.p3DLast = self.coordinateConversion(self.p2DLast)
        QtOpenGL.QGLWidget.mousePressEvent(self, event)

    def mouseMoveEvent(self, event):
        p2DNew = event.pos()
        if ((p2DNew.x() < 0) or (p2DNew.x() > self.width()) or
                (p2DNew.y() < 0) or (p2DNew.y() > self.height())):
            return

        bNewPointOk, p3DNew = self.coordinateConversion(p2DNew)

        dx = float(p2DNew.x() - self.p2DLast.x())
        dy = float(p2DNew.y() - self.p2DLast.y())

        w = float(self.width())
        h = float(self.height())

        self.makeCurrent()
        if event.buttons() & QtCore.Qt.MidButton:
            z = - (self.modelview[0][2] * self.aCenter[0] +
                   self.modelview[1][2] * self.aCenter[1] +
                   self.modelview[2][2] * self.aCenter[2] +
                   self.modelview[3][2]) / (self.modelview[0][3] * self.aCenter[0] +
                                                    self.modelview[1][3] * self.aCenter[1] +
                                                    self.modelview[2][3] * self.aCenter[2] +
                                                    self.modelview[3][3])

            fFovy = 45.0
            aspect = w / h
            n = 0.01 * self.fRadius
            up = math.tan(fFovy / 2.0 * math.pi / 180.0) * n
            right = aspect * up

            self.translate([2.0 * dx / w * right / n * z,
                            -2.0 * dy / h * up / n * z,
                            0.0])

        # rotate
        elif (event.buttons() & QtCore.Qt.LeftButton):
            if not self.bIsInRotation:
                self.bIsInRotation = True

            axis = [0.0, 0.0, 0.0]
            angle = 0.0

            if self.bLastPointOk and bNewPointOk:
                axis = numpy.cross(self.p3DLast, p3DNew)
                cos_angle = numpy.dot(self.p3DLast, p3DNew)
                if abs(cos_angle) < 1.0:
                    angle = math.acos(cos_angle) * 180.0 / math.pi
                    angle *= 2.0
                self.rotate(axis, angle)

        # remember this point
        self.p2DLast = p2DNew
        self.p3DLast = p3DNew
        self.bLastPointOk = bNewPointOk

        # trigger redraw
        self.updateGL()

    def mouseReleaseEvent(self, event):
        if self.bIsInRotation:
            self.bIsInRotation = False
        self.bLastPointOk = False

    def mouseDoubleClickEvent(self, QMouseEvent):
        # self.resetRotation()
        # self.resetTranslate()
        # self.updateGL()
        if QMouseEvent.buttons() == QtCore.Qt.LeftButton or QMouseEvent.buttons() == QtCore.Qt.MidButton:
            self.resetRotation()
            self.resetTranslate()
            self.updateGL()
        if QMouseEvent.buttons() == QtCore.Qt.RightButton:
            glLoadIdentity()  # 调用glLoadIdentity()之后，您实际上将当前点移到了屏幕中心：类似于一个复位操作
            self.modelview = glGetDoublev(GL_MODELVIEW_MATRIX)
            self.setCenter([0.0, 0.0, 0.0])
            # # self.translate([0.0, 0.0, ZOOM_Z_INIT + self.nZoomValue])
            self.translate_mid([0.0, 0.0, ZOOM_Z_INIT])
            self.updateGL()

    def setRegistration(self, bStationRegistration):
        self.bRegistration = bStationRegistration

    def setSelectCube(self, bSelectCube):
        self.bSelectCube = bSelectCube

