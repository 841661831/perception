# const color string
WHITE = "#FFFFFF"
BLACK = "#000000"
RED = "#FF0000"
GREEN = "#00FF00"
BLUE = "#0000FF"
PURPLE = "#B23AEE"
WATCHET = "#1C86EE"
LIGHTGREEN = "#ECFEFE"
BLUEGREEN = "#33CCCC"
DEEPBLUEGREEN = "#015F5F"
DARKBLUEGREEN = "#28AAAA"
GRAY = "#999999"
LIGHTGRAY = "#CCCCCC"
GRAY1 = "#808080"


def getFontQss(fontname, fontcolor):
    str1 = "QObject{font-family:%s;color:%s}" % (fontname, fontcolor)
    str2 = "QObject#FileDlg{font-family:%s;color:#515151}" % (fontname)
    return str1 + str2


def getPushButtonQss(normalColor, normalTextColor, hoverColor, hoverTextColor, pressedColor, pressedTextColor,
                     disableColor, disableTextColor):
    str1 = "QPushButton{padding:0px;border-radius:5px;color:%s;background:%s;border:2px solid %s;}" % (
        normalTextColor, normalColor, normalColor)
    str2 = "QPushButton:hover{color:%s;background:%s;}" % (hoverTextColor, hoverColor)
    str3 = "QPushButton:pressed{color:%s;background:%s;}" % (pressedTextColor, pressedColor)
    str4 = "QPushButton:disabled{color:%s;background:%s;}" % (disableTextColor, disableColor)
    str6 = "QPushButton#btnPcSetting{padding:0px;border-radius:5px;color:%s;background:%s;border:2px solid %s;}" % (
        normalTextColor, "rgba(0, 125, 0, 0)", normalColor)
    str7 = "QPushButton#btnPcSetting:hover{color:%s;background:%s;}" % ("rgba(0, 125, 0, 0)",pressedTextColor)
    str8 = "QPushButton#btnPcSetting:pressed{color:%s;background:%s;}" % (pressedTextColor, "rgba(0, 125, 0, 0)")
    str9 = "QPushButton#btnPcSetting:disabled{color:%s;background:%s;}" % (disableTextColor, "rgba(0, 125, 0, 0)")
    #str5 = "QPushButton{background-color: rgba(0, 125, 0, 0)"btnPcSetting
    return str1+ str2 + str3 + str4 #+ str5 +str6 +str7 +str8 +str9

def getSettingPushButtonQss(normalColor, normalTextColor, disableTextColor, pressedTextColor, normimageurl, normimageurlhover):
    str1 = "QPushButton#btnPcSetting{padding:0px;border-radius:5px;color:%s;background:%s;border:2px solid %s;}" % (
        normalTextColor, "rgba(0, 125, 0, 0)", normalColor)
    str2 = "QPushButton#btnPcSetting:hover{color:%s;background:%s;}" % ("rgba(0, 125, 0, 0)", normalTextColor)
    str3 = "QPushButton#btnPcSetting:pressed{color:%s;background:%s;}" % (normalTextColor, "rgba(0, 125, 0, 0)")
    str4 = "QPushButton#btnPcSetting:disabled{color:%s;background:%s;}" % (disableTextColor, "rgba(0, 125, 0, 0)")
    #str5 = "QPushButton#btnPcSetting{border-image: url{:%s};}" % (normimageurlhover)
    str6 = "QPushButton#btnPcSetting:hover{border-image: url{%s};}" % (normimageurlhover)

    str7 = "QPushButton#actionTimeLeft{padding:0px;border-radius:5px;color:%s;background:%s;border:2px solid %s;}" % (
        "#515151", "rgba(0, 125, 0, 0)", pressedTextColor)
    str8 = "QPushButton#actionTimeLeft:hover{border:2px solid %s;}" % (pressedTextColor)
    str9 = "QPushButton#actionTimeLeft{font-size: 13px;color:%s}" % (pressedTextColor)
    return str1 + str2 + str3 + str4 + str7 +str8 +str9#+ str5 + str6

def getLineeditQss(normalColor, focusColor, fontLight):
    # str1 = "QLineEdit{border-style:none;padding:2px;border-radius:5px;border:2px solid %s;selection-color:%s;selection-background-color:%s;}" % (
    #     normalColor, WHITE, focusColor)
    # str1 = "QLineEdit{border-style:none;padding:2px;border-radius:5px;border:2px solid %s;selection-color:%s;selection-background-color:%s;background-color:%s;}" % (
    #     normalColor, WHITE, focusColor, BLACK)
    str1 = "QLineEdit{border-style:none;padding:2px;border-radius:5px;border:2px solid %s;selection-color:%s;selection-background-color:%s;background-color:%s;}" % (
        normalColor, WHITE, focusColor, "#515151")
    str2 = "QLineEdit:focus{border:2px solid %s;}" % (fontLight)
    str3 = "QLineEdit:disabled{color:%s;}" % (LIGHTGRAY)
    str4 = "QLineEdit{font-family:微软雅黑; color:%s}" % (fontLight)
    #str3 = "QLineEdit{border:2px solid %s;color:%s;}" % (LIGHTGRAY, LIGHTGRAY)
    return str1 + str2 + str3 + str4


def getPlaineditQss(normalColor, focusColor):
    str1 = "QPlainTextEdit{border-style:none;padding:2px;border-radius:5px;border:2px solid %s;font-family:宋体;selection-color:%s;selection-background-color:%s}" % (
        normalColor, WHITE, focusColor)
    str2 = "QPlainTextEdit:focus{border:2px solid %s;}" % (focusColor)
    return str1 + str2


def getTextBrowerQss(normalColor, focusColor):
    str1 = "QTextBrowser{border-style:none;padding:2px;border-radius:5px;border:2px solid %s;font-family:宋体;selection-color:%s;selection-background-color:%s}" % (
        normalColor, WHITE, focusColor)
    str2 = "QTextBrowser:focus{border:2px solid %s;}" % (focusColor)
    return str1 + str2


def getComboxQss(backgroundColor, normalColor, focusColor, fontDark, arrowimageurl):
    str1 = "QComboBox{background:%s;padding:2px;border-radius:5px;border:2px solid %s;}" % (
        backgroundColor, normalColor)
    str2 = "QComboBox:focus{border:2px solid %s;}" % (fontDark)
    str3 = "QComboBox:on{border:2px solid %s;}" % (focusColor)
    str4 = "QComboBox:disabled{color:%s;}" % (LIGHTGRAY)
    str5 = "QComboBox::drop-down{border-style:solid;}"
    str6 = "QComboBox QAbstractItemView{border:2px solid %s;border-radius:5px;background:transparent;selection-background-color:%s;}" % (
        focusColor, focusColor)
    str7 = "QComboBox::down-arrow{image:url(%s)}" % (arrowimageurl)

    str8 = "QComboBox#titleTableBoxType{font-family:微软雅黑; color:%s}" % (fontDark)
    str11 = "QComboBox#titleShowLidarBoxIp{font-family:微软雅黑; color:%s}" % (fontDark)
    str9 = "QComboBox#comboType{font-family:微软雅黑; color:%s}" % (fontDark)
    str10 = "QComboBox{font-family:微软雅黑; color:%s}" % ("#515151")
    return str1 + str2 + str3 + str4 + str5 + str6 + str7 + str10 + str8 + str9 + str11


def getProgressBarQss(normalColor, chunkColor):
    barHeight = str(8)
    barRadius = str(8)
    str1 = "QProgressBar{font:9pt;height:%spx;background:%s;border-radius:%spx;text-align:center;border:1px solid %s;}" % (
        barHeight, normalColor, barRadius, normalColor)
    str2 = "QProgressBar:chunk{border-radius:%spx;background-color:%s;margin:2px}" % (barRadius, chunkColor)
    return str1 + str2


def getSliderQss(normalColor, grooveColor, handleColor):
    sliderHeight = str(8)
    sliderRadius = str(4)
    handleWidth = str(13)
    handleRadius = str(6)
    handleOffset = str(3)
    str1 = "QSlider::groove:horizontal,QSlider::add-page:horizontal{height:%spx;border-radius:%spx;background:%s;}" % (
        sliderHeight, sliderRadius, normalColor)
    str2 = "QSlider::sub-page:horizontal{height:%spx;border-radius:%spx;background:%s;}" % (
        sliderHeight, sliderRadius, grooveColor)
    str3 = "QSlider::handle:horizontal{width:%spx;margin-top:-%spx;margin-bottom:-%spx;border-radius:%spx;background:qradialgradient(spread:pad,cx:0.5,cy:0.5,radius:0.5,fx:0.5,fy:0.5,stop:0.6 #FFFFFF,stop:0.8 %s);}" % (
        handleWidth, handleOffset, handleOffset, handleRadius, handleColor)
    return str1 + str2 + str3


def getRadioButtonQss(normimageurl, downimageurl, normimageurlhover, downimageurlhover):
    str1 = "QRadioButton::indicator{width:15px;height:15px;}"
    str2 = "QRadioButton::indicator:unchecked{image: url(%s);}" % (normimageurl)
    str3 = "QRadioButton::indicator:checked{image: url(%s);}" % (downimageurl)
    str4 = "QRadioButton::indicator:checked:hover{image: url(%s);}" % (downimageurlhover)
    return str1 + str2 + str3 + str4


def getCheckBoxQss(normimageurl, checkimageurl, normimageurlhover, checkimageurlhover):
    str1 = "QCheckBox::indicator{width:15px;height:15px;}"
    str2 = "QCheckBox::indicator:unchecked{image: url(%s);}" % (normimageurl)
    str3 = "QCheckBox::indicator:checked{image: url(%s);}" % (checkimageurl)
    str4 = "QCheckBox::indicator:unchecked:hover{image: url(%s);}" % (normimageurlhover)
    str5 = "QCheckBox::indicator:checked:hover{image: url(%s);}" % (checkimageurlhover)
    str6 = "QCheckBox{font-family:微软雅黑; color:%s}" % ("#F5F5F5")
    return str1 + str2 + str3 + str4 + str5 + str6


def getTabWidgetQss(normalTabColor, normalTabTextColor, tabBorderColor):
    str1 = "QTabWidget{color:%s; background:%s;}" % (normalTabTextColor, normalTabColor)
    str2 = "QTabWidget::tab-bar{left:5px}"
    str3 = "QTabBar::tab{color:%s; background:%s;width:100px;height:25px;border:2px solid %s;border-radius:2px}" % (
        normalTabTextColor, normalTabColor, tabBorderColor)
    str4 = "QTabBar::tab:hover{color:%s; background:%s;}" % (normalTabColor, normalTabTextColor)
    str5 = "QTabBar::tab:selected{color:%s; background:%s;}" % (normalTabColor, normalTabTextColor)
    return str1 + str3 + str4 + str5

def getTableWidgetQss(headColor = "#515151", FontColor = "#DCDCDC", widgetColor = "#515151"):
    str1 = "QTableWidget{color:%s;background:%s;font:9pt '宋体';color: %s;}" % (headColor, widgetColor, FontColor)
    str2 = "QHeaderView{color: %s;background-color:%s;font:9pt '宋体';color: %s;}" %(headColor, widgetColor, "#FFFFFF")#
    #str2 = "QHeaderView::section{background-color:%s;font:11pt '宋体';}" %(headColor)#color: 加了section后标标题栏最大最小也改变了
    str3 = "QListWidget::Item:hover{background:%s;padding-top:0px; padding-bottom:0px; }" %("#6495ED")
    return str1 + str3 + str2

def getMenuBarQss(backColor = RED):
    str1 = "QMenuBar{color:%s;background-color:%s;}" % (backColor, backColor)
    str2 = ""#"QMenu{color:%s;background:%s;}" % (backColor, backColor)
    return str1 + str2

def getToolBarQss():#工具栏
    #str1 = "QToolBar{color:%s;background-color:%s;}" % ("#000000", "#000000")
    #str1 = "QToolBar{color:%s;background-color:%s;}" % ("#DCDCDC", "#DCDCDC")
    str1 = "QToolBar{color:%s;background-color:%s;}" % ("#515151", "#515151")
    return str1

def getMainWindowQss():
    #str1 = "QMainWindow{color:%s;background-color: #DCDCDC}" %("#DCDCDC")
    #str1 = "QMainWindow{color:%s;background-color: #000000}" % ("#000000")
    str1 = "QMainWindow{color:%s;background-color: #515151}" % ("#515151")
    return str1

def getDialogQss():
    str1 = "QDialog{color:%s;background-color: #515151}" % ("#515151")
    str2 = "QDialog::item:hover{background-color:%s;}" % ("#515151")
    return str1 + str2

def getComboBoxQss():
    str1 = "QComboBox{color:%s;background-color: #C0C0C0}" %("#F5F5F5")
    str2 = "QComboBox{border:2px solid %s;}" % ("#FFFFFF")

    return str1 #+ str2

def getDoubleSpinBoxQss():
    str1 = "QDoubleSpinBox{font-family:微软雅黑;font-size: 13px;color:#F5F5F5;}"
    str2 = "QDoubleSpinBox{background-color: #515151}"#color:%s; % ("#515151")
    return str1 + str2

def getQActionQss():
    str1 = "QAction:hover{color:%s; background-color:%s;padding-top:0px; padding-bottom:0px; }" % ("#FF0000", "#FF0000")
    return str1

def getQFileDialogQss():
    str1 = "QFileDialog{color:%s; background-color:%s; }" % ("#515151", "#515151")
    str2 = "QFileDialog{font-family:微软雅黑;font-size: 13px;color:#515151;}"
    return str1 + str2

def getQLabelQss(normal, fontLight, fontDark):
    str1 = "QLabel{font-size:12 px; color:%s}"%(fontDark)
    return str1

def getStatusBarQss():
    str1 = "QStatusBar{font-family:微软雅黑;font-size: 13px;color:#F5F5F5;}"
    return str1

def getQSpinBoxQss():
    str1 = "QSpinBox{font-family:微软雅黑;font-size: 13px;color:#F5F5F5;}"
    str2 = "QSpinBox{color:%s; background-color:%s; }" % ("#515151", "#515151")

    return str2 + str1

def getScrollbarQss(handlebarcolor):
    str1 = "QScrollBar{background:transparent;width:10px;padding-top:11px;padding-bottom:11px}"
    str2 = "QScrollBar::handle{background:%s;border-radius:5px;min-height:10px}" % (handlebarcolor)
    str3 = "QScrollBar::handle:pressed{background:%s}" % (GRAY)
    return str1 + str2 + str3
