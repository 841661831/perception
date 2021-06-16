# -*-coding:utf-8-*
import time
tStart = time.time()
from multiprocessing import freeze_support
from Widget import View
from Widget.View import *
sys.path.append(os.path.join(os.path.dirname(__file__), 'cureUi-master'))


if __name__ == '__main__':
    freeze_support()
    # mp.set_start_method('spawn', force=True)
    mp.set_start_method('fork', force=True)
    MyLog.writeLog("{}[:{}] - Application start, pid={}".
             format(__file__.split('/')[len(__file__.split('/')) - 1],
                   sys._getframe().f_lineno, os.getpid()), LOG_INFO, ENABLE_WRITE_LOG, ENABLE_PRINT)

    app = QApplication(sys.argv)
    # 下面这三行就是汉化的
    # translator = QTranslator()
    # translator.load("./Configs/Translation/View_zh_CN.qm")
    # app.installTranslator(translator)
    strTranslationPath = strExePath + "/Configs/Translation/"
    if my_isExist(strTranslationPath):
        dir_list = os.listdir(strTranslationPath)
        if len(dir_list) > 0:
            for file in dir_list:
                filename, extension = os.path.splitext(file)
                if extension == ".qm":
                    translator = QTranslator(app)
                    translator.load(strTranslationPath + file)
                    b = app.installTranslator(translator)
                    print(strTranslationPath + file, b)
    ui = View.MainWindow()
    ui.init(tStart)
    ui.show()
    ui.sigQuit.connect(app.quit)
    if (stUiParam.SoftwareStyle==0):
        swin = cure.Windows(ui, "WJ", "blueDeep", 'WJ-ISFP', "./Configs/Icon/View.png")
    # 杀掉除自己以外的所有同名进程，必须放到最后！
    # 否则会杀掉自己的关联进程,导致程序崩溃！
    my_killProExceptSelf()

    sys.exit(app.exec())
