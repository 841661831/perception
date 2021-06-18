# -*-coding:utf-8-*
from WinCommon import *
from Widget import View

if __name__ == '__main__':
    freeze_support()
    app = QApplication(sys.argv)
    ui = View.MainWindow()
    ui.init(win_param_path)
    ui.show()
    ui.sigQuit.connect(app.quit)
    if stUiParam.SoftwareStyle == 0:
        swin = cure.Windows(ui, "WJ", "blueDeep", '全域软件', "./Configs/Icon/View.png")
    # 杀掉除自己以外的所有同名进程，必须放到最后！
    # 否则会杀掉自己的关联进程,导致程序崩溃！
    # killProExceptSelf()
    sys.exit(app.exec())