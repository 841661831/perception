from CommonDefine import *
from PyQt5.QtWidgets import QComboBox, QLineEdit, QListWidget, QListWidgetItem, QCheckBox

class ComboCheckBox(QComboBox):
    def __init__(self, items: list):

        super(ComboCheckBox, self).__init__()
        self.items = ["select all"] + items  # items list
        self.box_list = []  # selected items
        self.text = QLineEdit()  # use to selected items
        self.state = 1  # use to record state
        self._translate = QtCore.QCoreApplication.translate

        list_widget = QListWidget()
        for i in range(len(self.items)):
            self.box_list.append(QCheckBox())
            self.box_list[i].setText(self.items[i])
            self.box_list[i].setChecked(True)
            item = QListWidgetItem(list_widget)
            list_widget.setItemWidget(item, self.box_list[i])
            if i == 0:
                self.box_list[i].stateChanged.connect(self.all_selected)
            else:
                self.box_list[i].stateChanged.connect(self.show_selected)

        list_widget.setStyleSheet("font-size: 16px; height: 40px; margin-left: 5px;")# font-weight: bold;
        list_widget.verticalScrollBar().setStyleSheet("QScrollBar{width:10px}")
        if (stUiParam.SoftwareStyle == 0):
            self.setStyleSheet("width: 100px; height: 16px; font-size: 13px;background-color:#515151;")#; font-weight: bold;background-color:#000000;515151
        #self.text.setStyleSheet("color:#000000;background-color:#000000;")
            self.text.setStyleSheet("color:#515151;background-color:#515151;border:0px ;")
        else:
            self.setStyleSheet("width: 100px; height: 16px; font-size: 13px;")
            #self.text.setStyleSheet("color:#515151;background-color:#515151;border:0px ;")

        self.text.setReadOnly(True)
        self.setLineEdit(self.text)
        self.setModel(list_widget.model())
        self.setView(list_widget)

        self.text.setText(self._translate("WJ-DepthViewer", "Please select..."))

    def all_selected(self):
        # change state
        if self.state == 0:
            self.state = 1
            for i in range(1, len(self.items)):
                self.box_list[i].setChecked(True)
        else:
            self.state = 0
            for i in range(1, len(self.items)):
                self.box_list[i].setChecked(False)
        self.show_selected()

    def get_selected(self) -> list:
        ret = []
        for i in range(1, len(self.items)):
            if self.box_list[i].isChecked():
                ret.append(self.box_list[i].text())
        return ret

    def show_selected(self):
        """
        show selected items
        """
        self.text.clear()
        ret = '; '.join(self.get_selected())
        self.text.setText(self._translate("WJ-DepthViewer","selected..."))