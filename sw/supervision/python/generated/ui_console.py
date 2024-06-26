# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui/ui_console.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Console(object):
    def setupUi(self, Console):
        Console.setObjectName("Console")
        Console.resize(709, 647)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(Console)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.splitter = QtWidgets.QSplitter(Console)
        self.splitter.setOrientation(QtCore.Qt.Horizontal)
        self.splitter.setObjectName("splitter")
        self.console_textedit = QtWidgets.QTextEdit(self.splitter)
        self.console_textedit.setReadOnly(True)
        self.console_textedit.setObjectName("console_textedit")
        self.filter_widget = QtWidgets.QWidget(self.splitter)
        self.filter_widget.setObjectName("filter_widget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.filter_widget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.label = QtWidgets.QLabel(self.filter_widget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy)
        self.label.setObjectName("label")
        self.verticalLayout.addWidget(self.label)
        self.programs_checkbox = QtWidgets.QCheckBox(self.filter_widget)
        self.programs_checkbox.setChecked(False)
        self.programs_checkbox.setObjectName("programs_checkbox")
        self.verticalLayout.addWidget(self.programs_checkbox)
        self.scrollArea = QtWidgets.QScrollArea(self.filter_widget)
        self.scrollArea.setWidgetResizable(True)
        self.scrollArea.setObjectName("scrollArea")
        self.programs_widget = QtWidgets.QWidget()
        self.programs_widget.setGeometry(QtCore.QRect(0, 0, 180, 525))
        self.programs_widget.setObjectName("programs_widget")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.programs_widget)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        spacerItem = QtWidgets.QSpacerItem(1, 167, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_4.addItem(spacerItem)
        self.scrollArea.setWidget(self.programs_widget)
        self.verticalLayout.addWidget(self.scrollArea)
        self.verticalLayout_2.addWidget(self.splitter)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_3 = QtWidgets.QLabel(Console)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_3.sizePolicy().hasHeightForWidth())
        self.label_3.setSizePolicy(sizePolicy)
        self.label_3.setObjectName("label_3")
        self.horizontalLayout_2.addWidget(self.label_3)
        self.log_level_slider = QtWidgets.QSlider(Console)
        self.log_level_slider.setMaximumSize(QtCore.QSize(100, 16777215))
        self.log_level_slider.setMaximum(3)
        self.log_level_slider.setPageStep(1)
        self.log_level_slider.setProperty("value", 3)
        self.log_level_slider.setOrientation(QtCore.Qt.Horizontal)
        self.log_level_slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.log_level_slider.setTickInterval(1)
        self.log_level_slider.setObjectName("log_level_slider")
        self.horizontalLayout_2.addWidget(self.log_level_slider)
        self.log_level_label = QtWidgets.QLabel(Console)
        self.log_level_label.setObjectName("log_level_label")
        self.horizontalLayout_2.addWidget(self.log_level_label)
        spacerItem1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem1)
        self.clear_button = QtWidgets.QPushButton(Console)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/icons/icons/clear.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.clear_button.setIcon(icon)
        self.clear_button.setObjectName("clear_button")
        self.horizontalLayout_2.addWidget(self.clear_button)
        self.horizontalLayout_2.setStretch(1, 1)
        self.horizontalLayout_2.setStretch(3, 2)
        self.verticalLayout_2.addLayout(self.horizontalLayout_2)
        self.verticalLayout_2.setStretch(0, 1)

        self.retranslateUi(Console)
        QtCore.QMetaObject.connectSlotsByName(Console)

    def retranslateUi(self, Console):
        _translate = QtCore.QCoreApplication.translate
        Console.setWindowTitle(_translate("Console", "Form"))
        self.label.setText(_translate("Console", "Filters"))
        self.programs_checkbox.setText(_translate("Console", "Programs"))
        self.label_3.setText(_translate("Console", "Log level:"))
        self.log_level_label.setText(_translate("Console", "All"))
        self.clear_button.setText(_translate("Console", "Clear console"))
from generated import resources_rc
