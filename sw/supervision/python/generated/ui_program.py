# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui/ui_program.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Program(object):
    def setupUi(self, Program):
        Program.setObjectName("Program")
        Program.resize(374, 32)
        self.horizontalLayout = QtWidgets.QHBoxLayout(Program)
        self.horizontalLayout.setContentsMargins(-1, 0, -1, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.icon_label = QtWidgets.QLabel(Program)
        self.icon_label.setObjectName("icon_label")
        self.horizontalLayout.addWidget(self.icon_label)
        self.program_lineedit = QtWidgets.QLineEdit(Program)
        self.program_lineedit.setObjectName("program_lineedit")
        self.horizontalLayout.addWidget(self.program_lineedit)
        self.line = QtWidgets.QFrame(Program)
        self.line.setFrameShape(QtWidgets.QFrame.VLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.horizontalLayout.addWidget(self.line)
        self.run_button = QtWidgets.QToolButton(Program)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/icons/icons/stop.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.run_button.setIcon(icon)
        self.run_button.setObjectName("run_button")
        self.horizontalLayout.addWidget(self.run_button)
        self.remove_button = QtWidgets.QToolButton(Program)
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(":/icons/icons/remove.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.remove_button.setIcon(icon1)
        self.remove_button.setObjectName("remove_button")
        self.horizontalLayout.addWidget(self.remove_button)

        self.retranslateUi(Program)
        QtCore.QMetaObject.connectSlotsByName(Program)

    def retranslateUi(self, Program):
        _translate = QtCore.QCoreApplication.translate
        Program.setWindowTitle(_translate("Program", "Form"))
        self.icon_label.setText(_translate("Program", "..."))
        self.run_button.setText(_translate("Program", "..."))
        self.remove_button.setText(_translate("Program", "..."))
from generated import resources_rc
