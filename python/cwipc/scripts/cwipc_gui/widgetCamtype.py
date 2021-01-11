# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'widgetCamtype.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Ui_SelectCamType(object):
    def setupUi(self, SelectCamType):
        if not SelectCamType.objectName():
            SelectCamType.setObjectName(u"SelectCamType")
        SelectCamType.resize(400, 300)
        self.layoutWidget = QWidget(SelectCamType)
        self.layoutWidget.setObjectName(u"layoutWidget")
        self.layoutWidget.setGeometry(QRect(0, 0, 98, 138))
        self.gridLayout = QGridLayout(self.layoutWidget)
        self.gridLayout.setObjectName(u"gridLayout")
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.camType_realsense = QRadioButton(self.layoutWidget)
        self.camType_realsense.setObjectName(u"camType_realsense")

        self.gridLayout.addWidget(self.camType_realsense, 2, 0, 1, 1)

        self.camType_kinect = QRadioButton(self.layoutWidget)
        self.camType_kinect.setObjectName(u"camType_kinect")

        self.gridLayout.addWidget(self.camType_kinect, 3, 0, 1, 1)

        self.camType_none = QRadioButton(self.layoutWidget)
        self.camType_none.setObjectName(u"camType_none")

        self.gridLayout.addWidget(self.camType_none, 1, 0, 1, 1)

        self.radioButton = QRadioButton(self.layoutWidget)
        self.radioButton.setObjectName(u"radioButton")

        self.gridLayout.addWidget(self.radioButton, 4, 0, 1, 1)

        self.label = QLabel(self.layoutWidget)
        self.label.setObjectName(u"label")

        self.gridLayout.addWidget(self.label, 0, 0, 1, 1)

        self.camType_autodetect = QPushButton(self.layoutWidget)
        self.camType_autodetect.setObjectName(u"camType_autodetect")

        self.gridLayout.addWidget(self.camType_autodetect, 5, 0, 1, 1)


        self.retranslateUi(SelectCamType)
        self.camType_autodetect.clicked.connect(SelectCamType.sdo_autodetect)

        QMetaObject.connectSlotsByName(SelectCamType)
    # setupUi

    def retranslateUi(self, SelectCamType):
        SelectCamType.setWindowTitle(QCoreApplication.translate("SelectCamType", u"Form", None))
        self.camType_realsense.setText(QCoreApplication.translate("SelectCamType", u"RealSense", None))
        self.camType_kinect.setText(QCoreApplication.translate("SelectCamType", u"Kinect", None))
        self.camType_none.setText(QCoreApplication.translate("SelectCamType", u"none", None))
        self.radioButton.setText(QCoreApplication.translate("SelectCamType", u"Synthetic", None))
        self.label.setText(QCoreApplication.translate("SelectCamType", u"Camera Type:", None))
        self.camType_autodetect.setText(QCoreApplication.translate("SelectCamType", u"Detect", None))
    # retranslateUi

