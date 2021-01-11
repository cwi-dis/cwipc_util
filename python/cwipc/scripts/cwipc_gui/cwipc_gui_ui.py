# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'cwipc_gui.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *

from cwipc.opengl import QOpenGLWidget_cwipc


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(977, 658)
        self.actionfirst_item = QAction(MainWindow)
        self.actionfirst_item.setObjectName(u"actionfirst_item")
        self.actionsecond_item = QAction(MainWindow)
        self.actionsecond_item.setObjectName(u"actionsecond_item")
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.openGLWidget = QOpenGLWidget_cwipc(self.centralwidget)
        self.openGLWidget.setObjectName(u"openGLWidget")
        self.openGLWidget.setGeometry(QRect(159, 9, 801, 491))
        self.tabWidget = QTabWidget(self.centralwidget)
        self.tabWidget.setObjectName(u"tabWidget")
        self.tabWidget.setGeometry(QRect(170, 510, 120, 80))
        self.tab = QWidget()
        self.tab.setObjectName(u"tab")
        self.tabWidget.addTab(self.tab, "")
        self.tab_2 = QWidget()
        self.tab_2.setObjectName(u"tab_2")
        self.tabWidget.addTab(self.tab_2, "")
        self.layoutWidget = QWidget(self.centralwidget)
        self.layoutWidget.setObjectName(u"layoutWidget")
        self.layoutWidget.setGeometry(QRect(18, 10, 131, 572))
        self.gridLayout_4 = QGridLayout(self.layoutWidget)
        self.gridLayout_4.setObjectName(u"gridLayout_4")
        self.gridLayout_4.setContentsMargins(0, 0, 0, 0)
        self.gridLayout = QGridLayout()
        self.gridLayout.setObjectName(u"gridLayout")
        self.label = QLabel(self.layoutWidget)
        self.label.setObjectName(u"label")

        self.gridLayout.addWidget(self.label, 0, 0, 1, 1)

        self.camType_autodetect = QPushButton(self.layoutWidget)
        self.camType_autodetect.setObjectName(u"camType_autodetect")

        self.gridLayout.addWidget(self.camType_autodetect, 5, 0, 1, 1)

        self.camType_kinect = QRadioButton(self.layoutWidget)
        self.camTypeButtonGroup = QButtonGroup(MainWindow)
        self.camTypeButtonGroup.setObjectName(u"camTypeButtonGroup")
        self.camTypeButtonGroup.addButton(self.camType_kinect)
        self.camType_kinect.setObjectName(u"camType_kinect")

        self.gridLayout.addWidget(self.camType_kinect, 3, 0, 1, 1)

        self.camType_realsenze = QRadioButton(self.layoutWidget)
        self.camTypeButtonGroup.addButton(self.camType_realsenze)
        self.camType_realsenze.setObjectName(u"camType_realsenze")

        self.gridLayout.addWidget(self.camType_realsenze, 2, 0, 1, 1)

        self.camType_none = QRadioButton(self.layoutWidget)
        self.camTypeButtonGroup.addButton(self.camType_none)
        self.camType_none.setObjectName(u"camType_none")

        self.gridLayout.addWidget(self.camType_none, 1, 0, 1, 1)

        self.radioButton = QRadioButton(self.layoutWidget)
        self.camTypeButtonGroup.addButton(self.radioButton)
        self.radioButton.setObjectName(u"radioButton")

        self.gridLayout.addWidget(self.radioButton, 4, 0, 1, 1)


        self.gridLayout_4.addLayout(self.gridLayout, 0, 0, 1, 1)

        self.gridLayout_2 = QGridLayout()
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.cameras = QListWidget(self.layoutWidget)
        self.cameras.setObjectName(u"cameras")

        self.gridLayout_2.addWidget(self.cameras, 1, 0, 1, 1)

        self.label_2 = QLabel(self.layoutWidget)
        self.label_2.setObjectName(u"label_2")

        self.gridLayout_2.addWidget(self.label_2, 0, 0, 1, 1)

        self.color_by_camera = QCheckBox(self.layoutWidget)
        self.color_by_camera.setObjectName(u"color_by_camera")

        self.gridLayout_2.addWidget(self.color_by_camera, 2, 0, 1, 1)


        self.gridLayout_4.addLayout(self.gridLayout_2, 1, 0, 1, 1)

        self.gridLayout_3 = QGridLayout()
        self.gridLayout_3.setObjectName(u"gridLayout_3")
        self.label_3 = QLabel(self.layoutWidget)
        self.label_3.setObjectName(u"label_3")

        self.gridLayout_3.addWidget(self.label_3, 0, 0, 1, 2)

        self.label_4 = QLabel(self.layoutWidget)
        self.label_4.setObjectName(u"label_4")

        self.gridLayout_3.addWidget(self.label_4, 1, 0, 1, 1)

        self.bbox_near = QSpinBox(self.layoutWidget)
        self.bbox_near.setObjectName(u"bbox_near")
        self.bbox_near.setMaximum(1000)
        self.bbox_near.setSingleStep(0)
        self.bbox_near.setStepType(QAbstractSpinBox.AdaptiveDecimalStepType)
        self.bbox_near.setDisplayIntegerBase(10)

        self.gridLayout_3.addWidget(self.bbox_near, 1, 1, 1, 1)

        self.label_5 = QLabel(self.layoutWidget)
        self.label_5.setObjectName(u"label_5")

        self.gridLayout_3.addWidget(self.label_5, 2, 0, 1, 1)

        self.bbox_far = QSpinBox(self.layoutWidget)
        self.bbox_far.setObjectName(u"bbox_far")
        self.bbox_far.setMaximum(1000)
        self.bbox_far.setSingleStep(0)
        self.bbox_far.setStepType(QAbstractSpinBox.AdaptiveDecimalStepType)
        self.bbox_far.setDisplayIntegerBase(10)

        self.gridLayout_3.addWidget(self.bbox_far, 2, 1, 1, 1)

        self.label_6 = QLabel(self.layoutWidget)
        self.label_6.setObjectName(u"label_6")

        self.gridLayout_3.addWidget(self.label_6, 3, 0, 1, 1)

        self.bbox_high = QSpinBox(self.layoutWidget)
        self.bbox_high.setObjectName(u"bbox_high")
        self.bbox_high.setMaximum(1000)
        self.bbox_high.setSingleStep(0)
        self.bbox_high.setStepType(QAbstractSpinBox.AdaptiveDecimalStepType)
        self.bbox_high.setDisplayIntegerBase(10)

        self.gridLayout_3.addWidget(self.bbox_high, 3, 1, 1, 1)

        self.label_7 = QLabel(self.layoutWidget)
        self.label_7.setObjectName(u"label_7")

        self.gridLayout_3.addWidget(self.label_7, 4, 0, 1, 1)

        self.bbox_low = QSpinBox(self.layoutWidget)
        self.bbox_low.setObjectName(u"bbox_low")
        self.bbox_low.setMaximum(1000)
        self.bbox_low.setSingleStep(0)
        self.bbox_low.setStepType(QAbstractSpinBox.AdaptiveDecimalStepType)
        self.bbox_low.setDisplayIntegerBase(10)

        self.gridLayout_3.addWidget(self.bbox_low, 4, 1, 1, 1)

        self.bbox_show = QCheckBox(self.layoutWidget)
        self.bbox_show.setObjectName(u"bbox_show")

        self.gridLayout_3.addWidget(self.bbox_show, 5, 0, 1, 2)

        self.bbox_apply = QCheckBox(self.layoutWidget)
        self.bbox_apply.setObjectName(u"bbox_apply")

        self.gridLayout_3.addWidget(self.bbox_apply, 6, 0, 1, 2)


        self.gridLayout_4.addLayout(self.gridLayout_3, 2, 0, 1, 1)

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 977, 22))
        self.menulabel_here = QMenu(self.menubar)
        self.menulabel_here.setObjectName(u"menulabel_here")
        self.menuSecond_menu = QMenu(self.menubar)
        self.menuSecond_menu.setObjectName(u"menuSecond_menu")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.menubar.addAction(self.menulabel_here.menuAction())
        self.menubar.addAction(self.menuSecond_menu.menuAction())
        self.menuSecond_menu.addAction(self.actionfirst_item)
        self.menuSecond_menu.addAction(self.actionsecond_item)

        self.retranslateUi(MainWindow)
        self.camType_autodetect.clicked.connect(MainWindow.camtype_DoDetect)
        self.radioButton.clicked.connect(MainWindow.camtype_DoSelect)
        self.camType_kinect.clicked.connect(MainWindow.camtype_DoSelect)
        self.camType_realsenze.clicked.connect(MainWindow.camtype_DoSelect)
        self.camType_none.clicked.connect(MainWindow.camtype_DoSelect)

        self.tabWidget.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.actionfirst_item.setText(QCoreApplication.translate("MainWindow", u"first item", None))
        self.actionsecond_item.setText(QCoreApplication.translate("MainWindow", u"second item", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), QCoreApplication.translate("MainWindow", u"Tab 1", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), QCoreApplication.translate("MainWindow", u"Tab 2", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"Camera Type:", None))
        self.camType_autodetect.setText(QCoreApplication.translate("MainWindow", u"Detect", None))
        self.camType_kinect.setText(QCoreApplication.translate("MainWindow", u"Kinect", None))
        self.camType_realsenze.setText(QCoreApplication.translate("MainWindow", u"RealSense", None))
        self.camType_none.setText(QCoreApplication.translate("MainWindow", u"none", None))
        self.radioButton.setText(QCoreApplication.translate("MainWindow", u"Synthetic", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"Cameras:", None))
        self.color_by_camera.setText(QCoreApplication.translate("MainWindow", u"Color by Camera", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"Bounding box:", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"Near", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"Far", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"High", None))
        self.label_7.setText(QCoreApplication.translate("MainWindow", u"Low", None))
        self.bbox_show.setText(QCoreApplication.translate("MainWindow", u"Show bbox", None))
        self.bbox_apply.setText(QCoreApplication.translate("MainWindow", u"Apply bbox", None))
        self.menulabel_here.setTitle(QCoreApplication.translate("MainWindow", u"First Menu", None))
        self.menuSecond_menu.setTitle(QCoreApplication.translate("MainWindow", u"Second menu", None))
    # retranslateUi

