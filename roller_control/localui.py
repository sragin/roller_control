# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'localui.ui'
##
## Created by: Qt User Interface Compiler version 6.5.3
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtWidgets import (QApplication, QDialog, QGridLayout, QGroupBox,
    QHBoxLayout, QPushButton, QRadioButton, QSizePolicy,
    QSpacerItem, QVBoxLayout, QWidget)

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        if not Dialog.objectName():
            Dialog.setObjectName(u"Dialog")
        Dialog.resize(1293, 620)
        sizePolicy = QSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(Dialog.sizePolicy().hasHeightForWidth())
        Dialog.setSizePolicy(sizePolicy)
        Dialog.setMinimumSize(QSize(800, 620))
        self.horizontalLayout_10 = QHBoxLayout(Dialog)
        self.horizontalLayout_10.setObjectName(u"horizontalLayout_10")
        self.webEngineView = QWebEngineView(Dialog)
        self.webEngineView.setObjectName(u"webEngineView")
        self.webEngineView.setMinimumSize(QSize(800, 600))
        self.webEngineView.setUrl(QUrl(u"about:blank"))

        self.horizontalLayout_10.addWidget(self.webEngineView)

        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.radioButtonManual = QRadioButton(Dialog)
        self.radioButtonManual.setObjectName(u"radioButtonManual")
        sizePolicy.setHeightForWidth(self.radioButtonManual.sizePolicy().hasHeightForWidth())
        self.radioButtonManual.setSizePolicy(sizePolicy)
        font = QFont()
        font.setFamilies([u"DejaVu Sans Mono"])
        font.setPointSize(14)
        self.radioButtonManual.setFont(font)
        self.radioButtonManual.setStyleSheet(u"QRadioButton::indicator\n"
"{\n"
"	width : 30px;\n"
"	height : 30px;\n"
"}")
        self.radioButtonManual.setChecked(True)

        self.horizontalLayout.addWidget(self.radioButtonManual)

        self.radioButtonAuto = QRadioButton(Dialog)
        self.radioButtonAuto.setObjectName(u"radioButtonAuto")
        sizePolicy.setHeightForWidth(self.radioButtonAuto.sizePolicy().hasHeightForWidth())
        self.radioButtonAuto.setSizePolicy(sizePolicy)
        self.radioButtonAuto.setFont(font)
        self.radioButtonAuto.setStyleSheet(u"QRadioButton::indicator\n"
"{\n"
"	width : 30px;\n"
"	height : 30px;\n"
"}")

        self.horizontalLayout.addWidget(self.radioButtonAuto)

        self.horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer)


        self.verticalLayout_3.addLayout(self.horizontalLayout)

        self.horizontalLayout_11 = QHBoxLayout()
        self.horizontalLayout_11.setObjectName(u"horizontalLayout_11")
        self.groupBox = QGroupBox(Dialog)
        self.groupBox.setObjectName(u"groupBox")
        font1 = QFont()
        font1.setFamilies([u"DejaVu Sans Mono"])
        font1.setPointSize(16)
        self.groupBox.setFont(font1)
        self.horizontalLayout_4 = QHBoxLayout(self.groupBox)
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.pushButtonLoadPathfile = QPushButton(self.groupBox)
        self.pushButtonLoadPathfile.setObjectName(u"pushButtonLoadPathfile")
        sizePolicy.setHeightForWidth(self.pushButtonLoadPathfile.sizePolicy().hasHeightForWidth())
        self.pushButtonLoadPathfile.setSizePolicy(sizePolicy)
        self.pushButtonLoadPathfile.setFont(font)

        self.horizontalLayout_4.addWidget(self.pushButtonLoadPathfile)

        self.pushButtonPlanPath = QPushButton(self.groupBox)
        self.pushButtonPlanPath.setObjectName(u"pushButtonPlanPath")
        sizePolicy.setHeightForWidth(self.pushButtonPlanPath.sizePolicy().hasHeightForWidth())
        self.pushButtonPlanPath.setSizePolicy(sizePolicy)
        self.pushButtonPlanPath.setFont(font)

        self.horizontalLayout_4.addWidget(self.pushButtonPlanPath)

        self.pushButtonPlanTask = QPushButton(self.groupBox)
        self.pushButtonPlanTask.setObjectName(u"pushButtonPlanTask")
        sizePolicy.setHeightForWidth(self.pushButtonPlanTask.sizePolicy().hasHeightForWidth())
        self.pushButtonPlanTask.setSizePolicy(sizePolicy)
        self.pushButtonPlanTask.setFont(font)

        self.horizontalLayout_4.addWidget(self.pushButtonPlanTask)


        self.horizontalLayout_11.addWidget(self.groupBox)


        self.verticalLayout_3.addLayout(self.horizontalLayout_11)

        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.groupBox_2 = QGroupBox(Dialog)
        self.groupBox_2.setObjectName(u"groupBox_2")
        self.groupBox_2.setFont(font1)
        self.horizontalLayout_5 = QHBoxLayout(self.groupBox_2)
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.pushButtonRepeat = QPushButton(self.groupBox_2)
        self.pushButtonRepeat.setObjectName(u"pushButtonRepeat")
        sizePolicy.setHeightForWidth(self.pushButtonRepeat.sizePolicy().hasHeightForWidth())
        self.pushButtonRepeat.setSizePolicy(sizePolicy)
        self.pushButtonRepeat.setFont(font)
        self.pushButtonRepeat.setCheckable(True)

        self.horizontalLayout_5.addWidget(self.pushButtonRepeat)

        self.pushButtonStartTask = QPushButton(self.groupBox_2)
        self.pushButtonStartTask.setObjectName(u"pushButtonStartTask")
        sizePolicy.setHeightForWidth(self.pushButtonStartTask.sizePolicy().hasHeightForWidth())
        self.pushButtonStartTask.setSizePolicy(sizePolicy)
        self.pushButtonStartTask.setFont(font)

        self.horizontalLayout_5.addWidget(self.pushButtonStartTask)

        self.pushButtonStop = QPushButton(self.groupBox_2)
        self.pushButtonStop.setObjectName(u"pushButtonStop")
        sizePolicy.setHeightForWidth(self.pushButtonStop.sizePolicy().hasHeightForWidth())
        self.pushButtonStop.setSizePolicy(sizePolicy)
        self.pushButtonStop.setFont(font)

        self.horizontalLayout_5.addWidget(self.pushButtonStop)

        self.pushButtonEStop = QPushButton(self.groupBox_2)
        self.pushButtonEStop.setObjectName(u"pushButtonEStop")
        sizePolicy.setHeightForWidth(self.pushButtonEStop.sizePolicy().hasHeightForWidth())
        self.pushButtonEStop.setSizePolicy(sizePolicy)
        self.pushButtonEStop.setFont(font)

        self.horizontalLayout_5.addWidget(self.pushButtonEStop)


        self.horizontalLayout_2.addWidget(self.groupBox_2)


        self.verticalLayout_3.addLayout(self.horizontalLayout_2)

        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.groupBox_6 = QGroupBox(Dialog)
        self.groupBox_6.setObjectName(u"groupBox_6")
        self.groupBox_6.setFont(font1)
        self.gridLayout = QGridLayout(self.groupBox_6)
        self.gridLayout.setObjectName(u"gridLayout")
        self.radioButtonTravelRamp = QRadioButton(self.groupBox_6)
        self.radioButtonTravelRamp.setObjectName(u"radioButtonTravelRamp")
        self.radioButtonTravelRamp.setFont(font)
        self.radioButtonTravelRamp.setStyleSheet(u"QRadioButton::indicator\n"
"{\n"
"	width : 24px;\n"
"	height : 24px;\n"
"}")

        self.gridLayout.addWidget(self.radioButtonTravelRamp, 0, 0, 1, 1)

        self.radioButtonTravelTurtle = QRadioButton(self.groupBox_6)
        self.radioButtonTravelTurtle.setObjectName(u"radioButtonTravelTurtle")
        self.radioButtonTravelTurtle.setFont(font)
        self.radioButtonTravelTurtle.setStyleSheet(u"QRadioButton::indicator\n"
"{\n"
"	width : 24px;\n"
"	height : 24px;\n"
"}")
        self.radioButtonTravelTurtle.setChecked(True)

        self.gridLayout.addWidget(self.radioButtonTravelTurtle, 0, 1, 1, 1)

        self.radioButtonTravelForwardUphill = QRadioButton(self.groupBox_6)
        self.radioButtonTravelForwardUphill.setObjectName(u"radioButtonTravelForwardUphill")
        self.radioButtonTravelForwardUphill.setFont(font)
        self.radioButtonTravelForwardUphill.setStyleSheet(u"QRadioButton::indicator\n"
"{\n"
"	width : 24px;\n"
"	height : 24px;\n"
"}")

        self.gridLayout.addWidget(self.radioButtonTravelForwardUphill, 1, 0, 1, 1)

        self.radioButtonTravelReverseUphill = QRadioButton(self.groupBox_6)
        self.radioButtonTravelReverseUphill.setObjectName(u"radioButtonTravelReverseUphill")
        self.radioButtonTravelReverseUphill.setFont(font)
        self.radioButtonTravelReverseUphill.setStyleSheet(u"QRadioButton::indicator\n"
"{\n"
"	width : 24px;\n"
"	height : 24px;\n"
"}")

        self.gridLayout.addWidget(self.radioButtonTravelReverseUphill, 1, 1, 1, 1)

        self.radioButtonTravelRabbit = QRadioButton(self.groupBox_6)
        self.radioButtonTravelRabbit.setObjectName(u"radioButtonTravelRabbit")
        self.radioButtonTravelRabbit.setFont(font)
        self.radioButtonTravelRabbit.setStyleSheet(u"QRadioButton::indicator\n"
"{\n"
"	width : 24px;\n"
"	height : 24px;\n"
"}")

        self.gridLayout.addWidget(self.radioButtonTravelRabbit, 2, 0, 1, 1)


        self.horizontalLayout_3.addWidget(self.groupBox_6)


        self.verticalLayout_3.addLayout(self.horizontalLayout_3)

        self.horizontalLayout_6 = QHBoxLayout()
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.groupBox_3 = QGroupBox(Dialog)
        self.groupBox_3.setObjectName(u"groupBox_3")
        self.groupBox_3.setFont(font1)
        self.horizontalLayout_7 = QHBoxLayout(self.groupBox_3)
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.radioButtonVibrationOff = QRadioButton(self.groupBox_3)
        self.radioButtonVibrationOff.setObjectName(u"radioButtonVibrationOff")
        self.radioButtonVibrationOff.setFont(font)
        self.radioButtonVibrationOff.setStyleSheet(u"QRadioButton::indicator\n"
"{\n"
"	width : 30px;\n"
"	height : 30px;\n"
"}")
        self.radioButtonVibrationOff.setChecked(True)

        self.horizontalLayout_7.addWidget(self.radioButtonVibrationOff)

        self.radioButtonVibrationLow = QRadioButton(self.groupBox_3)
        self.radioButtonVibrationLow.setObjectName(u"radioButtonVibrationLow")
        self.radioButtonVibrationLow.setFont(font)
        self.radioButtonVibrationLow.setStyleSheet(u"QRadioButton::indicator\n"
"{\n"
"	width : 30px;\n"
"	height : 30px;\n"
"}")

        self.horizontalLayout_7.addWidget(self.radioButtonVibrationLow)

        self.radioButtonVibrationHigh = QRadioButton(self.groupBox_3)
        self.radioButtonVibrationHigh.setObjectName(u"radioButtonVibrationHigh")
        self.radioButtonVibrationHigh.setStyleSheet(u"QRadioButton::indicator\n"
"{\n"
"	width : 30px;\n"
"	height : 30px;\n"
"}")

        self.horizontalLayout_7.addWidget(self.radioButtonVibrationHigh)

        self.pushButtonVibrationON = QPushButton(self.groupBox_3)
        self.pushButtonVibrationON.setObjectName(u"pushButtonVibrationON")

        self.horizontalLayout_7.addWidget(self.pushButtonVibrationON)


        self.horizontalLayout_6.addWidget(self.groupBox_3)

        self.groupBox_4 = QGroupBox(Dialog)
        self.groupBox_4.setObjectName(u"groupBox_4")
        self.groupBox_4.setFont(font1)
        self.horizontalLayout_8 = QHBoxLayout(self.groupBox_4)
        self.horizontalLayout_8.setObjectName(u"horizontalLayout_8")
        self.pushButtonHornOn = QPushButton(self.groupBox_4)
        self.pushButtonHornOn.setObjectName(u"pushButtonHornOn")

        self.horizontalLayout_8.addWidget(self.pushButtonHornOn)


        self.horizontalLayout_6.addWidget(self.groupBox_4)


        self.verticalLayout_3.addLayout(self.horizontalLayout_6)

        self.verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_3.addItem(self.verticalSpacer)


        self.horizontalLayout_10.addLayout(self.verticalLayout_3)


        self.retranslateUi(Dialog)

        QMetaObject.connectSlotsByName(Dialog)
    # setupUi

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(QCoreApplication.translate("Dialog", u"Roller Control GUI", None))
        self.radioButtonManual.setText(QCoreApplication.translate("Dialog", u"Manual", None))
        self.radioButtonAuto.setText(QCoreApplication.translate("Dialog", u"Automatic", None))
        self.groupBox.setTitle(QCoreApplication.translate("Dialog", u"Planning", None))
        self.pushButtonLoadPathfile.setText(QCoreApplication.translate("Dialog", u"Load Json\n"
"Pathfile", None))
        self.pushButtonPlanPath.setText(QCoreApplication.translate("Dialog", u"Plan Path", None))
        self.pushButtonPlanTask.setText(QCoreApplication.translate("Dialog", u"Plan Task", None))
        self.groupBox_2.setTitle(QCoreApplication.translate("Dialog", u"Controlling", None))
        self.pushButtonRepeat.setText(QCoreApplication.translate("Dialog", u"Auto\n"
"Repeat", None))
        self.pushButtonStartTask.setText(QCoreApplication.translate("Dialog", u"Start", None))
        self.pushButtonStop.setText(QCoreApplication.translate("Dialog", u"Stop", None))
        self.pushButtonEStop.setText(QCoreApplication.translate("Dialog", u"E-STOP", None))
        self.groupBox_6.setTitle(QCoreApplication.translate("Dialog", u"Travel Mode", None))
        self.radioButtonTravelRamp.setText(QCoreApplication.translate("Dialog", u"Ramp", None))
        self.radioButtonTravelTurtle.setText(QCoreApplication.translate("Dialog", u"Turtle", None))
        self.radioButtonTravelForwardUphill.setText(QCoreApplication.translate("Dialog", u"Forward uphill", None))
        self.radioButtonTravelReverseUphill.setText(QCoreApplication.translate("Dialog", u"Reverse uphill", None))
        self.radioButtonTravelRabbit.setText(QCoreApplication.translate("Dialog", u"Rabbit", None))
        self.groupBox_3.setTitle(QCoreApplication.translate("Dialog", u"Vibration", None))
        self.radioButtonVibrationOff.setText(QCoreApplication.translate("Dialog", u"Off", None))
        self.radioButtonVibrationLow.setText(QCoreApplication.translate("Dialog", u"Low", None))
        self.radioButtonVibrationHigh.setText(QCoreApplication.translate("Dialog", u"High", None))
        self.pushButtonVibrationON.setText(QCoreApplication.translate("Dialog", u"ON", None))
        self.groupBox_4.setTitle(QCoreApplication.translate("Dialog", u"Horn", None))
        self.pushButtonHornOn.setText(QCoreApplication.translate("Dialog", u"ON", None))
    # retranslateUi

