#-------------------------------------------------
#
# Project created by QtCreator 2018-07-21T01:24:56
#
#-------------------------------------------------

QT       += core gui
QT       += widgets printsupport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = RRT_STAR
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += /home/haris/Desktop/V-REP_PRO_EDU_V3_5_0_Linux/programming/remoteApi
INCLUDEPATH += /home/haris/Desktop/V-REP_PRO_EDU_V3_5_0_Linux/programming/include
DEFINES += NON_MATLAB_PARSING
DEFINES += MAX_EXT_API_CONNECTIONS=255
DEFINES += DO_NOT_USE_SHARED_MEMORY

SOURCES += \
        main.cpp \
        mainwindow.cpp \
    qcustomplot.cpp \
    rrtnode.cpp \
    kdtree.cpp \
    kdtreenode.cpp

HEADERS += \
        mainwindow.h \
    qcustomplot.h \
    rrtnode.h \
    kdtree.h \
    kdtreenode.h

FORMS += \
        mainwindow.ui

LIBS += -lccd
LIBS += -lfcl
