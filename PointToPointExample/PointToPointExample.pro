#-------------------------------------------------
#
# Project created by QtCreator 2016-02-05T00:34:07
#
#-------------------------------------------------

QT       += core gui

include(../SimpleMotionV2Library/SimpleMotionV2.pri)


greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = PointToPointExample
TEMPLATE = app


SOURCES += main.cpp\
        mw.cpp \
        axis.cpp

HEADERS  += mw.h axis.h

FORMS    += mw.ui

