#-------------------------------------------------
#
# Project created by QtCreator 2016-02-06T01:00:36
#
#-------------------------------------------------

QT       += core gui

include(../SimpleMotionV2Library/SimpleMotionV2.pri)

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = BufferedMotionStreamExample
TEMPLATE = app


SOURCES += main.cpp\
        mw.cpp

HEADERS  += mw.h

FORMS    += mw.ui

RC_FILE = ../icon/graniteicon.rc
