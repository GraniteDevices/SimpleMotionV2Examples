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



# Deployment - Automatically Detect and Copy Dependencies to Build Folder

TARGET_CUSTOM_EXT = .exe
DEPLOY_COMMAND = windeployqt

DEPLOY_OPTIONS = "--no-svg --no-system-d3d-compiler --no-opengl --no-angle"

CONFIG( debug, debug|release ) {
    # debug
    DEPLOY_TARGET = $$shell_quote($$shell_path($${OUT_PWD}/debug/$${TARGET}$${TARGET_CUSTOM_EXT}))
    DEPLOY_OPTIONS += "--debug"
} else {
    # release
    DEPLOY_TARGET = $$shell_quote($$shell_path($${OUT_PWD}/release/$${TARGET}$${TARGET_CUSTOM_EXT}))
    DEPLOY_OPTIONS += "--release"
}

# Uncomment the following line to help debug the deploy command when running qmake
message($${DEPLOY_COMMAND} $${DEPLOY_OPTIONS} $${DEPLOY_TARGET})
#message("hello")

QMAKE_POST_LINK = $${DEPLOY_COMMAND} $${DEPLOY_OPTIONS} $${DEPLOY_TARGET}

