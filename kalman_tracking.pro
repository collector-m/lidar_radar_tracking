QT += core
QT -= gui

TARGET = kalman_tracking
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    inc/object.cpp \
    inc/objectsimulator.cpp \
    src/object.cpp \
    src/objectsimulator.cpp \
    visualizer.cpp \
    src/visualizer.cpp \
    inc/tracker.cpp \
    src/tracker.cpp \
    src/utils.cpp

HEADERS += \
    inc/global.h \
    inc/object.h \
    inc/objectsimulator.h \
    visualizer.h \
    inc/visualizer.h \
    inc/tracker.h \
    inc/utils.h

DISTFILES += \
    CMakeLists.txt

