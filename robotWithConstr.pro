QT += core


TEMPLATE = app
CONFIG += console c++14
CONFIG -= app_bundle

#INCLUDEPATH += $$PWD/../Dokumente/simulator/simulatoren_extern/optimize/nlopt/api
LIBS += -lnlopt

SOURCES += main.cpp \
    robot.cpp \
    costfunction.cpp \
    vectorhelper.cpp \
    constraintneigbour.cpp

HEADERS += \
    robot.h \
    costfunction.h \
    vectorhelper.h \
    constraintneigbour.h
