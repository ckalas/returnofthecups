# -------------------------------------------------
# Project created by QtCreator 2013-12-18T13:27:44
# -------------------------------------------------
QT -= gui
TARGET = Multi_Dynamixel_Motor_Control
CONFIG += console
CONFIG -= app_bundle
INCLUDEPATH += ../include/
LIBS += -L../lib/ \
    -ldxl

TEMPLATE = app
SOURCES += main.cpp \
    multi_dnmx_motor.cpp
HEADERS += multi_dnmx_motor.h
