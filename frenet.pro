TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

HEADERS +=  \
    src/frenet.h \
    src/globalpath.h \
    src/helper/CubicSpline1D.h \
    src/helper/chisquare.h \
    src/helper/CubicSpline2D.h \
    src/helper/matplotlibcpp.h \
    src/common.h \
    src/draw.h \
    src/mapper.h \
    src/measurement_package.h \
    src/quartic_poly.h \
    src/quintic_poly.h \
    src/utils.h \
    src/car.h

SOURCES += \
    src/frenet.cpp \
    src/CubicSpline1D.cpp \
    src/CubicSpline2D.cpp \
    src/globalpath.cpp \
    src/main.cpp \
    src/quartic_poly.cpp \
    src/quintic_poly.cpp \
    src/car.cpp

RESOURCES     = frenet.qrc

INCLUDEPATH += /opt/local/Library/Frameworks/Python.framework/Versions/3.9/include/python3.9
INCLUDEPATH += /opt/local/Library/Frameworks/Python.framework/Versions/3.9/lib/python3.9/site-packages/numpy/core/include
LIBS += -L/opt/local/Library/Frameworks/Python.framework/Versions/3.9/lib -lpython3.9

