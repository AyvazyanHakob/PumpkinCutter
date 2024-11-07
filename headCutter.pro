TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH += $$PWD/3rdParty/opencv/include
LIBS += -L$$PWD/3rdParty/opencv/x64/vc17/lib

LIBS += -lopencv_world4100d

SOURCES += \
        main.cpp

HEADERS += \
    utils.h

DISTFILES += \
    .gitignore
