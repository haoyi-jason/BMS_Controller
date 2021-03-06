QT -= gui
QT += core serialbus network serialport

CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        ../BMS_HY01/bms_bcudevice.cpp \
        ../BMS_HY01/bms_bmudevice.cpp \
        ../BMS_HY01/bms_def.cpp \
        ../BMS_HY01/bms_localconfig.cpp \
        ../BMS_HY01/bms_stack.cpp \
        ../BMS_HY01/bms_svidevice.cpp \
        ../BMS_HY01/bms_system.cpp \
        ../BMS_HY01/secs.cpp \
        bms_controller.cpp \
        main.cpp

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

HEADERS += \
    ../BMS_HY01/bms_bcudevice.h \
    ../BMS_HY01/bms_bmudevice.h \
    ../BMS_HY01/bms_def.h \
    ../BMS_HY01/bms_localconfig.h \
    ../BMS_HY01/bms_stack.h \
    ../BMS_HY01/bms_svidevice.h \
    ../BMS_HY01/bms_system.h \
    ../BMS_HY01/secs.h \
    bms_controller.h
