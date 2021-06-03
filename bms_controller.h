#ifndef BMS_CONTROLLER_H
#define BMS_CONTROLLER_H

#include <QObject>
#include <QCanBus>
#include <QCanBusDevice>
#include <QCanBusDeviceInfo>
#include "../BMS_HY01/bms_def.h"

class QTcpSocket;
class QTcpServer;
class BMS_System;
class QTimer;
class RemoteSystem;
class QSerialPort;
class QModbusRtuSerialSlave;

class CANBUSDevice{
public:
    QString name="";
    bool connected=false;
    QList<int> groupList;
    bool error=false;
    QString errorString="";
    int bitrate;
    QCanBusDevice *dev = nullptr;
    QCanBusDeviceInfo *info = nullptr;
};

class MODBUSDevice{
public:
    QString portName;
    int bitrate;
    bool connected=false;
    QModbusRtuSerialSlave *dev;
};


static QMap<QString, int> cmd_map{{"READ",0},{"DO",1},{"VO",2},{"PORT",3},{"BCU",4}};
static QMap<QString, int> bcu_cmd_map{{"DO",0},{"VO",1},{"AIMAP",2},{"SAVE",3}};
class BMS_Controller : public QObject
{
    Q_OBJECT
public:
    explicit BMS_Controller(QObject *parent = nullptr);

    bool startServer();
    void stopServer();
    void startCANHandler(QString device);
    void stopCANHandler(QString device);
    bool loadConfig();
    bool log(QString message);
    bool isConnected();
    bool isSimulating();
signals:

public slots:
    void handleSocketDataReceived();
    void handleDisconnection();
    void handleNewConnection();
    void handleTimeout();
    void OnCanBusError(QCanBusDevice::CanBusError error);
    void OnCanbusReceived();
    void OnSerialCanRead();

private:
    void prepareModbusRegister();
    void updateModbusRegister();

private:
    QTcpServer *m_server = nullptr;
    //QList<QTcpSocket*> m_clients;
    QList<RemoteSystem*> m_clients;
    bool m_simulator = false;
    BMS_System *m_bmsSystem = nullptr;
    QTimer *mTimer;
    //QList<QCanBusDevice*> m_canbusDevices;
    QList<QString> m_devices;
    QList<QCanBusDeviceInfo> m_canbusDevInfo;
    QList<CANBUSDevice*> m_canbusDevice;
    MODBUSDevice *m_modbusDev = nullptr;
    QSerialPort *m_serialPort= nullptr;
    QString m_logPath = "./log";
    bool m_connected=false;
};

#endif // BMS_CONTROLLER_H
