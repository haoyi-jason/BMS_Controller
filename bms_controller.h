#ifndef BMS_CONTROLLER_H
#define BMS_CONTROLLER_H

#include <QObject>
#include <QCanBus>
#include <QCanBusDevice>
#include <QCanBusDeviceInfo>
#include "../BMS_HY01/bms_def.h"

class QTcpSocket;
class QTcpServer;
class BMS_SystemInfo;
class QTimer;
class RemoteSystem;

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
};


class BMS_Controller : public QObject
{
    Q_OBJECT
public:
    explicit BMS_Controller(QObject *parent = nullptr);

    bool startServer(QString ipAddress, int port);
    void stopServer();
    void startCANHandler(QString device);
    void stopCANHandler(QString device);
    void loadConfig();

signals:

public slots:
    void handleSocketDataReceived();
    void handleDisconnection();
    void handleNewConnection();
    void handleTimeout();
    void OnCanBusError(QCanBusDevice::CanBusError error);
    void OnCanbusReceived();



private:
    QTcpServer *m_server = nullptr;
    //QList<QTcpSocket*> m_clients;
    QList<RemoteSystem*> m_clients;
    bool m_simulator = false;
    BMS_SystemInfo *m_bmsSystem = nullptr;
    QTimer *mTimer;
    QList<QCanBusDevice*> m_canbusDevices;
    QList<QString> m_devices;
    QList<QCanBusDeviceInfo> m_canbusDevInfo;
    QList<CANBUSDevice*> m_canbusDevice;
    MODBUSDevice *m_modbusDev = nullptr;
};

#endif // BMS_CONTROLLER_H
