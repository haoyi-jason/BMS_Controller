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
class BMS_StateMachine;

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


static QMap<QString, int> cmd_map{{"SYS",0},{"DO",1},{"VO",2},{"PORT",3},{"BCU",4},{"SVI",5},{"BMU",6},{"SIM",7}};
static QMap<QString, int> bcu_cmd_map{{"DO",0},{"VO",1},{"AIMAP",2},{"SAVE",3}};
static QMap<QString, int> svi_cmd_map{{"AIMAP",0}};
static QMap<QString, int> bmu_cmd_map{{"BV",0},{"BE",1}};
static QMap<QString, int> sim_cmd_map{{"CV",0},{"CT",1},{"SV",2},{"SA",3}};

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
    void terminate();
signals:

public slots:
    void handleSocketDataReceived();
    void handleDisconnection();
    void handleNewConnection();
    void handleTimeout();
    void handleStateMachTimeout();
    void OnCanBusError(QCanBusDevice::CanBusError error);
    void OnCanbusReceived();
    void OnSerialCanRead();

private slots:
    void setBalancingVoltage(ushort v);

private:
    void prepareModbusRegister();
    void updateModbusRegister();
    bool writeFrame(CAN_Packet *p);

private:
    QTcpServer *m_server = nullptr;
    //QList<QTcpSocket*> m_clients;
    QList<RemoteSystem*> m_clients;
    bool m_simulator = false;
    BMS_System *m_bmsSystem = nullptr;
    QTimer *mTimer=nullptr, *mStateTimer = nullptr;
    //QList<QCanBusDevice*> m_canbusDevices;
    QList<QString> m_devices;
    QList<QCanBusDeviceInfo> m_canbusDevInfo;
    QList<CANBUSDevice*> m_canbusDevice;
    MODBUSDevice *m_modbusDev = nullptr;
    QSerialPort *m_serialPort= nullptr;
    QString m_logPath = "./log";
    bool m_connected=false;
    QThread *m_thread = nullptr;
    BMS_StateMachine *m_stateMach = nullptr;
    int m_broadcastCounter=10;
    int m_heartbeatCounter=10;
};


class BMS_StateMachine : public QObject
{
    Q_OBJECT
public:
    enum BMS_Controller_State{
        STATE_NONE,
        STATE_NOT_INITIALIZED,
        STATE_INITIALIZING,
        STATE_INITIALIZED,
        STATE_IDLE,
        STATE_NORMAL,
        STATE_WRITE_FRAME,
        STATE_WAIT_RESP,
        STATE_TERMINATE,
        STATE_TERMINATED,
    };
    Q_ENUM(BMS_Controller_State)

    explicit BMS_StateMachine(QObject *parent = nullptr);

    void setState(BMS_Controller_State state);
    bool isWaitResp();
    void feedFrame(QCanBusFrame f);
    void add_packet(CAN_Packet *p);
    void add_emg_packet(CAN_Packet *p);
    //BMS_Controller_State state();
    CAN_Packet *popPacket();

signals:
    void message(const QString &message);

public slots:

public:
    BMS_Controller_State state = STATE_NONE;
    BMS_Controller_State pendState = STATE_NONE;
    int subState=0;
    QList<CAN_Packet*> m_pendPacket;
    CAN_Packet *m_currPacket = nullptr;
};

#endif // BMS_CONTROLLER_H
