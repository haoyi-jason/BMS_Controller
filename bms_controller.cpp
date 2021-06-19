#include "bms_controller.h"
#include <QtNetwork>
#include <QtCore>
#include <QCanBus>
#include <QCanBusDevice>
#include <QCanBusDeviceInfo>
#include <QCanBusFrame>
#include <QProcess>
#include <QDateTime>
#include <QSysInfo>
#include<QJsonDocument>
#include<QJsonObject>
#include<QJsonValue>
#include<QJsonArray>
#include <QtSerialPort/QSerialPort>
#include <QModbusRtuSerialSlave>
#include "../BMS_HY01/bms_def.h"
#include "../BMS_HY01/secs.h"
#include "../BMS_HY01/bms_bmudevice.h"
#include "../BMS_HY01/bms_bcudevice.h"
#include "../BMS_HY01/bms_svidevice.h"
#include "../BMS_HY01/bms_stack.h"
#include "../BMS_HY01/bms_system.h"


BMS_Controller::BMS_Controller(QObject *parent) : QObject(parent)
{
    m_devices.append("can0");
    m_devices.append("can1");

    m_server = new QTcpServer(this);

    m_bmsSystem = new BMS_System();

    m_stateMach = new BMS_StateMachine;

    // load from file
    QString path;

    if(QSysInfo::productType().contains("win")){
        path = "./config/local.json";
    }
    else{
       path = QCoreApplication::applicationDirPath()+"/config/local.json";
    }
    qDebug()<<"Current Path:"<<path;

    QFile f(path);
    bool success = false;
    if(f.exists() && f.open(QIODevice::ReadOnly)){
        success = m_bmsSystem->Configuration(f.readAll());
        f.close();
    }
    else{
        log(QString("File local.json not exist"));
        qDebug()<<"No Configuration available 1"<<f.exists();
    }

    if(success){
        // load local interface configuration
        if(this->loadConfig()){
            if(this->isSimulating()){
                log("Start Simulator");
                m_bmsSystem->startSimulator(1000);
                this->m_connected = true;
            }
            else{
                if(this->m_canbusDevice.count()>0){
                    // start can device
                    QString cmd;
                    QProcess *proc = new QProcess();

                    foreach (CANBUSDevice *dev, m_canbusDevice) {
                        cmd = QString("ip link set %1 down").arg(dev->name);
                        proc->start("sh",QStringList()<<"-c"<<cmd);
                        proc->waitForFinished();
                        qDebug()<<"Proc Result:"<<proc->readAll();
                        cmd = QString("ip link set %1 up type can bitrate %2").arg(dev->name).arg(dev->bitrate);
                        proc->start("sh",QStringList()<<"-c"<<cmd);
                        proc->waitForFinished();
                        qDebug()<<"Proc Result:"<<proc->readAll();
                    }
                    proc->close();
                }
                // check if device count match config
                // load canbus device
                QString errorString;
                m_canbusDevInfo = QCanBus::instance()->availableDevices(QStringLiteral("socketcan"),&errorString);

                if(!errorString.isEmpty()){
                    log(QString("CANBUS Error: %1").arg(errorString));
                    qDebug()<<errorString;
                }
                qDebug()<<"Device found:"<<m_canbusDevInfo.size();
                if(m_canbusDevInfo.size() == m_canbusDevice.size()){
                    m_simulator = false;
                    // start can device and register handler
                    foreach (CANBUSDevice *dev, m_canbusDevice) {
                        dev->dev = QCanBus::instance()->createDevice(QStringLiteral("socketcan"),QString(dev->name),&errorString);
                        connect(dev->dev,&QCanBusDevice::errorOccurred,this,&BMS_Controller::OnCanBusError);
                        connect(dev->dev,&QCanBusDevice::framesReceived,this,&BMS_Controller::OnCanbusReceived);
                        dev->dev->connectDevice();
                        dev->connected = true;
                    }
                    this->m_connected = true;
                }
            }

            // start MODBUS Slave
            m_modbusDev->dev = new QModbusRtuSerialSlave();
            m_modbusDev->dev->setConnectionParameter(QModbusDevice::SerialPortNameParameter,m_modbusDev->portName);
            m_modbusDev->dev->setConnectionParameter(QModbusDevice::SerialBaudRateParameter,m_modbusDev->bitrate);
            m_modbusDev->dev->setConnectionParameter(QModbusDevice::SerialDataBitsParameter,QSerialPort::Data8);
            m_modbusDev->dev->setConnectionParameter(QModbusDevice::SerialParityParameter,QSerialPort::NoParity);
            m_modbusDev->dev->setConnectionParameter(QModbusDevice::SerialStopBitsParameter,QSerialPort::OneStop);

            if(m_modbusDev->dev->connectDevice()){
                m_modbusDev->connected = true;
                prepareModbusRegister();
            }
            mTimer = new QTimer();
            connect(mTimer,&QTimer::timeout,this,&BMS_Controller::handleTimeout);
//            mTimer->start(100);

            mStateTimer = new QTimer();
            connect(mStateTimer,&QTimer::timeout,this,&BMS_Controller::handleStateMachTimeout);
            mStateTimer->start(100);

            m_bmsSystem->enableAlarmSystem(true);
            connect(m_bmsSystem,&BMS_System::setBalancingVoltage,this,&BMS_Controller::setBalancingVoltage);
        }
    }

  //  m_bmsSystem->On_BMU_ov(0x10);
}

bool BMS_Controller::startServer()
{
    m_server->listen(QHostAddress(m_bmsSystem->connectionString),m_bmsSystem->connectionPort);
    if(m_server->isListening()){
        connect(m_server,&QTcpServer::newConnection,this,&BMS_Controller::handleNewConnection);
        log("Server Start Successfully\n");
        return true;
    }
    log("Server Start Fail!\n");
    return false;
}

void BMS_Controller::stopServer()
{
    m_server->close();
    log("Server Stopped\n");
}

void BMS_Controller::startCANHandler(QString device)
{

}

void BMS_Controller::stopCANHandler(QString device)
{

}

void BMS_Controller::handleSocketDataReceived()
{
    QTcpSocket *s = (QTcpSocket*)sender();

    if(s->bytesAvailable()){
        QString str = QString(s->readAll());
//        if(str == "READ:CFG"){
//            qDebug()<<"Read Config:";
//            QString path;
//            if(QSysInfo::productType().contains("win")){
//                path = "./config/local.json";
//            }
//            else{
//                path = QCoreApplication::applicationDirPath()+"/config/local.json";
//            }
//            QFile f(path);
//            if(f.exists() && f.open(QIODevice::ReadOnly)){
//                qDebug()<<"Reply config";
//                QByteArray b = f.readAll();
//                f.close();
//                b.insert(0,hsmsParser::genHeader(hsmsParser::BMS_CONFIG,b.size()));
//                s->write(b);
//                foreach (RemoteSystem *sys, m_clients) {
//                    if(sys->socket == s){
//                        sys->configReady = true;
//                    }
//                }
//            }
//        }
        // parse string
        QStringList sl = str.split(":");
        if(sl.size() > 1){
            switch(cmd_map.value(sl[0])){
            case 0: // SYS
                if(sl[1].compare("CFG",Qt::CaseInsensitive)==0){
                    foreach (RemoteSystem *sys, m_clients) {
                        if(sys->socket == s){
                            sys->configReady = true;
                        }
                    }
                }
                else if(sl[1].compare("ALMRST",Qt::CaseInsensitive) == 0){
                    m_bmsSystem->clearAlarm();
                    if(m_bmsSystem->bcu()->digitalOutState(0)==1){
                        CAN_Packet *p = nullptr;
                        if((p = m_bmsSystem->setDigitalOut(0,0)) != nullptr){
                            p->Command |= (0x01 << 12); // bcudevice
                            this->writeFrame(p);
                        }

                    }
                }
                else if(sl[1].compare("INIT_TIME",Qt::CaseInsensitive)==0){
                    qint64 epoch = m_bmsSystem->startTime().toSecsSinceEpoch();
                    QByteArray b;
                    QDataStream ds(&b,QIODevice::WriteOnly);
                    ds << epoch;
                    b.insert(0,hsmsParser::genHeader(hsmsParser::BMS_SYS_DATETIME,b.size()));
                    s->write(b);
                }
                break;
//            case 1:  // DO
//                if(sl.size()==3){
//                   int ch = sl[1].toInt();
//                   int value = sl[2].toInt()==0?0:1;
//                   CAN_Packet *p = this->m_bmsSystem->setDigitalOut(ch,value);
//                   if(p != nullptr){
//                       QCanBusFrame frame;
//                       quint32 id = p->Command | (0x01 << 12);
//                       frame.setFrameId(id);
//                       frame.setPayload(p->data);
//                       frame.setFrameType(QCanBusFrame::DataFrame);
//                       if(m_canbusDevice.size()>0){
//                           if(m_canbusDevice[1]->dev->writeFrame(frame)){
//                               qDebug()<<"Write frame OK";
//                           }
//                           else{
//                               qDebug()<<"Write frame Fail";
//                           }
//                       }

//                   }
//                   //this->m_bmsSystem->flushBCU();
//                }
//                break;
//            case 2: // AO
//                if(sl.size() == 3){
//                    int ch = sl[1].toInt();
//                    int value = sl[2].toInt();
//                    CAN_Packet *p = this->m_bmsSystem->setVoltageSource(ch,value,value!=0);
//                    if(p != nullptr){
//                        QCanBusFrame frame;
//                        quint32 id = p->Command | (0x01 << 12);
//                        frame.setFrameId(id);
//                        frame.setPayload(p->data);
//                        frame.setFrameType(QCanBusFrame::DataFrame);
//                        if(m_canbusDevice.size()>0){
//                            if(m_canbusDevice[1]->dev->writeFrame(frame)){
//                                qDebug()<<"Write frame OK";
//                            }
//                            else{
//                                qDebug()<<"Write frame Fail";
//                            }
//                        }
//                    }
//                }
//                break;
//            case 3: // port
//                if(sl[1] == "OPEN"){
//                    m_serialPort = new QSerialPort();
//                    m_serialPort->setPortName(sl[2]);
//                    m_serialPort->setBaudRate(sl[3].toInt());
//                    if(!m_serialPort->open(QIODevice::ReadWrite)){
//                        m_serialPort->close();
//                        m_serialPort->deleteLater();
//                        m_serialPort = nullptr;
//                    }
//                    else{
//                        connect(m_serialPort,&QSerialPort::readyRead,this,&BMS_Controller::OnSerialCanRead);
//                    }
//                }
//                else if(sl[1] == "CLOSE")
//                {
//                    if(m_serialPort->isOpen()){
//                        m_serialPort->close();
//                        m_serialPort->deleteLater();
//                        m_serialPort = nullptr;
//                    }
//                }
//                else if(sl[1] == "WRITE")
//                {
//                    if(m_serialPort->isOpen()){
//                        m_serialPort->write(sl[2].toUtf8());
//                    }
//                }
//                break;
            case 4: // BCU
                if(sl.size()<2) return;
                if(this->m_bmsSystem->bcu() == nullptr) return;
                switch(bcu_cmd_map.value(sl[1])){
                case 0: // DO
                    if(sl.size() == 4){
                        CAN_Packet *p = this->m_bmsSystem->bcu()->setDigitalOut(sl[2].toInt(),sl[3].toInt()==0?0:1);
                        if(p != nullptr){
                            QCanBusFrame frame;
                            quint32 id = p->Command | (0x01 << 12);
                            frame.setFrameId(id);
                            frame.setPayload(p->data);
                            frame.setFrameType(QCanBusFrame::DataFrame);
                            if(m_canbusDevice.size()>0){
                                if(m_canbusDevice[1]->dev->writeFrame(frame)){
                                    qDebug()<<"Write frame OK";
                                }
                                else{
                                    qDebug()<<"Write frame Fail";
                                }
                            }

                        }
                    }
                    break;
                case 1: // VO
                    if(sl.size() == 4){
                        CAN_Packet *p = this->m_bmsSystem->bcu()->setVoltageSource(sl[2].toInt(),sl[3].toInt(),sl[3].toInt()!=0);
                        if(p != nullptr){
                            QCanBusFrame frame;
                            quint32 id = p->Command | (0x01 << 12);
                            frame.setFrameId(id);
                            frame.setPayload(p->data);
                            frame.setFrameType(QCanBusFrame::DataFrame);
                            if(m_canbusDevice.size()>0){
                                if(m_canbusDevice[1]->dev->writeFrame(frame)){
                                    qDebug()<<"Write frame OK";
                                }
                                else{
                                    qDebug()<<"Write frame Fail";
                                }
                            }
                        }
                    }
                    break;
                case 2: // AIMAP
                    if(sl.size() == 5){
                        CAN_Packet *p = nullptr;
                        switch(sl[3].toInt()){
                        case 0: // raw low
                            p = this->m_bmsSystem->bcu()->setADCRawLow(sl[2].toInt(),sl[4].toInt());
                            break;
                        case 1: // raw high
                            p = this->m_bmsSystem->bcu()->setADCRawHigh(sl[2].toInt(),sl[4].toInt());
                            break;
                        case 2: // eng low
                            p = this->m_bmsSystem->bcu()->setADCEngLow(sl[2].toInt(),sl[4].toFloat());
                            break;
                        case 3: // eng high
                            p = this->m_bmsSystem->bcu()->setADCEngHigh(sl[2].toInt(),sl[4].toFloat());
                            break;
                        }
                        if(p != nullptr){
                            QCanBusFrame frame;
                            quint32 id = p->Command | (0x01 << 12);
                            frame.setFrameId(id);
                            frame.setPayload(p->data);
                            frame.setFrameType(QCanBusFrame::DataFrame);
                            if(m_canbusDevice.size()>0){
                                if(m_canbusDevice[1]->dev->writeFrame(frame)){
                                    qDebug()<<"Write frame OK";
                                }
                                else{
                                    qDebug()<<"Write frame Fail";
                                }
                            }
                        }
                    }
                    break;
                case 3: {// Save param
                    CAN_Packet *p = this->m_bmsSystem->bcu()->saveParam();
                    QCanBusFrame frame;
                    quint32 id = p->Command | (0x01 << 12);;
                    frame.setFrameId(id);
                    frame.setPayload(p->data);
                    frame.setFrameType(QCanBusFrame::DataFrame);
                    if(m_canbusDevice.size()>0){
                        if(m_canbusDevice[1]->dev->writeFrame(frame)){
                            qDebug()<<"Write frame OK";
                        }
                        else{
                            qDebug()<<"Write frame Fail";
                        }
                    }
                    }
                    break;
                }
                break;
            case 5: // SVI
                switch(svi_cmd_map.value(sl[1])){
                case 0: //SVI:AINMAP:CH:OPT:VALUE
                    if(sl.size() == 5){
                        CAN_Packet *p = nullptr;
                        switch(sl[3].toInt()){
                        case 0: // raw low
                            p = this->m_bmsSystem->bcu()->setADCRawLow(sl[2].toInt(),sl[4].toInt());
                            break;
                        case 1: // raw high
                            p = this->m_bmsSystem->bcu()->setADCRawHigh(sl[2].toInt(),sl[4].toInt());
                            break;
                        case 2: // eng low
                            p = this->m_bmsSystem->bcu()->setADCEngLow(sl[2].toInt(),sl[4].toFloat());
                            break;
                        case 3: // eng high
                            p = this->m_bmsSystem->bcu()->setADCEngHigh(sl[2].toInt(),sl[4].toFloat());
                            break;
                        }
                        if(p != nullptr){
                            QCanBusFrame frame;
                            quint32 id = p->Command | (0x1F << 12); // SVI ID always = 0x1F (31D)
                            frame.setFrameId(id);
                            frame.setPayload(p->data);
                            frame.setFrameType(QCanBusFrame::DataFrame);
                            if(m_canbusDevice.size()>0){
                                if(m_canbusDevice[1]->dev->writeFrame(frame)){
                                    qDebug()<<"Write frame OK";
                                }
                                else{
                                    qDebug()<<"Write frame Fail";
                                }
                            }
                        }
                    }
                    break;
                case 1: // SVI:SOHT:1/0, tracking soh
                    if(sl.size()==3){
                        bool en = (sl[2].toInt()==1);
                        foreach (BMS_Stack *s, this->m_bmsSystem->stacks()) {
                            s->sviDevice()->setSOHTracking(en);
                        }
                    }
                    break;
                case 2: // SVI:SSOC:GID:V, set soc to v
                    if(sl.size() == 4){
                        quint8 id = (quint8)sl[2].toInt();
                        float soc = sl[3].toFloat();
                        foreach (BMS_Stack *s, this->m_bmsSystem->stacks()) {
                            if(s->groupID() == GROUP_OF(id)){
                                s->sviDevice()->soc(soc);
                            }
                        }
                    }
                    break;
                }
                break;
            case 6: // BMU
                if(sl.size() == 7){ // enable/disable
                    quint16 bv = (ushort)sl[2].toInt();
                    quint8 hv = (quint8)sl[3].toInt();
                    quint8 ev = (quint8)sl[4].toInt();
                    quint16 on =(quint16)sl[5].toInt();
                    quint16 off =(quint16)sl[6].toInt();
                    CAN_Packet *p = this->m_bmsSystem->setBalancing(bv,hv,ev,on,off);
                    QCanBusFrame frame;
                    quint32 id = p->Command; // broadcasting
                    frame.setFrameId(id);
                    frame.setPayload(p->data);
                    frame.setFrameType(QCanBusFrame::DataFrame);
                    foreach(CANBUSDevice *dev, m_canbusDevice){
                        if(dev->dev->writeFrame(frame)){
                            qDebug()<<"Frame Write OK";
                        }
                        else{
                            qDebug()<<"Frame Write Fail";
                        }
                    }
                }
                break;
            case 7: // SIM
                switch(sim_cmd_map.value(sl[1])){
                case 0: // SIM:CV:BID:CID:V
                    if(sl.size() == 5){
                        foreach(BMS_Stack *s, m_bmsSystem->stacks()){
                            quint8 id = (quint8)sl[2].toInt();
                            if(s->groupID() == GROUP_OF(id) ){
                                s->setSimCellData((quint8)sl[2].toInt(),(quint8)sl[3].toInt(),(quint16)sl[4].toInt());
                            }
                        }
                    }
                    break;
                case 1: // // SIM:CT:BID:CID:V
                    if(sl.size() == 5){
                        foreach(BMS_Stack *s, m_bmsSystem->stacks()){
                            quint8 id = (quint8)sl[2].toInt();
                            if(s->groupID() == GROUP_OF(id) ){
                                s->setSimTempData((quint8)sl[2].toInt(),(quint8)sl[3].toInt(),(quint16)sl[4].toInt());
                            }
                        }
                    }
                    break;
                case 2: // SIM:SV:GID:V, simulate stack voltage
                    if(sl.size() == 4){
                        foreach (BMS_Stack *s, m_bmsSystem->stacks()) {
                            quint8 id = (quint8)sl[2].toInt();
                            if(s->groupID() == (id)){
                                s->sviDevice()->setSimVoltage(sl[3].toInt());
                            }
                        }
                    }
                    break;
                case 3: // SIM:SA:GID:V, simulate stack current
                    if(sl.size() == 4){
                        foreach (BMS_Stack *s, m_bmsSystem->stacks()) {
                            quint8 id = (quint8)sl[2].toInt();
                            if(s->groupID() == (id)){
                                s->sviDevice()->setSimAmpere(sl[3].toInt());
                            }
                        }
                    }
                    break;
                case 4: // SIM_SSOC:GID:V, simlate stack soc
                    if(sl.size() == 4){
                        foreach (BMS_Stack *s, m_bmsSystem->stacks()) {
                            quint8 id = (quint8)sl[2].toInt();
                            if(s->groupID() == (id)){
                                s->sviDevice()->setSimSOC(sl[3].toInt());
                            }
                        }
                    }
                    break;
                }
                break;
            }
        }

    }

}

void BMS_Controller::OnSerialCanRead()
{
    QSerialPort *p = (QSerialPort*)sender();
    QByteArray b = p->readAll();

    foreach (RemoteSystem *sys, m_clients) {
        if(sys->socket != nullptr){
            b.insert(0,hsmsParser::genHeader(hsmsParser::BMS_SERIAL_DATA,b.size()));
            sys->socket->write(b);
        }

    }

}

void BMS_Controller::handleDisconnection()
{
    QTcpSocket *s = (QTcpSocket*)sender();
    foreach (RemoteSystem *sys, m_clients) {
        if(sys->socket == s){
            m_clients.removeOne(sys);
        }
    }

    qDebug()<<"Client disconnect";
}

void BMS_Controller::handleTimeout()
{
    if(m_broadcastCounter > 0){
        m_broadcastCounter--;
    }
    if(m_broadcastCounter == 0){
        m_broadcastCounter = 10;
        QByteArray b;
        QDataStream d(&b,QIODevice::ReadWrite);
        d << m_bmsSystem;
        m_bmsSystem->log(b);
        updateModbusRegister();
        b.insert(0,hsmsParser::genHeader(hsmsParser::BMS_STACK,b.size()));
        foreach (RemoteSystem *sys, m_clients) {

            if(sys->configReady){
                sys->socket->write(b);
            }
        }
    }

    // get lowest voltage and send to bmus




}

void BMS_Controller::handleStateMachTimeout()
{
    switch(m_stateMach->state){
    case BMS_StateMachine::STATE_NONE:
        if(this->isSimulating()){
            m_stateMach->state = BMS_StateMachine::STATE_INITIALIZED;
            m_bmsSystem->bcu()->simulating(true);
        }
        else{
            m_stateMach->state = BMS_StateMachine::STATE_NOT_INITIALIZED;
            m_balancingDelay = 100;
        }
        log("Start State Machine");
        break;
    case BMS_StateMachine::STATE_NOT_INITIALIZED:
        switch(m_stateMach->subState){
        case 0:
            if(m_stateMach->stateDelay > 0)
                m_stateMach->stateDelay--;
            if(m_stateMach->stateDelay == 0)
                m_stateMach->subState = 1;
            break;
        case 1: // start bcu power
            if(m_bmsSystem->bcu() != nullptr){
                CAN_Packet *p = m_bmsSystem->bcu()->setVoltageSource(0,m_bmsSystem->bcu()->vsource_limit(0));
                if(writeFrame(p)){
                    m_stateMach->subState++;
                    m_stateMach->stateDelay = 10;
                    log("Start BCU Voltage source channel 0 success");
                }
                else{
                    log("Start BCU Voltage source channel 0 failed");
                }
                delete p;
            }
            break;
        case 2:
            if(m_stateMach->stateDelay > 0)
                m_stateMach->stateDelay--;
            if(m_stateMach->stateDelay == 0)
                m_stateMach->subState = 3;
            break;
        case 3:
            if(m_bmsSystem->bcu() != nullptr){
                CAN_Packet *p = m_bmsSystem->bcu()->setVoltageSource(1,m_bmsSystem->bcu()->vsource_limit(1));
                if(writeFrame(p)){
                    m_stateMach->subState = 0;
                    m_stateMach->state = BMS_StateMachine::STATE_INITIALIZING;
                    m_stateMach->stateDelay = 10;
                    log("Start BCU Voltage source channel 1 success");
                }
                else{
                    log("Start BCU Voltage source channel 1 failed");
                }
                delete p;
            }
            break;
        }

        break;
    case BMS_StateMachine::STATE_INITIALIZING:
        if(m_stateMach->stateDelay > 0){
            m_stateMach->stateDelay--;
        }
        if(m_stateMach->stateDelay == 0){
            // start bmus
            if(m_bmsSystem != nullptr){
                CAN_Packet *p = m_bmsSystem->startBMUs(true);
                if(writeFrame(p)){
                    m_stateMach->state = BMS_StateMachine::STATE_INITIALIZED;
                    log("Start BMU Devices success");
                }
                else{
                    log("Start BMU Devices failed");
                }
                delete p;
            }
        }
        break;
    case BMS_StateMachine::STATE_INITIALIZED:
        mTimer->start(100);
        m_stateMach->state = BMS_StateMachine::STATE_NORMAL;
        log("BMS System initialized");

        break;
    case BMS_StateMachine::STATE_NORMAL:
        // generate heartbeat packet
        if(m_heartbeatCounter > 0){
            m_heartbeatCounter--;
        }
        if(m_heartbeatCounter == 0){
            m_heartbeatCounter = 10;
            CAN_Packet *p = m_bmsSystem->heartBeat();
            if(writeFrame(p)){
            }
        }
        if(m_ioDelay > 0){
            m_ioDelay--;
        }
        if(m_ioDelay == 0){
            m_ioDelay = 10;
            if(m_bmsSystem->alarmState() != 0){
                // check if bcu's digital output state is set or not
                if(m_bmsSystem->bcu()->digitalOutState(0) == 0){
                    CAN_Packet *p = m_bmsSystem->bcu()->setDigitalOut(m_bmsSystem->warinig_out_id(),1);
                    //p->Command |= (1 <<12);
                    writeFrame(p);
                }
            }
            else if(!m_bmsSystem->warinig_latch()){
                if(m_bmsSystem->bcu()->digitalOutState(0) == 0){
                    CAN_Packet *p = m_bmsSystem->bcu()->setDigitalOut(m_bmsSystem->warinig_out_id(),0);
                    //p->Command |= (1 <<12);
                    writeFrame(p);
                }
            }
        }

        if(m_balancingDelay>0){
            m_balancingDelay--;
        }else{
            writeFrame(m_bmsSystem->broadcastBalancing());
        }

        if(m_stateMach->pendState != BMS_StateMachine::STATE_NONE){
            m_stateMach->state = m_stateMach->pendState;
            m_stateMach->pendState = BMS_StateMachine::STATE_NONE;
        }
        break;
    case BMS_StateMachine::STATE_WRITE_FRAME:
        break;
    case BMS_StateMachine::STATE_WAIT_RESP:
        break;
    case BMS_StateMachine::STATE_TERMINATE:
        switch(m_stateMach->subState){
        case 0: // stop bcu power
            if(m_bmsSystem->bcu() != nullptr){
                CAN_Packet *p = m_bmsSystem->bcu()->setVoltageSource(0,m_bmsSystem->bcu()->vsource_limit(0),false);
                if(writeFrame(p)){
                    m_stateMach->subState++;
                }
                delete p;
            }
            break;
        case 1:
            if(m_bmsSystem->bcu() != nullptr){
                CAN_Packet *p = m_bmsSystem->bcu()->setVoltageSource(0,m_bmsSystem->bcu()->vsource_limit(1),false);
                if(writeFrame(p)){
                    m_stateMach->subState = 0;
                    m_stateMach->state = BMS_StateMachine::STATE_NONE;
                }
                delete p;
            }
            break;
        }

        // stop bcu power
        break;
    case BMS_StateMachine::STATE_TERMINATED:
        // stop servers

        break;
    default:break;
    }
}

void BMS_Controller::terminate()
{
    m_stateMach->pendState = BMS_StateMachine::STATE_TERMINATE;
}

void BMS_Controller::handleNewConnection()
{
    qDebug()<<"New connection request";
    if(m_clients.size() < 2){
        QTcpSocket *client = m_server->nextPendingConnection();
        connect(client,&QTcpSocket::readyRead,this,&BMS_Controller::handleSocketDataReceived);
        connect(client,&QTcpSocket::disconnected,this,&BMS_Controller::handleDisconnection);
        RemoteSystem *sys = new RemoteSystem();
        sys->configReady = false;
        sys->socket = client;

        m_clients.append(sys);
    }


}

void BMS_Controller::OnCanBusError(QCanBusDevice::CanBusError error)
{
    QCanBusDevice *dev = (QCanBusDevice*)sender();
    foreach(CANBUSDevice *d, m_canbusDevice){
        if(d->dev == dev){
            qDebug()<<"Canbus Error:";
        }
    }
}

void BMS_Controller::OnCanbusReceived()
{
    QCanBusDevice *dev = (QCanBusDevice*)sender();
    foreach(CANBUSDevice *d, m_canbusDevice){
        if(d->dev == dev){
            while(d->dev->framesAvailable()){
                QCanBusFrame f= d->dev->readFrame();
                //qDebug()<<"Frame Received:";
                if(f.frameType() == QCanBusFrame::DataFrame){
                    m_bmsSystem->feedData(f.frameId(),f.payload());
                }
            }
        }
    }
}

bool BMS_Controller::loadConfig()
{
    // load from file
    QString path;
    if(QSysInfo::productType().contains("win")){
        path = "./config/interface.json";
    }
    else{
        path = QCoreApplication::applicationDirPath()+"/config/interface.json";
    }
    QFile f(path);
    bool success = false;
    if(f.exists() && f.open(QIODevice::ReadOnly)){
        QJsonParseError e;
        QJsonDocument d = QJsonDocument::fromJson(f.readAll());
        f.close();
        if(d.isNull()){
            qDebug()<<"Wrong config file";
            return false;
        }
        QJsonObject obj = d.object();
        if(obj.contains("canbus")){
            QJsonArray o = obj["canbus"].toArray();
            for(auto v:o){
                QJsonObject q = v.toObject();
                CANBUSDevice *dev = new CANBUSDevice();
                dev->name = q["name"].toString();
                dev->bitrate = q["bitrate"].toInt();
                QJsonArray a = q["groups"].toArray();
                for(int i=0;i<a.size();i++){
                    dev->groupList.append(a[i].toInt());
                }
                this->m_canbusDevice.append(dev);
            }
        }

        if(obj.contains("mbslave")){
            QJsonObject o = obj["mbslave"].toObject();
            this->m_modbusDev = new MODBUSDevice();
            this->m_modbusDev->bitrate = o["bitrate"].toInt();
            this->m_modbusDev->portName = o["port"].toString();
        }
        if(obj.contains("config")){
            QJsonObject o = obj["config"].toObject();
            if(QSysInfo::productType().contains("win")){
                this->m_logPath = o["log_path"].toString();
            }
            else{
                this->m_logPath = QCoreApplication::applicationDirPath()+o["log_path"].toString();
            }
            // check if folder presents
            if(!QDir(this->m_logPath).exists()){
                QDir().mkdir(this->m_logPath);
            }
        }
        if(obj.contains("simulate")){
            this->m_simulator = obj["simulate"].toBool();
        }
        return true;
    }
    else{
        qDebug()<<"No Configuration available 2";
        return false;
    }
    return false;
}

bool BMS_Controller::log(QString message)
{

    QString path =QString("%1/log-%2.txt").arg(this->m_logPath).arg(QDateTime::currentDateTime().toString("yyyyMMdd"));
    QFile f(path);
//    QString msg = QString("%1:%2").arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss")).arg(message);
    QString logText = QString("%1:%2\n").arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss:")).arg(message);
    if(f.open(QIODevice::WriteOnly | QIODevice::Append)){
        f.write(logText.toUtf8());
        f.close();
        return true;
    }
    f.close();
    return false;
}

bool BMS_Controller::isConnected()
{
    return m_connected;
}

bool BMS_Controller::isSimulating()
{
    return m_simulator;
}

void BMS_Controller::prepareModbusRegister()
{
    QModbusDataUnitMap reg;
    reg.insert(QModbusDataUnit::HoldingRegisters,{QModbusDataUnit::HoldingRegisters,0,8000});
    m_modbusDev->dev->setMap(reg);

    // update static variable
    m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,0,m_bmsSystem->Stacks);
    m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,1,m_bmsSystem->batteriesPerStack().at(0));
    m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,2,8);
    m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,3,5);
}

void BMS_Controller::updateModbusRegister()
{
    m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,4,m_bmsSystem->stacks().at(0)->stackVoltage());

    for(int i=0;i<7;i++){
        m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,6+i,100);
    }

    int offset = 13;
    ushort csum = 0;
    int stack = 0;
    foreach (BMS_Stack *s, m_bmsSystem->stacks()) {
        m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,offset++,s->stackCurrent());
        csum += s->stackCurrent();

        m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,1000*stack,s->maxCellVoltage());
        m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,1000*stack + 1,s->minCellVoltage());
        m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,1000*stack + 2,s->maxStackTemperature());
        m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,1000*stack + 3,s->minStackTemperature());

        int oo = 4;
        foreach (BMS_BMUDevice *b, s->batteries()) {
            for(int i=0;i<b->cellCount();i++){
                m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,1000*stack + oo++,b->cellVoltage(i));
            }
            for(int i=0;i<b->ntcCount();i++){
                m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,1000*stack + oo++,b->packTemperature(i));
            }
        }
    }

}

bool BMS_Controller::writeFrame(CAN_Packet *p)
{
    if(p == nullptr) return false;
    bool ret = false;
    //if(m_stateMach == nullptr) return false;

   // m_stateMach->add_packet(p);

    if(this->isSimulating()){
        QString str(p->data.toHex(' ').toUpper());
        qDebug()<<QString("W:CMD[%1], DATA[%2]").arg(p->Command,4,16).arg(str);
        ret = true;
    }
    else{
        QCanBusFrame frame;
        frame.setFrameId(p->Command);
        frame.setPayload(p->data);
        frame.setFrameType(p->remote?QCanBusFrame::RemoteRequestFrame:QCanBusFrame::DataFrame);
        frame.setExtendedFrameFormat(true);

        foreach (CANBUSDevice *c, m_canbusDevice) {
            c->dev->writeFrame(frame);
        }

        if(m_canbusDevice.size()>0){
            if(m_canbusDevice[1]->dev->writeFrame(frame)){
                qDebug()<<"Write frame OK";
                ret = true;
            }
            else{
                qDebug()<<"Write frame Fail";
                ret = false;
            }
        }
    }
    return ret;
}

void BMS_Controller::setBalancingVoltage(ushort v)
{
    CAN_Packet *p = new CAN_Packet;
    p->Command = 0x001;
    QDataStream ds(&p->data,QIODevice::WriteOnly);
    ds.setByteOrder(QDataStream::LittleEndian);
    ds << v;
    this->writeFrame(p);
    delete p;
}



//************** BMS_StateMachine *****************//
BMS_StateMachine::BMS_StateMachine(QObject *parent) : QObject(parent)
{

}

//void BMS_StateMachine::setState(BMS_Controller_State state)
//{
//    if(m_pendState == STATE_IDLE){
//        m_pendState = state;
//    }
//}

bool BMS_StateMachine::isWaitResp()
{
    if(m_pendPacket.size()>0){
        return m_pendPacket[0]->readFrame;
    }
    return false;
}

void BMS_StateMachine::feedFrame(QCanBusFrame f)
{

}

void BMS_StateMachine::add_packet(CAN_Packet *p)
{
    m_pendPacket.append(p);
}

void BMS_StateMachine::add_emg_packet(CAN_Packet *p)
{
    m_pendPacket.insert(0,p);
}

CAN_Packet *BMS_StateMachine::popPacket()
{
    CAN_Packet *p = nullptr;
    if(m_pendPacket.size() > 0){
        p = m_pendPacket.first();
        m_pendPacket.removeFirst();
    }
    return p;
}

//void BMS_StateMachine::run()
//{
//    switch(this->m_state){
//    case STATE_NOT_INITIALIZED:
//        break;
//    case STATE_INITIALIZING:
//        break;
//    case STATE_INITIALIZED:
//        break;
//    case STATE_NORMAL:
//        break;
//    case STATE_WRITE_FRAME:
//        break;
//    case STATE_WAIT_RESP:
//        break;
//    default:break;
//    }
//}
