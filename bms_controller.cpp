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
#include <QModbusTcpServer>
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
        //path = "./config/local.json";
        path="d:/temp/bms/config/controller.json";
    }
    else{
       //path = QCoreApplication::applicationDirPath()+"/config/local.json";
        path = "/opt/bms/config/controller.json"; //-- change after Jul. 21'
    }
    //qDebug()<<"Current Path:"<<path;

    QFile f(path);
    bool success = false;
    if(f.exists()){
        //success = m_bmsSystem->Configuration(f.readAll());
        success = m_bmsSystem->Configuration2(path);
        //f.close();
    }
    else{
        log(QString("File local.json not exist"));
        qDebug()<<"No Configuration available 1"<<f.exists();
    }

    if(success){
        // load local interface configuration
        if(this->loadConfig()){

            if(m_bmsSystem->isSimulate()){
                log("Start Simulator");
                m_bmsSystem->startSimulator(1000);
                this->m_connected = true;
            }
            else{
                if(this->m_canbusDevice.count()>0){
                    // start can device
                    log("Start Socket CAN Device");
                    QString cmd;
                    QProcess *proc = new QProcess();

                    foreach (CANBUSDevice *dev, m_canbusDevice) {
                        cmd = QString("ip link set %1 down").arg(dev->name);
                        proc->start("sh",QStringList()<<"-c"<<cmd);
                        proc->waitForFinished();
                        //qDebug()<<"Proc Result:"<<proc->readAll();
                        cmd = QString("ip link set %1 up type can bitrate %2 restart-ms 1000").arg(dev->name).arg(dev->bitrate);
                        proc->start("sh",QStringList()<<"-c"<<cmd);
                        proc->waitForFinished();
                        //qDebug()<<"Proc Result:"<<proc->readAll();
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
               // qDebug()<<"Device found:"<<m_canbusDevInfo.size();
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

            // check network config
            configNetwork();
            // start MODBUS Slave
            if(m_bmsSystem->localConfig()->modbus.ConfigReady){
                this->m_modbusDev = new MODBUSDevice();
                this->m_modbusDev->bitrate = m_bmsSystem->localConfig()->modbus.Bitrate.toInt();
                this->m_modbusDev->portName = m_bmsSystem->localConfig()->modbus.Port;

                log(QString("Start MODBUS RTU Slave at %1, baudrate=%2").arg(m_modbusDev->portName).arg(m_modbusDev->bitrate));

                /*
                 * modbus.Enable = TRUE for RTU, False for TCP
                 */
                QString ipAddress;
                const QHostAddress &localhost = QHostAddress(QHostAddress::LocalHost);
                for (const QHostAddress &address: QNetworkInterface::allAddresses()) {
                    if (address.protocol() == QAbstractSocket::IPv4Protocol && address != localhost){
                         qDebug() << address.toString();
                        ipAddress = address.toString();
                    }
                }
                if(!m_bmsSystem->localConfig()->modbus.Enable){
                    m_modbusDev->dev = new QModbusTcpServer();
                    this->m_modbusDev->tcpPort = m_bmsSystem->localConfig()->modbus.TCPPort.toInt();
                    //const QUrl url = QUrl::fromUserInput("127.0.0.1:502");
                    //this->m_modbusDev->dev_tcp->setConnectionParameter(QModbusDevice::NetworkPortParameter, url.port());
                    this->m_modbusDev->dev->setConnectionParameter(QModbusDevice::NetworkAddressParameter, m_bmsSystem->localConfig()->network.ip.trimmed());
                    this->m_modbusDev->dev->setConnectionParameter(QModbusDevice::NetworkPortParameter,m_modbusDev->tcpPort);
                }
                else{
                    m_modbusDev->dev = new QModbusRtuSerialSlave();
                    if(QSysInfo::productType().contains("win")){
                        m_modbusDev->dev->setConnectionParameter(QModbusDevice::SerialPortNameParameter,"COM1");
                    }
                    else{
                        m_modbusDev->dev->setConnectionParameter(QModbusDevice::SerialPortNameParameter,m_modbusDev->portName);
                    }
                    m_modbusDev->dev->setConnectionParameter(QModbusDevice::SerialBaudRateParameter,m_modbusDev->bitrate);
                    m_modbusDev->dev->setConnectionParameter(QModbusDevice::SerialDataBitsParameter,QSerialPort::Data8);
                    QString parity = m_bmsSystem->localConfig()->modbus.Parity;
                    if(parity == "EVEN"){
                        m_modbusDev->dev->setConnectionParameter(QModbusDevice::SerialParityParameter,QSerialPort::EvenParity);
                    }
                    else if(parity == "ODD"){
                        m_modbusDev->dev->setConnectionParameter(QModbusDevice::SerialParityParameter,QSerialPort::OddParity);
                    }
                    else{
                        m_modbusDev->dev->setConnectionParameter(QModbusDevice::SerialParityParameter,QSerialPort::NoParity);
                    }

                    m_modbusDev->dev->setConnectionParameter(QModbusDevice::SerialStopBitsParameter,QSerialPort::OneStop);

                }
                m_modbusDev->dev->setServerAddress(m_bmsSystem->localConfig()->modbus.ID.toInt());


                if(m_modbusDev->dev->connectDevice()){
                    m_modbusDev->connected = true;
                    prepareModbusRegister();
                    log("Modbus Server start successfully");
                    qDebug()<<"Modbus Start success";
                }
                else{
                    log("MODBUS Server start failed");
                    qDebug()<<"Modbus Startu fail";
                }

            }
            mTimer = new QTimer();
            connect(mTimer,&QTimer::timeout,this,&BMS_Controller::handleTimeout);
//            mTimer->start(100);

            mStateTimer = new QTimer();
            connect(mStateTimer,&QTimer::timeout,this,&BMS_Controller::handleStateMachTimeout);
            mStateTimer->start(100);

//            m_bmsSystem->enableAlarmSystem(true); // move to state machine
            //connect(m_bmsSystem,&BMS_System::setBalancingVoltage,this,&BMS_Controller::setBalancingVoltage);
            connect(m_bmsSystem,&BMS_System::logMessage,this,&BMS_Controller::log);
            connect(m_bmsSystem,&BMS_System::addPacket,this,&BMS_Controller::addFrame);
        }
    }

  //  m_bmsSystem->On_BMU_ov(0x10);
// 220117 remove mount
//    if(!QSysInfo::productType().contains("win")){
//        if(!QDir("/mnt/t").exists()){

//            QDir().mkdir("/mnt/t");
//        }
//        // try to mount sd
//        QProcess proc;

//        proc.execute("mount /dev/mmcblk1p1 /mnt/t");
//        proc.waitForFinished();
//        log(QString("Try to Mount SD Card: %1").arg(QString(proc.readAll())));
//    }
}

void BMS_Controller::configNetwork()
{
   //if(!m_bmsSystem->localConfig()->modbus.ConfigReady) return;

//   //qDebug()<<Q_FUNC_INFO;
//   QString cmd;
//   QString nicName;
//   QProcess proc;
//   if(!m_bmsSystem->localConfig()->network.Dhcp){
//       QString ipAddress;
//       //QHostAddress ha;
//       const QHostAddress &localhost = QHostAddress(QHostAddress::LocalHost);
//       for (const QHostAddress &address: QNetworkInterface::allAddresses()) {
//           if (address.protocol() == QAbstractSocket::IPv4Protocol && address != localhost){
//                //qDebug() << address.toString();
//               ipAddress = address.toString();
//               break;
//               //ha = address;
//           }
//       }
//       proc.start("connmanctl services");
//       proc.waitForFinished();
//       QStringList sl = QString(proc.readAll()).split(" ",QString::SkipEmptyParts);
//       nicName = sl.at(2).trimmed();
//       if(nicName.isEmpty()){
//            qDebug()<<"No Nic presents";
//           return; // no nic
//       }
////       qDebug()<<"connmanctl:"<<sl.at(2);
//       qDebug()<<Q_FUNC_INFO<<" Compare:"<<ipAddress<<":"<<m_bmsSystem->localConfig()->network.ip;
//        if(QString::compare(ipAddress.trimmed(),m_bmsSystem->localConfig()->network.ip.trimmed(),Qt::CaseInsensitive)){
////            ha.setAddress(m_bmsSystem->localConfig()->network.ip.trimmed());
//            qDebug()<<Q_FUNC_INFO<< " Config to IP:"<<m_bmsSystem->localConfig()->network.ip;
//            QString ip = m_bmsSystem->localConfig()->network.ip;
//            //cmd = QString("connmanctl config %1 --ipv4 manual %2").arg(nicName).arg(ip);
////            //proc.execute(cmd);
////            //proc.start("sh",QStringList()<<"-c");
////            //proc.waitForFinished();
//            cmd = QString("/bin/sh -c \"ifconfig eth0 %1 up\"").arg(ip);
////            proc->start("/bin/sh",QStringList()<<"-c" << QString("if config eth0 %1 netmask 255.255.255.0 up").arg(ip));
////            proc.start(QString("if config eth0 %1 netmask 255.255.255.0 up").arg(ip));
////            qDebug()<<"connmanctl:"<<QString(proc.readAll());
//            qDebug()<<"Execute command:"<<cmd;
//            proc.start(cmd);
//            proc.waitForFinished();
//        }
//        else{
//            qDebug()<<"Network Address OK";
//        }

//   }else{
//       cmd = QString("connmanctl config %1 --ipv4 dhcp").arg(nicName);
//       qDebug()<<"Execute command:"<<cmd;
//       proc.start(cmd);
//       proc.waitForFinished();
//   }

   // start udp broadcaster
   mUdpSocket = new QUdpSocket(this);
   connect(&mUdpTimer,&QTimer::timeout,this,&BMS_Controller::broadCastUDPPacket);
   mUdpTimer.start(5000); // every 5000 ms

}

void BMS_Controller::broadCastUDPPacket()
{
    QByteArray msg = "BMS_HMI\n";
    mUdpSocket->writeDatagram(msg,QHostAddress::Broadcast,5329);
    //qDebug()<<Q_FUNC_INFO;
}

bool BMS_Controller::startServer()
{
//    m_server->listen(QHostAddress(m_bmsSystem->connectionString),m_bmsSystem->connectionPort);
    m_server->listen(QHostAddress::Any,m_bmsSystem->connectionPort);
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
    QTcpSocket *s = static_cast<QTcpSocket*>(sender());

    if(s->bytesAvailable()){
        QString str = QString(s->readAll());
        // parse string
        QStringList sl = str.split(":");
        if(sl.size() > 1){
            switch(cmd_map.value(sl[0])){
            case 0: // SYS
                switch(sys_cmd_map.value(sl[1])){
                case 0: // SYS:CFG
                    foreach (RemoteSystem *sys, m_clients) {
                        if(sys->socket == s){
                            sys->configReady = true;
                        }
                    }
                    break;
                case 1: // SYS:ALMRST
                    m_bmsSystem->clearAlarm();
                    if(m_bmsSystem->bcu()->digitalOutState(0)==1){
                        CAN_Packet *p = nullptr;
                        if((p = m_bmsSystem->bcu()->setDigitalOut(0,0)) != nullptr){
                            p->Command |= (0x01 << 12); // bcudevice
                            this->writeFrame(p);
                        }

                    }
                    if(m_bmsSystem->bcu()->digitalOutState(1)==1){
                        CAN_Packet *p = nullptr;
                        if((p = m_bmsSystem->bcu()->setDigitalOut(1,0)) != nullptr){
                            p->Command |= (0x01 << 12); // bcudevice
                            this->writeFrame(p);
                        }

                    }
                    break;
                case 2: // SYS:INIT_TIME:
                    break;
                case 3: // SYS:CFGFR , read config file
                    {
                    QString path;
                    if(QSysInfo::productType().contains("win")){
                        //path = "./config/local.json";
                        path="d:/temp/bms/config/controller.json";
                    }
                    else{
//                        path = QCoreApplication::applicationDirPath()+"/config/local.json";
                        path = "/opt/bms/config/controller.json"; //-- change after Jul. 21'
                    }
                    QFile f(path);
                    if(f.exists() && f.open(QIODevice::ReadOnly)){
                        QByteArray b = f.readAll();
                        f.close();
                        //b.insert(0,QByteArray::number(BMS_CONTROLLER_VERSION,16));
                        b.insert(0,hsmsParser::genHeader(hsmsParser::BMS_CONFIG,b.size()));
                        s->write(b);
                        foreach (RemoteSystem *sys, m_clients) {
                            if(sys->socket == s){
                                sys->configReady = true;
                            }
                        }
                    }
                    }
                    break;
                case 4: // SYS:CFGFW

                    break;
                case 5: // SYS:VER
                    s->write(QByteArray::number(BMS_CONTROLLER_VERSION,16));
                    break;
                }
                break;
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
            case 5: // SVC
                switch(svi_cmd_map.value(sl[1])){
                case 0: //SVI:AINMAP:GID:CH:OPT:VALUE
                    //qDebug()<<sl;
                    if(sl.size() == 6){
                        CAN_Packet *p;
                        switch(sl[4].toInt()){
                        case 0: // raw low
                            p = BMS_SVIDevice::rawLow(sl[2].toInt(),sl[3].toInt(),sl[5].toInt());
                            break;
                        case 1: // raw high
                            p = BMS_SVIDevice::rawHigh(sl[2].toInt(),sl[3].toInt(),sl[5].toInt());
                            break;
                        case 2: // eng low
                            p = BMS_SVIDevice::engLow(sl[2].toInt(),sl[3].toInt(),sl[5].toFloat());
                            break;
                        case 3: // eng high
                            p = BMS_SVIDevice::engHigh(sl[2].toInt(),sl[3].toInt(),sl[5].toFloat());
                            break;
                        case 4: // zero cal
                            p = BMS_SVIDevice::zeroCalibration(sl[2].toInt(),sl[3].toInt());
                            break;
                        case 5: // band cal
                            p = BMS_SVIDevice::bandCalibration(sl[2].toInt(),sl[3].toInt(),sl[5].toFloat());
                            break;
                        }
                        if(p != nullptr){
                            addFrame(p);
//                            qDebug()<<"Write SVC Calibration";
//                            QCanBusFrame frame;
//                            frame.setFrameId(p->Command);
//                            frame.setPayload(p->data);
//                            frame.setFrameType(QCanBusFrame::DataFrame);
//                            //qDebug()<<"Size="<<p->data.size() << "/" << frame.payload().size();
//                            if(m_canbusDevice.size()>0){
//                                if(m_canbusDevice[0]->dev->writeFrame(frame)){
//                                    qDebug()<<"Write frame OK";
//                                }
//                                else{
//                                    qDebug()<<"Write frame Fail";
//                                }
//                            }
//                            delete p;
                        }
                    }
                    break;
                case 1: // SVI:SOHT:1/0, tracking soh
                    if(sl.size()==3){
                        bool en = (sl[2].toInt()==1);
                        foreach (BMS_Stack *st, this->m_bmsSystem->stacks()) {
                            st->sviDevice()->setSOHTracking(en);
                        }
                    }
                    break;
                case 2: // SVI:SSOC:GID:V, set soc to v
                    if(sl.size() == 4){
                        quint8 id = (quint8)sl[2].toInt();
                        float soc = sl[3].toFloat();
                        foreach (BMS_Stack *st, this->m_bmsSystem->stacks()) {
                            if(st->groupID() == id){
                                st->sviDevice()->soc(soc);
                            }
                        }
                    }
                    break;
                case 3: // SVI:SBW:gid:temp:current
                    if(sl.size() == 5){
                        quint8 id = (quint8)sl[2].toInt();
                        float temp = sl[3].toFloat();
                        float current = sl[4].toFloat();
                        foreach (BMS_Stack *st, this->m_bmsSystem->stacks()) {
                            if(st->groupID() == id){
                                CAN_Packet *p,*p2;
                                p = BMS_SVIDevice::writeFanSetting(id,0,temp);
                                addFrame(p);
                                p2 = BMS_SVIDevice::writeFanSetting(id,1,current);
                                addFrame(p2);
                            }
                        }
                    }
                    break;
                case 4: // SVI:SBR:gid
                    if(sl.size() == 3){
                        quint8 id = (quint8)sl[2].toInt();
                        CAN_Packet *p,*p2;
                        p = BMS_SVIDevice::readFanSetting(id,0);
                        addFrame(p);
                        p2 = BMS_SVIDevice::readFanSetting(id,1);
                        addFrame(p2);
                    }
                    break;
                case 5: // SVI:SBF:gid:fid:state
                    if(sl.size() == 5){
                        quint8 id = (quint8)sl[2].toInt();
                        quint8 fid =(quint8)sl[3].toInt();
                        quint8 on =(quint8)sl[4].toInt();
                        CAN_Packet *p;
                        p = BMS_SVIDevice::writeFanControl(id,fid,on==1);
                        addFrame(p);
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
                        foreach(BMS_Stack *st, m_bmsSystem->stacks()){
                            quint8 id = (quint8)sl[2].toInt();
                            if(st->groupID() == GROUP_OF(id) ){
                                st->setSimCellData((quint8)sl[2].toInt(),(quint8)sl[3].toInt(),(quint16)sl[4].toInt());
                            }
                        }
                    }
                    break;
                case 1: // // SIM:CT:BID:CID:V
                    if(sl.size() == 5){
                        foreach(BMS_Stack *st, m_bmsSystem->stacks()){
                            quint8 id = (quint8)sl[2].toInt();
                            if(st->groupID() == GROUP_OF(id) ){
                                st->setSimTempData((quint8)sl[2].toInt(),(quint8)sl[3].toInt(),(quint16)sl[4].toInt());
                            }
                        }
                    }
                    break;
                case 2: // SIM:SV:GID:V, simulate stack voltage
                    if(sl.size() == 4){
                        foreach (BMS_Stack *st, m_bmsSystem->stacks()) {
                            quint8 id = (quint8)sl[2].toInt();
                            if(st->groupID() == (id)){
                                st->sviDevice()->setSimVoltage(sl[3].toInt());
                            }
                        }
                    }
                    break;
                case 3: // SIM:SA:GID:V, simulate stack current
                    if(sl.size() == 4){
                        foreach (BMS_Stack *st, m_bmsSystem->stacks()) {
                            quint8 id = (quint8)sl[2].toInt();
                            if(st->groupID() == (id)){
                                st->sviDevice()->setSimAmpere(sl[3].toInt());
                            }
                        }
                    }
                    break;
                case 4: // SIM_SSOC:GID:V, simlate stack soc
                    if(sl.size() == 4){
                        foreach (BMS_Stack *st, m_bmsSystem->stacks()) {
                            quint8 id = (quint8)sl[2].toInt();
                            if(st->groupID() == (id)){
                                st->sviDevice()->setSimSOC(sl[3].toInt());
                            }
                        }
                    }
                    break;
                case 5: // SIM:RST
                    foreach (BMS_Stack *st, m_bmsSystem->stacks()) {
                        st->sviDevice()->setSimSOC(0);
                        st->sviDevice()->setSimVoltage(0);
                        st->sviDevice()->setSimAmpere(0);
                        st->setSimCellData(0xff,0,0);
                        st->setSimTempData(0xff,0,0);
                    }
                }
                break;
            }
        }

    }

}

void BMS_Controller::OnSerialCanRead()
{
    QSerialPort *p = static_cast<QSerialPort*>(sender());
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
    QTcpSocket *s = static_cast<QTcpSocket*>(sender());
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
        //m_bmsSystem->rec_log(b);
        //m_bmss
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

/**
 * @brief BMS_Controller::handleStateMachTimeout
 * Description: this function called every 100 ms
 */

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
            if(m_stateMach->stateDelay == 0){
                m_stateMach->stateRetry = 0;
                m_stateMach->subState = 1;
            }
            break;
        case 1: // start bcu power
            if(m_bmsSystem->isSimulate()){
                m_stateMach->subState++;
                m_stateMach->stateDelay = 10;
                m_stateMach->stateRetry = 0;
            }
            else{
                if(m_bmsSystem->bcu() != nullptr){
                    CAN_Packet *p = m_bmsSystem->bcu()->setVoltageSource(0,m_bmsSystem->bcu()->vsource_limit(0));
                    if(writeFrame(p)){
                        m_stateMach->subState++;
                        m_stateMach->stateDelay = 10;
                        m_stateMach->stateRetry = 0;
                        log("Start BCU Voltage source channel 0 success");
                    }
                    else{
                        log("Start BCU Voltage source channel 0 failed");
                        m_stateMach->stateRetry++;
                        m_stateMach->stateDelay = 10;
                        m_stateMach->subState = 0; // short delay
                    }
                    delete p;
                    if(m_stateMach->stateRetry > 10){
                        log("BCU Stat Failed, controller terminated");
                        exit(-1);
                    }
                }
            }
            break;
        case 2:
            if(m_stateMach->stateDelay > 0)
                m_stateMach->stateDelay--;
            if(m_stateMach->stateDelay == 0)
                m_stateMach->subState = 3;
            break;
        case 3:
            if(m_bmsSystem->isSimulate()){
                m_stateMach->subState = 0;
                m_stateMach->state = BMS_StateMachine::STATE_INITIALIZING;
                m_stateMach->stateDelay = 10;
            }
            else{
                if(m_bmsSystem->bcu() != nullptr){
                    CAN_Packet *p = m_bmsSystem->bcu()->setVoltageSource(1,m_bmsSystem->bcu()->vsource_limit(1));
                    if(writeFrame(p)){
                        m_stateMach->subState = 0;
                        m_stateMach->state = BMS_StateMachine::STATE_INITIALIZING;
                        m_stateMach->stateDelay = 10;
                        m_stateMach->stateRetry = 0;
                        log("Start BCU Voltage source channel 1 success");
                    }
                    else{
                        log("Start BCU Voltage source channel 1 failed");
                        m_stateMach->stateRetry++;
                        m_stateMach->stateDelay = 10;
                        m_stateMach->subState = 2; // short delay
                    }
                    delete p;
                    if(m_stateMach->stateRetry > 10){
                        log("BCU Stat Failed, controller terminated");
                        exit(-1);
                    }
                }
            }
            break;
        }

        break;
    case BMS_StateMachine::STATE_INITIALIZING:
        if(m_stateMach->stateDelay > 0){
            m_stateMach->stateDelay--;
        }
        if(m_stateMach->stateDelay == 0){
            if(m_bmsSystem->isSimulate()){
                m_stateMach->state = BMS_StateMachine::STATE_INITIALIZED;
            }
            else{
                // start bmus
                if(m_bmsSystem != nullptr){
                    CAN_Packet *p = m_bmsSystem->startBMUs(true);
                    if(writeFrame(p)){
                        m_stateMach->state = BMS_StateMachine::STATE_INITIALIZED;
                        m_stateMach->stateDelay = 0;
                        log("Start BMU Devices success");
                    }
                    else{
                        log("Start BMU Devices failed");
                        m_stateMach->stateRetry++;
                        m_stateMach->stateDelay = 10;
                    }
                    delete p;
                }
                if(m_stateMach->stateRetry > 5){
                    log("Start BMU failed");
                    exit(-1);
                }
            }
        }
        break;
    case BMS_StateMachine::STATE_INITIALIZED:
        if(m_stateMach->stateDelay > 0){
            m_stateMach->stateDelay--;
        }
        else if(m_stateMach->stateDelay == 0){
            if(m_stateMach->stateDelay == 0){
                mTimer->start(100);
                m_stateMach->state = BMS_StateMachine::STATE_NORMAL;
                m_validDelay = 50;
                log("BMS System initialized");
            }
        }

        break;
    case BMS_StateMachine::STATE_NORMAL:
        // check if can enable alarm system
        if(m_validDelay > 0){
            m_validDelay--;
            if(m_validDelay == 0){
                m_bmsSystem->enableAlarmSystem(true);
            }
        }
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
            quint32 alarm = m_bmsSystem->alarmState();
            //qDebug()<<Q_FUNC_INFO<<QString("Alarm:%1").arg(alarm);

            /*
             *  20221119: modify digital output 0 to control switch box
             *  Warning on StackOV && Cell OV (alarm bit mask == 0x05)
             *
             *  20230614: modify digital output 0 to control switch box fan status
             *  ON: fan error
             *  OFF: fan OK
             *  bit : 12
             */
            // warning @ low 16-bit
//            if((alarm & 0xFFFF) != 0){
            //qDebug()<<"Alarm:"<<alarm;
            if((alarm & (1<<bms::SVI_FAN)) == 0){
                // check if bcu's digital output state is set or not
                if(m_bmsSystem->bcu()->digitalOutState(m_bmsSystem->warinig_out_id()) == 1){
                    CAN_Packet *p = m_bmsSystem->bcu()->setDigitalOut(m_bmsSystem->warinig_out_id(),0);
                    //p->Command |= (1 <<12);
                    writeFrame(p);
                }
            }
            else // if(!m_bmsSystem->warinig_latch())
            {
                if(m_bmsSystem->bcu()->digitalOutState(m_bmsSystem->warinig_out_id()) == 0){
                    CAN_Packet *p = m_bmsSystem->bcu()->setDigitalOut(m_bmsSystem->warinig_out_id(),1);
                    //p->Command |= (1 <<12);
                    writeFrame(p);
                }
            }

            if((alarm & 0xffff0000) != 0){
                // check if bcu's digital output state is set or not
                if(m_bmsSystem->bcu()->digitalOutState(m_bmsSystem->alarm_out_id()) == 0){
                    CAN_Packet *p = m_bmsSystem->bcu()->setDigitalOut(m_bmsSystem->alarm_out_id(),1);
                    //p->Command |= (1 <<12);
                    writeFrame(p);
                }
            }
            else if(!m_bmsSystem->alarm_latch()){
                if(m_bmsSystem->bcu()->digitalOutState(m_bmsSystem->alarm_out_id()) == 1){
                    CAN_Packet *p = m_bmsSystem->bcu()->setDigitalOut(m_bmsSystem->alarm_out_id(),0);
                    //p->Command |= (1 <<12);
                    writeFrame(p);
                }
            }
        }

        if(m_balancingDelay>0){
            m_balancingDelay--;
        }else{
            writeFrame(m_bmsSystem->broadcastBalancing());
            m_balancingDelay = 50;

            // add 210730, set vsource on every 5 second to prevent bus error
            // remove 210820, use bcu to monitoring this function
            //CAN_Packet *p = m_bmsSystem->bcu()->setVoltageSource(0,m_bmsSystem->bcu()->vsource_limit(0));
            //writeFrame(p);

            //p = m_bmsSystem->bcu()->setVoltageSource(1,m_bmsSystem->bcu()->vsource_limit(1));
            //writeFrame(p);
            if(m_bmsSystem->bcu()->getWorkingCurrent(0) < 20)
            {
                writeFrame(m_bmsSystem->bcu()->setVoltageSource(0,m_bmsSystem->bcu()->vsource_limit(0)));
            }
            else if(m_bmsSystem->bcu()->getWorkingCurrent(1) < 20){
                writeFrame(m_bmsSystem->bcu()->setVoltageSource(1,m_bmsSystem->bcu()->vsource_limit(1)));
            }

        }

        if(m_stateMach->pendState != BMS_StateMachine::STATE_NONE){
            m_stateMach->state = m_stateMach->pendState;
            m_stateMach->pendState = BMS_StateMachine::STATE_NONE;
        }

        m_bmsSystem->ms_poll_100();

        if(m_pendPackets.size() > 0){
            writeFrame(m_pendPackets.first());
            m_pendPackets.removeFirst();
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
            qDebug()<<"Canbus Error:"<<error << "Device:" << (dev->objectName());
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
        //path = "./config/interface.json";
        path = "d:/temp/bms/config/interface.json";
    }
    else{
//        path = QCoreApplication::applicationDirPath()+"/config/interface.json";
        path="/opt/bms/config/interface.json";
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

//        if(obj.contains("mbslave")){
//            QJsonObject o = obj["mbslave"].toObject();
//            this->m_modbusDev = new MODBUSDevice();
//            this->m_modbusDev->bitrate = o["bitrate"].toInt();
//            this->m_modbusDev->portName = o["port"].toString();
//        }
//        if(obj.contains("config")){
//            QJsonObject o = obj["config"].toObject();
//            if(QSysInfo::productType().contains("win")){
//                this->m_logPath = o["log_path"].toString();
//            }
//            else{
//                this->m_logPath = QCoreApplication::applicationDirPath()+o["log_path"].toString();
//            }
//            // check if folder presents
//            if(!QDir(this->m_logPath).exists()){
//                QDir().mkdir(this->m_logPath);
//            }
//        }
//        if(obj.contains("simulate")){
//            this->m_simulator = obj["simulate"].toBool();
//        }
        return true;
    }
    else{
        qDebug()<<"No Configuration available 2";
        return false;
    }
    return false;
}

void BMS_Controller::log(QString message)
{

    QString path =QString("%1/sys/log-%2.txt").arg(this->m_bmsSystem->localConfig()->record.LogPath).arg(QDateTime::currentDateTime().toString("yyyy-MM-dd"));
    QFile f(path);
    QString logText = QString("%1:%2\n").arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss:")).arg(message);
    if(f.open(QIODevice::ReadWrite | QIODevice::Append)){
        f.write(logText.toUtf8());
        f.close();
        //return true;
    }
    //f.close();
    //return false;
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
    m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,0,m_bmsSystem->stacks().size());
    m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,1,m_bmsSystem->batteriesPerStack().at(0));
    m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,2,m_bmsSystem->stacks().at(0)->batteries().at(0)->cellCount());
    m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,3,m_bmsSystem->stacks().at(0)->batteries().at(0)->ntcCount());

    // 40020
    m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,20,m_bmsSystem->localConfig()->criteria.cell.volt_warning.Low_Set.toFloat()*1000);
    m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,21,m_bmsSystem->localConfig()->criteria.cell.volt_warning.High_Set.toFloat()*1000);
    m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,22,0);
    m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,23,m_bmsSystem->localConfig()->criteria.cell.volt_alarm.Low_Set.toFloat()*1000);
    m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,24,m_bmsSystem->localConfig()->criteria.cell.volt_alarm.High_Set.toFloat()*1000);
    m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,25,0);
    m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,26,m_bmsSystem->localConfig()->criteria.cell.temp_warning.Low_Set.toFloat());
    m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,27,m_bmsSystem->localConfig()->criteria.cell.temp_warning.High_Set.toFloat());
    m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,28,m_bmsSystem->localConfig()->criteria.cell.temp_alarm.Low_Set.toInt());
    m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,29,m_bmsSystem->localConfig()->criteria.cell.temp_alarm.High_Set.toInt());
    m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,29,m_bmsSystem->localConfig()->event_output.alarm_latch?1:0);
    m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,29,m_bmsSystem->localConfig()->event_output.warning_latch?1:0);


}

void BMS_Controller::updateModbusRegister()
{

    int offset = 13;
    int sv_offset = 35;
    short csum = 0;
    short vsum = 0;
    int stack = 1;
    foreach (BMS_Stack *s, m_bmsSystem->stacks()) {
        m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,offset++,s->stackCurrent());
        m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,sv_offset++,s->stackVoltage());
        csum += s->stackCurrent();
        vsum += s->stackVoltage();
        m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,1000*stack    ,s->maxCellVoltage());
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
        // SOC
        m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,stack+5, s->soc());

        // warning /alarm
        quint32 alarmState = s->alarmState();
        short err = 0;
        if((alarmState & 0x0F) != 0x00){
            err += 1;
        }

        if((alarmState & 0x30 ) != 0x00){
            err += 2;
        }
        m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,100 + (stack-1)*2,err);
        m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,101 + (stack-1)*2,err);
        stack++;
    }
    vsum /= (stack-1);
    m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,4,vsum);
    m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,5,csum);

    // digital output state
    m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,33,m_bmsSystem->bcu()->digitalOutState(m_bmsSystem->warinig_out_id()));
    m_modbusDev->dev->setData(QModbusDataUnit::HoldingRegisters,34,m_bmsSystem->bcu()->digitalOutState(m_bmsSystem->alarm_out_id()));

}

void BMS_Controller::addFrame(CAN_Packet *p)
{
    m_pendPackets.append(p);
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

        bool fail = false;
        foreach (CANBUSDevice *c, m_canbusDevice) {
            if(c->dev != nullptr){
                fail = !c->dev->writeFrame(frame);
                //QThread::msleep(50);
            }
        }
        ret = !fail;

//        if(m_canbusDevice.size()>0){
//            if(m_canbusDevice[1]->dev->writeFrame(frame)){
//                //qDebug()<<"Write frame OK";
//                ret = true;
//            }
//            else{
//                //qDebug()<<"Write frame Fail";
//                ret = false;
//            }
//        }
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
