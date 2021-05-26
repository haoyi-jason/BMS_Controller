#include "bms_controller.h"
#include <QtNetwork>
#include <QtCore>
#include <QCanBus>
#include <QCanBusDevice>
#include <QCanBusDeviceInfo>
#include <QCanBusFrame>
#include <QProcess>
#include<QJsonDocument>
#include<QJsonObject>
#include<QJsonValue>
#include<QJsonArray>
#include "../BMS_HY01/bms_def.h"
#include "../BMS_HY01/secs.h"

BMS_Controller::BMS_Controller(QObject *parent) : QObject(parent)
{
    m_devices.append("can0");
    m_devices.append("can1");

    m_server = new QTcpServer(this);

    m_bmsSystem = new BMS_SystemInfo();


    // load from file
    QString path = "config/local.json";

    QFile f(path);
    bool success = false;
    if(f.exists() && f.open(QIODevice::ReadOnly)){
        success = m_bmsSystem->Configuration(f.readAll());
        f.close();
    }
    else{
        qDebug()<<"No Configuration available";
    }

    if(success){
        // load local interface configuration
        if(this->loadConfig()){
            if(this->m_canbusDevice.count()>0){
                // start can device
                QString cmd;
                QProcess *proc = new QProcess();
                foreach (CANBUSDevice *dev, m_canbusDevice) {
                    cmd = QString("ip link set %1 down").arg(dev->name);
                    proc->execute(cmd);
                    proc->waitForFinished();
                    cmd = QString("ip link set %1 up type can bitrate %2").arg(dev->name).arg(dev->bitrate);
                    proc->execute(cmd);
                    proc->waitForFinished();
                }
            }
            // check if device count match config
            // load canbus device
            QString errorString;
            m_canbusDevInfo = QCanBus::instance()->availableDevices(QStringLiteral("socketcan"),&errorString);

            if(!errorString.isEmpty()){
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
//                QCanBusDevice *dev = QCanBus::instance()->createDevice(QStringLiteral("socketcan"),QString(d.name()),&errorString);
//                connect(dev,&QCanBusDevice::errorOccurred,this,&BMS_Controller::OnCanBusError);
//                connect(dev,&QCanBusDevice::framesReceived,this,&BMS_Controller::OnCanbusReceived);
//                dev->connectDevice();
//                m_canbusDevices.append(dev);

            }
            else{
                m_simulator = true;
            }
        }
//        if(m_canbusDevInfo.size() == 0){
//            // try to launch canbus device
//            QProcess *proc = new QProcess();
//            proc->execute("ip link set can0 up type can bitrate 250000");
//            proc->waitForFinished();
//            proc->execute("ip link set can1 up type can bitrate 250000");
//            proc->waitForFinished();
//            m_canbusDevInfo = QCanBus::instance()->availableDevices(QStringLiteral("socketcan"),&errorString);
//        }
//        if(m_canbusDevInfo.size() > 0){
//            m_canbusDevices.clear();
//            foreach(QCanBusDeviceInfo d, m_canbusDevInfo){
//                qDebug()<<d.name();
//                QCanBusDevice *dev = QCanBus::instance()->createDevice(QStringLiteral("socketcan"),QString(d.name()),&errorString);
//                connect(dev,&QCanBusDevice::errorOccurred,this,&BMS_Controller::OnCanBusError);
//                connect(dev,&QCanBusDevice::framesReceived,this,&BMS_Controller::OnCanbusReceived);
//                dev->connectDevice();
//                m_canbusDevices.append(dev);
//            }
//        }
//        else{
//            qDebug()<<"No valid device found";
//            m_simulator = true;
//        }
        if(m_simulator){
            m_bmsSystem->startSimulator(1000);
            mTimer = new QTimer();
            connect(mTimer,&QTimer::timeout,this,&BMS_Controller::handleTimeout);
            mTimer->start(2000);
        }
    }
}

bool BMS_Controller::startServer(QString ipAddress, int port)
{
    m_server->listen(QHostAddress(ipAddress),port);
    if(m_server->isListening()){
        connect(m_server,&QTcpServer::newConnection,this,&BMS_Controller::handleNewConnection);
        return true;
    }
    return false;
}

void BMS_Controller::stopServer()
{
    m_server->close();
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
    // echo server
//    if(s->bytesAvailable()){
//        QByteArray b = s->readAll();
//        s->write("ECHO:");
//        s->write(b);
//    }

    if(s->bytesAvailable()){
        QString str = QString(s->readAll());
        if(str == "READ:CFG"){
            qDebug()<<"Read Config:";
            QFile f("config/local.json");
            if(f.exists() && f.open(QIODevice::ReadOnly)){
                qDebug()<<"Reply config";
                QByteArray b = f.readAll();
                f.close();
                b.insert(0,hsmsParser::genHeader(hsmsParser::BMS_CONFIG,b.size()));
                s->write(b);
                foreach (RemoteSystem *sys, m_clients) {
                    if(sys->socket == s){
                        sys->configReady = true;
                    }
                }
            }
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
//    QByteArray b = m_bmsSystem->data();
    QByteArray b;
    QDataStream d(&b,QIODevice::ReadWrite);
    d << m_bmsSystem;
    b.insert(0,hsmsParser::genHeader(hsmsParser::BMS_STACK,b.size()));
    foreach (RemoteSystem *sys, m_clients) {
        if(sys->configReady){
            sys->socket->write(b);
        }
    }
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
    QString path = "config/interface.json";
    QFile f(path);
    bool success = false;
    if(f.exists() && f.open(QIODevice::ReadOnly)){
        QJsonParseError e;
        QJsonDocument d = QJsonDocument::fromJson(f.readAll());
        f.close();
        if(d.isNull()){
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
        return true;
    }
    else{
        qDebug()<<"No Configuration available";
        return false;
    }
    return false;
}
