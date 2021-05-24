#include "bms_controller.h"
#include <QtNetwork>
#include "../BMS_HY01/bms_def.h"
#include "../BMS_HY01/secs.h"
//#include "i7565/VCI_CAN.h"
//#include "windows.h"

BMS_Controller::BMS_Controller(QObject *parent) : QObject(parent)
{
    m_server = new QTcpServer(this);

    m_bmsSystem = new BMS_SystemInfo();

    // load from file
    QString path = "config\\local.json";

    QFile f(path);
    if(f.exists() && f.open(QIODevice::ReadOnly)){
        m_bmsSystem->Configuration(f.readAll());
        f.close();
    }
    else{
        qDebug()<<"No Configuration available";
    }

    m_simulator = true;

    if(m_simulator){
        m_bmsSystem->startSimulator(1000);
    }

//    else{
//        m_simulator = true;
//        m_bmsSystem->generateDummySystem();

//    }

    mTimer = new QTimer();
    connect(mTimer,&QTimer::timeout,this,&BMS_Controller::handleTimeout);
    mTimer->start(2000);

//    hsmsParser *p = new hsmsParser();
//    QByteArray a = p->genHeader(hsmsParser::FORMAT_BIN,100);
//    QByteArray b = p->genHeader(hsmsParser::FORMAT_BIN,1000);
//    QByteArray c = p->genHeader(hsmsParser::FORMAT_BIN,10000);
//    QByteArray a;
//    int len;
//    quint8 type;
//    a.append(hsmsParser::BMS_CONFIG<<2 | 0x1);
//    a.append(111);
//    type = hsmsParser::getHeader(a,&len);
//    a.clear();

//    a.append(hsmsParser::BMS_CONFIG<<2 | 0x2);
//    a.append(11);
//    a.append(11);
//    type = hsmsParser::getHeader(a,&len);
//    a.clear();

//    a.append(hsmsParser::BMS_CONFIG<<2 | 0x3);
//    a.append(22);
//    a.append(22);
//    a.append(22);
//    type = hsmsParser::getHeader(a,&len);
//    a.clear();
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
