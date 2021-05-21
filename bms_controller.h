#ifndef BMS_CONTROLLER_H
#define BMS_CONTROLLER_H

#include <QObject>
class QTcpSocket;
class QTcpServer;
class BMS_SystemInfo;
class QTimer;
class BMS_Controller : public QObject
{
    Q_OBJECT
public:
    explicit BMS_Controller(QObject *parent = nullptr);

    bool startServer(QString ipAddress, int port);
    void stopServer();
    void startCANHandler(QString device);
    void stopCANHandler(QString device);

signals:

public slots:
    void handleSocketDataReceived();
    void handleDisconnection();
    void handleNewConnection();
    void handleTimeout();


private:
    QTcpServer *m_server = nullptr;
    QList<QTcpSocket*> m_clients;
    bool m_simulator = false;
    BMS_SystemInfo *m_bmsSystem = nullptr;
    QTimer *mTimer;
};

#endif // BMS_CONTROLLER_H
