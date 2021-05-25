#include <QCoreApplication>
#include "bms_controller.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    BMS_Controller *c = new BMS_Controller();
    c->startServer("192.168.0.126",5001);
    return a.exec();
}
