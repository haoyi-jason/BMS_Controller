#include <QCoreApplication>
#include "bms_controller.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    BMS_Controller *c = new BMS_Controller();
    c->startServer("127.0.0.1",5001);
    return a.exec();
}
