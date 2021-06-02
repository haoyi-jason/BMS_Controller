#include <QCoreApplication>
#include "bms_controller.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    BMS_Controller *c = new BMS_Controller();
    if(c->isConnected()){
        if(!c->startServer()){

            return -1;
        }
    }
    else{
        return -1;
    }
    return a.exec();
}
