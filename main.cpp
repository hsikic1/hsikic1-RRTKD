#include "mainwindow.h"
#include <QApplication>
#include <string.h>

#define ROBOT_DOT 1
#define ROBOT_PLANAR 2
#define ROBOT_3D 3

int robotType(ROBOT_3D);

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w(0, robotType);
    w.show();
    if(robotType == ROBOT_3D)
        w.hide();
    return a.exec();
}
