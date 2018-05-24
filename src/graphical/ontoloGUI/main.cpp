#include "include/ontoloGenius/graphical/ontoloGUI/ontologui.h"
#include "include/ontoloGenius/graphical/ontoloGUI/DarkStyle.h"
#include <QApplication>

#include "ros/ros.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    a.setStyle(new DarkStyle);
    ontoloGUI w;
    w.show();

    ros::init(argc, argv, "ontoloGUI");

    ros::NodeHandle n;

    w.init(&n);
    w.wait();

    w.start();
    w.loadArguers();

    return a.exec();
}
