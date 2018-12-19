#include "include/ontoloGenius/graphical/ontoloGUI/ontologui.h"
#include "include/ontoloGenius/graphical/ontoloGUI/DarkStyle.h"
#include <QApplication>

#include "ros/ros.h"
#include <ros/package.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    a.setStyle(new DarkStyle);

    std::string path = ros::package::getPath("ontologenius");
    path = path + "/docs/img/logo/ontologenius.ico";
    QIcon icon(QString::fromStdString(path));
    a.setWindowIcon(icon);

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
