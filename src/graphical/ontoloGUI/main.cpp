#include "include/ontologenius/graphical/ontoloGUI/DarkStyle.h"
#include "include/ontologenius/graphical/ontoloGUI/ontologui.h"

#include <QApplication>

#include <csignal>
#include <thread>

#include <ros/package.h>
#include <ros/ros.h>

void spinThread(bool* run)
{
  ros::Rate r(100);
  while(*run == true)
  {
    ros::spinOnce();
    r.sleep();
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ontoloGUI");
  
    QApplication a(argc, argv);

    a.setStyle(new DarkStyle);

    std::string path = ros::package::getPath("ontologenius");
    path = path + "/docs/images/ontologenius.ico";
    QIcon icon(QString::fromStdString(path));
    a.setWindowIcon(icon);

    ontoloGUI w;
    w.show();

    bool run = true;

    w.init();
    w.wait();

    w.start();
    w.loadReasoners();

    std::thread spin_thread(spinThread, &run);

    signal(SIGINT, SIG_DFL);
    auto a_exec = a.exec();

    run = false;
    spin_thread.join();

    return a_exec;
}
