#include "include/ontologenius/graphical/ontoloGUI/DarkStyle.h"
#include "include/ontologenius/graphical/ontoloGUI/ontologui.h"

#include <QApplication>

#include <thread>
#include <signal.h>

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
    bool run = true;

    w.init(&n);
    w.wait();

    w.start();
    w.loadReasoners();

    std::thread spin_thread(spinThread,&run);

    signal(SIGINT, SIG_DFL);
    auto a_exec = a.exec();

    run = false;
    spin_thread.join();

    return a_exec;
}
