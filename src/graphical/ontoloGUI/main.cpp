#include <QApplication>
#include <QIcon>
#include <QString>
#include <csignal>
#include <string>
#include <thread>

#include "ontologenius/compat/ros.h"
#include "ontologenius/graphical/ontoloGUI/DarkStyle.h"
#include "ontologenius/graphical/ontoloGUI/ontologui.h"
#include "ontologenius/utils/Commands.h"
#include "qcoreapplication.h"

void spinThread(bool* /*run*/)
{
  ontologenius::compat::onto_ros::Node::get().spin();
}

int main(int argc, char* argv[])
{
  ontologenius::compat::onto_ros::Node::init(argc, argv, "ontoloGUI");

  bool run = true;
  std::thread spin_thread(spinThread, &run);

  const QApplication a(argc, argv);

  QApplication::setStyle(new DarkStyle);

  // std::string path = ament_index_cpp::get_package_share_directory("ontologenius");
  std::string path = ontologenius::findPackage("ontologenius");
  path = path + "/docs/images/ontologenius.ico";

  const QIcon icon(QString::fromStdString(path));
  QApplication::setWindowIcon(icon);

  OntoloGUI w;
  w.show();

  w.init();
  w.wait();

  w.start();
  w.loadReasoners();

  signal(SIGINT, SIG_DFL);
  auto a_exec = QApplication::exec();

  ontologenius::compat::onto_ros::Node::shutdown();

  run = false;
  spin_thread.join();

  return a_exec;
}
