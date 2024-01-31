#include "include/ontologenius/graphical/ontoloGUI/DarkStyle.h"
#include "include/ontologenius/graphical/ontoloGUI/ontologui.h"

#include <QApplication>

#include <csignal>
#include <thread>

#include "ontologenius/compat/ros.h"
#include "ontologenius/utils/Commands.h"

//#include <ament_index_cpp/get_package_share_directory.hpp>

void spinThread(bool* run)
{
  ontologenius::compat::onto_ros::Node::get().spin();
}

#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
  ontologenius::compat::onto_ros::Node::init(argc, argv, "ontoloGUI");

  bool run = true;
  std::thread spin_thread(spinThread, &run);

  QApplication a(argc, argv);

  a.setStyle(new DarkStyle);

  //std::string path = ament_index_cpp::get_package_share_directory("ontologenius");
  std::string path = ontologenius::findPackage("ontologenius");
  path = path + "/docs/images/ontologenius.ico";

  QIcon icon(QString::fromStdString(path));
  a.setWindowIcon(icon);

  ontoloGUI w;
  w.show();

  w.init();
  w.wait();

  w.start();
  w.loadReasoners();

  signal(SIGINT, SIG_DFL);
  auto a_exec = a.exec();

  ontologenius::compat::onto_ros::Node::shutdown();

  run = false;
  spin_thread.join();

  return a_exec;
}
