#ifndef ONTOLOGUI_H
#define ONTOLOGUI_H

#include <QMainWindow>
#include "include/ontoloGenius/graphical/ontoloGUI/QCheckBoxExtended.h"

#include "ros/ros.h"
#include <vector>
#include <string>

namespace Ui {
class ontoloGUI;
}

class ontoloGUI : public QMainWindow
{
    Q_OBJECT

public:
  explicit ontoloGUI(QWidget *parent = 0);
  ~ontoloGUI();

  void init(ros::NodeHandle* n) {n_ = n; }
  void wait();
  void start();
  void loadArguers();

private:
  Ui::ontoloGUI *ui;
  ros::NodeHandle* n_;
  std::vector<std::string> arguers_names_;
  std::vector<std::string> arguers_description_;

  void displayUnClosed();
  void constructArguersCheckBoxs();
  size_t getArguerIndex(QCheckBoxExtended* box);
  std::string getArguerDescription(std::string box);
  void displayOntologiesList();
  void displayErrorInfo(std::string text);

  std::string vector2string(std::vector<std::string> vect);
  std::string vector2html(std::vector<std::string> vect);

public slots:
  void ClasshoverEnterSlot();
  void ClasshoverLeaveSlot();
  void objectPropertyhoverEnterSlot();
  void objectPropertyhoverLeaveSlot();
  void dataPropertyhoverEnterSlot();
  void dataPropertyhoverLeaveSlot();
  void IndividualCheckBoxhoverEnterSlot();
  void IndividualhoverEnterSlot();
  void IndividualhoverLeaveSlot();

  void classClickedSlot();
  void objectPropertyClickedSlot();
  void dataPropertyClickedSlot();
  void individualClickedSlot();
  void closeOntologySlot();
  void nameEditingFinishedSlot();
  void ArguerClickedSlot(int);
  void ArguerhoverEnterSlot();
  void ArguerhoverLeaveSlot();
  void currentTabChangedSlot(int);

  void displayOntologiesListSlot();
  void addOntologySlot();
  void deleteOntologySlot();
  void differenceOntologySlot();
};

#endif // ONTOLOGUI_H
