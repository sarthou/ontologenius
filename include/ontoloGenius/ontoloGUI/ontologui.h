#ifndef ONTOLOGUI_H
#define ONTOLOGUI_H

#include <QMainWindow>

#include "ros/ros.h"

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

private:
    Ui::ontoloGUI *ui;
    ros::NodeHandle* n_;

public slots:
  void ClasshoverEnterSlot();
  void ClasshoverLeaveSlot();
  void PropertyhoverEnterSlot();
  void PropertyhoverLeaveSlot();
  void IndividualhoverEnterSlot();
  void IndividualhoverLeaveSlot();
  void classClickedSlot();
  void propertyClickedSlot();
  void individualClickedSlot();
  void closeOntologySlot();
};

#endif // ONTOLOGUI_H
