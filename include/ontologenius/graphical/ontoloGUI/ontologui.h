#ifndef ONTOLOGUI_H
#define ONTOLOGUI_H

#include <QMainWindow>
#include "include/ontologenius/graphical/ontoloGUI/QCheckBoxExtended.h"
#include <QTextCursor>

#include "ontologenius/OntologiesManipulator.h"
#include <ros/ros.h>
#include <vector>
#include <string>

#include "std_msgs/String.h"

namespace Ui {
class ontoloGUI;
}

class ontoloGUI : public QMainWindow
{
    Q_OBJECT

public:
  explicit ontoloGUI(QWidget *parent = 0);
  ~ontoloGUI();

  void init();
  void wait();
  void start();
  void loadReasoners();

private:
  Ui::ontoloGUI *ui;

  onto::OntologiesManipulator ontos_;
  onto::OntologyManipulator* onto_;
  bool multi_usage_;

  std::string feeder_notifications_;

  std::vector<std::string> reasoners_names_;
  std::vector<std::string> reasoners_description_;

  void displayUnClosed();
  void constructReasonersCheckBoxs();
  void updateReasonersCheckBoxs();
  size_t getReasonerIndex(QCheckBoxExtended* box);
  std::string getReasonerDescription(std::string box);
  void displayOntologiesList();
  void displayErrorInfo(const std::string& text);

  std::string vector2string(const std::vector<std::string>& vect);
  std::string vector2html(const std::vector<std::string>& vect);

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
  void ReasonerClickedSlot(int);
  void ReasonerhoverEnterSlot();
  void ReasonerhoverLeaveSlot();
  void currentTabChangedSlot(int);

  void displayOntologiesListSlot();
  void addOntologySlot();
  void deleteOntologySlot();
  void saveOntologySlot();
  void differenceOntologySlot();
  void OntologyNameAddDelChangedSlot(const QString&);
  void OntologyNameChangedSlot(const QString&);

  void feederCallback(const std::string& msg);
  void feederAddSlot();
  void feederDelSlot();
  void feederCommitSlot();
  void feederCheckoutSlot();

  bool updateOntoPtr();

signals:
  void feederSetHtmlSignal(QString);
};

#endif // ONTOLOGUI_H
