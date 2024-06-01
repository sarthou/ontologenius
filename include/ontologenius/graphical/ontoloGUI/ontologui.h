#ifndef ONTOLOGUI_H
#define ONTOLOGUI_H

#include <QMainWindow>
#include <QTextCursor>
#include <string>
#include <vector>

#include "include/ontologenius/graphical/ontoloGUI/QCheckBoxExtended.h"
#include "ontologenius/OntologiesManipulator.h"

namespace Ui {
  class OntoloGUI;
}

class OntoloGUI : public QMainWindow
{
  Q_OBJECT

public:
  explicit OntoloGUI(QWidget* parent = nullptr);
  ~OntoloGUI() override;

  void init();
  void wait();
  void start();
  void loadReasoners();

private:
  Ui::OntoloGUI* ui_;

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
  std::string getReasonerDescription(const std::string& box);
  void displayOntologiesList();
  void displayErrorInfo(const std::string& text);

  static std::string vector2string(const std::vector<std::string>& vect);
  static std::string vector2html(const std::vector<std::string>& vect);

public slots:
  void classhoverEnterSlot();
  void classhoverLeaveSlot();
  void objectPropertyhoverEnterSlot();
  void objectPropertyhoverLeaveSlot();
  void dataPropertyhoverEnterSlot();
  void dataPropertyhoverLeaveSlot();
  void individualCheckBoxhoverEnterSlot();
  void individualhoverEnterSlot();
  void individualhoverLeaveSlot();

  void classClickedSlot();
  void objectPropertyClickedSlot();
  void dataPropertyClickedSlot();
  void individualClickedSlot();
  void closeOntologySlot();
  void nameEditingFinishedSlot();
  void reasonerClickedSlot(int);
  void reasonerhoverEnterSlot();
  void reasonerhoverLeaveSlot();
  void currentTabChangedSlot(int);

  void displayOntologiesListSlot();
  void addOntologySlot();
  void deleteOntologySlot();
  void saveOntologySlot();
  void differenceOntologySlot();
  void ontologyNameAddDelChangedSlot(const QString&);
  void ontologyNameChangedSlot(const QString&);

  void feederCallback(const std::string& msg);
  void feederAddSlot();
  void feederDelSlot();
  void feederCommitSlot();
  void feederCheckoutSlot();

  bool updateOntoPtr();

signals:
  void feederSetHtmlSignal(QString);
  void feederScrollSignal(QString);
};

#endif // ONTOLOGUI_H
