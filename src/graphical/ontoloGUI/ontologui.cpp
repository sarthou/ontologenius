#include "ontologenius/graphical/ontoloGUI/ontologui.h"

#include <QMainWindow>
#include <QObject>
#include <QScrollBar>
#include <QWidget>
#include <regex>

#include "ontologenius/graphical/ontoloGUI/QLineEditExtended.h"
#include "ontologenius/graphical/ontoloGUI/qpushbuttonextended.h"
#include "ui_ontologui.h"

OntoloGUI::OntoloGUI(QWidget* parent) : QMainWindow(parent),
                                        ui(new Ui::OntoloGUI)
{
  ui->setupUi(this);

  QObject::connect(ui->Class_getName, SIGNAL(hoverEnter()), this, SLOT(classhoverEnterSlot()));
  QObject::connect(ui->Class_getName, SIGNAL(hoverLeave()), this, SLOT(classhoverLeaveSlot()));
  QObject::connect(ui->Class_exist, SIGNAL(hoverEnter()), this, SLOT(classhoverEnterSlot()));
  QObject::connect(ui->Class_exist, SIGNAL(hoverLeave()), this, SLOT(classhoverLeaveSlot()));
  QObject::connect(ui->Class_findRegex, SIGNAL(hoverEnter()), this, SLOT(classhoverEnterSlot()));
  QObject::connect(ui->Class_findRegex, SIGNAL(hoverLeave()), this, SLOT(classhoverLeaveSlot()));
  QObject::connect(ui->Class_find, SIGNAL(hoverEnter()), this, SLOT(classhoverEnterSlot()));
  QObject::connect(ui->Class_find, SIGNAL(hoverLeave()), this, SLOT(classhoverLeaveSlot()));
  QObject::connect(ui->Class_findFuzzy, SIGNAL(hoverEnter()), this, SLOT(classhoverEnterSlot()));
  QObject::connect(ui->Class_findFuzzy, SIGNAL(hoverLeave()), this, SLOT(classhoverLeaveSlot()));
  QObject::connect(ui->Class_findSub, SIGNAL(hoverEnter()), this, SLOT(classhoverEnterSlot()));
  QObject::connect(ui->Class_findSub, SIGNAL(hoverLeave()), this, SLOT(classhoverLeaveSlot()));
  QObject::connect(ui->Class_getUp, SIGNAL(hoverEnter()), this, SLOT(classhoverEnterSlot()));
  QObject::connect(ui->Class_getUp, SIGNAL(hoverLeave()), this, SLOT(classhoverLeaveSlot()));
  QObject::connect(ui->Class_getDown, SIGNAL(hoverEnter()), this, SLOT(classhoverEnterSlot()));
  QObject::connect(ui->Class_getDown, SIGNAL(hoverLeave()), this, SLOT(classhoverLeaveSlot()));
  QObject::connect(ui->Class_getDisjoint, SIGNAL(hoverEnter()), this, SLOT(classhoverEnterSlot()));
  QObject::connect(ui->Class_getDisjoint, SIGNAL(hoverLeave()), this, SLOT(classhoverLeaveSlot()));
  QObject::connect(ui->Class_getRelatedOn, SIGNAL(hoverEnter()), this, SLOT(classhoverEnterSlot()));
  QObject::connect(ui->Class_getRelatedOn, SIGNAL(hoverLeave()), this, SLOT(classhoverLeaveSlot()));
  QObject::connect(ui->Class_getRelationOn, SIGNAL(hoverEnter()), this, SLOT(classhoverEnterSlot()));
  QObject::connect(ui->Class_getRelationOn, SIGNAL(hoverLeave()), this, SLOT(classhoverLeaveSlot()));
  QObject::connect(ui->Class_getRelatedFrom, SIGNAL(hoverEnter()), this, SLOT(classhoverEnterSlot()));
  QObject::connect(ui->Class_getRelatedFrom, SIGNAL(hoverLeave()), this, SLOT(classhoverLeaveSlot()));
  QObject::connect(ui->Class_getRelatedWith, SIGNAL(hoverEnter()), this, SLOT(classhoverEnterSlot()));
  QObject::connect(ui->Class_getRelatedWith, SIGNAL(hoverLeave()), this, SLOT(classhoverLeaveSlot()));
  QObject::connect(ui->Class_getRelationFrom, SIGNAL(hoverEnter()), this, SLOT(classhoverEnterSlot()));
  QObject::connect(ui->Class_getRelationFrom, SIGNAL(hoverLeave()), this, SLOT(classhoverLeaveSlot()));
  QObject::connect(ui->Class_getRelationWith, SIGNAL(hoverEnter()), this, SLOT(classhoverEnterSlot()));
  QObject::connect(ui->Class_getRelationWith, SIGNAL(hoverLeave()), this, SLOT(classhoverLeaveSlot()));
  QObject::connect(ui->Class_getFrom, SIGNAL(hoverEnter()), this, SLOT(classhoverEnterSlot()));
  QObject::connect(ui->Class_getFrom, SIGNAL(hoverLeave()), this, SLOT(classhoverLeaveSlot()));
  QObject::connect(ui->Class_getOn, SIGNAL(hoverEnter()), this, SLOT(classhoverEnterSlot()));
  QObject::connect(ui->Class_getOn, SIGNAL(hoverLeave()), this, SLOT(classhoverLeaveSlot()));
  QObject::connect(ui->Class_getWith, SIGNAL(hoverEnter()), this, SLOT(classhoverEnterSlot()));
  QObject::connect(ui->Class_getWith, SIGNAL(hoverLeave()), this, SLOT(classhoverLeaveSlot()));
  QObject::connect(ui->Class_getDomainOf, SIGNAL(hoverEnter()), this, SLOT(classhoverEnterSlot()));
  QObject::connect(ui->Class_getDomainOf, SIGNAL(hoverLeave()), this, SLOT(classhoverLeaveSlot()));
  QObject::connect(ui->Class_getRangeOf, SIGNAL(hoverEnter()), this, SLOT(classhoverEnterSlot()));
  QObject::connect(ui->Class_getRangeOf, SIGNAL(hoverLeave()), this, SLOT(classhoverLeaveSlot()));
  QObject::connect(ui->classParameter, SIGNAL(hoverEnter()), this, SLOT(classhoverEnterSlot()));
  QObject::connect(ui->classParameter, SIGNAL(hoverLeave()), this, SLOT(classhoverLeaveSlot()));

  QObject::connect(ui->ObjectProperty_getName, SIGNAL(hoverEnter()), this, SLOT(objectPropertyhoverEnterSlot()));
  QObject::connect(ui->ObjectProperty_getName, SIGNAL(hoverLeave()), this, SLOT(objectPropertyhoverLeaveSlot()));
  QObject::connect(ui->ObjectProperty_exist, SIGNAL(hoverEnter()), this, SLOT(objectPropertyhoverEnterSlot()));
  QObject::connect(ui->ObjectProperty_exist, SIGNAL(hoverLeave()), this, SLOT(objectPropertyhoverLeaveSlot()));
  QObject::connect(ui->ObjectProperty_findRegex, SIGNAL(hoverEnter()), this, SLOT(objectPropertyhoverEnterSlot()));
  QObject::connect(ui->ObjectProperty_findRegex, SIGNAL(hoverLeave()), this, SLOT(objectPropertyhoverLeaveSlot()));
  QObject::connect(ui->ObjectProperty_find, SIGNAL(hoverEnter()), this, SLOT(objectPropertyhoverEnterSlot()));
  QObject::connect(ui->ObjectProperty_find, SIGNAL(hoverLeave()), this, SLOT(objectPropertyhoverLeaveSlot()));
  QObject::connect(ui->ObjectProperty_findFuzzy, SIGNAL(hoverEnter()), this, SLOT(objectPropertyhoverEnterSlot()));
  QObject::connect(ui->ObjectProperty_findFuzzy, SIGNAL(hoverLeave()), this, SLOT(objectPropertyhoverLeaveSlot()));
  QObject::connect(ui->ObjectProperty_findSub, SIGNAL(hoverEnter()), this, SLOT(objectPropertyhoverEnterSlot()));
  QObject::connect(ui->ObjectProperty_findSub, SIGNAL(hoverLeave()), this, SLOT(objectPropertyhoverLeaveSlot()));
  QObject::connect(ui->ObjectProperty_getDisjoint, SIGNAL(hoverEnter()), this, SLOT(objectPropertyhoverEnterSlot()));
  QObject::connect(ui->ObjectProperty_getDisjoint, SIGNAL(hoverLeave()), this, SLOT(objectPropertyhoverLeaveSlot()));
  QObject::connect(ui->ObjectProperty_getDomain, SIGNAL(hoverEnter()), this, SLOT(objectPropertyhoverEnterSlot()));
  QObject::connect(ui->ObjectProperty_getDomain, SIGNAL(hoverLeave()), this, SLOT(objectPropertyhoverLeaveSlot()));
  QObject::connect(ui->ObjectProperty_getDown, SIGNAL(hoverEnter()), this, SLOT(objectPropertyhoverEnterSlot()));
  QObject::connect(ui->ObjectProperty_getDown, SIGNAL(hoverLeave()), this, SLOT(objectPropertyhoverLeaveSlot()));
  QObject::connect(ui->ObjectProperty_getInverse, SIGNAL(hoverEnter()), this, SLOT(objectPropertyhoverEnterSlot()));
  QObject::connect(ui->ObjectProperty_getInverse, SIGNAL(hoverLeave()), this, SLOT(objectPropertyhoverLeaveSlot()));
  QObject::connect(ui->ObjectProperty_getRange, SIGNAL(hoverEnter()), this, SLOT(objectPropertyhoverEnterSlot()));
  QObject::connect(ui->ObjectProperty_getRange, SIGNAL(hoverLeave()), this, SLOT(objectPropertyhoverLeaveSlot()));
  QObject::connect(ui->ObjectProperty_getUp, SIGNAL(hoverEnter()), this, SLOT(objectPropertyhoverEnterSlot()));
  QObject::connect(ui->ObjectProperty_getUp, SIGNAL(hoverLeave()), this, SLOT(objectPropertyhoverLeaveSlot()));
  QObject::connect(ui->objectPropertyParameter, SIGNAL(hoverEnter()), this, SLOT(objectPropertyhoverEnterSlot()));
  QObject::connect(ui->objectPropertyParameter, SIGNAL(hoverLeave()), this, SLOT(objectPropertyhoverLeaveSlot()));

  QObject::connect(ui->DataProperty_getName, SIGNAL(hoverEnter()), this, SLOT(dataPropertyhoverEnterSlot()));
  QObject::connect(ui->DataProperty_getName, SIGNAL(hoverLeave()), this, SLOT(dataPropertyhoverLeaveSlot()));
  QObject::connect(ui->DataProperty_exist, SIGNAL(hoverEnter()), this, SLOT(dataPropertyhoverEnterSlot()));
  QObject::connect(ui->DataProperty_exist, SIGNAL(hoverLeave()), this, SLOT(dataPropertyhoverLeaveSlot()));
  QObject::connect(ui->DataProperty_findRegex, SIGNAL(hoverEnter()), this, SLOT(dataPropertyhoverEnterSlot()));
  QObject::connect(ui->DataProperty_findRegex, SIGNAL(hoverLeave()), this, SLOT(dataPropertyhoverLeaveSlot()));
  QObject::connect(ui->DataProperty_find, SIGNAL(hoverEnter()), this, SLOT(dataPropertyhoverEnterSlot()));
  QObject::connect(ui->DataProperty_find, SIGNAL(hoverLeave()), this, SLOT(dataPropertyhoverLeaveSlot()));
  QObject::connect(ui->DataProperty_findFuzzy, SIGNAL(hoverEnter()), this, SLOT(dataPropertyhoverEnterSlot()));
  QObject::connect(ui->DataProperty_findFuzzy, SIGNAL(hoverLeave()), this, SLOT(dataPropertyhoverLeaveSlot()));
  QObject::connect(ui->DataProperty_findSub, SIGNAL(hoverEnter()), this, SLOT(dataPropertyhoverEnterSlot()));
  QObject::connect(ui->DataProperty_findSub, SIGNAL(hoverLeave()), this, SLOT(dataPropertyhoverLeaveSlot()));
  QObject::connect(ui->DataProperty_getDisjoint, SIGNAL(hoverEnter()), this, SLOT(dataPropertyhoverEnterSlot()));
  QObject::connect(ui->DataProperty_getDisjoint, SIGNAL(hoverLeave()), this, SLOT(dataPropertyhoverLeaveSlot()));
  QObject::connect(ui->DataProperty_getDomain, SIGNAL(hoverEnter()), this, SLOT(dataPropertyhoverEnterSlot()));
  QObject::connect(ui->DataProperty_getDomain, SIGNAL(hoverLeave()), this, SLOT(dataPropertyhoverLeaveSlot()));
  QObject::connect(ui->DataProperty_getDown, SIGNAL(hoverEnter()), this, SLOT(dataPropertyhoverEnterSlot()));
  QObject::connect(ui->DataProperty_getDown, SIGNAL(hoverLeave()), this, SLOT(dataPropertyhoverLeaveSlot()));
  QObject::connect(ui->DataProperty_getRange, SIGNAL(hoverEnter()), this, SLOT(dataPropertyhoverEnterSlot()));
  QObject::connect(ui->DataProperty_getRange, SIGNAL(hoverLeave()), this, SLOT(dataPropertyhoverLeaveSlot()));
  QObject::connect(ui->DataProperty_getUp, SIGNAL(hoverEnter()), this, SLOT(dataPropertyhoverEnterSlot()));
  QObject::connect(ui->DataProperty_getUp, SIGNAL(hoverLeave()), this, SLOT(dataPropertyhoverLeaveSlot()));
  QObject::connect(ui->dataPropertyParameter, SIGNAL(hoverEnter()), this, SLOT(dataPropertyhoverEnterSlot()));
  QObject::connect(ui->dataPropertyParameter, SIGNAL(hoverLeave()), this, SLOT(dataPropertyhoverLeaveSlot()));

  QObject::connect(ui->Individual_getDistinct, SIGNAL(hoverEnter()), this, SLOT(individualhoverEnterSlot()));
  QObject::connect(ui->Individual_getDistinct, SIGNAL(hoverLeave()), this, SLOT(individualhoverLeaveSlot()));
  QObject::connect(ui->Individual_getRelatedFrom, SIGNAL(hoverEnter()), this, SLOT(individualhoverEnterSlot()));
  QObject::connect(ui->Individual_getRelatedFrom, SIGNAL(hoverLeave()), this, SLOT(individualhoverLeaveSlot()));
  QObject::connect(ui->Individual_getRelatedOn, SIGNAL(hoverEnter()), this, SLOT(individualhoverEnterSlot()));
  QObject::connect(ui->Individual_getRelatedOn, SIGNAL(hoverLeave()), this, SLOT(individualhoverLeaveSlot()));
  QObject::connect(ui->Individual_getRelatedWith, SIGNAL(hoverEnter()), this, SLOT(individualhoverEnterSlot()));
  QObject::connect(ui->Individual_getRelatedWith, SIGNAL(hoverLeave()), this, SLOT(individualhoverLeaveSlot()));
  QObject::connect(ui->Individual_getRelationFrom, SIGNAL(hoverEnter()), this, SLOT(individualhoverEnterSlot()));
  QObject::connect(ui->Individual_getRelationFrom, SIGNAL(hoverLeave()), this, SLOT(individualhoverLeaveSlot()));
  QObject::connect(ui->Individual_getRelationOn, SIGNAL(hoverEnter()), this, SLOT(individualhoverEnterSlot()));
  QObject::connect(ui->Individual_getRelationOn, SIGNAL(hoverLeave()), this, SLOT(individualhoverLeaveSlot()));
  QObject::connect(ui->Individual_getRelationWith, SIGNAL(hoverEnter()), this, SLOT(individualhoverEnterSlot()));
  QObject::connect(ui->Individual_getRelationWith, SIGNAL(hoverLeave()), this, SLOT(individualhoverLeaveSlot()));
  QObject::connect(ui->Individual_getSame, SIGNAL(hoverEnter()), this, SLOT(individualhoverEnterSlot()));
  QObject::connect(ui->Individual_getSame, SIGNAL(hoverLeave()), this, SLOT(individualhoverLeaveSlot()));
  QObject::connect(ui->Individual_getUp, SIGNAL(hoverEnter()), this, SLOT(individualhoverEnterSlot()));
  QObject::connect(ui->Individual_getUp, SIGNAL(hoverLeave()), this, SLOT(individualhoverLeaveSlot()));
  QObject::connect(ui->Individual_getOn, SIGNAL(hoverEnter()), this, SLOT(individualhoverEnterSlot()));
  QObject::connect(ui->Individual_getOn, SIGNAL(hoverLeave()), this, SLOT(individualhoverLeaveSlot()));
  QObject::connect(ui->Individual_getFrom, SIGNAL(hoverEnter()), this, SLOT(individualhoverEnterSlot()));
  QObject::connect(ui->Individual_getFrom, SIGNAL(hoverLeave()), this, SLOT(individualhoverLeaveSlot()));
  QObject::connect(ui->Individual_getWith, SIGNAL(hoverEnter()), this, SLOT(individualhoverEnterSlot()));
  QObject::connect(ui->Individual_getWith, SIGNAL(hoverLeave()), this, SLOT(individualhoverLeaveSlot()));
  QObject::connect(ui->Individual_getDomainOf, SIGNAL(hoverEnter()), this, SLOT(individualhoverEnterSlot()));
  QObject::connect(ui->Individual_getDomainOf, SIGNAL(hoverLeave()), this, SLOT(individualhoverLeaveSlot()));
  QObject::connect(ui->Individual_getRangeOf, SIGNAL(hoverEnter()), this, SLOT(individualhoverEnterSlot()));
  QObject::connect(ui->Individual_getRangeOf, SIGNAL(hoverLeave()), this, SLOT(individualhoverLeaveSlot()));
  QObject::connect(ui->Individual_getName, SIGNAL(hoverEnter()), this, SLOT(individualhoverEnterSlot()));
  QObject::connect(ui->Individual_getName, SIGNAL(hoverLeave()), this, SLOT(individualhoverLeaveSlot()));
  QObject::connect(ui->Individual_exist, SIGNAL(hoverEnter()), this, SLOT(individualhoverEnterSlot()));
  QObject::connect(ui->Individual_exist, SIGNAL(hoverLeave()), this, SLOT(individualhoverLeaveSlot()));
  QObject::connect(ui->Individual_findRegex, SIGNAL(hoverEnter()), this, SLOT(individualhoverEnterSlot()));
  QObject::connect(ui->Individual_findRegex, SIGNAL(hoverLeave()), this, SLOT(individualhoverLeaveSlot()));
  QObject::connect(ui->Individual_find, SIGNAL(hoverEnter()), this, SLOT(individualhoverEnterSlot()));
  QObject::connect(ui->Individual_find, SIGNAL(hoverLeave()), this, SLOT(individualhoverLeaveSlot()));
  QObject::connect(ui->Individual_findFuzzy, SIGNAL(hoverEnter()), this, SLOT(individualhoverEnterSlot()));
  QObject::connect(ui->Individual_findFuzzy, SIGNAL(hoverLeave()), this, SLOT(individualhoverLeaveSlot()));
  QObject::connect(ui->Individual_findSub, SIGNAL(hoverEnter()), this, SLOT(individualhoverEnterSlot()));
  QObject::connect(ui->Individual_findSub, SIGNAL(hoverLeave()), this, SLOT(individualhoverLeaveSlot()));
  QObject::connect(ui->Individual_getType, SIGNAL(hoverEnter()), this, SLOT(individualhoverEnterSlot()));
  QObject::connect(ui->Individual_getType, SIGNAL(hoverLeave()), this, SLOT(individualhoverLeaveSlot()));
  QObject::connect(ui->individualParameter, SIGNAL(hoverEnter()), this, SLOT(individualhoverEnterSlot()));
  QObject::connect(ui->individualParameter, SIGNAL(hoverLeave()), this, SLOT(individualhoverLeaveSlot()));

  QObject::connect(ui->Class_getName, SIGNAL(clicked()), this, SLOT(classClickedSlot()));
  QObject::connect(ui->Class_exist, SIGNAL(clicked()), this, SLOT(classClickedSlot()));
  QObject::connect(ui->Class_findRegex, SIGNAL(clicked()), this, SLOT(classClickedSlot()));
  QObject::connect(ui->Class_find, SIGNAL(clicked()), this, SLOT(classClickedSlot()));
  QObject::connect(ui->Class_findSub, SIGNAL(clicked()), this, SLOT(classClickedSlot()));
  QObject::connect(ui->Class_findFuzzy, SIGNAL(clicked()), this, SLOT(classClickedSlot()));
  QObject::connect(ui->Class_getUp, SIGNAL(clicked()), this, SLOT(classClickedSlot()));
  QObject::connect(ui->Class_getDown, SIGNAL(clicked()), this, SLOT(classClickedSlot()));
  QObject::connect(ui->Class_getDisjoint, SIGNAL(clicked()), this, SLOT(classClickedSlot()));
  QObject::connect(ui->Class_getRelatedOn, SIGNAL(clicked()), this, SLOT(classClickedSlot()));
  QObject::connect(ui->Class_getRelationOn, SIGNAL(clicked()), this, SLOT(classClickedSlot()));
  QObject::connect(ui->Class_getRelatedFrom, SIGNAL(clicked()), this, SLOT(classClickedSlot()));
  QObject::connect(ui->Class_getRelatedWith, SIGNAL(clicked()), this, SLOT(classClickedSlot()));
  QObject::connect(ui->Class_getRelationFrom, SIGNAL(clicked()), this, SLOT(classClickedSlot()));
  QObject::connect(ui->Class_getRelationWith, SIGNAL(clicked()), this, SLOT(classClickedSlot()));
  QObject::connect(ui->Class_getFrom, SIGNAL(clicked()), this, SLOT(classClickedSlot()));
  QObject::connect(ui->Class_getOn, SIGNAL(clicked()), this, SLOT(classClickedSlot()));
  QObject::connect(ui->Class_getWith, SIGNAL(clicked()), this, SLOT(classClickedSlot()));
  QObject::connect(ui->Class_getDomainOf, SIGNAL(clicked()), this, SLOT(classClickedSlot()));
  QObject::connect(ui->Class_getRangeOf, SIGNAL(clicked()), this, SLOT(classClickedSlot()));

  QObject::connect(ui->ObjectProperty_getName, SIGNAL(clicked()), this, SLOT(objectPropertyClickedSlot()));
  QObject::connect(ui->ObjectProperty_exist, SIGNAL(clicked()), this, SLOT(objectPropertyClickedSlot()));
  QObject::connect(ui->ObjectProperty_findRegex, SIGNAL(clicked()), this, SLOT(objectPropertyClickedSlot()));
  QObject::connect(ui->ObjectProperty_find, SIGNAL(clicked()), this, SLOT(objectPropertyClickedSlot()));
  QObject::connect(ui->ObjectProperty_findFuzzy, SIGNAL(clicked()), this, SLOT(objectPropertyClickedSlot()));
  QObject::connect(ui->ObjectProperty_findSub, SIGNAL(clicked()), this, SLOT(objectPropertyClickedSlot()));
  QObject::connect(ui->ObjectProperty_getDisjoint, SIGNAL(clicked()), this, SLOT(objectPropertyClickedSlot()));
  QObject::connect(ui->ObjectProperty_getDomain, SIGNAL(clicked()), this, SLOT(objectPropertyClickedSlot()));
  QObject::connect(ui->ObjectProperty_getDown, SIGNAL(clicked()), this, SLOT(objectPropertyClickedSlot()));
  QObject::connect(ui->ObjectProperty_getInverse, SIGNAL(clicked()), this, SLOT(objectPropertyClickedSlot()));
  QObject::connect(ui->ObjectProperty_getRange, SIGNAL(clicked()), this, SLOT(objectPropertyClickedSlot()));
  QObject::connect(ui->ObjectProperty_getUp, SIGNAL(clicked()), this, SLOT(objectPropertyClickedSlot()));

  QObject::connect(ui->DataProperty_getName, SIGNAL(clicked()), this, SLOT(dataPropertyClickedSlot()));
  QObject::connect(ui->DataProperty_exist, SIGNAL(clicked()), this, SLOT(dataPropertyClickedSlot()));
  QObject::connect(ui->DataProperty_findRegex, SIGNAL(clicked()), this, SLOT(dataPropertyClickedSlot()));
  QObject::connect(ui->DataProperty_find, SIGNAL(clicked()), this, SLOT(dataPropertyClickedSlot()));
  QObject::connect(ui->DataProperty_findFuzzy, SIGNAL(clicked()), this, SLOT(dataPropertyClickedSlot()));
  QObject::connect(ui->DataProperty_findSub, SIGNAL(clicked()), this, SLOT(dataPropertyClickedSlot()));
  QObject::connect(ui->DataProperty_getDisjoint, SIGNAL(clicked()), this, SLOT(dataPropertyClickedSlot()));
  QObject::connect(ui->DataProperty_getDomain, SIGNAL(clicked()), this, SLOT(dataPropertyClickedSlot()));
  QObject::connect(ui->DataProperty_getDown, SIGNAL(clicked()), this, SLOT(dataPropertyClickedSlot()));
  QObject::connect(ui->DataProperty_getRange, SIGNAL(clicked()), this, SLOT(dataPropertyClickedSlot()));
  QObject::connect(ui->DataProperty_getUp, SIGNAL(clicked()), this, SLOT(dataPropertyClickedSlot()));

  QObject::connect(ui->Individual_getDistinct, SIGNAL(clicked()), this, SLOT(individualClickedSlot()));
  QObject::connect(ui->Individual_getRelatedFrom, SIGNAL(clicked()), this, SLOT(individualClickedSlot()));
  QObject::connect(ui->Individual_getRelatedOn, SIGNAL(clicked()), this, SLOT(individualClickedSlot()));
  QObject::connect(ui->Individual_getRelatedWith, SIGNAL(clicked()), this, SLOT(individualClickedSlot()));
  QObject::connect(ui->Individual_getRelationFrom, SIGNAL(clicked()), this, SLOT(individualClickedSlot()));
  QObject::connect(ui->Individual_getRelationOn, SIGNAL(clicked()), this, SLOT(individualClickedSlot()));
  QObject::connect(ui->Individual_getRelationWith, SIGNAL(clicked()), this, SLOT(individualClickedSlot()));
  QObject::connect(ui->Individual_getSame, SIGNAL(clicked()), this, SLOT(individualClickedSlot()));
  QObject::connect(ui->Individual_getUp, SIGNAL(clicked()), this, SLOT(individualClickedSlot()));
  QObject::connect(ui->Individual_getOn, SIGNAL(clicked()), this, SLOT(individualClickedSlot()));
  QObject::connect(ui->Individual_getFrom, SIGNAL(clicked()), this, SLOT(individualClickedSlot()));
  QObject::connect(ui->Individual_getWith, SIGNAL(clicked()), this, SLOT(individualClickedSlot()));
  QObject::connect(ui->Individual_getDomainOf, SIGNAL(clicked()), this, SLOT(individualClickedSlot()));
  QObject::connect(ui->Individual_getRangeOf, SIGNAL(clicked()), this, SLOT(individualClickedSlot()));
  QObject::connect(ui->Individual_getName, SIGNAL(clicked()), this, SLOT(individualClickedSlot()));
  QObject::connect(ui->Individual_exist, SIGNAL(clicked()), this, SLOT(individualClickedSlot()));
  QObject::connect(ui->Individual_findRegex, SIGNAL(clicked()), this, SLOT(individualClickedSlot()));
  QObject::connect(ui->Individual_find, SIGNAL(clicked()), this, SLOT(individualClickedSlot()));
  QObject::connect(ui->Individual_findFuzzy, SIGNAL(clicked()), this, SLOT(individualClickedSlot()));
  QObject::connect(ui->Individual_findSub, SIGNAL(clicked()), this, SLOT(individualClickedSlot()));
  QObject::connect(ui->Individual_getType, SIGNAL(clicked()), this, SLOT(individualClickedSlot()));

  QObject::connect(ui->CloseButton, SIGNAL(clicked()), this, SLOT(closeOntologySlot()));
  QObject::connect(ui->RefreshButton, SIGNAL(clicked()), this, SLOT(displayOntologiesListSlot()));
  QObject::connect(ui->AddPushButton, SIGNAL(clicked()), this, SLOT(addOntologySlot()));
  QObject::connect(ui->DelPushButton, SIGNAL(clicked()), this, SLOT(deleteOntologySlot()));
  QObject::connect(ui->SaveButton, SIGNAL(clicked()), this, SLOT(saveOntologySlot()));
  QObject::connect(ui->DiffPushButton, SIGNAL(clicked()), this, SLOT(differenceOntologySlot()));

  QObject::connect(ui->FeederAddButton, SIGNAL(clicked()), this, SLOT(feederAddSlot()));
  QObject::connect(ui->FeederDelButton, SIGNAL(clicked()), this, SLOT(feederDelSlot()));
  QObject::connect(ui->FeederCommitButton, SIGNAL(clicked()), this, SLOT(feederCommitSlot()));
  QObject::connect(ui->FeederCheckoutButton, SIGNAL(clicked()), this, SLOT(feederCheckoutSlot()));

  QObject::connect(ui->OntologyNameAddDel, SIGNAL(textEdited(const QString&)), this, SLOT(ontologyNameAddDelChangedSlot(const QString&)));
  QObject::connect(ui->OntologyName, SIGNAL(textEdited(const QString&)), this, SLOT(ontologyNameChangedSlot(const QString&)));
  QObject::connect(ui->OntologyName, SIGNAL(editingFinished()), this, SLOT(nameEditingFinishedSlot()));
  QObject::connect(ui->tabWidget, SIGNAL(currentChanged(int)), this, SLOT(currentTabChangedSlot(int)));

  QObject::connect(this, SIGNAL(feederSetHtmlSignal(QString)), ui->FeederInfo, SLOT(setHtml(QString)), Qt::BlockingQueuedConnection);
  QObject::connect(this, SIGNAL(feederScrollSignal(QString)), ui->FeederInfo, SLOT(scrollToAnchor(QString)), Qt::BlockingQueuedConnection);
}

OntoloGUI::~OntoloGUI()
{
  delete ui;
}

void OntoloGUI::init()
{
  if(ontos_.waitInit(1) == false)
  {
    onto_ = new onto::OntologyManipulator();
    onto_->feeder.registerFeederNotificationCallback([this](auto msg) { this->feederCallback(msg); });
    multi_usage_ = false;
  }
  else
  {
    onto_ = nullptr;
    multi_usage_ = true;
  }
}

void OntoloGUI::wait()
{
  const QString html = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
                       "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">"
                       "p, li { white-space: pre-wrap; }"
                       "</style></head><body style=\" font-family:'Noto Sans'; font-size:9pt; font-weight:400; font-style:normal;\">"
                       "<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; color:#a40000;\">Wainting for </span><span style=\" font-size:12pt; font-weight:600; color:#a40000;\">ontologenius</span></p></body></html>";
  ui->InfoArea->setHtml(html);
}

void OntoloGUI::start()
{
  const QString html = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
                       "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">"
                       "p, li { white-space: pre-wrap; }"
                       "</style></head><body style=\" font-family:'Noto Sans'; font-size:9pt; font-weight:400; font-style:normal;\">"
                       "<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; font-weight:600; color:#4e9a06;\">OntoloGenius</span><span style=\" font-size:12pt; color:#4e9a06;\"> detected</span></p></body></html>";
  ui->InfoArea->setHtml(html);
}

void OntoloGUI::classhoverEnterSlot()
{
  ui->ClassDescription->setText(dynamic_cast<QWidget*>(sender())->whatsThis());
}

void OntoloGUI::classhoverLeaveSlot()
{
  ui->ClassDescription->setText("");
}

void OntoloGUI::objectPropertyhoverEnterSlot()
{
  ui->ObjectPropertyDescription->setText(dynamic_cast<QWidget*>(sender())->whatsThis());
}

void OntoloGUI::objectPropertyhoverLeaveSlot()
{
  ui->ObjectPropertyDescription->setText("");
}

void OntoloGUI::dataPropertyhoverEnterSlot()
{
  ui->DataPropertyDescription->setText(dynamic_cast<QWidget*>(sender())->whatsThis());
}

void OntoloGUI::dataPropertyhoverLeaveSlot()
{
  ui->DataPropertyDescription->setText("");
}

void OntoloGUI::individualCheckBoxhoverEnterSlot()
{
  ui->IndividualDescription->setText(dynamic_cast<QCheckBoxExtended*>(sender())->whatsThis());
}

void OntoloGUI::individualhoverEnterSlot()
{
  ui->IndividualDescription->setText(dynamic_cast<QWidget*>(sender())->whatsThis());
}

void OntoloGUI::individualhoverLeaveSlot()
{
  ui->IndividualDescription->setText("");
}

void OntoloGUI::classClickedSlot()
{
  if(ui->OntologyName->text().toStdString().find('=') != std::string::npos)
  {
    displayErrorInfo("ontology instance name cannot have the symbol = : \'" + ui->OntologyName->text().toStdString() + "\'\n Once an ontology instance copy has been performed, use the new ontology instance name.");
    return;
  }

  const std::string action = dynamic_cast<QPushButtonExtended*>(sender())->text().toStdString();
  const std::string param = ui->classParameter->text().toStdString();
  if(updateOntoPtr() == false)
    return;

  const QString text = dynamic_cast<QPushButtonExtended*>(sender())->text() + " : " + ui->classParameter->text();
  ui->ClassDescription->setText(text);

  auto res_vect = onto_->classes.call(action, param);
  const int err = onto_->classes.getErrorCode();
  if(err == -1)
    displayErrorInfo("client call failed");
  else
  {
    start();
    const std::string res_str = vector2string(res_vect);
    ui->ResultArea->setText(QString::fromStdString(res_str));
    if(err == 3)
      displayUnClosed();
  }
}

void OntoloGUI::objectPropertyClickedSlot()
{
  if(ui->OntologyName->text().toStdString().find('=') != std::string::npos)
  {
    displayErrorInfo("ontology instance name cannot have the symbol = : \'" + ui->OntologyName->text().toStdString() + "\'\n Once an ontology instance copy has been performed, use the new ontology instance name.");
    return;
  }

  const std::string action = dynamic_cast<QPushButtonExtended*>(sender())->text().toStdString();
  const std::string param = ui->objectPropertyParameter->text().toStdString();
  if(updateOntoPtr() == false)
    return;

  const QString text = dynamic_cast<QPushButtonExtended*>(sender())->text() + " : " + ui->objectPropertyParameter->text();
  ui->ObjectPropertyDescription->setText(text);

  auto res_vect = onto_->objectProperties.call(action, param);
  const int err = onto_->objectProperties.getErrorCode();
  if(err == -1)
    displayErrorInfo("client call failed");
  else
  {
    start();
    const std::string res_str = vector2string(res_vect);
    ui->ResultArea->setText(QString::fromStdString(res_str));
    if(err == 3)
      displayUnClosed();
  }
}

void OntoloGUI::dataPropertyClickedSlot()
{
  if(ui->OntologyName->text().toStdString().find('=') != std::string::npos)
  {
    displayErrorInfo("ontology instance name cannot have the symbol = : \'" + ui->OntologyName->text().toStdString() + "\'\n Once an ontology instance copy has been performed, use the new ontology instance name.");
    return;
  }

  const std::string action = dynamic_cast<QPushButtonExtended*>(sender())->text().toStdString();
  const std::string param = ui->dataPropertyParameter->text().toStdString();
  if(updateOntoPtr() == false)
    return;

  const QString text = dynamic_cast<QPushButtonExtended*>(sender())->text() + " : " + ui->dataPropertyParameter->text();
  ui->DataPropertyDescription->setText(text);

  auto res_vect = onto_->dataProperties.call(action, param);
  const int err = onto_->dataProperties.getErrorCode();
  if(err == -1)
    displayErrorInfo("client call failed");
  else
  {
    start();
    const std::string res_str = vector2string(res_vect);
    ui->ResultArea->setText(QString::fromStdString(res_str));
    if(err == 3)
      displayUnClosed();
  }
}

void OntoloGUI::individualClickedSlot()
{
  if(ui->OntologyName->text().toStdString().find('=') != std::string::npos)
  {
    displayErrorInfo("ontology instance name cannot have the symbol = : \'" + ui->OntologyName->text().toStdString() + "\'\n Once an ontology instance copy has been performed, use the new ontology instance name.");
    return;
  }

  const std::string action = dynamic_cast<QPushButtonExtended*>(sender())->text().toStdString();
  const std::string param = ui->individualParameter->text().toStdString();
  if(updateOntoPtr() == false)
    return;

  const QString text = dynamic_cast<QPushButtonExtended*>(sender())->text() + " : " + ui->individualParameter->text();
  ui->IndividualDescription->setText(text);

  auto res_vect = onto_->individuals.call(action, param);
  const int err = onto_->individuals.getErrorCode();
  if(err == -1)
    displayErrorInfo("client call failed");
  else
  {
    start();
    const std::string res_str = vector2string(res_vect);
    ui->ResultArea->setText(QString::fromStdString(res_str));
    if(err == 3)
      displayUnClosed();
  }
}

void OntoloGUI::closeOntologySlot()
{
  if(ui->OntologyName->text().toStdString().find('=') != std::string::npos)
  {
    displayErrorInfo("ontology instance name cannot have the symbol = : \'" + ui->OntologyName->text().toStdString() + "\'\n Once an ontology instance copy has been performed, use the new ontology instance name.");
    return;
  }

  if(updateOntoPtr() == false)
    return;

  onto_->close();
  const int err = onto_->actions.getErrorCode();
  if(err == -1)
    displayErrorInfo("client call failed");
  else
  {
    ui->ResultArea->setText("");
    start();
  }
}

void OntoloGUI::nameEditingFinishedSlot()
{
  if(updateOntoPtr() == false)
    return;
  loadReasoners();
}

void OntoloGUI::reasonerClickedSlot(int /*unused*/)
{
  if(ui->OntologyName->text().toStdString().find('=') != std::string::npos)
  {
    displayErrorInfo("ontology instance name cannot have the symbol = : \'" + ui->OntologyName->text().toStdString() + "\'\n Once an ontology instance copy has been performed, use the new ontology instance name.");
    return;
  }

  const std::string param = dynamic_cast<QCheckBoxExtended*>(sender())->text().toStdString();
  if(updateOntoPtr() == false)
    return;

  if(dynamic_cast<QCheckBoxExtended*>(sender())->isChecked())
    onto_->reasoners.activate(param);
  else
    onto_->reasoners.deactivate(param);

  const int err = onto_->reasoners.getErrorCode();
  if(err == -1)
    displayErrorInfo("client call failed");
  else
  {
    start();
    ui->ResultArea->setText("");
  }
}

void OntoloGUI::reasonerhoverEnterSlot()
{
  const size_t index = getReasonerIndex(dynamic_cast<QCheckBoxExtended*>(sender()));
  ui->ReasonerDescription->setText(QString::fromStdString(reasoners_description_[index]));
}

void OntoloGUI::reasonerhoverLeaveSlot()
{
  ui->ReasonerDescription->setText("");
}

void OntoloGUI::displayUnClosed()
{
  const QString html = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
                       "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">"
                       "p, li { white-space: pre-wrap; }"
                       "</style></head><body style=\" font-family:'Noto Sans'; font-size:9pt; font-weight:400; font-style:normal;\">"
                       "<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; color:#a40000;\">Ontology is not </span><span style=\" font-size:12pt; font-weight:600; color:#a40000;\">closed</span></p></body></html>";
  ui->InfoArea->setHtml(html);
}

void OntoloGUI::loadReasoners()
{
  if(ui->OntologyName->text().toStdString().find('=') != std::string::npos)
  {
    displayErrorInfo("ontology instance name cannot have the symbol = : \'" + ui->OntologyName->text().toStdString() + "\'\n Once an ontology instance copy has been performed, use the new ontology instance name.");
    return;
  }

  QLayoutItem* item = nullptr;
  while((item = ui->ReasonerListLayout->takeAt(1)) != nullptr)
  {
    delete item->widget();
    delete item;
  }

  if(updateOntoPtr() == false)
    return;

  auto res_vect = onto_->reasoners.list();
  const int err = onto_->reasoners.getErrorCode();

  if(err == -1)
    displayErrorInfo("client call failed");
  else
  {
    start();
    reasoners_names_ = res_vect;
    ui->ResultArea->setText("");

    constructReasonersCheckBoxs();
    updateReasonersCheckBoxs();
  }
}

void OntoloGUI::constructReasonersCheckBoxs()
{
  for(const auto& reasoners_name : reasoners_names_)
  {
    reasoners_description_.push_back(getReasonerDescription(reasoners_name));
    auto* box = new QCheckBoxExtended(QString::fromStdString(reasoners_name), this);
    ui->ReasonerListLayout->addWidget(box);
    QObject::connect(box, SIGNAL(stateChanged(int)), this, SLOT(reasonerClickedSlot(int)));
    QObject::connect(box, SIGNAL(hoverEnter()), this, SLOT(reasonerhoverEnterSlot()));
    QObject::connect(box, SIGNAL(hoverLeave()), this, SLOT(reasonerhoverLeaveSlot()));
  }
}

void OntoloGUI::updateReasonersCheckBoxs()
{
  if(ui->OntologyName->text().toStdString().find('=') != std::string::npos)
  {
    displayErrorInfo("ontology instance name cannot have the symbol = : \'" + ui->OntologyName->text().toStdString() + "\'\n Once an ontology instance copy has been performed, use the new ontology instance name.");
    return;
  }

  if(updateOntoPtr() == false)
    return;

  auto active_reasoners = onto_->reasoners.activeList();
  const int err = onto_->reasoners.getErrorCode();

  if(err == -1)
    displayErrorInfo("client call failed");
  else
  {
    for(int i = 1; i < ui->ReasonerListLayout->count(); ++i)
    {
      QWidget* widget = ui->ReasonerListLayout->itemAt(i)->widget();
      if(widget != nullptr)
      {
        Qt::CheckState checked = Qt::Unchecked;
        for(const auto& active_reasoner : active_reasoners)
        {
          if(dynamic_cast<QCheckBoxExtended*>(widget)->text().toStdString() == active_reasoner)
            checked = Qt::Checked;
        }
        QObject::disconnect(dynamic_cast<QCheckBoxExtended*>(widget), SIGNAL(stateChanged(int)), this, SLOT(reasonerClickedSlot(int)));
        dynamic_cast<QCheckBoxExtended*>(widget)->setCheckState(checked);
        QObject::connect(dynamic_cast<QCheckBoxExtended*>(widget), SIGNAL(stateChanged(int)), this, SLOT(reasonerClickedSlot(int)));
      }
    }
  }
}

size_t OntoloGUI::getReasonerIndex(QCheckBoxExtended* box)
{
  size_t index = 0;
  for(size_t i = 0; i < reasoners_names_.size(); i++)
    if(reasoners_names_[i] == box->text().toStdString())
    {
      index = i;
      break;
    }
  return index;
}

std::string OntoloGUI::getReasonerDescription(const std::string& box)
{
  if(ui->OntologyName->text().toStdString().find('=') != std::string::npos)
  {
    displayErrorInfo("ontology instance name cannot have the symbol = : \'" + ui->OntologyName->text().toStdString() + "\'\n Once an ontology instance copy has been performed, use the new ontology instance name.");
    return "";
  }

  if(updateOntoPtr() == false)
    return "";

  auto description = onto_->reasoners.getDescription(box);
  const int err = onto_->reasoners.getErrorCode();

  if(err == -1)
    displayErrorInfo("client call failed");
  else
    return description;

  return "";
}

void OntoloGUI::displayErrorInfo(const std::string& text)
{
  const std::string html = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
                           "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">"
                           "p, li { white-space: pre-wrap; }"
                           "</style></head><body style=\" font-family:'Noto Sans'; font-size:9pt; font-weight:400; font-style:normal;\">"
                           "<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; color:#a40000;\">" +
                           text + "</span></p></body></html>";
  ui->InfoArea->setHtml(QString::fromStdString(html));
}

void OntoloGUI::displayOntologiesList()
{
  auto res_vect = ontos_.list();
  const int err = ontos_.getErrorCode();
  std::string html;
  if(err == -1)
  {
    html = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
           "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">"
           "p, li { white-space: pre-wrap; }"
           "</style></head><body style=\" font-family:'Noto Sans'; font-size:9pt; font-weight:400; font-style:normal;\">"
           "<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; color:#a40000;\">ontologenius is not running in multi mode.</span></p></body></html>";
  }
  else
  {
    const std::string text = vector2html(res_vect);
    html = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
           "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">"
           "p, li { white-space: pre-wrap; }"
           "</style></head><body style=\" font-family:'Noto Sans'; font-size:9pt; font-weight:400; font-style:normal;\">"
           "<p align=\"left\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; \">" +
           text + "</span></p></body></html>";
  }
  ui->OntologiesList->setHtml(QString::fromStdString(html));
}

void OntoloGUI::displayOntologiesListSlot()
{
  displayOntologiesList();
}

std::string OntoloGUI::vector2string(const std::vector<std::string>& vect)
{
  std::string res;
  for(const auto& v : vect)
    res += v + "\n";
  return res;
}

std::string OntoloGUI::vector2html(const std::vector<std::string>& vect)
{
  std::string res;
  for(const auto& v : vect)
    res += " - " + v + "<br>";
  return res;
}

void OntoloGUI::currentTabChangedSlot(int index)
{
  if(index == 4)
    loadReasoners();
  else if(index == 6)
    displayOntologiesList();
}

void OntoloGUI::addOntologySlot()
{
  const std::string param = ui->OntologyNameAddDel->text().toStdString();
  std::string onto_name = param;

  const std::regex base_regex("(.*)=(.*)");
  std::smatch base_match;
  if(std::regex_match(param, base_match, base_regex))
  {
    if(base_match.size() == 3)
    {
      ontos_.copy(base_match[1].str(), base_match[2].str());
      onto_name = base_match[1].str();
    }
  }
  else
    ontos_.add(param);

  const int err = ontos_.getErrorCode();
  if(err == -1)
    displayErrorInfo("ontologenius/manage client call failed");
  else
  {
    start();
    if(err == 4)
      ui->ResultArea->setText(QString::fromStdString(param + " already created"));
    else if(err == 1)
      ui->ResultArea->setText(QString::fromStdString("fail to stop " + param + " : please retry"));
    else
    {
      ui->ResultArea->setText(QString::fromStdString(""));
      ontos_.get(onto_name)->feeder.registerFeederNotificationCallback([this](auto msg) { this->feederCallback(msg); });
    }

    displayOntologiesList();
  }
}

void OntoloGUI::deleteOntologySlot()
{
  const std::string param = ui->OntologyNameAddDel->text().toStdString();
  ontos_.del(param);
  const int err = ontos_.getErrorCode();

  if(err == -1)
    displayErrorInfo("ontologenius/manage client call failed");
  else
  {
    start();
    if(err == 4)
      ui->ResultArea->setText(QString::fromStdString(param + " don't exist"));
    else
      ui->ResultArea->setText(QString::fromStdString(""));
    displayOntologiesList();
  }
}

void OntoloGUI::saveOntologySlot()
{
  if(ui->OntologyName->text().toStdString().find('=') != std::string::npos)
  {
    displayErrorInfo("ontology instance name cannot have the symbol = : \'" + ui->OntologyName->text().toStdString() + "\'\n Once an ontology instance copy has been performed, use the new ontology instance name.");
    return;
  }

  if(updateOntoPtr() == false)
    return;

  const std::string param = ui->OntologSavePath->text().toStdString();
  onto_->actions.save(param);
  const int err = onto_->actions.getErrorCode();

  if(err == -1)
    displayErrorInfo("client call failed");
  else
  {
    if(err == 4)
      ui->ResultArea->setText(QString::fromStdString(param + " don't exist"));
    else
      ui->ResultArea->setText(QString::fromStdString(""));
  }
}

void OntoloGUI::differenceOntologySlot()
{
  const std::string param1 = ui->OntologyDiffName1->text().toStdString();
  const std::string param2 = ui->OntologyDiffName2->text().toStdString();
  const std::string concept = ui->OntologyDiffConcept->text().toStdString();

  auto diff = ontos_.getDifference(param1, param2, concept);
  const int err = ontos_.getErrorCode();

  if(err == -1)
    displayErrorInfo("ontologenius/manage client call failed");
  else
  {
    start();
    if(err == 4)
      ui->ResultArea->setText("no effect");
    else if(err == 0)
    {
      std::string res = vector2string(diff);
      ui->ResultArea->setText(QString::fromStdString(res));
    }
    else
      ui->ResultArea->setText(QString::fromStdString(std::to_string(err)));
    displayOntologiesList();
  }
}

void OntoloGUI::ontologyNameAddDelChangedSlot(const QString& text)
{
  if(ui->OntologyName->text() != text)
  {
    size_t equal_pose = text.toStdString().find('=');
    if(equal_pose != std::string::npos)
      ui->OntologyName->setText(text.mid(0, equal_pose));
    else
      ui->OntologyName->setText(text);
  }
}

void OntoloGUI::ontologyNameChangedSlot(const QString& text)
{
  if(ui->OntologyNameAddDel->text() != text)
    ui->OntologyNameAddDel->setText(text);
}

void OntoloGUI::feederCallback(const std::string& msg)
{
  feeder_notifications_ += "<p>-" + msg + "</p>";

  const std::string html = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
                           "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">"
                           "p, li { whicommitte-space: pre-wrap; }"
                           "</style></head><body style=\" font-family:'Noto Sans'; font-size:9pt; font-weight:400; font-style:normal;\">"
                           "<p align=\"left\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; \">" +
                           feeder_notifications_ + R"(<a name="scrollToMe" href="#scroll"></a> <br></span></p></body></html>)";

  feederSetHtmlSignal(QString::fromStdString(html));
  feederScrollSignal("scrollToMe");
}

void OntoloGUI::feederAddSlot()
{
  if(updateOntoPtr() == false)
    return;
  onto_->feeder.addProperty(ui->FeederSubject->text().toStdString(), ui->FeederProperty->text().toStdString(), ui->FeederObject->text().toStdString());
}

void OntoloGUI::feederDelSlot()
{
  if(updateOntoPtr() == false)
    return;
  onto_->feeder.removeProperty(ui->FeederSubject->text().toStdString(), ui->FeederProperty->text().toStdString(), ui->FeederObject->text().toStdString());
}

void OntoloGUI::feederCommitSlot()
{
  if(updateOntoPtr() == false)
    return;
  onto_->feeder.commit(ui->FeederCommitName->text().toStdString());
}

void OntoloGUI::feederCheckoutSlot()
{
  if(updateOntoPtr() == false)
    return;
  onto_->feeder.checkout(ui->FeederCommitName->text().toStdString());
}

bool OntoloGUI::updateOntoPtr()
{
  if(multi_usage_ == false)
    return true;

  const std::string instance_name = ui->OntologyName->text().toStdString();
  onto_ = ontos_.get(instance_name);
  if(onto_ == nullptr)
  {
    auto intances_name = ontos_.list();
    if(std::find(intances_name.begin(), intances_name.end(), instance_name) != intances_name.end())
    {
      ontos_.add(instance_name);
      onto_ = ontos_.get(instance_name);
      if(onto_ != nullptr)
        return true;
    }

    if(instance_name.empty() == false)
      displayErrorInfo("Ontology " + instance_name + " does not exist");
    return false;
  }
  else
    return true;
}
