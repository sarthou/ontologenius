#include "include/ontoloGenius/ontoloGUI/ontologui.h"
#include "include/ontoloGenius/ontoloGUI/qpushbuttonextended.h"
#include "ui_ontologui.h"

#include "ontologenius/standard_service.h"
#include <string>

ontoloGUI::ontoloGUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ontoloGUI)
{
    ui->setupUi(this);

    QObject::connect(ui->Class_getUp, SIGNAL(hoverEnter()),this, SLOT(ClasshoverEnterSlot()));
    QObject::connect(ui->Class_getUp, SIGNAL(hoverLeave()),this, SLOT(ClasshoverLeaveSlot()));
    QObject::connect(ui->Class_getDown, SIGNAL(hoverEnter()),this, SLOT(ClasshoverEnterSlot()));
    QObject::connect(ui->Class_getDown, SIGNAL(hoverLeave()),this, SLOT(ClasshoverLeaveSlot()));
    QObject::connect(ui->Class_getDisjoint, SIGNAL(hoverEnter()),this, SLOT(ClasshoverEnterSlot()));
    QObject::connect(ui->Class_getDisjoint, SIGNAL(hoverLeave()),this, SLOT(ClasshoverLeaveSlot()));

    QObject::connect(ui->Property_getDisjoint, SIGNAL(hoverEnter()),this, SLOT(PropertyhoverEnterSlot()));
    QObject::connect(ui->Property_getDisjoint, SIGNAL(hoverLeave()),this, SLOT(PropertyhoverLeaveSlot()));
    QObject::connect(ui->Property_getDomain, SIGNAL(hoverEnter()),this, SLOT(PropertyhoverEnterSlot()));
    QObject::connect(ui->Property_getDomain, SIGNAL(hoverLeave()),this, SLOT(PropertyhoverLeaveSlot()));
    QObject::connect(ui->Property_getDown, SIGNAL(hoverEnter()),this, SLOT(PropertyhoverEnterSlot()));
    QObject::connect(ui->Property_getDown, SIGNAL(hoverLeave()),this, SLOT(PropertyhoverLeaveSlot()));
    QObject::connect(ui->Property_getInverse, SIGNAL(hoverEnter()),this, SLOT(PropertyhoverEnterSlot()));
    QObject::connect(ui->Property_getInverse, SIGNAL(hoverLeave()),this, SLOT(PropertyhoverLeaveSlot()));
    QObject::connect(ui->Property_getRange, SIGNAL(hoverEnter()),this, SLOT(PropertyhoverEnterSlot()));
    QObject::connect(ui->Property_getRange, SIGNAL(hoverLeave()),this, SLOT(PropertyhoverLeaveSlot()));
    QObject::connect(ui->Property_getUp, SIGNAL(hoverEnter()),this, SLOT(PropertyhoverEnterSlot()));
    QObject::connect(ui->Property_getUp, SIGNAL(hoverLeave()),this, SLOT(PropertyhoverLeaveSlot()));

    QObject::connect(ui->Individual_getDistinct, SIGNAL(hoverEnter()),this, SLOT(IndividualhoverEnterSlot()));
    QObject::connect(ui->Individual_getDistinct, SIGNAL(hoverLeave()),this, SLOT(IndividualhoverLeaveSlot()));
    QObject::connect(ui->Individual_getRelatedFrom, SIGNAL(hoverEnter()),this, SLOT(IndividualhoverEnterSlot()));
    QObject::connect(ui->Individual_getRelatedFrom, SIGNAL(hoverLeave()),this, SLOT(IndividualhoverLeaveSlot()));
    QObject::connect(ui->Individual_getRelatedOn, SIGNAL(hoverEnter()),this, SLOT(IndividualhoverEnterSlot()));
    QObject::connect(ui->Individual_getRelatedOn, SIGNAL(hoverLeave()),this, SLOT(IndividualhoverLeaveSlot()));
    QObject::connect(ui->Individual_getRelatedWith, SIGNAL(hoverEnter()),this, SLOT(IndividualhoverEnterSlot()));
    QObject::connect(ui->Individual_getRelatedWith, SIGNAL(hoverLeave()),this, SLOT(IndividualhoverLeaveSlot()));
    QObject::connect(ui->Individual_getRelationFrom, SIGNAL(hoverEnter()),this, SLOT(IndividualhoverEnterSlot()));
    QObject::connect(ui->Individual_getRelationFrom, SIGNAL(hoverLeave()),this, SLOT(IndividualhoverLeaveSlot()));
    QObject::connect(ui->Individual_getRelationOn, SIGNAL(hoverEnter()),this, SLOT(IndividualhoverEnterSlot()));
    QObject::connect(ui->Individual_getRelationOn, SIGNAL(hoverLeave()),this, SLOT(IndividualhoverLeaveSlot()));
    QObject::connect(ui->Individual_getRelationWith, SIGNAL(hoverEnter()),this, SLOT(IndividualhoverEnterSlot()));
    QObject::connect(ui->Individual_getRelationWith, SIGNAL(hoverLeave()),this, SLOT(IndividualhoverLeaveSlot()));
    QObject::connect(ui->Individual_getSame, SIGNAL(hoverEnter()),this, SLOT(IndividualhoverEnterSlot()));
    QObject::connect(ui->Individual_getSame, SIGNAL(hoverLeave()),this, SLOT(IndividualhoverLeaveSlot()));
    QObject::connect(ui->Individual_getUp, SIGNAL(hoverEnter()),this, SLOT(IndividualhoverEnterSlot()));
    QObject::connect(ui->Individual_getUp, SIGNAL(hoverLeave()),this, SLOT(IndividualhoverLeaveSlot()));

    QObject::connect(ui->Class_getUp, SIGNAL(clicked()),this, SLOT(classClickedSlot()));
    QObject::connect(ui->Class_getDown, SIGNAL(clicked()),this, SLOT(classClickedSlot()));
    QObject::connect(ui->Class_getDisjoint, SIGNAL(clicked()),this, SLOT(classClickedSlot()));

    QObject::connect(ui->Property_getDisjoint, SIGNAL(clicked()),this, SLOT(propertyClickedSlot()));
    QObject::connect(ui->Property_getDomain, SIGNAL(clicked()),this, SLOT(propertyClickedSlot()));
    QObject::connect(ui->Property_getDown, SIGNAL(clicked()),this, SLOT(propertyClickedSlot()));
    QObject::connect(ui->Property_getInverse, SIGNAL(clicked()),this, SLOT(propertyClickedSlot()));
    QObject::connect(ui->Property_getRange, SIGNAL(clicked()),this, SLOT(propertyClickedSlot()));
    QObject::connect(ui->Property_getUp, SIGNAL(clicked()),this, SLOT(propertyClickedSlot()));

    QObject::connect(ui->Individual_getDistinct, SIGNAL(clicked()),this, SLOT(individualClickedSlot()));
    QObject::connect(ui->Individual_getRelatedFrom, SIGNAL(clicked()),this, SLOT(individualClickedSlot()));
    QObject::connect(ui->Individual_getRelatedOn, SIGNAL(clicked()),this, SLOT(individualClickedSlot()));
    QObject::connect(ui->Individual_getRelatedWith, SIGNAL(clicked()),this, SLOT(individualClickedSlot()));
    QObject::connect(ui->Individual_getRelationFrom, SIGNAL(clicked()),this, SLOT(individualClickedSlot()));
    QObject::connect(ui->Individual_getRelationOn, SIGNAL(clicked()),this, SLOT(individualClickedSlot()));
    QObject::connect(ui->Individual_getRelationWith, SIGNAL(clicked()),this, SLOT(individualClickedSlot()));
    QObject::connect(ui->Individual_getSame, SIGNAL(clicked()),this, SLOT(individualClickedSlot()));
    QObject::connect(ui->Individual_getUp, SIGNAL(clicked()),this, SLOT(individualClickedSlot()));

    QObject::connect(ui->CloseButton, SIGNAL(clicked()),this, SLOT(closeOntologySlot()));
}

ontoloGUI::~ontoloGUI()
{
    delete ui;
}

void ontoloGUI::wait()
{
  QString html = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
                  "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">"
                  "p, li { white-space: pre-wrap; }"
                  "</style></head><body style=\" font-family:'Noto Sans'; font-size:9pt; font-weight:400; font-style:normal;\">"
                  "<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; color:#a40000;\">Wainting for </span><span style=\" font-size:12pt; font-weight:600; color:#a40000;\">ontoloGenius</span></p></body></html>";
  ui->InfoArea->setHtml(html);
  ros::service::waitForService("ontoloGenius/arguer", -1);
}

void ontoloGUI::start()
{
  QString html = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
                  "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">"
                  "p, li { white-space: pre-wrap; }"
                  "</style></head><body style=\" font-family:'Noto Sans'; font-size:9pt; font-weight:400; font-style:normal;\">"
                  "<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; font-weight:600; color:#4e9a06;\">OntoloGenius</span><span style=\" font-size:12pt; color:#4e9a06;\"> detected</span></p></body></html>";
  ui->InfoArea->setHtml(html);
}

void ontoloGUI::ClasshoverEnterSlot()
{
  ui->ClassDescription->setText(((QPushButtonExtended*)sender())->whatsThis());
}

void ontoloGUI::ClasshoverLeaveSlot()
{
  ui->ClassDescription->setText("");
}

void ontoloGUI::PropertyhoverEnterSlot()
{
  ui->PropertyDescription->setText(((QPushButtonExtended*)sender())->whatsThis());
}

void ontoloGUI::PropertyhoverLeaveSlot()
{
  ui->PropertyDescription->setText("");
}
void ontoloGUI::IndividualhoverEnterSlot()
{
  ui->IndividualDescription->setText(((QPushButtonExtended*)sender())->whatsThis());
}

void ontoloGUI::IndividualhoverLeaveSlot()
{
  ui->IndividualDescription->setText("");
}

void ontoloGUI::classClickedSlot()
{
  ros::ServiceClient client = n_->serviceClient<ontologenius::standard_service>("ontoloGenius/class");

  ontologenius::standard_service srv;
  srv.request.action = ((QPushButtonExtended*)sender())->text().toStdString();
  srv.request.param = ui->classParameter->text().toStdString();
  QString text = ((QPushButtonExtended*)sender())->text() + " : " + ui->classParameter->text();
  ui->ClassDescription->setText(text);

  if(!client.call(srv))
    ui->InfoArea->setText("ontoloGenius/class client call failed");
  else
  {
    std::string res = srv.response.value;
    ui->ResultArea->setText(QString::fromStdString(res));
  }
}

void ontoloGUI::propertyClickedSlot()
{
  ros::ServiceClient client = n_->serviceClient<ontologenius::standard_service>("ontoloGenius/property");

  ontologenius::standard_service srv;
  srv.request.action = ((QPushButtonExtended*)sender())->text().toStdString();
  srv.request.param = ui->propertyParameter->text().toStdString();
  QString text = ((QPushButtonExtended*)sender())->text() + " : " + ui->propertyParameter->text();
  ui->PropertyDescription->setText(text);

  if(!client.call(srv))
    ui->InfoArea->setText("ontoloGenius/property client call failed");
  else
  {
    std::string res = srv.response.value;
    ui->ResultArea->setText(QString::fromStdString(res));
  }
}

void ontoloGUI::individualClickedSlot()
{
  ros::ServiceClient client = n_->serviceClient<ontologenius::standard_service>("ontoloGenius/individual");

  ontologenius::standard_service srv;
  srv.request.action = ((QPushButtonExtended*)sender())->text().toStdString();
  srv.request.param = ui->individualParameter->text().toStdString();
  QString text = ((QPushButtonExtended*)sender())->text() + " : " + ui->individualParameter->text();
  ui->IndividualDescription->setText(text);

  if(!client.call(srv))
    ui->InfoArea->setText("ontoloGenius/individual client call failed");
  else
  {
    std::string res = srv.response.value;
    ui->ResultArea->setText(QString::fromStdString(res));
  }
}

void ontoloGUI::closeOntologySlot()
{
  ros::ServiceClient client = n_->serviceClient<ontologenius::standard_service>("ontoloGenius/actions");

  ontologenius::standard_service srv;
  srv.request.action = "close";

  if(!client.call(srv))
    ui->InfoArea->setText("ontoloGenius/actions client call failed");
  else
  {
    std::string res = srv.response.value;
    ui->ResultArea->setText(QString::fromStdString(res));
  }
}
