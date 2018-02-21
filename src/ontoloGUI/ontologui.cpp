#include "include/ontoloGenius/ontoloGUI/ontologui.h"
#include "include/ontoloGenius/ontoloGUI/qpushbuttonextended.h"
#include "ui_ontologui.h"

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
}

ontoloGUI::~ontoloGUI()
{
    delete ui;
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
    QString text = ((QPushButtonExtended*)sender())->text() + " : " + ui->classParameter->text();
    ui->ClassDescription->setText(text);
}

void ontoloGUI::propertyClickedSlot()
{
    QString text = ((QPushButtonExtended*)sender())->text() + " : " + ui->propertyParameter->text();
    ui->PropertyDescription->setText(text);
}

void ontoloGUI::individualClickedSlot()
{
    QString text = ((QPushButtonExtended*)sender())->text() + " : " + ui->individualParameter->text();
    ui->IndividualDescription->setText(text);
}
