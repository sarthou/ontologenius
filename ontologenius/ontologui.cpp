#include "ontologui.h"
#include "ui_ontologui.h"

ontoloGUI::ontoloGUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ontoloGUI)
{
    ui->setupUi(this);
}

ontoloGUI::~ontoloGUI()
{
    delete ui;
}
