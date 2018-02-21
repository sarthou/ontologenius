#ifndef ONTOLOGUI_H
#define ONTOLOGUI_H

#include <QMainWindow>

namespace Ui {
class ontoloGUI;
}

class ontoloGUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit ontoloGUI(QWidget *parent = 0);
    ~ontoloGUI();

private:
    Ui::ontoloGUI *ui;
};

#endif // ONTOLOGUI_H
