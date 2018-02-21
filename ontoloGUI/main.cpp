#include "ontologui.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ontoloGUI w;
    w.show();

    return a.exec();
}
