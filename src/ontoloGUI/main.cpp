#include "include/ontoloGenius/ontoloGUI/ontologui.h"
#include "include/ontoloGenius/ontoloGUI/DarkStyle.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    a.setStyle(new DarkStyle);
    ontoloGUI w;
    w.show();

    return a.exec();
}
