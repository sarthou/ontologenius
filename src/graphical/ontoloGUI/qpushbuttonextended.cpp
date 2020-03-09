#include "include/ontologenius/graphical/ontoloGUI/qpushbuttonextended.h"

#include <QEvent>
#include <QHoverEvent>
#include <QWidget>

QPushButtonExtended::QPushButtonExtended(QWidget *parent) : QPushButton(parent)
{
    setMouseTracking(true);
    setAttribute(Qt::WA_Hover);
}

void QPushButtonExtended::hoverEnter(QHoverEvent* /*event*/)
{
    QFont font = this->font();
    font.setBold(true);
    this->setFont(font);
    repaint();
    hoverEnter();
}

void QPushButtonExtended::hoverLeave(QHoverEvent* /*event*/)
{
    QFont font = this->font();
    font.setBold(false);
    this->setFont(font);
    repaint();
    hoverLeave();
}

void QPushButtonExtended::hoverMove(QHoverEvent* /*event*/)
{
    QFont font = this->font();
    font.setBold(true);
    this->setFont(font);
    repaint();
}

bool QPushButtonExtended::event(QEvent *event)
{
    switch(event->type())
    {
    case QEvent::HoverEnter:
        hoverEnter(dynamic_cast<QHoverEvent*>(event));
        return true;
        break;
    case QEvent::HoverLeave:
        hoverLeave(dynamic_cast<QHoverEvent*>(event));
        return true;
        break;
    case QEvent::HoverMove:
        hoverMove(dynamic_cast<QHoverEvent*>(event));
        return true;
        break;
    default:
        break;
    }
    return QPushButton::event(event);
}
