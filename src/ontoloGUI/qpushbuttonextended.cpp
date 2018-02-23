#include "include/ontoloGenius/ontoloGUI/qpushbuttonextended.h"

#include <QWidget>
#include <QEvent>
#include <QHoverEvent>

QPushButtonExtended::QPushButtonExtended(QWidget *parent) : QPushButton(parent)
{
    setMouseTracking(true);
    setAttribute(Qt::WA_Hover);
}

void QPushButtonExtended::hoverEnter(QHoverEvent *event)
{
    QFont font = this->font();
    font.setBold(true);
    this->setFont(font);
    repaint();
    hoverEnter();
}

void QPushButtonExtended::hoverLeave(QHoverEvent *event)
{
    QFont font = this->font();
    font.setBold(false);
    this->setFont(font);
    repaint();
    hoverLeave();
}

void QPushButtonExtended::hoverMove(QHoverEvent *event)
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
        hoverEnter(static_cast<QHoverEvent*>(event));
        return true;
        break;
    case QEvent::HoverLeave:
        hoverLeave(static_cast<QHoverEvent*>(event));
        return true;
        break;
    case QEvent::HoverMove:
        hoverMove(static_cast<QHoverEvent*>(event));
        return true;
        break;
    default:
        break;
    }
    return QWidget::event(event);
}
