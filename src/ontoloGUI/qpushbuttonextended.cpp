#include "qpushbuttonextended.h"

#include <QWidget>
#include <QEvent>
#include <QHoverEvent>

QPushButtonExtended::QPushButtonExtended()
{

}

void hoverEnter(QHoverEvent *event)
{

}

void hoverLeave(QHoverEvent *event)
{

}

void hoverMove(QHoverEvent *event)
{

}

bool event(QEvent *event)
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
