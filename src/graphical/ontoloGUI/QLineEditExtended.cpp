#include "include/ontoloGenius/graphical/ontoloGUI/QLineEditExtended.h"

#include <QWidget>
#include <QEvent>
#include <QHoverEvent>

QLineEditExtended::QLineEditExtended(QWidget *parent) : QLineEdit(parent)
{
    setMouseTracking(true);
    setAttribute(Qt::WA_Hover);
}

void QLineEditExtended::hoverEnter(QHoverEvent *event)
{
    hoverEnter();
}

void QLineEditExtended::hoverLeave(QHoverEvent *event)
{
    hoverLeave();
}

void QLineEditExtended::hoverMove(QHoverEvent *event)
{
}

bool QLineEditExtended::event(QEvent *event)
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
