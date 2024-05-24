#include "include/ontologenius/graphical/ontoloGUI/QLineEditExtended.h"

#include <QEvent>
#include <QHoverEvent>
#include <QWidget>

QLineEditExtended::QLineEditExtended(QWidget* parent) : QLineEdit(parent)
{
  setMouseTracking(true);
  setAttribute(Qt::WA_Hover);
}

void QLineEditExtended::hoverEnter(QHoverEvent* /*event*/)
{
  hoverEnter();
}

void QLineEditExtended::hoverLeave(QHoverEvent* /*event*/)
{
  hoverLeave();
}

void QLineEditExtended::hoverMove(QHoverEvent* /*event*/)
{}

bool QLineEditExtended::event(QEvent* event)
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
  return QLineEdit::event(event);
}
