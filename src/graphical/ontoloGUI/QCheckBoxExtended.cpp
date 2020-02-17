#include "include/ontologenius/graphical/ontoloGUI/QCheckBoxExtended.h"

#include <QEvent>
#include <QHoverEvent>
#include <QWidget>

QCheckBoxExtended::QCheckBoxExtended(QWidget *parent) : QCheckBox(parent)
{
    setMouseTracking(true);
    setAttribute(Qt::WA_Hover);
}

QCheckBoxExtended::QCheckBoxExtended(const QString& text, QWidget *parent) : QCheckBox(text, parent)
{
    setMouseTracking(true);
    setAttribute(Qt::WA_Hover);
}

void QCheckBoxExtended::hoverEnter(QHoverEvent* /*event*/)
{
    QFont font = this->font();
    font.setBold(true);
    this->setFont(font);
    repaint();
    hoverEnter();
}

void QCheckBoxExtended::hoverLeave(QHoverEvent* /*event*/)
{
    QFont font = this->font();
    font.setBold(false);
    this->setFont(font);
    repaint();
    hoverLeave();
}

void QCheckBoxExtended::hoverMove(QHoverEvent* /*event*/)
{
    QFont font = this->font();
    font.setBold(true);
    this->setFont(font);
    repaint();
}

bool QCheckBoxExtended::event(QEvent* event)
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
    return QCheckBox::event(event);
}
