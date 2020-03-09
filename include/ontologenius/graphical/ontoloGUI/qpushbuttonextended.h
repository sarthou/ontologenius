#ifndef QPUSHBUTTONEXTENDED_H
#define QPUSHBUTTONEXTENDED_H

#include <QPushButton>

class QPushButtonExtended : public QPushButton
{
    Q_OBJECT
public:
    explicit QPushButtonExtended(QWidget *parent=0);
    virtual ~QPushButtonExtended() {}

protected:
    void hoverEnter(QHoverEvent *event);
    void hoverLeave(QHoverEvent *event);
    void hoverMove(QHoverEvent *event);
    bool event(QEvent *event);

Q_SIGNALS:
    void hoverEnter();
    void hoverLeave();

public slots:
};

#endif // QPUSHBUTTONEXTENDED_H
