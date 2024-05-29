#ifndef QLINEEDITEXTENDED_H
#define QLINEEDITEXTENDED_H

#include <QLineEdit>

class QLineEditExtended : public QLineEdit
{
  Q_OBJECT
public:
  explicit QLineEditExtended(QWidget* parent = 0);
  virtual ~QLineEditExtended() = default;

protected:
  void hoverEnter(QHoverEvent* event);
  void hoverLeave(QHoverEvent* event);
  void hoverMove(QHoverEvent* event);
  bool event(QEvent* event);

Q_SIGNALS:
  void hoverEnter();
  void hoverLeave();

public slots:
};

#endif // QLINEEDITEXTENDED_H
