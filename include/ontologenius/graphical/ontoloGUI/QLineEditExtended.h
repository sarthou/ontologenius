#ifndef QLINEEDITEXTENDED_H
#define QLINEEDITEXTENDED_H

#include <QLineEdit>

class QLineEditExtended : public QLineEdit
{
  Q_OBJECT
public:
  explicit QLineEditExtended(QWidget* parent = nullptr);
  ~QLineEditExtended() override = default;

protected:
  void hoverEnter(QHoverEvent* event);
  void hoverLeave(QHoverEvent* event);
  void hoverMove(QHoverEvent* event);
  bool event(QEvent* event) override;

Q_SIGNALS:
  void hoverEnter();
  void hoverLeave();
};

#endif // QLINEEDITEXTENDED_H
