#ifndef QPUSHBUTTONEXTENDED_H
#define QPUSHBUTTONEXTENDED_H

#include <QPushButton>

class QPushButtonExtended : public QPushButton
{
  Q_OBJECT
public:
  explicit QPushButtonExtended(QWidget* parent = nullptr);
  ~QPushButtonExtended() override = default;

protected:
  void hoverEnter(QHoverEvent* event);
  void hoverLeave(QHoverEvent* event);
  void hoverMove(QHoverEvent* event);
  bool event(QEvent* event) override;

Q_SIGNALS:
  void hoverEnter();
  void hoverLeave();
};

#endif // QPUSHBUTTONEXTENDED_H
