#ifndef QCHECKBOXEXTENDED_H
#define QCHECKBOXEXTENDED_H

#include <QCheckBox>

class QCheckBoxExtended : public QCheckBox
{
  Q_OBJECT
public:
  explicit QCheckBoxExtended(QWidget* parent = nullptr);
  explicit QCheckBoxExtended(const QString& text, QWidget* parent = nullptr);
  ~QCheckBoxExtended() override = default;

protected:
  void hoverEnter(QHoverEvent* event);
  void hoverLeave(QHoverEvent* event);
  void hoverMove(QHoverEvent* event);
  bool event(QEvent* event) override;

Q_SIGNALS:
  void hoverEnter();
  void hoverLeave();
};

#endif // QCHECKBOXEXTENDED_H
